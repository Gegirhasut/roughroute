//! Routing: waypoint snapping, A\* pathfinding, multi-point concatenation,
//! and the straight-line fallback.
//!
//! Since format v2 (M4) edges may be collapsed degree-2 chains carrying
//! intermediate geometry; A\* therefore tracks *which edge* reached each node
//! (parallel edges between the same node pair are real), and line assembly
//! expands every traversed edge's geometry so the returned polyline still
//! follows the actual road shape (F6).

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::geo;
use crate::graph::Graph;
use crate::profile::Profile;

/// Errors produced by [`Router::route`].
#[derive(Debug, Clone, PartialEq)]
pub enum RouteError {
    /// Fewer than two waypoints were supplied.
    TooFewWaypoints,
    /// Waypoint `index` is farther than `max_snap_meters` from any graph
    /// node. `meters` is the distance to the nearest node found, or
    /// `f64::INFINITY` when nothing was found within the search bound.
    SnapTooFar {
        /// Index of the offending waypoint in the request.
        index: usize,
        /// Haversine distance to the nearest node found, if any.
        meters: f64,
    },
    /// No path exists between the snapped endpoints of leg `segment`
    /// (waypoint `segment` → waypoint `segment + 1`) and fallback is
    /// disabled.
    NoPath {
        /// Index of the leg whose endpoints are not connected.
        segment: usize,
    },
}

impl core::fmt::Display for RouteError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RouteError::TooFewWaypoints => write!(f, "at least two waypoints are required"),
            RouteError::SnapTooFar { index, meters } => write!(
                f,
                "waypoint {index} is too far from any road (nearest node: {meters:.1} m)"
            ),
            RouteError::NoPath { segment } => write!(
                f,
                "no path between waypoints {segment} and {} (fallback disabled)",
                segment + 1
            ),
        }
    }
}

impl std::error::Error for RouteError {}

/// Options controlling a [`Router`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteOptions {
    /// Which edges may be traversed.
    pub profile: Profile,
    /// When no path exists between two snapped waypoints, bridge them with a
    /// straight segment (and set `fallback: true`) instead of failing.
    /// Default `true` — for the spoofer, "some route" beats an honest refusal.
    pub allow_fallback: bool,
    /// Maximum snapping distance: a waypoint farther than this from every
    /// graph node yields [`RouteError::SnapTooFar`]. Default `200.0`.
    pub max_snap_meters: f64,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions { profile: Profile::Car, allow_fallback: true, max_snap_meters: 200.0 }
    }
}

/// A routing result: the road-following polyline and its length.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteResult {
    /// The polyline as `[lat, lon]` degree pairs, dense along road geometry:
    /// every graph node passed *and* every intermediate geometry point of
    /// every traversed (collapsed) edge, in travel order. Consecutive
    /// duplicates are removed at leg junctions.
    pub line: Vec<[f64; 2]>,
    /// Total length in meters: the haversine sum over `line` itself (spec
    /// §8.5), so it always matches the returned geometry.
    pub meters: f64,
    /// `true` if at least one leg had no road path and was bridged with a
    /// straight segment.
    pub fallback: bool,
}

/// A router over a borrowed [`Graph`]. Cheap to construct; make one per
/// options set. Multiple routers may share one graph across threads.
pub struct Router<'g> {
    graph: &'g Graph,
    opts: RouteOptions,
}

impl<'g> Router<'g> {
    /// Create a router over `graph` with the given options.
    pub fn new(graph: &'g Graph, opts: RouteOptions) -> Self {
        Router { graph, opts }
    }

    /// Build a route through `waypoints` (at least two, `[lat, lon]`
    /// degrees): snap each waypoint to its nearest graph node, run A\*
    /// between consecutive pairs, and concatenate the legs (dropping the
    /// duplicated junction node between legs).
    pub fn route(&self, waypoints: &[[f64; 2]]) -> Result<RouteResult, RouteError> {
        if waypoints.len() < 2 {
            return Err(RouteError::TooFewWaypoints);
        }

        let mask = self.opts.profile.mask();

        let mut snapped = Vec::with_capacity(waypoints.len());
        for (index, &[lat, lon]) in waypoints.iter().enumerate() {
            match self.graph.nearest_node(lat, lon, self.opts.max_snap_meters) {
                Some((node, meters)) if meters <= self.opts.max_snap_meters => snapped.push(node),
                Some((_, meters)) => return Err(RouteError::SnapTooFar { index, meters }),
                None => return Err(RouteError::SnapTooFar { index, meters: f64::INFINITY }),
            }
        }

        // Resolve every leg first, then assemble coordinates. Legs share
        // their junction node (leg i ends where leg i+1 starts), so every leg
        // after the first appends *from after* that shared node.
        let mut line: Vec<[f64; 2]> = Vec::new();
        let mut fallback = false;
        for (segment, pair) in snapped.windows(2).enumerate() {
            let (a, b) = (pair[0], pair[1]);
            let leg: Option<Vec<u32>> = if a == b {
                None // coinciding waypoints: nothing to add beyond the shared node
            } else {
                match self.astar(a, b, mask) {
                    Some(edges) => Some(edges),
                    None if self.opts.allow_fallback => {
                        fallback = true;
                        None
                    }
                    None => return Err(RouteError::NoPath { segment }),
                }
            };

            if line.is_empty() {
                line.push(self.graph.node_latlon(a));
            }
            match leg {
                None if a == b => {}
                None => line.push(self.graph.node_latlon(b)), // fallback straight segment
                Some(edges) => {
                    for &edge_index in &edges {
                        let edge = self.graph.edge(edge_index);
                        self.push_edge_geometry(&mut line, edge);
                        line.push(self.graph.node_latlon(edge.target));
                    }
                }
            }
        }

        let meters = line
            .windows(2)
            .map(|w| geo::haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]))
            .sum();

        Ok(RouteResult { line, meters, fallback })
    }

    /// Append `edge`'s intermediate geometry to `line` in travel order (the
    /// pool stores the canonical direction; reversed edges walk it
    /// back-to-front).
    fn push_edge_geometry(&self, line: &mut Vec<[f64; 2]>, edge: &crate::graph::Edge) {
        let points = self.graph.edge_geometry(edge);
        let to_deg =
            |&[lat, lon]: &[i32; 2]| [geo::fixed_to_deg(lat), geo::fixed_to_deg(lon)];
        if edge.reversed {
            line.extend(points.iter().rev().map(to_deg));
        } else {
            line.extend(points.iter().map(to_deg));
        }
    }

    /// Plain A\* from `start` to `goal` over edges matching `mask`, returning
    /// the traversed global edge indices (in travel order) or `None` when the
    /// nodes are not connected. Tracking edges — not just nodes — matters
    /// because collapsed graphs legitimately contain parallel edges with
    /// different geometry.
    ///
    /// Determinism (F9): costs are integer decimeters, the priority is
    /// `(f, node_index)` so equal-cost frontier entries pop in node-index
    /// order, and adjacency lists have a fixed on-disk order (first
    /// strictly-better relaxation wins, so among equal-cost parallel edges
    /// the one earliest in CSR order is chosen).
    ///
    /// The heuristic is the haversine distance in decimeters, floored. A
    /// straight line never exceeds the road distance, so this is admissible
    /// up to `length_dm` rounding (< 0.5 dm per original segment — a path
    /// found may be "suboptimal" by centimeters, irrelevant at rough-routing
    /// accuracy).
    fn astar(&self, start: u32, goal: u32, mask: u8) -> Option<Vec<u32>> {
        let n = self.graph.node_count() as usize;
        let [goal_lat, goal_lon] = self.graph.node_latlon(goal);
        let h = |node: u32| -> u64 {
            let [lat, lon] = self.graph.node_latlon(node);
            (geo::haversine_m(lat, lon, goal_lat, goal_lon) * 10.0).floor() as u64
        };

        // Dense per-query state: O(node_count) but trivially correct — and
        // node_count shrank several-fold with the M4 collapse.
        let mut g_cost: Vec<u64> = vec![u64::MAX; n];
        let mut parent_node: Vec<u32> = vec![u32::MAX; n];
        let mut parent_edge: Vec<u32> = vec![u32::MAX; n];
        let mut closed: Vec<bool> = vec![false; n];
        let mut heap: BinaryHeap<Reverse<(u64, u32)>> = BinaryHeap::new();

        g_cost[start as usize] = 0;
        heap.push(Reverse((h(start), start)));

        while let Some(Reverse((_, node))) = heap.pop() {
            if closed[node as usize] {
                continue; // stale heap entry
            }
            closed[node as usize] = true;
            if node == goal {
                return Some(reconstruct(&parent_node, &parent_edge, start, goal));
            }
            let first_edge = self.graph.first_edge_index(node);
            for (i, edge) in self.graph.edges_from(node).iter().enumerate() {
                if edge.access & mask == 0 || closed[edge.target as usize] {
                    continue;
                }
                let candidate = g_cost[node as usize].saturating_add(u64::from(edge.length_dm));
                if candidate < g_cost[edge.target as usize] {
                    g_cost[edge.target as usize] = candidate;
                    parent_node[edge.target as usize] = node;
                    parent_edge[edge.target as usize] = first_edge + i as u32;
                    heap.push(Reverse((candidate.saturating_add(h(edge.target)), edge.target)));
                }
            }
        }
        None
    }
}

/// Walk the parent chain from `goal` back to `start`, collecting the edge
/// indices, and reverse into travel order.
fn reconstruct(parent_node: &[u32], parent_edge: &[u32], start: u32, goal: u32) -> Vec<u32> {
    let mut edges = Vec::new();
    let mut node = goal;
    // The chain is acyclic by construction (parents are set once, before a
    // node is expandable); the length bound is a defensive backstop.
    while node != start && edges.len() <= parent_node.len() {
        edges.push(parent_edge[node as usize]);
        node = parent_node[node as usize];
    }
    edges.reverse();
    edges
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::deg_to_fixed;
    use crate::graph::Edge;
    use crate::profile::{ACCESS_ALL, ACCESS_CAR, ACCESS_FOOT};

    fn fx(lat: f64, lon: f64) -> [i32; 2] {
        [deg_to_fixed(lat), deg_to_fixed(lon)]
    }

    /// Build a graph from an undirected edge list `(a, b, access)`; lengths
    /// come from node coordinates. No geometry (every node is real).
    fn graph_from(nodes: Vec<[i32; 2]>, undirected: &[(u32, u32, u8)]) -> Graph {
        let mut directed: Vec<(u32, Edge)> = Vec::new();
        for &(a, b, access) in undirected {
            let [alat, alon] = nodes[a as usize];
            let [blat, blon] = nodes[b as usize];
            let m = geo::haversine_m(
                geo::fixed_to_deg(alat),
                geo::fixed_to_deg(alon),
                geo::fixed_to_deg(blat),
                geo::fixed_to_deg(blon),
            );
            let length_dm = ((m * 10.0).round() as u32).max(1);
            let mk = |target| Edge {
                target,
                length_dm,
                geo_off: 0,
                geo_len: 0,
                reversed: false,
                access,
            };
            directed.push((a, mk(b)));
            directed.push((b, mk(a)));
        }
        directed.sort_by_key(|(s, e)| (*s, e.target, e.length_dm, e.access));
        let mut offsets = vec![0u32; nodes.len() + 1];
        for (s, _) in &directed {
            offsets[*s as usize + 1] += 1;
        }
        for i in 1..offsets.len() {
            offsets[i] += offsets[i - 1];
        }
        let edges = directed.into_iter().map(|(_, e)| e).collect();
        Graph::from_parts(nodes, offsets, edges, vec![]).unwrap()
    }

    /// 0 - 1 - 2
    ///     |
    ///     3     (edge 1-3 is foot-only)
    /// plus isolated pair 4 - 5 far away.
    fn test_graph() -> Graph {
        let nodes = vec![
            fx(35.000, 33.000),
            fx(35.000, 33.010),
            fx(35.000, 33.020),
            fx(34.990, 33.010),
            fx(35.200, 33.200),
            fx(35.200, 33.210),
        ];
        graph_from(
            nodes,
            &[
                (0, 1, ACCESS_ALL),
                (1, 2, ACCESS_ALL),
                (1, 3, ACCESS_FOOT),
                (4, 5, ACCESS_ALL),
            ],
        )
    }

    /// Two junctions (0, 1) joined by a collapsed edge whose shape bows east
    /// through two intermediate points, plus a second, *parallel* collapsed
    /// edge bowing west that is slightly longer.
    fn collapsed_graph() -> Graph {
        let nodes = vec![fx(35.000, 33.000), fx(35.030, 33.000)];
        let geometry = vec![
            // east bow (shorter detour), canonical direction 0 -> 1
            fx(35.010, 33.001),
            fx(35.020, 33.001),
            // west bow (longer detour), canonical direction 0 -> 1
            fx(35.010, 32.995),
            fx(35.020, 32.995),
        ];
        let east = 33_420u32; // sums of the underlying chains, east < west
        let west = 33_800u32;
        let edges = vec![
            Edge { target: 1, length_dm: east, geo_off: 0, geo_len: 2, reversed: false, access: ACCESS_ALL },
            Edge { target: 1, length_dm: west, geo_off: 2, geo_len: 2, reversed: false, access: ACCESS_ALL },
            Edge { target: 0, length_dm: east, geo_off: 0, geo_len: 2, reversed: true, access: ACCESS_ALL },
            Edge { target: 0, length_dm: west, geo_off: 2, geo_len: 2, reversed: true, access: ACCESS_ALL },
        ];
        let offsets = vec![0, 2, 4];
        Graph::from_parts(nodes, offsets, edges, geometry).unwrap()
    }

    fn graph_from_owned(nodes: &[[i32; 2]], und: &[(u32, u32, u8)]) -> Graph {
        graph_from(nodes.to_vec(), und)
    }

    fn wp(g: &Graph, n: u32) -> [f64; 2] {
        g.node_latlon(n)
    }

    #[test]
    fn simple_route_follows_the_road() {
        let g = test_graph();
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[wp(&g, 0), wp(&g, 2)]).unwrap();
        assert!(!res.fallback);
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 1), wp(&g, 2)]);
        assert!(res.meters > 0.0);
    }

    #[test]
    fn collapsed_edge_geometry_is_expanded_in_travel_order() {
        let g = collapsed_graph();
        let router = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        let d = |i: usize| {
            let [lat, lon] = [
                geo::fixed_to_deg(g.edge_geometry(&g.edges_from(0)[0])[i][0]),
                geo::fixed_to_deg(g.edge_geometry(&g.edges_from(0)[0])[i][1]),
            ];
            [lat, lon]
        };
        // Forward: 0, east-bow points in order, 1. A* picks the shorter
        // (east) of the two parallel edges.
        let res = router.route(&[wp(&g, 0), wp(&g, 1)]).unwrap();
        assert_eq!(res.line, vec![wp(&g, 0), d(0), d(1), wp(&g, 1)]);

        // Backward: same edge traversed 1 -> 0 must yield exactly the
        // reversed line.
        let back = router.route(&[wp(&g, 1), wp(&g, 0)]).unwrap();
        let mut expected: Vec<[f64; 2]> = res.line.clone();
        expected.reverse();
        assert_eq!(back.line, expected);
        assert_eq!(back.meters, res.meters);
    }

    #[test]
    fn too_few_waypoints() {
        let g = test_graph();
        let router = Router::new(&g, RouteOptions::default());
        assert_eq!(router.route(&[]), Err(RouteError::TooFewWaypoints));
        assert_eq!(router.route(&[[35.0, 33.0]]), Err(RouteError::TooFewWaypoints));
    }

    #[test]
    fn snap_too_far_reports_index_and_distance() {
        let g = test_graph();
        let router = Router::new(&g, RouteOptions::default());
        // Second waypoint ~5.5 km from the nearest node, cutoff 200 m.
        let err = router.route(&[wp(&g, 0), [35.05, 33.01]]).unwrap_err();
        match err {
            RouteError::SnapTooFar { index, meters } => {
                assert_eq!(index, 1);
                assert!(meters > 200.0);
            }
            other => panic!("expected SnapTooFar, got {other:?}"),
        }
    }

    #[test]
    fn profile_filtering_and_fallback() {
        let g = test_graph();
        // Foot can reach node 3 via the foot-only edge.
        let foot = Router::new(
            &g,
            RouteOptions { profile: Profile::Foot, ..RouteOptions::default() },
        );
        let res = foot.route(&[wp(&g, 0), wp(&g, 3)]).unwrap();
        assert!(!res.fallback);
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 1), wp(&g, 3)]);

        // Car cannot: fallback straight line (waypoints snap to 0 and 3).
        let car = Router::new(&g, RouteOptions::default());
        let res = car.route(&[wp(&g, 0), wp(&g, 3)]).unwrap();
        assert!(res.fallback);
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 3)]);

        // With fallback disabled it's an error naming the leg.
        let strict = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        assert_eq!(
            strict.route(&[wp(&g, 0), wp(&g, 3)]),
            Err(RouteError::NoPath { segment: 0 })
        );
    }

    #[test]
    fn disconnected_component_falls_back() {
        let g = test_graph();
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[wp(&g, 0), wp(&g, 4)]).unwrap();
        assert!(res.fallback);
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 4)]);
    }

    #[test]
    fn multipoint_dedups_junctions() {
        let g = test_graph();
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[wp(&g, 0), wp(&g, 1), wp(&g, 2)]).unwrap();
        // Node 1 is the junction of both legs and must appear once.
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 1), wp(&g, 2)]);
    }

    #[test]
    fn coinciding_waypoints_are_deduped() {
        let g = test_graph();
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[wp(&g, 0), wp(&g, 0), wp(&g, 2)]).unwrap();
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 1), wp(&g, 2)]);
        // Fully degenerate: both waypoints snap to the same node.
        let res = router.route(&[wp(&g, 0), wp(&g, 0)]).unwrap();
        assert_eq!(res.line, vec![wp(&g, 0)]);
        assert_eq!(res.meters, 0.0);
        assert!(!res.fallback);
    }

    #[test]
    fn meters_matches_polyline_haversine_sum() {
        // Includes intermediate geometry points, not just nodes (spec §8.5).
        let g = collapsed_graph();
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[wp(&g, 0), wp(&g, 1)]).unwrap();
        assert_eq!(res.line.len(), 4);
        let expected: f64 = res
            .line
            .windows(2)
            .map(|w| geo::haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]))
            .sum();
        assert_eq!(res.meters, expected);
    }

    #[test]
    fn astar_picks_shorter_of_two_paths_and_ties_break_by_index() {
        // Square 0-1-2 (via top) vs 0-3-2 (via bottom, longer).
        let nodes = vec![
            fx(35.000, 33.000), // 0
            fx(35.001, 33.005), // 1 top middle
            fx(35.000, 33.010), // 2
            fx(34.995, 33.005), // 3 bottom middle (farther out)
        ];
        let g = graph_from_owned(
            &nodes,
            &[(0, 1, ACCESS_CAR), (1, 2, ACCESS_CAR), (0, 3, ACCESS_CAR), (3, 2, ACCESS_CAR)],
        );
        let router = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        let res = router.route(&[wp(&g, 0), wp(&g, 2)]).unwrap();
        assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 1), wp(&g, 2)]);

        // Perfect tie: symmetric diamond. The path through the
        // smaller-indexed middle node must win, reproducibly.
        let nodes = vec![
            fx(35.000, 33.000), // 0
            fx(35.001, 33.005), // 1 top middle
            fx(34.999, 33.005), // 2 bottom middle, mirror of 1
            fx(35.000, 33.010), // 3
        ];
        let g = graph_from_owned(
            &nodes,
            &[(0, 1, ACCESS_CAR), (1, 3, ACCESS_CAR), (0, 2, ACCESS_CAR), (2, 3, ACCESS_CAR)],
        );
        let router = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        for _ in 0..5 {
            let res = router.route(&[wp(&g, 0), wp(&g, 3)]).unwrap();
            assert_eq!(res.line, vec![wp(&g, 0), wp(&g, 1), wp(&g, 3)]);
        }
    }

    #[test]
    fn empty_graph_snaps_too_far() {
        let g = Graph::from_parts(vec![], vec![0], vec![], vec![]).unwrap();
        let router = Router::new(&g, RouteOptions::default());
        let err = router.route(&[[35.0, 33.0], [35.1, 33.0]]).unwrap_err();
        assert!(matches!(err, RouteError::SnapTooFar { index: 0, .. }));
    }
}
