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
    /// Waypoint `index` is farther than `max_snap_meters` from any road the
    /// profile can use (F10 edge snapping, profile-aware — D16). `meters` is
    /// the distance to the nearest such road found, or `f64::INFINITY` when
    /// nothing was found within the search bound.
    SnapTooFar {
        /// Index of the offending waypoint in the request.
        index: usize,
        /// Haversine distance to the nearest usable road found, if any.
        meters: f64,
    },
    /// No path exists between the snapped endpoints of leg `segment`
    /// (waypoint `segment` → waypoint `segment + 1`) and fallback is
    /// disabled.
    NoPath {
        /// Index of the leg whose endpoints are not connected.
        segment: usize,
    },
    /// [`RouteOptions::max_snap_meters`] is not a usable cutoff (NaN, negative,
    /// or infinite). Returned up front rather than silently making every
    /// waypoint snap fail.
    InvalidMaxSnapMeters,
}

impl core::fmt::Display for RouteError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RouteError::TooFewWaypoints => write!(f, "at least two waypoints are required"),
            RouteError::SnapTooFar { index, meters } => write!(
                f,
                "waypoint {index} is too far from any usable road (nearest: {meters:.1} m)"
            ),
            RouteError::NoPath { segment } => write!(
                f,
                "no path between waypoints {segment} and {} (fallback disabled)",
                segment + 1
            ),
            RouteError::InvalidMaxSnapMeters => {
                write!(f, "max_snap_meters must be a finite, non-negative number")
            }
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

/// A waypoint snapped onto a road (F10, `docs/DECISIONS.md` D16): a position
/// along a canonical edge, plus everything the virtual-endpoint search needs.
struct Snapped {
    /// Canonical edge (source index < target index) the point lies on.
    edge_index: u32,
    /// Shape segment and position within it (see `grid::RoadSnap`).
    seg: u16,
    t: f64,
    /// The projected point, `[lat, lon]` degrees — a route line endpoint.
    point: [f64; 2],
    /// The edge's endpoint nodes.
    src: u32,
    dst: u32,
    /// Distance from `src` to the projection along the shape, in dm,
    /// clamped so `d_src + (length - d_src) == length` exactly.
    d_src: u64,
    /// The edge's `length_dm`.
    length: u64,
}

impl Snapped {
    /// Cost in dm from the projection to the given endpoint node of its edge.
    fn partial_to(&self, node: u32) -> Option<u64> {
        if node == self.src {
            Some(self.d_src)
        } else if node == self.dst {
            Some(self.length - self.d_src)
        } else {
            None
        }
    }

    /// Order of two positions along the same edge's shape.
    fn along_key(&self) -> (u16, f64) {
        (self.seg, self.t)
    }
}

/// Outcome of resolving one leg between two snapped waypoints.
enum Leg {
    /// Coinciding projections: contributes only the shared point.
    Point,
    /// Stay on the shared edge between the two projections (no node visited).
    AlongEdge,
    /// A road path: entry node, traversed global edge indices, exit node.
    Network { entry: u32, edges: Vec<u32>, exit: u32 },
    /// No path; bridged with a straight segment between the projections.
    Straight,
}

impl<'g> Router<'g> {
    /// Create a router over `graph` with the given options.
    pub fn new(graph: &'g Graph, opts: RouteOptions) -> Self {
        Router { graph, opts }
    }

    /// Build a route through `waypoints` (at least two, `[lat, lon]`
    /// degrees): snap each waypoint onto the nearest road (the perpendicular
    /// projection onto an edge's shape — F10), run A\* between consecutive
    /// snapped positions treating them as virtual points on their edges, and
    /// concatenate the legs. The returned line starts and ends exactly at the
    /// projected points, with partial edge geometry spliced in.
    pub fn route(&self, waypoints: &[[f64; 2]]) -> Result<RouteResult, RouteError> {
        if waypoints.len() < 2 {
            return Err(RouteError::TooFewWaypoints);
        }
        // A NaN/negative/inf cutoff would make every `meters <= cutoff` snap
        // check fail; reject it explicitly instead of returning a misleading
        // SnapTooFar for every waypoint.
        if !self.opts.max_snap_meters.is_finite() || self.opts.max_snap_meters < 0.0 {
            return Err(RouteError::InvalidMaxSnapMeters);
        }

        let mask = self.opts.profile.mask();

        let mut snapped: Vec<Snapped> = Vec::with_capacity(waypoints.len());
        for (index, &[lat, lon]) in waypoints.iter().enumerate() {
            // Snapping is profile-aware (D16): only roads this profile can
            // use are candidates, so a snapped leg is always traversable.
            match self.graph.road_snap(lat, lon, self.opts.max_snap_meters, mask) {
                Some(s) if s.meters <= self.opts.max_snap_meters => {
                    let edge = self.graph.edge(s.edge_index);
                    snapped.push(Snapped {
                        edge_index: s.edge_index,
                        seg: s.seg,
                        t: s.t,
                        point: s.point,
                        src: self.graph.edge_source(s.edge_index),
                        dst: edge.target,
                        d_src: self.graph.distance_along_dm(s.edge_index, s.seg, s.t),
                        length: u64::from(edge.length_dm),
                    });
                }
                Some(s) => return Err(RouteError::SnapTooFar { index, meters: s.meters }),
                None => return Err(RouteError::SnapTooFar { index, meters: f64::INFINITY }),
            }
        }

        let mut line: Vec<[f64; 2]> = Vec::new();
        let mut fallback = false;
        for (segment, pair) in snapped.windows(2).enumerate() {
            let (a, b) = (&pair[0], &pair[1]);
            let leg = self.resolve_leg(a, b, mask);
            if let Leg::Straight = leg {
                if !self.opts.allow_fallback {
                    return Err(RouteError::NoPath { segment });
                }
                fallback = true;
            }
            self.append_leg(&mut line, a, b, leg);
        }
        // Leg boundaries and t = 0/1 projections coincide exactly with their
        // neighbors; one dedup pass keeps the line free of zero-length steps.
        line.dedup();

        let meters = line
            .windows(2)
            .map(|w| geo::haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]))
            .sum();

        Ok(RouteResult { line, meters, fallback })
    }

    /// Decide how to travel from snap `a` to snap `b`.
    fn resolve_leg(&self, a: &Snapped, b: &Snapped, mask: u8) -> Leg {
        let same_edge = a.edge_index == b.edge_index;
        if same_edge && a.along_key() == b.along_key() {
            return Leg::Point;
        }
        let network = self.astar_virtual(a, b, mask);
        if same_edge {
            // Staying on the edge costs |Δd|; a network path can still win on
            // hairpin-shaped edges. Ties prefer the simpler along-edge leg.
            let direct = a.d_src.abs_diff(b.d_src);
            return match network {
                Some((total, entry, edges, exit)) if total < direct => {
                    Leg::Network { entry, edges, exit }
                }
                _ => Leg::AlongEdge,
            };
        }
        match network {
            Some((_, entry, edges, exit)) => Leg::Network { entry, edges, exit },
            None => Leg::Straight,
        }
    }

    /// Append leg coordinates: the start projection, the traveled shape, the
    /// end projection. Exact duplicates at boundaries are removed by the
    /// caller's final dedup.
    fn append_leg(&self, line: &mut Vec<[f64; 2]>, a: &Snapped, b: &Snapped, leg: Leg) {
        line.push(a.point);
        match leg {
            Leg::Point => {}
            Leg::Straight => line.push(b.point),
            Leg::AlongEdge => {
                // Shape vertices strictly between the two projections.
                let e = a.edge_index;
                if a.along_key() <= b.along_key() {
                    for k in (u32::from(a.seg) + 1)..=u32::from(b.seg) {
                        line.push(self.graph.shape_point_latlon(e, k));
                    }
                } else {
                    for k in ((u32::from(b.seg) + 1)..=u32::from(a.seg)).rev() {
                        line.push(self.graph.shape_point_latlon(e, k));
                    }
                }
                line.push(b.point);
            }
            Leg::Network { entry, edges, exit } => {
                // Projection → entry endpoint of the start edge…
                let m_a = u32::from(self.graph.edge(a.edge_index).geo_len);
                if entry == a.src {
                    for k in (1..=u32::from(a.seg)).rev() {
                        line.push(self.graph.shape_point_latlon(a.edge_index, k));
                    }
                } else {
                    for k in (u32::from(a.seg) + 1)..=m_a {
                        line.push(self.graph.shape_point_latlon(a.edge_index, k));
                    }
                }
                line.push(self.graph.node_latlon(entry));
                // …the network path…
                for &edge_index in &edges {
                    let edge = self.graph.edge(edge_index);
                    self.push_edge_geometry(line, edge);
                    line.push(self.graph.node_latlon(edge.target));
                }
                // …exit endpoint of the goal edge → projection.
                let m_b = u32::from(self.graph.edge(b.edge_index).geo_len);
                if exit == b.src {
                    for k in 1..=u32::from(b.seg) {
                        line.push(self.graph.shape_point_latlon(b.edge_index, k));
                    }
                } else {
                    for k in ((u32::from(b.seg) + 1)..=m_b).rev() {
                        line.push(self.graph.shape_point_latlon(b.edge_index, k));
                    }
                }
                line.push(b.point);
            }
        }
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

    /// A\* between two on-edge positions (virtual endpoints, DECISIONS D16):
    /// the frontier is seeded with both endpoint nodes of `a`'s edge at their
    /// partial costs, and a path may finish at either endpoint node of `b`'s
    /// edge plus that endpoint's partial. Returns
    /// `(total_dm, entry_node, edge_indices, exit_node)` for the best
    /// completion, or `None` when no connection exists.
    ///
    /// Determinism (F9): integer-dm costs, `(f, node_index)` heap order,
    /// fixed adjacency order, first-settled goal endpoint wins cost ties.
    /// The heuristic — haversine to `b`'s projection, floored dm — is
    /// admissible (straight line ≤ any road continuation + goal partial).
    fn astar_virtual(
        &self,
        a: &Snapped,
        b: &Snapped,
        mask: u8,
    ) -> Option<(u64, u32, Vec<u32>, u32)> {
        let n = self.graph.node_count() as usize;
        let [goal_lat, goal_lon] = b.point;
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

        for seed in [a.src, a.dst] {
            // partial_to is Some for both endpoints by construction.
            if let Some(cost) = a.partial_to(seed) {
                if cost < g_cost[seed as usize] {
                    g_cost[seed as usize] = cost;
                    heap.push(Reverse((cost.saturating_add(h(seed)), seed)));
                }
            }
        }

        let mut best: Option<(u64, u32)> = None;
        while let Some(Reverse((f, node))) = heap.pop() {
            if let Some((best_total, _)) = best {
                if f >= best_total {
                    break; // nothing on the frontier can improve the answer
                }
            }
            if closed[node as usize] {
                continue; // stale heap entry
            }
            closed[node as usize] = true;
            if let Some(partial) = b.partial_to(node) {
                let total = g_cost[node as usize].saturating_add(partial);
                if best.is_none_or(|(bt, _)| total < bt) {
                    best = Some((total, node));
                }
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

        let (total, exit) = best?;
        // Walk the parent chain from the exit back to whichever seed started
        // the winning path (seeds carry the u32::MAX sentinel).
        let mut edges = Vec::new();
        let mut node = exit;
        while parent_node[node as usize] != u32::MAX && edges.len() <= n {
            edges.push(parent_edge[node as usize]);
            node = parent_node[node as usize];
        }
        edges.reverse();
        Some((total, node, edges, exit))
    }
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

        // Car cannot: snapping is profile-aware (D16), and no drivable road
        // exists within max_snap_meters of node 3 — SnapTooFar, telling the
        // caller the honest distance to the nearest drivable road.
        let car = Router::new(&g, RouteOptions::default());
        let err = car.route(&[wp(&g, 0), wp(&g, 3)]).unwrap_err();
        match err {
            RouteError::SnapTooFar { index, meters } => {
                assert_eq!(index, 1);
                assert!(meters > 200.0);
            }
            other => panic!("expected SnapTooFar, got {other:?}"),
        }

        // Disconnected-but-snappable endpoints with fallback disabled: an
        // error naming the leg.
        let strict = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        assert_eq!(
            strict.route(&[wp(&g, 0), wp(&g, 4)]),
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

    #[test]
    fn invalid_max_snap_meters_is_rejected_up_front() {
        let g = test_graph();
        for bad in [f64::NAN, -1.0, f64::INFINITY] {
            let router = Router::new(
                &g,
                RouteOptions { max_snap_meters: bad, ..RouteOptions::default() },
            );
            assert_eq!(
                router.route(&[wp(&g, 0), wp(&g, 2)]),
                Err(RouteError::InvalidMaxSnapMeters),
                "cutoff {bad} should be rejected"
            );
        }
        // Zero is a valid (if useless) cutoff: not rejected up front.
        let router = Router::new(&g, RouteOptions { max_snap_meters: 0.0, ..RouteOptions::default() });
        assert!(!matches!(
            router.route(&[wp(&g, 0), wp(&g, 2)]),
            Err(RouteError::InvalidMaxSnapMeters)
        ));
    }

    // ---- F10 edge snapping (M5) ----

    #[test]
    fn mid_edge_snap_splices_partial_geometry_both_directions() {
        // collapsed_graph: nodes 0/1 joined by the east bow through
        // (35.010, 33.001) and (35.020, 33.001). Query beside the middle
        // geometry segment: the route must start at the projection and
        // include only the remaining shape toward the exit.
        let g = collapsed_graph();
        let router = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        let q = [35.015, 33.0012]; // projects onto the east bow at ~(35.015, 33.001)
        let res = router.route(&[q, wp(&g, 1)]).unwrap();
        let proj = res.line[0];
        assert!((proj[0] - 35.015).abs() < 1e-6, "{proj:?}");
        assert!((proj[1] - 33.001).abs() < 1e-6, "{proj:?}");
        // proj → second geometry point → node 1: partial splice, not the
        // whole edge.
        assert_eq!(res.line.len(), 3);
        assert_eq!(res.line[1], [35.020, 33.001]);
        assert_eq!(res.line[2], wp(&g, 1));

        // Same query toward node 0: the other partial, reversed travel.
        let res_back = router.route(&[q, wp(&g, 0)]).unwrap();
        assert_eq!(res_back.line.len(), 3);
        assert_eq!(res_back.line[0], proj, "same projection");
        assert_eq!(res_back.line[1], [35.010, 33.001]);
        assert_eq!(res_back.line[2], wp(&g, 0));
    }

    #[test]
    fn same_edge_waypoints_travel_the_sub_polyline() {
        let g = collapsed_graph();
        let router = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );
        // Both queries sit beside the east bow, near its two ends.
        let qa = [35.005, 33.0007];
        let qb = [35.025, 33.0007];
        let res = router.route(&[qa, qb]).unwrap();
        assert!(!res.fallback);
        // proj_a, both geometry vertices, proj_b — never leaves the edge.
        assert_eq!(res.line.len(), 4);
        assert_eq!(res.line[1], [35.010, 33.001]);
        assert_eq!(res.line[2], [35.020, 33.001]);
        // Reversed waypoint order gives exactly the reversed line.
        let back = router.route(&[qb, qa]).unwrap();
        let mut expected = res.line.clone();
        expected.reverse();
        assert_eq!(back.line, expected);
        assert_eq!(back.meters, res.meters);
    }

    #[test]
    fn hairpin_same_edge_routes_around_when_shorter() {
        // Nodes A (0) and B (1) are 91 m apart, connected both by a short
        // direct edge and by a huge hairpin via 35.05°N (~11 km). Waypoints
        // sit on the hairpin near its ends: staying on the hairpin would cost
        // ~11 km, exiting to A, taking the direct edge, and re-entering costs
        // ~0.2 km. The router must leave the edge.
        let nodes = vec![fx(35.000, 33.000), fx(35.000, 33.001)];
        let geometry = vec![fx(35.050, 33.000), fx(35.050, 33.001)];
        let shape = [
            [35.000, 33.000],
            [35.050, 33.000],
            [35.050, 33.001],
            [35.000, 33.001],
        ];
        let hairpin_m: f64 = shape
            .windows(2)
            .map(|w| geo::haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]))
            .sum();
        let hairpin_dm = (hairpin_m * 10.0).round() as u32;
        let direct_m = geo::haversine_m(35.0, 33.0, 35.0, 33.001);
        let direct_dm = (direct_m * 10.0).round() as u32;
        let edges = vec![
            // node 0: direct first (shorter length sorts first), then hairpin
            Edge { target: 1, length_dm: direct_dm, geo_off: 0, geo_len: 0, reversed: false, access: ACCESS_ALL },
            Edge { target: 1, length_dm: hairpin_dm, geo_off: 0, geo_len: 2, reversed: false, access: ACCESS_ALL },
            Edge { target: 0, length_dm: direct_dm, geo_off: 0, geo_len: 0, reversed: false, access: ACCESS_ALL },
            Edge { target: 0, length_dm: hairpin_dm, geo_off: 0, geo_len: 2, reversed: true, access: ACCESS_ALL },
        ];
        let offsets = vec![0, 2, 4];
        let g = Graph::from_parts(nodes, offsets, edges, geometry).unwrap();
        let router = Router::new(
            &g,
            RouteOptions { allow_fallback: false, ..RouteOptions::default() },
        );

        // On the hairpin, ~55 m up each vertical arm (t ≈ 0.01 of ~5.5 km).
        let qa = [35.0005, 33.000];
        let qb = [35.0005, 33.001];
        let res = router.route(&[qa, qb]).unwrap();
        // proj_a → A → B (direct edge) → proj_b.
        assert_eq!(res.line.len(), 4, "{:?}", res.line);
        let close = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9;
        assert!(close(res.line[0], qa), "{:?}", res.line);
        assert_eq!(res.line[1], wp(&g, 0));
        assert_eq!(res.line[2], wp(&g, 1));
        assert!(close(res.line[3], qb), "{:?}", res.line);
        assert!(res.meters < 300.0, "took the hairpin: {} m", res.meters);

        // Control: waypoints near the hairpin's *top* stay on the edge.
        let res = router.route(&[[35.0495, 33.0], [35.0495, 33.001]]).unwrap();
        assert!(res.meters < 300.0);
        assert_eq!(res.line.len(), 4); // proj, both top vertices, proj
        assert_eq!(res.line[1], [35.050, 33.000]);
        assert_eq!(res.line[2], [35.050, 33.001]);
    }

    #[test]
    fn snap_prefers_road_over_distant_node() {
        // Query beside the middle of the collapsed east bow: node snapping
        // would report ~1.1 km (to node 0/1), edge snapping ~20 m.
        let g = collapsed_graph();
        let (_, node_m) = g.nearest_node(35.015, 33.0012, f64::INFINITY).unwrap();
        let (point, road_m) = g.nearest_road(35.015, 33.0012, f64::INFINITY).unwrap();
        assert!(node_m > 1_000.0, "node snap {node_m}");
        assert!(road_m < 30.0, "road snap {road_m}");
        assert!((point[1] - 33.001).abs() < 1e-6);
        // And routing from there works rather than SnapTooFar-ing (D14 fix).
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[[35.015, 33.0012], wp(&g, 1)]).unwrap();
        assert!(!res.fallback);
    }
}
