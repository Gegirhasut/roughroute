//! Property-based tests: random waypoints inside (and around) the graph bbox
//! must yield a valid route or a controlled error/fallback — never a panic
//! and never a malformed result.

use proptest::prelude::*;
use roughroute_core::geo::{deg_to_fixed, haversine_m};
use roughroute_core::graph::{Edge, Graph};
use roughroute_core::profile::{ACCESS_ALL, ACCESS_CAR, ACCESS_FOOT};
use roughroute_core::{Profile, RouteError, RouteOptions, Router};

/// A synthetic street grid around (35.0, 33.0): `size × size` nodes spaced
/// ~0.002° with all horizontal/vertical streets, alternating access masks so
/// both profiles matter, plus one disconnected pair far to the north-east so
/// fallback paths are reachable by random points.
fn synthetic_graph(size: u32) -> Graph {
    let spacing = 0.002;
    let mut nodes = Vec::new();
    for r in 0..size {
        for c in 0..size {
            nodes.push([
                deg_to_fixed(35.0 + spacing * f64::from(r)),
                deg_to_fixed(33.0 + spacing * f64::from(c)),
            ]);
        }
    }
    let island_base = nodes.len() as u32;
    nodes.push([deg_to_fixed(35.05), deg_to_fixed(33.05)]);
    nodes.push([deg_to_fixed(35.05), deg_to_fixed(33.052)]);

    let mut undirected: Vec<(u32, u32, u8)> = Vec::new();
    let idx = |r: u32, c: u32| r * size + c;
    for r in 0..size {
        for c in 0..size {
            // Streets: rows alternate car/all, columns alternate foot/all.
            if c + 1 < size {
                let access = if r % 3 == 0 { ACCESS_CAR } else { ACCESS_ALL };
                undirected.push((idx(r, c), idx(r, c + 1), access));
            }
            if r + 1 < size {
                let access = if c % 3 == 0 { ACCESS_FOOT } else { ACCESS_ALL };
                undirected.push((idx(r, c), idx(r + 1, c), access));
            }
        }
    }
    undirected.push((island_base, island_base + 1, ACCESS_ALL));

    // Directed CSR out of the undirected list.
    let mut directed: Vec<(u32, Edge)> = Vec::new();
    for (a, b, access) in undirected {
        let [alat, alon] = nodes[a as usize];
        let [blat, blon] = nodes[b as usize];
        let m = haversine_m(
            f64::from(alat) / 1e7,
            f64::from(alon) / 1e7,
            f64::from(blat) / 1e7,
            f64::from(blon) / 1e7,
        );
        let length_dm = ((m * 10.0).round() as u32).max(1);
        directed.push((a, Edge { target: b, length_dm, access }));
        directed.push((b, Edge { target: a, length_dm, access }));
    }
    directed.sort_by_key(|(s, e)| (*s, e.target));
    let mut offsets = vec![0u32; nodes.len() + 1];
    for (s, _) in &directed {
        offsets[*s as usize + 1] += 1;
    }
    for i in 1..offsets.len() {
        offsets[i] += offsets[i - 1];
    }
    let edges = directed.into_iter().map(|(_, e)| e).collect();
    Graph::from_parts(nodes, offsets, edges).expect("synthetic graph is valid")
}

fn check_result_invariants(route: &roughroute_core::RouteResult) {
    // meters is exactly the haversine sum over the returned line …
    let sum: f64 = route
        .line
        .windows(2)
        .map(|w| haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]))
        .sum();
    assert_eq!(route.meters, sum);
    assert!(route.meters.is_finite() && route.meters >= 0.0);
    // … and every coordinate is a real lat/lon.
    for &[lat, lon] in &route.line {
        assert!((-90.0..=90.0).contains(&lat) && (-180.0..=180.0).contains(&lon));
    }
    assert!(!route.line.is_empty());
    // No consecutive duplicate points (junctions deduped).
    assert!(route.line.windows(2).all(|w| w[0] != w[1]));
}

proptest! {
    /// Random points in a box that covers the graph *and* a margin outside
    /// it: every outcome must be a valid route or a controlled error.
    #[test]
    fn random_waypoints_never_panic(
        pts in prop::collection::vec((34.98f64..35.06, 32.98f64..33.07), 0..5),
        car in any::<bool>(),
        allow_fallback in any::<bool>(),
    ) {
        let graph = synthetic_graph(12);
        let opts = RouteOptions {
            profile: if car { Profile::Car } else { Profile::Foot },
            allow_fallback,
            max_snap_meters: 200.0,
        };
        let router = Router::new(&graph, opts);
        let waypoints: Vec<[f64; 2]> = pts.iter().map(|&(la, lo)| [la, lo]).collect();

        match router.route(&waypoints) {
            Ok(route) => {
                check_result_invariants(&route);
                if !allow_fallback {
                    prop_assert!(!route.fallback);
                }
            }
            Err(RouteError::TooFewWaypoints) => prop_assert!(waypoints.len() < 2),
            Err(RouteError::SnapTooFar { index, meters }) => {
                prop_assert!(index < waypoints.len());
                prop_assert!(meters > 200.0);
            }
            Err(RouteError::NoPath { segment }) => {
                prop_assert!(!allow_fallback);
                prop_assert!(segment + 1 < waypoints.len());
            }
        }
    }

    /// Points snapped on the connected grid always route without fallback,
    /// and the route is deterministic across repeated calls.
    #[test]
    fn connected_grid_routes_are_deterministic(
        a in (35.0f64..35.022, 33.0f64..33.022),
        b in (35.0f64..35.022, 33.0f64..33.022),
    ) {
        let graph = synthetic_graph(12);
        let router = Router::new(&graph, RouteOptions {
            profile: Profile::Foot,
            allow_fallback: false,
            max_snap_meters: 200.0,
        });
        let wp = [[a.0, a.1], [b.0, b.1]];
        let first = router.route(&wp);
        let second = router.route(&wp);
        prop_assert_eq!(&first, &second);
        let route = first.unwrap();
        check_result_invariants(&route);
        prop_assert!(!route.fallback);
    }
}
