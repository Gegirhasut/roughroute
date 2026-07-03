//! Integration tests (spec §11) over a synthetic in-memory fixture: the full
//! build → serialize → load → route pipeline, invariant checks, and
//! golden/determinism tests. The real-PBF test at the bottom is `#[ignore]`d
//! and runs only when `testdata/cyprus.osm.pbf` exists
//! (`cargo test -- --ignored`).

use std::collections::BTreeMap;

use roughroute_build::{build_graph, RawWay};
use roughroute_core::geo::haversine_m;
use roughroute_core::profile::{ACCESS_CAR, ACCESS_FOOT};
use roughroute_core::{Graph, Profile, RouteOptions, RouteResult, Router};

/// A little synthetic town around (35.00, 33.00):
///
/// - a main east-west street (ids 1..=5, residential),
/// - a north-south avenue crossing it at id 3 (ids 20, 3, 21),
/// - a pedestrian-only shortcut from id 2 to id 21,
/// - a motorway bypass from id 1 to id 5 (car-only, longer),
/// - a disconnected hamlet far east (ids 40, 41).
fn town() -> (Vec<RawWay>, BTreeMap<i64, [f64; 2]>) {
    let coords: BTreeMap<i64, [f64; 2]> = [
        (1, [35.000, 33.000]),
        (2, [35.000, 33.002]),
        (3, [35.000, 33.004]),
        (4, [35.000, 33.006]),
        (5, [35.000, 33.008]),
        (20, [35.002, 33.004]),
        (21, [34.998, 33.004]),
        (30, [35.003, 33.000]), // motorway shape node
        (31, [35.003, 33.008]),
        (40, [35.000, 33.100]),
        (41, [35.000, 33.102]),
    ]
    .into_iter()
    .collect();

    let all = ACCESS_CAR | ACCESS_FOOT;
    let ways = vec![
        RawWay { node_ids: vec![1, 2, 3, 4, 5], access: all },
        RawWay { node_ids: vec![20, 3, 21], access: all },
        RawWay { node_ids: vec![2, 21], access: ACCESS_FOOT },
        RawWay { node_ids: vec![1, 30, 31, 5], access: ACCESS_CAR },
        RawWay { node_ids: vec![40, 41], access: all },
    ];
    (ways, coords)
}

fn town_graph() -> Graph {
    let (ways, coords) = town();
    let (graph, _) = build_graph(&ways, &coords).unwrap();
    graph
}

fn router(graph: &Graph, profile: Profile) -> Router<'_> {
    Router::new(graph, RouteOptions { profile, ..RouteOptions::default() })
}

/// Spec §11 invariants on a route between `from`/`to` waypoints.
/// `max_step` is the "line is connected" bound: the longest plausible single
/// edge (consecutive way nodes) for the graph under test.
fn assert_route_invariants(
    route: &RouteResult,
    waypoints: &[[f64; 2]],
    max_snap: f64,
    max_step: f64,
) {
    // The line is connected: adjacent points within a reasonable step.
    for w in route.line.windows(2) {
        let step = haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]);
        assert!(step > 0.0 && step < max_step, "disconnected step of {step} m");
    }
    // Endpoints snapped within max_snap_meters of the requested points.
    let first = route.line.first().unwrap();
    let last = route.line.last().unwrap();
    let [flat, flon] = waypoints[0];
    let [tlat, tlon] = waypoints[waypoints.len() - 1];
    assert!(haversine_m(first[0], first[1], flat, flon) <= max_snap);
    assert!(haversine_m(last[0], last[1], tlat, tlon) <= max_snap);
    // meters > 0 and monotonically growing along the traversal.
    assert!(route.meters > 0.0);
    let mut acc = 0.0;
    for w in route.line.windows(2) {
        let next = acc + haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]);
        assert!(next > acc);
        acc = next;
    }
    assert_eq!(acc, route.meters);
}

#[test]
fn build_serialize_load_route_end_to_end() {
    let graph = town_graph();
    // Through the binary format, as the runtime would consume it.
    let loaded = Graph::from_bytes(&graph.to_bytes()).unwrap();

    let waypoints = [[35.0001, 33.0001], [35.0001, 33.0079]]; // near ids 1 and 5
    for profile in [Profile::Car, Profile::Foot] {
        let route = router(&loaded, profile).route(&waypoints).unwrap();
        assert!(!route.fallback, "{profile:?} should route on roads");
        assert_route_invariants(&route, &waypoints, 200.0, 1_000.0);
    }
}

#[test]
fn car_and_foot_take_different_valid_roads_where_it_matters() {
    let graph = town_graph();
    // From near id 2 to near id 21: foot has a direct pedestrian shortcut;
    // car must go around via the junction at id 3.
    let waypoints = [[35.0001, 33.0021], [34.9979, 33.0041]];
    let foot = router(&graph, Profile::Foot).route(&waypoints).unwrap();
    let car = router(&graph, Profile::Car).route(&waypoints).unwrap();
    assert!(!foot.fallback && !car.fallback);
    assert_route_invariants(&foot, &waypoints, 200.0, 1_000.0);
    assert_route_invariants(&car, &waypoints, 200.0, 1_000.0);
    // Foot cuts across the pedestrian shortcut from node id 2; the car must
    // go around via the junction at id 3.
    assert!(
        foot.line.contains(&[35.000, 33.002]) && !foot.line.contains(&[35.000, 33.004]),
        "foot should use the shortcut via id 2: {:?}",
        foot.line
    );
    assert!(
        car.line.contains(&[35.000, 33.004]),
        "car should go around via id 3: {:?}",
        car.line
    );
    assert!(foot.meters < car.meters);
}

#[test]
fn multipoint_route_is_connected_and_deduped() {
    let graph = town_graph();
    let waypoints = [[35.002, 33.004], [35.000, 33.000], [34.998, 33.004]];
    let route = router(&graph, Profile::Foot).route(&waypoints).unwrap();
    assert!(!route.fallback);
    assert_route_invariants(&route, &waypoints, 200.0, 1_000.0);
    // Junction dedup: no consecutive duplicates anywhere.
    assert!(route.line.windows(2).all(|w| w[0] != w[1]));
}

#[test]
fn disconnected_hamlet_triggers_fallback_only_when_allowed() {
    let graph = town_graph();
    let waypoints = [[35.000, 33.004], [35.000, 33.101]];
    let route = router(&graph, Profile::Car).route(&waypoints).unwrap();
    assert!(route.fallback);
    assert!(route.meters > 0.0);

    let strict = Router::new(
        &graph,
        RouteOptions { allow_fallback: false, ..RouteOptions::default() },
    );
    assert!(strict.route(&waypoints).is_err());
}

#[test]
fn golden_determinism_same_input_identical_output() {
    // Build twice, with ways in different orders: identical bytes (F9).
    let (ways, coords) = town();
    let (g1, _) = build_graph(&ways, &coords).unwrap();
    let reversed: Vec<RawWay> = ways.iter().rev().cloned().collect();
    let (g2, _) = build_graph(&reversed, &coords).unwrap();
    let bytes = g1.to_bytes();
    assert_eq!(bytes, g2.to_bytes(), "graph build must be byte-stable");

    // Load → serialize round-trip is byte-identical.
    assert_eq!(Graph::from_bytes(&bytes).unwrap().to_bytes(), bytes);

    // Golden header snapshot: pin the on-disk prefix so accidental format
    // drift fails loudly. The M4 collapse swallows interior nodes 4 (main
    // street) and 30, 31 (motorway shape): 8 kept nodes, 8 road segments →
    // 16 directed edges, 3 geometry points.
    assert_eq!(&bytes[0..4], b"RRG1");
    assert_eq!(u16::from_le_bytes([bytes[4], bytes[5]]), 2); // format version
    assert_eq!(u16::from_le_bytes([bytes[6], bytes[7]]), 0); // flags
    let node_count = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
    let edge_count = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
    let geo_count = u32::from_le_bytes(bytes[32..36].try_into().unwrap());
    assert_eq!(node_count, 8);
    assert_eq!(edge_count, 16);
    assert_eq!(geo_count, 3);
    assert_eq!(bytes.len(), 40 + 8 * 8 + 9 * 4 + 16 * 16 + 3 * 8);

    // Repeated routing over one loaded graph: identical results.
    let graph = Graph::from_bytes(&bytes).unwrap();
    let waypoints = [[35.0001, 33.0001], [34.9979, 33.0041]];
    let r = router(&graph, Profile::Foot);
    assert_eq!(r.route(&waypoints), r.route(&waypoints));
}

/// The key M4 regression guard (spec F6): collapsing degree-2 chains must not
/// change the *shape* of returned routes. Routes over the collapsed graph —
/// loaded through its serialized bytes, as the runtime would — must be
/// point-for-point identical to routes over the uncollapsed (v1-topology)
/// graph, in both travel directions and for both profiles.
#[test]
fn collapse_preserves_route_shape_through_the_binary_format() {
    let (ways, coords) = town();
    let (collapsed, stats) =
        roughroute_build::build_graph_with_options(&ways, &coords, true).unwrap();
    let (dense, _) = roughroute_build::build_graph_with_options(&ways, &coords, false).unwrap();
    assert!(stats.interior_nodes_collapsed > 0, "fixture must exercise the collapse");
    assert!(collapsed.node_count() < dense.node_count());
    assert!(collapsed.to_bytes().len() < dense.to_bytes().len());

    // Consume the collapsed graph the way the runtime does: via its bytes.
    let collapsed = Graph::from_bytes(&collapsed.to_bytes()).unwrap();

    // Waypoints at kept-node coordinates (junctions/dead-ends exist in both
    // graphs, so both snap identically; see docs/DECISIONS.md D14).
    let main_street = [[35.000, 33.000], [35.000, 33.008]]; // ids 1 -> 5, over swallowed id 4
    let reversed = [main_street[1], main_street[0]];
    for waypoints in [main_street, reversed] {
        for profile in [Profile::Car, Profile::Foot] {
            let c = router(&collapsed, profile).route(&waypoints).unwrap();
            let d = router(&dense, profile).route(&waypoints).unwrap();
            assert_eq!(c.line, d.line, "shape changed: {profile:?} {waypoints:?}");
            assert_eq!(c.meters, d.meters);
            assert_eq!(c.fallback, d.fallback);
            assert!(c.line.len() >= 3, "route must include swallowed geometry");
        }
    }
}

/// Runs only with `cargo test -- --ignored` and only when the Cyprus fixture
/// has been downloaded to `testdata/cyprus.osm.pbf` (see README).
#[test]
#[ignore = "needs testdata/cyprus.osm.pbf (a real Geofabrik extract)"]
fn cyprus_fixture_end_to_end() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/cyprus.osm.pbf");
    if !path.exists() {
        panic!("fixture missing: {}", path.display());
    }
    let (ways, coords) = roughroute_build::read_road_network(&path).unwrap();
    let (graph, stats) = build_graph(&ways, &coords).unwrap();
    assert!(graph.node_count() > 10_000, "Cyprus should be sizeable");
    eprintln!(
        "cyprus: {} nodes, {} edges, stats {stats:?}",
        graph.node_count(),
        graph.edge_count()
    );

    // Limassol seafront → Nicosia center, both profiles.
    let waypoints = [[34.6841, 33.0379], [35.1739, 33.3643]];
    for profile in [Profile::Car, Profile::Foot] {
        let route = router(&graph, profile).route(&waypoints).unwrap();
        assert!(!route.fallback, "{profile:?} Limassol→Nicosia must be on-road");
        assert_route_invariants(&route, &waypoints, 200.0, 5_000.0);
        // Straight line is ~63 km; the road route must be at least that and
        // sane (< 3× straight).
        assert!(route.meters > 60_000.0 && route.meters < 200_000.0);
    }

    // Determinism on the real graph.
    let bytes = graph.to_bytes();
    let (g2, _) = build_graph(&ways, &coords).unwrap();
    assert_eq!(bytes, g2.to_bytes());

    // M4 shape regression on real data: with waypoints at kept-node
    // coordinates (so both graphs snap identically), the collapsed graph must
    // return point-for-point the same polyline as the uncollapsed topology.
    let (dense, _) =
        roughroute_build::build_graph_with_options(&ways, &coords, false).unwrap();
    let a = graph.node_latlon(graph.nearest_node(34.6841, 33.0379, 200.0).unwrap().0);
    let b = graph.node_latlon(graph.nearest_node(35.1739, 33.3643, 200.0).unwrap().0);
    for (from, to) in [(a, b), (b, a)] {
        for profile in [Profile::Car, Profile::Foot] {
            let c = router(&graph, profile).route(&[from, to]).unwrap();
            let d = router(&dense, profile).route(&[from, to]).unwrap();
            assert_eq!(c.meters, d.meters, "{profile:?} {from:?}->{to:?}");
            assert_eq!(c.line, d.line, "shape changed: {profile:?} {from:?}->{to:?}");
        }
    }
}
