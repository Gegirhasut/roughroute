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
fn assert_route_invariants(route: &RouteResult, waypoints: &[[f64; 2]], max_snap: f64) {
    // The line is connected: adjacent points within a reasonable step (our
    // town's longest single segment is well under 1 km).
    for w in route.line.windows(2) {
        let step = haversine_m(w[0][0], w[0][1], w[1][0], w[1][1]);
        assert!(step > 0.0 && step < 1_000.0, "disconnected step of {step} m");
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
        assert_route_invariants(&route, &waypoints, 200.0);
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
    assert_route_invariants(&foot, &waypoints, 200.0);
    assert_route_invariants(&car, &waypoints, 200.0);
    assert!(foot.line.len() < car.line.len(), "foot takes the shortcut");
    assert!(foot.meters < car.meters);
}

#[test]
fn multipoint_route_is_connected_and_deduped() {
    let graph = town_graph();
    let waypoints = [[35.002, 33.004], [35.000, 33.000], [34.998, 33.004]];
    let route = router(&graph, Profile::Foot).route(&waypoints).unwrap();
    assert!(!route.fallback);
    assert_route_invariants(&route, &waypoints, 200.0);
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
    // drift fails loudly. Every way node is a graph node in v1 (no
    // collapse yet): 11 nodes, 11 undirected segments → 22 directed edges.
    assert_eq!(&bytes[0..4], b"RRG1");
    assert_eq!(u16::from_le_bytes([bytes[4], bytes[5]]), 1); // format version
    assert_eq!(u16::from_le_bytes([bytes[6], bytes[7]]), 0); // flags
    let node_count = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
    let edge_count = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
    assert_eq!(node_count, 11);
    assert_eq!(edge_count, 22);
    assert_eq!(bytes.len(), 32 + 11 * 8 + 12 * 4 + 22 * 12);

    // Repeated routing over one loaded graph: identical results.
    let graph = Graph::from_bytes(&bytes).unwrap();
    let waypoints = [[35.0001, 33.0001], [34.9979, 33.0041]];
    let r = router(&graph, Profile::Foot);
    assert_eq!(r.route(&waypoints), r.route(&waypoints));
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
        assert_route_invariants(&route, &waypoints, 200.0);
        // Straight line is ~63 km; the road route must be at least that and
        // sane (< 3× straight).
        assert!(route.meters > 60_000.0 && route.meters < 200_000.0);
    }

    // Determinism on the real graph.
    let bytes = graph.to_bytes();
    let (g2, _) = build_graph(&ways, &coords).unwrap();
    assert_eq!(bytes, g2.to_bytes());
}
