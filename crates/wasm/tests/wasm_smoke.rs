//! Cross-target smoke test (spec §11): build a trivial graph, load it through
//! `WasmRouter`, and route over it — inside an actual WASM runtime.
//!
//! Runs under `wasm-bindgen-test`:
//! `cargo test -p roughroute-wasm --target wasm32-unknown-unknown`
//! (with `wasm-bindgen-test-runner` on PATH; see crates/wasm/CLAUDE.md).

#![cfg(target_arch = "wasm32")]

use roughroute_core::geo::deg_to_fixed;
use roughroute_core::graph::{Edge, Graph};
use roughroute_core::profile::ACCESS_ALL;
use roughroute_wasm::WasmRouter;
use wasm_bindgen::JsValue;
use wasm_bindgen_test::wasm_bindgen_test;

/// A graph exercising collapsed geometry across the WASM boundary: nodes
/// 0 — 1 joined directly, and 1 — 2 via a collapsed edge carrying one
/// intermediate geometry point (at 35.02), serialized to `.graph` bytes.
/// This proves geometry expansion works end-to-end through `WasmRouter`.
fn tiny_graph_bytes() -> Vec<u8> {
    let nodes = vec![
        [deg_to_fixed(35.00), deg_to_fixed(33.00)],
        [deg_to_fixed(35.01), deg_to_fixed(33.00)],
        [deg_to_fixed(35.03), deg_to_fixed(33.00)],
    ];
    let geometry = vec![[deg_to_fixed(35.02), deg_to_fixed(33.00)]];
    let offsets = vec![0, 1, 3, 4];
    let plain = |target| Edge {
        target,
        length_dm: 11120,
        geo_off: 0,
        geo_len: 0,
        reversed: false,
        access: ACCESS_ALL,
    };
    let edges = vec![
        plain(1),
        plain(0),
        Edge { target: 2, length_dm: 22230, geo_off: 0, geo_len: 1, reversed: false, access: ACCESS_ALL },
        Edge { target: 1, length_dm: 22230, geo_off: 0, geo_len: 1, reversed: true, access: ACCESS_ALL },
    ];
    Graph::from_parts(nodes, offsets, edges, geometry).unwrap().to_bytes()
}

#[wasm_bindgen_test]
fn constructs_and_routes_a_contract_request() {
    let router = WasmRouter::new(&tiny_graph_bytes()).unwrap();
    assert_eq!(router.node_count(), 3);

    let req = js_sys::JSON::parse(
        r#"{ "waypoints": [[35.0, 33.0], [35.03, 33.0]], "profile": "foot" }"#,
    )
    .unwrap();
    let res = router.route(req).unwrap();

    let json = js_sys::JSON::stringify(&res).unwrap().as_string().unwrap();
    assert!(json.contains("\"fallback\":false"), "got {json}");
    // Includes the collapsed edge's intermediate point at 35.02: geometry
    // expansion crossed the JS boundary intact.
    assert!(
        json.contains("\"line\":[[35,33],[35.01,33],[35.02,33],[35.03,33]]"),
        "got {json}"
    );
}

#[wasm_bindgen_test]
fn bad_graph_bytes_throw() {
    assert!(WasmRouter::new(b"not a graph").is_err());
}

#[wasm_bindgen_test]
fn bad_request_throws() {
    let router = WasmRouter::new(&tiny_graph_bytes()).unwrap();
    // One waypoint → TooFewWaypoints, surfaced as a thrown JS error.
    let req = js_sys::JSON::parse(r#"{ "waypoints": [[35.0, 33.0]], "profile": "car" }"#).unwrap();
    assert!(router.route(req).is_err());
    // Junk request shape.
    assert!(router.route(JsValue::from_str("nonsense")).is_err());
}
