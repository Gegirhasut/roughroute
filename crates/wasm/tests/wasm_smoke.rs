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

/// A 3-node path graph 0 — 1 — 2, serialized to `.graph` bytes.
fn tiny_graph_bytes() -> Vec<u8> {
    let nodes = vec![
        [deg_to_fixed(35.00), deg_to_fixed(33.00)],
        [deg_to_fixed(35.01), deg_to_fixed(33.00)],
        [deg_to_fixed(35.02), deg_to_fixed(33.00)],
    ];
    let offsets = vec![0, 1, 3, 4];
    let mk = |target| Edge {
        target,
        length_dm: 11120,
        geo_off: 0,
        geo_len: 0,
        reversed: false,
        access: ACCESS_ALL,
    };
    let edges = vec![mk(1), mk(0), mk(2), mk(1)];
    Graph::from_parts(nodes, offsets, edges, vec![]).unwrap().to_bytes()
}

#[wasm_bindgen_test]
fn constructs_and_routes_a_contract_request() {
    let router = WasmRouter::new(&tiny_graph_bytes()).unwrap();
    assert_eq!(router.node_count(), 3);

    let req = js_sys::JSON::parse(
        r#"{ "waypoints": [[35.0, 33.0], [35.02, 33.0]], "profile": "foot" }"#,
    )
    .unwrap();
    let res = router.route(req).unwrap();

    let json = js_sys::JSON::stringify(&res).unwrap().as_string().unwrap();
    assert!(json.contains("\"fallback\":false"), "got {json}");
    assert!(json.contains("\"line\":[[35,33],[35.01,33],[35.02,33]]"), "got {json}");
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
