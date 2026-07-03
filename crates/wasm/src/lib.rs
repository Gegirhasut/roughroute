//! `roughroute-wasm` — the `wasm-bindgen` shell around `roughroute-core`
//! (spec §6.3).
//!
//! Translates JS values ↔ the neutral JSON contract (spec §6.1) and holds the
//! loaded graph in linear memory; contains no routing logic of its own.
//! Coordinates are `[lat, lon]` in both request and response — contract
//! order, never GeoJSON order.
//!
//! ```js
//! const bytes = await loadRegionGraph();          // host app's job
//! const router = new WasmRouter(new Uint8Array(bytes));
//! const res = router.route({ waypoints: [[34.7071, 33.0226], [34.6841, 33.0379]],
//!                            profile: "car" });
//! // res: { line: [[lat, lon], …], meters: number, fallback: boolean }
//! ```

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]
#![deny(missing_docs)]

use roughroute_core::{Graph, Profile, RouteOptions, RouteResult, Router};
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

/// The contract request (spec §6.1): `{ waypoints, profile }`.
///
/// `max_snap_meters` and `allow_fallback` are optional extensions with the
/// spec's default behavior when absent, so plain contract requests work
/// unchanged.
#[derive(Deserialize)]
struct ContractRequest {
    /// `[lat, lon]` pairs, at least two.
    waypoints: Vec<[f64; 2]>,
    profile: ContractProfile,
    #[serde(default = "default_max_snap_meters")]
    max_snap_meters: f64,
    #[serde(default = "default_allow_fallback")]
    allow_fallback: bool,
}

fn default_max_snap_meters() -> f64 {
    RouteOptions::default().max_snap_meters
}

fn default_allow_fallback() -> bool {
    RouteOptions::default().allow_fallback
}

/// Profile names as they appear on the wire (`"car"` / `"foot"`).
#[derive(Deserialize, Clone, Copy)]
#[serde(rename_all = "lowercase")]
enum ContractProfile {
    Car,
    Foot,
}

impl From<ContractProfile> for Profile {
    fn from(p: ContractProfile) -> Profile {
        match p {
            ContractProfile::Car => Profile::Car,
            ContractProfile::Foot => Profile::Foot,
        }
    }
}

/// The contract response (spec §6.1): `{ line, meters, fallback }`,
/// coordinates `[lat, lon]`.
#[derive(Serialize)]
struct ContractResponse {
    line: Vec<[f64; 2]>,
    meters: f64,
    fallback: bool,
}

impl From<RouteResult> for ContractResponse {
    fn from(r: RouteResult) -> Self {
        ContractResponse { line: r.line, meters: r.meters, fallback: r.fallback }
    }
}

/// An offline router holding a loaded road graph in WASM linear memory.
#[wasm_bindgen]
pub struct WasmRouter {
    graph: Graph,
}

#[wasm_bindgen]
impl WasmRouter {
    /// Load a router from the bytes of a pre-built `.graph` file
    /// (a `Uint8Array` over the `ArrayBuffer` the host app fetched from its
    /// bundle or cache). Touches no network and no storage.
    ///
    /// Throws (as a JS error) when the buffer is not a valid `.graph`.
    #[wasm_bindgen(constructor)]
    pub fn new(graph_bytes: &[u8]) -> Result<WasmRouter, JsError> {
        let graph = Graph::from_bytes(graph_bytes).map_err(|e| JsError::new(&e.to_string()))?;
        Ok(WasmRouter { graph })
    }

    /// Route a contract request `{ waypoints: [[lat,lon],…], profile:
    /// "car"|"foot" }` to a contract response `{ line, meters, fallback }`.
    ///
    /// Errors (too few waypoints, waypoint too far from a road, no path with
    /// fallback disabled) are thrown as JS errors — the contract itself
    /// carries no error field (spec §6.1).
    pub fn route(&self, req: JsValue) -> Result<JsValue, JsError> {
        let req: ContractRequest =
            serde_wasm_bindgen::from_value(req).map_err(|e| JsError::new(&e.to_string()))?;
        let router = Router::new(
            &self.graph,
            RouteOptions {
                profile: req.profile.into(),
                allow_fallback: req.allow_fallback,
                max_snap_meters: req.max_snap_meters,
            },
        );
        let result = router.route(&req.waypoints).map_err(|e| JsError::new(&e.to_string()))?;
        serde_wasm_bindgen::to_value(&ContractResponse::from(result))
            .map_err(|e| JsError::new(&e.to_string()))
    }

    /// Number of nodes in the loaded graph (handy for host-side sanity checks).
    #[wasm_bindgen(js_name = nodeCount)]
    pub fn node_count(&self) -> u32 {
        self.graph.node_count()
    }
}

/// Native-side tests for the contract (de)serialization shape; the
/// wasm-in-browser smoke test lives in `tests/wasm_smoke.rs`.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contract_request_parses_with_and_without_extensions() {
        let req: ContractRequest = serde_json::from_str(
            r#"{ "waypoints": [[34.7071, 33.0226], [34.6841, 33.0379]], "profile": "car" }"#,
        )
        .unwrap();
        assert_eq!(req.waypoints.len(), 2);
        assert!(matches!(req.profile, ContractProfile::Car));
        assert_eq!(req.max_snap_meters, 200.0);
        assert!(req.allow_fallback);

        let req: ContractRequest = serde_json::from_str(
            r#"{ "waypoints": [[1,2],[3,4]], "profile": "foot",
                 "max_snap_meters": 50.0, "allow_fallback": false }"#,
        )
        .unwrap();
        assert!(matches!(req.profile, ContractProfile::Foot));
        assert_eq!(req.max_snap_meters, 50.0);
        assert!(!req.allow_fallback);

        // Unknown profile strings are rejected.
        assert!(serde_json::from_str::<ContractRequest>(
            r#"{ "waypoints": [[1,2],[3,4]], "profile": "horse" }"#
        )
        .is_err());
    }

    #[test]
    fn contract_response_serializes_in_spec_shape() {
        let json = serde_json::to_string(&ContractResponse {
            line: vec![[34.7071, 33.0226], [34.6841, 33.0379]],
            meters: 2143.7,
            fallback: false,
        })
        .unwrap();
        assert_eq!(
            json,
            r#"{"line":[[34.7071,33.0226],[34.6841,33.0379]],"meters":2143.7,"fallback":false}"#
        );
    }
}
