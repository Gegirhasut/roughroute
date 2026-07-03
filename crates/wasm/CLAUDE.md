# crates/wasm — agent context

**Single responsibility:** the `wasm-bindgen` shell around `core` (milestone
M1). Translates JS values ↔ the §6.1 JSON contract; contains no routing logic.

## Surface (spec §6.3 — fixed)
```rust
#[wasm_bindgen]
pub struct WasmRouter { /* owns a core::Graph in linear memory */ }
// constructor: new(graph_bytes: &[u8]) -> Result<WasmRouter, JsError>
// route(req: JsValue) -> Result<JsValue, JsError>
//   req:  { waypoints: [[lat,lon],…], profile: "car"|"foot" }
//   resp: { line: [[lat,lon],…], meters: f64, fallback: bool }
```
Errors surface as thrown `JsError` (the contract carries no error field).

## Hard constraints
- Depends on `core` (+ `wasm-bindgen`, `serde-wasm-bindgen`) only. If `osmpbf`
  or any fs/network crate shows up in `cargo tree` for the wasm target,
  something is wrong — fix the dependency graph, don't ship it.
- Coordinates in and out are `[lat, lon]` (DECISIONS D9) — the webview side of
  route-spoofer expects contract order, not GeoJSON order.
- Build: `cargo build -p roughroute-wasm --target wasm32-unknown-unknown`
  (JS glue via `wasm-bindgen` CLI, version-matched to the crate dep, or
  `wasm-pack` if preferred). Native-side contract-shape tests run in plain
  `cargo test -p roughroute-wasm`; the in-runtime smoke test
  (`tests/wasm_smoke.rs`) runs with
  `CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUNNER=wasm-bindgen-test-runner
  cargo test -p roughroute-wasm --target wasm32-unknown-unknown`.
- Extensions beyond the spec shape (kept optional, spec-default when absent):
  request fields `max_snap_meters`, `allow_fallback`; method `nodeCount()`.
- Status: implemented (M1). Excluded from workspace default-members so plain
  `cargo test` stays fast; test it explicitly as above.
