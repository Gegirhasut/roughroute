# crates/ffi — agent context

**Single responsibility:** the UniFFI shell around `core` for Kotlin/Android
(milestone M2). No routing logic; just type mapping and error translation.

## Surface (spec §6.4 — fixed)
```
interface Router {
  [Throws=RouterError] constructor(bytes data);
  [Throws=RouterError] RouteResult route(sequence<Coord> waypoints, Profile profile);
};
dictionary RouteResult { sequence<Coord> line; double meters; boolean fallback; };
dictionary Coord { double lat; double lon; };
enum Profile { "Car", "Foot" };
[Error] enum RouterError { "TooFewWaypoints", "SnapTooFar", "BadGraph" };
```
Error mapping: `core::RouteError::TooFewWaypoints → TooFewWaypoints`,
`SnapTooFar → SnapTooFar`, `core::GraphError::* → BadGraph`.

## Hard constraints
- Depends on `core` + `uniffi` only — no `build`/`osmpbf`, no network.
- `Coord` uses named `lat`/`lon` fields (no array-order ambiguity; still
  lat-first conceptually per DECISIONS D9).
- Interface is defined with UniFFI proc-macros (`uniffi::Object/Record/Enum/
  Error` + `setup_scaffolding!`), not a `.udl` file — same shape as the
  spec §6.4 sketch.
- Kotlin binding generation:
  `cargo build -p roughroute-ffi && cargo run -p roughroute-ffi --features cli
  --bin uniffi-bindgen -- generate --library
  <target-dir>/debug/libroughroute_ffi.so --language kotlin --out-dir
  bindings/kotlin`.
- Android builds (documented, not run in this environment): `cargo ndk -t
  arm64-v8a -t armeabi-v7a -o jniLibs build --release -p roughroute-ffi`.
- Status: implemented (M2). Excluded from workspace default-members; test with
  `cargo test -p roughroute-ffi`.
