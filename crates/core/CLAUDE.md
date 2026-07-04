# crates/core — agent context

**Single responsibility:** the pure algorithmic core — parse a `.graph` binary
from `&[u8]`, hold the CSR road graph, snap waypoints, run A\*, produce a
`RouteResult`. Nothing else lives here.

## Hard constraints (non-negotiable)
- **No network, no filesystem, no `osmpbf`/`build` dependency.** The only input
  is `&[u8]` (spec §5.1). Must stay WASM-friendly: no dependencies that don't
  compile to `wasm32-unknown-unknown`.
- No `unwrap()`/`expect()`/`panic!()` in library code — `Result` with
  `GraphError` / `RouteError` (spec §6.2). `from_bytes` validates everything
  up front (DECISIONS D6) so later code needs no defensive checks.
- `Graph` is immutable after load, `Send + Sync`; many `Router`s may share one
  `Graph` (spec §9).
- Deterministic: A\* tie-breaks `(f, node_index)`; no hash-iteration-order in
  any output path (DECISIONS D3).
- Coordinates `[lat, lon]` in the public API; internal fixed-point 1e7 pairs
  stored lat-then-lon (DECISIONS D9).

## Public surface (fixed by spec §6.2 — do not reshape)
`Graph::from_bytes / bbox / node_count`, `Router::new / route`,
`RouteOptions { profile, allow_fallback, max_snap_meters }`,
`RouteResult { line, meters, fallback }`, `Profile { Car, Foot }`,
`RouteError`, `GraphError`. Plus a `.graph` serializer (`to_bytes`) consumed by
`build` and round-trip tests. Everything public has rustdoc.

## Modules (spec §5)
- `geo` — haversine (m), bearing, fixed-point conversions.
- `profile` — access bitmask: `bit0 = car`, `bit1 = foot`.
- `format` — `RRG1` **v3** layout (M4 geometry block D12, M7 delta-
  compression D19): 40-byte header, `i32` fixed-point node coords, CSR
  `offsets`, 16-byte `Edge` records (`target`, `length_dm`, `geo_off`,
  `geo_len: u16`, flags, access), geometry pool stored as zigzag-delta
  LEB128 varints (`geo_bytes` header field = exact section length) and
  decoded once at load into the same absolute `Vec<[i32;2]>` v1/v2 held
  directly — nothing downstream is aware the on-disk encoding changed.
  Header `flags` bit0 = `HEADER_FLAG_LON_SHIFTED` (D25): the region
  crosses ±180° and longitudes are stored in a shifted continuous frame
  (bbox monotonic, may read past ±180°; queries normalized in at
  `road_snap`/`nearest_node`, results normalized out at `route()`/
  `node_latlon`/`nearest_road`). All other flag bits are refused — which
  is also what makes pre-D25 readers refuse a shifted graph cleanly.
  Non-crossing graphs never set the flag; their bytes are unchanged.
  Little-endian, explicit decode; fixed-width sections stay 4-byte aligned
  in-file (DECISIONS D2), though the variable-length geometry section rules
  out true zero-copy for it regardless. **v1 and v2 files are both refused**
  with `UnsupportedVersion` — rebuild the graph.
- `graph` — owned CSR structure + geometry pool + accessors
  (`edge_geometry` returns an edge's intermediate shape; `reversed` edges
  traverse it back-to-front).
- `router` — **F10 edge snapping** (M5, DECISIONS D15/D16): waypoints project
  onto the nearest shape segment of a profile-usable edge (grid segment
  index built at load; `SnapTooFar` = no usable road in range), then A\* runs
  between *virtual on-edge points* (frontier seeded with both start-edge
  endpoints at partial-dm costs, finishing via either goal-edge endpoint;
  same-edge legs compare along-edge vs around). Line splicing returns
  partial start/end geometry so routes begin exactly at the projections;
  interior legs are plain M4 geometry expansion. Parallel edges are real
  post-collapse; junction dedup via one final consecutive-duplicate pass;
  fallback (DECISIONS D10) bridges projections; `meters` from polyline
  haversine sum (spec §8.5).
