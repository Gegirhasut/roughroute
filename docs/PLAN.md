# roughroute — Implementation Plan

Source of truth: `./roughroute-SPEC-EN.md` (spec v1). Design choices that the
spec leaves open are recorded in [`DECISIONS.md`](DECISIONS.md); this file is the
*order of work*.

## Ground rules

- Milestones are strictly sequential: a milestone is done only when `cargo test`
  passes and `cargo clippy --all-targets` is clean for everything built so far.
- Fixed boundaries that must not drift while implementing: JSON contract (spec §6.1),
  public Rust API (§6.2), binary `.graph` format (§7). Everything else may be refined.
- v1 simplicity rule: plain A\*, every way node is a graph node, grid index built at
  load time. No v2 optimizations (degree-2 collapse, baked index, edge snapping).

## Prerequisites (before M0)

- [ ] Install Rust toolchain (rustup, stable) — not present on this machine yet.
- [ ] Scaffold the Cargo workspace exactly as spec §5:
      `crates/{core,build,cli,wasm,ffi}` + `testdata/`. `wasm`/`ffi` start as
      empty stubs excluded from the default build until M1/M2.

## M0 — Core + build + CLI (priority)

Order of work inside M0 (each step lands with its unit tests):

1. **`core::geo`** — haversine (meters), bearing, fixed-point 1e7 ↔ degrees
   conversion helpers. Pure functions, tested against known reference values.
2. **`core::profile`** — `Profile { Car, Foot }`, access bitmask constants
   (`bit0 = car`, `bit1 = foot`).
3. **`core::format` + `core::graph`** — the `RRG1` binary layout: a `Graph`
   struct (owned `Vec`s, see DECISIONS D2), `Graph::from_bytes(&[u8])` with full
   bounds/validity checking (`GraphError`), and a serializer
   (`Graph::to_bytes`) used by `build` and by round-trip tests. `bbox()`,
   `node_count()`. Unit tests: header errors (BadMagic / UnsupportedVersion /
   Truncated), round-trip build→bytes→load→compare, CSR traversal on a
   hand-made graph.
4. **`core::router` — snapping** — uniform grid built at load (DECISIONS D4),
   nearest-node query with `max_snap_meters` cutoff, ring-search correctness
   tests (including "nearest node is in an adjacent cell").
5. **`core::router` — A\*** — binary heap, cost = `length_dm`, heuristic =
   haversine in dm (floored, admissible), edge filtering by profile mask,
   deterministic tie-break by node index (DECISIONS D3). Then multi-point
   concatenation with junction dedup, fallback straight segments,
   `meters` = haversine sum over the final polyline. `RouteError` as in §6.2.
6. **`build` crate** — two layers (DECISIONS D8):
   a. graph construction from an in-memory road network (testable without PBF):
      way filtering by `highway` tag → access masks, OSM-node-id dedup across
      ways (DECISIONS D1), deterministic node indexing (D3), edge emission both
      directions, edge dedup;
   b. thin `.osm.pbf` front-end using `osmpbf` (two passes: ways then nodes).
7. **`cli` crate** — `roughroute build` and `roughroute route` per §6.5, output
   formats `json` (the §6.1 contract object), `geojson` (LineString,
   `[lon,lat]` — the one allowed flip, commented), `gpx`.
8. **Integration + property + golden tests** (spec §11) on a synthetic fixture
   built through the `build` crate's in-memory layer, plus an `#[ignore]`d
   integration test that runs against `testdata/cyprus.osm.pbf` when present:
   - line connectivity (adjacent points within a reasonable step),
   - endpoints within `max_snap_meters` of requests,
   - `meters > 0`, prefix sums monotonic,
   - `car` vs `foot` both valid,
   - proptest: random bbox points → valid route / controlled error / fallback, no panic,
   - golden: identical input → byte-identical `.graph` and identical route JSON.

**Exit criteria:** CLI builds a `.graph` from a `.pbf` and routes to
json/geojson/gpx; all §11 test classes pass; clippy clean.

## M1 — WASM

- `crates/wasm`: `WasmRouter` per §6.3 (`wasm-bindgen`, `serde-wasm-bindgen`),
  constructor from `&[u8]`, `route(JsValue) -> JsValue` speaking the §6.1 contract,
  errors as `JsError`.
- `wasm-bindgen-test` smoke test on a trivial graph; `wasm-pack build` artifact;
  verify the `build`/`osmpbf` dependency cannot reach the wasm target
  (feature/dependency audit).

## M2 — Native / Kotlin (UniFFI)

- `crates/ffi`: UniFFI interface per §6.4 (`Router`, `RouteResult`, `Coord`,
  `Profile`, `RouterError`), mapping core errors → `RouterError`.
- Generate the Kotlin binding; smoke test through the generated scaffolding on
  a trivial graph. Document (not necessarily run here) arm64/armv7 NDK builds.

## M3 — Profiles & polish

- Explicit reviewed `highway` tag lists for `car`/`foot` (spec §14.2) replacing
  the coarse M0 lists (DECISIONS D7); document the final table.
- `max_snap_meters` exposed end-to-end (CLI flag, WASM/FFI option).
- Optional duration estimate (F11) from profile speeds.

## M4 — v2 optimizations

**Done:** degree-2 chain collapse + separate geometry block (format v2,
DECISIONS D12–D14). **Remaining candidates** (not scheduled): baked spatial
index (`flags` bit), geometry delta-compression, true zero-copy load
(D2 keeps the format ready).

## M5 — F10 edge snapping (DONE)

**Why now:** M4 regressed snapping to kept-nodes-only (D14): starts can land
~55 m off and `SnapTooFar` is likelier on long rural roads. This must be
fixed **before** any batch region-building, so the regression is never baked
into published `.graph` files. No format change is needed — v2's geometry
pool already stores every shape point — so existing graphs stay valid and
there is no version bump.

**Approach — snap to the nearest point *on* an edge, splice partial geometry:**

1. **Segment index (load-time, like the node grid — spec §7.5).** An edge's
   shape is `node A, geo[geo_off..+geo_len], node B` = `geo_len + 1`
   segments. Index every segment of every *canonical* edge (skip `reversed`
   duplicates) in the existing uniform grid: bucket a compact segment ref
   `(edge_index: u32, seg_index: u16)` into every cell its bbox overlaps.
   Cyprus scale: ~1.9 M segments ≈ 15–20 MB extra linear memory — acceptable
   for v1 of F10; a baked index (`flags` bit) is the later optimization if
   load time or memory bites.
2. **Query.** Ring search as today, but distance = point-to-segment via a
   local equirectangular projection around the query (meter-accurate at
   snap scales). Result: `(edge, seg_index, t ∈ [0,1])` → the projected
   point and its distance. Deterministic tie-break:
   `(distance, edge_index, seg_index, t)`.
3. **Virtual endpoints in A\*.** A snapped location is "edge `e` at offset
   `d_a` dm from endpoint A / `d_b` from B" (integer dm along the shape,
   consistent with `length_dm`; distribute rounding so `d_a + d_b =
   length_dm`). Start: seed the heap with both endpoints of the start edge
   at costs `d_a`/`d_b` (profile-filtered). Goal: target both endpoints of
   the goal edge, final cost = arrival cost + that endpoint's partial; take
   the better. Special case: both waypoints on the *same* edge → the route
   is the sub-polyline between the two projections (no search).
4. **Line splicing.** The returned line starts at the projected point,
   follows the partial shape of the start edge to the chosen endpoint node,
   then normal edge expansion as today, then the goal edge's partial shape
   ending at the goal projection. `meters` stays the haversine sum over the
   final line (§8.5) — unchanged contract, `[lat, lon]` order everywhere.
5. **Errors/fallback unchanged:** `SnapTooFar` still measured against
   `max_snap_meters` (now against the true road, strictly better than v1
   node snapping, not just M4 healing); fallback still bridges only
   "snapped OK but no path".

**Interaction with the M4 pool:** projections index into the shared canonical
pool ranges; `reversed` edges reuse the canonical segment refs (a hit on a
canonical edge serves both directions — partials for the reverse direction
are `length_dm − d`). Splicing must respect traversal direction exactly as
M4's geometry expansion does.

**Tests:** projection math units (on-segment / clamped-to-endpoint / ties);
mid-chain waypoint on a long rural road snaps within centimeters (where M4
gave `SnapTooFar`); same-edge waypoint pair; start/end accuracy vs the
uncollapsed graph (edge snapping on collapsed ≥ as accurate as v1 node
snapping, by construction ≤ segment resolution); determinism/golden; proptest
"snapped distance ≤ nearest-kept-node distance, never a panic"; WASM smoke
unchanged.

**Out of scope for M5:** turn costs at virtual points, multi-edge candidate
sets (K-nearest edges), baked segment index.

## M6 — Region batch build + publishing (DONE)

`roughroute batch` (D17): iterate `regions.toml`, and per region strictly
check disk headroom → download → build → verify (`Graph::from_bytes` on the
written file + trivial routes in both profiles) → delete the `.pbf`; write
`index.json` (region → url, bytes, sha256, nodes/edges, bbox, format
version) for the host app to discover regions. Publishing = `gh release
create` with graphs + index as assets (documented in README, no credentials
in-repo). Hosting can later move to Cloudflare R2 by regenerating URLs only.

## Status

- [x] Prerequisites (rustup stable 1.96.1 installed 2026-07-03; note: builds
      must target the VM-local disk, see `.cargo/config.toml`)
- [x] M0 core+build+cli — all tests pass, clippy clean, Cyprus end-to-end OK
- [x] M1 wasm — wasm32 build + smoke tests pass in a real WASM runtime
      (Node via `wasm-bindgen-test-runner` 0.2.126, version-matched to the
      crate); JS glue + `.d.ts` generated with `wasm-bindgen --target web`;
      dependency audit confirms no osmpbf/network crates on the wasm target.
- [x] M2 ffi/kotlin — UniFFI proc-macro interface (spec §6.4 shape), 2/2
      tests, clippy clean; Kotlin binding generated via the crate's
      `uniffi-bindgen` bin (Router/Coord/RouteResult/Profile/RouterException
      all present). Android NDK cross-builds documented in
      `crates/ffi/CLAUDE.md`, not run here (no NDK in this environment).
- [ ] M3 profiles & polish — tag table D7 stands as the reviewed v1 list;
      `max_snap_meters` is exposed in CLI (`--max-snap-meters`) and WASM
      (optional request field); FFI deliberately keeps the fixed §6.4
      surface (defaults: 200 m, fallback on). Remaining: duration estimate
      (F11, would extend the contract — needs a route-spoofer decision).
- [x] M4 degree-2 collapse + geometry block (format v2, DECISIONS D12–D14) —
      Cyprus: 63.7 → 25.4 MB (2.5×), nodes 1.80 M → 267 k, edges 3.77 M →
      705 k; route shapes proven identical to the uncollapsed topology on
      synthetic fixtures and real Cyprus data (same kept-node endpoints, both
      profiles, both directions). Route contract unchanged. v1 files refused
      (`UnsupportedVersion`). Snapping now targets kept nodes only (D14).
      Still open from M4's spec scope: baked spatial index, edge snapping
      (F10). Geometry delta-compression is the next size lever
      (pool + edges dominate the v2 file).
- [x] M5 F10 edge snapping (DECISIONS D15/D16) — waypoints project onto the
      nearest profile-usable road segment; virtual-endpoint A\* with partial
      splicing; no format change. D14 case healed on real data: the
      documented Cyprus waypoint went from a 41.4 m kept-node snap to a
      22.8 m on-road projection (the true perpendicular), and the island's
      worst mid-chain vertex (nearest kept node 2,248 m — hard `SnapTooFar`
      under M4) now snaps at 0.000 m and routes. Property guard: road snap
      ≤ node snap over random points, enforced by proptest. Semantics
      change (deliberate, D16): "profile with no usable road in range" is
      now `SnapTooFar` instead of a fallback line from an unleavable node.
- [x] M6 region batch tooling (D17) — `roughroute batch`, `regions.toml`
      (cyprus/malta/andorra seeded), `index.json`, README publishing guide;
      disk rules from CLAUDE.md enforced in code. Hard 800 MB `.pbf` safety
      ceiling added 2026-07-03 (see "M6 scaling test" below) alongside the
      existing headroom check; an unknown probed size also aborts.
- [x] M6.1 incremental batch (2026-07-03, D17 addendum) — a region already
      recorded in `index.json` with matching size/sha256 and the current
      `format_version` is skipped entirely: no HEAD probe, no download, no
      rebuild. `--force` overrides. Verified live: after a one-time
      migration rebuild (needed because the pre-incremental `index.json`
      lacked per-region `format_version`), a second full run of the same
      4-region manifest **skipped all 4, downloaded nothing, finished in
      10 s** (vs the ~6.5 min first pass). Adding a 5th region (Austria)
      touched only Austria — the others stayed skipped — confirming the
      core acceptance criterion before Austria itself was dropped (below)
      for an unrelated reason.

### M6 scaling test (2026-07-03): Slovenia, a mid-size region

Added Slovenia (`.pbf` probed 309.6 MB via HEAD, well under the 800 MB hard
ceiling and the 500 MB target) to prove the pipeline beyond tiny islands
without risking the disk or an unattended run. Full 4-region batch
(cyprus/malta/andorra/slovenia): ~5.5 minutes debug, disk unchanged
(48 GB free before and after), zero leftover `.pbf`/scratch, every
`index.json` hash/size cross-checked against the written files.

| region | `.pbf` | `.graph` | nodes | edges |
|---|---|---|---|---|
| Cyprus | 36.7 MB | 25.4 MB | 267,459 | 705,406 |
| Malta | 8.8 MB | 3.7 MB | 52,292 | 134,612 |
| Andorra | 3.4 MB | 1.6 MB | 9,998 | 28,030 |
| Slovenia | 309.6 MB | 74.6 MB | 657,285 | 1,624,850 |

Slovenia's graph (74.6 MB) stays **under the 100 MB "interesting" mark** —
it does not itself trigger the deferred geometry delta-compression (D12).
The pipeline scales roughly linearly (pbf→graph ratio ~24% here vs ~71% for
Cyprus, reflecting Slovenia's lower node density per pbf byte); a region in
the 400–800 MB pbf range would likely cross 100 MB and become the concrete
trigger, the way the 64 MB Cyprus v1 number motivated M4.

**Austria attempted, then dropped (2026-07-03).** Raised
`HARD_MAX_PBF_BYTES` to 1.2 GB to admit Austria (803.1 MB pbf) as a bigger
scaling point; the build was **OOM-killed** by the kernel at ~4.8 GB RSS
(still climbing) on this dev VM's 5.8 GB total RAM — Slovenia by contrast
peaked around 1 GB. A real machine constraint, not a code bug; the disk
gates (headroom + hard ceiling) worked exactly as designed and weren't the
problem — `roughroute batch` simply has no RAM gate at all (`docs/
DECISIONS.md` D18). Dropped Austria and reverted the ceiling to 800 MB
rather than chase it further; Slovenia stays the largest region built here.
Trigger-point speculation above is therefore still open — Slovenia (56.4 MB
v3, see M7) stayed well under 100 MB even after collapse, and Austria would
have been the natural next data point.

## M7 — Format v3: geometry delta-compression (DONE, D19)

Deferred-since-M4 pool compression, cashed in: zigzag-delta LEB128 varints
over the geometry pool, decoded once at load into the same absolute
`Vec<[i32;2]>` v2 used — `graph.rs`/`grid.rs`/`router.rs` untouched, so the
"routes must be identical" invariant follows from the loader producing a
bit-identical in-memory `Graph`, not from re-auditing the algorithms.
Version bumped 2 → 3 (v1 and v2 both refused, same discipline as the v1→v2
bump). No new dependency (core stays dependency-free, D11) — hand-rolled
~20-line zigzag+LEB128 codec, proptest-covered for round-trip fidelity and
no-panic-on-garbage-bytes.

**Measured, all four regions rebuilt v2 → v3 (2026-07-03):**

| region | v2 `.graph` | v3 `.graph` | factor |
|---|---|---|---|
| Cyprus | 26.6 MB | 20.8 MB | 1.28× |
| Malta | 3.9 MB | 3.4 MB | 1.15× |
| Andorra | 1.7 MB | 1.1 MB | 1.50× |
| Slovenia | 78.2 MB | 56.4 MB | 1.39× |

**Route-identity proof, real data:** Slovenia (Ljubljana ↔ Maribor,
~123 km driving / ~128 km walking) routed via the CLI on the v2 graph and
again on a from-scratch v3 rebuild of the same `.pbf`, both profiles, both
directions — all four JSON outputs (`line` + `meters`, full float precision)
**byte-for-byte identical** between v2 and v3. Determinism/golden tests,
the full unit/integration/property suite, and the WASM in-runtime smoke
suite all stayed green throughout; `cargo clippy` clean.

Incremental batch (M6.1) behaved correctly under the version bump: every
cached entry's `format_version` (2) no longer matched the new builder (3),
so the next `roughroute batch` run rebuilt all four regions automatically —
no special-case code needed, exactly per the D17 staleness rule.

**Not done in this milestone:** per-chain-anchored delta encoding (D19
explains why the simpler whole-array scheme was chosen instead), a RAM gate
for `roughroute batch` (the Austria lesson above), reduced peak build
memory in the `build` crate.

### M0 measured reality (2026-07-03, cyprus-latest ≈ 36 MB pbf, debug build)

- Graph: 1,796,541 nodes / 3,765,810 directed edges; `.graph` = 64 MB.
  The spec's "a few MB" predates both Cyprus data growth and the v1
  every-way-node representation — this is the expected v1 cost and the
  concrete motivation for M4's degree-2 collapse.
- `roughroute build`: ~46 s debug (fine for an offline CI step).
- `roughroute route` (Limassol, ~3.3 km, 144 points): 0.73 s wall total,
  dominated by loading/validating the 64 MB graph + building the grid;
  the A\* query itself is milliseconds. Release build will shrink both.
- Real-data quirk encoded in tests: single road edges can exceed 1 km
  (motorway shape gaps), so the "connected line" invariant uses a 5 km step
  bound for Cyprus vs 1 km for the synthetic town.
