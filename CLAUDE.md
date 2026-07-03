# roughroute — agent context (workspace root)

Offline OSM mini-router: given ordered waypoints + a profile (`car`/`foot`),
return a polyline that follows OSM roads ("roughly correct" — legality of
maneuvers is explicitly NOT guaranteed; `oneway`/turn restrictions/`access`
are ignored by design). Compiles to CLI, WASM, and a UniFFI Kotlin binding.

## Read these first
- `../docs/roughroute-SPEC-EN.md` — the spec, source of truth. §6 (contract/API)
  and §7 (binary format) are **fixed boundaries** — never change them casually.
- `docs/PLAN.md` — milestone plan and current status.
- `docs/DECISIONS.md` — resolved design decisions (D1–D11). Check here before
  re-deciding anything; amend it when a decision changes.

## Workspace layout (spec §5)
- `crates/core` — pure algorithmic core: `.graph` parsing, CSR graph, A\*,
  snapping, fallback. Input is `&[u8]`. See its CLAUDE.md for hard constraints.
- `crates/build` — preprocessor `.osm.pbf` → `.graph` (only place `osmpbf` is
  allowed). Layered: pure graph construction + thin PBF front-end.
- `crates/cli` — `roughroute` binary: `build` and `route` subcommands
  (json/geojson/gpx output).
- `crates/wasm` — `wasm-bindgen` wrapper (M1). `crates/ffi` — UniFFI/Kotlin (M2).
- `testdata/` — fixtures; `cyprus.osm.pbf` is optional (real-PBF tests are
  `#[ignore]`d without it).

## Build / test
- Everything: `cargo test` and `cargo clippy --all-targets` from `app/`
  (workspace root). Both must be clean before a milestone is called done.
- Real-fixture integration: `cargo test -- --ignored` (needs
  `testdata/cyprus.osm.pbf`).
- WASM (M1+): `wasm-pack build crates/wasm`. FFI (M2+): see `crates/ffi/CLAUDE.md`.

## Disk usage (standing rules — this machine has ~50 GB free locally)

Source `.osm.pbf` extracts are large (hundreds of MB to GB per country) and
build scratch adds up. Rules:

- A `.pbf` is build *input only*: once its `.graph` is built and verified,
  **delete the `.pbf` in the same step** — with one exception:
  `testdata/cyprus.osm.pbf` is the committed, reproducible test fixture and
  **must never be deleted**.
- Never leave downloaded `.pbf`s or scratch intermediates behind after a
  build step completes (the CLI's `--pbf-url` path already downloads to a
  temp file and removes it after a successful build — keep it that way).
- **Before downloading any new extract, run `df -h .`**; refuse or warn if
  the download (plus its ~40%-of-pbf-size `.graph`) would risk filling the
  disk.
- Keep only: the committed fixture, the current `.graph` outputs under test,
  and code. Everything else is disposable — remove promptly. Obsolete-format
  `.graph` files (pre-version-bump) are always disposable.
- The batch tool (`roughroute batch`, M6/D17) enforces this **per region**:
  headroom check → download → build → verify → delete the `.pbf`, never
  accumulating sources; it aborts before any download that could fill the
  disk. Use it (not ad-hoc loops) for multi-region builds.
- The cargo target dir lives at `~/.cache/roughroute-target` (see
  `.cargo/config.toml`; ~3 GB when warm). It's a build cache, not scratch —
  keep it, but it's the first thing to `cargo clean` if space runs out.

## Key invariants (details in docs/DECISIONS.md)
- Coordinates are `[lat, lon]` everywhere; `[lon, lat]` only inside the GeoJSON
  serializer (D9).
- Determinism: same input → identical output. Byte-stable `.graph` builds
  (node index = ascending OSM id rank, sorted adjacency), A\* tie-break by node
  index (D3). Golden tests enforce this — don't introduce hash-order or
  timestamp nondeterminism.
- `length_dm` is clamped to ≥ 1 (D5); response `meters` comes from the
  polyline, not from `length_dm`.
- Fallback straight line only for "snapped OK but no path"; snap failure is
  always `SnapTooFar` (D10).
- No `unwrap`/`panic!` in library code; errors via `GraphError`/`RouteError`.
- Format is **v3** (M4 geometry block + M7 delta-compression): degree-2
  chains collapse into edges carrying intermediate geometry (D12/D13), and
  since v3 that geometry pool is zigzag-delta LEB128-varint compressed on
  disk (D19) — decoded once at load into the same in-memory representation
  v2 used, so `graph.rs`/`grid.rs`/`router.rs` are untouched by the bump.
  v1 *and* v2 `.graph` files are refused — rebuild. The route *contract* is
  unchanged (returned lines follow full road shape, F6; proven byte-for-byte
  identical to v2 on real data, D19).
- Snapping is **F10 edge snapping** (M5, D15/D16): waypoints project onto the
  nearest profile-usable road segment (collapsed geometry included) and
  routes start/end exactly at the projections; `SnapTooFar` means "no road
  this profile can use within `max_snap_meters`". No format change — the
  segment index is built at load.
- `roughroute batch` (M6) is **incremental** (D17 addendum): a region is
  rebuilt only if new, its file is missing, its hash mismatches, or the
  format version changed — never on every run. `--force` overrides.
- The batch tool's disk gates (headroom + `.pbf` size ceiling) have **no RAM
  equivalent** — a large region can still be OOM-killed on a
  memory-constrained machine even after the disk checks pass (hit this
  building Austria on a 5.8 GB-RAM dev VM; D18). Known gap, not fixed.
- Still out of scope: baked spatial index, duration estimate (F11), a RAM
  gate for `batch` — candidates for later milestones.
