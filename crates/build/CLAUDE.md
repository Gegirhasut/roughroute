# crates/build — agent context

**Single responsibility:** the ahead-of-time preprocessor — turn a `.osm.pbf`
extract into a `core` `Graph` / `.graph` bytes. Runs on dev machines and CI
only; **never** compiled into the WASM or native phone runtime (spec §5.1).

## Hard constraints
- This is the **only** crate allowed to depend on `osmpbf` (and, for the
  optional `--pbf-url` path, an HTTP client — build-time download only,
  spec §3).
- Output must be byte-stable: node index = ascending OSM node id rank,
  adjacency sorted `(target, length_dm, access)`, no hash-order/timestamps
  (DECISIONS D3). Golden tests compare raw `.graph` bytes.
- Junction correctness depends on OSM node-id dedup across ways — one global
  `osm_node_id → node index` map (DECISIONS D1). Never slice ways into
  per-way node copies.

## Structure (DECISIONS D8 — keep the layering; D23 — keep it compact)
- **Pure layer** (the bulk): in-memory road network → `core::Graph`. There
  is exactly **one** copy of the algorithm — `build_graph_compact_with_options`
  over a `CompactNetwork` (flat way-ref array + sorted parallel id/coord
  arrays quantized to fixed-point at read; the sorted merged edge list is
  reused in place as CSR adjacency; big transients freed stage by stage).
  The `build_graph(&[RawWay], &BTreeMap)` entry points are thin adapters
  over it (tests/synthetic fixtures use these). D23 cut peak build RSS
  3.5–4.4× with **byte-identical output**, proven by sha256 on all 10
  then-buildable regions — any change here must preserve that: identical
  values in identical order, only representation may differ. Two stages:
  1. v1 pipeline: profile tag table (DECISIONS D7), drop mask-0 ways, skip
     repeated refs, drop segments with missing nodes (clipped extracts),
     dedup `(source,target)` edges (OR masks, min length), clamp
     `length_dm ≥ 1` (D5), emit both edge directions.
  2. **M4 degree-2 collapse** (DECISIONS D13): interior = exactly 2 edges to
     distinct neighbors with equal access; maximal chains → single edges
     with summed `length_dm` + geometry in the shared pool (D12). Parallel
     chains survive; P-loops are dropped (counted); pure cycles keep their
     two lowest-index members. `build_graph_with_options(…, false)` skips
     the collapse — the regression tests compare shapes against it.
- **PBF front-end** (thin): two passes with `osmpbf` — ways first (collect
  accepted ways + needed node ids), then nodes (coordinates for that set) —
  producing a `CompactNetwork` directly (no `BTreeMap`, no per-way `Vec`;
  D23).

All tests except one `#[ignore]`d real-fixture test use the pure layer with
synthetic networks — no `.pbf` needed.
