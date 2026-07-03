# roughroute — Design Decisions

Decisions the spec (v1) leaves open, resolved here so implementation and review
have one place to check. Each entry: what we do, and why. Amend, don't rewrite —
if a decision changes, note the change and the reason.

---

## D1. Graph topology: OSM node-id dedup across ways

**Problem.** A junction is where two ways *share an OSM node id*. If `build`
sliced each way into nodes independently (fresh node per way occurrence), every
junction would be duplicated, the graph would decompose into per-way islands,
and nearly every query would hit the fallback straight line.

**Decision.** `build` keeps a single global map `osm_node_id → graph node index`
covering all accepted ways:

1. Filter ways by `highway` tag; compute the access mask per way (D7).
2. Collect the set of all node ids referenced by accepted ways.
3. Resolve coordinates for that set from the PBF node pool.
4. Every referenced node id becomes exactly **one** graph node (spec §7.4:
   every way node is a graph node in v1 — including non-junction shape nodes).
5. For each way, emit an edge per consecutive ref pair `(a, b)` — **both**
   directions (`a→b` and `b→a`), since the format stores a directed graph and
   v1 ignores `oneway`.

Because shared ids map to the same index, ways connect at junctions for free.

Edge cases:
- Consecutive identical refs in a way (OSM data glitch): skip, no edge.
- Refs whose node is missing from the extract (clipped geometry): drop the
  affected segment(s), keep the rest of the way. Warn in build output.
- The same node pair appearing in several ways (or twice in one): dedup to one
  directed edge per `(source, target)`; access masks are OR-ed, length keeps
  the minimum (deterministic and admissible-safe).

## D2. Load strategy: parse-on-load, format stays zero-copy-ready

**Decision.** v1 `Graph::from_bytes` parses the byte slice into owned `Vec`s
(`nodes: Vec<i32>`, `offsets: Vec<u32>`, `edges: Vec<Edge>`). We do **not**
claim or implement mmap/zero-copy in v1.

**Why.** Target regions are a few MB (spec §9); the copy is microseconds. WASM
copies the `ArrayBuffer` into linear memory anyway. Parsing also gives one
natural place to validate everything (D6), after which the rest of `core` can
be panic-free by construction. Endianness becomes a non-issue (file is LE;
we decode explicitly).

**Keeping v2 possible.** The file layout guarantees alignment so a future
zero-copy loader can cast sections in place:
- Header is exactly 32 bytes; every subsequent section starts at an offset
  that is a multiple of 4 from the start of the file (nodes: `i32×2n`,
  offsets: `u32×(n+1)`, edges: 12-byte records of `u32`/`u8` fields —
  all multiples of 4).
- `Edge` is specified as exactly 12 bytes with 4-byte alignment; the
  `_pad: [u8;3]` field is **always written as zeros** and must be ignored on
  read (so byte-level golden tests stay stable).
- A v2 zero-copy loader would additionally require the *caller's buffer* to be
  4-byte aligned; v1 accepts any `&[u8]`.

## D3. Determinism (spec F9) — build side and route side

**Build side (byte-stable `.graph`).**
- Graph node index = rank of the OSM node id in **ascending id order**. OSM ids
  are unique, so indexing is reproducible regardless of PBF block/thread order.
- Within each node's CSR adjacency run, edges are sorted by
  `(target, length_dm, access)`.
- No timestamps, no map/hash iteration order anywhere in the output path
  (collect → sort, or use `BTreeMap`).
- Result: same `.pbf` + same builder version **on the same platform/toolchain**
  → byte-identical `.graph` (golden test). The qualifier matters: edge
  `length_dm` is `round(haversine_m × 10)`, and haversine flows through libm
  `sin`/`cos`/`asin`, which are not correctly-rounded and can differ in the
  last ULP across OS/libc/architecture. A coordinate pair landing within an
  ULP of a `.5` rounding boundary can therefore round differently on another
  platform, yielding different `.graph` bytes (and a different `index.json`
  sha256). Reproducibility is **same-platform**; a fixed software trig
  implementation would be the fix if cross-platform byte-identity ever
  mattered (it currently does not — graphs are rebuilt, not diffed across
  machines). See the "Known limitations" note at the end of this file.

**Route side (identical output).**
- A\* priority = `(f, node_index)`: ties in `f` break by **smaller node index**.
  With deterministic adjacency order (above) the expansion order, and therefore
  the chosen path among equal-cost alternatives, is fully reproducible.
- Costs are integer decimeters (`u32` per edge, `u64` accumulator) — no
  float-accumulation nondeterminism in the search itself. Floats appear only in
  the heuristic (pure function of coordinates) and the final `meters` sum,
  both order-fixed.

## D4. Uniform grid for snapping: sizing heuristic

**Decision.** Grid is built at load time (spec §7.5) over the header bbox:
- Target **≈1 node per cell on average**: `total_cells ≈ clamp(node_count, 1, 2^20)`.
- Split into `cols × rows` proportional to the bbox's *metric* extents
  (longitude span scaled by `cos(mid_lat)`), so cells are roughly square on the
  ground, not in degrees.
- Storage is CSR-style (cell offsets + node indices sorted by cell, ties by
  node index), i.e. 4 bytes per cell + 4 bytes per node.

**Why this survives a sea-heavy bbox** (e.g. Cyprus extract whose bbox is
mostly water): empty cells cost only 4 bytes each in the offsets array, and the
`2^20` cap bounds that at ~4 MB worst case. If 80% of cells are empty, occupied
cells average ~5 nodes — still fine for nearest-node scans. We deliberately do
*not* target nodes-per-occupied-cell, which can't be known before building the
grid.

**Query.** Expanding ring search from the query point's cell: scan ring 0, then
ring 1, … keeping the best haversine distance; stop when the *minimum possible*
distance of the next ring exceeds the current best (correctness: the nearest
node may sit in a neighboring cell across a cell border) or exceeds
`max_snap_meters` (→ `SnapTooFar { index, meters }` with the best distance
found, or the ring bound if nothing was found at all).

## D5. `length_dm` rounding

**Decision.** `length_dm = max(1, round(haversine_meters × 10))` for every
emitted edge. Consecutive identical node refs never become edges at all (D1),
so the clamp applies to distinct-but-near-coincident coordinates
(< 5 cm apart, which 1e7 fixed-point can produce).

**Why.** A zero-cost edge lets A\* traverse it "for free": zero-length cycles
would make equal-cost path sets larger and tie-breaking load-bearing, and any
future cost tweak dividing by length would UB-adjacent. 1 dm of error on a
sub-decimeter edge is far below GPS noise. Note `meters` in the response is
summed from the polyline coordinates (spec §8.5), not from `length_dm`, so the
clamp never distorts reported length.

## D6. Validation at load (`GraphError`)

`from_bytes` fully validates before constructing `Graph`, so routing code never
bounds-checks defensively:
- magic ≠ `RRG1` → `BadMagic`; version ≠ 1 → `UnsupportedVersion(v)`;
- any section shorter than the header promises → `Truncated`;
- structural checks → `Malformed(reason)` (extra variant beyond the spec's
  list, allowed by its `/* ... */`): `offsets[0] != 0`, offsets not
  non-decreasing, `offsets[n] != edge_count`, `edge.target >= node_count`,
  node coordinates outside the header bbox or outside valid lat/lon range,
  unknown `flags` bits set (v1 defines none).

## D7. Profile tag lists (coarse for M0, reviewed in M3)

Access mask: `bit0 = car`, `bit1 = foot` (spec §7.3). Per `highway` value:

| highway value | car | foot |
|---|---|---|
| motorway, motorway_link | ✓ | ✗ |
| trunk, trunk_link, primary, primary_link, secondary, secondary_link, tertiary, tertiary_link, unclassified, residential, living_street, service | ✓ | ✓ |
| track | ✓ | ✓ |
| footway, path, pedestrian, steps, bridleway | ✗ | ✓ |
| cycleway | ✗ | ✓ |
| everything else (construction, proposed, raceway, bus_guideway, …) | ✗ | ✗ |

Ways whose mask is 0 are dropped entirely. This table is deliberately
permissive (`track`, `service` for car) per the spec's "roughly correct, not
legal" stance; M3 revisits it (spec §14.2).

## D8. `build` crate is layered for testability

`build` = (a) a pure graph-construction layer taking an in-memory road network
(list of ways: node-id sequences + highway class; map of node id → coordinate)
and returning a `core` Graph, plus (b) a thin `osmpbf` front-end that produces
that in-memory form via two passes over the file (ways first to learn the
needed id set, then nodes). Unit/integration/property/golden tests use layer
(a) with synthetic fixtures — no `.pbf` file or network needed; a real-PBF
integration test is `#[ignore]`d and runs only when `testdata/cyprus.osm.pbf`
exists.

## D9. Coordinate order

`[lat, lon]` **everywhere**: JSON contract (spec §6.1 as written — spec §14.1
offers flipping to match the tool's UI, we keep the spec default until the tool
says otherwise), Rust API (`[f64; 2]` = `[lat, lon]`), internal storage
(fixed-point pairs stored lat-then-lon), CLI `--via lat,lon`, GPX
(`lat=`/`lon=` attributes are named, no ambiguity). The **single** exception:
GeoJSON export emits `[lon, lat]` per RFC 7946, flipped at the last moment in
the GeoJSON serializer with a loud comment.

## D10. Error/fallback boundary

Fallback (straight segment + `fallback: true`) covers exactly one situation:
**both endpoints of a segment snapped successfully but A\* found no path**
(disconnected components, or no profile-allowed edges — spec §10). Snap
failures are always errors (`SnapTooFar`) regardless of `allow_fallback`:
inventing a line to a point that isn't near any road is not "rough routing",
it's fiction. `waypoints.len() < 2` → `TooFewWaypoints`. Coinciding consecutive
waypoints produce a zero-length segment that dedups away and cannot flip
`fallback`.

## D11. Workspace/toolchain hygiene

- Workspace root = `app/`, members `crates/{core,build,cli,wasm,ffi}` (spec §5).
- `core`: no default features pulling I/O; dependencies limited to pure-Rust,
  WASM-friendly crates. No `osmpbf`, no filesystem, no network — enforced by
  simply not having the dependencies, and re-checked when M1's wasm build runs.
- `wasm` and `ffi` crates are scaffolding-only until their milestones;
  they are excluded from `default-members` so M0 builds don't need their
  toolchains.
- No `unwrap`/`expect`/`panic!` in library code paths; `clippy::unwrap_used`
  denied in `core`, `build` lib layer.

## D12. Format v2 (M4): geometry block + collapsed edges

Format version bumps to **2**; v1 files are refused with
`UnsupportedVersion(1)` (spec §13 — no silent misreads). Layout, little-endian,
every section still starting 4-byte-aligned (keeps D2's zero-copy option):

| section | bytes | content |
|---|---|---|
| header | **40** | v1's 32 bytes (magic `RRG1`, `version=2`, flags, node_count, edge_count, bbox i32×4) + `geo_point_count: u32` + `_reserved: u32` (must be 0) |
| nodes | `node_count × 8` | unchanged: `[lat, lon]` fixed-point 1e7 |
| offsets | `(node_count+1) × 4` | unchanged CSR |
| edges | `edge_count × 16` | `target: u32`, `length_dm: u32`, `geo_off: u32`, `geo_len: u16`, `flags: u8` (bit0 = geometry reversed; other bits must be 0), `access: u8` |
| geometry | `geo_point_count × 8` | shared pool of intermediate `[lat, lon]` fixed-point points |

- An edge's intermediate shape is `geometry[geo_off .. geo_off+geo_len]`,
  **exclusive** of its endpoint nodes. Uncollapsed edges have
  `geo_len = 0` and, canonically, `geo_off = 0` and `flags = 0` (enforced at
  load so byte-golden tests stay meaningful).
- The two directed edges of one road segment **share** one pool range; the
  pool stores the canonical direction (lower node index → higher) and the
  opposite edge sets the `reversed` flag. This halves the pool vs storing
  both directions.
- `geo_len` is `u16`; the builder splits any chain with more than
  `60_000` intermediate points by keeping a split node as a real node
  (never hit by real data; guards the field width).
- The header bbox covers nodes **and** geometry points; loader validation
  extends D6 with: `geo_off + geo_len ≤ geo_point_count` (u64 math), geometry
  coordinates in valid lat/lon range and inside the bbox, unknown edge flag
  bits rejected, `_reserved = 0`.
- **No delta/varint compression of the pool yet** — deliberately skipped for
  M4 v1 simplicity; the pool is the next size lever if needed (most deltas
  would fit 2 bytes, ~4× pool shrink available).

**Measured on Cyprus** (cyprus-latest.osm.pbf, 2026-07-03):
nodes 1,796,541 → 267,459 (6.7×), directed edges 3,765,810 → 705,406 (5.3×),
geometry pool 1,516,923 points, 1,120 P-loop chains dropped. File size
63.7 MB (v1) → **25.4 MB (v2), 2.5× smaller**. Post-collapse the file is
dominated by the geometry pool (11.6 MB) and edges (10.8 MB) — which is why
delta-compressing the pool is the designated next lever if 2.5× isn't enough.

## D13. Degree-2 collapse rules (build)

Runs after the v1 pipeline (D1's dedup/merge) on the merged directed edges.

- A node is **interior** (collapsible) iff it has exactly 2 outgoing edges,
  to two *distinct* neighbors, with *equal access masks*. Everything else —
  junctions (degree ≠ 2), dead-ends, access-change points, nodes with
  parallel edges — is kept. Access equality matters: collapsing across an
  access change would misroute a profile.
- Maximal chains of interior nodes between two kept nodes become one
  undirected road segment: `length_dm` = **sum of the chain's edge
  `length_dm`s** (not re-measured end-to-end — preserves exact A\* costs, so
  collapsed and uncollapsed graphs choose identical shortest paths),
  access = the (uniform) chain access, geometry = the swallowed nodes'
  coordinates in canonical order.
- **Parallel segments survive.** Two distinct chains between the same pair of
  kept nodes (dual carriageways, loop roads) stay as two edges with their own
  geometry; deduplication applies only to the pre-collapse single edges (D1).
  A\* handles parallel edges by cost; ties are impossible to get wrong
  deterministically because adjacency sorts by `(target, length_dm, geo_off)`
  and `geo_off` is unique per chain-with-geometry.
- **P-loops** (a chain leaving and re-entering the *same* kept node) would
  collapse to self-loops, which A\* can never profitably traverse and which
  stopped being snap targets anyway — they are **dropped** and counted in
  build stats.
- **Pure interior cycles** (an isolated ring with no junction at all): the two
  lowest-index members are kept as artificial junctions, turning the ring
  into two parallel edges — ring routing keeps working with 2 real nodes.
- Kept nodes are renumbered by ascending old index (which was ascending OSM
  id), so D3 determinism carries over; chains sort canonically before pool
  offsets are assigned.

## D14. Snapping after collapse

> **Healed by M5 (F10 edge snapping, D15/D16).** The regression described
> below existed between M4 and M5 only; routing now snaps onto edge geometry
> and is strictly better than both M4 and v1 node snapping.
> `Graph::nearest_node` still exists (kept nodes only) for diagnostics.

The snap grid indexes **real nodes only** — junctions, dead-ends, and the
other kept cases above. Mid-chain points are no longer snap targets (in v1
every way-node was). Consequences: in cities (junction every 50–150 m) nothing
observable changes with the default `max_snap_meters = 200`; on long rural
roads a waypoint may now be farther than 200 m from the nearest *kept* node
and yield `SnapTooFar` where v1 would have snapped mid-road. This is the
documented cost of M4-without-F10; **edge snapping (F10, projecting onto the
collapsed edge's geometry) is the designated follow-up** that restores — and
improves on — v1 snapping. Callers can raise `max_snap_meters` meanwhile.

## D15. F10 segment index and projection query (M5)

- The load-time grid (D4) gains a second CSR: every shape segment of every
  **canonical** edge (source index < target index — the direction whose pool
  order the geometry is stored in, D12) is bucketed into each cell its
  bounding rectangle overlaps. A segment is therefore always present in the
  cell containing its closest point to any query, which keeps the
  expanding-ring lower-bound termination proof intact. Reversed twins are
  not indexed (same shape); a hit serves both directions.
- **No format change, no version bump** — the index is derived at load from
  the v2 geometry pool. Cyprus cost: ~1.9 M segment refs ≈ tens of MB linear
  memory and a subsecond build; a baked index (`flags` bit) remains the
  later optimization.
- Query: point-to-segment projection in a local equirectangular frame around
  the query (meter-accurate at snap scales), reported distance = haversine
  from the query to the projected point. Deterministic tie-break
  `(distance, edge_index, seg_index)`. Degenerate (zero-length) segments
  project to `t = 0`; NaN queries match nothing.
- `Graph::nearest_road` (public) exposes the any-profile query; the router
  uses the profile-filtered variant (D16).

## D16. F10 virtual-endpoint routing (M5)

- **Snapping is profile-aware:** only segments of edges the profile can use
  are candidates. The D15 tie-break `(distance, edge_index, seg_index)` is
  applied *after* this filtering and is stable: the winner is a strict
  lexicographic minimum over all filtered candidates, and ring termination
  (`lower_bound > best`, strictly) can never cut off an equidistant
  candidate — so grid scan order never leaks into the result.
  A foot query beside a motorway snaps to the nearest
  *walkable* road instead. Consequence (semantics change vs M4, deliberate):
  "no usable road within `max_snap_meters`" is now `SnapTooFar` — including
  the spec §10 "profile with no allowed edges at a point" row, which
  previously produced a fallback straight line from a node the profile
  couldn't leave. The new error is more honest and the router never plans
  travel over forbidden partials.
- A snapped waypoint is a **virtual point on its edge**: distance `d_src`
  from the edge's source along the shape, in integer dm, clamped into
  `[0, length_dm]` so the two partials always sum to `length_dm` exactly.
  Reversed-twin bookkeeping never arises: snaps live on canonical edges and
  partials are expressed from the canonical source.
- A\* seeds the frontier with **both** endpoints of the start edge at their
  partial costs and may finish at either endpoint of the goal edge plus that
  endpoint's partial; the search stops when the frontier minimum can no
  longer beat the best finish (admissible heuristic: haversine to the goal
  projection). First-settled endpoint wins cost ties (deterministic).
- **Same-edge legs:** staying on the edge costs `|Δd_src|`, but a network
  path can win on hairpin-shaped edges (out via one endpoint, back via the
  other); both are evaluated and ties prefer the simpler along-edge leg.
  Identical projections collapse to a point leg.
- **Line splicing (F6):** the returned line starts/ends exactly at the
  projected points; partial shape vertices of the start/goal edges are
  spliced in travel order (reversed ranges when traveling against canonical
  order). Interior legs are unchanged M4 geometry expansion — kept-node to
  kept-node routes produce byte-identical middles. One final consecutive-
  duplicate dedup absorbs `t = 0/1` boundary coincidences.
- Fallback (disconnected components) bridges the two *projections* now, and
  `NoPath` keeps its leg-index semantics.

## D17. Region batch tooling: manifest, index.json, publishing (M6)

**Implementation choice: a Rust subcommand (`roughroute batch`), not a shell
script.** It reuses `roughroute-build` in-process (no argv/re-exec fragility),
verifies through the *same* `Graph::from_bytes` the runtime uses, and gets
clap/serde for free. It lives in the CLI crate — dev/CI only, never in the
runtime (spec §2.1: delivery is the host app's job; the router never
downloads at runtime).

**Manifest** — committed, human-editable `regions.toml` at the workspace root:

```toml
[[region]]
id = "cyprus"        # kebab-case; becomes the artifact name <id>.graph
name = "Cyprus"      # display name for the host app's UI
pbf_url = "https://download.geofabrik.de/europe/cyprus-latest.osm.pbf"
```

Ids must match `[a-z0-9-]+` and be unique; the file is the single source of
truth for what gets built. Seeded with small regions (Cyprus, Malta,
Andorra) — never the planet.

**Index** — `index.json` written next to the built graphs; the discovery
entry point route-spoofer fetches first:

```json
{
  "schema_version": 1,
  "format_version": 2,
  "attribution": "© OpenStreetMap contributors (ODbL)",
  "regions": [
    {
      "id": "cyprus",
      "name": "Cyprus",
      "file": "cyprus.graph",
      "url": "https://github.com/OWNER/REPO/releases/download/TAG/cyprus.graph",
      "bytes": 26631432,
      "sha256": "…hex…",
      "nodes": 267459,
      "edges": 705406,
      "bbox": [34.5638, 32.27, 35.695, 34.5875],
      "source_pbf_url": "https://download.geofabrik.de/europe/cyprus-latest.osm.pbf"
    }
  ]
}
```

- `schema_version` guards the index shape itself; `format_version` is the
  `.graph` format (the header also embeds it — the app must check on load
  and refuse mismatches, spec §13).
- `url` is filled from `--release-url-base` when given, else falls back to
  the bare file name (relative to wherever the index is hosted) — so moving
  hosting (e.g. GitHub Releases → Cloudflare R2 for free egress) only means
  regenerating/rewriting URLs, no router or app-logic change.
- **Hash: SHA-256** (`sha2` crate) of the exact `.graph` bytes — standard,
  collision-resistant, cheap at these sizes; the app verifies after
  download, which also guards truncated fetches. `bbox` lets the app pick a
  region by coordinate without downloading anything.
- No timestamps in the index: builds stay reproducible byte-for-byte from
  the same inputs (F9 discipline); Geofabrik's `-latest` URL is the moving
  part, recorded as `source_pbf_url`.

**Disk discipline** (the CLAUDE.md rule, enforced in code): per region,
strictly download → build → verify → delete the `.pbf` (deleted even when
build/verify fails); never two `.pbf`s at once. Before each download the
tool checks free space (`df -P -B1`, parsed) against
`2 × Content-Length + 1 GiB` (HEAD request; 5 GiB assumed when the server
sends no length) and **aborts the whole run** with a clear message when
short. Verification = re-read the written file, `Graph::from_bytes`, and a
trivial route between two node coordinates of the loaded graph.

**Hard size ceiling (added 2026-07-03, `HARD_MAX_PBF_BYTES = 800_000_000`):**
checked *before* the headroom math, on the same HEAD-probed
`Content-Length` — a technically-fitting-but-huge extract (a continent, a
country the size of the US) would otherwise pass the headroom check on a
big-enough disk yet still derail an unattended run's wall-clock budget. A
probe that returns no length also aborts (refuse to fly blind past a safety
limit) rather than falling back to the headroom path's harsher-but-permissive
5 GiB assumption. Note the probe must follow redirects: Geofabrik's
`-latest.osm.pbf` URLs 302 to a dated file, and `ureq`'s default agent
follows redirects for HEAD the same as GET — a redirect-naive HEAD would
read the tiny HTML redirect body's length instead. Raising the constant is a
deliberate code change, not a config flag, so adding a legitimately larger
region is a visible diff.

**Publishing**: manual/CI `gh release create` with the `.graph` files +
`index.json` as assets (each far under the 2 GiB asset limit). No
credentials in the repo; the tool prints the exact command. ODbL
attribution ships in the index, the release notes, and the README.

**Incremental (added 2026-07-03).** Rebuilding every region on every run
re-downloads and re-builds sources that never changed — wasteful and, for
larger regions, slow. Before touching a region, `batch` loads the existing
`index.json` (if any) and checks that region's cached entry against the
file actually on disk:

- `IndexRegion` gains a per-region `format_version: u16` (`#[serde(default)]`
  so pre-incremental index files — which lack the field — parse as `0`,
  which never matches a real version and so always looks stale — a safe
  default that forces exactly one rebuild pass to backfill the field).
- A cached entry is "fresh" (region skipped — no HEAD probe, no download, no
  rebuild) iff: `format_version` matches the current builder
  (`roughroute_core::format::VERSION`), the file exists, its size matches
  `bytes`, and its SHA-256 matches `sha256`. Any mismatch (new region,
  missing file, edited/corrupted file, or a format version bump) triggers a
  full rebuild of just that region.
- The top-level `Index.format_version` field is now purely informational
  (the version *this run's builder* targets); it is not used for staleness
  — only the per-region field is, since regions can legitimately sit at
  mixed actual versions during a migration window (some rebuilt at a new
  version, some not yet touched).
- Skipped regions still get their `url` recomputed from the current
  `--release-url-base` (cheap, no I/O) so re-running `batch` after changing
  the release tag doesn't leave stale download URLs for untouched regions.
- `--force` bypasses the freshness check for every region unconditionally
  (rebuild regardless — for a source or format change the tool can't detect
  on its own, e.g. Geofabrik republishing the same-named extract with new
  data).
- A version bump (the D19 case below) naturally forces a **one-time full
  rebuild** of every region on the next `batch` run, since every cached
  entry's `format_version` no longer matches — no special-case code needed;
  the general staleness rule already covers it.

## D18. Hard `.pbf` ceiling: raised, then reverted, after an Austria OOM (2026-07-03)

`HARD_MAX_PBF_BYTES` was raised from `800_000_000` to `1_200_000_000`
(1.2 GB) to admit Austria (`.pbf` probed at 803.1 MB — just over the old
800 MB ceiling) as a second, larger scaling data point beyond Slovenia. This
was a deliberate, documented raise, not a loosening of the safety principle:
the constant stays a hard-coded ceiling requiring a visible code change to
move (D17), and the disk-headroom check (`2 × content_length + 1 GiB`) is
independent of it — confirmed before building that `2 × 803.1 MB + 1 GiB
≈ 2.6 GiB` fit comfortably in the ~48 GiB free at the time.

**The disk gate worked exactly as designed and was not the problem.** The
Austria build was **OOM-killed by the kernel** partway through parsing
(`dmesg`: `Out of memory: Killed process … (roughroute) … anon-rss:4766620kB`)
— this dev VM has only **5.8 GB total RAM**, a resource this batch tool has
no gate for at all (its checks are entirely disk-based: `df` headroom + the
`.pbf` size ceiling). Slovenia (309.6 MB pbf) had peaked at ~1 GB RSS to
*build*; Austria (2.6× the pbf size) was still climbing past 4.8 GB and
growing when killed — a much steeper-than-linear scaling, plausibly a
denser OSM way network per byte for Austria than Slovenia, though the exact
cause wasn't isolated.

**Decision: dropped Austria rather than chase the memory ceiling.**
Slovenia alone is a solid, real-world mid-size data point for the v3
delta-compression measurement (D19); pursuing a bigger one on a 5.8 GB-RAM
dev machine wasn't worth the risk of repeat OOM kills. `HARD_MAX_PBF_BYTES`
**reverted to `800_000_000`** — its only justification for being higher no
longer applies. `regions.toml` keeps Slovenia as the largest built region,
with a comment noting the dropped Austria attempt.

**Known gap, not fixed here:** `roughroute batch` has no RAM check or
ceiling of any kind. Building a region on a memory-constrained machine can
still be OOM-killed after the disk gates pass cleanly. A future fix would
either probe available memory (`free`/`/proc/meminfo`, Linux/macOS-specific
like the existing `df` parsing) and gate on it similarly, or reduce the
`build` crate's peak memory (e.g. streaming the ways/coords accumulation
instead of holding a full `Vec<RawWay>` + `BTreeMap` in memory at once).
Neither attempted in this session.

## D19. Format v3: geometry pool delta-compression (PROPOSED, then implemented)

The v2 geometry pool (`docs/DECISIONS.md` D12) stores every intermediate
point as two raw `i32` (8 bytes). It is the dominant chunk of a v2 file
(Cyprus: geometry ≈ 11.6 MB of 25.4 MB, i.e. 1,516,923 points × 8 bytes;
Slovenia/Austria proportionally larger — see the measured table below). D12
explicitly deferred
compressing it: "most deltas would fit 2 bytes, ~4× pool shrink available."
This decision cashes that in.

**Design chosen: whole-array sequential zigzag-delta varints, no per-chain
bookkeeping.**

- The pool is still logically `Vec<[i32; 2]>` of `geo_point_count` points,
  addressed by `geo_off`/`geo_len` exactly as in v2 — **nothing about how
  edges reference the pool changes**.
- On disk, point 0's `lat`/`lon` are each zigzag+LEB128-varint-encoded
  directly (equivalent to "delta from the origin"); point *i* (`i > 0`) is
  encoded as the zigzag-varint of `(lat_i − lat_{i-1}, lon_i − lon_{i-1})` —
  the delta from the immediately *preceding point in array order*,
  regardless of which edge/chain either point belongs to.
- **Deliberately not** per-chain-anchored (e.g. delta from each chain's edge
  source node, resetting at every `geo_off` boundary). That design was
  considered and rejected: it requires decode to know chain boundaries,
  which requires sorting canonical edges by `geo_off` and enforcing a new
  "chains exactly tile `[0, geo_point_count)` with no gaps" invariant on top
  of the existing bounds check — real, but avoidable complexity. Since
  `network.rs` already sorts chains by endpoint index before laying out the
  pool (D13), and ascending-OSM-id node indexing (D3) correlates with
  spatial locality in practice, consecutive pool points are already usually
  close together; whole-array delta gets most of the same compression with
  a function signature of `Vec<[i32;2]> -> Vec<u8>` and back, independent of
  edges/offsets/nodes entirely. The cost is one larger-than-typical delta at
  each chain boundary — bounded by the same bbox as everything else, at
  most 2×5 varint bytes — a rounding error against the aggregate saving.
- Zigzag: `((v << 1) ^ (v >> 31)) as u32`, so small negative and positive
  deltas both encode as small varints (plain two's-complement would make
  every negative delta need the full 5 bytes). LEB128: 7 payload bits/byte,
  high bit = continuation; a delta fitting in 6 bits (±31, i.e. ≤ 3.1 m at
  1e7 fixed-point) costs 1 byte, up to ±8191 (≈ 819 m) costs 2 bytes — the
  common case for consecutive OSM way-shape points.
- Hand-rolled, dependency-free (no crate added): `core` has zero
  dependencies by design (D11) and must stay `wasm32-unknown-unknown`-clean;
  ~20 lines of zigzag+LEB128 encode/decode isn't worth a dependency.

**Header change: reuse the v2 `_reserved: u32` field.** It was always `0` in
v2 (reserved, unused); v3 repurposes it as `geo_bytes: u32` — the exact byte
length of the (now variable-size) geometry section, needed since it can no
longer be inferred as `geo_point_count × 8`. Header stays **40 bytes**, no
growth. `geo_point_count` keeps its v2 meaning (decoded point count) and is
used to validate the decode consumed exactly that many points from exactly
`geo_bytes` bytes with nothing left over.

| section  | bytes | v3 change from v2 |
|---|---|---|
| header   | 40 (unchanged) | `_reserved` renamed `geo_bytes: u32` (was always 0) |
| nodes    | `node_count × 8` | unchanged |
| offsets  | `(node_count+1) × 4` | unchanged |
| edges    | `edge_count × 16` | **unchanged** — `geo_off`/`geo_len` still index the logical decoded point stream |
| geometry | `geo_bytes` (variable) | zigzag-delta LEB128 varints, was `geo_point_count × 8` fixed |

**Where decode happens: on load, not on traversal.** `Graph::from_bytes`
decodes the varint section straight into the same absolute `Vec<[i32; 2]>`
the v2 loader produced. **Nothing in `graph.rs`, `grid.rs`, or `router.rs`
changes** — `edge_geometry`, the segment index, A\*, and line splicing all
operate on the identical in-memory representation as v2. This follows D2's
existing precedent (parse-on-load into owned `Vec`s) and makes the
"routes must be identical to v2" invariant close to free: after loading, a
v3-loaded `Graph` is bit-for-bit the same struct a v2 loader would have
produced from the same source data, so every downstream algorithm is
provably unaffected rather than merely re-tested. Total decode cost is one
linear pass over the geometry section, same order as v2's direct read.

**Version bump:** `VERSION` 2 → **3**; `parse()`'s existing
`version != VERSION` check means v1 *and* v2 files are both refused with
`UnsupportedVersion` (spec §13 discipline, unchanged from the D12 bump) —
no silent misreads of the old fixed-width layout as delta-varints.

**`Graph::from_parts` and the `build` crate are untouched.** Delta encoding
is purely a `format.rs` (`serialize`/`parse`) concern; the builder keeps
producing an absolute `Vec<[i32; 2]>` exactly as before and never needs to
know the on-disk representation changed.

**Measured (2026-07-03, all four then-current regions rebuilt v2 → v3):**

| region | v2 `.graph` | v3 `.graph` | factor |
|---|---|---|---|
| Cyprus | 26.6 MB | 20.8 MB | 1.28× |
| Malta | 3.9 MB | 3.4 MB | 1.15× |
| Andorra | 1.7 MB | 1.1 MB | 1.50× |
| Slovenia | 78.2 MB | 56.4 MB | 1.39× |

Node/edge counts identical in every case (structure untouched, confirming
only the on-disk geometry encoding changed). Smaller than the "~4×" upper
bound floated in D12 — real OSM way-shape deltas run larger than the
best-case 1-byte estimate, and every point still costs a minimum 2 bytes
even at zero delta (LEB128 has no zero-length encoding), so realistic
average delta size is closer to 2–4 bytes/coordinate than 1. Still a solid,
free-standing win, and the pool remains the next lever if a future
milestone needs more.

**Route-identity proof (the invariant this decision is on the hook for):**
Slovenia's route was captured through the CLI (`roughroute route`, real
data) for both profiles × both directions between two real cities
(Ljubljana ↔ Maribor, ~123 km driving / ~128 km walking) on the v2 graph,
then again on the v3 rebuild of the *same* source `.pbf`. All four JSON
outputs (`line` array and `meters`, to full float precision) are **byte-for-
byte identical** between v2 and v3 — not just "close," identical — exactly
as predicted by the "decode on load, nothing downstream changes" argument
above.

## Known limitations (deliberate v1 scope)

These are known and out of scope for v1 — documented so they're a choice, not
a surprise. Each notes the real fix if it ever becomes necessary.

- **Antimeridian-spanning regions** (Fiji, Chukotka, NZ + Chathams): the
  snapping projection is a local equirectangular frame that computes
  `plon − lon` without wrapping, so it is wrong across the ±180° seam. Rather
  than misroute silently, `build_graph` **rejects** any region whose longitude
  span exceeds 180° with `BuildError::AntimeridianSpanning`. Full support
  (wrapping the projection and the grid at the seam) is deferred.

- **Cross-platform byte-identical builds**: reproducibility is
  **same-platform/toolchain only** (D3). `length_dm` depends on libm trig,
  which isn't correctly-rounded, so a `.graph`'s bytes (and its `index.json`
  sha256) can differ across OS/arch for the same input. Fine as long as graphs
  are rebuilt per platform rather than diffed across machines; a fixed software
  trig path for `length_dm` is the fix if that ever changes.

- **Hostile-graph huge allocation via pathological segment coverage**: a
  crafted-but-valid `.graph` where a single edge's bounding rectangle covers
  the whole grid could make `Grid::build` allocate an enormous segment index.
  In practice this is gated by the host app sha256-verifying downloads
  (D17 / README), so a hostile graph never reaches a real device. A
  per-segment cell-coverage cap in `Grid::build` is the real fix and is
  deferred (it touches the correctness-critical snapping index, so it wants
  its own careful change rather than being bolted on here).

- **4 GiB+ geometry pool**: the on-disk `geo_bytes` length is a `u32`.
  `Graph::from_parts` now rejects any pool that would delta-encode past that
  with a clean `GraphError::Malformed` (D19), so the former silent truncation
  is gone. Unreachable at target scale (it needs a multi-GB in-memory graph);
  no further work planned.
