# Spec: `roughroute` — fast offline OSM mini-router for route-spoofer

> Working crate name: `roughroute` (from "rough routing"). The name is a placeholder — change it if you like.
> Document status: draft v1, ready to implement. Open questions are collected in section 14.

---

## 1. Goal and non-goals

### 1.1. In one sentence
A standalone open-source Rust library that, given an ordered set of points (`A→B→C`) and a profile (`car`/`foot`), returns a **polyline that follows OSM roads** — "roughly correct", with no guarantee that the maneuver is legal. The library is fully offline at runtime and embeds as **WASM** (inside the Capacitor webview), as a **native library** (Android NDK), and runs as a **CLI**.

### 1.2. Why route-spoofer needs it
Today route-spoofer plays back a route the user taps out by hand. `roughroute` provides a route source of the form "give me A→B → get a road-following line", which the tool plugs in as one of several swappable route providers (alongside GPX and an external OSRM/Valhalla). The router and the tool are **separate repositories**, connected only by a JSON contract (section 6.1).

### 1.3. Goals (must)
- Build a connected route over the OSM road network between ordered waypoints.
- Run fully **offline at runtime**: input is a pre-built binary graph, output is a polyline. The runtime never touches the network.
- The same algorithmic core compiles to **WASM**, to a **native .so / UniFFI binding**, and to a **CLI**.
- Fast: a route query over a city/island-sized region completes in single-digit milliseconds.
- `car` and `foot` profiles (differing by the set of allowed roads).
- On no path found, don't fail — return a fallback straight line with a flag (for the spoofer, "some route" matters more than "an honest refusal").

### 1.4. Non-goals (explicitly out of scope)
- ❌ We ignore `oneway`, `turn restrictions`, `access`, prohibitions, height/weight — maneuver legality is not guaranteed. This is a deliberate simplification.
- ❌ We don't compute ETA / travel time like a production router (speed is coarse, taken from the profile).
- ❌ We don't compete with or replace OSRM/Valhalla. For "honest" routing the user plugs those in as an external provider.
- ❌ No geocoding (address → coordinate). Input is coordinates only.
- ❌ No Contraction Hierarchies / MLD. Plain A\* is sufficient for the target scale.

### 1.5. Definition of Done (v1)
- The CLI builds a `.graph` from a `.osm.pbf` and returns a route in JSON/GPX/GeoJSON.
- The WASM module loads a `.graph` from an `ArrayBuffer` and returns a contract response.
- The Kotlin binding (UniFFI) is callable from route-spoofer's Android code.
- On the fixture (Cyprus) a route between two points is built, endpoints snap to roads, the line is connected, tests pass.
- README with OSM attribution (ODbL) and graph-build instructions.

---

## 2. Context and separation of responsibilities

```
┌─────────────────────────────────────────────────────────────┐
│  OFFLINE, AHEAD OF TIME (dev machine / CI)                   │
│                                                              │
│  Geofabrik .osm.pbf  ──►  roughroute build  ──►  region.graph│
│  (downloaded manually     (preprocessor,         (compact    │
│   or in CI)                 separate CLI)         binary)     │
└─────────────────────────────────────────────────────────────┘
                                   │
                     published as a release asset / on a CDN
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────┐
│  RUNTIME (phone, webview) — FULLY OFFLINE                    │
│                                                              │
│  route-spoofer  ──(1) obtains region.graph──►  on-disk cache │
│       │             (bundle or download+cache — tool decides)│
│       │                                                      │
│       └─(2) waypoints+profile──► roughroute (WASM/native) ──►│
│                                   line + meters ─────────────┤
│                                          │                   │
│                                          ▼                   │
│                              route-spoofer's existing        │
│                              playback pipeline (step along   │
│                              the line, emit mock coordinates)│
└─────────────────────────────────────────────────────────────┘
```

### 2.1. Key decision: "who downloads the files"
The fork you asked to settle. Decision:

- **The runtime router (`roughroute` as WASM/native) never touches the network.** Its input is the bytes of an already-built `.graph`. This keeps it clean, small, testable, and offline by design.
- **Downloading/parsing the `.osm.pbf` is a separate preprocessing step** (`roughroute build`), run on a dev machine or in CI — not on the phone. The `.pbf` is heavy, and parsing it on-device is a bad idea.
- **Delivering the finished `region.graph` to the device is the host app's job (route-spoofer), not the router's.** Two options, both valid; the tool picks:
  - **Bundle** — ship a default region's `region.graph` in the APK assets. Pro: works offline immediately. Con: APK size, single region.
  - **Download + cache** — the tool downloads the needed region's `region.graph` from a CDN/GitHub release by URL, stores it in its cache, then works offline. Pro: any region, small APK. Con: needs the network the first time.

Bottom line for you: **the router is offline and "dumb" — it eats a ready graph. The network (if needed at all) is the tool's concern during file delivery, and even then optional.** This removes coupling and gives maximum flexibility.

> Optionally, `build` can also be exposed as a library function (not just a CLI) if you ever want to build a graph inside some desktop GUI. But we never pull a `.pbf` parser into the phone runtime.

---

## 3. Data source and license

- **Geodata source:** regional OpenStreetMap extracts from Geofabrik (`https://download.geofabrik.de/`) in `.osm.pbf` format. For small regions the files are a few MB.
- **Data license:** ODbL. **Requirement:** any product that displays a route (including route-spoofer) must show the attribution `© OpenStreetMap contributors`. Put it in the router's README and in the tool's UI/about.
- **`roughroute` code license:** MIT (aligned with route-spoofer).
- The router does not automate `.pbf` downloading at runtime; in the `build` CLI it's fine to accept either a path to a local `.pbf` or a URL (in which case it downloads at build time, not in the phone runtime).

---

## 4. Functional requirements

| # | Requirement | Priority |
|---|---|---|
| F1 | Build a route between two waypoints over the road graph | must |
| F2 | Multi-point: ordered `A→B→C→…`, concatenation with junction dedup | must |
| F3 | Snap a waypoint to the nearest graph node | must |
| F4 | `car` and `foot` profiles (difference = set of allowed roads) | must |
| F5 | Output: polyline as a list of coordinates + total length in meters | must |
| F6 | The line follows the real road shape (not just "node-to-node"), i.e. includes intermediate way points | must |
| F7 | On no path, a fallback straight line between snapped nodes + `fallback: true` flag | must |
| F8 | CLI export to JSON / GeoJSON / GPX | must |
| F9 | Determinism: identical input → identical output | must |
| F10 | Snap to the nearest point on an edge (not just to a node) | nice-to-have |
| F11 | Coarse duration estimate from profile speed | nice-to-have |
| F12 | Multiple alternative routes | won't (v1) |

Explicitly not required: oneway, turn restrictions, access tags, vehicle restrictions (see 1.4).

---

## 5. Crate architecture

A Cargo workspace with a clean split of "core with no outside world" / "target-specific shells".

```
roughroute/
├── Cargo.toml                # workspace
├── crates/
│   ├── core/                 # PURE algorithmic core, no_std-friendly where possible
│   │   ├── graph.rs          #   graph structure (CSR), loading .graph from &[u8]
│   │   ├── router.rs         #   A*, snapping, multi-point, fallback
│   │   ├── format.rs         #   parse/serialize the binary .graph format
│   │   ├── profile.rs        #   car/foot profile masks
│   │   └── geo.rs            #   haversine, bearing, geo utilities
│   ├── build/                # PREPROCESSOR: .osm.pbf -> .graph (depends on osmpbf)
│   ├── cli/                  # bin: `roughroute` (build + route), desktop/CI
│   ├── wasm/                 # wasm-bindgen wrapper (feature = "wasm")
│   └── ffi/                  # UniFFI wrapper for Kotlin (feature = "native")
└── testdata/
    └── cyprus.osm.pbf        # fixture (or downloaded by a test)
```

### 5.1. Feature flags
- `core` — default, no outside world, no heavy dependencies.
- `build` — pulls an `osmpbf`/`osmium`-style parser; **CLI/CI only**, never reaches the WASM/native runtime.
- `wasm` — `wasm-bindgen`, `serde-wasm-bindgen`.
- `native` — `uniffi`.

Rule: `core` depends on neither `build`, nor the network, nor the filesystem (it takes `&[u8]`). This guarantees offline operation and testability.

---

## 6. Contracts and API

### 6.1. Neutral JSON contract (shared with route-spoofer)
This is the boundary between the tool and any route provider. The same contract serves `roughroute`, the GPX provider, and an external OSRM adapter.

**Request:**
```json
{
  "waypoints": [[34.7071, 33.0226], [34.6841, 33.0379]],
  "profile": "car"
}
```

**Response:**
```json
{
  "line": [[34.7071, 33.0226], [34.7069, 33.0231], "...", [34.6841, 33.0379]],
  "meters": 2143.7,
  "fallback": false
}
```

- **Coordinate order: `[lat, lon]`** everywhere in the contract (see open question 14.1 — if the tool's UI is already `[lon,lat]`, fix it the other way, but consistently everywhere).
- `line` — a dense polyline following road geometry (F6).
- `meters` — total length by haversine.
- `fallback` — `true` if at least one segment had to be filled with a straight line (F7).
- Errors don't go through the contract but through the target's error channel (exception in WASM, `Result` in Rust, throw in Kotlin). Empty `waypoints` (<2 points) → validation error.

> ⚠️ GeoJSON uses `[lon, lat]`. On GeoJSON export the order flips — this is the only place where `[lon,lat]` is allowed, and it must be explicitly commented in the code.

### 6.2. Public Rust API (`core`)
```rust
pub struct Graph { /* CSR, nodes, profile masks */ }

impl Graph {
    /// Load from an already-built binary. Reads no files, touches no network.
    pub fn from_bytes(bytes: &[u8]) -> Result<Graph, GraphError>;
    pub fn bbox(&self) -> BBox;
    pub fn node_count(&self) -> u32;
}

pub struct Router<'g> { graph: &'g Graph, opts: RouteOptions }

pub struct RouteOptions {
    pub profile: Profile,        // Car | Foot
    pub allow_fallback: bool,    // default true
    pub max_snap_meters: f64,    // snap cutoff, e.g. default 200.0
}

pub struct RouteResult {
    pub line: Vec<[f64; 2]>,     // [lat, lon]
    pub meters: f64,
    pub fallback: bool,
}

impl<'g> Router<'g> {
    pub fn new(graph: &'g Graph, opts: RouteOptions) -> Self;
    /// Main call: >=2 waypoints [lat,lon].
    pub fn route(&self, waypoints: &[[f64; 2]]) -> Result<RouteResult, RouteError>;
}

pub enum Profile { Car, Foot }

pub enum RouteError { TooFewWaypoints, SnapTooFar { index: usize, meters: f64 }, /* ... */ }
pub enum GraphError { BadMagic, UnsupportedVersion(u16), Truncated, /* ... */ }
```

### 6.3. WASM API (`crates/wasm`, feature `wasm`)
```rust
#[wasm_bindgen]
pub struct WasmRouter { /* holds Graph in linear memory */ }

#[wasm_bindgen]
impl WasmRouter {
    /// graph_bytes — Uint8Array from an ArrayBuffer passed in from JS.
    #[wasm_bindgen(constructor)]
    pub fn new(graph_bytes: &[u8]) -> Result<WasmRouter, JsError>;

    /// req — JS object { waypoints, profile }; returns { line, meters, fallback }.
    #[wasm_bindgen]
    pub fn route(&self, req: JsValue) -> Result<JsValue, JsError>;
}
```
Usage inside the tool's webview (TS):
```ts
const bytes = await loadRegionGraph(); // from bundle or cache (the tool's job)
const router = new WasmRouter(new Uint8Array(bytes));
const res = router.route({ waypoints, profile: "car" });
// res.line -> into route-spoofer's existing playback
```

### 6.4. Native API (`crates/ffi`, feature `native`, UniFFI → Kotlin)
UDL sketch:
```
namespace roughroute {};
interface Router {
  [Throws=RouterError]
  constructor(bytes data);
  [Throws=RouterError]
  RouteResult route(sequence<Coord> waypoints, Profile profile);
};
dictionary RouteResult { sequence<Coord> line; double meters; boolean fallback; };
dictionary Coord { double lat; double lon; };
enum Profile { "Car", "Foot" };
[Error] enum RouterError { "TooFewWaypoints", "SnapTooFar", "BadGraph" };
```
Kotlin call in route-spoofer's Android part:
```kotlin
val router = Router(graphBytes)                 // graphBytes from assets/cache
val res = router.route(waypoints, Profile.CAR)  // res.line -> foreground service
```

### 6.5. CLI (`crates/cli`)
```
# Build a graph (dev/CI, offline preprocessing)
roughroute build --pbf cyprus.osm.pbf --out cyprus.graph --profiles car,foot
roughroute build --pbf-url https://download.geofabrik.de/europe/cyprus-latest.osm.pbf --out cyprus.graph

# Route (including GPX generation, which the tool's GPX provider will pick up)
roughroute route --graph cyprus.graph --profile car \
  --via 34.7071,33.0226 --via 34.6841,33.0379 \
  --format gpx > route.gpx
# --format: json | geojson | gpx
```
The `route` mode with GPX output closes the loop with the GPX provider: you can prepare routes offline ahead of time and feed them to the tool as a file.

---

## 7. Binary graph format `.graph`

Compact, mmap-friendly, versioned. Little-endian.

> **⚠ Superseded — this section describes the original v1 layout only.** The
> shipped on-disk format has since evolved: **v2** (M4) added the degree-2
> collapse and a shared intermediate-geometry block, and **v3** (M7)
> delta-compresses that geometry pool. The header is 40 bytes and edge
> records are 16 bytes in the current format; older versions are refused with
> `UnsupportedVersion`. The authoritative current layout is
> `docs/DECISIONS.md` D12 (v2) and D19 (v3). The design goals below (compact,
> versioned, every way-node reconstructable) still hold; the byte offsets do
> not.

### 7.1. Header (fixed)
| Field | Type | Description |
|---|---|---|
| magic | `[u8;4]` | `b"RRG1"` |
| version | `u16` | format version, starts at `1` |
| flags | `u16` | bit flags (e.g. presence of spatial index) |
| node_count | `u32` | number of nodes |
| edge_count | `u32` | number of directed edges (the graph is stored directed, but at build time both directions are duplicated → "undirected") |
| min_lat, min_lon, max_lat, max_lon | `i32×4` | bbox in fixed-point 1e7 |

### 7.2. Nodes
- `nodes: [i32; node_count*2]` — coordinates as fixed-point `round(deg * 1e7)`. 4 bytes/coordinate instead of 8 (f64). Convert back to degrees by dividing by 1e7.

### 7.3. Edges in CSR (Compressed Sparse Row) format
- `offsets: [u32; node_count + 1]` — for node `n`, its edges live in `edges[offsets[n]..offsets[n+1]]`.
- `edges: [Edge; edge_count]`, where
  ```
  Edge { target: u32, length_dm: u32, access: u8, _pad: [u8;3] }
  ```
  - `target` — index of the neighbor node.
  - `length_dm` — edge length in decimeters (u32 covers 429,000 km).
  - `access` — profile bitmask: `bit0 = car`, `bit1 = foot`. A single graph serves both profiles; filtering happens in A\* by mask.

### 7.4. Geometry (F6)
The simplest path for v1 is to make **every OSM way node a graph node** (not just junctions). Then edges are short and the "node→node→…" polyline follows the road shape automatically — no separate intermediate geometry needed. The graph is a bit larger, but for regional scale this is acceptable and radically simpler.

> Future optimization (v2): collapse degree-2 chains into a single edge and store intermediate geometry in a separate block (polyline compression). Shrinks the graph several-fold. Not done in v1.

### 7.5. Spatial index for snapping
For nearest-node. v1 option: **build it at load time** as a uniform grid over the bbox — simple and fast for small regions, doesn't bloat the file. If `flags` indicates the index is baked in, read it from the file (a later optimization). We start with "build on load".

---

## 8. Algorithms

### 8.1. Snap waypoint → node
- Find the nearest graph node to the coordinate (nearest-node) via the grid index.
- Distance is haversine. If the nearest node is farther than `max_snap_meters` → `RouteError::SnapTooFar` (point in the sea/desert/outside the region).
- v2: snap to the nearest point on an edge (projection onto the segment) for a more accurate start.

### 8.2. Pathfinding — A\*
- Classic A\* over the CSR graph.
- Edge cost = `length_dm` (in "rough" mode cost = length; profile speed doesn't yet affect the path choice, only optionally the F11 time estimate).
- Heuristic = haversine to the target in decimeters (admissible/consistent, since a straight line ≤ road distance).
- Filter edges by `access & profile_mask` — skip edges not allowed for the profile.
- Priority queue (binary heap). At regional scale CH is not needed.

### 8.3. Multi-point
- Run a separate A\* for each consecutive pair of waypoints.
- Concatenate results, dedup the junction point (end of segment == start of the next).
- If any segment falls back, the overall `fallback=true`.

### 8.4. Fallback (F7)
- If A\* finds no path (disconnected components) and `allow_fallback=true`, insert a straight segment between the snapped nodes and set `fallback=true`.
- If `allow_fallback=false`, return a `RouteError`.
- For the spoofer the default is `allow_fallback=true`: "some route".

### 8.5. Length
- `meters` — the sum of haversine over the final polyline (not the sum of `length_dm`), so it matches what we actually return in `line`.

---

## 9. Non-functional requirements

| Category | Requirement (target) |
|---|---|
| Query speed | city/island-sized region — single-digit ms for A\* |
| Graph build speed | Cyprus — seconds on a dev machine |
| Runtime memory | Cyprus graph — a few to tens of MB in linear memory |
| `.graph` size | Cyprus — a few MB |
| WASM bundle size | compact, tree-shakeable; without the `build` feature |
| Offline | runtime core with no network and no filesystem; input is `&[u8]` |
| Determinism | stable traversal order, tie-break by node index |
| Platforms | WASM (webview), Android arm64/armv7 (NDK), x86_64 (CLI/CI), iOS if desired |
| Threading | `Graph` is `Send + Sync`, immutable after load; multiple `Router`s over one `Graph` |

---

## 10. Error handling and edge cases

| Case | Behavior |
|---|---|
| `waypoints.len() < 2` | `TooFewWaypoints` |
| Point farther than `max_snap_meters` from roads | `SnapTooFar { index, meters }` |
| Disconnected components between A and B | fallback straight line (or error if fallback is off) |
| Corrupt/truncated `.graph` | `GraphError::Truncated` / `BadMagic` / `UnsupportedVersion` |
| Empty region (no `highway`) | empty graph → any routing = `SnapTooFar` |
| Coinciding consecutive waypoints | zero-length segment, deduped |
| Profile with no allowed edges at a point | treated as no path → fallback |

---

## 11. Testing

- **Fixture:** a small region (Cyprus or a single town), stored in `testdata/` or downloaded in CI (not in the unit runtime).
- **Unit:** haversine/bearing, CSR traversal, format parser (round-trip: build→bytes→load→compare), snapping.
- **Integration:** build a `.graph` from the fixture → route between known points → invariant checks:
  - the line is connected (adjacent points within a reasonable step);
  - the line endpoints are snapped within `max_snap_meters` of the requested points;
  - `meters` > 0 and grows monotonically along the traversal;
  - `car` and `foot` produce valid (possibly different) lines.
- **Property-based** (proptest): random points in the bbox → either a valid route or a controlled error/fallback, with no panics.
- **Golden/determinism:** rerunning with the same input → identical output.
- **Cross-target smoke:** WASM (wasm-bindgen-test) and native (UniFFI) build and respond on a trivial graph.

---

## 12. Roadmap

| Milestone | Content | Result |
|---|---|---|
| **M0 — Core + CLI** | `.graph` format, `build` from `.pbf`, A\*, snapping, fallback, CLI `route` (json/geojson/gpx) | graph and route build on desktop; the link to the tool's GPX provider already works |
| **M1 — WASM** | `wasm-bindgen` wrapper, load graph from `ArrayBuffer`, contract response | the router runs in route-spoofer's webview offline |
| **M2 — Native/Kotlin** | UniFFI wrapper, arm64/armv7 builds, Kotlin binding | the router is called from the tool's Android part as a native library |
| **M3 — Profiles & polish** | proper tag sets for `car`/`foot`, `max_snap_meters`, duration estimate (F11) | two sensible profiles, snap settings |
| **M4 — Optimizations (v2)** | collapse degree-2 chains + geometry in a separate block, baked spatial index, edge snapping (F10) | smaller graph, more accurate start |

The M0→M1→M2 order is intentional: usefulness on desktop first (GPX generation for the tool is already here), then two embedding paths.

---

## 13. Repository, license, CI

- A separate public repository, MIT, neutral name (not `spoofer-*`) — so the router can be reused without route-spoofer.
- README: goal, `© OpenStreetMap contributors` attribution (ODbL), `build` instructions, WASM/CLI example.
- CI (GitHub Actions): `cargo test`, build the WASM artifact, build native artifacts (arm64/armv7), publish a default region's `region.graph` as a release asset (so the tool can download it — see 2.1).
- Version the `.graph` format via the `version` field; breaking format changes → bump the version + refuse to load old data with a clear error.

---

## 14. Open questions (decide before/during M0)

1. **Coordinate order in the contract:** `[lat,lon]` (as in this spec) or `[lon,lat]`? Fix it to match route-spoofer's current UI to avoid pointless conversion. Internally — consistent.
2. **`foot` profile scope:** which `highway` tags to include (`footway`, `path`, `steps`, `pedestrian`, `track`?) and what to exclude from `car` (`motorway` — yes; `service`/`track` — ?). Draw up explicit tag lists during M3.
3. **Default region in the bundle:** ship some `region.graph` right in the APK for "works out of the box", or always download+cache? (Decided on the tool side, but it affects the router's CI artifacts.)
4. **`region.graph` hosting:** are GitHub release assets enough, or is a CDN needed? Who rebuilds the graph, and how often, as OSM updates.
5. **Snap precision:** is node snapping (v1) acceptable, or do we need edge snapping (F10) right away for an accurate start on long edges?
6. **Region size policy:** up to what region size do we keep a single `.graph` without the v2 optimizations (degree-2 collapse). A reference point for "when M4 becomes mandatory".
7. **iOS:** is the target needed now (UniFFI can do Swift), or Android+WASM only to start?

---

*End of spec v1. Edit sections 6 (contract) and 7 (format) first — they fix the boundaries between the router, the tool, and the data; the rest can be refined as M0 proceeds.*
