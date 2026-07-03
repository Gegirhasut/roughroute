# roughroute

A small, fully-offline OSM router: give it ordered waypoints and a profile
(`car` / `foot`), get back a polyline that follows OpenStreetMap roads plus its
length in meters. "Roughly correct" by design — it deliberately ignores
`oneway`, turn restrictions and access rules, so the route is plausible, not
necessarily legal. The same core runs as a CLI, as WASM (webview), and as a
native Android library (UniFFI/Kotlin).

At runtime the router never touches the network or the filesystem: it consumes
the bytes of a pre-built binary graph (`.graph`). Building that graph from an
OSM extract is a separate ahead-of-time step.

## Geodata attribution

Map data **© OpenStreetMap contributors**, licensed under the
[Open Database License (ODbL)](https://www.openstreetmap.org/copyright).
Any product that displays routes produced by this library must show this
attribution. Regional extracts courtesy of [Geofabrik](https://download.geofabrik.de/).

Code license: MIT.

## Build

Requires stable Rust. The Cargo workspace root is this directory (`app/`).

```sh
cargo build --release            # core + build + cli
cargo test                       # unit/integration/property/golden tests
cargo test -- --ignored          # extra tests against testdata/cyprus.osm.pbf, if present
```

## CLI usage

1. Get a regional extract (a few MB for small regions):

```sh
curl -LO https://download.geofabrik.de/europe/cyprus-latest.osm.pbf
```

2. Build the graph (ahead of time, on a dev machine or CI — never on-device):

```sh
roughroute build --pbf cyprus-latest.osm.pbf --out cyprus.graph
```

> `.graph` format version: **3** (degree-2 chains collapsed with an
> intermediate-geometry block, several-fold smaller than v1 — and since v3
> that geometry pool is itself delta-compressed, several-fold smaller again;
> see `docs/DECISIONS.md` D12/D19). Graphs built by an older version are
> refused with a clear `UnsupportedVersion` error; rebuild them with the
> current `roughroute build`.

3. Route. Coordinates are `lat,lon`; repeat `--via` for multi-point routes:

```sh
roughroute route --graph cyprus.graph --profile car \
  --via 34.7071,33.0226 --via 34.6841,33.0379 \
  --format json
```

Output formats: `json` (the neutral contract below), `geojson`
(RFC 7946, `[lon,lat]` — the only place that order appears), `gpx`.

JSON contract (`[lat, lon]` order):

```json
{
  "line": [[34.7071, 33.0226], [34.7069, 33.0231], [34.6841, 33.0379]],
  "meters": 2143.7,
  "fallback": false
}
```

`fallback: true` means at least one leg had no road path and was bridged with
a straight line (the router prefers "some route" over refusing).

## Regional graphs (batch build + publishing)

Pre-built regional `.graph` files are published as **GitHub Release assets**
so route-spoofer (or any host app) can download them by URL and cache them
locally; the router itself never touches the network (delivery is the host
app's job). The discovery entry point is **`index.json`**, published
alongside the graphs: it maps each region to its download URL, size in
bytes, SHA-256, node/edge counts, bbox, and the `.graph` **format version**.
The app should verify the hash after download, and must check the format
version (also embedded in every `.graph` header) — the router refuses
mismatched versions with `UnsupportedVersion`, so ship graphs rebuilt for
the version your router binary expects.

**ODbL applies to every published graph:** any release and any app shipping
these files must carry the `© OpenStreetMap contributors` attribution.

### Adding a region

Append a block to `regions.toml` (kebab-case id; the id becomes the file
name `<id>.graph`):

```toml
[[region]]
id = "malta"
name = "Malta"
pbf_url = "https://download.geofabrik.de/europe/malta-latest.osm.pbf"
```

### Running the batch build

```sh
roughroute batch --out-dir dist \
  --release-url-base https://github.com/OWNER/REPO/releases/download/graphs-v1
```

**Incremental:** a region already in `dist/index.json` whose file matches
the recorded size + SHA-256 *and* the current `.graph` format version is
skipped entirely — no HEAD probe, no download, no rebuild. Pass `--force` to
rebuild everything regardless. A format-version bump makes every cached
entry look stale and triggers a one-time full rebuild on the next run — that
's expected, not a bug: the old bytes are genuinely obsolete under the new
format.

Per region that does need building, strictly in order: probe the `.pbf`
size (HEAD request) and **abort the whole run** if it exceeds an 800 MB hard
safety ceiling (a blunt guard against an accidentally huge region —
continent/country-scale extracts — derailing an unattended run; an unknown
size also aborts, since the check can't be skipped) → check disk headroom
(aborts if a download could fill the disk) → download the `.pbf` to scratch
→ build the `.graph` → verify it (re-load from disk + a trivial route in
both profiles) → **delete the `.pbf`** — sources are never accumulated.
Failed regions are skipped and reported; the exit code is non-zero if any
failed. `index.json` is written to the output directory last.

> Note: this hard ceiling and the disk-headroom check only protect *disk*
> space. Building a region can still hit a *memory* ceiling on a
> constrained machine — see the Austria note below and `docs/DECISIONS.md`
> D18.

Seeded regions and their measured results (2026-07-03, debug build, v3
format):

| region | `.pbf` | `.graph` (v3) | nodes | edges |
|---|---|---|---|---|
| Cyprus | 36.7 MB | 20.8 MB | 267,459 | 705,406 |
| Malta | 8.8 MB | 3.4 MB | 52,292 | 134,612 |
| Andorra | 3.4 MB | 1.1 MB | 9,998 | 28,030 |
| Slovenia | 309.6 MB | 56.4 MB | 657,285 | 1,624,850 |

(v3's geometry delta-compression, D19, shrinks these another 1.15–1.5×
versus the v2 sizes recorded when M4/M5 first built them — see
`docs/DECISIONS.md` D19 for the full before/after table and the real-data
proof that routes are byte-for-byte unchanged.)

Slovenia is the largest region built in this environment — a mid-size
scaling test (a single moderate country, well under the 500 MB target and
the 800 MB hard ceiling). A larger attempt (Austria, 803.1 MB pbf) was
**OOM-killed on this dev VM's 5.8 GB RAM** partway through building and was
dropped rather than pursued further (`docs/DECISIONS.md` D18) — a real
memory constraint of this dev machine, not a code defect; `roughroute batch`
has no RAM gate today, only disk gates.

### Publishing

No credentials live in this repo; publish manually or from CI with the
GitHub CLI (each regional graph is far below the 2 GiB asset limit):

```sh
gh release create graphs-v1 dist/*.graph dist/index.json \
  --title "Region graphs" \
  --notes "Map data © OpenStreetMap contributors (ODbL). Format v3."
```

Bump the tag (`graphs-v2`, …) when regraphs are rebuilt or the format
version changes, and pass the matching `--release-url-base` when generating
`index.json`.

**Future migration path (not set up now):** if release-asset egress ever
becomes a problem, the same files can move to Cloudflare R2 (free egress)
with zero router changes — only the `url` fields in `index.json` change,
regenerated via `--release-url-base`.

## WASM usage (milestone M1)

```ts
import { WasmRouter } from "roughroute-wasm";

const bytes = await loadRegionGraph();          // host app's job: bundle or download+cache
const router = new WasmRouter(new Uint8Array(bytes));
const res = router.route({ waypoints: [[34.7071, 33.0226], [34.6841, 33.0379]],
                           profile: "car" });
// res: { line: [[lat, lon], …], meters: number, fallback: boolean }
```

## Kotlin usage (milestone M2)

```kotlin
val router = Router(graphBytes)                    // graphBytes from assets or cache
val res = router.route(waypoints, Profile.CAR)     // res.line, res.meters, res.fallback
```

## Known limitations

Deliberately out of scope for v1 (see `docs/DECISIONS.md` "Known limitations"
for the full notes and the real fix for each):

- **Antimeridian-spanning regions** (Fiji, Chukotka, NZ + Chathams) are
  **not supported**: the snapping projection is wrong across the ±180° seam,
  so `roughroute build` refuses any region spanning more than 180° of
  longitude with a clear error rather than building one that misroutes.
- **Reproducible builds are same-platform only.** The same `.pbf` yields
  byte-identical `.graph` bytes on the same OS/toolchain, but edge lengths go
  through platform trig that isn't correctly-rounded, so bytes (and the
  `index.json` sha256) can differ across machines. Rebuild per platform;
  don't diff graphs across machines.
- **Snap the app, not the router, to the network.** The runtime never
  downloads; hostile `.graph` inputs are mitigated by the host app verifying
  the published sha256 before use.

## Project docs

- `docs/PLAN.md` — implementation plan and milestone status.
- `docs/DECISIONS.md` — design decisions (binary format details, determinism,
  snapping grid, profile tag table) and known limitations.
- `docs/roughroute-SPEC-EN.md` — the full specification (§7's byte layout is
  superseded by the v2/v3 format in DECISIONS D12/D19).
