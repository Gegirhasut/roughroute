# crates/cli — agent context

**Single responsibility:** the `roughroute` binary — a thin shell over
`build` and `core`. All routing/parsing logic lives in those crates; the CLI
only does argument parsing, file I/O, and output formatting.

## Commands (spec §6.5 — fixed shape)
```
roughroute build --pbf <file> --out <file.graph> [--profiles car,foot]
roughroute build --pbf-url <url> --out <file.graph>      # downloads at build time only
roughroute route --graph <file.graph> --profile car|foot \
    --via <lat>,<lon> --via <lat>,<lon> [...] --format json|geojson|gpx
```
`--via` order is `lat,lon` (DECISIONS D9); at least two required.

## Output formats
- `json` — exactly the spec §6.1 contract object:
  `{ "line": [[lat,lon],…], "meters": …, "fallback": … }`.
- `geojson` — RFC 7946 `Feature` with a `LineString`; **the only place
  coordinates flip to `[lon,lat]`** — flip happens in the geojson serializer
  with a loud comment (DECISIONS D9). `meters`/`fallback` go in `properties`.
- `gpx` — `<trk><trkseg><trkpt lat=… lon=…/>` (attributes are named; no order
  ambiguity).

## Batch subcommand (M6, DECISIONS D17)

`roughroute batch [--manifest regions.toml] [--out-dir dist]
[--release-url-base URL] [--force]` — dev/CI region pipeline.

**Incremental** (D17 addendum): before anything else, a region already in
`index.json` is checked against the file on disk — matching size, sha256,
*and* `format_version` (per-region field) means it's skipped entirely: no
HEAD probe, no download, no rebuild, just carrying the entry forward
(`url` still recomputed from `--release-url-base`). `--force` bypasses this
for every region. A format version bump makes every cached entry look
stale (their recorded `format_version` no longer matches), which is exactly
right — those files are objectively obsolete under the new format.

For a region that does need building, strictly: **hard size ceiling**
(HEAD-probed `.pbf` size vs `HARD_MAX_PBF_BYTES = 800_000_000`; an unknown
size also aborts — the whole run stops, not just the region) →
disk-headroom check (`df -P -B1`; also aborts the run when short) →
download → build → verify (re-load written bytes + trivial routes) →
**delete the `.pbf`** (also on failure; never two sources at once — the
CLAUDE.md disk rules, enforced in code). Writes `index.json` (D17 schema:
url/bytes/sha256/nodes/edges/bbox/format_version). Publishing is manual/CI
via `gh release create` (see README); no credentials here.

**No RAM gate exists** — only the disk checks above. An attempt to raise
the ceiling to 1.2 GB and build Austria (803.1 MB pbf) was OOM-killed on a
5.8 GB-RAM dev VM (D18); dropped, ceiling reverted to 800 MB. If a future
region needs more than the current ceiling admits, check available memory
first, not just disk.

Seeded regions: cyprus, malta, andorra (tiny), slovenia (mid-size scaling
test, 309.6 MB pbf — the largest region built in this environment). See
PLAN.md "M6 scaling test" / "M6.1 incremental batch" / "M7" for measured
numbers.

**CI publishing (M8/D20).** `.github/workflows/build-regions.yml` runs this
subcommand on a GitHub-hosted runner on a push that changes `regions.toml`,
so region graphs are built/published without a local `.pbf` download and on
the runner's larger RAM. The workflow is a thin driver — it fetches the
published `index.json` **and** the `.graph` assets (the skip re-hashes graph
bytes from disk, so they must be present), runs batch, and uploads only
changed graphs + a fresh index. Don't move batch logic into YAML. `batch`
logs free disk before/after each region so the runner's download→delete
cycle is visible. Peak build RAM is unbounded (no streaming) — a multi-GB
extract can OOM even the runner.

## Constraints
- Route output goes to stdout (pipeable, per spec examples); diagnostics and
  build warnings to stderr. Errors → nonzero exit code with a clear message.
- Keep the binary deterministic: no timestamps injected into outputs
  (GPX metadata included).
