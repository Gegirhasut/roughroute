//! `roughroute batch` — build and index every region in `regions.toml`
//! (docs/DECISIONS.md D17). Dev/CI tooling only; the runtime never downloads.
//!
//! Disk discipline (the CLAUDE.md rule, enforced in code): per region,
//! strictly check-headroom → download → build → verify → **delete the
//! `.pbf`** (also on failure) before touching the next region. Never two
//! `.pbf`s at once; the whole run aborts before a download that could fill
//! the disk.
//!
//! **Incremental** (docs/DECISIONS.md D17 addendum): before touching a
//! region, its existing `index.json` entry is checked against the file on
//! disk — same `sha256`/size and `format_version` as the graph currently
//! written means the region is already up to date, and it is skipped
//! entirely (no HEAD probe, no download, no rebuild). `--force` bypasses
//! this for every region. Adding one new region to `regions.toml` therefore
//! touches only that region on the next run.

use std::collections::HashMap;
use std::error::Error;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use roughroute_core::{Graph, Profile, RouteOptions, Router};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::net;

/// The committed region manifest (`regions.toml`, D17).
#[derive(Deserialize)]
struct Manifest {
    #[serde(rename = "region", default)]
    regions: Vec<ManifestRegion>,
}

#[derive(Deserialize)]
struct ManifestRegion {
    /// kebab-case id; becomes the artifact name `<id>.graph`.
    id: String,
    /// Display name for the host app's UI.
    name: String,
    /// Geofabrik (or compatible) `.osm.pbf` URL.
    pbf_url: String,
}

/// The published discovery index (`index.json`, D17).
///
/// `format_version` here is informational — the format this run's builder
/// targets — since regions can carry mixed *actual* versions during a
/// migration window (some rebuilt at a new version, some not yet). Each
/// [`IndexRegion`] carries its own authoritative `format_version` for
/// staleness checks.
#[derive(Serialize)]
struct Index<'a> {
    schema_version: u32,
    format_version: u16,
    attribution: &'static str,
    regions: &'a [IndexRegion],
}

/// A minimal reader for an existing `index.json`, tolerant of older schemas
/// (missing fields default rather than failing the whole parse) — a stale
/// schema just means those entries look outdated and get rebuilt.
#[derive(Deserialize, Default)]
struct ExistingIndex {
    #[serde(default)]
    regions: Vec<IndexRegion>,
}

#[derive(Serialize, Deserialize, Clone)]
struct IndexRegion {
    id: String,
    name: String,
    file: String,
    url: String,
    bytes: u64,
    sha256: String,
    nodes: u32,
    edges: u32,
    /// `[min_lat, min_lon, max_lat, max_lon]`, degrees.
    bbox: [f64; 4],
    source_pbf_url: String,
    /// The `.graph` format version this entry's file was built with.
    /// Missing in pre-incremental `index.json` files (defaults to 0, which
    /// never matches a real version — such entries are always rebuilt).
    #[serde(default)]
    format_version: u16,
}

/// A region failure that must stop the whole run (disk safety) vs one that
/// only skips the region.
enum RegionError {
    AbortRun(String),
    Skip(String),
}

pub fn cmd_batch(
    manifest_path: &Path,
    out_dir: &Path,
    release_url_base: Option<&str>,
    force: bool,
) -> Result<(), Box<dyn Error>> {
    let manifest: Manifest = toml::from_str(&fs::read_to_string(manifest_path).map_err(
        |e| format!("cannot read manifest {}: {e}", manifest_path.display()),
    )?)?;
    validate_manifest(&manifest)?;

    fs::create_dir_all(out_dir)?;
    // Startup sweep: wipe any scratch left by a prior run that died mid-build
    // (a panic or OOM kill can leave a partial .pbf behind — the Austria case,
    // docs/DECISIONS.md D18), so a fresh run never inherits stale files.
    let tmp_dir = out_dir.join(".tmp");
    let _ = fs::remove_dir_all(&tmp_dir);
    fs::create_dir_all(&tmp_dir)?;

    // One timeout-configured agent for every download this run (net::agent).
    let agent = net::agent();

    let existing = load_existing_index(&out_dir.join("index.json"));

    let mut index_entries: Vec<IndexRegion> = Vec::new();
    let mut built_count = 0u32;
    let mut skipped_count = 0u32;
    let mut failed: Vec<(String, String)> = Vec::new();
    for region in &manifest.regions {
        eprintln!("──── {} ({}) ────", region.id, region.pbf_url);

        if !force {
            if let Some(cached) = existing.get(&region.id) {
                let graph_path = out_dir.join(&cached.file);
                if cached_entry_is_fresh(cached, &graph_path) {
                    let mut entry = cached.clone();
                    entry.url = compute_url(release_url_base, &entry.file);
                    eprintln!(
                        "  skipped (up to date): {} ({:.1} MB, sha256 {}…)",
                        entry.file,
                        entry.bytes as f64 / (1024.0 * 1024.0),
                        &entry.sha256[..12],
                    );
                    index_entries.push(entry);
                    skipped_count += 1;
                    continue;
                }
            }
        }

        match build_region(&agent, region, out_dir, &tmp_dir, release_url_base) {
            Ok(entry) => {
                eprintln!(
                    "  ok: {} ({:.1} MB, {} nodes, {} edges, sha256 {}…)",
                    entry.file,
                    entry.bytes as f64 / (1024.0 * 1024.0),
                    entry.nodes,
                    entry.edges,
                    &entry.sha256[..12],
                );
                index_entries.push(entry);
                built_count += 1;
            }
            Err(RegionError::AbortRun(msg)) => {
                let _ = fs::remove_dir_all(&tmp_dir);
                return Err(format!("aborting batch run: {msg}").into());
            }
            Err(RegionError::Skip(msg)) => {
                eprintln!("  FAILED, skipping region: {msg}");
                failed.push((region.id.clone(), msg));
            }
        }
    }
    let _ = fs::remove_dir_all(&tmp_dir); // no scratch left behind

    let index_path = out_dir.join("index.json");
    write_atomic(&index_path, render_index(&index_entries)?.as_bytes())?;
    eprintln!(
        "\nwrote {} ({} built, {} skipped up to date, {} failed)",
        index_path.display(),
        built_count,
        skipped_count,
        failed.len()
    );
    for (id, msg) in &failed {
        eprintln!("  failed: {id}: {msg}");
    }

    eprintln!("\nTo publish as a GitHub release (see README \"Regional graphs\"):");
    eprintln!(
        "  gh release create graphs-v<N> {}/*.graph {} \\\n    --title \"Region graphs\" \\\n    --notes \"Map data © OpenStreetMap contributors (ODbL). Format v{}.\"",
        out_dir.display(),
        index_path.display(),
        roughroute_core::format::VERSION,
    );

    if failed.is_empty() {
        Ok(())
    } else {
        Err(format!("{} region(s) failed to build", failed.len()).into())
    }
}

/// Read `index.json` at `path` into an id → entry map, if it exists and
/// parses. Missing or unparsable index means every region is treated as new
/// (first run, or a corrupted index — safer to rebuild than to trust it).
fn load_existing_index(path: &Path) -> HashMap<String, IndexRegion> {
    let Ok(text) = fs::read_to_string(path) else {
        return HashMap::new();
    };
    let Ok(index) = serde_json::from_str::<ExistingIndex>(&text) else {
        return HashMap::new();
    };
    index.regions.into_iter().map(|r| (r.id.clone(), r)).collect()
}

/// Is `cached` still an accurate description of the file at `graph_path`?
/// Requires the current format version (a version bump invalidates every
/// cached entry — the old bytes are objectively obsolete) and an exact
/// size + sha256 match against the file actually on disk.
fn cached_entry_is_fresh(cached: &IndexRegion, graph_path: &Path) -> bool {
    if cached.format_version != roughroute_core::format::VERSION {
        return false;
    }
    let Ok(bytes) = fs::read(graph_path) else {
        return false;
    };
    bytes.len() as u64 == cached.bytes && sha256_hex(&bytes) == cached.sha256
}

fn compute_url(release_url_base: Option<&str>, file: &str) -> String {
    match release_url_base {
        Some(base) => format!("{}/{}", base.trim_end_matches('/'), file),
        None => file.to_string(),
    }
}

fn validate_manifest(manifest: &Manifest) -> Result<(), Box<dyn Error>> {
    if manifest.regions.is_empty() {
        return Err("manifest lists no [[region]] entries".into());
    }
    let mut seen = std::collections::BTreeSet::new();
    for r in &manifest.regions {
        if r.id.is_empty()
            || !r.id.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
        {
            return Err(format!("region id '{}' must be kebab-case [a-z0-9-]", r.id).into());
        }
        if !seen.insert(&r.id) {
            return Err(format!("duplicate region id '{}'", r.id).into());
        }
        if r.name.is_empty() || r.pbf_url.is_empty() {
            return Err(format!("region '{}' needs both name and pbf_url", r.id).into());
        }
    }
    Ok(())
}

/// The full per-region cycle. The `.pbf` is removed before this returns,
/// success or failure.
fn build_region(
    agent: &ureq::Agent,
    region: &ManifestRegion,
    out_dir: &Path,
    tmp_dir: &Path,
    release_url_base: Option<&str>,
) -> Result<IndexRegion, RegionError> {
    // 1. Shared pre-download safety gate (hard size ceiling + disk headroom).
    //    A gate failure aborts the whole run rather than risking a full disk;
    //    the .pbf lands next to the built graph, so headroom is checked on
    //    out_dir. No byte is fetched here beyond a HEAD request.
    net::gate_download(agent, &region.pbf_url, out_dir, &region.id).map_err(RegionError::AbortRun)?;

    // Free disk before the download — paired with the "after cleanup" line
    // below, this makes the per-region download→delete cycle visible in the
    // log (a CI run confirms the disk stays flat region to region).
    log_free_disk(out_dir, "before download");

    // 2. Download to scratch.
    let pbf_path = tmp_dir.join(format!("{}.osm.pbf", region.id));
    let downloaded = net::download(agent, &region.pbf_url, &pbf_path).map_err(RegionError::Skip);

    // 3–4. Build + verify, with the .pbf removed afterwards no matter what.
    let result = downloaded.and_then(|_| {
        let (mut ways, coords) = roughroute_build::read_road_network(&pbf_path)
            .map_err(|e| RegionError::Skip(format!("pbf read failed: {e}")))?;
        let keep = Profile::Car.mask() | Profile::Foot.mask();
        for way in &mut ways {
            way.access &= keep;
        }
        let (graph, _) = roughroute_build::build_graph(&ways, &coords)
            .map_err(|e| RegionError::Skip(format!("graph build failed: {e}")))?;
        let file = format!("{}.graph", region.id);
        let graph_path = out_dir.join(&file);
        write_atomic(&graph_path, &graph.to_bytes())
            .map_err(|e| RegionError::Skip(format!("cannot write graph: {e}")))?;
        drop(graph); // verify the *file*, exactly as the runtime will see it

        let bytes = fs::read(&graph_path)
            .map_err(|e| RegionError::Skip(format!("cannot re-read graph: {e}")))?;
        let (nodes, edges, bbox) = verify_graph_bytes(&bytes).map_err(RegionError::Skip)?;
        let url = compute_url(release_url_base, &file);
        Ok(IndexRegion {
            id: region.id.clone(),
            name: region.name.clone(),
            url,
            bytes: bytes.len() as u64,
            sha256: sha256_hex(&bytes),
            nodes,
            edges,
            bbox,
            file,
            source_pbf_url: region.pbf_url.clone(),
            format_version: roughroute_core::format::VERSION,
        })
    });

    // 5. Delete the .pbf before the next region — the core disk rule.
    let _ = fs::remove_file(&pbf_path);
    log_free_disk(out_dir, "after cleanup");
    result
}

/// Best-effort per-region disk log (reuses the headroom gate's `df`): the
/// `before download` / `after cleanup` pair should stay roughly equal,
/// showing the `.pbf` was removed and disk isn't accumulating.
fn log_free_disk(dir: &Path, when: &str) {
    if let Ok(bytes) = net::available_bytes(dir) {
        eprintln!("  disk {when}: {:.1} GiB free", bytes as f64 / (1u64 << 30) as f64);
    }
}

/// Load the written bytes the way the runtime does and prove a trivial route
/// works: `(nodes, edges, bbox)` on success.
///
/// For each profile that has *any* usable road, route between the endpoints
/// of one such edge — both are exact node coordinates directly joined by a
/// road the profile can use, so this cannot false-negative the way a fixed
/// probe point could (a point whose neighborhood is, say, motorway-only
/// would fail a Foot snap even though the graph is perfectly fine). A profile
/// with no usable road anywhere is simply not verified for that profile.
fn verify_graph_bytes(bytes: &[u8]) -> Result<(u32, u32, [f64; 4]), String> {
    let graph = Graph::from_bytes(bytes).map_err(|e| format!("verification load failed: {e}"))?;
    if graph.node_count() == 0 {
        return Err("graph has no nodes (empty or non-road region?)".into());
    }
    if graph.edge_count() == 0 {
        return Err("graph has nodes but no edges (broken extract?)".into());
    }
    for profile in [Profile::Car, Profile::Foot] {
        let Some((a, b)) = first_usable_edge_endpoints(&graph, profile) else {
            continue; // no road this profile can use — nothing to verify for it
        };
        let router = Router::new(
            &graph,
            RouteOptions { profile, allow_fallback: true, max_snap_meters: 1_000.0 },
        );
        let route = router
            .route(&[a, b])
            .map_err(|e| format!("verification route failed ({profile:?}): {e}"))?;
        if route.line.is_empty() {
            return Err(format!("verification route is empty ({profile:?})"));
        }
    }
    let bb = graph.bbox();
    Ok((
        graph.node_count(),
        graph.edge_count(),
        [bb.min_lat, bb.min_lon, bb.max_lat, bb.max_lon],
    ))
}

/// Coordinates of the endpoints of the first edge usable by `profile`, or
/// `None` if the graph has no road that profile can use.
fn first_usable_edge_endpoints(graph: &Graph, profile: Profile) -> Option<([f64; 2], [f64; 2])> {
    let mask = profile.mask();
    for n in 0..graph.node_count() {
        for e in graph.edges_from(n) {
            if e.access & mask != 0 {
                return Some((graph.node_latlon(n), graph.node_latlon(e.target)));
            }
        }
    }
    None
}

/// Write `bytes` to `final_path` atomically: write to a sibling temp file in
/// the same directory, then rename over the target. A concurrent reader — or
/// a crash mid-write — sees either the old file or the complete new one,
/// never a truncated one. (Same-directory rename is atomic on the local
/// filesystems this dev/CI tool targets.)
fn write_atomic(final_path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let dir = final_path.parent().filter(|p| !p.as_os_str().is_empty());
    let dir = dir.unwrap_or_else(|| Path::new("."));
    let name = final_path.file_name().and_then(|n| n.to_str()).unwrap_or("out");
    let tmp = dir.join(format!(".{name}.{}.tmp", std::process::id()));
    fs::write(&tmp, bytes)?;
    fs::rename(&tmp, final_path)?;
    Ok(())
}

fn render_index(regions: &[IndexRegion]) -> Result<String, Box<dyn Error>> {
    let index = Index {
        schema_version: 1,
        format_version: roughroute_core::format::VERSION,
        attribution: "© OpenStreetMap contributors (ODbL)",
        regions,
    };
    Ok(serde_json::to_string_pretty(&index)?)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut hex = String::with_capacity(64);
    for byte in digest {
        let _ = write!(hex, "{byte:02x}");
    }
    hex
}


#[cfg(test)]
mod tests {
    use super::*;
    use roughroute_build::RawWay;
    use roughroute_core::profile::ACCESS_ALL;

    #[test]
    fn manifest_parses_and_validates() {
        let manifest: Manifest = toml::from_str(
            r#"
            [[region]]
            id = "cyprus"
            name = "Cyprus"
            pbf_url = "https://example.invalid/cyprus.osm.pbf"

            [[region]]
            id = "malta"
            name = "Malta"
            pbf_url = "https://example.invalid/malta.osm.pbf"
            "#,
        )
        .unwrap();
        assert_eq!(manifest.regions.len(), 2);
        assert_eq!(manifest.regions[0].id, "cyprus");
        validate_manifest(&manifest).unwrap();
    }

    #[test]
    fn manifest_rejects_bad_ids_and_duplicates() {
        let bad_id: Manifest = toml::from_str(
            r#"[[region]]
               id = "Cyprus!"
               name = "x"
               pbf_url = "u""#,
        )
        .unwrap();
        assert!(validate_manifest(&bad_id).is_err());

        let dup: Manifest = toml::from_str(
            r#"[[region]]
               id = "a"
               name = "x"
               pbf_url = "u"
               [[region]]
               id = "a"
               name = "y"
               pbf_url = "v""#,
        )
        .unwrap();
        assert!(validate_manifest(&dup).is_err());
        assert!(validate_manifest(&Manifest { regions: vec![] }).is_err());
    }

    #[test]
    fn verify_accepts_a_real_graph_and_rejects_junk() {
        // A tiny two-way town through the real build pipeline.
        let coords: std::collections::BTreeMap<i64, [f64; 2]> =
            [(1, [35.0, 33.0]), (2, [35.0, 33.01]), (3, [35.01, 33.01])]
                .into_iter()
                .collect();
        let ways = vec![RawWay { node_ids: vec![1, 2, 3], access: ACCESS_ALL }];
        let (graph, _) = roughroute_build::build_graph(&ways, &coords).unwrap();
        let (nodes, edges, bbox) = verify_graph_bytes(&graph.to_bytes()).unwrap();
        // Node 2 is degree-2 and collapses into edge geometry (M4).
        assert_eq!((nodes, edges), (2, 2));
        assert!(bbox[0] <= bbox[2] && bbox[1] <= bbox[3]);

        assert!(verify_graph_bytes(b"not a graph").is_err());
        // Valid but empty graph: refused (nothing to route on).
        let empty = roughroute_core::Graph::from_parts(vec![], vec![0], vec![], vec![]).unwrap();
        assert!(verify_graph_bytes(&empty.to_bytes()).is_err());
    }

    #[test]
    fn verify_passes_a_car_only_graph_without_false_negating_on_foot() {
        // A region with only drivable roads must verify (the Foot check is
        // skipped, not failed). The old fixed-probe-point logic would route
        // Foot from a node coordinate and get SnapTooFar, dropping the whole
        // region — this is the false-negative the per-profile fix removes.
        use roughroute_core::profile::ACCESS_CAR;
        let coords: std::collections::BTreeMap<i64, [f64; 2]> =
            [(1, [35.0, 33.0]), (2, [35.0, 33.02])].into_iter().collect();
        let ways = vec![RawWay { node_ids: vec![1, 2], access: ACCESS_CAR }];
        let (graph, _) = roughroute_build::build_graph(&ways, &coords).unwrap();
        assert!(verify_graph_bytes(&graph.to_bytes()).is_ok());
    }

    #[test]
    fn verify_rejects_a_nodes_but_no_edges_graph() {
        // Two nodes, zero edges (broken extract): nothing routes, refuse it.
        let nodes =
            vec![[roughroute_core::geo::deg_to_fixed(35.0), roughroute_core::geo::deg_to_fixed(33.0)]];
        let graph = roughroute_core::Graph::from_parts(nodes, vec![0, 0], vec![], vec![]).unwrap();
        assert!(verify_graph_bytes(&graph.to_bytes()).is_err());
    }

    fn sample_entry() -> IndexRegion {
        IndexRegion {
            id: "cyprus".into(),
            name: "Cyprus".into(),
            file: "cyprus.graph".into(),
            url: "https://example.invalid/dl/cyprus.graph".into(),
            bytes: 42,
            sha256: "ab".repeat(32),
            nodes: 3,
            edges: 4,
            bbox: [34.5, 32.2, 35.7, 34.6],
            source_pbf_url: "https://example.invalid/cyprus.osm.pbf".into(),
            format_version: roughroute_core::format::VERSION,
        }
    }

    #[test]
    fn index_json_has_the_d17_shape() {
        let entry = sample_entry();
        let json = render_index(std::slice::from_ref(&entry)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v["schema_version"], 1);
        assert_eq!(v["format_version"], roughroute_core::format::VERSION);
        assert_eq!(v["attribution"], "© OpenStreetMap contributors (ODbL)");
        assert_eq!(v["regions"][0]["id"], "cyprus");
        assert_eq!(v["regions"][0]["sha256"].as_str().unwrap().len(), 64);
        assert_eq!(v["regions"][0]["bbox"].as_array().unwrap().len(), 4);
        assert_eq!(v["regions"][0]["format_version"], roughroute_core::format::VERSION);
    }

    #[test]
    fn cached_entry_matches_file_on_disk_is_fresh() {
        let dir = std::env::temp_dir().join(format!("rr-batch-test-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("cyprus.graph");
        let bytes = b"pretend graph bytes";
        fs::write(&path, bytes).unwrap();

        let mut entry = sample_entry();
        entry.bytes = bytes.len() as u64;
        entry.sha256 = sha256_hex(bytes);
        assert!(cached_entry_is_fresh(&entry, &path));

        // Wrong format version: never fresh, even with matching bytes.
        let mut stale_version = entry.clone();
        stale_version.format_version = 0;
        assert!(!cached_entry_is_fresh(&stale_version, &path));

        // Bytes on disk changed underneath the recorded hash.
        fs::write(&path, b"different content, same length!!!!!").unwrap();
        let mut wrong_hash = entry.clone();
        wrong_hash.bytes = "different content, same length!!!!!".len() as u64;
        assert!(!cached_entry_is_fresh(&wrong_hash, &path));

        // File missing entirely.
        fs::remove_file(&path).unwrap();
        assert!(!cached_entry_is_fresh(&entry, &path));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn existing_index_round_trips_and_tolerates_missing_format_version() {
        let dir = std::env::temp_dir().join(format!("rr-batch-idx-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("index.json");

        // Fresh schema (current writer).
        let entry = sample_entry();
        fs::write(&path, render_index(std::slice::from_ref(&entry)).unwrap()).unwrap();
        let loaded = load_existing_index(&path);
        assert_eq!(loaded.len(), 1);
        assert_eq!(loaded["cyprus"].format_version, roughroute_core::format::VERSION);

        // Pre-incremental schema (no format_version field at all): parses,
        // defaults to 0, so cached_entry_is_fresh always rejects it.
        let old_schema = r#"{"schema_version":1,"format_version":2,
            "attribution":"x","regions":[{"id":"malta","name":"Malta",
            "file":"malta.graph","url":"u","bytes":1,"sha256":"ab",
            "nodes":1,"edges":1,"bbox":[0.0,0.0,0.0,0.0],
            "source_pbf_url":"u"}]}"#;
        fs::write(&path, old_schema).unwrap();
        let loaded = load_existing_index(&path);
        assert_eq!(loaded["malta"].format_version, 0);

        // Missing file entirely: empty map, not an error.
        let missing = dir.join("does-not-exist.json");
        assert!(load_existing_index(&missing).is_empty());

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn compute_url_uses_base_or_falls_back_to_bare_file() {
        assert_eq!(
            compute_url(Some("https://example.invalid/dl/"), "cyprus.graph"),
            "https://example.invalid/dl/cyprus.graph"
        );
        assert_eq!(
            compute_url(Some("https://example.invalid/dl"), "cyprus.graph"),
            "https://example.invalid/dl/cyprus.graph"
        );
        assert_eq!(compute_url(None, "cyprus.graph"), "cyprus.graph");
    }

    #[test]
    fn sha256_matches_known_vector() {
        // SHA-256("abc")
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
