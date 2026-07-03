//! `roughroute batch` — build and index every region in `regions.toml`
//! (docs/DECISIONS.md D17). Dev/CI tooling only; the runtime never downloads.
//!
//! Disk discipline (the CLAUDE.md rule, enforced in code): per region,
//! strictly check-headroom → download → build → verify → **delete the
//! `.pbf`** (also on failure) before touching the next region. Never two
//! `.pbf`s at once; the whole run aborts before a download that could fill
//! the disk.

use std::error::Error;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;
use std::process::Command;

use roughroute_core::{Graph, Profile, RouteOptions, Router};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Free space that must remain after a download (beyond the estimated
/// artifacts), so a build never runs the disk to the wire.
const HEADROOM_FLOOR_BYTES: u64 = 1 << 30; // 1 GiB
/// Assumed download size when the server sends no Content-Length.
const UNKNOWN_PBF_ESTIMATE_BYTES: u64 = 5 << 30; // 5 GiB, deliberately harsh
/// Hard safety ceiling on a single `.pbf`: regardless of disk headroom, a
/// probed (or unknown) size above this aborts the whole run rather than
/// downloading. This is a blunt guard against an accidentally huge region
/// (a continent, a whole country the size of the US) landing in the
/// manifest and eating an unattended run's disk or wall-clock budget; raise
/// it deliberately if a legitimately larger region is ever added.
const HARD_MAX_PBF_BYTES: u64 = 800_000_000; // 800 MB (decimal)

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
#[derive(Serialize)]
struct Index<'a> {
    schema_version: u32,
    format_version: u16,
    attribution: &'static str,
    regions: &'a [IndexRegion],
}

#[derive(Serialize)]
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
) -> Result<(), Box<dyn Error>> {
    let manifest: Manifest = toml::from_str(&fs::read_to_string(manifest_path).map_err(
        |e| format!("cannot read manifest {}: {e}", manifest_path.display()),
    )?)?;
    validate_manifest(&manifest)?;

    fs::create_dir_all(out_dir)?;
    let tmp_dir = out_dir.join(".tmp");
    fs::create_dir_all(&tmp_dir)?;

    let mut built: Vec<IndexRegion> = Vec::new();
    let mut skipped: Vec<(String, String)> = Vec::new();
    for region in &manifest.regions {
        eprintln!("──── {} ({}) ────", region.id, region.pbf_url);
        match build_region(region, out_dir, &tmp_dir, release_url_base) {
            Ok(entry) => {
                eprintln!(
                    "  ok: {} ({:.1} MB, {} nodes, {} edges, sha256 {}…)",
                    entry.file,
                    entry.bytes as f64 / (1024.0 * 1024.0),
                    entry.nodes,
                    entry.edges,
                    &entry.sha256[..12],
                );
                built.push(entry);
            }
            Err(RegionError::AbortRun(msg)) => {
                let _ = fs::remove_dir_all(&tmp_dir);
                return Err(format!("aborting batch run: {msg}").into());
            }
            Err(RegionError::Skip(msg)) => {
                eprintln!("  FAILED, skipping region: {msg}");
                skipped.push((region.id.clone(), msg));
            }
        }
    }
    let _ = fs::remove_dir_all(&tmp_dir); // no scratch left behind

    let index_path = out_dir.join("index.json");
    fs::write(&index_path, render_index(&built)?)?;
    eprintln!(
        "\nwrote {} ({} regions built, {} failed)",
        index_path.display(),
        built.len(),
        skipped.len()
    );
    for (id, msg) in &skipped {
        eprintln!("  failed: {id}: {msg}");
    }

    eprintln!("\nTo publish as a GitHub release (see README \"Regional graphs\"):");
    eprintln!(
        "  gh release create graphs-v<N> {}/*.graph {} \\\n    --title \"Region graphs\" \\\n    --notes \"Map data © OpenStreetMap contributors (ODbL). Format v{}.\"",
        out_dir.display(),
        index_path.display(),
        roughroute_core::format::VERSION,
    );

    if skipped.is_empty() {
        Ok(())
    } else {
        Err(format!("{} region(s) failed to build", skipped.len()).into())
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
    region: &ManifestRegion,
    out_dir: &Path,
    tmp_dir: &Path,
    release_url_base: Option<&str>,
) -> Result<IndexRegion, RegionError> {
    // 1a. Hard size ceiling — checked before anything else, and before the
    //     headroom math (which would otherwise happily approve a technically
    //     fitting but unattended-run-hostile multi-GB download). An unknown
    //     size (HEAD gave no Content-Length) is treated as a failure of this
    //     gate too: we refuse to download blind past a safety limit.
    let content_length = probe_content_length(&region.pbf_url);
    match content_length {
        Some(len) if len > HARD_MAX_PBF_BYTES => {
            return Err(RegionError::AbortRun(format!(
                "{}: .pbf is {:.1} MB, over the {:.0} MB hard safety ceiling — refusing to \
                 download; pick a smaller region or raise HARD_MAX_PBF_BYTES deliberately",
                region.id,
                len as f64 / 1_000_000.0,
                HARD_MAX_PBF_BYTES as f64 / 1_000_000.0,
            )));
        }
        Some(len) => eprintln!("  .pbf size: {:.1} MB (within the safety ceiling)", len as f64 / 1_000_000.0),
        None => {
            return Err(RegionError::AbortRun(format!(
                "{}: could not determine .pbf size via HEAD request; refusing to download \
                 without a size safety check",
                region.id
            )));
        }
    }

    // 1b. Disk headroom gate — before the download, aborting the run rather
    //     than risking a full disk (CLAUDE.md "Disk usage").
    let need = estimated_need_bytes(content_length);
    let avail = available_bytes(out_dir)
        .map_err(|e| RegionError::AbortRun(format!("cannot determine free disk space: {e}")))?;
    if avail < need {
        return Err(RegionError::AbortRun(format!(
            "insufficient disk for {}: {:.1} GiB free, {:.1} GiB needed (download + build + 1 GiB floor)",
            region.id,
            avail as f64 / (1 << 30) as f64,
            need as f64 / (1 << 30) as f64,
        )));
    }

    // 2. Download to scratch.
    let pbf_path = tmp_dir.join(format!("{}.osm.pbf", region.id));
    let downloaded = download(&region.pbf_url, &pbf_path);

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
        fs::write(&graph_path, graph.to_bytes())
            .map_err(|e| RegionError::Skip(format!("cannot write graph: {e}")))?;
        drop(graph); // verify the *file*, exactly as the runtime will see it

        let bytes = fs::read(&graph_path)
            .map_err(|e| RegionError::Skip(format!("cannot re-read graph: {e}")))?;
        let (nodes, edges, bbox) = verify_graph_bytes(&bytes).map_err(RegionError::Skip)?;
        let url = match release_url_base {
            Some(base) => format!("{}/{}", base.trim_end_matches('/'), file),
            None => file.clone(),
        };
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
        })
    });

    // 5. Delete the .pbf before the next region — the core disk rule.
    let _ = fs::remove_file(&pbf_path);
    result
}

/// Load the written bytes the way the runtime does and prove a trivial route
/// works: `(nodes, edges, bbox)` on success.
fn verify_graph_bytes(bytes: &[u8]) -> Result<(u32, u32, [f64; 4]), String> {
    let graph = Graph::from_bytes(bytes).map_err(|e| format!("verification load failed: {e}"))?;
    if graph.node_count() == 0 {
        return Err("graph has no nodes (empty or non-road region?)".into());
    }
    // Route between two node coordinates; fallback allowed, so only a snap
    // failure (impossible from a node coordinate) or a bug can fail this.
    let a = graph.node_latlon(0);
    let b = graph.node_latlon(graph.node_count() / 2);
    for profile in [Profile::Car, Profile::Foot] {
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

/// HEAD the URL and read `Content-Length` (following redirects — Geofabrik's
/// `-latest.osm.pbf` URLs 302 to a dated file, so a redirect-naive probe
/// would see the tiny HTML redirect body's length instead of the real one).
/// `ureq`'s default agent follows redirects for HEAD same as GET.
fn probe_content_length(url: &str) -> Option<u64> {
    ureq::head(url)
        .call()
        .ok()
        .and_then(|resp| resp.header("content-length").and_then(|v| v.parse::<u64>().ok()))
}

/// `2 × content_length + 1 GiB` (D17): source + built graph + floor. A harsh
/// default when the length is unknown.
fn estimated_need_bytes(content_length: Option<u64>) -> u64 {
    match content_length {
        Some(len) => len.saturating_mul(2).saturating_add(HEADROOM_FLOOR_BYTES),
        None => UNKNOWN_PBF_ESTIMATE_BYTES,
    }
}

/// Free bytes on the filesystem holding `path`, via `df -P -B1` (POSIX output
/// format; this is dev/CI tooling for Linux/macOS runners).
fn available_bytes(path: &Path) -> Result<u64, Box<dyn Error>> {
    let output = Command::new("df").arg("-P").arg("-B1").arg(path).output()?;
    if !output.status.success() {
        return Err(format!("df exited with {}", output.status).into());
    }
    parse_df_available(&String::from_utf8_lossy(&output.stdout))
}

/// Parse the "Available" column of `df -P` output (second line, 4th field).
fn parse_df_available(df_output: &str) -> Result<u64, Box<dyn Error>> {
    let line = df_output.lines().nth(1).ok_or("df output has no data line")?;
    let field = line.split_whitespace().nth(3).ok_or("df line has no Available field")?;
    Ok(field.parse::<u64>()?)
}

fn download(url: &str, to: &Path) -> Result<(), RegionError> {
    let skip = |e: String| RegionError::Skip(format!("download failed: {e}"));
    let response = ureq::get(url).call().map_err(|e| skip(e.to_string()))?;
    let mut file = fs::File::create(to).map_err(|e| skip(e.to_string()))?;
    let copied = std::io::copy(&mut response.into_reader(), &mut file)
        .map_err(|e| skip(e.to_string()))?;
    eprintln!("  downloaded {:.1} MB", copied as f64 / (1024.0 * 1024.0));
    Ok(())
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
    fn index_json_has_the_d17_shape() {
        let entry = IndexRegion {
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
        };
        let json = render_index(std::slice::from_ref(&entry)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v["schema_version"], 1);
        assert_eq!(v["format_version"], roughroute_core::format::VERSION);
        assert_eq!(v["attribution"], "© OpenStreetMap contributors (ODbL)");
        assert_eq!(v["regions"][0]["id"], "cyprus");
        assert_eq!(v["regions"][0]["sha256"].as_str().unwrap().len(), 64);
        assert_eq!(v["regions"][0]["bbox"].as_array().unwrap().len(), 4);
    }

    #[test]
    fn sha256_matches_known_vector() {
        // SHA-256("abc")
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn estimated_need_scales_with_known_length_and_is_harsh_when_unknown() {
        let need = estimated_need_bytes(Some(300_000_000));
        assert_eq!(need, 300_000_000 * 2 + HEADROOM_FLOOR_BYTES);
        assert_eq!(estimated_need_bytes(None), UNKNOWN_PBF_ESTIMATE_BYTES);
    }

    #[test]
    fn hard_max_pbf_ceiling_is_800mb() {
        // Pin the constant so a casual edit doesn't silently loosen the
        // safety gate.
        assert_eq!(HARD_MAX_PBF_BYTES, 800_000_000);
    }

    #[test]
    fn df_output_parses() {
        let out = "Filesystem     1-blocks       Used  Available Capacity Mounted on\n\
                   /dev/sda1    105089261568 46349357056 53355900928      47% /\n";
        assert_eq!(parse_df_available(out).unwrap(), 53_355_900_928);
        assert!(parse_df_available("").is_err());
        assert!(parse_df_available("header only\n").is_err());
    }
}
