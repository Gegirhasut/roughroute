//! `roughroute` — the command-line tool (spec §6.5).
//!
//! Two subcommands: `build` (ahead-of-time `.osm.pbf` → `.graph`
//! preprocessing) and `route` (offline routing over a `.graph`, exported as
//! contract JSON, GeoJSON, or GPX). Route output goes to stdout; diagnostics
//! go to stderr.

mod batch;
mod export;
mod mem;
mod net;

use std::error::Error;
use std::fs;
use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};
use roughroute_build::{build_graph_compact, read_road_network};
use roughroute_core::{Graph, Profile, RouteOptions, Router};

#[derive(Parser)]
#[command(
    name = "roughroute",
    about = "Fast offline OSM mini-router: build road graphs and route over them.\n\
             Map data © OpenStreetMap contributors (ODbL)."
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Build a .graph road network from an OSM extract (dev/CI preprocessing).
    Build {
        /// Path to a local .osm.pbf extract.
        #[arg(long, conflicts_with = "pbf_url", required_unless_present = "pbf_url")]
        pbf: Option<PathBuf>,
        /// URL of a .osm.pbf to download first (build-time only; the routing
        /// runtime never touches the network).
        #[arg(long)]
        pbf_url: Option<String>,
        /// Output .graph path.
        #[arg(long)]
        out: PathBuf,
        /// Profiles to include, comma-separated (ways usable by none are dropped).
        #[arg(long, value_delimiter = ',', default_values = ["car", "foot"])]
        profiles: Vec<CliProfile>,
        /// Download safety gate (only relevant with --pbf-url).
        #[command(flatten)]
        gate: PbfGateArgs,
    },
    /// Build, verify, and index every region in the manifest (dev/CI):
    /// per region download -> build -> verify -> delete the .pbf, then write
    /// index.json next to the graphs. See README "Regional graphs".
    ///
    /// Incremental: a region already present in index.json with a matching
    /// file hash and the current format version is skipped entirely (no
    /// probe, no download, no rebuild). Pass --force to rebuild everything.
    Batch {
        /// Region manifest (docs/DECISIONS.md D17).
        #[arg(long, default_value = "regions.toml")]
        manifest: PathBuf,
        /// Output directory for <id>.graph files and index.json.
        #[arg(long, default_value = "dist")]
        out_dir: PathBuf,
        /// Base URL the graphs will be served from (e.g. a GitHub release
        /// download URL); index.json falls back to bare file names when
        /// omitted.
        #[arg(long)]
        release_url_base: Option<String>,
        /// Rebuild every region even if index.json says it's up to date.
        #[arg(long)]
        force: bool,
        /// Trust index.json's recorded hash/format_version for a region
        /// instead of re-reading its .graph file from disk — no local .graph
        /// needed to decide a skip. For CI, where downloading every published
        /// graph just to confirm it's unchanged would defeat the point of
        /// incrementality. Off by default: local runs use the stronger
        /// disk re-hash. See docs/DECISIONS.md D20 addendum.
        #[arg(long)]
        trust_index: bool,
        /// Download safety gate (applies to every region's .pbf).
        #[command(flatten)]
        gate: PbfGateArgs,
    },
    /// Build a route over a .graph and print it to stdout.
    Route {
        /// Path to the .graph file produced by `roughroute build`.
        #[arg(long)]
        graph: PathBuf,
        /// Routing profile.
        #[arg(long, default_value = "car")]
        profile: CliProfile,
        /// Waypoint as `lat,lon` (repeat at least twice, in visit order).
        #[arg(long = "via", required = true, num_args = 1..)]
        via: Vec<String>,
        /// Output format.
        #[arg(long, default_value = "json")]
        format: Format,
        /// Maximum waypoint-to-road snapping distance in meters.
        #[arg(long, default_value_t = 200.0)]
        max_snap_meters: f64,
        /// Fail instead of bridging unreachable legs with a straight line.
        #[arg(long)]
        no_fallback: bool,
    },
}

/// Shared download-safety options, flattened into every subcommand that can
/// reach [`net::gate_download`] (`build --pbf-url`, `batch`) so the flag is
/// defined once. Resolved by [`net::resolve_max_pbf_ceiling`] (flag > env >
/// default) before the gate runs.
#[derive(Args)]
struct PbfGateArgs {
    /// Max size (decimal GB) of a single .osm.pbf the builder will download [default: 6].
    #[arg(
        long = "max-pbf-gb",
        value_name = "GB",
        value_parser = net::parse_max_pbf_gb,
        long_help = "Max size of a single .osm.pbf the builder will download, in decimal GB \
(6, 6.5, 12). Default 6 GB.\n\n\
WHY: a safety brake so an accidental or mistyped huge source — a US-sized country, or the \
whole planet — can't silently start a multi-hour download that fills the disk on an \
unattended run.\n\n\
NOT A MEMORY LIMIT: the builder never gates on RAM; per-region peak RAM is only measured \
and logged. Raising this will NOT prevent an out-of-memory kill.\n\n\
BEFORE RAISING: peak build RAM is roughly 4.4x the .pbf size (the build is in-memory — no \
streaming yet), so ensure about 4.4x that much free RAM, and separately at least 2x the \
pbf size + 1 GiB of free disk.\n\n\
STILL ENFORCED: the disk-headroom check (2x pbf + 1 GiB free) always applies, independent \
of this flag.\n\n\
PRECEDENCE: --max-pbf-gb overrides the ROUGHROUTE_MAX_PBF_BYTES env var, which overrides \
the built-in 6 GB default."
    )]
    max_pbf_gb: Option<f64>,
}

#[derive(Clone, Copy, ValueEnum)]
enum CliProfile {
    Car,
    Foot,
}

impl From<CliProfile> for Profile {
    fn from(p: CliProfile) -> Profile {
        match p {
            CliProfile::Car => Profile::Car,
            CliProfile::Foot => Profile::Foot,
        }
    }
}

#[derive(Clone, Copy, ValueEnum)]
enum Format {
    Json,
    Geojson,
    Gpx,
}

fn main() {
    if let Err(err) = run(Cli::parse()) {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::Build { pbf, pbf_url, out, profiles, gate } => {
            cmd_build(pbf, pbf_url, out, &profiles, gate.max_pbf_gb)
        }
        Command::Batch { manifest, out_dir, release_url_base, force, trust_index, gate } => {
            batch::cmd_batch(
                &manifest,
                &out_dir,
                release_url_base.as_deref(),
                force,
                trust_index,
                gate.max_pbf_gb,
            )
        }
        Command::Route { graph, profile, via, format, max_snap_meters, no_fallback } => {
            cmd_route(&graph, profile, &via, format, max_snap_meters, no_fallback)
        }
    }
}

fn cmd_build(
    pbf: Option<PathBuf>,
    pbf_url: Option<String>,
    out: PathBuf,
    profiles: &[CliProfile],
    max_pbf_gb: Option<f64>,
) -> Result<(), Box<dyn Error>> {
    // Resolve the input: a local path is used directly; a URL is downloaded
    // through the same safety gate `batch` uses (size probe, hard ceiling,
    // disk headroom, timeouts) and deleted on every exit path.
    match (pbf, pbf_url) {
        (Some(path), _) => build_from_pbf(&path, &out, profiles),
        (None, Some(url)) => {
            let agent = net::agent();
            // The .pbf lands next to the graph output so the headroom check
            // (on the output directory) covers the same filesystem both files
            // use; a bare `--out name.graph` means the current directory.
            let dest_dir = out.parent().filter(|p| !p.as_os_str().is_empty());
            let dest_dir = dest_dir.unwrap_or_else(|| std::path::Path::new("."));
            // Ceiling by precedence (flag > env > default); logged once if moved.
            let max_pbf_bytes = net::resolve_max_pbf_ceiling(max_pbf_gb.map(net::gb_to_bytes));
            net::gate_download(&agent, &url, dest_dir, "download", max_pbf_bytes)?;
            eprintln!("downloading {url} …");
            let pbf_path =
                dest_dir.join(format!(".roughroute-download-{}.osm.pbf", std::process::id()));
            net::download(&agent, &url, &pbf_path)?;
            // Build, then remove the scratch .pbf whether the build succeeded
            // or failed (no leaked download on error).
            let result = build_from_pbf(&pbf_path, &out, profiles);
            let _ = fs::remove_file(&pbf_path);
            result
        }
        (None, None) => unreachable!("clap enforces --pbf or --pbf-url"),
    }
}

/// Build a `.graph` from a local `.osm.pbf` and print build statistics.
fn build_from_pbf(
    pbf_path: &std::path::Path,
    out: &std::path::Path,
    profiles: &[CliProfile],
) -> Result<(), Box<dyn Error>> {
    let keep_mask: u8 = profiles.iter().map(|&p| Profile::from(p).mask()).fold(0, |a, m| a | m);

    let mut network = read_road_network(pbf_path)?;
    // `--profiles` narrows the graph to ways usable by the selected profiles.
    network.mask_access(keep_mask);
    let (graph, stats) = build_graph_compact(network)?;
    let bytes = graph.to_bytes();
    fs::write(out, &bytes)?;

    let bb = graph.bbox();
    eprintln!(
        "wrote {}: {} nodes, {} directed edges, {} geometry points, bbox [{:.4}, {:.4}] – [{:.4}, {:.4}]",
        out.display(),
        graph.node_count(),
        graph.edge_count(),
        graph.geometry_point_count(),
        bb.min_lat,
        bb.min_lon,
        bb.max_lat,
        bb.max_lon,
    );
    if graph.lon_shifted() {
        eprintln!(
            "  antimeridian-crossing region: longitudes stored in a shifted continuous frame \
             (bbox lon may read past ±180°; route output stays [-180, 180] — \
             docs/DECISIONS.md D25); {} coincident seam node(s) stitched",
            stats.seam_nodes_merged,
        );
    }
    eprintln!(
        "  ways used: {}, segments dropped (missing nodes): {}, duplicate edges merged: {}",
        stats.ways_used, stats.segments_dropped_missing_node, stats.duplicate_edges_merged,
    );
    // What the M4 degree-2 collapse bought (spec §7.4 / §12): the "before"
    // size is what the same network costs in the v1 layout (32-byte header,
    // 12-byte edges, every way node a graph node).
    let v1_equivalent = 32
        + stats.nodes_before_collapse * 8
        + (stats.nodes_before_collapse + 1) * 4
        + stats.edges_before_collapse * 12;
    let mb = |b: u64| b as f64 / (1024.0 * 1024.0);
    eprintln!(
        "  collapse: {} → {} nodes, {} → {} directed edges ({} interior collapsed, {} loop chains dropped)",
        stats.nodes_before_collapse,
        graph.node_count(),
        stats.edges_before_collapse,
        graph.edge_count(),
        stats.interior_nodes_collapsed,
        stats.loop_chains_dropped,
    );
    eprintln!(
        "  size: {:.1} MB (was {:.1} MB uncollapsed v1-format) — {:.1}x smaller",
        mb(bytes.len() as u64),
        mb(v1_equivalent),
        v1_equivalent as f64 / bytes.len() as f64,
    );
    Ok(())
}

fn cmd_route(
    graph_path: &PathBuf,
    profile: CliProfile,
    via: &[String],
    format: Format,
    max_snap_meters: f64,
    no_fallback: bool,
) -> Result<(), Box<dyn Error>> {
    let waypoints: Vec<[f64; 2]> =
        via.iter().map(|s| parse_latlon(s)).collect::<Result<_, _>>()?;

    let bytes = fs::read(graph_path)?;
    let graph = Graph::from_bytes(&bytes)?;
    let router = Router::new(
        &graph,
        RouteOptions {
            profile: profile.into(),
            allow_fallback: !no_fallback,
            max_snap_meters,
        },
    );
    let route = router.route(&waypoints)?;

    let rendered = match format {
        Format::Json => export::to_contract_json(&route)?,
        Format::Geojson => export::to_geojson(&route)?,
        Format::Gpx => export::to_gpx(&route),
    };
    println!("{rendered}");
    Ok(())
}

/// Parse a `--via` argument: `lat,lon` in degrees (contract order, D9).
fn parse_latlon(s: &str) -> Result<[f64; 2], String> {
    let err = || format!("invalid --via '{s}': expected 'lat,lon' (e.g. 34.7071,33.0226)");
    let (lat, lon) = s.split_once(',').ok_or_else(err)?;
    let lat: f64 = lat.trim().parse().map_err(|_| err())?;
    let lon: f64 = lon.trim().parse().map_err(|_| err())?;
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Err(format!("--via '{s}' out of range: lat in [-90,90], lon in [-180,180]"));
    }
    Ok([lat, lon])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_definition_is_consistent() {
        use clap::CommandFactory;
        Cli::command().debug_assert();
    }

    #[test]
    fn parse_latlon_accepts_contract_order_and_rejects_junk() {
        assert_eq!(parse_latlon("34.7071,33.0226").unwrap(), [34.7071, 33.0226]);
        assert_eq!(parse_latlon(" 34.7 , 33.0 ").unwrap(), [34.7, 33.0]);
        assert!(parse_latlon("34.7071").is_err());
        assert!(parse_latlon("a,b").is_err());
        assert!(parse_latlon("91.0,33.0").is_err());
        assert!(parse_latlon("34.0,181.0").is_err());
    }
}
