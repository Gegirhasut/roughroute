//! `roughroute-build` — the ahead-of-time preprocessor turning an OSM
//! extract (`.osm.pbf`) into a routable `roughroute` graph.
//!
//! Runs on dev machines and CI only; it is never part of the WASM or mobile
//! runtime (spec §5.1). The crate is layered for testability
//! (`docs/DECISIONS.md` D8):
//!
//! - [`network`] — the pure graph-construction layer: an in-memory road
//!   network (ways + node coordinates) in, a validated [`roughroute_core::Graph`]
//!   out. All correctness-critical logic (junction dedup, determinism, edge
//!   merging) lives here and is tested without any `.pbf` file.
//! - [`pbf`] — the thin `osmpbf` front-end producing that in-memory form.
//! - [`tags`] — the `highway` tag → profile access mask table.

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]
#![deny(missing_docs)]

pub mod network;
pub mod pbf;
pub mod tags;

pub use network::{build_graph, build_graph_with_options, BuildStats, RawWay};
pub use pbf::read_road_network;

use roughroute_core::GraphError;

/// Errors produced while building a graph.
#[derive(Debug)]
pub enum BuildError {
    /// The road network exceeds the format's `u32` node/edge indices.
    TooLarge,
    /// The region's longitude span exceeds 180° — it crosses the antimeridian
    /// or is more than half the globe wide, which the snapping projection does
    /// not support. Not a code fault: such regions are out of scope for v1.
    AntimeridianSpanning,
    /// The assembled graph failed `roughroute-core` validation (indicates a
    /// bug in this crate rather than bad input).
    Graph(GraphError),
    /// Reading the `.osm.pbf` failed (I/O or decode).
    Pbf(String),
}

impl core::fmt::Display for BuildError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BuildError::TooLarge => {
                write!(f, "road network too large for the .graph format (exceeds u32 node/edge indices)")
            }
            BuildError::AntimeridianSpanning => write!(
                f,
                "region spans more than 180° of longitude (crosses the antimeridian?) — unsupported"
            ),
            BuildError::Graph(e) => write!(f, "graph assembly failed: {e}"),
            BuildError::Pbf(e) => write!(f, "failed to read .osm.pbf: {e}"),
        }
    }
}

impl std::error::Error for BuildError {}

impl From<GraphError> for BuildError {
    fn from(e: GraphError) -> Self {
        BuildError::Graph(e)
    }
}
