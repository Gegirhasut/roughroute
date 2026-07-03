//! `roughroute-core` — the pure algorithmic heart of the roughroute OSM
//! mini-router.
//!
//! # Hard constraints
//!
//! - **No I/O of any kind.** No network, filesystem, or `.osm.pbf` parsing
//!   here (that will live in a separate `roughroute-build` crate).
//! - **Deterministic.** Identical input produces identical output.
//! - **Panic-free library code.**
//!
//! # Coordinate order
//!
//! `[lat, lon]` everywhere in this crate's API, matching the shared JSON
//! contract.

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]
#![deny(missing_docs)]

pub mod format;
pub mod geo;
pub mod graph;
pub mod profile;

mod grid;

pub use graph::{BBox, Edge, Graph, GraphError};
pub use profile::Profile;
