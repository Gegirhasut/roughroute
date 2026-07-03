//! `roughroute-core` — the pure algorithmic heart of the roughroute OSM
//! mini-router.
//!
//! Given the bytes of a pre-built binary road graph (the `.graph` format,
//! see [`format`]) and an ordered list of `[lat, lon]` waypoints, this crate
//! snaps each waypoint to the nearest graph node and runs A\* between
//! consecutive pairs, returning a road-following polyline plus its length in
//! meters.
//!
//! # Hard constraints
//!
//! - **No I/O of any kind.** The only input is `&[u8]`
//!   ([`Graph::from_bytes`]); there is no network, filesystem, or `.osm.pbf`
//!   parsing here (that lives in the `roughroute-build` crate).
//! - **Deterministic.** Identical input produces identical output: A\* breaks
//!   cost ties by node index and all traversal orders are fixed.
//! - **Panic-free library code.** All fallible paths return [`GraphError`] or
//!   [`RouteError`].
//!
//! # Coordinate order
//!
//! `[lat, lon]` everywhere in this crate's API, matching the shared JSON
//! contract. (GeoJSON's `[lon, lat]` flip happens in the CLI exporter, never
//! here.)

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]
#![deny(missing_docs)]

pub mod format;
pub mod geo;
pub mod graph;
pub mod profile;
pub mod router;

mod grid;

pub use graph::{BBox, Edge, Graph, GraphError};
pub use profile::Profile;
pub use router::{RouteError, RouteOptions, RouteResult, Router};
