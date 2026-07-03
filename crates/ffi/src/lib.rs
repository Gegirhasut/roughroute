//! `roughroute-ffi` — the UniFFI shell around `roughroute-core` for
//! Kotlin/Android (spec §6.4).
//!
//! Interface shape (matches the spec's UDL sketch, expressed with UniFFI
//! proc-macros — the maintained definition style):
//!
//! ```kotlin
//! val router = Router(graphBytes)                 // graphBytes from assets/cache
//! val res = router.route(waypoints, Profile.CAR)  // res.line, res.meters, res.fallback
//! ```
//!
//! No routing logic lives here — only type mapping and error translation.
//! `Coord` uses named `lat`/`lon` fields, so there is no array-order
//! ambiguity on this boundary.

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]
#![deny(missing_docs)]

use std::sync::Arc;

use roughroute_core::{Graph, RouteOptions};

uniffi::setup_scaffolding!();

/// A geographic coordinate in degrees.
#[derive(Debug, Clone, Copy, uniffi::Record)]
pub struct Coord {
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
}

/// A routing result: the road-following polyline and its length.
#[derive(Debug, Clone, uniffi::Record)]
pub struct RouteResult {
    /// Polyline following road geometry, start to end.
    pub line: Vec<Coord>,
    /// Total length in meters (haversine sum over `line`).
    pub meters: f64,
    /// `true` if at least one leg was bridged with a straight line.
    pub fallback: bool,
}

/// Routing profile: which roads may be used.
#[derive(Debug, Clone, Copy, uniffi::Enum)]
pub enum Profile {
    /// Motor vehicle (includes motorways).
    Car,
    /// Pedestrian (excludes motorways, includes footways).
    Foot,
}

/// Errors thrown across the FFI boundary (spec §6.4).
#[derive(Debug, uniffi::Error)]
pub enum RouterError {
    /// Fewer than two waypoints were supplied.
    TooFewWaypoints,
    /// A waypoint is farther than the snap limit from any road.
    SnapTooFar,
    /// The graph bytes are not a valid `.graph` file.
    BadGraph,
}

impl core::fmt::Display for RouterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RouterError::TooFewWaypoints => write!(f, "at least two waypoints are required"),
            RouterError::SnapTooFar => write!(f, "waypoint too far from any road"),
            RouterError::BadGraph => write!(f, "invalid .graph data"),
        }
    }
}

impl std::error::Error for RouterError {}

impl From<roughroute_core::RouteError> for RouterError {
    fn from(e: roughroute_core::RouteError) -> Self {
        match e {
            roughroute_core::RouteError::TooFewWaypoints => RouterError::TooFewWaypoints,
            roughroute_core::RouteError::SnapTooFar { .. } => RouterError::SnapTooFar,
            // NoPath can only surface with fallback disabled; this wrapper
            // always routes with the spec default (fallback on), so this arm
            // is unreachable in practice but total for safety.
            roughroute_core::RouteError::NoPath { .. } => RouterError::SnapTooFar,
        }
    }
}

/// An offline router holding a loaded road graph.
#[derive(uniffi::Object)]
pub struct Router {
    graph: Graph,
}

#[uniffi::export]
impl Router {
    /// Load a router from the bytes of a pre-built `.graph` file (from APK
    /// assets or the app's cache). Touches no network and no storage.
    #[uniffi::constructor]
    pub fn new(data: Vec<u8>) -> Result<Arc<Self>, RouterError> {
        let graph = Graph::from_bytes(&data).map_err(|_| RouterError::BadGraph)?;
        Ok(Arc::new(Router { graph }))
    }

    /// Route through `waypoints` (at least two, visit order) with `profile`.
    /// Unreachable legs are bridged with straight lines (`fallback = true`),
    /// matching the contract's default behavior.
    pub fn route(&self, waypoints: Vec<Coord>, profile: Profile) -> Result<RouteResult, RouterError> {
        let core_profile = match profile {
            Profile::Car => roughroute_core::Profile::Car,
            Profile::Foot => roughroute_core::Profile::Foot,
        };
        let router = roughroute_core::Router::new(
            &self.graph,
            RouteOptions { profile: core_profile, ..RouteOptions::default() },
        );
        let pts: Vec<[f64; 2]> = waypoints.iter().map(|c| [c.lat, c.lon]).collect();
        let result = router.route(&pts)?;
        Ok(RouteResult {
            line: result.line.iter().map(|&[lat, lon]| Coord { lat, lon }).collect(),
            meters: result.meters,
            fallback: result.fallback,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roughroute_core::geo::deg_to_fixed;
    use roughroute_core::graph::Edge;
    use roughroute_core::profile::ACCESS_ALL;

    /// 3-node path graph 0 — 1 — 2 as `.graph` bytes.
    fn tiny_graph_bytes() -> Vec<u8> {
        let nodes = vec![
            [deg_to_fixed(35.00), deg_to_fixed(33.00)],
            [deg_to_fixed(35.01), deg_to_fixed(33.00)],
            [deg_to_fixed(35.02), deg_to_fixed(33.00)],
        ];
        let offsets = vec![0, 1, 3, 4];
        let mk = |target| Edge { target, length_dm: 11120, access: ACCESS_ALL };
        let edges = vec![mk(1), mk(0), mk(2), mk(1)];
        roughroute_core::Graph::from_parts(nodes, offsets, edges).unwrap().to_bytes()
    }

    #[test]
    fn constructs_and_routes_through_the_ffi_types() {
        let router = Router::new(tiny_graph_bytes()).unwrap();
        let res = router
            .route(
                vec![Coord { lat: 35.0, lon: 33.0 }, Coord { lat: 35.02, lon: 33.0 }],
                Profile::Car,
            )
            .unwrap();
        assert!(!res.fallback);
        assert_eq!(res.line.len(), 3);
        assert!((res.line[1].lat - 35.01).abs() < 1e-9);
        assert!(res.meters > 2000.0);
    }

    #[test]
    fn errors_map_to_the_ffi_enum() {
        assert!(matches!(Router::new(b"junk".to_vec()), Err(RouterError::BadGraph)));
        let router = Router::new(tiny_graph_bytes()).unwrap();
        assert!(matches!(
            router.route(vec![], Profile::Car),
            Err(RouterError::TooFewWaypoints)
        ));
        assert!(matches!(
            router.route(
                vec![Coord { lat: 35.0, lon: 33.0 }, Coord { lat: 40.0, lon: 40.0 }],
                Profile::Foot
            ),
            Err(RouterError::SnapTooFar)
        ));
    }
}
