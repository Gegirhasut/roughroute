//! Geographic primitives: haversine distance, bearing, and the fixed-point
//! coordinate encoding used by the `.graph` format.

/// Mean Earth radius in meters (IUGG value, the conventional haversine radius).
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Scale of the fixed-point coordinate encoding: degrees × 1e7 → `i32`.
pub const FIXED_POINT_SCALE: f64 = 1e7;

/// Encode degrees as fixed-point 1e7 (`round(deg * 1e7)`), the on-disk node
/// coordinate representation (spec §7.2). ~1.1 cm resolution at the equator.
#[inline]
pub fn deg_to_fixed(deg: f64) -> i32 {
    (deg * FIXED_POINT_SCALE).round() as i32
}

/// Decode a fixed-point 1e7 coordinate back to degrees.
#[inline]
pub fn fixed_to_deg(fixed: i32) -> f64 {
    f64::from(fixed) / FIXED_POINT_SCALE
}

/// Great-circle distance in meters between two `(lat, lon)` points given in
/// degrees, by the haversine formula.
pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();

    let a = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    // `a` can exceed 1.0 by a few ULPs for antipodal points; clamp before sqrt.
    let a = a.clamp(0.0, 1.0);
    2.0 * EARTH_RADIUS_M * a.sqrt().asin()
}

/// Initial great-circle bearing in degrees `[0, 360)` from point 1 to point 2
/// (both `(lat, lon)` in degrees). Returns 0.0 for coincident points.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dlambda = (lon2 - lon1).to_radians();

    let y = dlambda.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlambda.cos();
    if y == 0.0 && x == 0.0 {
        return 0.0;
    }
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Meters per degree of latitude (WGS-84 mean). Used for rough metric sizing
/// of the snapping grid; not for distance measurement.
pub const METERS_PER_DEG_LAT: f64 = 110_574.0;

/// Meters per degree of longitude at the equator (scale by `cos(lat)`).
pub const METERS_PER_DEG_LON_EQUATOR: f64 = 111_320.0;

/// Meters per degree on the haversine sphere of radius [`EARTH_RADIUS_M`]
/// (a latitude degree, and a longitude degree at the equator scaled by
/// `cos(lat)`). Unlike the WGS-84 constants above, this is *consistent with*
/// [`haversine_m`]: a degree measured by that function equals exactly this
/// times the relevant cosine. Use it wherever a distance bound must under- or
/// exactly estimate a haversine distance — e.g. a spatial-index ring
/// lower bound, where the WGS-84 lon value (which slightly over-estimates the
/// sphere) could otherwise stop a search one ring early.
pub const METERS_PER_DEG_SPHERE: f64 =
    2.0 * std::f64::consts::PI * EARTH_RADIUS_M / 360.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_point_round_trips_within_resolution() {
        for &deg in &[0.0, 34.7071, -33.0226, 89.9999999, -180.0, 179.9999999] {
            let back = fixed_to_deg(deg_to_fixed(deg));
            assert!((back - deg).abs() <= 0.5 / FIXED_POINT_SCALE, "{deg} -> {back}");
        }
    }

    #[test]
    fn sphere_constant_matches_haversine_and_is_a_true_bound() {
        // A latitude degree measured by haversine equals METERS_PER_DEG_SPHERE
        // to within rounding — that's the property the ring bound relies on.
        let one_deg = haversine_m(34.0, 33.0, 35.0, 33.0);
        assert!((one_deg - METERS_PER_DEG_SPHERE).abs() < 1e-3, "{one_deg}");
        // And it is a true lower bound vs the WGS-84 lon constant that the
        // ring termination previously used (which over-estimated the sphere).
        const { assert!(METERS_PER_DEG_SPHERE < METERS_PER_DEG_LON_EQUATOR) };
    }

    #[test]
    fn haversine_known_values() {
        // Coincident points.
        assert_eq!(haversine_m(35.0, 33.0, 35.0, 33.0), 0.0);
        // One degree of latitude ≈ 111.2 km on the sphere of radius 6371 km.
        let d = haversine_m(35.0, 33.0, 36.0, 33.0);
        assert!((d - 111_195.0).abs() < 100.0, "got {d}");
        // Limassol -> Nicosia is roughly 63–65 km straight line.
        let d = haversine_m(34.6857, 33.0299, 35.1856, 33.3823);
        assert!((55_000.0..70_000.0).contains(&d), "got {d}");
        // Symmetry.
        assert_eq!(
            haversine_m(34.0, 33.0, 35.0, 34.0),
            haversine_m(35.0, 34.0, 34.0, 33.0)
        );
    }

    #[test]
    fn bearing_cardinal_directions() {
        assert!((bearing_deg(35.0, 33.0, 36.0, 33.0) - 0.0).abs() < 1e-9); // north
        assert!((bearing_deg(35.0, 33.0, 34.0, 33.0) - 180.0).abs() < 1e-9); // south
        assert!((bearing_deg(0.0, 33.0, 0.0, 34.0) - 90.0).abs() < 1e-9); // east on equator
        assert!((bearing_deg(0.0, 34.0, 0.0, 33.0) - 270.0).abs() < 1e-9); // west on equator
        assert_eq!(bearing_deg(35.0, 33.0, 35.0, 33.0), 0.0); // coincident
    }
}
