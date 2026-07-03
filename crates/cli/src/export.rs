//! Route output formats: the neutral JSON contract, GeoJSON, and GPX.
//!
//! Coordinate order is `[lat, lon]` everywhere (docs/DECISIONS.md D9) with
//! exactly one exception, marked loudly below in [`to_geojson`].

use roughroute_core::RouteResult;
use serde::Serialize;

/// The neutral JSON contract response shared with route-spoofer (spec §6.1).
/// Field order matters for readability only; coordinate order is `[lat, lon]`.
#[derive(Serialize)]
struct ContractResponse<'a> {
    line: &'a [[f64; 2]],
    meters: f64,
    fallback: bool,
}

/// Render the spec §6.1 contract JSON: `{ "line": [[lat,lon],…], "meters": …,
/// "fallback": … }`.
pub fn to_contract_json(route: &RouteResult) -> Result<String, serde_json::Error> {
    serde_json::to_string(&ContractResponse {
        line: &route.line,
        meters: route.meters,
        fallback: route.fallback,
    })
}

#[derive(Serialize)]
struct GeoJsonFeature {
    #[serde(rename = "type")]
    kind: &'static str,
    properties: GeoJsonProperties,
    geometry: GeoJsonGeometry,
}

#[derive(Serialize)]
struct GeoJsonProperties {
    meters: f64,
    fallback: bool,
}

#[derive(Serialize)]
struct GeoJsonGeometry {
    #[serde(rename = "type")]
    kind: &'static str,
    coordinates: Vec<[f64; 2]>,
}

/// Render a GeoJSON `Feature` with a `LineString` geometry.
///
/// ⚠️ **This is the ONLY place in the project where coordinates are
/// `[lon, lat]`** — GeoJSON (RFC 7946 §3.1.1) mandates longitude first,
/// opposite to the `[lat, lon]` used by the contract, the Rust API, and every
/// internal structure. The flip happens here and nowhere else.
pub fn to_geojson(route: &RouteResult) -> Result<String, serde_json::Error> {
    // RFC 7946 requires a LineString to have at least two positions;
    // `RouteResult::line` already guarantees ≥2 (a degenerate route is a
    // two-point zero-length line), so no padding is needed here.
    let coordinates: Vec<[f64; 2]> =
        route.line.iter().map(|&[lat, lon]| [lon, lat]).collect(); // ← the flip
    serde_json::to_string(&GeoJsonFeature {
        kind: "Feature",
        properties: GeoJsonProperties { meters: route.meters, fallback: route.fallback },
        geometry: GeoJsonGeometry { kind: "LineString", coordinates },
    })
}

/// Render a GPX 1.1 document with a single track. Coordinates are named
/// attributes (`lat=`/`lon=`), so no order ambiguity exists. Deliberately no
/// timestamps or other run-varying metadata: identical routes must serialize
/// identically (determinism F9).
pub fn to_gpx(route: &RouteResult) -> String {
    let mut out = String::with_capacity(128 + route.line.len() * 48);
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str(
        "<gpx version=\"1.1\" creator=\"roughroute\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n",
    );
    out.push_str("  <trk>\n    <trkseg>\n");
    for [lat, lon] in &route.line {
        out.push_str(&format!("      <trkpt lat=\"{lat}\" lon=\"{lon}\"/>\n"));
    }
    out.push_str("    </trkseg>\n  </trk>\n</gpx>\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> RouteResult {
        RouteResult {
            line: vec![[34.7071, 33.0226], [34.6841, 33.0379]],
            meters: 2143.7,
            fallback: false,
        }
    }

    #[test]
    fn contract_json_is_lat_lon_with_spec_field_order() {
        let json = to_contract_json(&sample()).unwrap();
        assert_eq!(
            json,
            r#"{"line":[[34.7071,33.0226],[34.6841,33.0379]],"meters":2143.7,"fallback":false}"#
        );
    }

    #[test]
    fn geojson_flips_to_lon_lat() {
        let json = to_geojson(&sample()).unwrap();
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v["type"], "Feature");
        assert_eq!(v["geometry"]["type"], "LineString");
        // lon first: 33.0226 comes before 34.7071.
        assert_eq!(v["geometry"]["coordinates"][0][0], 33.0226);
        assert_eq!(v["geometry"]["coordinates"][0][1], 34.7071);
        assert_eq!(v["properties"]["meters"], 2143.7);
        assert_eq!(v["properties"]["fallback"], false);
    }

    #[test]
    fn geojson_degenerate_line_is_a_valid_two_position_linestring() {
        // A degenerate route from the router is a two-point zero-length line;
        // GeoJSON passes it through as a valid (RFC 7946 ≥2) LineString.
        let route =
            RouteResult { line: vec![[34.7, 33.0], [34.7, 33.0]], meters: 0.0, fallback: false };
        let json = to_geojson(&route).unwrap();
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v["geometry"]["coordinates"].as_array().unwrap().len(), 2);
    }

    #[test]
    fn gpx_has_named_lat_lon_attributes_and_no_timestamps() {
        let gpx = to_gpx(&sample());
        assert!(gpx.contains(r#"<trkpt lat="34.7071" lon="33.0226"/>"#));
        assert!(gpx.contains("http://www.topografix.com/GPX/1/1"));
        assert!(!gpx.contains("<time>"));
        // Deterministic: identical input, identical bytes.
        assert_eq!(gpx, to_gpx(&sample()));
    }
}
