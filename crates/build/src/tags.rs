//! The `highway` tag → profile access mask table.
//!
//! Deliberately permissive (`track`, `service` are drivable) per the spec's
//! "roughly correct, not legal" stance; the table is reviewed in milestone M3
//! (spec §14.2). Full rationale: `docs/DECISIONS.md` D7.

use roughroute_core::profile::{ACCESS_CAR, ACCESS_FOOT};

/// Access mask for a way with the given `highway` tag value. `0` means the
/// way takes part in no profile and is dropped entirely.
pub fn access_for_highway(value: &str) -> u8 {
    match value {
        // Car-only: pedestrians are banned from motorways.
        "motorway" | "motorway_link" => ACCESS_CAR,

        // The shared road network: drivable and walkable.
        "trunk" | "trunk_link" | "primary" | "primary_link" | "secondary" | "secondary_link"
        | "tertiary" | "tertiary_link" | "unclassified" | "residential" | "living_street"
        | "service" | "track" => ACCESS_CAR | ACCESS_FOOT,

        // Foot-only ways and paths.
        "footway" | "path" | "pedestrian" | "steps" | "bridleway" | "cycleway" => ACCESS_FOOT,

        // construction, proposed, raceway, bus_guideway, corridor, …
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn representative_values() {
        assert_eq!(access_for_highway("motorway"), ACCESS_CAR);
        assert_eq!(access_for_highway("residential"), ACCESS_CAR | ACCESS_FOOT);
        assert_eq!(access_for_highway("footway"), ACCESS_FOOT);
        assert_eq!(access_for_highway("steps"), ACCESS_FOOT);
        assert_eq!(access_for_highway("proposed"), 0);
        assert_eq!(access_for_highway(""), 0);
        assert_eq!(access_for_highway("Residential"), 0); // OSM values are lowercase
    }
}
