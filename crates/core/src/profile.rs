//! Routing profiles and the edge access bitmask they filter on.

/// Bit set on an edge's `access` byte when cars may use it (spec §7.3).
pub const ACCESS_CAR: u8 = 1 << 0;
/// Bit set on an edge's `access` byte when pedestrians may use it (spec §7.3).
pub const ACCESS_FOOT: u8 = 1 << 1;
/// All access bits defined by format v1; anything else must be zero.
pub const ACCESS_ALL: u8 = ACCESS_CAR | ACCESS_FOOT;

/// A routing profile. Profiles differ only by which edges they may traverse
/// (the graph stores a per-edge access bitmask); costs are identical.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Profile {
    /// Motor vehicle: uses roads drivable by car (including `motorway`).
    Car,
    /// Pedestrian: uses walkable roads and paths (excluding `motorway`).
    Foot,
}

impl Profile {
    /// The access bit this profile requires on an edge.
    #[inline]
    pub fn mask(self) -> u8 {
        match self {
            Profile::Car => ACCESS_CAR,
            Profile::Foot => ACCESS_FOOT,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn masks_are_distinct_bits() {
        assert_eq!(Profile::Car.mask(), 0b01);
        assert_eq!(Profile::Foot.mask(), 0b10);
        assert_eq!(Profile::Car.mask() & Profile::Foot.mask(), 0);
    }
}
