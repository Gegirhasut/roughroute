//! Thin `.osm.pbf` front-end: extract the road network in the in-memory form
//! consumed by [`crate::network::build_graph`].
//!
//! Two single-threaded passes over the file (element order within a PBF is
//! not guaranteed to be nodes-before-ways, and two passes keep memory at
//! "accepted ways + needed nodes" instead of "every node in the extract"):
//!
//! 1. ways — keep those with a routable `highway` tag, collect referenced
//!    node ids;
//! 2. nodes — coordinates for exactly that id set.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use osmpbf::{Element, ElementReader};

use crate::network::RawWay;
use crate::tags::access_for_highway;
use crate::BuildError;

/// The in-memory road network extracted from a `.osm.pbf`: accepted ways plus
/// the `[lat, lon]` (degrees) coordinates of every node they reference.
pub type RoadNetwork = (Vec<RawWay>, BTreeMap<i64, [f64; 2]>);

/// Read the road network (accepted ways + coordinates of their nodes) from a
/// `.osm.pbf` file.
///
/// Ways are filtered by `highway` tag through [`access_for_highway`]; ways
/// with mask 0 or fewer than two refs are dropped here. Missing coordinates
/// (clipped extracts) are *not* an error — [`crate::network::build_graph`]
/// drops the affected segments and counts them.
pub fn read_road_network(path: &Path) -> Result<RoadNetwork, BuildError> {
    let to_err = |e: osmpbf::Error| BuildError::Pbf(e.to_string());

    // Pass 1: ways.
    let mut ways: Vec<RawWay> = Vec::new();
    let reader = ElementReader::from_path(path).map_err(to_err)?;
    reader
        .for_each(|element| {
            if let Element::Way(way) = element {
                let access = way
                    .tags()
                    .find(|(k, _)| *k == "highway")
                    .map(|(_, v)| access_for_highway(v))
                    .unwrap_or(0);
                if access == 0 {
                    return;
                }
                let node_ids: Vec<i64> = way.refs().collect();
                if node_ids.len() >= 2 {
                    ways.push(RawWay { node_ids, access });
                }
            }
        })
        .map_err(to_err)?;

    // Pass 2: coordinates for the referenced ids only.
    let needed: BTreeSet<i64> = ways.iter().flat_map(|w| w.node_ids.iter().copied()).collect();
    let mut coords: BTreeMap<i64, [f64; 2]> = BTreeMap::new();
    let reader = ElementReader::from_path(path).map_err(to_err)?;
    reader
        .for_each(|element| match element {
            Element::Node(node) if needed.contains(&node.id()) => {
                coords.insert(node.id(), [node.lat(), node.lon()]);
            }
            Element::DenseNode(node) if needed.contains(&node.id()) => {
                coords.insert(node.id(), [node.lat(), node.lon()]);
            }
            _ => {}
        })
        .map_err(to_err)?;

    Ok((ways, coords))
}
