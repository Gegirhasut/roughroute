//! Thin `.osm.pbf` front-end: extract the road network in the compact
//! in-memory form consumed by [`crate::network::build_graph_compact`].
//!
//! Two single-threaded passes over the file (element order within a PBF is
//! not guaranteed to be nodes-before-ways, and two passes keep memory at
//! "accepted ways + needed nodes" instead of "every node in the extract"):
//!
//! 1. ways — keep those with a routable `highway` tag, collect referenced
//!    node ids;
//! 2. nodes — coordinates for exactly that id set.
//!
//! Since D23 the output is a [`CompactNetwork`]: way refs go into one flat
//! array (not one heap `Vec` per way), the needed-id set is a sorted array
//! probed by binary search (not a `BTreeSet`), and coordinates land in a
//! parallel array already quantized to fixed-point — applying the same
//! `deg_to_fixed` to the same `f64` the old `BTreeMap<i64, [f64; 2]>` path
//! fed downstream, so every derived value (and the output bytes) is
//! unchanged; only the container is denser.

use std::path::Path;

use osmpbf::{Element, ElementReader};

use roughroute_core::geo;

use crate::network::CompactNetwork;
use crate::tags::access_for_highway;
use crate::BuildError;

/// Read the road network (accepted ways + coordinates of their nodes) from a
/// `.osm.pbf` file.
///
/// Ways are filtered by `highway` tag through [`access_for_highway`]; ways
/// with mask 0 or fewer than two refs are dropped here. Missing coordinates
/// (clipped extracts) are *not* an error —
/// [`crate::network::build_graph_compact`] drops the affected segments and
/// counts them.
pub fn read_road_network(path: &Path) -> Result<CompactNetwork, BuildError> {
    let to_err = |e: osmpbf::Error| BuildError::Pbf(e.to_string());

    // Pass 1: ways.
    let mut net = CompactNetwork::new();
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
                net.push_way(way.refs(), access);
            }
        })
        .map_err(to_err)?;

    // The referenced ids, sorted and deduped: the binary-search target for
    // pass 2 and (filtered to the ids actually found) the network's node
    // table.
    let mut needed: Vec<i64> = net.way_refs.clone();
    needed.sort_unstable();
    needed.dedup();

    // Pass 2: coordinates for the referenced ids only, quantized at read.
    // A repeated node id keeps the last occurrence, as map insertion did.
    let mut coords: Vec<[i32; 2]> = vec![[0, 0]; needed.len()];
    let mut found: Vec<bool> = vec![false; needed.len()];
    let reader = ElementReader::from_path(path).map_err(to_err)?;
    reader
        .for_each(|element| {
            let (id, lat, lon) = match element {
                Element::Node(node) => (node.id(), node.lat(), node.lon()),
                Element::DenseNode(node) => (node.id(), node.lat(), node.lon()),
                _ => return,
            };
            if let Ok(pos) = needed.binary_search(&id) {
                coords[pos] = [geo::deg_to_fixed(lat), geo::deg_to_fixed(lon)];
                found[pos] = true;
            }
        })
        .map_err(to_err)?;

    // Compact to the ids actually present (clipped extracts leave gaps), in
    // place — the sorted order is preserved, satisfying the
    // [`CompactNetwork`] node-table invariant.
    let mut write = 0usize;
    for read in 0..needed.len() {
        if found[read] {
            needed[write] = needed[read];
            coords[write] = coords[read];
            write += 1;
        }
    }
    needed.truncate(write);
    coords.truncate(write);
    net.node_ids = needed;
    net.node_coords = coords;
    Ok(net)
}
