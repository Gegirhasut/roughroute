//! The pure graph-construction layer: in-memory road network → validated
//! `roughroute_core::Graph`.
//!
//! Junction correctness hinges on OSM node-id dedup across ways: one global
//! `osm id → node index` map means two ways sharing a node id connect at
//! that node. Determinism: node index = rank of the OSM id in ascending
//! order; adjacency sorted by `(target, length_dm, access)`; no hash-order
//! or timestamps anywhere.

use std::collections::BTreeMap;

use roughroute_core::geo;
use roughroute_core::graph::{Edge, Graph};

use crate::BuildError;

/// One accepted OSM way: its ordered node refs and the profile access mask
/// derived from its `highway` tag (see [`crate::tags`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawWay {
    /// Ordered OSM node ids along the way.
    pub node_ids: Vec<i64>,
    /// Profile bitmask (`bit0 = car`, `bit1 = foot`); ways with mask 0 are
    /// dropped by the caller.
    pub access: u8,
}

/// Counters describing what [`build_graph`] did — printed by the CLI so
/// surprising inputs (clipped extracts, heavy dedup) are visible.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct BuildStats {
    /// Ways that contributed at least one edge.
    pub ways_used: u64,
    /// Way segments dropped because a node ref was missing from the extract
    /// (clipped geometry) — the rest of the way is kept.
    pub segments_dropped_missing_node: u64,
    /// Duplicate directed edges merged (same source and target; access masks
    /// OR-ed, minimum length kept).
    pub duplicate_edges_merged: u64,
}

/// Build a routable graph from accepted ways plus a node-id → `[lat, lon]`
/// (degrees) coordinate map.
///
/// Only nodes that end up on at least one edge become graph nodes; ways with
/// mask 0 or fewer than two resolvable refs contribute nothing. An empty
/// network yields a valid empty graph.
pub fn build_graph(
    ways: &[RawWay],
    coords: &BTreeMap<i64, [f64; 2]>,
) -> Result<(Graph, BuildStats), BuildError> {
    let mut stats = BuildStats::default();

    // 1. Resolve ways into segments over OSM ids: consecutive distinct-id
    //    pairs whose both endpoints have coordinates.
    let mut segments: Vec<(i64, i64, u8)> = Vec::new();
    for way in ways {
        if way.access == 0 || way.node_ids.len() < 2 {
            continue;
        }
        let mut contributed = false;
        for pair in way.node_ids.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a == b {
                continue; // repeated ref, an OSM data glitch
            }
            if !coords.contains_key(&a) || !coords.contains_key(&b) {
                stats.segments_dropped_missing_node += 1;
                continue;
            }
            segments.push((a, b, way.access));
            contributed = true;
        }
        if contributed {
            stats.ways_used += 1;
        }
    }

    // 2. Node universe = ids appearing in at least one segment, indexed by
    //    ascending OSM id (reproducible regardless of input order).
    let mut ids: Vec<i64> = segments.iter().flat_map(|&(a, b, _)| [a, b]).collect();
    ids.sort_unstable();
    ids.dedup();
    if ids.len() > (u32::MAX - 1) as usize {
        return Err(BuildError::TooLarge);
    }
    let index_of = |id: i64| -> u32 {
        // ids is sorted and contains every segment endpoint by construction.
        match ids.binary_search(&id) {
            Ok(i) => i as u32,
            Err(_) => unreachable!("segment endpoint missing from id table"),
        }
    };

    // Coordinates are quantized to the on-disk fixed-point representation
    // *before* measuring lengths, so lengths always agree with the geometry a
    // reader of the .graph file sees.
    let nodes: Vec<[i32; 2]> = ids
        .iter()
        .map(|id| {
            let [lat, lon] = coords[id];
            [geo::deg_to_fixed(lat), geo::deg_to_fixed(lon)]
        })
        .collect();

    // 3. Directed edges, both directions per segment (v1 ignores `oneway`).
    let mut directed: Vec<(u32, Edge)> = Vec::with_capacity(segments.len() * 2);
    for &(a, b, access) in &segments {
        let (ai, bi) = (index_of(a), index_of(b));
        let [alat, alon] = nodes[ai as usize];
        let [blat, blon] = nodes[bi as usize];
        let m = geo::haversine_m(
            geo::fixed_to_deg(alat),
            geo::fixed_to_deg(alon),
            geo::fixed_to_deg(blat),
            geo::fixed_to_deg(blon),
        );
        // Clamp to ≥ 1 dm so no edge is free to traverse.
        let length_dm = ((m * 10.0).round().min(f64::from(u32::MAX)) as u32).max(1);
        directed.push((ai, Edge { target: bi, length_dm, access }));
        directed.push((bi, Edge { target: ai, length_dm, access }));
    }

    // 4. Deterministic order, then merge duplicates: same (source, target)
    //    from overlapping ways collapses to one edge with OR-ed access and
    //    the minimum length. Sorting by length puts the minimum first.
    directed.sort_by_key(|(s, e)| (*s, e.target, e.length_dm, e.access));
    let mut merged: Vec<(u32, Edge)> = Vec::with_capacity(directed.len());
    for (s, e) in directed {
        match merged.last_mut() {
            Some((ls, le)) if *ls == s && le.target == e.target => {
                le.access |= e.access;
                stats.duplicate_edges_merged += 1;
            }
            _ => merged.push((s, e)),
        }
    }

    // 5. CSR offsets.
    let mut offsets = vec![0u32; ids.len() + 1];
    for &(s, _) in &merged {
        offsets[s as usize + 1] += 1;
    }
    for i in 1..offsets.len() {
        offsets[i] += offsets[i - 1];
    }
    let edges: Vec<Edge> = merged.into_iter().map(|(_, e)| e).collect();

    let graph = Graph::from_parts(nodes, offsets, edges)?;
    Ok((graph, stats))
}

#[cfg(test)]
mod tests {
    use super::*;
    use roughroute_core::profile::ACCESS_ALL;

    fn coords(entries: &[(i64, f64, f64)]) -> BTreeMap<i64, [f64; 2]> {
        entries.iter().map(|&(id, lat, lon)| (id, [lat, lon])).collect()
    }

    #[test]
    fn shared_node_ids_connect_ways_at_junctions() {
        // Way 1: 10-20-30, Way 2: 40-20-50 — they cross at OSM node 20.
        let ways = vec![
            RawWay { node_ids: vec![10, 20, 30], access: ACCESS_ALL },
            RawWay { node_ids: vec![40, 20, 50], access: ACCESS_ALL },
        ];
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.000, 33.010),
            (30, 35.000, 33.020),
            (40, 34.990, 33.010),
            (50, 35.010, 33.010),
        ]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.node_count(), 5);
        // Node index of OSM id 20 is 1 (ascending id rank), and it must have
        // degree 4: the junction connects both ways.
        assert_eq!(g.edges_from(1).len(), 4);
    }

    #[test]
    fn node_indexing_is_ascending_osm_id_rank() {
        // Ids deliberately out of order in the way.
        let ways = vec![RawWay { node_ids: vec![300, 100, 200], access: ACCESS_ALL }];
        let coords = coords(&[(100, 35.0, 33.0), (200, 35.0, 33.1), (300, 35.0, 33.2)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        // index 0 -> id 100, 1 -> id 200, 2 -> id 300.
        assert_eq!(g.node_latlon(0), [35.0, 33.0]);
        assert_eq!(g.node_latlon(1), [35.0, 33.1]);
        assert_eq!(g.node_latlon(2), [35.0, 33.2]);
    }

    #[test]
    fn missing_nodes_drop_segments_not_ways() {
        // Node 20 missing: way 10-20-30 keeps neither segment, but the
        // separate segment 30-40 survives.
        let ways = vec![RawWay { node_ids: vec![10, 20, 30, 40], access: ACCESS_ALL }];
        let coords = coords(&[(10, 35.0, 33.0), (30, 35.0, 33.2), (40, 35.0, 33.3)]);
        let (g, stats) = build_graph(&ways, &coords).unwrap();
        assert_eq!(stats.segments_dropped_missing_node, 2);
        // Only nodes 30 and 40 carry an edge; node 10 is edge-less and dropped.
        assert_eq!(g.node_count(), 2);
        assert_eq!(g.edge_count(), 2);
    }

    #[test]
    fn duplicate_edges_merge_with_or_access() {
        // The same physical segment tagged car-only in one way and foot-only
        // in another.
        let ways = vec![
            RawWay { node_ids: vec![10, 20], access: roughroute_core::profile::ACCESS_CAR },
            RawWay { node_ids: vec![10, 20], access: roughroute_core::profile::ACCESS_FOOT },
        ];
        let coords = coords(&[(10, 35.0, 33.0), (20, 35.0, 33.1)]);
        let (g, stats) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.edge_count(), 2); // one per direction
        assert_eq!(g.edges_from(0)[0].access, ACCESS_ALL);
        assert_eq!(stats.duplicate_edges_merged, 2);
    }

    #[test]
    fn repeated_refs_and_zero_access_ways_are_skipped() {
        let ways = vec![
            RawWay { node_ids: vec![10, 10, 20], access: roughroute_core::profile::ACCESS_CAR },
            RawWay { node_ids: vec![20, 30], access: 0 },
        ];
        let coords = coords(&[(10, 35.0, 33.0), (20, 35.0, 33.1), (30, 35.0, 33.2)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.node_count(), 2); // 10 and 20; 30 only on the dropped way
        assert_eq!(g.edge_count(), 2);
    }

    #[test]
    fn near_coincident_nodes_get_minimum_length() {
        // Two distinct ids ~1 mm apart: length clamps to 1 dm.
        let ways =
            vec![RawWay { node_ids: vec![10, 20], access: roughroute_core::profile::ACCESS_CAR }];
        let coords = coords(&[(10, 35.0, 33.0), (20, 35.00000001, 33.0)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.edges_from(0)[0].length_dm, 1);
    }

    #[test]
    fn empty_network_builds_empty_graph() {
        let (g, stats) = build_graph(&[], &BTreeMap::new()).unwrap();
        assert_eq!(g.node_count(), 0);
        assert_eq!(g.edge_count(), 0);
        assert_eq!(stats, BuildStats::default());
    }

    #[test]
    fn build_is_deterministic_across_way_order() {
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.000, 33.010),
            (30, 35.000, 33.020),
            (40, 34.990, 33.010),
        ]);
        let a = vec![
            RawWay { node_ids: vec![10, 20, 30], access: ACCESS_ALL },
            RawWay { node_ids: vec![40, 20], access: roughroute_core::profile::ACCESS_CAR },
        ];
        let b: Vec<RawWay> = a.iter().rev().cloned().collect();
        let (ga, _) = build_graph(&a, &coords).unwrap();
        let (gb, _) = build_graph(&b, &coords).unwrap();
        assert_eq!(ga.to_bytes(), gb.to_bytes());
    }
}
