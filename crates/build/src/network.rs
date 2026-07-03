//! The pure graph-construction layer: in-memory road network → validated
//! `roughroute_core::Graph`.
//!
//! Two stages:
//!
//! 1. **v1 pipeline** (`docs/DECISIONS.md` D1/D3): OSM node-id dedup across
//!    ways, ascending-OSM-id node indexing, per-segment directed edges both
//!    ways, duplicate-edge merge.
//! 2. **M4 degree-2 collapse** (D13): maximal chains of interior nodes
//!    become single edges carrying summed `length_dm` and the swallowed
//!    nodes' coordinates as intermediate geometry (D12), shared between the
//!    two directions.
//!
//! Determinism (D3) is preserved end to end: kept nodes are renumbered in
//! ascending pre-collapse index (= ascending OSM id), chains are discovered
//! and laid out in ascending scan order, and adjacency sorts by
//! `(target, length_dm, geo_off)`.

use std::collections::BTreeMap;

use roughroute_core::geo;
use roughroute_core::graph::{Edge, Graph};

use crate::BuildError;

/// Split threshold for `geo_len: u16`: chains with more intermediate points
/// than this keep a split node as a real graph node. Far above anything real
/// data produces; guards the field width (D12).
const MAX_GEO_PER_EDGE: usize = 60_000;

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
/// surprising inputs (clipped extracts, heavy dedup) and the effect of the
/// M4 collapse are visible.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct BuildStats {
    /// Ways that contributed at least one edge.
    pub ways_used: u64,
    /// Way segments dropped because a node ref was missing from the extract
    /// (clipped geometry) — the rest of the way is kept.
    pub segments_dropped_missing_node: u64,
    /// Duplicate directed edges merged (same source and target; access masks
    /// OR-ed, minimum length kept) before the collapse.
    pub duplicate_edges_merged: u64,
    /// Nodes before the degree-2 collapse (every way node, D1).
    pub nodes_before_collapse: u64,
    /// Directed edges before the degree-2 collapse.
    pub edges_before_collapse: u64,
    /// Interior (degree-2) nodes swallowed into edge geometry.
    pub interior_nodes_collapsed: u64,
    /// Points in the shared geometry pool.
    pub geometry_points: u64,
    /// Chains that left and re-entered the same kept node (P-loops): dropped,
    /// see D13.
    pub loop_chains_dropped: u64,
}

/// Build a routable graph from accepted ways plus a node-id → `[lat, lon]`
/// (degrees) coordinate map. Degree-2 chains are collapsed (D13); the
/// returned graph is in format v2 with intermediate geometry (D12).
///
/// Only nodes that end up on at least one edge become graph nodes; ways with
/// mask 0 or fewer than two resolvable refs contribute nothing. An empty
/// network yields a valid empty graph (spec §10).
pub fn build_graph(
    ways: &[RawWay],
    coords: &BTreeMap<i64, [f64; 2]>,
) -> Result<(Graph, BuildStats), BuildError> {
    build_graph_with_options(ways, coords, true)
}

/// [`build_graph`] with the M4 collapse toggleable. `collapse = false`
/// reproduces the v1 every-way-node topology (in the v2 container format,
/// with empty geometry) — used by tests to prove the collapse does not change
/// returned route shapes, and handy for debugging.
pub fn build_graph_with_options(
    ways: &[RawWay],
    coords: &BTreeMap<i64, [f64; 2]>,
    collapse: bool,
) -> Result<(Graph, BuildStats), BuildError> {
    let mut stats = BuildStats::default();

    // ---- v1 pipeline -----------------------------------------------------

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
                continue; // repeated ref, an OSM data glitch (D1)
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
    //    ascending OSM id (D3: reproducible regardless of input order).
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

    // Reject a region whose longitude span exceeds 180°: it either crosses the
    // antimeridian or is more than half the globe wide. The snapping
    // projection (a local equirectangular frame) is wrong across the ±180°
    // seam, so building such a region would silently misroute near it. Fail
    // loudly instead — full antimeridian support is out of scope (see the
    // "Known limitations" note in the README / docs/DECISIONS.md).
    if let (Some(min_lon), Some(max_lon)) =
        (nodes.iter().map(|n| n[1]).min(), nodes.iter().map(|n| n[1]).max())
    {
        // 180° in fixed-point 1e7.
        if (max_lon as i64 - min_lon as i64) > 1_800_000_000 {
            return Err(BuildError::AntimeridianSpanning);
        }
    }

    // 3. Directed edges, both directions per segment (v1 ignores `oneway`).
    let mut directed: Vec<(u32, u32, u32, u8)> = Vec::with_capacity(segments.len() * 2);
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
        // Clamp to ≥ 1 dm so no edge is free to traverse (D5).
        let length_dm = ((m * 10.0).round().min(f64::from(u32::MAX)) as u32).max(1);
        directed.push((ai, bi, length_dm, access));
        directed.push((bi, ai, length_dm, access));
    }

    // 4. Deterministic order, then merge duplicates: same (source, target)
    //    from overlapping ways collapses to one edge with OR-ed access and
    //    the minimum length (D1). Sorting by length puts the minimum first.
    directed.sort_unstable();
    let mut merged: Vec<(u32, u32, u32, u8)> = Vec::with_capacity(directed.len());
    for (s, t, len, acc) in directed {
        match merged.last_mut() {
            Some((ls, lt, _, lacc)) if *ls == s && *lt == t => {
                *lacc |= acc;
                stats.duplicate_edges_merged += 1;
            }
            _ => merged.push((s, t, len, acc)),
        }
    }
    stats.nodes_before_collapse = ids.len() as u64;
    stats.edges_before_collapse = merged.len() as u64;

    // ---- M4 degree-2 collapse (D13) --------------------------------------

    // Adjacency over the merged edges: per node, (target, length_dm, access),
    // sorted by construction (merged is sorted by (source, target)).
    let n = nodes.len();
    let mut adj: Vec<Vec<(u32, u32, u8)>> = vec![Vec::new(); n];
    for &(s, t, len, acc) in &merged {
        adj[s as usize].push((t, len, acc));
    }

    // Interior (collapsible) node: exactly two edges, to distinct neighbors,
    // with equal access. Everything else stays a real node.
    let mut kept: Vec<bool> = (0..n)
        .map(|i| {
            let e = &adj[i];
            !(collapse
                && e.len() == 2
                && e[0].0 != e[1].0
                && e[0].0 != i as u32
                && e[1].0 != i as u32
                && e[0].2 == e[1].2)
        })
        .collect();

    /// A road segment surviving the collapse: kept endpoints, summed cost,
    /// and the swallowed nodes (pre-collapse indices) in a→b order.
    struct Chain {
        a: u32,
        b: u32,
        length_dm: u64,
        access: u8,
        interior: Vec<u32>,
    }
    let mut consumed = vec![false; n];
    let mut chains: Vec<Chain> = Vec::new();

    // Walk one maximal chain from kept node `a` through interior `first`.
    // Splits over-long chains by promoting the split point to a kept node
    // (D12's geo_len is u16); a split-restart that immediately meets a kept
    // node is discarded — that direct edge is emitted by the direct pass
    // below, which runs after all promotions are known.
    let walk = |a: u32,
                    first: u32,
                    first_len: u32,
                    access: u8,
                    kept: &mut Vec<bool>,
                    consumed: &mut Vec<bool>,
                    chains: &mut Vec<Chain>,
                    stats: &mut BuildStats| {
        let mut chain =
            Chain { a, b: a, length_dm: u64::from(first_len), access, interior: Vec::new() };
        let mut prev = a;
        let mut cur = first;
        loop {
            consumed[cur as usize] = true;
            chain.interior.push(cur);
            if chain.interior.len() >= MAX_GEO_PER_EDGE {
                kept[cur as usize] = true;
                chain.interior.pop();
                chain.b = cur;
                let done = std::mem::replace(
                    &mut chain,
                    Chain { a: cur, b: cur, length_dm: 0, access, interior: Vec::new() },
                );
                chains.push(done);
            }
            let (n1, l1, _) = adj[cur as usize][0];
            let (n2, l2, _) = adj[cur as usize][1];
            let (next, step) = if n1 == prev { (n2, l2) } else { (n1, l1) };
            chain.length_dm += u64::from(step);
            if kept[next as usize] {
                chain.b = next;
                break;
            }
            prev = cur;
            cur = next;
        }
        if chain.a == chain.b {
            // P-loop: leaves and re-enters the same kept node (D13) — or the
            // empty remainder of a split; either way there is nothing to keep.
            if !chain.interior.is_empty() {
                stats.loop_chains_dropped += 1;
            }
            return;
        }
        if chain.interior.is_empty() {
            return; // split remainder that is a plain direct edge (see above)
        }
        // Canonicalize to lower→higher endpoint (split remainders can start
        // at the higher one).
        if chain.a > chain.b {
            std::mem::swap(&mut chain.a, &mut chain.b);
            chain.interior.reverse();
        }
        chains.push(chain);
    };

    // Phase 1: chains anchored at kept nodes, ascending scan → each chain is
    // walked from its lower kept endpoint first (canonical direction, D12).
    for a in 0..n as u32 {
        if !kept[a as usize] {
            continue;
        }
        for &(t, len, acc) in &adj[a as usize] {
            if !kept[t as usize] && !consumed[t as usize] {
                walk(a, t, len, acc, &mut kept, &mut consumed, &mut chains, &mut stats);
            }
        }
    }

    // Phase 2: pure interior cycles — anything interior still unconsumed.
    // Keep the lowest-index member and its lowest neighbor, turning the ring
    // into a direct edge plus one chain (both directions of travel survive).
    for start in 0..n as u32 {
        if kept[start as usize] || consumed[start as usize] {
            continue;
        }
        kept[start as usize] = true;
        let (n1, l1, acc) = adj[start as usize][0];
        let n2 = adj[start as usize][1].0;
        kept[n1.min(n2) as usize] = true;
        // Walk the long way around (via the *other* neighbor when the lowest
        // is now kept and adjacent — its direct edge comes from the pass
        // below). Both of start's edges are tried; kept/consumed guards keep
        // this correct in every ring shape.
        for &(t, len, a2) in &[(n1, l1, acc), adj[start as usize][1]] {
            if !kept[t as usize] && !consumed[t as usize] {
                walk(start, t, len, a2, &mut kept, &mut consumed, &mut chains, &mut stats);
            }
        }
    }

    // Phase 3: direct kept–kept edges, emitted from the merged edge list once
    // per segment. Running after all promotions guarantees no double emit
    // around split points.
    for &(s, t, len, acc) in &merged {
        if s < t && kept[s as usize] && kept[t as usize] {
            chains.push(Chain { a: s, b: t, length_dm: u64::from(len), access: acc, interior: Vec::new() });
        }
    }

    // ---- Renumber, lay out geometry, emit CSR ----------------------------

    // Kept nodes renumbered by ascending old index — preserves the
    // ascending-OSM-id order that D3's determinism rests on.
    let mut new_index = vec![u32::MAX; n];
    let mut new_nodes: Vec<[i32; 2]> = Vec::new();
    for i in 0..n {
        if kept[i] {
            new_index[i] = new_nodes.len() as u32;
            new_nodes.push(nodes[i]);
        } else {
            stats.interior_nodes_collapsed += 1;
        }
    }

    // Deterministic chain order before pool layout: by renumbered endpoints,
    // then length, then shape.
    chains.sort_by_cached_key(|c| {
        (
            new_index[c.a as usize],
            new_index[c.b as usize],
            c.length_dm,
            c.interior.iter().map(|&i| nodes[i as usize]).collect::<Vec<_>>(),
        )
    });

    let mut geometry: Vec<[i32; 2]> = Vec::new();
    let mut directed_v2: Vec<(u32, Edge)> = Vec::with_capacity(chains.len() * 2);
    for c in &chains {
        let (na, nb) = (new_index[c.a as usize], new_index[c.b as usize]);
        let length_dm = u32::try_from(c.length_dm).unwrap_or(u32::MAX).max(1);
        let (geo_off, geo_len) = if c.interior.is_empty() {
            (0u32, 0u16)
        } else {
            let off = geometry.len() as u32;
            geometry.extend(c.interior.iter().map(|&i| nodes[i as usize]));
            // interior length is capped at MAX_GEO_PER_EDGE < u16::MAX.
            (off, c.interior.len() as u16)
        };
        let mk = |target: u32, reversed: bool| Edge {
            target,
            length_dm,
            geo_off,
            geo_len,
            // Canonical form: geometry-less edges never set `reversed` (D12).
            reversed: reversed && geo_len > 0,
            access: c.access,
        };
        directed_v2.push((na, mk(nb, false)));
        directed_v2.push((nb, mk(na, true)));
    }
    if directed_v2.len() > u32::MAX as usize {
        return Err(BuildError::TooLarge);
    }
    stats.geometry_points = geometry.len() as u64;

    // CSR. geo_off is unique per chain-with-geometry, making the order total
    // even for parallel edges (D13).
    directed_v2
        .sort_unstable_by_key(|&(s, e)| (s, e.target, e.length_dm, e.geo_off, e.reversed));
    let mut offsets = vec![0u32; new_nodes.len() + 1];
    for &(s, _) in &directed_v2 {
        offsets[s as usize + 1] += 1;
    }
    for i in 1..offsets.len() {
        offsets[i] += offsets[i - 1];
    }
    let edges: Vec<Edge> = directed_v2.into_iter().map(|(_, e)| e).collect();

    let graph = Graph::from_parts(new_nodes, offsets, edges, geometry)?;
    Ok((graph, stats))
}

#[cfg(test)]
mod tests {
    use super::*;
    use roughroute_core::profile::{ACCESS_ALL, ACCESS_CAR};
    use roughroute_core::{Profile, RouteOptions, Router};

    fn coords(entries: &[(i64, f64, f64)]) -> BTreeMap<i64, [f64; 2]> {
        entries.iter().map(|&(id, lat, lon)| (id, [lat, lon])).collect()
    }

    #[test]
    fn antimeridian_spanning_region_is_rejected() {
        // A way from +179° to -179° spans ~358° of longitude: refuse it
        // rather than build a graph the snapping projection misroutes near.
        let ways = vec![RawWay { node_ids: vec![10, 20], access: ACCESS_ALL }];
        let coords = coords(&[(10, 51.0, 179.0), (20, 51.0, -179.0)]);
        assert!(matches!(build_graph(&ways, &coords), Err(BuildError::AntimeridianSpanning)));
    }

    #[test]
    fn wide_but_sub_180_region_is_accepted() {
        // Contiguous-US-ish span (~59°) is well under the 180° limit.
        let ways = vec![RawWay { node_ids: vec![10, 20], access: ACCESS_ALL }];
        let coords = coords(&[(10, 40.0, -125.0), (20, 40.0, -66.0)]);
        assert!(build_graph(&ways, &coords).is_ok());
    }

    #[test]
    fn shared_node_ids_connect_ways_at_junctions() {
        // Way 1: 10-20-30, Way 2: 40-20-50 — they cross at OSM node 20.
        // Every node is a junction or dead-end here, so nothing collapses.
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
    fn degree2_chain_collapses_into_one_edge_with_geometry() {
        // 10 - 20 - 30 - 40: 20 and 30 are interior.
        let ways = vec![RawWay { node_ids: vec![10, 20, 30, 40], access: ACCESS_ALL }];
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.000, 33.010),
            (30, 35.000, 33.020),
            (40, 35.000, 33.030),
        ]);
        let (g, stats) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.node_count(), 2); // 10 and 40 kept
        assert_eq!(g.edge_count(), 2); // one collapsed segment, both directions
        assert_eq!(g.geometry_point_count(), 2); // 20, 30 as shape
        assert_eq!(stats.interior_nodes_collapsed, 2);
        assert_eq!(stats.nodes_before_collapse, 4);
        assert_eq!(stats.edges_before_collapse, 6);

        // Forward edge (0 -> 1) carries the canonical geometry; the reverse
        // is flagged. Length is the sum of the three original segments.
        let fwd = &g.edges_from(0)[0];
        let bwd = &g.edges_from(1)[0];
        assert!(!fwd.reversed && bwd.reversed);
        assert_eq!(fwd.geo_len, 2);
        assert_eq!(g.edge_geometry(fwd), g.edge_geometry(bwd));
        // ~0.01° lon at 35°N ≈ 910.9 m (haversine) each; sum ≈ 2732.6 m.
        assert!((27_320..=27_335).contains(&fwd.length_dm), "{}", fwd.length_dm);

        // Routing across it reproduces the full original shape.
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[[35.0, 33.0], [35.0, 33.03]]).unwrap();
        assert_eq!(res.line.len(), 4);
        assert!((res.line[1][1] - 33.010).abs() < 1e-9);
        assert!((res.line[2][1] - 33.020).abs() < 1e-9);
    }

    #[test]
    fn access_change_breaks_a_chain() {
        // 10-20 drivable, 20-30 foot-only: 20 has degree 2 but differing
        // access masks, so it must stay a real node.
        let ways = vec![
            RawWay { node_ids: vec![10, 20], access: ACCESS_ALL },
            RawWay { node_ids: vec![20, 30], access: roughroute_core::profile::ACCESS_FOOT },
        ];
        let coords = coords(&[(10, 35.0, 33.0), (20, 35.0, 33.01), (30, 35.0, 33.02)]);
        let (g, stats) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.node_count(), 3);
        assert_eq!(stats.interior_nodes_collapsed, 0);
    }

    #[test]
    fn parallel_chains_between_same_junctions_both_survive() {
        // Two roads from 10 to 40 (north via 20, south via 30 — longer),
        // plus stems 5-10 and 40-45 so 10 and 40 are true junctions
        // (degree 3), not members of a pure cycle.
        let ways = vec![
            RawWay { node_ids: vec![10, 20, 40], access: ACCESS_ALL },
            RawWay { node_ids: vec![10, 30, 40], access: ACCESS_ALL },
            RawWay { node_ids: vec![5, 10], access: ACCESS_ALL },
            RawWay { node_ids: vec![40, 45], access: ACCESS_ALL },
        ];
        let coords = coords(&[
            (5, 35.000, 32.990),
            (10, 35.000, 33.000),
            (20, 35.002, 33.010), // slight north bow
            (40, 35.000, 33.020),
            (30, 34.994, 33.010), // bigger south bow
            (45, 35.000, 33.030),
        ]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.node_count(), 4); // 5, 10, 40, 45
        // Two parallel collapsed segments + two stems, × two directions.
        assert_eq!(g.edge_count(), 8);
        assert_eq!(g.geometry_point_count(), 2);
        // A* must take the shorter (north) branch and reproduce its shape.
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[[35.0, 33.0], [35.0, 33.02]]).unwrap();
        assert_eq!(res.line.len(), 3);
        assert!((res.line[1][0] - 35.002).abs() < 1e-9, "took the wrong branch");
    }

    #[test]
    fn p_loop_is_dropped_and_counted() {
        // Stem 10-20, then a balloon 20-30-40-20.
        let ways = vec![RawWay { node_ids: vec![10, 20, 30, 40, 20], access: ACCESS_CAR }];
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.000, 33.010),
            (30, 35.002, 33.015),
            (40, 34.998, 33.015),
        ]);
        let (g, stats) = build_graph(&ways, &coords).unwrap();
        assert_eq!(stats.loop_chains_dropped, 1);
        // 10 (dead-end) and 20 (junction: degree 3) survive; the loop is gone.
        assert_eq!(g.node_count(), 2);
        assert_eq!(g.edge_count(), 2);
    }

    #[test]
    fn pure_cycle_keeps_two_nodes_as_parallel_edges() {
        // An isolated ring 10-20-30-40-10: all degree 2.
        let ways = vec![RawWay { node_ids: vec![10, 20, 30, 40, 10], access: ACCESS_CAR }];
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.000, 33.010),
            (30, 35.005, 33.010),
            (40, 35.005, 33.000),
        ]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        // Lowest member (10) + its lowest neighbor (20) kept; ring becomes
        // two parallel edges between them (one direct, one via 30 and 40).
        assert_eq!(g.node_count(), 2);
        assert_eq!(g.edge_count(), 4);
        assert_eq!(g.geometry_point_count(), 2);
        // Both arcs are routable; A* picks the short (direct) one.
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[[35.0, 33.0], [35.0, 33.01]]).unwrap();
        assert!(!res.fallback);
        assert_eq!(res.line.len(), 2);
    }

    #[test]
    fn node_indexing_is_ascending_osm_id_rank() {
        // Ids deliberately out of order in the way; middle node collapses,
        // endpoints keep ascending-id order.
        let ways = vec![RawWay { node_ids: vec![300, 100, 200], access: ACCESS_CAR }];
        let coords = coords(&[(100, 35.0, 33.1), (200, 35.0, 33.2), (300, 35.0, 33.0)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        // Kept: 200 and 300 (100 is interior on the path 300-100-200).
        assert_eq!(g.node_count(), 2);
        assert_eq!(g.node_latlon(0), [35.0, 33.2]); // id 200
        assert_eq!(g.node_latlon(1), [35.0, 33.0]); // id 300
    }

    #[test]
    fn missing_nodes_drop_segments_not_ways() {
        // Node 20 missing: way 10-20-30-40 keeps only segment 30-40.
        let ways = vec![RawWay { node_ids: vec![10, 20, 30, 40], access: ACCESS_CAR }];
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
            RawWay { node_ids: vec![10, 20], access: ACCESS_CAR },
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
            RawWay { node_ids: vec![10, 10, 20], access: ACCESS_CAR },
            RawWay { node_ids: vec![20, 30], access: 0 },
        ];
        let coords = coords(&[(10, 35.0, 33.0), (20, 35.0, 33.1), (30, 35.0, 33.2)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert_eq!(g.node_count(), 2); // 10 and 20; 30 only on the dropped way
        assert_eq!(g.edge_count(), 2);
    }

    #[test]
    fn near_coincident_nodes_get_minimum_length() {
        // Two distinct ids ~1 mm apart: length clamps to 1 dm (D5).
        let ways = vec![RawWay { node_ids: vec![10, 20], access: ACCESS_CAR }];
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
            (50, 35.000, 33.030),
        ]);
        let a = vec![
            RawWay { node_ids: vec![10, 20, 30, 50], access: ACCESS_ALL },
            RawWay { node_ids: vec![40, 20], access: ACCESS_CAR },
        ];
        let b: Vec<RawWay> = a.iter().rev().cloned().collect();
        let (ga, _) = build_graph(&a, &coords).unwrap();
        let (gb, _) = build_graph(&b, &coords).unwrap();
        assert_eq!(ga.to_bytes(), gb.to_bytes());
    }

    #[test]
    fn collapsed_and_uncollapsed_graphs_route_identically() {
        // The key M4 regression guard at unit level: a wiggly road with a
        // junction in the middle; waypoints at junction coordinates.
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.001, 33.005),
            (30, 35.000, 33.010), // junction (side spur 30-60)
            (40, 34.999, 33.015),
            (50, 35.000, 33.020),
            (60, 34.995, 33.010),
        ]);
        let ways = vec![
            RawWay { node_ids: vec![10, 20, 30, 40, 50], access: ACCESS_ALL },
            RawWay { node_ids: vec![30, 60], access: ACCESS_ALL },
        ];
        let (collapsed, _) = build_graph_with_options(&ways, &coords, true).unwrap();
        let (dense, _) = build_graph_with_options(&ways, &coords, false).unwrap();
        assert!(collapsed.node_count() < dense.node_count());

        let route = |g: &Graph, from: [f64; 2], to: [f64; 2]| {
            Router::new(
                g,
                RouteOptions { profile: Profile::Foot, ..RouteOptions::default() },
            )
            .route(&[from, to])
            .unwrap()
        };
        for (from, to) in [
            ([35.0, 33.0], [35.0, 33.02]),
            ([35.0, 33.02], [35.0, 33.0]),
            ([34.995, 33.01], [35.0, 33.02]),
        ] {
            let c = route(&collapsed, from, to);
            let d = route(&dense, from, to);
            assert_eq!(c.line, d.line, "shape changed for {from:?} -> {to:?}");
            assert_eq!(c.meters, d.meters);
            assert!(!c.fallback && !d.fallback);
        }
    }
}
