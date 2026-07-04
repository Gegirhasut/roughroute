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
//!
//! **Memory (D23).** There is exactly one copy of this algorithm —
//! [`build_graph_compact_with_options`], operating on a [`CompactNetwork`]
//! (flat way-ref array + sorted parallel id/coordinate arrays + the sorted
//! merged edge list reused in place as CSR adjacency), with large transients
//! freed as soon as the next stage no longer needs them. The original
//! `(&[RawWay], &BTreeMap)` entry points are thin adapters into it, so peak
//! RSS stops scaling with per-way/per-node heap allocations while the
//! computed values — and therefore the output bytes — stay identical.

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
    /// Antimeridian seam stitches (D25): nodes merged into a coincident
    /// partner at exactly ±180° longitude. OSM ways cannot cross the
    /// antimeridian, so mappers split roads there with two coincident end
    /// nodes (distinct ids, one per side); id-based dedup (D1) cannot join
    /// them, so they are merged by exact coordinate — without this the
    /// graph falls apart precisely at the seam (verified on Fiji: 4 such
    /// pairs). Always 0 for non-crossing regions.
    pub seam_nodes_merged: u64,
}

/// The compact in-memory road network (`docs/DECISIONS.md` D23): the same
/// information as the `(&[RawWay], &BTreeMap<i64, [f64; 2]>)` pair the
/// original entry points take, in a dense representation — every way's refs
/// in one flat array instead of one heap `Vec` per way, and node coordinates
/// as sorted parallel arrays (binary-search lookup) instead of a `BTreeMap`,
/// already quantized to the on-disk fixed-point form.
///
/// Invariant (upheld by both producers, not validated here): `node_ids` is
/// sorted and unique, `node_coords` is parallel to it, and every listed node
/// has a known coordinate.
#[derive(Debug, Clone)]
pub struct CompactNetwork {
    /// All ways' node refs, concatenated.
    pub(crate) way_refs: Vec<i64>,
    /// Way `w` occupies `way_refs[way_starts[w]..way_starts[w + 1]]`.
    pub(crate) way_starts: Vec<usize>,
    /// Per-way profile access mask, parallel to the ways.
    pub(crate) way_access: Vec<u8>,
    /// Sorted, unique OSM node ids with known coordinates.
    pub(crate) node_ids: Vec<i64>,
    /// Fixed-point `[lat, lon]`, parallel to `node_ids`. Quantized with the
    /// same [`geo::deg_to_fixed`] the map-based path applies, so lengths and
    /// bytes downstream are identical (D23).
    pub(crate) node_coords: Vec<[i32; 2]>,
}

impl CompactNetwork {
    pub(crate) fn new() -> Self {
        CompactNetwork {
            way_refs: Vec::new(),
            way_starts: vec![0],
            way_access: Vec::new(),
            node_ids: Vec::new(),
            node_coords: Vec::new(),
        }
    }

    /// Append one way. Ways with fewer than two refs are dropped here —
    /// exactly the ways the build loop would skip anyway.
    pub(crate) fn push_way(&mut self, refs: impl IntoIterator<Item = i64>, access: u8) {
        let start = self.way_refs.len();
        self.way_refs.extend(refs);
        if self.way_refs.len() - start < 2 {
            self.way_refs.truncate(start);
            return;
        }
        self.way_starts.push(self.way_refs.len());
        self.way_access.push(access);
    }

    /// Narrow every way's access mask to `keep` (the CLI's `--profiles`
    /// filter). Ways masked to 0 are skipped by the build, same as before.
    pub fn mask_access(&mut self, keep: u8) {
        for access in &mut self.way_access {
            *access &= keep;
        }
    }
}

/// Build a routable graph from accepted ways plus a node-id → `[lat, lon]`
/// (degrees) coordinate map. Degree-2 chains are collapsed (D13); the
/// returned graph is in format v2 with intermediate geometry (D12).
///
/// Only nodes that end up on at least one edge become graph nodes; ways with
/// mask 0 or fewer than two resolvable refs contribute nothing. An empty
/// network yields a valid empty graph (spec §10).
///
/// This is a thin adapter over [`build_graph_compact`] (D23) — kept as the
/// convenient entry point for tests and synthetic fixtures.
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
    let mut net = CompactNetwork::new();
    for way in ways {
        net.push_way(way.node_ids.iter().copied(), way.access);
    }
    // BTreeMap iteration is ascending by key: sorted + unique for free.
    net.node_ids = coords.keys().copied().collect();
    net.node_coords = coords
        .values()
        .map(|&[lat, lon]| [geo::deg_to_fixed(lat), geo::deg_to_fixed(lon)])
        .collect();
    build_graph_compact_with_options(net, collapse)
}

/// Build a routable graph from a [`CompactNetwork`], consuming it (large
/// transients are freed the moment the next stage no longer needs them —
/// D23). Semantics are identical to [`build_graph`]; this *is* the single
/// implementation the adapters feed.
pub fn build_graph_compact(net: CompactNetwork) -> Result<(Graph, BuildStats), BuildError> {
    build_graph_compact_with_options(net, true)
}

/// [`build_graph_compact`] with the M4 collapse toggleable (see
/// [`build_graph_with_options`]).
pub fn build_graph_compact_with_options(
    net: CompactNetwork,
    collapse: bool,
) -> Result<(Graph, BuildStats), BuildError> {
    let mut stats = BuildStats::default();
    let CompactNetwork { way_refs, way_starts, way_access, node_ids, node_coords } = net;
    let way = |w: usize| &way_refs[way_starts[w]..way_starts[w + 1]];

    // ---- v1 pipeline (compact form, D23) ----------------------------------

    // 1. First scan over the ways: mark which known-coordinate nodes are an
    //    endpoint of at least one valid segment (consecutive distinct-id pair
    //    with both coordinates known), and count segments/stats. This
    //    replaces materializing the segment list — step 3 re-scans the ways
    //    over the same predicate, so the derived values are identical.
    let mut is_endpoint = vec![false; node_ids.len()];
    let mut segment_count: usize = 0;
    for (w, &access) in way_access.iter().enumerate() {
        if access == 0 || way(w).len() < 2 {
            continue;
        }
        let mut contributed = false;
        for pair in way(w).windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a == b {
                continue; // repeated ref, an OSM data glitch (D1)
            }
            match (node_ids.binary_search(&a), node_ids.binary_search(&b)) {
                (Ok(pa), Ok(pb)) => {
                    is_endpoint[pa] = true;
                    is_endpoint[pb] = true;
                    segment_count += 1;
                    contributed = true;
                }
                _ => stats.segments_dropped_missing_node += 1,
            }
        }
        if contributed {
            stats.ways_used += 1;
        }
    }

    // 2. Node universe = ids appearing in at least one segment, indexed by
    //    ascending OSM id (D3: node_ids is sorted, so the filtered order is
    //    the ascending-id rank order the original pipeline computed). The
    //    full known-node table is freed before edges are built (D23).
    let kept_nodes = is_endpoint.iter().filter(|&&e| e).count();
    if kept_nodes > (u32::MAX - 1) as usize {
        return Err(BuildError::TooLarge);
    }
    let mut ids: Vec<i64> = Vec::with_capacity(kept_nodes);
    let mut nodes: Vec<[i32; 2]> = Vec::with_capacity(kept_nodes);
    for (i, &e) in is_endpoint.iter().enumerate() {
        if e {
            ids.push(node_ids[i]);
            nodes.push(node_coords[i]);
        }
    }
    drop(is_endpoint);
    drop(node_ids);
    drop(node_coords);

    // A region whose longitude span exceeds 180° either genuinely crosses
    // the ±180° antimeridian or is more than half the globe wide. Crossing
    // regions are re-expressed in a *shifted continuous* longitude frame
    // (D25) so the bbox, grid, and snapping projection never see the seam;
    // genuinely-too-wide data is still rejected. Regions that pass the span
    // check take this branch's `false` arm — i.e. exactly the pre-D25 code
    // path, byte-identical output by construction.
    let mut lon_shifted = false;
    // Seam stitch bookkeeping (D25): `(position, canonical position)` pairs,
    // in the *pre-removal* numbering, sorted by position. Empty for every
    // non-crossing region.
    let mut merged_away: Vec<(u32, u32)> = Vec::new();
    if let (Some(min_lon), Some(max_lon)) =
        (nodes.iter().map(|n| n[1]).min(), nodes.iter().map(|n| n[1]).max())
    {
        // 180° / 360° in fixed-point 1e7.
        const HALF_TURN: i64 = 1_800_000_000;
        const FULL_TURN: i64 = 3_600_000_000;
        // The i32 fixed-point domain caps longitudes at ±214.7°; keep a
        // round safety margin so the shifted frame provably fits.
        const WINDOW: i64 = 2_100_000_000; // ±210°
        if i64::from(max_lon) - i64::from(min_lon) > HALF_TURN {
            // Candidate wrap: negative longitudes + 360° (an exact integer
            // shift — no floating-point rounding anywhere). A genuine
            // crossing region is *narrow* in this frame; junk that is wide
            // in both frames is not an antimeridian case.
            let wrapped =
                |lon: i32| -> i64 { i64::from(lon) + if lon < 0 { FULL_TURN } else { 0 } };
            let (mut wmin, mut wmax) = (i64::MAX, i64::MIN);
            for n in nodes.iter() {
                let w = wrapped(n[1]);
                wmin = wmin.min(w);
                wmax = wmax.max(w);
            }
            if wmax - wmin > HALF_TURN {
                return Err(BuildError::AntimeridianSpanning);
            }
            // Recenter westward when the wrapped frame overflows the i32
            // window (an Alaska-style region mostly east of the seam).
            let offset = if wmax > WINDOW { -FULL_TURN } else { 0 };
            if wmin + offset < -WINDOW {
                // Overhangs the seam by ≳30° on both sides — no real
                // extract does; refuse rather than overflow fixed-point.
                return Err(BuildError::AntimeridianWindow);
            }
            for n in nodes.iter_mut() {
                // Fits i32: |value| ≤ WINDOW < i32::MAX by the checks above.
                n[1] = (wrapped(n[1]) + offset) as i32;
            }
            lon_shifted = true;

            // Seam stitch: OSM ways cannot cross the antimeridian, so
            // mappers split roads at ±180° with two *coincident* end nodes
            // (distinct ids, one per side). Id-dedup (D1) cannot join them,
            // so merge nodes coinciding exactly on the seam meridian: same
            // lat, lon exactly ±180° — the editing convention. The lowest
            // OSM id (= lowest position, ids ascending) is canonical, so
            // the merge is deterministic (D3).
            let seam_fixed = (HALF_TURN + offset) as i32;
            let mut on_seam: Vec<(i32, u32)> = nodes
                .iter()
                .enumerate()
                .filter(|(_, n)| n[1] == seam_fixed)
                .map(|(i, n)| (n[0], i as u32))
                .collect();
            on_seam.sort_unstable();
            for pair in on_seam.windows(2) {
                if pair[0].0 == pair[1].0 {
                    // Same lat: merge the later node into the earlier one
                    // (which may itself be the canonical head of a run).
                    let canon = merged_away
                        .last()
                        .filter(|&&(p, _)| p == pair[0].1)
                        .map_or(pair[0].1, |&(_, c)| c);
                    merged_away.push((pair[1].1, canon));
                }
            }
            stats.seam_nodes_merged = merged_away.len() as u64;
            // The group scan above orders by (lat, position); the redirect
            // lookups and the removal sweep below need position order.
            merged_away.sort_unstable();
            // Remove the merged-away nodes from the universe (they must not
            // remain as isolated snap targets); `ids` keeps every entry so
            // the merged ids still resolve, via the redirect in `lookup`.
            if !merged_away.is_empty() {
                let mut r = 0usize;
                let mut w = 0usize;
                for i in 0..nodes.len() {
                    if r < merged_away.len() && merged_away[r].0 as usize == i {
                        r += 1;
                        continue;
                    }
                    nodes[w] = nodes[i];
                    w += 1;
                }
                nodes.truncate(w);
            }
        }
    }

    // Resolve an OSM id to its final graph node index: rank in `ids`, then
    // the D25 seam redirect (merged node → its coincident partner) and the
    // downshift past removed slots. For non-crossing regions `merged_away`
    // is empty and this is exactly the plain binary search it always was.
    let lookup = |id: i64| -> Option<u32> {
        let pos = ids.binary_search(&id).ok()? as u32;
        let pos = match merged_away.binary_search_by_key(&pos, |&(p, _)| p) {
            Ok(i) => merged_away[i].1,
            Err(_) => pos,
        };
        let removed_before = merged_away.partition_point(|&(p, _)| p < pos) as u32;
        Some(pos - removed_before)
    };

    // 3. Directed edges, both directions per segment (v1 ignores `oneway`).
    //    Same scan and predicate as step 1; a pair is valid there iff both
    //    endpoints are in `ids` here, so the emitted segment sequence is
    //    exactly the one the original pipeline materialized (drops were
    //    already counted in step 1).
    let mut directed: Vec<(u32, u32, u32, u8)> = Vec::with_capacity(segment_count * 2);
    for (w, &access) in way_access.iter().enumerate() {
        if access == 0 || way(w).len() < 2 {
            continue;
        }
        for pair in way(w).windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a == b {
                continue;
            }
            let (Some(ai), Some(bi)) = (lookup(a), lookup(b)) else {
                continue;
            };
            if ai == bi {
                continue; // seam-stitched partners (D25): no self-edge
            }
            let [alat, alon] = nodes[ai as usize];
            let [blat, blon] = nodes[bi as usize];
            // Coordinates were quantized to the on-disk fixed-point
            // representation *before* measuring lengths, so lengths always
            // agree with the geometry a reader of the .graph file sees.
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
    }
    // Post seam-stitch count: `nodes` is the true universe; `ids` may keep
    // merged-away entries so their refs still resolve (identical for every
    // non-crossing region).
    stats.nodes_before_collapse = nodes.len() as u64;
    // The ways and the id table are no longer needed: only index-based data
    // from here on (D23 consume-and-free).
    drop(way_refs);
    drop(way_starts);
    drop(way_access);
    drop(ids);

    // 4. Deterministic order, then merge duplicates in place: same (source,
    //    target) from overlapping ways collapses to one edge with OR-ed
    //    access and the minimum length (D1). Sorting by length puts the
    //    minimum first. In-place compaction replaces the second full-size
    //    `merged` allocation the original pipeline made (D23).
    directed.sort_unstable();
    let mut write = 0usize;
    for read in 0..directed.len() {
        let (s, t, len, acc) = directed[read];
        if write > 0 {
            let (ls, lt, _, lacc) = &mut directed[write - 1];
            if *ls == s && *lt == t {
                *lacc |= acc;
                stats.duplicate_edges_merged += 1;
                continue;
            }
        }
        directed[write] = (s, t, len, acc);
        write += 1;
    }
    directed.truncate(write);
    let merged = directed;
    stats.edges_before_collapse = merged.len() as u64;

    // ---- M4 degree-2 collapse (D13) --------------------------------------

    // Adjacency over the merged edges — the sorted merged list itself is the
    // CSR payload (D23): node `i`'s edges are the contiguous run
    // `merged[adj_off[i]..adj_off[i + 1]]`, holding `(source, target,
    // length_dm, access)` tuples in exactly the order the per-node Vecs used
    // to (merged is sorted by (source, target)).
    let n = nodes.len();
    if merged.len() > u32::MAX as usize {
        return Err(BuildError::TooLarge);
    }
    let mut adj_off = vec![0u32; n + 1];
    for &(s, _, _, _) in &merged {
        adj_off[s as usize + 1] += 1;
    }
    for i in 1..adj_off.len() {
        adj_off[i] += adj_off[i - 1];
    }
    let adj =
        |i: usize| -> &[(u32, u32, u32, u8)] { &merged[adj_off[i] as usize..adj_off[i + 1] as usize] };

    // Interior (collapsible) node: exactly two edges, to distinct neighbors,
    // with equal access. Everything else stays a real node.
    let mut kept: Vec<bool> = (0..n)
        .map(|i| {
            let e = adj(i);
            !(collapse
                && e.len() == 2
                && e[0].1 != e[1].1
                && e[0].1 != i as u32
                && e[1].1 != i as u32
                && e[0].3 == e[1].3)
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
            let (_, n1, l1, _) = adj(cur as usize)[0];
            let (_, n2, l2, _) = adj(cur as usize)[1];
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
        for &(_, t, len, acc) in adj(a as usize) {
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
        let (_, n1, l1, acc) = adj(start as usize)[0];
        let (_, n2, l2, acc2) = adj(start as usize)[1];
        kept[n1.min(n2) as usize] = true;
        // Walk the long way around (via the *other* neighbor when the lowest
        // is now kept and adjacent — its direct edge comes from the pass
        // below). Both of start's edges are tried; kept/consumed guards keep
        // this correct in every ring shape.
        for &(t, len, a2) in &[(n1, l1, acc), (n2, l2, acc2)] {
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

    // The chains now carry everything the emit stage needs: free the merged
    // edge list (the largest remaining transient) before the geometry pool
    // and the post-collapse edge list are allocated (D23).
    drop(merged);
    drop(adj_off);
    drop(consumed);

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

    let flags =
        if lon_shifted { roughroute_core::format::HEADER_FLAG_LON_SHIFTED } else { 0 };
    let graph = Graph::from_parts_with_flags(flags, new_nodes, offsets, edges, geometry)?;
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
    fn antimeridian_crossing_region_builds_in_shifted_frame() {
        // A way from +179° to -179° naively spans ~358° of longitude but is
        // a genuine 2°-wide seam crossing: since D25 it builds in a shifted
        // continuous frame instead of being rejected.
        let ways = vec![RawWay { node_ids: vec![10, 20], access: ACCESS_ALL }];
        let coords = coords(&[(10, 51.0, 179.0), (20, 51.0, -179.0)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert!(g.lon_shifted());
        // bbox is monotonic in the shifted frame: [179°, 181°].
        let bb = g.bbox();
        assert!((bb.min_lon - 179.0).abs() < 1e-9 && (bb.max_lon - 181.0).abs() < 1e-9);
        // Public node coordinates are normalized back to [-180, 180].
        let lons: Vec<f64> = (0..g.node_count()).map(|i| g.node_latlon(i)[1]).collect();
        assert!(lons.iter().all(|&l| (-180.0..=180.0).contains(&l)), "{lons:?}");
        // The edge is measured across the seam (~2° of lon at 51°N ≈ 140 km),
        // not the wrong way around the globe (~358° ≈ 25,000 km).
        let e = &g.edges_from(0)[0];
        assert!((1_300_000..1_500_000).contains(&e.length_dm), "{}", e.length_dm);
        // And the graph round-trips through its bytes with the flag intact.
        let loaded = roughroute_core::Graph::from_bytes(&g.to_bytes()).unwrap();
        assert!(loaded.lon_shifted());
        assert_eq!(loaded.to_bytes(), g.to_bytes());
    }

    #[test]
    fn seam_split_ways_are_stitched_across_the_antimeridian() {
        // The OSM editing convention: a road crossing 180° is split into two
        // ways with *coincident but distinct* end nodes (ids 3 and 4 here —
        // same physical point, one node per side). Id-dedup alone would
        // leave the graph disconnected exactly at the seam; the D25 stitch
        // must merge them so routing crosses.
        let ways = vec![
            RawWay { node_ids: vec![1, 2, 3], access: ACCESS_ALL }, // west half, ends at +180°
            RawWay { node_ids: vec![4, 5, 6], access: ACCESS_ALL }, // east half, starts at −180°
        ];
        let coords = coords(&[
            (1, -16.80, 179.96),
            (2, -16.80, 179.98),
            (3, -16.80, 180.0),
            (4, -16.80, -180.0),
            (5, -16.80, -179.98),
            (6, -16.80, -179.96),
        ]);
        let (g, stats) = build_graph(&ways, &coords).unwrap();
        assert!(g.lon_shifted());
        assert_eq!(stats.seam_nodes_merged, 1);
        // One continuous degree-2 chain end to end: 2 kept nodes, and the
        // merged seam point lives on as edge geometry.
        assert_eq!(g.node_count(), 2);
        let router = Router::new(&g, RouteOptions::default());
        let res = router.route(&[[-16.80, 179.96], [-16.80, -179.96]]).unwrap();
        assert!(!res.fallback, "the stitched seam must be routable");
        // 0.08° of lon at 16.8°S ≈ 8.5 km — the short way across the seam.
        assert!((8_000.0..9_000.0).contains(&res.meters), "meters = {}", res.meters);
        assert!(res.line.iter().all(|p| (-180.0..=180.0).contains(&p[1])));
    }

    #[test]
    fn genuinely_wide_region_is_still_rejected() {
        // Data wide in BOTH frames (spread across ~200° through Greenwich —
        // points near 0° make the wrapped frame just as wide): not an
        // antimeridian case, refuse as before.
        let ways = vec![RawWay { node_ids: vec![10, 20, 30], access: ACCESS_ALL }];
        let coords = coords(&[(10, 51.0, -95.0), (20, 51.0, 0.0), (30, 51.0, 100.0)]);
        assert!(matches!(build_graph(&ways, &coords), Err(BuildError::AntimeridianSpanning)));
    }

    #[test]
    fn seam_overhang_beyond_the_i32_window_is_rejected() {
        // Two clusters 190° apart whose tight window runs through the seam
        // but sticks out more than ~30° on both sides of ±180°: the shifted
        // frame cannot fit fixed-point i32, refuse with the specific error.
        let ways = vec![RawWay { node_ids: vec![10, 20], access: ACCESS_ALL }];
        let coords = coords(&[(10, 51.0, 140.0), (20, 51.0, -50.0)]);
        assert!(matches!(build_graph(&ways, &coords), Err(BuildError::AntimeridianWindow)));
    }

    #[test]
    fn alaska_style_crossing_recenters_west() {
        // Mostly east of the seam (172°E .. -130°W): the wrapped frame
        // [172°, 230°] overflows the +210° window, so it recenters to
        // [-188°, -130°] — bbox min_lon < -180 signals the wrap.
        let ways = vec![RawWay { node_ids: vec![10, 20], access: ACCESS_ALL }];
        let coords = coords(&[(10, 60.0, 172.0), (20, 60.0, -130.0)]);
        let (g, _) = build_graph(&ways, &coords).unwrap();
        assert!(g.lon_shifted());
        let bb = g.bbox();
        assert!((bb.min_lon + 188.0).abs() < 1e-9, "{}", bb.min_lon);
        assert!((bb.max_lon + 130.0).abs() < 1e-9, "{}", bb.max_lon);
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
    fn compact_push_way_drops_sub_two_ref_ways_and_masking_narrows() {
        let mut net = CompactNetwork::new();
        net.push_way([10i64], ACCESS_ALL); // dropped: < 2 refs
        net.push_way([10i64, 20], ACCESS_ALL);
        net.push_way(std::iter::empty(), ACCESS_CAR); // dropped: empty
        net.push_way([20i64, 30], ACCESS_ALL);
        assert_eq!(net.way_access.len(), 2);
        assert_eq!(net.way_starts, vec![0, 2, 4]);
        assert_eq!(net.way_refs, vec![10, 20, 20, 30]);
        net.mask_access(ACCESS_CAR);
        assert_eq!(net.way_access, vec![ACCESS_CAR, ACCESS_CAR]);
    }

    #[test]
    fn compact_direct_and_adapter_paths_build_identical_bytes() {
        // The D23 safety spine at unit level: a hand-assembled CompactNetwork
        // (as the pbf front-end would produce — sorted quantized node table,
        // flat refs) must yield byte-identical output to the map-based
        // adapter for the same network.
        let ways = vec![
            RawWay { node_ids: vec![10, 20, 30, 40], access: ACCESS_ALL },
            RawWay { node_ids: vec![30, 50], access: ACCESS_CAR },
        ];
        let coords = coords(&[
            (10, 35.000, 33.000),
            (20, 35.000, 33.010),
            (30, 35.000, 33.020),
            (40, 35.000, 33.030),
            (50, 34.990, 33.020),
        ]);
        let (via_adapter, stats_a) = build_graph(&ways, &coords).unwrap();

        let mut net = CompactNetwork::new();
        for w in &ways {
            net.push_way(w.node_ids.iter().copied(), w.access);
        }
        net.node_ids = coords.keys().copied().collect();
        net.node_coords = coords
            .values()
            .map(|&[lat, lon]| [geo::deg_to_fixed(lat), geo::deg_to_fixed(lon)])
            .collect();
        let (direct, stats_d) = build_graph_compact(net).unwrap();

        assert_eq!(via_adapter.to_bytes(), direct.to_bytes());
        assert_eq!(stats_a, stats_d);
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
