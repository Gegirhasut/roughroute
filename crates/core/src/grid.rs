//! Uniform-grid spatial index for snapping.
//!
//! Two indexes share one grid geometry (built at load time, spec §7.5):
//!
//! - **nodes** — nearest kept node (`nearest`), the M0–M4 behavior, still
//!   exposed via [`crate::Graph::nearest_node`];
//! - **segments** — nearest point on any edge's shape (`nearest_segment`),
//!   the F10 edge-snapping query (`docs/DECISIONS.md` D15). Every segment of
//!   every *canonical* edge (source index < target index — the direction
//!   whose geometry the pool stores, D12) is bucketed into each grid cell its
//!   bounding rectangle overlaps.
//!
//! Sizing heuristic and the sea-heavy-bbox rationale live in
//! `docs/DECISIONS.md` D4: target ≈1 node per cell on average, total cells
//! capped at 2^20, cells roughly square *in meters*.

use crate::geo;
use crate::graph::Edge;

/// Hard cap on total grid cells: bounds the offsets array at ~4 MB even for
/// huge, mostly-empty (sea) bounding boxes.
const MAX_CELLS: u64 = 1 << 20;

/// Floor for the conservative cell size used in ring lower bounds; prevents a
/// degenerate (polar/zero-span) grid from making ring search unbounded.
const MIN_CELL_METERS: f64 = 0.001;

/// A point-on-edge snap result (F10): the canonical edge, which of its shape
/// segments, the position along that segment, the projected point, and its
/// distance from the query.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RoadSnap {
    /// Global index of the canonical edge (source < target).
    pub edge_index: u32,
    /// Shape segment within the edge: segment `k` joins shape points `k` and
    /// `k + 1` (0 = source node, `geo_len + 1` = target node).
    pub seg: u16,
    /// Position along the segment, `0.0..=1.0`.
    pub t: f64,
    /// The projected point, `[lat, lon]` degrees.
    pub point: [f64; 2],
    /// Haversine distance from the query to `point`, meters.
    pub meters: f64,
}

pub(crate) struct Grid {
    cols: u32,
    rows: u32,
    min_lat: f64,
    min_lon: f64,
    cell_h_deg: f64,
    cell_w_deg: f64,
    /// Conservative lower bound on a cell's smaller ground dimension, used to
    /// bound how far a ring can still contain a closer candidate.
    min_cell_m: f64,
    /// CSR over cells: node indices of cell `c` are
    /// `node_idx[starts[c]..starts[c+1]]`, ascending.
    starts: Vec<u32>,
    node_idx: Vec<u32>,
    /// CSR over cells for edge-shape segments: `(edge_index, seg)` refs of
    /// cell `c` are `seg_refs[seg_starts[c]..seg_starts[c+1]]`, sorted by
    /// `(edge_index, seg)`. A segment appears in every cell its bbox
    /// overlaps, so it is always present in the cell containing its closest
    /// point to any query.
    seg_starts: Vec<u32>,
    seg_refs: Vec<(u32, u16)>,
}

/// Source node of global edge `edge_index`: the node whose CSR range
/// contains it.
pub(crate) fn edge_source(offsets: &[u32], edge_index: u32) -> u32 {
    (offsets.partition_point(|&o| o <= edge_index) - 1) as u32
}

/// Shape point `k` of edge `edge_index` in travel order, fixed-point:
/// `k = 0` is the source node, `1..=geo_len` the intermediate geometry
/// (pool order honoring `reversed`), `geo_len + 1` the target node.
pub(crate) fn shape_point(
    nodes: &[[i32; 2]],
    offsets: &[u32],
    edges: &[Edge],
    geometry: &[[i32; 2]],
    edge_index: u32,
    k: u32,
) -> [i32; 2] {
    let e = &edges[edge_index as usize];
    let m = u32::from(e.geo_len);
    if k == 0 {
        nodes[edge_source(offsets, edge_index) as usize]
    } else if k == m + 1 {
        nodes[e.target as usize]
    } else {
        let i = if e.reversed { m - k } else { k - 1 };
        geometry[e.geo_off as usize + i as usize]
    }
}

impl Grid {
    /// Build both indexes over the graph sections within
    /// `bbox_fixed = [min_lat, min_lon, max_lat, max_lon]`.
    pub fn build(
        nodes: &[[i32; 2]],
        offsets: &[u32],
        edges: &[Edge],
        geometry: &[[i32; 2]],
        bbox_fixed: [i32; 4],
    ) -> Grid {
        let min_lat = geo::fixed_to_deg(bbox_fixed[0]);
        let min_lon = geo::fixed_to_deg(bbox_fixed[1]);
        let max_lat = geo::fixed_to_deg(bbox_fixed[2]);
        let max_lon = geo::fixed_to_deg(bbox_fixed[3]);

        let lat_span = (max_lat - min_lat).max(0.0);
        let lon_span = (max_lon - min_lon).max(0.0);
        let mid_lat = (min_lat + max_lat) / 2.0;
        // cos clamped away from 0 so polar-ish boxes still get a sane aspect.
        let cos_mid = mid_lat.to_radians().cos().max(0.01);

        let height_m = lat_span * geo::METERS_PER_DEG_LAT;
        let width_m = lon_span * geo::METERS_PER_DEG_LON_EQUATOR * cos_mid;

        // ≈1 node per cell on average (D4); shape follows the metric aspect
        // ratio so cells come out roughly square on the ground.
        let target_cells = (nodes.len() as u64).clamp(1, MAX_CELLS) as f64;
        let aspect = if height_m > 0.0 && width_m > 0.0 {
            (width_m / height_m).clamp(1e-6, 1e6)
        } else {
            1.0
        };
        let cols = ((target_cells * aspect).sqrt().round() as u64).clamp(1, 4096) as u32;
        let rows = ((target_cells / f64::from(cols)).round() as u64)
            .clamp(1, MAX_CELLS / u64::from(cols).max(1))
            as u32;

        let cell_w_deg = (lon_span / f64::from(cols)).max(1e-12);
        let cell_h_deg = (lat_span / f64::from(rows)).max(1e-12);

        // Conservative (smallest) metric size of a cell anywhere in the bbox:
        // use the latitude with the smallest cos for the width.
        let cos_worst = f64::max(min_lat.abs(), max_lat.abs()).to_radians().cos().max(0.0);
        let cell_w_m = cell_w_deg * geo::METERS_PER_DEG_LON_EQUATOR * cos_worst;
        let cell_h_m = cell_h_deg * geo::METERS_PER_DEG_LAT;
        let min_cell_m = cell_w_m.min(cell_h_m).max(MIN_CELL_METERS);

        let mut grid = Grid {
            cols,
            rows,
            min_lat,
            min_lon,
            cell_h_deg,
            cell_w_deg,
            min_cell_m,
            starts: Vec::new(),
            node_idx: Vec::new(),
            seg_starts: Vec::new(),
            seg_refs: Vec::new(),
        };

        // --- node index: counting sort of nodes into cells (CSR). Node order
        // within a cell stays ascending by node index (deterministic ties).
        let cell_count = (grid.cols as usize) * (grid.rows as usize);
        let mut counts = vec![0u32; cell_count + 1];
        let cells: Vec<u32> = nodes
            .iter()
            .map(|&[lat, lon]| grid.cell_of(geo::fixed_to_deg(lat), geo::fixed_to_deg(lon)))
            .collect();
        for &c in &cells {
            counts[c as usize + 1] += 1;
        }
        for i in 1..counts.len() {
            counts[i] += counts[i - 1];
        }
        let starts = counts.clone();
        let mut node_idx = vec![0u32; nodes.len()];
        let mut cursor = counts;
        for (i, &c) in cells.iter().enumerate() {
            node_idx[cursor[c as usize] as usize] = i as u32;
            cursor[c as usize] += 1;
        }
        grid.starts = starts;
        grid.node_idx = node_idx;

        // --- segment index: every shape segment of every canonical edge
        // (source < target), bucketed into each cell of its bbox rectangle.
        let mut tagged: Vec<(u32, u32, u16)> = Vec::new(); // (cell, edge, seg)
        for s in 0..nodes.len() as u32 {
            let first = offsets[s as usize];
            let last = offsets[s as usize + 1];
            for edge_index in first..last {
                let e = &edges[edge_index as usize];
                if s >= e.target {
                    continue; // the reversed twin indexes the same shape
                }
                let m = u32::from(e.geo_len);
                for k in 0..=m {
                    let p1 = shape_point(nodes, offsets, edges, geometry, edge_index, k);
                    let p2 = shape_point(nodes, offsets, edges, geometry, edge_index, k + 1);
                    let (c1, r1) =
                        grid.col_row_of(geo::fixed_to_deg(p1[0]), geo::fixed_to_deg(p1[1]));
                    let (c2, r2) =
                        grid.col_row_of(geo::fixed_to_deg(p2[0]), geo::fixed_to_deg(p2[1]));
                    for row in r1.min(r2)..=r1.max(r2) {
                        for col in c1.min(c2)..=c1.max(c2) {
                            tagged.push((row * grid.cols + col, edge_index, k as u16));
                        }
                    }
                }
            }
        }
        tagged.sort_unstable();
        let mut seg_starts = vec![0u32; cell_count + 1];
        for &(c, _, _) in &tagged {
            seg_starts[c as usize + 1] += 1;
        }
        for i in 1..seg_starts.len() {
            seg_starts[i] += seg_starts[i - 1];
        }
        grid.seg_starts = seg_starts;
        grid.seg_refs = tagged.into_iter().map(|(_, e, k)| (e, k)).collect();

        grid
    }

    /// Cell index of a `(lat, lon)` point, clamped into the grid (points
    /// outside the bbox map to the nearest border cell).
    fn cell_of(&self, lat: f64, lon: f64) -> u32 {
        let (col, row) = self.col_row_of(lat, lon);
        row * self.cols + col
    }

    fn col_row_of(&self, lat: f64, lon: f64) -> (u32, u32) {
        let col = ((lon - self.min_lon) / self.cell_w_deg).floor();
        let row = ((lat - self.min_lat) / self.cell_h_deg).floor();
        // NaN-safe clamp: NaN compares false everywhere, so guard explicitly.
        let clamp = |v: f64, hi: u32| -> u32 {
            if v.is_nan() || v < 0.0 {
                0
            } else if v >= f64::from(hi) {
                hi - 1
            } else {
                v as u32
            }
        };
        (clamp(col, self.cols), clamp(row, self.rows))
    }

    /// Iterate rings around the query cell, calling `scan_cell` for each
    /// in-bounds cell, until `scan_cell`'s best-so-far (returned via
    /// `current_best_m`) or `max_meters` rules farther rings out.
    fn ring_search(
        &self,
        lat: f64,
        lon: f64,
        max_meters: f64,
        mut current_best_m: impl FnMut() -> Option<f64>,
        mut scan_cell: impl FnMut(usize),
    ) {
        let (qc, qr) = self.col_row_of(lat, lon);
        let max_ring = [qc, self.cols - 1 - qc, qr, self.rows - 1 - qr]
            .into_iter()
            .max()
            .unwrap_or(0);

        for r in 0..=max_ring {
            let ring_lower_bound_m = f64::from(r.saturating_sub(1)) * self.min_cell_m;
            if ring_lower_bound_m > max_meters {
                break;
            }
            if let Some(best) = current_best_m() {
                if ring_lower_bound_m > best {
                    break;
                }
            }
            // Visit every in-bounds cell at Chebyshev distance r.
            let (qc, qr, r) = (i64::from(qc), i64::from(qr), i64::from(r));
            let mut visit = |col: i64, row: i64| {
                if col >= 0 && row >= 0 && col < i64::from(self.cols) && row < i64::from(self.rows)
                {
                    scan_cell((row as usize) * (self.cols as usize) + col as usize);
                }
            };
            if r == 0 {
                visit(qc, qr);
                continue;
            }
            for col in (qc - r)..=(qc + r) {
                visit(col, qr - r);
                visit(col, qr + r);
            }
            for row in (qr - r + 1)..=(qr + r - 1) {
                visit(qc - r, row);
                visit(qc + r, row);
            }
        }
    }

    /// Nearest node to `(lat, lon)` as `(node_index, meters)`, or `None` when
    /// nothing lies within the search bound. Distance ties break toward the
    /// smaller node index (determinism F9).
    pub fn nearest(
        &self,
        nodes: &[[i32; 2]],
        lat: f64,
        lon: f64,
        max_meters: f64,
    ) -> Option<(u32, f64)> {
        let mut best: Option<(u32, f64)> = None;
        // Both closures need the best distance; a Cell lets the scanner set
        // it while the ring-bound reader gets it, without a double borrow.
        let best_m = std::cell::Cell::new(None::<f64>);
        let starts = &self.starts;
        let node_idx = &self.node_idx;
        self.ring_search(
            lat,
            lon,
            max_meters,
            || best_m.get(),
            |cell| {
                let lo = starts[cell] as usize;
                let hi = starts[cell + 1] as usize;
                for &idx in &node_idx[lo..hi] {
                    let [nlat, nlon] = nodes[idx as usize];
                    let d = geo::haversine_m(
                        lat,
                        lon,
                        geo::fixed_to_deg(nlat),
                        geo::fixed_to_deg(nlon),
                    );
                    let better = match best {
                        // NaN distances (NaN query coordinates) never qualify.
                        None => !d.is_nan(),
                        // Strict inequality + ascending node order per cell
                        // makes distance ties resolve to the smallest index
                        // regardless of scan order.
                        Some((bi, bd)) => d < bd || (d == bd && idx < bi),
                    };
                    if better {
                        best = Some((idx, d));
                        best_m.set(Some(d));
                    }
                }
            },
        );
        best
    }

    /// Nearest point on the shape of any edge matching `access_mask` to
    /// `(lat, lon)` (F10, DECISIONS D15/D16 — snapping is profile-aware so a
    /// waypoint never snaps onto a road its profile cannot use), or `None`
    /// when nothing lies within the search bound. Ties break toward the
    /// smaller `(edge_index, seg)` (determinism F9).
    #[allow(clippy::too_many_arguments)]
    pub fn nearest_segment(
        &self,
        nodes: &[[i32; 2]],
        offsets: &[u32],
        edges: &[Edge],
        geometry: &[[i32; 2]],
        lat: f64,
        lon: f64,
        max_meters: f64,
        access_mask: u8,
    ) -> Option<RoadSnap> {
        // Local equirectangular frame around the query: meter-accurate at
        // snapping scales, cheap, and deterministic.
        let cos_q = lat.to_radians().cos();
        let to_xy = |[plat, plon]: [i32; 2]| -> (f64, f64) {
            let plat = geo::fixed_to_deg(plat);
            let plon = geo::fixed_to_deg(plon);
            (
                (plon - lon) * geo::METERS_PER_DEG_LON_EQUATOR * cos_q,
                (plat - lat) * geo::METERS_PER_DEG_LAT,
            )
        };

        let mut best: Option<RoadSnap> = None;
        let best_m = std::cell::Cell::new(None::<f64>);
        let seg_starts = &self.seg_starts;
        let seg_refs = &self.seg_refs;
        self.ring_search(
            lat,
            lon,
            max_meters,
            || best_m.get(),
            |cell| {
                let lo = seg_starts[cell] as usize;
                let hi = seg_starts[cell + 1] as usize;
                for &(edge_index, seg) in &seg_refs[lo..hi] {
                    if edges[edge_index as usize].access & access_mask == 0 {
                        continue; // road not usable by this profile
                    }
                    let p1 = shape_point(nodes, offsets, edges, geometry, edge_index, u32::from(seg));
                    let p2 =
                        shape_point(nodes, offsets, edges, geometry, edge_index, u32::from(seg) + 1);
                    let (x1, y1) = to_xy(p1);
                    let (x2, y2) = to_xy(p2);
                    let (dx, dy) = (x2 - x1, y2 - y1);
                    let len2 = dx * dx + dy * dy;
                    // Query is the frame origin; project it onto the segment.
                    let t = if len2 > 0.0 {
                        (-(x1 * dx + y1 * dy) / len2).clamp(0.0, 1.0)
                    } else {
                        0.0 // degenerate (coincident endpoints)
                    };
                    let plat = geo::fixed_to_deg(p1[0])
                        + t * (geo::fixed_to_deg(p2[0]) - geo::fixed_to_deg(p1[0]));
                    let plon = geo::fixed_to_deg(p1[1])
                        + t * (geo::fixed_to_deg(p2[1]) - geo::fixed_to_deg(p1[1]));
                    let d = geo::haversine_m(lat, lon, plat, plon);
                    let better = match &best {
                        None => !d.is_nan(),
                        Some(b) => {
                            d < b.meters
                                || (d == b.meters && (edge_index, seg) < (b.edge_index, b.seg))
                        }
                    };
                    if better {
                        best = Some(RoadSnap {
                            edge_index,
                            seg,
                            t,
                            point: [plat, plon],
                            meters: d,
                        });
                        best_m.set(Some(d));
                    }
                }
            },
        );
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::deg_to_fixed;
    use crate::profile::ACCESS_ALL;

    fn fx(lat: f64, lon: f64) -> [i32; 2] {
        [deg_to_fixed(lat), deg_to_fixed(lon)]
    }

    fn bbox_of(points: &[[i32; 2]]) -> [i32; 4] {
        let mut bb = [points[0][0], points[0][1], points[0][0], points[0][1]];
        for &[lat, lon] in points {
            bb[0] = bb[0].min(lat);
            bb[1] = bb[1].min(lon);
            bb[2] = bb[2].max(lat);
            bb[3] = bb[3].max(lon);
        }
        bb
    }

    /// Grid over bare nodes (no edges) for the node-index tests.
    fn node_grid(nodes: &[[i32; 2]]) -> Grid {
        let offsets = vec![0u32; nodes.len() + 1];
        Grid::build(nodes, &offsets, &[], &[], bbox_of(nodes))
    }

    #[test]
    fn nearest_finds_node_in_adjacent_cell() {
        // Many nodes spread over ~10 km so the grid has multiple cells; the
        // true nearest node to the query sits across a cell border.
        let mut nodes = Vec::new();
        for i in 0..100 {
            for j in 0..100 {
                nodes.push(fx(35.0 + 0.001 * f64::from(i), 33.0 + 0.001 * f64::from(j)));
            }
        }
        let grid = node_grid(&nodes);
        let (qlat, qlon) = (35.0424, 33.0571);
        let brute = nodes
            .iter()
            .enumerate()
            .map(|(i, &[la, lo])| {
                (
                    i as u32,
                    geo::haversine_m(qlat, qlon, geo::fixed_to_deg(la), geo::fixed_to_deg(lo)),
                )
            })
            .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)))
            .unwrap();
        let got = grid.nearest(&nodes, qlat, qlon, 10_000.0).unwrap();
        assert_eq!(got.0, brute.0);
        assert!((got.1 - brute.1).abs() < 1e-9);
    }

    #[test]
    fn nearest_respects_max_meters() {
        let nodes = vec![fx(35.0, 33.0)];
        let grid = node_grid(&nodes);
        // ~11 km away, cutoff 200 m: single-cell grid still reports the node
        // (ring 0 is always scanned); the caller compares against the cutoff.
        let got = grid.nearest(&nodes, 35.1, 33.0, 200.0);
        if let Some((idx, d)) = got {
            assert_eq!(idx, 0);
            assert!(d > 200.0);
        }
    }

    #[test]
    fn empty_grid_returns_none() {
        let grid = Grid::build(&[], &[0], &[], &[], [0, 0, 0, 0]);
        assert_eq!(grid.nearest(&[], 35.0, 33.0, f64::INFINITY), None);
        assert!(grid
            .nearest_segment(&[], &[0], &[], &[], 35.0, 33.0, f64::INFINITY, ACCESS_ALL)
            .is_none());
    }

    #[test]
    fn query_outside_bbox_clamps_and_finds() {
        let nodes = vec![fx(35.0, 33.0), fx(35.01, 33.0)];
        let grid = node_grid(&nodes);
        // Query far south of the bbox: nearest is node 0.
        let (idx, d) = grid.nearest(&nodes, 34.9, 33.0, 100_000.0).unwrap();
        assert_eq!(idx, 0);
        assert!(d > 10_000.0 && d < 12_000.0);
    }

    #[test]
    fn nan_query_is_handled_without_panic() {
        let nodes = vec![fx(35.0, 33.0)];
        let grid = node_grid(&nodes);
        // haversine of NaN is NaN; NaN distances are never "better", so no
        // candidate qualifies.
        assert_eq!(grid.nearest(&nodes, f64::NAN, 33.0, 100.0), None);
    }

    /// Graph sections of a long two-segment edge 0 —— g —— 1 for projection
    /// tests: (nodes, offsets, edges, geometry).
    type Sections = (Vec<[i32; 2]>, Vec<u32>, Vec<Edge>, Vec<[i32; 2]>);

    fn edge_fixture() -> Sections {
        let nodes = vec![fx(35.00, 33.00), fx(35.00, 33.02)];
        let geometry = vec![fx(35.01, 33.01)]; // bows north in the middle
        let offsets = vec![0, 1, 2];
        let edges = vec![
            Edge { target: 1, length_dm: 30_000, geo_off: 0, geo_len: 1, reversed: false, access: ACCESS_ALL },
            Edge { target: 0, length_dm: 30_000, geo_off: 0, geo_len: 1, reversed: true, access: ACCESS_ALL },
        ];
        (nodes, offsets, edges, geometry)
    }

    #[test]
    fn nearest_segment_projects_onto_the_shape() {
        let (nodes, offsets, edges, geometry) = edge_fixture();
        let grid = Grid::build(&nodes, &offsets, &edges, &geometry, bbox_of(&nodes));

        // Query near the middle of the first segment (node 0 → geo point).
        let q = (35.006, 33.0048);
        let s = grid
            .nearest_segment(&nodes, &offsets, &edges, &geometry, q.0, q.1, 10_000.0, ACCESS_ALL)
            .unwrap();
        assert_eq!(s.edge_index, 0, "must snap onto the canonical edge");
        assert_eq!(s.seg, 0);
        assert!(s.t > 0.0 && s.t < 1.0, "projection is interior, got t={}", s.t);
        // The projected point is far closer than either endpoint.
        let d0 = geo::haversine_m(q.0, q.1, 35.0, 33.0);
        assert!(s.meters < d0 / 3.0, "{} vs node dist {}", s.meters, d0);
        // Projection sits on the segment: distance point->line ≈ point dist.
        assert!(s.meters < 160.0, "{}", s.meters);

        // Query beyond the far node clamps to t = 1 on the last segment.
        let s = grid
            .nearest_segment(&nodes, &offsets, &edges, &geometry, 35.0, 33.03, 10_000.0, ACCESS_ALL)
            .unwrap();
        assert_eq!((s.seg, s.t), (1, 1.0));
        assert_eq!(s.point, [35.0, 33.02]); // exactly the far node
    }

    #[test]
    fn nearest_segment_beats_or_matches_node_snapping() {
        let (nodes, offsets, edges, geometry) = edge_fixture();
        let grid = Grid::build(&nodes, &offsets, &edges, &geometry, bbox_of(&nodes));
        for q in [(35.006, 33.0048), (35.02, 33.01), (34.99, 33.0), (35.005, 33.015)] {
            let (_, nd) = grid.nearest(&nodes, q.0, q.1, f64::INFINITY).unwrap();
            let s = grid
                .nearest_segment(&nodes, &offsets, &edges, &geometry, q.0, q.1, f64::INFINITY, ACCESS_ALL)
                .unwrap();
            assert!(s.meters <= nd, "segment {} > node {} at {q:?}", s.meters, nd);
        }
    }
}
