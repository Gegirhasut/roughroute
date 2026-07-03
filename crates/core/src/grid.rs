//! Uniform-grid spatial index for nearest-node snapping.
//!
//! Built at load time over the graph's bbox: simple and fast for regional
//! scale, without bloating the file (the index is never persisted).

use crate::geo;

/// Hard cap on total grid cells: bounds the offsets array at ~4 MB even for
/// huge, mostly-empty (sea) bounding boxes.
const MAX_CELLS: u64 = 1 << 20;

/// Floor for the conservative cell size used in ring lower bounds; prevents a
/// degenerate (polar/zero-span) grid from making ring search unbounded.
const MIN_CELL_METERS: f64 = 0.001;

pub(crate) struct Grid {
    cols: u32,
    rows: u32,
    min_lat: f64,
    min_lon: f64,
    cell_h_deg: f64,
    cell_w_deg: f64,
    /// Conservative lower bound on a cell's smaller ground dimension, used to
    /// bound how far a ring can still contain a closer node.
    min_cell_m: f64,
    /// CSR over cells: node indices of cell `c` are
    /// `node_idx[starts[c]..starts[c+1]]`, ascending.
    starts: Vec<u32>,
    node_idx: Vec<u32>,
}

impl Grid {
    /// Build the index over `nodes` (fixed-point `[lat, lon]` pairs) within
    /// `bbox_fixed = [min_lat, min_lon, max_lat, max_lon]`.
    pub fn build(nodes: &[[i32; 2]], bbox_fixed: [i32; 4]) -> Grid {
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

        // ≈1 node per cell on average; shape follows the metric aspect ratio
        // so cells come out roughly square on the ground.
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
        };

        // Counting sort of nodes into cells (CSR). Node order within a cell
        // stays ascending by node index — required for deterministic ties.
        let cell_count = (cols as usize) * (rows as usize);
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

    /// Nearest node to `(lat, lon)` as `(node_index, meters)`, or `None` when
    /// nothing lies within (roughly) `max_meters`.
    ///
    /// Expanding ring search from the query point's cell: scan ring 0, then
    /// ring 1, … keeping the best haversine distance; stop when the *minimum
    /// possible* distance of the next ring exceeds the current best
    /// (correctness: the nearest node may sit in a neighboring cell across a
    /// cell border) or exceeds `max_meters` (nothing admissible remains).
    /// Distance ties break toward the smaller node index (determinism).
    pub fn nearest(
        &self,
        nodes: &[[i32; 2]],
        lat: f64,
        lon: f64,
        max_meters: f64,
    ) -> Option<(u32, f64)> {
        let (qc, qr) = self.col_row_of(lat, lon);
        let max_ring = [qc, self.cols - 1 - qc, qr, self.rows - 1 - qr]
            .into_iter()
            .max()
            .unwrap_or(0);

        let mut best: Option<(u32, f64)> = None;
        for r in 0..=max_ring {
            let ring_lower_bound_m = f64::from(r.saturating_sub(1)) * self.min_cell_m;
            if ring_lower_bound_m > max_meters {
                break;
            }
            if let Some((_, best_m)) = best {
                if ring_lower_bound_m > best_m {
                    break;
                }
            }
            self.scan_ring(nodes, [lat, lon], (qc, qr), r, &mut best);
        }
        best
    }

    /// Visit every in-bounds cell at Chebyshev distance `r` from `center`,
    /// improving `best` with any node closer to the `[lat, lon]` query.
    fn scan_ring(
        &self,
        nodes: &[[i32; 2]],
        query: [f64; 2],
        center: (u32, u32),
        r: u32,
        best: &mut Option<(u32, f64)>,
    ) {
        let [lat, lon] = query;
        let qc = i64::from(center.0);
        let qr = i64::from(center.1);
        let r = i64::from(r);
        let mut visit = |col: i64, row: i64| {
            if col < 0 || row < 0 || col >= i64::from(self.cols) || row >= i64::from(self.rows) {
                return;
            }
            let cell = (row as usize) * (self.cols as usize) + col as usize;
            let lo = self.starts[cell] as usize;
            let hi = self.starts[cell + 1] as usize;
            for &idx in &self.node_idx[lo..hi] {
                let [nlat, nlon] = nodes[idx as usize];
                let d = geo::haversine_m(
                    lat,
                    lon,
                    geo::fixed_to_deg(nlat),
                    geo::fixed_to_deg(nlon),
                );
                let better = match *best {
                    None => !d.is_nan(),
                    // Strict inequality + ascending node order per cell makes
                    // distance ties resolve to the smallest node index
                    // regardless of scan order.
                    Some((bi, bd)) => d < bd || (d == bd && idx < bi),
                };
                if better {
                    *best = Some((idx, d));
                }
            }
        };

        if r == 0 {
            visit(qc, qr);
            return;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::deg_to_fixed;

    fn fx(lat: f64, lon: f64) -> [i32; 2] {
        [deg_to_fixed(lat), deg_to_fixed(lon)]
    }

    fn bbox_of(nodes: &[[i32; 2]]) -> [i32; 4] {
        let mut bb = [nodes[0][0], nodes[0][1], nodes[0][0], nodes[0][1]];
        for &[lat, lon] in nodes {
            bb[0] = bb[0].min(lat);
            bb[1] = bb[1].min(lon);
            bb[2] = bb[2].max(lat);
            bb[3] = bb[3].max(lon);
        }
        bb
    }

    #[test]
    fn nearest_finds_node_in_adjacent_cell() {
        let mut nodes = Vec::new();
        for i in 0..100 {
            for j in 0..100 {
                nodes.push(fx(35.0 + 0.001 * f64::from(i), 33.0 + 0.001 * f64::from(j)));
            }
        }
        let grid = Grid::build(&nodes, bbox_of(&nodes));
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
        let grid = Grid::build(&nodes, bbox_of(&nodes));
        let got = grid.nearest(&nodes, 35.1, 33.0, 200.0);
        if let Some((idx, d)) = got {
            assert_eq!(idx, 0);
            assert!(d > 200.0);
        }
    }

    #[test]
    fn empty_grid_returns_none() {
        let grid = Grid::build(&[], [0, 0, 0, 0]);
        assert_eq!(grid.nearest(&[], 35.0, 33.0, f64::INFINITY), None);
    }

    #[test]
    fn query_outside_bbox_clamps_and_finds() {
        let nodes = vec![fx(35.0, 33.0), fx(35.01, 33.0)];
        let grid = Grid::build(&nodes, bbox_of(&nodes));
        let (idx, d) = grid.nearest(&nodes, 34.9, 33.0, 100_000.0).unwrap();
        assert_eq!(idx, 0);
        assert!(d > 10_000.0 && d < 12_000.0);
    }

    #[test]
    fn nan_query_is_handled_without_panic() {
        let nodes = vec![fx(35.0, 33.0)];
        let grid = Grid::build(&nodes, bbox_of(&nodes));
        assert_eq!(grid.nearest(&nodes, f64::NAN, 33.0, 100.0), None);
    }
}
