//! The in-memory road graph: an immutable CSR adjacency structure loaded from
//! `.graph` bytes, plus the load-time snapping grid and the shared
//! intermediate-geometry pool (format v2, M4).

use crate::format;
use crate::geo;
use crate::grid::Grid;
use crate::profile::ACCESS_ALL;

/// Errors produced while loading or validating a `.graph` buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GraphError {
    /// The buffer does not start with the `RRG1` magic bytes.
    BadMagic,
    /// The format version is not one this build can read (v1 graphs must be
    /// rebuilt with the current `roughroute build`).
    UnsupportedVersion(u16),
    /// The buffer is shorter than its header promises.
    Truncated,
    /// The buffer has the right shape but violates a structural invariant
    /// (non-monotonic CSR offsets, out-of-range coordinates, dangling
    /// geometry references, …).
    Malformed(&'static str),
}

impl core::fmt::Display for GraphError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GraphError::BadMagic => write!(f, "not a .graph file (bad magic)"),
            GraphError::UnsupportedVersion(v) => {
                write!(
                    f,
                    "unsupported .graph format version {v} (expected {}; rebuild the graph)",
                    format::VERSION
                )
            }
            GraphError::Truncated => write!(f, ".graph data is truncated"),
            GraphError::Malformed(why) => write!(f, "malformed .graph data: {why}"),
        }
    }
}

impl std::error::Error for GraphError {}

/// One directed edge of the road graph.
///
/// The graph is stored directed; the builder emits both directions for every
/// road segment (v1 deliberately ignores `oneway`). Since format v2 an edge
/// may be a *collapsed chain* of degree-2 way nodes: its intermediate shape
/// lives in the graph's shared geometry pool (see [`Graph::edge_geometry`]),
/// and the two directions of one segment share a single pool range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Edge {
    /// Index of the neighbor node.
    pub target: u32,
    /// Edge length in decimeters, always ≥ 1. For a collapsed chain this is
    /// the sum of the original per-segment lengths (`docs/DECISIONS.md` D13),
    /// so collapsed and uncollapsed graphs produce identical A\* costs.
    pub length_dm: u32,
    /// Start of this edge's intermediate geometry in the pool.
    pub geo_off: u32,
    /// Number of intermediate geometry points (0 for an uncollapsed edge).
    pub geo_len: u16,
    /// The pool stores the canonical (lower node index → higher) direction;
    /// `true` means this edge traverses it back-to-front.
    pub reversed: bool,
    /// Profile bitmask: `bit0 = car`, `bit1 = foot` (see [`crate::profile`]).
    pub access: u8,
}

/// Geographic bounding box of the graph, in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BBox {
    /// Southernmost latitude.
    pub min_lat: f64,
    /// Westernmost longitude.
    pub min_lon: f64,
    /// Northernmost latitude.
    pub max_lat: f64,
    /// Easternmost longitude.
    pub max_lon: f64,
}

/// An immutable road graph: node coordinates, CSR adjacency, the shared
/// intermediate-geometry pool, and a uniform-grid spatial index built at load
/// time for waypoint snapping.
///
/// `Graph` is `Send + Sync`; any number of [`crate::Router`]s may share one
/// instance across threads.
pub struct Graph {
    flags: u16,
    /// `[min_lat, min_lon, max_lat, max_lon]`, fixed-point 1e7, covering
    /// nodes *and* geometry. Carried verbatim from the file so `to_bytes`
    /// round-trips byte-identically.
    bbox_fixed: [i32; 4],
    /// Per node `[lat, lon]`, fixed-point 1e7, indexed by node index.
    nodes: Vec<[i32; 2]>,
    /// CSR row offsets: edges of node `n` are `edges[offsets[n]..offsets[n+1]]`.
    offsets: Vec<u32>,
    edges: Vec<Edge>,
    /// Shared intermediate-geometry pool (fixed-point `[lat, lon]`).
    geometry: Vec<[i32; 2]>,
    grid: Grid,
}

impl Graph {
    /// Load a graph from an already-built `.graph` binary buffer.
    ///
    /// Reads no files, touches no network. The buffer is fully validated up
    /// front: after this returns `Ok`, every node index, CSR offset,
    /// geometry reference, and coordinate in the graph is known to be in
    /// range, so routing never has to re-check. The bytes are copied into
    /// owned storage; the buffer may be dropped afterwards.
    pub fn from_bytes(bytes: &[u8]) -> Result<Graph, GraphError> {
        Self::assemble(format::parse(bytes)?)
    }

    /// Build a graph from raw sections (used by the `roughroute-build`
    /// preprocessor and by tests). The bounding box is computed tight over
    /// nodes and geometry; the same validation as [`Graph::from_bytes`]
    /// applies.
    pub fn from_parts(
        nodes: Vec<[i32; 2]>,
        offsets: Vec<u32>,
        edges: Vec<Edge>,
        geometry: Vec<[i32; 2]>,
    ) -> Result<Graph, GraphError> {
        let mut bbox_fixed = [0i32; 4];
        let mut points = nodes.iter().chain(geometry.iter());
        if let Some(first) = points.next() {
            bbox_fixed = [first[0], first[1], first[0], first[1]];
            for [lat, lon] in points {
                bbox_fixed[0] = bbox_fixed[0].min(*lat);
                bbox_fixed[1] = bbox_fixed[1].min(*lon);
                bbox_fixed[2] = bbox_fixed[2].max(*lat);
                bbox_fixed[3] = bbox_fixed[3].max(*lon);
            }
        }
        Self::assemble(format::Parts { flags: 0, bbox_fixed, nodes, offsets, edges, geometry })
    }

    /// Validate all semantic invariants and construct the graph + snapping
    /// grid. Single validation path for both `from_bytes` and `from_parts`.
    fn assemble(parts: format::Parts) -> Result<Graph, GraphError> {
        let format::Parts { flags, bbox_fixed, nodes, offsets, edges, geometry } = parts;

        // v2 defines no header flag bits; refusing unknown ones keeps forward
        // compatibility honest (a file that needs a feature we lack fails
        // loudly instead of routing wrongly).
        if flags != 0 {
            return Err(GraphError::Malformed("unknown flags bits set"));
        }
        if nodes.len() > u32::MAX as usize - 1
            || edges.len() > u32::MAX as usize
            || geometry.len() > u32::MAX as usize
        {
            return Err(GraphError::Malformed("too many nodes, edges, or geometry points"));
        }
        if offsets.len() != nodes.len() + 1 {
            return Err(GraphError::Malformed("offsets length != node_count + 1"));
        }
        if offsets.first() != Some(&0) {
            return Err(GraphError::Malformed("offsets[0] != 0"));
        }
        if offsets.windows(2).any(|w| w[0] > w[1]) {
            return Err(GraphError::Malformed("offsets not non-decreasing"));
        }
        // offsets is non-empty (checked above), so last() exists.
        if offsets.last().copied() != Some(edges.len() as u32) {
            return Err(GraphError::Malformed("offsets[node_count] != edge_count"));
        }

        let [min_lat, min_lon, max_lat, max_lon] = bbox_fixed;
        if min_lat > max_lat || min_lon > max_lon {
            return Err(GraphError::Malformed("inverted bbox"));
        }
        let coord_ok = |&[lat, lon]: &[i32; 2]| {
            (-900_000_000..=900_000_000).contains(&lat)
                && (-1_800_000_000..=1_800_000_000).contains(&lon)
                && lat >= min_lat
                && lat <= max_lat
                && lon >= min_lon
                && lon <= max_lon
        };
        if !nodes.iter().all(coord_ok) {
            return Err(GraphError::Malformed("node coordinate out of range or outside bbox"));
        }
        if !geometry.iter().all(coord_ok) {
            return Err(GraphError::Malformed(
                "geometry coordinate out of range or outside bbox",
            ));
        }

        let node_count = nodes.len() as u32;
        for e in &edges {
            if e.target >= node_count {
                return Err(GraphError::Malformed("edge target out of range"));
            }
            if e.length_dm == 0 {
                return Err(GraphError::Malformed("zero-length edge"));
            }
            if e.access & !ACCESS_ALL != 0 {
                return Err(GraphError::Malformed("unknown access bits set"));
            }
            if u64::from(e.geo_off) + u64::from(e.geo_len) > geometry.len() as u64 {
                return Err(GraphError::Malformed("edge geometry reference out of range"));
            }
            // Canonical form for geometry-less edges keeps golden byte
            // comparisons meaningful.
            if e.geo_len == 0 && (e.geo_off != 0 || e.reversed) {
                return Err(GraphError::Malformed(
                    "geometry-less edge with non-canonical geo_off/flags",
                ));
            }
        }

        let grid = Grid::build(&nodes, bbox_fixed);
        Ok(Graph { flags, bbox_fixed, nodes, offsets, edges, geometry, grid })
    }

    /// Serialize back to the `.graph` binary format. Loading the result with
    /// [`Graph::from_bytes`] reproduces this graph exactly, and
    /// `g.to_bytes()` is byte-identical for byte-identical inputs
    /// (determinism requirement F9).
    pub fn to_bytes(&self) -> Vec<u8> {
        format::serialize(&format::Parts {
            flags: self.flags,
            bbox_fixed: self.bbox_fixed,
            nodes: self.nodes.clone(),
            offsets: self.offsets.clone(),
            edges: self.edges.clone(),
            geometry: self.geometry.clone(),
        })
    }

    /// The graph's bounding box in degrees, as stored in the file header
    /// (covers nodes and edge geometry).
    pub fn bbox(&self) -> BBox {
        BBox {
            min_lat: geo::fixed_to_deg(self.bbox_fixed[0]),
            min_lon: geo::fixed_to_deg(self.bbox_fixed[1]),
            max_lat: geo::fixed_to_deg(self.bbox_fixed[2]),
            max_lon: geo::fixed_to_deg(self.bbox_fixed[3]),
        }
    }

    /// Number of nodes in the graph.
    pub fn node_count(&self) -> u32 {
        self.nodes.len() as u32
    }

    /// Number of directed edges in the graph.
    pub fn edge_count(&self) -> u32 {
        self.edges.len() as u32
    }

    /// Number of points in the shared intermediate-geometry pool.
    pub fn geometry_point_count(&self) -> u32 {
        self.geometry.len() as u32
    }

    /// Coordinates of node `index` as `[lat, lon]` degrees.
    ///
    /// `index` must be `< node_count()`; passing an out-of-range index is a
    /// caller bug and will panic (all indices stored in the graph itself are
    /// validated at load).
    pub fn node_latlon(&self, index: u32) -> [f64; 2] {
        let [lat, lon] = self.nodes[index as usize];
        [geo::fixed_to_deg(lat), geo::fixed_to_deg(lon)]
    }

    /// Outgoing edges of node `index` (same range contract as
    /// [`Graph::node_latlon`]).
    pub fn edges_from(&self, index: u32) -> &[Edge] {
        let start = self.offsets[index as usize] as usize;
        let end = self.offsets[index as usize + 1] as usize;
        &self.edges[start..end]
    }

    /// Index of node `index`'s first edge in the global edge array; edge `i`
    /// of `edges_from(index)` is global edge `first_edge_index(index) + i`.
    pub(crate) fn first_edge_index(&self, index: u32) -> u32 {
        self.offsets[index as usize]
    }

    /// The edge with global index `edge_index` (as recorded by A\* — the
    /// position in the file's edge section).
    ///
    /// `edge_index` must be `< edge_count()` (caller-bug panic contract as
    /// with [`Graph::node_latlon`]).
    pub fn edge(&self, edge_index: u32) -> &Edge {
        &self.edges[edge_index as usize]
    }

    /// The intermediate shape of `edge`, in the pool's canonical storage
    /// order (fixed-point `[lat, lon]`; empty for uncollapsed edges).
    ///
    /// When `edge.reversed` is set, traverse the returned slice back-to-front
    /// to follow the edge's direction of travel.
    pub fn edge_geometry(&self, edge: &Edge) -> &[[i32; 2]] {
        &self.geometry[edge.geo_off as usize..edge.geo_off as usize + edge.geo_len as usize]
    }

    /// Nearest graph node to `(lat, lon)` (degrees) within roughly
    /// `max_meters`, as `(node_index, haversine_meters)`.
    ///
    /// Returns `None` when no node exists within the search bound. Distance
    /// ties break toward the smaller node index (determinism F9).
    pub fn nearest_node(&self, lat: f64, lon: f64, max_meters: f64) -> Option<(u32, f64)> {
        self.grid.nearest(&self.nodes, lat, lon, max_meters)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::deg_to_fixed;

    fn fx(lat: f64, lon: f64) -> [i32; 2] {
        [deg_to_fixed(lat), deg_to_fixed(lon)]
    }

    fn plain(target: u32, length_dm: u32, access: u8) -> Edge {
        Edge { target, length_dm, geo_off: 0, geo_len: 0, reversed: false, access }
    }

    /// A 3-node path graph: 0 —— 1 —— 2 along a line of longitude.
    fn path_graph() -> Graph {
        let nodes = vec![fx(35.00, 33.00), fx(35.01, 33.00), fx(35.02, 33.00)];
        let offsets = vec![0, 1, 3, 4];
        let edges = vec![
            plain(1, 11120, 0b11),
            plain(0, 11120, 0b11),
            plain(2, 11120, 0b11),
            plain(1, 11120, 0b11),
        ];
        Graph::from_parts(nodes, offsets, edges, vec![]).unwrap()
    }

    /// Two nodes joined by a collapsed edge with two intermediate points.
    fn collapsed_graph() -> Graph {
        let nodes = vec![fx(35.00, 33.00), fx(35.03, 33.00)];
        let geometry = vec![fx(35.01, 33.001), fx(35.02, 32.999)];
        let offsets = vec![0, 1, 2];
        let edges = vec![
            Edge { target: 1, length_dm: 33400, geo_off: 0, geo_len: 2, reversed: false, access: 0b11 },
            Edge { target: 0, length_dm: 33400, geo_off: 0, geo_len: 2, reversed: true, access: 0b11 },
        ];
        Graph::from_parts(nodes, offsets, edges, geometry).unwrap()
    }

    #[test]
    fn graph_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Graph>();
    }

    #[test]
    fn csr_traversal() {
        let g = path_graph();
        assert_eq!(g.node_count(), 3);
        assert_eq!(g.edge_count(), 4);
        assert_eq!(g.edges_from(0).len(), 1);
        assert_eq!(g.edges_from(1).len(), 2);
        assert_eq!(g.edges_from(2).len(), 1);
        assert_eq!(g.edges_from(1)[0].target, 0);
        assert_eq!(g.edges_from(1)[1].target, 2);
        // Undirected pairing: every edge has its reverse.
        for n in 0..g.node_count() {
            for e in g.edges_from(n) {
                assert!(g.edges_from(e.target).iter().any(|r| r.target == n));
            }
        }
    }

    #[test]
    fn geometry_pool_is_shared_between_directions() {
        let g = collapsed_graph();
        assert_eq!(g.geometry_point_count(), 2);
        let fwd = &g.edges_from(0)[0];
        let bwd = &g.edges_from(1)[0];
        assert!(!fwd.reversed && bwd.reversed);
        assert_eq!(g.edge_geometry(fwd), g.edge_geometry(bwd));
        assert_eq!(g.edge_geometry(fwd).len(), 2);
    }

    #[test]
    fn bbox_covers_nodes_and_geometry() {
        let g = collapsed_graph();
        let bb = g.bbox();
        // Geometry point at lon 32.999 widens the bbox beyond the nodes.
        assert!((bb.min_lon - 32.999).abs() < 1e-9);
        assert!((bb.max_lon - 33.001).abs() < 1e-9);
        assert!((bb.min_lat - 35.00).abs() < 1e-9);
        assert!((bb.max_lat - 35.03).abs() < 1e-9);
    }

    #[test]
    fn bytes_round_trip_preserves_graph_and_bytes() {
        for g in [path_graph(), collapsed_graph()] {
            let bytes = g.to_bytes();
            let g2 = Graph::from_bytes(&bytes).unwrap();
            assert_eq!(g2.node_count(), g.node_count());
            assert_eq!(g2.edge_count(), g.edge_count());
            assert_eq!(g2.geometry_point_count(), g.geometry_point_count());
            assert_eq!(g2.bbox(), g.bbox());
            for n in 0..g.node_count() {
                assert_eq!(g2.node_latlon(n), g.node_latlon(n));
                assert_eq!(g2.edges_from(n), g.edges_from(n));
            }
            // Byte-stable: to_bytes(from_bytes(b)) == b (determinism F9).
            assert_eq!(g2.to_bytes(), bytes);
        }
    }

    #[test]
    fn empty_graph_loads() {
        let g = Graph::from_parts(vec![], vec![0], vec![], vec![]).unwrap();
        assert_eq!(g.node_count(), 0);
        let bytes = g.to_bytes();
        let g2 = Graph::from_bytes(&bytes).unwrap();
        assert_eq!(g2.node_count(), 0);
        assert_eq!(g2.nearest_node(35.0, 33.0, 1e9), None);
    }

    #[test]
    fn semantic_validation_rejects_bad_graphs() {
        let nodes = vec![fx(35.0, 33.0), fx(35.01, 33.0)];
        let ok_edge = plain(1, 10, 0b11);

        // Non-monotonic offsets.
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 0], vec![ok_edge], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // offsets[n] != edge_count.
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 2], vec![ok_edge], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Edge target out of range.
        let bad = Edge { target: 9, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Zero-length edge (violates D5).
        let bad = Edge { length_dm: 0, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Unknown access bits.
        let bad = Edge { access: 0b100, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Node coordinate out of range.
        let r = Graph::from_parts(vec![[i32::MAX, 0]], vec![0, 0], vec![], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Geometry reference past the pool.
        let bad = Edge { geo_off: 0, geo_len: 3, ..ok_edge };
        let r = Graph::from_parts(
            nodes.clone(),
            vec![0, 1, 1],
            vec![bad],
            vec![fx(35.005, 33.0)],
        );
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Geometry-less edge with a non-canonical reversed flag.
        let bad = Edge { reversed: true, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
    }
}
