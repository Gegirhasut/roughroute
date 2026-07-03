//! The in-memory road graph: an immutable CSR adjacency structure loaded from
//! `.graph` bytes, plus the load-time snapping grid.

use crate::format;
use crate::geo;
use crate::grid::Grid;
use crate::profile::ACCESS_ALL;

/// Errors produced while loading or validating a `.graph` buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GraphError {
    /// The buffer does not start with the `RRG1` magic bytes.
    BadMagic,
    /// The format version is not one this build can read.
    UnsupportedVersion(u16),
    /// The buffer is shorter than its header promises.
    Truncated,
    /// The buffer has the right shape but violates a structural invariant
    /// (non-monotonic CSR offsets, out-of-range coordinates, …).
    Malformed(&'static str),
}

impl core::fmt::Display for GraphError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GraphError::BadMagic => write!(f, "not a .graph file (bad magic)"),
            GraphError::UnsupportedVersion(v) => {
                write!(f, "unsupported .graph format version {v} (expected {})", format::VERSION)
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
/// road segment (v1 deliberately ignores `oneway`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Edge {
    /// Index of the neighbor node.
    pub target: u32,
    /// Edge length in decimeters, always ≥ 1.
    pub length_dm: u32,
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

/// An immutable road graph: node coordinates plus CSR adjacency, with a
/// uniform-grid spatial index built at load time for waypoint snapping.
///
/// `Graph` is `Send + Sync`; any number of [`crate::Router`]s may share one
/// instance across threads.
pub struct Graph {
    flags: u16,
    /// `[min_lat, min_lon, max_lat, max_lon]`, fixed-point 1e7. Carried
    /// verbatim from the file so `to_bytes` round-trips byte-identically.
    bbox_fixed: [i32; 4],
    /// Per node `[lat, lon]`, fixed-point 1e7, indexed by node index.
    nodes: Vec<[i32; 2]>,
    /// CSR row offsets: edges of node `n` are `edges[offsets[n]..offsets[n+1]]`.
    offsets: Vec<u32>,
    edges: Vec<Edge>,
    grid: Grid,
}

impl Graph {
    /// Load a graph from an already-built `.graph` binary buffer.
    ///
    /// Reads no files, touches no network. The buffer is fully validated up
    /// front: after this returns `Ok`, every node index, CSR offset, and
    /// coordinate in the graph is known to be in range, so routing never has
    /// to re-check. The bytes are copied into owned storage; the buffer may
    /// be dropped afterwards.
    pub fn from_bytes(bytes: &[u8]) -> Result<Graph, GraphError> {
        Self::assemble(format::parse(bytes)?)
    }

    /// Build a graph from raw sections (used by the `roughroute-build`
    /// preprocessor and by tests). The bounding box is computed tight over
    /// the nodes; the same validation as [`Graph::from_bytes`] applies.
    pub fn from_parts(
        nodes: Vec<[i32; 2]>,
        offsets: Vec<u32>,
        edges: Vec<Edge>,
    ) -> Result<Graph, GraphError> {
        let mut bbox_fixed = [0i32; 4];
        if let Some(first) = nodes.first() {
            bbox_fixed = [first[0], first[1], first[0], first[1]];
            for [lat, lon] in &nodes[1..] {
                bbox_fixed[0] = bbox_fixed[0].min(*lat);
                bbox_fixed[1] = bbox_fixed[1].min(*lon);
                bbox_fixed[2] = bbox_fixed[2].max(*lat);
                bbox_fixed[3] = bbox_fixed[3].max(*lon);
            }
        }
        Self::assemble(format::Parts { flags: 0, bbox_fixed, nodes, offsets, edges })
    }

    /// Validate all semantic invariants and construct the graph + snapping
    /// grid. Single validation path for both `from_bytes` and `from_parts`.
    fn assemble(parts: format::Parts) -> Result<Graph, GraphError> {
        let format::Parts { flags, bbox_fixed, nodes, offsets, edges } = parts;

        // v1 defines no flag bits; refusing unknown ones keeps forward
        // compatibility honest (a file that needs a feature we lack fails
        // loudly instead of routing wrongly).
        if flags != 0 {
            return Err(GraphError::Malformed("unknown flags bits set"));
        }
        if nodes.len() > u32::MAX as usize - 1 || edges.len() > u32::MAX as usize {
            return Err(GraphError::Malformed("too many nodes or edges"));
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
        for &[lat, lon] in &nodes {
            if !(-900_000_000..=900_000_000).contains(&lat)
                || !(-1_800_000_000..=1_800_000_000).contains(&lon)
            {
                return Err(GraphError::Malformed("node coordinate out of lat/lon range"));
            }
            if lat < min_lat || lat > max_lat || lon < min_lon || lon > max_lon {
                return Err(GraphError::Malformed("node coordinate outside header bbox"));
            }
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
        }

        let grid = Grid::build(&nodes, bbox_fixed);
        Ok(Graph { flags, bbox_fixed, nodes, offsets, edges, grid })
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
        })
    }

    /// The graph's bounding box in degrees, as stored in the file header.
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

    /// A 3-node path graph: 0 —— 1 —— 2 along a line of longitude.
    fn path_graph() -> Graph {
        let nodes = vec![fx(35.00, 33.00), fx(35.01, 33.00), fx(35.02, 33.00)];
        let offsets = vec![0, 1, 3, 4];
        let edges = vec![
            Edge { target: 1, length_dm: 11120, access: 0b11 },
            Edge { target: 0, length_dm: 11120, access: 0b11 },
            Edge { target: 2, length_dm: 11120, access: 0b11 },
            Edge { target: 1, length_dm: 11120, access: 0b11 },
        ];
        Graph::from_parts(nodes, offsets, edges).unwrap()
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
    fn bbox_is_tight_over_nodes() {
        let g = path_graph();
        let bb = g.bbox();
        assert!((bb.min_lat - 35.00).abs() < 1e-9);
        assert!((bb.max_lat - 35.02).abs() < 1e-9);
        assert!((bb.min_lon - 33.00).abs() < 1e-9);
        assert!((bb.max_lon - 33.00).abs() < 1e-9);
    }

    #[test]
    fn bytes_round_trip_preserves_graph_and_bytes() {
        let g = path_graph();
        let bytes = g.to_bytes();
        let g2 = Graph::from_bytes(&bytes).unwrap();
        assert_eq!(g2.node_count(), g.node_count());
        assert_eq!(g2.edge_count(), g.edge_count());
        assert_eq!(g2.bbox(), g.bbox());
        for n in 0..g.node_count() {
            assert_eq!(g2.node_latlon(n), g.node_latlon(n));
            assert_eq!(g2.edges_from(n), g.edges_from(n));
        }
        // Byte-stable: to_bytes(from_bytes(b)) == b (determinism F9).
        assert_eq!(g2.to_bytes(), bytes);
    }

    #[test]
    fn empty_graph_loads() {
        let g = Graph::from_parts(vec![], vec![0], vec![]).unwrap();
        assert_eq!(g.node_count(), 0);
        let bytes = g.to_bytes();
        let g2 = Graph::from_bytes(&bytes).unwrap();
        assert_eq!(g2.node_count(), 0);
        assert_eq!(g2.nearest_node(35.0, 33.0, 1e9), None);
    }

    #[test]
    fn semantic_validation_rejects_bad_graphs() {
        let nodes = vec![fx(35.0, 33.0), fx(35.01, 33.0)];
        let ok_edge = Edge { target: 1, length_dm: 10, access: 0b11 };

        // Non-monotonic offsets.
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 0], vec![ok_edge]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // offsets[n] != edge_count.
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 2], vec![ok_edge]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Edge target out of range.
        let bad = Edge { target: 9, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Zero-length edge.
        let bad = Edge { length_dm: 0, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Unknown access bits.
        let bad = Edge { access: 0b100, ..ok_edge };
        let r = Graph::from_parts(nodes.clone(), vec![0, 1, 1], vec![bad]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
        // Node coordinate out of range.
        let r = Graph::from_parts(vec![[i32::MAX, 0]], vec![0, 0], vec![]);
        assert!(matches!(r, Err(GraphError::Malformed(_))));
    }
}
