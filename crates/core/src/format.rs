//! The `RRG1` binary `.graph` format, version 2 (spec §7 + M4 geometry
//! block, `docs/DECISIONS.md` D12). Little-endian.
//!
//! Layout (all offsets from the start of the buffer):
//!
//! | section  | bytes | content |
//! |---|---|---|
//! | header   | 40 | see below |
//! | nodes    | `node_count × 8` | per node: `lat: i32`, `lon: i32` fixed-point 1e7 |
//! | offsets  | `(node_count+1) × 4` | CSR row offsets into the edge section |
//! | edges    | `edge_count × 16` | per edge: `target: u32`, `length_dm: u32`, `geo_off: u32`, `geo_len: u16`, `flags: u8` (bit0 = geometry reversed; other bits must be 0), `access: u8` |
//! | geometry | `geo_point_count × 8` | shared pool of intermediate `[lat, lon]` fixed-point points |
//!
//! Header: `magic: [u8;4] = "RRG1"`, `version: u16 = 2`, `flags: u16`,
//! `node_count: u32`, `edge_count: u32`, `min_lat, min_lon, max_lat,
//! max_lon: i32` (fixed-point 1e7 bbox over nodes *and* geometry),
//! `geo_point_count: u32`, `_reserved: u32` (must be 0).
//!
//! An edge's intermediate shape is `geometry[geo_off .. geo_off + geo_len]`.
//! The two directed edges of one road segment share a single pool range
//! stored in the canonical direction (lower node index → higher); the
//! opposite direction sets edge flag bit0 (`FLAG_REVERSED`). Uncollapsed
//! edges have `geo_len = 0`, `geo_off = 0`, `flags = 0`.
//!
//! Version 1 (the pre-M4, geometry-less layout) is **refused** with
//! [`GraphError::UnsupportedVersion`] — old graphs must be rebuilt (spec §13).
//!
//! # Alignment (kept zero-copy-ready, D2)
//!
//! The header is exactly 40 bytes and every section size is a multiple of 4,
//! so each section starts 4-byte aligned relative to the buffer start. v2
//! still parses into owned `Vec`s and accepts a buffer of any alignment.

use crate::graph::{Edge, GraphError};

/// The magic bytes opening every `.graph` file.
pub const MAGIC: [u8; 4] = *b"RRG1";
/// The format version this crate reads and writes.
pub const VERSION: u16 = 2;
/// Size of the fixed header in bytes.
pub const HEADER_BYTES: usize = 40;
/// Size of one node record (two fixed-point `i32` coordinates).
pub const NODE_BYTES: usize = 8;
/// Size of one edge record.
pub const EDGE_BYTES: usize = 16;
/// Size of one geometry-pool point (two fixed-point `i32` coordinates).
pub const GEO_POINT_BYTES: usize = 8;

/// Edge flag bit0: traverse the geometry range back-to-front (the pool stores
/// the canonical, lower-to-higher-node-index direction).
pub(crate) const FLAG_REVERSED: u8 = 1 << 0;

/// Raw sections decoded from a `.graph` buffer, before semantic validation
/// (which happens in `Graph::assemble`).
#[derive(Debug)]
pub(crate) struct Parts {
    pub flags: u16,
    /// `[min_lat, min_lon, max_lat, max_lon]`, fixed-point 1e7.
    pub bbox_fixed: [i32; 4],
    /// Per node `[lat, lon]`, fixed-point 1e7.
    pub nodes: Vec<[i32; 2]>,
    pub offsets: Vec<u32>,
    pub edges: Vec<Edge>,
    /// Shared intermediate-geometry pool, `[lat, lon]` fixed-point 1e7.
    pub geometry: Vec<[i32; 2]>,
}

#[inline]
fn read_u16(bytes: &[u8], at: usize) -> u16 {
    // Callers guarantee bounds; slicing a checked range keeps this panic-free
    // in practice and obvious in review.
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

#[inline]
fn read_u32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

#[inline]
fn read_i32(bytes: &[u8], at: usize) -> i32 {
    read_u32(bytes, at) as i32
}

/// Decode the byte-level structure of a `.graph` buffer.
///
/// Only structural properties are checked here (magic, version, exact section
/// sizes, unknown flag bits); semantic invariants (CSR monotonicity,
/// coordinate ranges, geometry references, …) are validated when the `Graph`
/// is assembled.
pub(crate) fn parse(bytes: &[u8]) -> Result<Parts, GraphError> {
    if bytes.len() < 4 {
        return Err(GraphError::Truncated);
    }
    if bytes[0..4] != MAGIC {
        return Err(GraphError::BadMagic);
    }
    if bytes.len() < 8 {
        return Err(GraphError::Truncated);
    }
    let version = read_u16(bytes, 4);
    if version != VERSION {
        // Version 1 graphs (no geometry block) land here too: rebuild them.
        return Err(GraphError::UnsupportedVersion(version));
    }
    if bytes.len() < HEADER_BYTES {
        return Err(GraphError::Truncated);
    }
    let flags = read_u16(bytes, 6);
    let node_count = read_u32(bytes, 8) as u64;
    let edge_count = read_u32(bytes, 12) as u64;
    let bbox_fixed = [
        read_i32(bytes, 16),
        read_i32(bytes, 20),
        read_i32(bytes, 24),
        read_i32(bytes, 28),
    ];
    let geo_count = read_u32(bytes, 32) as u64;
    if read_u32(bytes, 36) != 0 {
        return Err(GraphError::Malformed("reserved header field is not zero"));
    }

    // All in u64: the counts come from u32 fields, so none of this can
    // overflow (max ≈ 2^32 × 16 < 2^37).
    let expected = HEADER_BYTES as u64
        + node_count * NODE_BYTES as u64
        + (node_count + 1) * 4
        + edge_count * EDGE_BYTES as u64
        + geo_count * GEO_POINT_BYTES as u64;
    if (bytes.len() as u64) < expected {
        return Err(GraphError::Truncated);
    }
    if bytes.len() as u64 > expected {
        return Err(GraphError::Malformed("trailing bytes after geometry section"));
    }

    let mut at = HEADER_BYTES;
    let mut nodes = Vec::with_capacity(node_count as usize);
    for _ in 0..node_count {
        nodes.push([read_i32(bytes, at), read_i32(bytes, at + 4)]);
        at += NODE_BYTES;
    }
    let mut offsets = Vec::with_capacity(node_count as usize + 1);
    for _ in 0..=node_count {
        offsets.push(read_u32(bytes, at));
        at += 4;
    }
    let mut edges = Vec::with_capacity(edge_count as usize);
    for _ in 0..edge_count {
        let edge_flags = bytes[at + 14];
        if edge_flags & !FLAG_REVERSED != 0 {
            return Err(GraphError::Malformed("unknown edge flag bits set"));
        }
        edges.push(Edge {
            target: read_u32(bytes, at),
            length_dm: read_u32(bytes, at + 4),
            geo_off: read_u32(bytes, at + 8),
            geo_len: read_u16(bytes, at + 12),
            reversed: edge_flags & FLAG_REVERSED != 0,
            access: bytes[at + 15],
        });
        at += EDGE_BYTES;
    }
    let mut geometry = Vec::with_capacity(geo_count as usize);
    for _ in 0..geo_count {
        geometry.push([read_i32(bytes, at), read_i32(bytes, at + 4)]);
        at += GEO_POINT_BYTES;
    }

    Ok(Parts { flags, bbox_fixed, nodes, offsets, edges, geometry })
}

/// Encode graph sections as a `.graph` buffer. The inverse of [`parse`]:
/// `parse(&serialize(p))` reproduces `p` exactly, and serializing a graph
/// loaded from bytes reproduces those bytes byte-for-byte (determinism F9).
pub(crate) fn serialize(parts: &Parts) -> Vec<u8> {
    let expected = HEADER_BYTES
        + parts.nodes.len() * NODE_BYTES
        + (parts.nodes.len() + 1) * 4
        + parts.edges.len() * EDGE_BYTES
        + parts.geometry.len() * GEO_POINT_BYTES;
    let mut out = Vec::with_capacity(expected);

    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&parts.flags.to_le_bytes());
    out.extend_from_slice(&(parts.nodes.len() as u32).to_le_bytes());
    out.extend_from_slice(&(parts.edges.len() as u32).to_le_bytes());
    for v in parts.bbox_fixed {
        out.extend_from_slice(&v.to_le_bytes());
    }
    out.extend_from_slice(&(parts.geometry.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // _reserved
    for [lat, lon] in &parts.nodes {
        out.extend_from_slice(&lat.to_le_bytes());
        out.extend_from_slice(&lon.to_le_bytes());
    }
    for off in &parts.offsets {
        out.extend_from_slice(&off.to_le_bytes());
    }
    for e in &parts.edges {
        out.extend_from_slice(&e.target.to_le_bytes());
        out.extend_from_slice(&e.length_dm.to_le_bytes());
        out.extend_from_slice(&e.geo_off.to_le_bytes());
        out.extend_from_slice(&e.geo_len.to_le_bytes());
        out.push(if e.reversed { FLAG_REVERSED } else { 0 });
        out.push(e.access);
    }
    for [lat, lon] in &parts.geometry {
        out.extend_from_slice(&lat.to_le_bytes());
        out.extend_from_slice(&lon.to_le_bytes());
    }

    debug_assert_eq!(out.len(), expected);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tiny_parts() -> Parts {
        Parts {
            flags: 0,
            bbox_fixed: [340000000, 330000000, 350000000, 340000000],
            nodes: vec![[340000000, 330000000], [350000000, 340000000]],
            offsets: vec![0, 1, 2],
            edges: vec![
                Edge { target: 1, length_dm: 42, geo_off: 0, geo_len: 2, reversed: false, access: 0b11 },
                Edge { target: 0, length_dm: 42, geo_off: 0, geo_len: 2, reversed: true, access: 0b11 },
            ],
            geometry: vec![[343000000, 333000000], [347000000, 337000000]],
        }
    }

    #[test]
    fn round_trip_is_exact() {
        let parts = tiny_parts();
        let bytes = serialize(&parts);
        let back = parse(&bytes).unwrap();
        assert_eq!(back.flags, parts.flags);
        assert_eq!(back.bbox_fixed, parts.bbox_fixed);
        assert_eq!(back.nodes, parts.nodes);
        assert_eq!(back.offsets, parts.offsets);
        assert_eq!(back.edges, parts.edges);
        assert_eq!(back.geometry, parts.geometry);
        // Byte-level stability: serialize(parse(b)) == b.
        assert_eq!(serialize(&back), bytes);
    }

    #[test]
    fn bad_magic() {
        let mut bytes = serialize(&tiny_parts());
        bytes[0] = b'X';
        assert!(matches!(parse(&bytes), Err(GraphError::BadMagic)));
    }

    #[test]
    fn old_and_future_versions_are_refused() {
        // A v1 graph (pre-M4 layout) must fail loudly, not be misread.
        let mut bytes = serialize(&tiny_parts());
        bytes[4] = 1;
        assert!(matches!(parse(&bytes), Err(GraphError::UnsupportedVersion(1))));
        bytes[4] = 3;
        assert!(matches!(parse(&bytes), Err(GraphError::UnsupportedVersion(3))));
    }

    #[test]
    fn truncated_everywhere() {
        let bytes = serialize(&tiny_parts());
        // Every strict prefix must fail with Truncated (or BadMagic for <4).
        for cut in 0..bytes.len() {
            let err = parse(&bytes[..cut]).unwrap_err();
            assert!(
                matches!(err, GraphError::Truncated),
                "prefix len {cut} gave {err:?}"
            );
        }
    }

    #[test]
    fn trailing_bytes_rejected() {
        let mut bytes = serialize(&tiny_parts());
        bytes.push(0);
        assert!(matches!(
            parse(&bytes),
            Err(GraphError::Malformed("trailing bytes after geometry section"))
        ));
    }

    #[test]
    fn unknown_edge_flags_and_reserved_field_rejected() {
        let parts = tiny_parts();
        let bytes = serialize(&parts);
        // Flip an unknown flag bit in the first edge record.
        let edge0_flags_at = HEADER_BYTES + parts.nodes.len() * NODE_BYTES
            + (parts.nodes.len() + 1) * 4
            + 14;
        let mut bad = bytes.clone();
        bad[edge0_flags_at] |= 0b10;
        assert!(matches!(parse(&bad), Err(GraphError::Malformed(_))));
        // Non-zero reserved header field.
        let mut bad = bytes;
        bad[36] = 7;
        assert!(matches!(parse(&bad), Err(GraphError::Malformed(_))));
    }

    #[test]
    fn empty_graph_round_trips() {
        let parts = Parts {
            flags: 0,
            bbox_fixed: [0; 4],
            nodes: vec![],
            offsets: vec![0],
            edges: vec![],
            geometry: vec![],
        };
        let bytes = serialize(&parts);
        assert_eq!(bytes.len(), HEADER_BYTES + 4);
        let back = parse(&bytes).unwrap();
        assert!(back.nodes.is_empty());
        assert_eq!(back.offsets, vec![0]);
        assert!(back.geometry.is_empty());
    }
}
