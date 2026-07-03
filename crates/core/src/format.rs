//! The `RRG1` binary `.graph` format, version 1. Little-endian.
//!
//! Layout (all offsets from the start of the buffer):
//!
//! | section | bytes | content |
//! |---|---|---|
//! | header  | 32 | see below |
//! | nodes   | `node_count × 8` | per node: `lat: i32`, `lon: i32` fixed-point 1e7 |
//! | offsets | `(node_count+1) × 4` | CSR row offsets into the edge section |
//! | edges   | `edge_count × 12` | per edge: `target: u32`, `length_dm: u32`, `access: u8`, `_pad: [u8;3]` |
//!
//! Header: `magic: [u8;4] = "RRG1"`, `version: u16 = 1`, `flags: u16`,
//! `node_count: u32`, `edge_count: u32`, `min_lat, min_lon, max_lat,
//! max_lon: i32` (fixed-point 1e7 bbox).
//!
//! # Alignment
//!
//! The header is exactly 32 bytes and every section size is a multiple of 4,
//! so each section starts 4-byte aligned relative to the buffer start. v1
//! nevertheless parses into owned `Vec`s and accepts a buffer of any
//! alignment.

use crate::graph::{Edge, GraphError};

/// The magic bytes opening every `.graph` file.
pub const MAGIC: [u8; 4] = *b"RRG1";
/// The format version this crate reads and writes.
pub const VERSION: u16 = 1;
/// Size of the fixed header in bytes.
pub const HEADER_BYTES: usize = 32;
/// Size of one node record (two fixed-point `i32` coordinates).
pub const NODE_BYTES: usize = 8;
/// Size of one edge record, including padding.
pub const EDGE_BYTES: usize = 12;

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
/// sizes); semantic invariants (CSR monotonicity, coordinate ranges, …) are
/// validated when the `Graph` is assembled.
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

    // All in u64: the counts come from u32 fields, so none of this can
    // overflow (max ≈ 2^32 × 12 < 2^36).
    let nodes_len = node_count * NODE_BYTES as u64;
    let offsets_len = (node_count + 1) * 4;
    let edges_len = edge_count * EDGE_BYTES as u64;
    let expected = HEADER_BYTES as u64 + nodes_len + offsets_len + edges_len;
    if (bytes.len() as u64) < expected {
        return Err(GraphError::Truncated);
    }
    if bytes.len() as u64 > expected {
        return Err(GraphError::Malformed("trailing bytes after edge section"));
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
        edges.push(Edge {
            target: read_u32(bytes, at),
            length_dm: read_u32(bytes, at + 4),
            access: bytes[at + 8],
            // bytes[at+9..at+12] are `_pad`, ignored on read by design.
        });
        at += EDGE_BYTES;
    }

    Ok(Parts { flags, bbox_fixed, nodes, offsets, edges })
}

/// Encode graph sections as a `.graph` buffer. The inverse of [`parse`]:
/// `parse(&serialize(p))` reproduces `p` exactly, and serializing a graph
/// loaded from bytes reproduces those bytes (padding is written as zeros).
pub(crate) fn serialize(parts: &Parts) -> Vec<u8> {
    let expected = HEADER_BYTES
        + parts.nodes.len() * NODE_BYTES
        + (parts.nodes.len() + 1) * 4
        + parts.edges.len() * EDGE_BYTES;
    let mut out = Vec::with_capacity(expected);

    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&parts.flags.to_le_bytes());
    out.extend_from_slice(&(parts.nodes.len() as u32).to_le_bytes());
    out.extend_from_slice(&(parts.edges.len() as u32).to_le_bytes());
    for v in parts.bbox_fixed {
        out.extend_from_slice(&v.to_le_bytes());
    }
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
        out.push(e.access);
        out.extend_from_slice(&[0u8; 3]); // `_pad`: always zeros
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
                Edge { target: 1, length_dm: 42, access: 0b11 },
                Edge { target: 0, length_dm: 42, access: 0b11 },
            ],
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
    fn unsupported_version() {
        let mut bytes = serialize(&tiny_parts());
        bytes[4] = 2;
        assert!(matches!(parse(&bytes), Err(GraphError::UnsupportedVersion(2))));
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
            Err(GraphError::Malformed("trailing bytes after edge section"))
        ));
    }

    #[test]
    fn empty_graph_round_trips() {
        let parts = Parts {
            flags: 0,
            bbox_fixed: [0; 4],
            nodes: vec![],
            offsets: vec![0],
            edges: vec![],
        };
        let bytes = serialize(&parts);
        assert_eq!(bytes.len(), HEADER_BYTES + 4);
        let back = parse(&bytes).unwrap();
        assert!(back.nodes.is_empty());
        assert_eq!(back.offsets, vec![0]);
    }
}
