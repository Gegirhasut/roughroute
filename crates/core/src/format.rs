//! The `RRG1` binary `.graph` format, version 3 (spec §7 + M4 geometry block
//! D12 + M6 delta-compression D19). Little-endian.
//!
//! Layout (all offsets from the start of the buffer):
//!
//! | section  | bytes | content |
//! |---|---|---|
//! | header   | 40 | see below |
//! | nodes    | `node_count × 8` | per node: `lat: i32`, `lon: i32` fixed-point 1e7 |
//! | offsets  | `(node_count + 1) × 4` | CSR row offsets into the edge section |
//! | edges    | `edge_count × 16` | per edge: `target: u32`, `length_dm: u32`, `geo_off: u32`, `geo_len: u16`, `flags: u8`, `access: u8` |
//! | geometry | `geo_bytes` (variable) | zigzag-delta LEB128 varints decoding to `geo_point_count` `[lat, lon]` fixed-point points |
//!
//! Header: `magic: [u8;4] = "RRG1"`, `version: u16 = 3`, `flags: u16`,
//! `node_count: u32`, `edge_count: u32`, `min_lat, min_lon, max_lat,
//! max_lon: i32` (fixed-point 1e7 bbox over nodes *and* geometry),
//! `geo_point_count: u32` (decoded point count), `geo_bytes: u32` (exact byte
//! length of the geometry section — was an always-zero `_reserved` field in
//! v2, repurposed here; header size is unchanged at 40 bytes).
//!
//! An edge's intermediate shape is `geometry[geo_off .. geo_off + geo_len]`
//! of the *decoded* point array — `geo_off`/`geo_len` semantics are
//! unchanged from v2 (docs/DECISIONS.md D12); only the on-disk encoding of
//! the pool itself changed (D19). The two directed edges of one road
//! segment share a single pool range stored in the canonical direction
//! (lower node index → higher); the opposite direction sets edge flag bit0
//! (`FLAG_REVERSED`). Uncollapsed edges have `geo_len = 0`, `geo_off = 0`,
//! `flags = 0`.
//!
//! # Geometry delta encoding (D19)
//!
//! Point 0's `lat`/`lon` are each zigzag+LEB128-varint-encoded directly
//! (delta from the origin); point *i* (`i > 0`) is the zigzag-varint of
//! `(lat_i − lat_{i-1}, lon_i − lon_{i-1})` — the delta from the
//! *immediately preceding point in array order*, independent of which
//! edge/chain either point belongs to (no per-chain anchor/reset — see D19
//! for why that simpler scheme was chosen over a per-chain-anchored one).
//!
//! Decoding happens once, at load, straight into the same absolute
//! `Vec<[i32; 2]>` the v2 format stored directly — nothing downstream
//! (`graph.rs`, `grid.rs`, `router.rs`) is aware the on-disk encoding
//! changed.
//!
//! Versions 1 and 2 (pre-delta layouts) are **refused** with
//! [`GraphError::UnsupportedVersion`] — old graphs must be rebuilt (spec
//! §13, same discipline as the v1→v2 bump).
//!
//! # Alignment (kept zero-copy-ready, D2)
//!
//! The header is exactly 40 bytes and every *fixed-width* section (nodes,
//! offsets, edges) starts 4-byte aligned relative to the buffer start. The
//! geometry section is now variable-length (delta-varint), so it is no
//! longer a candidate for zero-copy regardless — v3 parses into owned
//! `Vec`s exactly as v1/v2 did (D2).

use crate::graph::{Edge, GraphError};

/// The magic bytes opening every `.graph` file.
pub const MAGIC: [u8; 4] = *b"RRG1";
/// The format version this crate reads and writes.
pub const VERSION: u16 = 3;
/// Size of the fixed header in bytes.
pub const HEADER_BYTES: usize = 40;
/// Size of one node record (two fixed-point `i32` coordinates).
pub const NODE_BYTES: usize = 8;
/// Size of one edge record.
pub const EDGE_BYTES: usize = 16;

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
    /// Shared intermediate-geometry pool, `[lat, lon]` fixed-point 1e7
    /// (decoded; the on-disk encoding is delta-varint, D19).
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

/// Zigzag-encode a signed value into an unsigned one so small negative *and*
/// positive deltas both produce small varints.
#[inline]
fn zigzag_encode(v: i32) -> u32 {
    ((v << 1) ^ (v >> 31)) as u32
}

#[inline]
fn zigzag_decode(v: u32) -> i32 {
    ((v >> 1) as i32) ^ -((v & 1) as i32)
}

/// Append `v` as a LEB128 varint (7 payload bits/byte, high bit = more bytes
/// follow).
fn write_varint(out: &mut Vec<u8>, mut v: u32) {
    loop {
        let byte = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            out.push(byte);
            return;
        }
        out.push(byte | 0x80);
    }
}

/// Read one LEB128 varint starting at `*at`, advancing `*at` past it. A u32
/// needs at most 5 continuation bytes; a 6th means malformed input rather
/// than a shift overflow.
fn read_varint(bytes: &[u8], at: &mut usize) -> Result<u32, GraphError> {
    let mut result: u32 = 0;
    for i in 0..5u32 {
        let &byte = bytes.get(*at).ok_or(GraphError::Truncated)?;
        *at += 1;
        result |= u32::from(byte & 0x7f) << (7 * i);
        if byte & 0x80 == 0 {
            return Ok(result);
        }
    }
    Err(GraphError::Malformed("geometry varint too long"))
}

/// Encode the geometry pool as sequential zigzag-delta varints (D19).
fn encode_geometry(points: &[[i32; 2]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(points.len() * 2);
    let mut prev = [0i32, 0i32];
    for &[lat, lon] in points {
        write_varint(&mut out, zigzag_encode(lat.wrapping_sub(prev[0])));
        write_varint(&mut out, zigzag_encode(lon.wrapping_sub(prev[1])));
        prev = [lat, lon];
    }
    out
}

/// Decode `count` zigzag-delta points from `bytes`, which must contain
/// exactly that many points with no leftover bytes.
fn decode_geometry(bytes: &[u8], count: u32) -> Result<Vec<[i32; 2]>, GraphError> {
    // Every point costs at least 2 bytes (a 1-byte varint per coordinate at
    // minimum); clamp the capacity hint so a bogus `count` can't force a huge
    // allocation before the mismatch is even detected.
    let capacity_hint = (bytes.len() / 2).min(count as usize);
    let mut out = Vec::with_capacity(capacity_hint);
    let mut at = 0usize;
    let mut prev = [0i32, 0i32];
    for _ in 0..count {
        let dlat = zigzag_decode(read_varint(bytes, &mut at)?);
        let dlon = zigzag_decode(read_varint(bytes, &mut at)?);
        let point = [prev[0].wrapping_add(dlat), prev[1].wrapping_add(dlon)];
        out.push(point);
        prev = point;
    }
    if at != bytes.len() {
        return Err(GraphError::Malformed("geometry section has trailing bytes"));
    }
    Ok(out)
}

/// Decode the byte-level structure of a `.graph` buffer.
///
/// Only structural properties are checked here (magic, version, exact section
/// sizes, unknown flag bits, well-formed varints); semantic invariants (CSR
/// monotonicity, coordinate ranges, geometry references, …) are validated
/// when the `Graph` is assembled.
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
        // v1 (no geometry) and v2 (fixed-width geometry) graphs both land
        // here: rebuild them with the current tool.
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
    let geo_point_count = read_u32(bytes, 32);
    let geo_bytes = read_u32(bytes, 36) as u64;

    // All in u64: the fixed-width counts come from u32 fields and geo_bytes
    // is itself a u32, so none of this can overflow (max well under 2^40).
    let expected = HEADER_BYTES as u64
        + node_count * NODE_BYTES as u64
        + (node_count + 1) * 4
        + edge_count * EDGE_BYTES as u64
        + geo_bytes;
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
    // The overall length check above guarantees exactly geo_bytes remain.
    let geometry = decode_geometry(&bytes[at..], geo_point_count)?;

    Ok(Parts { flags, bbox_fixed, nodes, offsets, edges, geometry })
}

/// Encode graph sections as a `.graph` buffer. The inverse of [`parse`]:
/// `parse(&serialize(p))` reproduces `p` exactly, and serializing a graph
/// loaded from bytes reproduces those bytes byte-for-byte (determinism F9).
pub(crate) fn serialize(parts: &Parts) -> Vec<u8> {
    let geometry_bytes = encode_geometry(&parts.geometry);
    let expected = HEADER_BYTES
        + parts.nodes.len() * NODE_BYTES
        + (parts.nodes.len() + 1) * 4
        + parts.edges.len() * EDGE_BYTES
        + geometry_bytes.len();
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
    out.extend_from_slice(&(geometry_bytes.len() as u32).to_le_bytes());
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
    out.extend_from_slice(&geometry_bytes);

    debug_assert_eq!(out.len(), expected);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

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
    fn old_versions_are_refused() {
        // v1 (no geometry) and v2 (fixed-width geometry) must both fail
        // loudly rather than being misread as v3 delta-varints.
        let mut bytes = serialize(&tiny_parts());
        bytes[4] = 1;
        assert!(matches!(parse(&bytes), Err(GraphError::UnsupportedVersion(1))));
        bytes[4] = 2;
        assert!(matches!(parse(&bytes), Err(GraphError::UnsupportedVersion(2))));
        bytes[4] = 4;
        assert!(matches!(parse(&bytes), Err(GraphError::UnsupportedVersion(4))));
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
    fn unknown_edge_flags_rejected() {
        let parts = tiny_parts();
        let bytes = serialize(&parts);
        // Flip an unknown flag bit in the first edge record.
        let edge0_flags_at = HEADER_BYTES + parts.nodes.len() * NODE_BYTES
            + (parts.nodes.len() + 1) * 4
            + 14;
        let mut bad = bytes;
        bad[edge0_flags_at] |= 0b10;
        assert!(matches!(parse(&bad), Err(GraphError::Malformed(_))));
    }

    #[test]
    fn corrupt_varint_in_geometry_is_rejected_not_panicking() {
        let bytes = serialize(&tiny_parts());
        // Force every geometry byte to be a "continuation" byte (high bit
        // set): decode must error (varint too long / truncated), never loop
        // forever or panic on a shift.
        let mut bad = bytes.clone();
        for b in bad.iter_mut().skip(HEADER_BYTES + 24 + 12 + 32) {
            *b |= 0x80;
        }
        assert!(parse(&bad).is_err());

        // Truncate mid-varint (drop the last geometry byte, which is a
        // trailing continuation byte's payload after the flip above, or in
        // the original bytes just a lone final byte): must be Truncated, not
        // a panic.
        let mut short = bytes;
        short.pop();
        assert!(matches!(parse(&short), Err(GraphError::Truncated)));
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

    #[test]
    fn varint_round_trips_full_i32_range() {
        for v in [0i32, 1, -1, 63, -63, 64, -64, 8191, -8191, 8192, i32::MAX, i32::MIN, -1000, 12345] {
            let mut out = Vec::new();
            write_varint(&mut out, zigzag_encode(v));
            let mut at = 0;
            let decoded = zigzag_decode(read_varint(&out, &mut at).unwrap());
            assert_eq!(decoded, v, "round trip failed for {v}");
            assert_eq!(at, out.len());
        }
    }

    #[test]
    fn small_deltas_are_compact() {
        // A chain of points ~10 m apart (typical OSM way-shape spacing) must
        // compress well below the v2 fixed 8 bytes/point.
        let points: Vec<[i32; 2]> = (0..100).map(|i| [340000000 + i * 900, 330000000 + i * 3]).collect();
        let encoded = encode_geometry(&points);
        assert!(
            encoded.len() < points.len() * 4,
            "expected well under 4 bytes/point, got {} bytes for {} points",
            encoded.len(),
            points.len()
        );
        assert_eq!(decode_geometry(&encoded, points.len() as u32).unwrap(), points);
    }

    proptest! {
        /// Round-trip fidelity for arbitrary geometry pools, not just the
        /// hand-picked cases above: encode-then-decode must reproduce the
        /// exact input for any point sequence (this is the correctness
        /// backbone of D19 — compression must never change a coordinate).
        #[test]
        fn geometry_round_trips_for_arbitrary_points(
            points in prop::collection::vec((any::<i32>(), any::<i32>()), 0..200)
        ) {
            let points: Vec<[i32; 2]> = points.into_iter().map(|(a, b)| [a, b]).collect();
            let encoded = encode_geometry(&points);
            let decoded = decode_geometry(&encoded, points.len() as u32).unwrap();
            prop_assert_eq!(decoded, points);
        }

        /// A truncated or randomly mutated geometry section must error, never
        /// panic (the "no panics in library code" discipline applied to
        /// adversarial/corrupt on-disk bytes).
        #[test]
        fn geometry_decode_never_panics_on_arbitrary_bytes(
            bytes in prop::collection::vec(any::<u8>(), 0..64),
            count in 0u32..20,
        ) {
            let _ = decode_geometry(&bytes, count);
        }
    }
}
