//! Versioned, quota-bounded binary envelope.
#![forbid(unsafe_code)]
/// Format magic.
pub const MAGIC: [u8; 4] = *b"AURA";
/// Current format version.
pub const VERSION: u16 = 1;
/// Default maximum payload (64 MiB).
pub const MAX_PAYLOAD: usize = 64 * 1024 * 1024;
/// Parsing error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Header is truncated.
    Truncated,
    /// Magic is wrong.
    BadMagic,
    /// Version is unsupported.
    UnsupportedVersion,
    /// Declared length is impossible or over quota.
    InvalidLength,
}
/// Encodes a deterministic little-endian envelope.
#[must_use]
pub fn encode(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(10 + payload.len());
    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    out.extend_from_slice(payload);
    out
}
/// Decodes an envelope with an explicit allocation/size quota. The returned slice borrows input.
pub fn decode(input: &[u8], quota: usize) -> Result<&[u8], Error> {
    if input.len() < 10 {
        return Err(Error::Truncated);
    }
    if input[..4] != MAGIC {
        return Err(Error::BadMagic);
    }
    let version = u16::from_le_bytes([input[4], input[5]]);
    if version != VERSION {
        return Err(Error::UnsupportedVersion);
    }
    let n = u32::from_le_bytes([input[6], input[7], input[8], input[9]]) as usize;
    if n > quota || n > MAX_PAYLOAD || n != input.len() - 10 {
        return Err(Error::InvalidLength);
    }
    Ok(&input[10..])
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn round_trip() {
        let x = encode(b"state");
        assert_eq!(decode(&x, 100), Ok(&b"state"[..]));
    }
    #[test]
    fn hostile_counts_bounded() {
        let mut x = encode(&[]);
        x[6..10].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(decode(&x, 16), Err(Error::InvalidLength));
    }
    #[test]
    fn every_truncation_fails() {
        let x = encode(b"abc");
        for n in 0..x.len() {
            assert!(decode(&x[..n], 100).is_err());
        }
    }
}
