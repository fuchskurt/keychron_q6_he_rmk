//! Endianness-explicit byte (de)serialisation primitives for the EEPROM
//! calibration block and the I²C device addressing.
//!
//! The byte order is spelled out with shifts and masks rather than the
//! `from_le_bytes`/`to_le_bytes` family so the wire layout is visible at the
//! definition site and unit-testable on the host.

/// Split a 16-bit word address into its two big-endian bytes (MSB first), as
/// required by the FT24C64 addressing protocol.
#[must_use]
#[inline]
pub const fn be_bytes_u16(value: u16) -> [u8; 2] {
    let hi = u8::try_from(value.wrapping_shr(8)).unwrap_or(0);
    let lo = u8::try_from(value & 0xFF).unwrap_or(0);
    [hi, lo]
}

/// Split a `u16` into its two little-endian bytes (LSB first).
#[must_use]
#[inline]
pub const fn le_bytes_u16(value: u16) -> [u8; 2] {
    [u8::try_from(value & 0xFF).unwrap_or(0), u8::try_from(value.wrapping_shr(8)).unwrap_or(0)]
}

/// Split a `u32` into its four little-endian bytes (LSB first).
#[must_use]
#[inline]
pub const fn le_bytes_u32(value: u32) -> [u8; 4] {
    [
        u8::try_from(value & 0xFF).unwrap_or(0),
        u8::try_from(value.wrapping_shr(8) & 0xFF).unwrap_or(0),
        u8::try_from(value.wrapping_shr(16) & 0xFF).unwrap_or(0),
        u8::try_from(value.wrapping_shr(24) & 0xFF).unwrap_or(0),
    ]
}

/// Assemble a little-endian `u16` from its two bytes.
#[must_use]
#[inline]
pub const fn le_u16(bytes: [u8; 2]) -> u16 {
    let [b0, b1] = bytes;
    u16::from(b0) | u16::from(b1).wrapping_shl(8)
}

/// Assemble a little-endian `u32` from its four bytes.
#[must_use]
#[inline]
pub const fn le_u32(bytes: [u8; 4]) -> u32 {
    let [b0, b1, b2, b3] = bytes;
    u32::from(b0) | u32::from(b1).wrapping_shl(8) | u32::from(b2).wrapping_shl(16) | u32::from(b3).wrapping_shl(24)
}

/// Copy exactly `N` bytes from `buf[start..end]` into a fixed-size array.
///
/// Returns `None` if the range is out of bounds or its length is not `N`.
/// Callers that need a hard failure (e.g. corrupt EEPROM data) should treat
/// `None` as a validation error rather than substituting a default, because
/// zero is a valid output for checksums and serialized values.
#[must_use]
#[inline]
pub fn read_array<const N: usize>(buf: &[u8], start: usize, end: usize) -> Option<[u8; N]> {
    if let Some(src) = buf.get(start..end)
        && let Ok(fixed) = <[u8; N]>::try_from(src)
    {
        return Some(fixed);
    }
    None
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{be_bytes_u16, le_bytes_u16, le_bytes_u32, le_u16, le_u32, read_array};

    #[test]
    fn le_u16_round_trips() {
        for value in [0_u16, 1, 0x00FF, 0x0100, 0xABCD, u16::MAX] {
            assert_eq!(le_u16(le_bytes_u16(value)), value);
        }
        assert_eq!(le_bytes_u16(0xABCD), [0xCD, 0xAB]);
        assert_eq!(le_u16([0xCD, 0xAB]), 0xABCD);
    }

    #[test]
    fn le_u32_round_trips() {
        for value in [0_u32, 1, 0xFF, 0x0100, 0x1234_5678, u32::MAX] {
            assert_eq!(le_u32(le_bytes_u32(value)), value);
        }
        assert_eq!(le_bytes_u32(0x1234_5678), [0x78, 0x56, 0x34, 0x12]);
        assert_eq!(le_u32([0x78, 0x56, 0x34, 0x12]), 0x1234_5678);
    }

    #[test]
    fn be_bytes_u16_is_msb_first() {
        assert_eq!(be_bytes_u16(0x0000), [0x00, 0x00]);
        assert_eq!(be_bytes_u16(0x00FF), [0x00, 0xFF]);
        assert_eq!(be_bytes_u16(0xAB12), [0xAB, 0x12]);
        assert_eq!(be_bytes_u16(u16::MAX), [0xFF, 0xFF]);
    }

    #[test]
    fn read_array_bounds_and_length() {
        let buf = [10_u8, 20, 30, 40, 50];
        assert_eq!(read_array::<2>(&buf, 1, 3), Some([20, 30]));
        assert_eq!(read_array::<4>(&buf, 0, 4), Some([10, 20, 30, 40]));
        // Out of range.
        assert_eq!(read_array::<2>(&buf, 4, 7), None);
        // Range length does not match N.
        assert_eq!(read_array::<2>(&buf, 0, 3), None);
        // Reversed range yields an empty/invalid slice, not a panic.
        assert_eq!(read_array::<1>(&buf, 3, 1), None);
    }

    #[test]
    fn le_helpers_are_const() {
        const BYTES: [u8; 4] = le_bytes_u32(0xDEAD_BEEF);
        const BACK: u32 = le_u32(BYTES);
        assert_eq!(BACK, 0xDEAD_BEEF);
    }
}
