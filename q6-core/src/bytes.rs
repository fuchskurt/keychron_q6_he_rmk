//! Byte-slice helper used by the EEPROM calibration block.
//!
//! Endianness-specific conversions use the standard library
//! [`u16::to_le_bytes`], [`u16::to_be_bytes`], [`u32::to_le_bytes`],
//! [`u16::from_le_bytes`], and [`u32::from_le_bytes`] family directly at the
//! call site, so this module only carries the slice-to-array helper.

#[cfg(test)] mod tests;

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
