//! Byte-slice helper used by the EEPROM calibration block.
//!
//! Endianness-specific conversions use the standard library
//! [`u16::to_le_bytes`], [`u16::to_be_bytes`], [`u32::to_le_bytes`],
//! [`u16::from_le_bytes`], and [`u32::from_le_bytes`] family directly at the
//! call site, so this module only carries the slice-to-array helper.
#![expect(
    clippy::arbitrary_source_item_ordering,
    reason = "trailing #[cfg(test)] module by convention"
)]

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
#[expect(
    clippy::inline_modules,
    clippy::tests_outside_test_module,
    reason = "inline #[cfg(test)] module kept next to the code it tests"
)]
mod tests {
    use super::read_array;

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
}
