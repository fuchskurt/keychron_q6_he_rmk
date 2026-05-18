use crate::{
    layout::{COL, ROW},
    matrix::analog_matrix::types::KeyEntry,
};
use core::mem::size_of;
use embassy_stm32::crc::Crc;

/// Pre-computed buffer length for the HE matrix.
pub const CALIB_BUF_LEN: usize = total_len(ROW, COL);
/// Byte length of the trailing CRC field.
const CRC_LEN: usize = size_of::<u32>();
/// EEPROM word address at which the calibration block begins.
pub const EEPROM_BASE_ADDR: u16 = 0x0000;
/// Byte length of a single serialized entry (one u16 full-travel value).
const ENTRY_LEN: usize = size_of::<u16>();
/// Byte length of the header: magic + version.
const HEADER_LEN: usize = size_of::<u32>().saturating_add(size_of::<u8>());
/// Format version. Increment on any incompatible layout change to force a
/// first-boot re-calibration when old EEPROM data is found.
const VERSION: u8 = 2;
/// Magic number identifying a valid Q6 HE calibration block.
const MAGIC: u32 = 0x5136_4845;

/// Assemble a little-endian `u16` from its two bytes.
///
/// Spelled out with explicit shifts and masks rather than `u16::from_le_bytes`
/// so the byte order is visible at the definition site.
const fn le_u16(bytes: [u8; 2]) -> u16 {
    let [b0, b1] = bytes;
    u16::from(b0) | u16::from(b1).wrapping_shl(8)
}

/// Assemble a little-endian `u32` from its four bytes.
///
/// Spelled out with explicit shifts and masks rather than `u32::from_le_bytes`
/// so the byte order is visible at the definition site.
const fn le_u32(bytes: [u8; 4]) -> u32 {
    let [b0, b1, b2, b3] = bytes;
    u32::from(b0) | u32::from(b1).wrapping_shl(8) | u32::from(b2).wrapping_shl(16) | u32::from(b3).wrapping_shl(24)
}

/// Split a `u16` into its two little-endian bytes (LSB first).
const fn le_bytes_u16(value: u16) -> [u8; 2] {
    [u8::try_from(value & 0xFF).unwrap_or(0), u8::try_from(value.wrapping_shr(8)).unwrap_or(0)]
}

/// Split a `u32` into its four little-endian bytes (LSB first).
const fn le_bytes_u32(value: u32) -> [u8; 4] {
    [
        u8::try_from(value & 0xFF).unwrap_or(0),
        u8::try_from(value.wrapping_shr(8) & 0xFF).unwrap_or(0),
        u8::try_from(value.wrapping_shr(16) & 0xFF).unwrap_or(0),
        u8::try_from(value.wrapping_shr(24) & 0xFF).unwrap_or(0),
    ]
}

/// Copy exactly `N` bytes from `buf[start..end]` into a fixed-size array.
///
/// Returns `None` if the range is out of bounds or its length is not `N`.
/// Callers that need a hard failure (e.g. corrupt EEPROM data) should
/// treat `None` as a validation error rather than substituting a default,
/// because zero is a valid output for checksums and serialized values.
#[inline]
fn read_array<const N: usize>(buf: &[u8], start: usize, end: usize) -> Option<[u8; N]> {
    if let Some(src) = buf.get(start..end)
        && let Ok(fixed) = <[u8; N]>::try_from(src)
    {
        return Some(fixed);
    }
    None
}

/// Compute CRC-32 over `data` using the STM32 hardware CRC peripheral.
///
/// Feeds data as 32-bit little-endian words. If `data.len()` is not a
/// multiple of 4 the final word is zero-padded before feeding.
fn crc32_of(crc: &mut Crc<'_>, data: &[u8]) -> u32 {
    crc.reset();
    let (chunks, remainder) = data.as_chunks::<4>();
    for &chunk in chunks {
        crc.feed_word(le_u32(chunk));
    }
    if !remainder.is_empty() {
        let mut last = [0_u8; 4];
        if let Some(dst) = last.get_mut(..remainder.len()) {
            dst.copy_from_slice(remainder);
        }
        crc.feed_word(le_u32(last));
    }
    crc.read()
}

/// Serialize `keys` (column-major `[[KeyEntry; ROW]; COL]`) into `buf`.
///
/// Entries are written in column-major order (col outer, row inner) matching
/// the in-memory layout, so no transposition is needed.
///
/// Format: magic (4 B LE) | version (1 B) | entries (COL×ROW×2 B LE) | CRC-32
/// (4 B LE). `buf` must be at least `total_len(ROW, COL)` bytes (see
/// [`total_len`]).
pub fn serialize<const ROW: usize, const COL: usize>(
    keys: &[[KeyEntry; ROW]; COL],
    buf: &mut [u8; CALIB_BUF_LEN],
    crc: &mut Crc<'_>,
) {
    // Write magic number, 4 bytes little-endian.
    if let Some(dst) = buf.get_mut(0..size_of::<u32>()) {
        dst.copy_from_slice(&le_bytes_u32(MAGIC));
    }
    // Write version byte.
    if let Some(version_byte) = buf.get_mut(size_of::<u32>()) {
        *version_byte = VERSION;
    }
    let mut pos = HEADER_LEN;
    for key_col in keys {
        for key in key_col {
            let end = pos.saturating_add(ENTRY_LEN);
            if let Some(dst) = buf.get_mut(pos..end) {
                dst.copy_from_slice(&le_bytes_u16(key.entry_full));
            }
            pos = end;
        }
    }
    // Compute CRC over the header + all entry bytes.
    let crc_start = pos;
    let crc_end = crc_start.saturating_add(CRC_LEN);
    let checksum = buf.get(..crc_start).map_or(0, |data| crc32_of(crc, data));
    if let Some(dst) = buf.get_mut(crc_start..crc_end) {
        dst.copy_from_slice(&le_bytes_u32(checksum));
    }
}

/// Attempt to deserialize a calibration block from `buf` into `out`.
///
/// Validates the magic number, version byte, and CRC-32 checksum.
/// Returns `true` and populates [`KeyEntry::entry_full`] for every position
/// in `out` on success. Returns `false` without modifying `out` on any
/// validation failure, including a VERSION mismatch which rejects data
/// written by an incompatible layout.
pub fn try_deserialize<const ROW: usize, const COL: usize>(
    buf: &[u8],
    out: &mut [[KeyEntry; ROW]; COL],
    crc: &mut Crc<'_>,
) -> bool {
    if buf.len() < CALIB_BUF_LEN {
        return false;
    }
    // Validate magic number.
    let magic_end = size_of::<u32>();
    let Some(magic_bytes) = read_array::<4>(buf, 0, magic_end) else { return false };
    if le_u32(magic_bytes) != MAGIC {
        return false;
    }
    // Validate version byte.
    let Some(&stored_version) = buf.get(size_of::<u32>()) else { return false };
    if stored_version != VERSION {
        return false;
    }
    // Validate CRC over header + entries.
    let crc_end = total_len(ROW, COL);
    let data_end = crc_end.saturating_sub(CRC_LEN);
    // None must not be silently replaced with 0 (0 is a valid CRC value).
    let Some(stored_crc_bytes) = read_array::<4>(buf, data_end, crc_end) else { return false };
    let stored_crc = le_u32(stored_crc_bytes);
    let computed_crc = buf.get(..data_end).map_or(0, |data| crc32_of(crc, data));
    if computed_crc != stored_crc {
        return false;
    }
    // Deserialize entries directly into each key's persistent calibration slot.
    let mut pos = HEADER_LEN;
    for key_col in out.iter_mut() {
        for key in key_col.iter_mut() {
            let end = pos.saturating_add(ENTRY_LEN);
            let Some(fb) = read_array::<2>(buf, pos, end) else { return false };
            key.entry_full = le_u16(fb);
            pos = end;
        }
    }
    true
}

/// Compute the total serialized byte length for a `rows × cols` matrix.
pub const fn total_len(rows: usize, cols: usize) -> usize {
    HEADER_LEN.saturating_add(rows.saturating_mul(cols).saturating_mul(ENTRY_LEN)).saturating_add(CRC_LEN)
}
