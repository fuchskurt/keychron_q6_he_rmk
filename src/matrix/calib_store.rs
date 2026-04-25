use crate::layout::{COL, ROW};
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
/// Format version. Must be incremented on any incompatible layout change.
const VERSION: u8 = 1;
/// Magic number identifying a valid Q6 HE calibration block.
const MAGIC: u32 = 0x5136_4845;

/// Per-key full-travel calibration value stored in EEPROM.
#[derive(Clone, Copy)]
pub struct CalibEntry {
    /// Raw ADC reading at full travel (key fully depressed to the PCB).
    pub full: u16,
}

/// Copy exactly `N` bytes from `buf[start..end]` into a fixed-size array.
///
/// Returns `None` if the range is out of bounds or its length is not `N`.
/// Callers that need a hard failure (e.g. corrupt EEPROM data) should
/// treat `None` as a validation error rather than substituting a default,
/// because zero is a valid output for checksums and serialized values.
#[inline]
fn read_array<const N: usize>(buf: &[u8], start: usize, end: usize) -> Option<[u8; N]> {
    buf.get(start..end)?.try_into().ok()
}

/// Compute CRC-32 over `data` using the STM32 hardware CRC peripheral.
///
/// Feeds data as 32-bit little-endian words. If `data.len()` is not a
/// multiple of 4 the final word is zero-padded before feeding.
fn crc32_of(crc: &mut Crc<'_>, data: &[u8]) -> u32 {
    crc.reset();
    let mut chunks = data.chunks_exact(4);
    for chunk in chunks.by_ref() {
        // chunks_exact guarantees 4 bytes; unwrap_or is unreachable.
        crc.feed_word(u32::from_le_bytes(chunk.try_into().unwrap_or([0u8; 4])));
    }
    let remainder = chunks.remainder();
    if !remainder.is_empty() {
        let mut last = [0u8; 4];
        last[..remainder.len()].copy_from_slice(remainder);
        crc.feed_word(u32::from_le_bytes(last));
    }
    crc.read()
}

/// Serialize `entries` (row-major, `ROW × COL`) into `buf`.
///
/// Writes the magic number, version byte, all full-travel entries, and a
/// CRC-32 checksum over all preceding bytes.
/// `buf` must be at least [`total_len`]`(ROW, COL)` bytes.
pub fn serialize<const ROW: usize, const COL: usize>(
    entries: &[[CalibEntry; COL]; ROW],
    buf: &mut [u8; CALIB_BUF_LEN],
    crc: &mut Crc<'_>,
) {
    // Write magic number, 4 bytes little-endian.
    if let Some(dst) = buf.get_mut(0..size_of::<u32>()) {
        dst.copy_from_slice(&MAGIC.to_le_bytes());
    }

    // Write version byte.
    if let Some(version_byte) = buf.get_mut(size_of::<u32>()) {
        *version_byte = VERSION;
    }

    let mut pos = HEADER_LEN;
    for row in entries {
        for entry in row {
            let end = pos.saturating_add(ENTRY_LEN);
            if let Some(dst) = buf.get_mut(pos..end) {
                dst.copy_from_slice(&entry.full.to_le_bytes());
            }
            pos = end;
        }
    }

    // Compute CRC over the header + all entry bytes.
    let crc_start = pos;
    let crc_end = crc_start.saturating_add(CRC_LEN);
    let checksum = buf.get(..crc_start).map_or(0, |data| crc32_of(crc, data));
    if let Some(dst) = buf.get_mut(crc_start..crc_end) {
        dst.copy_from_slice(&checksum.to_le_bytes());
    }
}

/// Attempt to deserialize a calibration block from `buf` into `out`.
///
/// Validates the magic number, version byte, and CRC-32 checksum.
/// Returns `true` and populates `out` on success. Returns `false` without
/// modifying `out` on any validation failure, including a VERSION mismatch
/// which rejects data written by an incompatible layout.
pub fn try_deserialize<const ROW: usize, const COL: usize>(
    buf: &[u8],
    out: &mut [[CalibEntry; COL]; ROW],
    crc: &mut Crc<'_>,
) -> bool {
    if buf.len() < CALIB_BUF_LEN {
        return false;
    }

    // Validate magic number.
    let magic_end = size_of::<u32>();
    let Some(magic_bytes) = read_array::<4>(buf, 0, magic_end) else { return false };
    if u32::from_le_bytes(magic_bytes) != MAGIC {
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
    let stored_crc = u32::from_le_bytes(stored_crc_bytes);
    let computed_crc = buf.get(..data_end).map_or(0, |data| crc32_of(crc, data));
    if computed_crc != stored_crc {
        return false;
    }

    // Deserialize entries.
    let mut pos = HEADER_LEN;
    for row in out.iter_mut() {
        for entry in row.iter_mut() {
            let end = pos.saturating_add(ENTRY_LEN);
            let Some(fb) = read_array::<2>(buf, pos, end) else { return false };
            entry.full = u16::from_le_bytes(fb);
            pos = end;
        }
    }
    true
}

/// Compute the total serialized byte length for a `rows × cols` matrix.
pub const fn total_len(rows: usize, cols: usize) -> usize {
    HEADER_LEN.saturating_add(rows.saturating_mul(cols).saturating_mul(ENTRY_LEN)).saturating_add(CRC_LEN)
}
