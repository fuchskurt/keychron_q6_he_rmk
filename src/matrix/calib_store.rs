//! Calibration data serialization for EEPROM storage.
//!
//! Binary layout (all multi-byte integers little-endian):
//!
//! ```text
//! Offset        Size    Field
//! 0             4       MAGIC (0xCAFE_BABE_u32)
//! 4             1       VERSION (1)
//! 5             N × 4   entries, row-major:
//!                         zero: u16  — ADC at zero travel (key at rest)
//!                         full: u16  — ADC at full travel (key bottomed out)
//! 5 + N × 4     2       CRC-16/CCITT over all preceding bytes
//! ```
//!
//! where N = ROW × COL.

/// Magic number identifying a valid calibration block.
const MAGIC: u32 = 0xCAFE_BABE;
/// Format version. Increment on any incompatible layout change.
const VERSION: u8 = 1;
/// Byte length of the header (MAGIC + VERSION).
const HEADER_LEN: usize = 5;
/// Byte length of the trailing CRC field.
const CRC_LEN: usize = 2;

/// EEPROM word address at which the calibration block begins.
pub const EEPROM_BASE_ADDR: u16 = 0x0000;

/// Compute the total serialized byte length for a `rows × cols` matrix.
pub const fn total_len(rows: usize, cols: usize) -> usize { HEADER_LEN + rows * cols * 4 + CRC_LEN }

/// Pre-computed buffer length for the Q6 HE ANSI matrix (6 × 21 = 126 keys).
///
/// Used as a fixed stack allocation size to avoid requiring
/// `#![feature(generic_const_exprs)]`.
pub const CALIB_BUF_LEN: usize = total_len(6, 21);

/// Per-key calibration values exchanged with the EEPROM.
#[derive(Clone, Copy)]
pub struct CalibEntry {
    /// Raw ADC reading at zero travel (key fully released, at rest).
    pub zero: u16,
    /// Raw ADC reading at full travel (key fully depressed to the PCB).
    pub full: u16,
}

/// Serialize `entries` (row-major, `ROW × COL`) into `buf`.
///
/// `buf` must be at least [`total_len`]`(ROW, COL)` bytes. Asserts in debug.
pub fn serialize<const ROW: usize, const COL: usize>(entries: &[[CalibEntry; COL]; ROW], buf: &mut [u8]) {
    debug_assert!(buf.len() >= total_len(ROW, COL));
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4] = VERSION;

    let mut i = HEADER_LEN;
    for row in entries {
        for entry in row {
            buf[i..i + 2].copy_from_slice(&entry.zero.to_le_bytes());
            buf[i + 2..i + 4].copy_from_slice(&entry.full.to_le_bytes());
            i += 4;
        }
    }

    let crc = crc16(&buf[..i]);
    buf[i..i + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
}

/// Attempt to deserialize a calibration block from `buf` into `out`.
///
/// Validates the magic number, version byte, and CRC-16/CCITT checksum.
/// Returns `true` and populates `out` on success. Returns `false` without
/// modifying `out` if any check fails (corrupted, blank, or incompatible data).
pub fn try_deserialize<const ROW: usize, const COL: usize>(buf: &[u8], out: &mut [[CalibEntry; COL]; ROW]) -> bool {
    if buf.len() < total_len(ROW, COL) {
        return false;
    }

    let Ok(magic_bytes) = buf[0..4].try_into() else { return false };
    if u32::from_le_bytes(magic_bytes) != MAGIC {
        return false;
    }
    if buf[4] != VERSION {
        return false;
    }

    let data_end = HEADER_LEN + ROW * COL * 4;
    let Ok(crc_bytes) = buf[data_end..data_end + CRC_LEN].try_into() else { return false };
    if crc16(&buf[..data_end]) != u16::from_le_bytes(crc_bytes) {
        return false;
    }

    let mut i = HEADER_LEN;
    for row in out.iter_mut() {
        for entry in row.iter_mut() {
            let Ok(zb) = buf[i..i + 2].try_into() else { return false };
            let Ok(fb) = buf[i + 2..i + 4].try_into() else { return false };
            entry.zero = u16::from_le_bytes(zb);
            entry.full = u16::from_le_bytes(fb);
            i += 4;
        }
    }
    true
}

/// CRC-16/CCITT: polynomial 0x1021, initial value 0xFFFF.
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}
