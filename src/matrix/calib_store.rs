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
//! ```
//!
//! where N = ROW × COL.
//!
//! Validation on load checks only the magic number and version byte. There
//! is no trailing checksum — the calibration buffer (≤ 511 bytes) already
//! fits entirely in RAM, and the magic + version pair is sufficient to
//! distinguish a valid write from a blank (0xFF) or garbage EEPROM.

/// Magic number identifying a valid calibration block.
const MAGIC: u32 = 0xCAFE_BABE;
/// Format version. Increment on any incompatible layout change.
const VERSION: u8 = 1;
/// Byte length of the header (MAGIC + VERSION).
const HEADER_LEN: usize = 5;

/// EEPROM word address at which the calibration block begins.
pub const EEPROM_BASE_ADDR: u16 = 0x0000;

/// Compute the total serialized byte length for a `rows × cols` matrix.
pub const fn total_len(rows: usize, cols: usize) -> usize { HEADER_LEN + rows * cols * 4 }

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
/// `buf` must be at least [`total_len`]`(ROW, COL)` bytes.
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
}

/// Attempt to deserialize a calibration block from `buf` into `out`.
///
/// Validates only the magic number and version byte. Returns `true` and
/// populates `out` on success. Returns `false` without modifying `out` if
/// either check fails — this reliably rejects blank (0xFF) EEPROMs since
/// `0xFFFF_FFFF != MAGIC`, and rejects future incompatible formats via the
/// version byte.
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
