//! External I2C EEPROM storage for hall-effect calibration persistence.
//!
//! The Q6 HE has a 1 KB 24-series EEPROM connected to I2C3:
//!
//! | Pin  | GPIO | Function      |
//! |------|------|---------------|
//! | SCL  | PA8  | I2C3_SCL AF4  |
//! | SDA  | PC9  | I2C3_SDA AF4  |
//! | WP   | PB10 | Write-protect |
//!
//! Drive WP **low** to enable writes; the driver handles this automatically.
//!
//! ## I2C address
//!
//! QMK defines the base address as `0xA2` (8-bit). Embassy uses 7-bit
//! addresses, so the correct value is `0xA2 >> 1 = 0x51`.
//! If your board has A0/A1/A2 all tied low the address is `0x50` instead.
//! Verify with a logic analyser or by toggling [`EEPROM_ADDR`] below.
//!
//! ## Data layout (from byte 0)
//!
//! ```text
//! Offset   Size          Content
//! 0        4             Magic bytes b"Q6HE"
//! 4        1             Format version (currently 1)
//! 5        1             XOR checksum of bytes [6..]
//! 6        ROW*COL*2     Zero-travel ADC values, row-major, u16 little-endian
//! ```
//!
//! Total for 6×21 matrix: 6 + 252 = 258 bytes (well within 1 KB capacity).

use core::array::from_fn;
use embassy_stm32::{
    gpio::Output,
    i2c::{Error as I2cError, I2c, mode::Master},
    mode::Blocking,
};
use embassy_time::{Duration, block_for};

/// 7-bit I2C address.
///
/// QMK base address 0xA2 → 7-bit 0x51. Change to 0x50 if A0=A1=A2=GND.
const EEPROM_ADDR: u8 = 0x51;

/// Maximum write chunk: AT24C08 page size.
const PAGE_SIZE: usize = 32;

/// Byte offset where per-key data begins (after the 6-byte header).
const DATA_OFFSET: u16 = 6;

/// Magic bytes confirming our data is present.
const MAGIC: [u8; 4] = *b"Q6HE";

/// Maximum data bytes this driver can handle (6 rows × 21 cols × 2 bytes).
const MAX_DATA: usize = 6_usize.saturating_mul(21).saturating_mul(2);

/// Bump this whenever the data layout changes to force re-calibration.
const VERSION: u8 = 1;

/// Errors produced by EEPROM operations.
#[derive(Debug)]
pub enum EepromError {
    /// Buffer would exceed the maximum supported matrix size.
    BufferTooSmall,
    /// Underlying I2C bus error.
    I2c,
}

impl From<I2cError> for EepromError {
    fn from(_: I2cError) -> Self { Self::I2c }
}

/// Blocking I2C driver for the external 24-series EEPROM.
///
/// Only used during initialisation (load) and after first boot calibration
/// (save), so blocking I2C is acceptable.
///
/// # Construction
///
/// ```ignore
/// let mut i2c_config = i2c::Config::default();
/// i2c_config.frequency = Hertz(400_000);
/// let i2c = I2c::new_blocking(p.I2C3, p.PA8, p.PC9, i2c_config);
/// let wp = Output::new(p.PB10, Level::High, Speed::Low);
/// let eeprom = EepromStorage::new(i2c, wp);
/// ```
// `single_char_lifetime_names`: use a descriptive lifetime name instead of
// `'d`.
pub struct EepromStorage<'peripheral> {
    /// Blocking I2C master handle (`I2c::new_blocking` produces this type).
    i2c: I2c<'peripheral, Blocking, Master>,
    /// Write-protect GPIO. HIGH = protected, LOW = writable.
    wp: Output<'peripheral>,
}

impl<'peripheral> EepromStorage<'peripheral> {
    /// Try to load per-key zero-travel ADC values from EEPROM.
    ///
    /// Returns `None` if the header magic, version, or checksum is wrong,
    /// indicating the EEPROM has never been written or the data is stale.
    pub fn load_calibration<const ROW: usize, const COL: usize>(&mut self) -> Option<[[u16; COL]; ROW]> {
        let data_len = ROW.saturating_mul(COL).saturating_mul(2);
        if data_len > MAX_DATA {
            return None;
        }

        let mut header = [0_u8; 6];
        match self.read_bytes(0, &mut header) {
            Ok(()) => {}
            Err(_) => return None,
        }

        if header.get(..4) != Some(&MAGIC) || header.get(4) != Some(&VERSION) {
            return None;
        }
        let stored_checksum = match header.get(5) {
            Some(bytes) => *bytes,
            None => return None,
        };

        let mut buf = [0_u8; MAX_DATA];
        match self.read_bytes(DATA_OFFSET, buf.get_mut(..data_len).unwrap_or(&mut [])) {
            Ok(()) => {}
            Err(_) => return None,
        }

        let checksum = buf.get(..data_len).unwrap_or(&[]).iter().fold(0_u8, |acc, &bytes| acc ^ bytes);
        if checksum != stored_checksum {
            return None;
        }

        Some(from_fn(|row| {
            from_fn(|col| {
                let i = row.saturating_mul(COL).saturating_add(col).saturating_mul(2);
                let lo = buf.get(i).copied().unwrap_or(0);
                let hi = buf.get(i.saturating_add(1)).copied().unwrap_or(0);
                // `little_endian_bytes`: avoid `from_le_bytes` — reconstruct manually.
                // little-endian: lo is bits [7:0], hi is bits [15:8].
                u16::from(lo) | (u16::from(hi) << 8_u16)
            })
        }))
    }

    /// Create a new storage driver.
    ///
    /// The WP pin should be initialised HIGH (write-protected) by the caller.
    pub const fn new(i2c: I2c<'peripheral, Blocking, Master>, wp: Output<'peripheral>) -> Self { Self { i2c, wp } }

    /// Sequential read starting at `word_addr`.
    fn read_bytes(&mut self, word_addr: u16, buf: &mut [u8]) -> Result<(), EepromError> {
        let hi = u8::try_from(word_addr >> 8_u16).unwrap_or(0);
        let lo = u8::try_from(word_addr & 0xFF).unwrap_or(0);
        let addr = [hi, lo];
        match self.i2c.blocking_write_read(EEPROM_ADDR, &addr, buf) {
            Ok(value) => Ok(value),
            Err(error) => Err(EepromError::from(error)),
        }
    }

    /// Persist per-key zero-travel ADC values to EEPROM with header and
    /// checksum.
    ///
    /// Writes a 6-byte header followed by `ROW × COL × 2` bytes of data.
    /// Takes roughly `ceil(total / 32) × 6 ms` due to page-write cycles.
    pub fn save_calibration<const ROW: usize, const COL: usize>(
        &mut self,
        zeros: &[[u16; COL]; ROW],
    ) -> Result<(), EepromError> {
        let data_len = ROW.saturating_mul(COL).saturating_mul(2);
        if data_len > MAX_DATA {
            return Err(EepromError::BufferTooSmall);
        }

        let mut data_buf = [0_u8; MAX_DATA];
        for row in 0..ROW {
            for col in 0..COL {
                let i = row.saturating_mul(COL).saturating_add(col).saturating_mul(2);
                let val = zeros.get(row).and_then(|row_slice| row_slice.get(col)).copied().unwrap_or(0);
                // `little_endian_bytes`: avoid `.to_le_bytes()` — write bytes manually.
                // little-endian: low byte at i, high byte at i+1.
                if let Some(bytes) = data_buf.get_mut(i) {
                    // Mask to low byte: val & 0xFF is always in u8 range.
                    *bytes = u8::try_from(val & 0xFF).unwrap_or(0);
                }
                if let Some(bytes) = data_buf.get_mut(i.saturating_add(1)) {
                    // High byte: val >> 8 is always in u8 range for an u16.
                    *bytes = u8::try_from(val >> 8_u16).unwrap_or(0);
                }
            }
        }

        let data = data_buf.get(..data_len).unwrap_or(&[]);
        let checksum = data.iter().fold(0_u8, |acc, &bytes| acc ^ bytes);

        let mut header = [0_u8; 6];
        if let Some(dst) = header.get_mut(..4) {
            dst.copy_from_slice(&MAGIC);
        }
        if let Some(bytes) = header.get_mut(4) {
            *bytes = VERSION;
        }
        if let Some(bytes) = header.get_mut(5) {
            *bytes = checksum;
        }

        let total = 6_usize.saturating_add(data_len);
        let mut full = [0_u8; 6_usize.saturating_add(MAX_DATA)];
        if let Some(dst) = full.get_mut(..6) {
            dst.copy_from_slice(&header);
        }
        if let (Some(dst), Some(src)) = (full.get_mut(6..total), data_buf.get(..data_len)) {
            dst.copy_from_slice(src);
        }
        match self.write_bytes(0, full.get(..total).unwrap_or(&[])) {
            Ok(()) => Ok(()),
            Err(error) => Err(error),
        }
    }

    /// Write `data` to EEPROM at `start_addr`, splitting on page boundaries.
    fn write_bytes(&mut self, start_addr: u16, data: &[u8]) -> Result<(), EepromError> {
        let mut offset = 0_usize;
        while offset < data.len() {
            let addr = start_addr.saturating_add(u16::try_from(offset).unwrap_or(u16::MAX));
            let page_space = PAGE_SIZE.saturating_sub(usize::from(addr).rem_euclid(PAGE_SIZE));
            let chunk = page_space.min(data.len().saturating_sub(offset));
            let end = offset.saturating_add(chunk);
            match self.write_page(addr, data.get(offset..end).unwrap_or(&[])) {
                Ok(()) => {}
                Err(error) => return Err(error),
            }
            offset = offset.saturating_add(chunk);
        }
        Ok(())
    }

    /// Write up to `PAGE_SIZE` bytes starting at `word_addr`.
    ///
    /// `word_addr + data.len()` must not cross a page boundary.
    fn write_page(&mut self, word_addr: u16, data: &[u8]) -> Result<(), EepromError> {
        let mut buf = [0_u8; 2_usize.saturating_add(PAGE_SIZE)];

        // `as_conversions`: mask into u8 range first, then safe-cast.
        let hi = u8::try_from(word_addr >> 8_u16).unwrap_or(0);
        let lo = u8::try_from(word_addr & 0xFF).unwrap_or(0);
        if let Some(bytes) = buf.get_mut(0) {
            *bytes = hi;
        }
        if let Some(bytes) = buf.get_mut(1) {
            *bytes = lo;
        }

        let len = data.len().min(PAGE_SIZE);
        let end = len.saturating_add(2);
        if let (Some(dst), Some(src)) = (buf.get_mut(2..end), data.get(..len)) {
            dst.copy_from_slice(src);
        }

        self.wp.set_low();
        let result = match self.i2c.blocking_write(EEPROM_ADDR, buf.get(..end).unwrap_or(&[])) {
            Ok(()) => Ok(()),
            Err(error) => Err(EepromError::from(error)),
        };
        block_for(Duration::from_millis(6));
        self.wp.set_high();
        result
    }
}
