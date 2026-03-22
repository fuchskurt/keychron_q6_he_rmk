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
//! 0        4             Magic bytes b"QKHE"
//! 4        1             Format version (currently 1)
//! 5        1             XOR checksum of bytes [6..]
//! 6        ROW*COL*2     Zero-travel ADC values, row-major, u16 little-endian
//! ```
//!
//! Total for 6×21 matrix: 6 + 252 = 258 bytes (well within 1 KB capacity).

use core::array::from_fn;

use embassy_stm32::{
    gpio::Output,
    i2c::{Error as I2cError, I2c},
    mode::Blocking,
    peripherals::I2C3,
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
const MAGIC: [u8; 4] = *b"QKHE";

/// Bump this whenever the data layout changes to force re-calibration.
const VERSION: u8 = 1;

/// Errors produced by EEPROM operations.
#[derive(Debug)]
pub enum EepromError {
    /// Underlying I2C bus error.
    I2c(I2cError),
    /// Buffer would exceed the maximum supported matrix size.
    BufferTooSmall,
}

impl From<I2cError> for EepromError {
    fn from(e: I2cError) -> Self { Self::I2c(e) }
}

/// Maximum data bytes this driver can handle (6 rows × 21 cols × 2 bytes).
const MAX_DATA: usize = 6 * 21 * 2;

/// Blocking I2C driver for the external 24-series EEPROM.
///
/// Only used during initialisation (load) and after first boot calibration
/// (save), so blocking I2C is acceptable.
pub struct EepromStorage<'d> {
    i2c: I2c<'d, I2C3, Blocking>,
    /// Write-protect GPIO. HIGH = protected, LOW = writable.
    wp: Output<'d>,
}

impl<'d> EepromStorage<'d> {
    /// Create a new storage driver.
    ///
    /// The WP pin should be initialised HIGH (write-protected) by the caller.
    pub fn new(i2c: I2c<'d, I2C3, Blocking>, wp: Output<'d>) -> Self { Self { i2c, wp } }

    /// Sequential read starting at `word_addr`.
    fn read_bytes(&mut self, word_addr: u16, buf: &mut [u8]) -> Result<(), EepromError> {
        let addr = [(word_addr >> 8) as u8, (word_addr & 0xFF) as u8];
        self.i2c.blocking_write_read(EEPROM_ADDR, &addr, buf)?;
        Ok(())
    }

    /// Write up to `PAGE_SIZE` bytes starting at `word_addr`.
    ///
    /// `word_addr + data.len()` must not cross a page boundary.
    fn write_page(&mut self, word_addr: u16, data: &[u8]) -> Result<(), EepromError> {
        debug_assert!(data.len() <= PAGE_SIZE);
        let mut buf = [0u8; 2 + PAGE_SIZE];
        buf[0] = (word_addr >> 8) as u8;
        buf[1] = (word_addr & 0xFF) as u8;
        let len = data.len().min(PAGE_SIZE);
        buf[2..2 + len].copy_from_slice(&data[..len]);

        self.wp.set_low();
        let result = self.i2c.blocking_write(EEPROM_ADDR, &buf[..2 + len]).map_err(EepromError::from);
        // AT24Cxx write cycle: 5 ms max. block_for does not yield so the
        // executor stays blocked, but this only happens at boot time.
        block_for(Duration::from_millis(6));
        self.wp.set_high();
        result
    }

    /// Write `data` to EEPROM at `start_addr`, splitting on page boundaries.
    fn write_bytes(&mut self, start_addr: u16, data: &[u8]) -> Result<(), EepromError> {
        let mut offset = 0usize;
        while offset < data.len() {
            let addr = start_addr + offset as u16;
            let page_space = PAGE_SIZE - (addr as usize % PAGE_SIZE);
            let chunk = page_space.min(data.len() - offset);
            self.write_page(addr, &data[offset..offset + chunk])?;
            offset += chunk;
        }
        Ok(())
    }

    /// Try to load per-key zero-travel ADC values from EEPROM.
    ///
    /// Returns `None` if the header magic, version, or checksum is wrong,
    /// indicating the EEPROM has never been written or the data is stale.
    pub fn load_calibration<const ROW: usize, const COL: usize>(
        &mut self,
    ) -> Option<[[u16; COL]; ROW]> {
        let data_len = ROW * COL * 2;
        if data_len > MAX_DATA {
            return None;
        }

        let mut header = [0u8; 6];
        self.read_bytes(0, &mut header).ok()?;

        if header[..4] != MAGIC || header[4] != VERSION {
            return None;
        }
        let stored_checksum = header[5];

        let mut buf = [0u8; MAX_DATA];
        self.read_bytes(DATA_OFFSET, &mut buf[..data_len]).ok()?;

        let checksum = buf[..data_len].iter().fold(0u8, |acc, &b| acc ^ b);
        if checksum != stored_checksum {
            return None;
        }

        Some(from_fn(|row| from_fn(|col| {
            let i = (row * COL + col) * 2;
            u16::from_le_bytes([buf[i], buf[i + 1]])
        })))
    }

    /// Persist per-key zero-travel ADC values to EEPROM with header and checksum.
    ///
    /// Writes a 6-byte header followed by `ROW × COL × 2` bytes of data.
    /// Takes roughly `ceil(total / 32) × 6 ms` due to page-write cycles.
    pub fn save_calibration<const ROW: usize, const COL: usize>(
        &mut self,
        zeros: &[[u16; COL]; ROW],
    ) -> Result<(), EepromError> {
        let data_len = ROW * COL * 2;
        if data_len > MAX_DATA {
            return Err(EepromError::BufferTooSmall);
        }

        let mut data_buf = [0u8; MAX_DATA];
        for row in 0..ROW {
            for col in 0..COL {
                let i = (row * COL + col) * 2;
                let bytes = zeros[row][col].to_le_bytes();
                data_buf[i] = bytes[0];
                data_buf[i + 1] = bytes[1];
            }
        }
        let data = &data_buf[..data_len];
        let checksum = data.iter().fold(0u8, |acc, &b| acc ^ b);

        let mut header = [0u8; 6];
        header[..4].copy_from_slice(&MAGIC);
        header[4] = VERSION;
        header[5] = checksum;

        // Write header + data as a contiguous stream starting at byte 0.
        let total = 6 + data_len;
        let mut full = [0u8; 6 + MAX_DATA];
        full[..6].copy_from_slice(&header);
        full[6..6 + data_len].copy_from_slice(data);
        self.write_bytes(0, &full[..total])
    }
}
