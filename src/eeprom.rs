//! FT24C64 64-Kbit I²C EEPROM driver.
//!
//! - SCL: PA8  (I²C3)
//! - SDA: PC9  (I²C3)
//! - WP:  PB10 (active-high write-protect)
//!
//! 7-bit device address with A0 = A1 = A2 tied to GND: 0x50.

use embassy_stm32::{
    gpio::{Flex, Pull},
    i2c::{mode::MasterMode, Error, I2c},
    mode::Async,
};
use embassy_time::Timer;

/// 7-bit I²C device address (A0 = A1 = A2 = GND).
const DEVICE_ADDR: u8 = 0x50;
/// Page write size in bytes per the FT24C64 datasheet §6.4.
const PAGE_SIZE: usize = 32;
/// Minimum write-cycle time tWR in milliseconds (datasheet §AC
/// Characteristics).
const WRITE_CYCLE_MS: u64 = 10;
/// I²C transmit scratch buffer: 2-byte word address + one full page of data.
const WRITE_BUF_LEN: usize = 2 + PAGE_SIZE;

/// Driver for the FT24C64 64-Kbit (8 K × 8) I²C EEPROM.
///
/// All reads use a single write-then-read (random-address sequential read).
/// Writes are automatically split at 32-byte page boundaries; the mandatory
/// tWR delay is observed after every page write. The write-protect pin is
/// held high (read-only) at all times except during the I²C write sequence,
/// and is restored to high before returning — including on error.
pub struct Ft24c64<'peripherals, IM: MasterMode> {
    i2c: I2c<'peripherals, Async, IM>,
    /// Active-high write-protect. High = read-only, Low = writable.
    wp: Flex<'peripherals>,
}

impl<'peripherals, IM: MasterMode> Ft24c64<'peripherals, IM> {
    /// Create a new driver. `wp` must already be driven high (write-protected).
    pub fn new(i2c: I2c<'peripherals, Async, IM>, mut wp: Flex<'peripherals>) -> Self {
        // Input pull-up: WP is asserted via the pull resistor.
        // Matches QMK: writePin(WP, 1); setPinInputHigh(WP).
        wp.set_as_input(Pull::Up);
        Self { i2c, wp }
    }

    /// Read `buf.len()` bytes from 16-bit word address `addr` (sequential
    /// read).
    pub async fn read(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c.write_read(DEVICE_ADDR, &addr.to_be_bytes(), buf).await
    }

    /// Write `data` starting at word address `addr`.
    ///
    /// Splits automatically at 32-byte page boundaries and waits tWR after
    /// each page. WP is lowered for the entire write and restored on return.
    pub async fn write(&mut self, start_addr: u16, data: &[u8]) -> Result<(), Error> {
        // Switch to output-low to deassert WP during the write cycle.
        // Matches QMK: setPinOutput(WP); writePin(WP, 0).
        self.wp.set_low();
        self.wp.set_as_output(embassy_stm32::gpio::Speed::Low);

        let mut offset = 0_usize;
        let mut result = Ok(());

        while offset < data.len() {
            let addr = start_addr.saturating_add(u16::try_from(offset).unwrap_or(u16::MAX));
            let page_remaining = PAGE_SIZE - usize::from(addr) % PAGE_SIZE;
            let chunk_len = page_remaining.min(data.len() - offset);

            let mut buf = [0_u8; WRITE_BUF_LEN];
            let [hi, lo] = addr.to_be_bytes();
            buf[0] = hi;
            buf[1] = lo;
            buf[2..2 + chunk_len].copy_from_slice(&data[offset..offset + chunk_len]);

            result = self.i2c.write(DEVICE_ADDR, &buf[..2 + chunk_len]).await;
            Timer::after_millis(WRITE_CYCLE_MS).await;

            if result.is_err() {
                break;
            }
            offset += chunk_len;
        }

        // Restore to input pull-up.
        self.wp.set_as_input(Pull::Up);
        result
    }
}
