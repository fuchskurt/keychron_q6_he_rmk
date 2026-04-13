use embassy_stm32::{
    gpio::{Flex, Pull, Speed},
    i2c::{Error, I2c, mode::MasterMode},
    mode::Async,
};
use embassy_time::{Duration, Timer};

/// 7-bit I²C device address (A0 = A1 = A2 = GND).
const DEVICE_ADDR: u8 = 0x51;
/// Total size of the FT24C64 in bytes (64 Kbit).
const EEPROM_SIZE: usize = 8192;
/// Page write size in bytes per the FT24C64 datasheet.
const PAGE_SIZE: usize = 32;
/// Write-cycle time tWR in milliseconds.
const WRITE_CYCLE_DELAY: Duration = Duration::from_millis(5);
/// I²C transmit buffer: 2-byte word address + one full page of data.
const WRITE_BUF_LEN: usize = 2_usize.saturating_add(PAGE_SIZE);

/// Driver for the FT24C64 64-Kbit (8 K × 8) I²C EEPROM.
pub struct Ft24c64<'peripherals, IM: MasterMode> {
    /// I²C peripheral used for all device communication.
    i2c: I2c<'peripherals, Async, IM>,
    /// Write-protect pin managed as input pull-up when idle, output-low when
    /// writing.
    wp: Flex<'peripherals>,
}

impl<'peripherals, IM: MasterMode> Ft24c64<'peripherals, IM> {
    /// Create a new driver.
    pub fn new(i2c: I2c<'peripherals, Async, IM>, mut wp: Flex<'peripherals>) -> Self {
        // Input pull-up: write-protect asserted via pull resistor.
        wp.set_as_input(Pull::Up);
        Self { i2c, wp }
    }

    /// Read `buf.len()` bytes starting at 16-bit word address `addr`.
    pub async fn read(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c.write_read(DEVICE_ADDR, &addr.to_be_bytes(), buf).await
    }

    /// Write `data` starting at 16-bit word address `start_addr`.
    pub async fn write(&mut self, start_addr: u16, data: &[u8]) -> Result<(), Error> {
        // Switch to output-low: write-protect deasserted.
        self.wp.set_low();
        self.wp.set_as_output(Speed::Low);

        let mut offset = 0_usize;
        let mut result = Ok(());

        while offset < data.len() {
            // Convert byte offset to an u16 address, saturating on overflow so
            // we never silently wrap into a wrong EEPROM location.
            let addr = start_addr.saturating_add(u16::try_from(offset).unwrap_or(u16::MAX));

            // Bytes remaining before the next 32-byte page boundary.
            let page_offset = usize::from(addr).rem_euclid(PAGE_SIZE);
            let page_remaining = PAGE_SIZE.saturating_sub(page_offset);

            // Bytes to write on this page, the smaller of the remaining page
            // space and the remaining data.
            let chunk_len = page_remaining.min(data.len().saturating_sub(offset));

            // Build the packet: [addr_hi, addr_lo, data...].
            // Invariants: chunk_len ≤ PAGE_SIZE, so 2 + chunk_len ≤ WRITE_BUF_LEN;
            // and offset + chunk_len ≤ data.len() by construction above.
            let data_end = 2_usize.saturating_add(chunk_len);
            let mut buf = [0_u8; WRITE_BUF_LEN];
            buf[..2].copy_from_slice(&addr.to_be_bytes());
            buf[2..data_end].copy_from_slice(&data[offset..offset.saturating_add(chunk_len)]);

            result = self.i2c.write(DEVICE_ADDR, &buf[..data_end]).await;
            Timer::after(WRITE_CYCLE_DELAY).await;

            if result.is_err() {
                break;
            }

            offset = offset.saturating_add(chunk_len);
        }

        // Restore to input pull-up: write-protect re-asserted.
        self.wp.set_as_input(Pull::Up);
        result
    }

    /// Erase the entire EEPROM by writing `0x00` to all 8 192 bytes.
    ///
    /// Pages are written sequentially from address `0x0000` to `0x1FFF` (256
    /// pages × 32 bytes). The [`WRITE_CYCLE_DELAY`] is observed after every
    /// page write, so this operation takes approximately 1.28 seconds to
    /// complete.
    ///
    /// Call this once on first boot before starting a calibration run to
    /// guarantee no stale data survives in any region of the device.
    ///
    /// # Errors
    ///
    /// Returns the first [`Error`] encountered. Any pages written before the
    /// failure are not rolled back.
    pub async fn zero_out(&mut self) -> Result<(), Error> {
        let blank = [0xFF_u8; PAGE_SIZE];
        let mut offset = 0_usize;
        while offset < EEPROM_SIZE {
            let addr = u16::try_from(offset).unwrap_or(u16::MAX);
            let result = self.write(addr, &blank).await;
            if let Err(e) = result {
                return Err(e);
            }
            offset = offset.saturating_add(PAGE_SIZE);
        }
        Ok(())
    }
}
