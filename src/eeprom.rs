use embassy_stm32::{
    gpio::{Flex, Pull, Speed},
    i2c::{Error, Error::Overrun, I2c, mode::MasterMode},
    mode::Async,
};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::Operation;

/// 7-bit I²C device address (A0 = A1 = A2 = GND).
const DEVICE_ADDR: u8 = 0x51;
/// Total size of the FT24C64 in bytes (64 Kbit).
const EEPROM_SIZE: usize = 8192;
/// Page write size in bytes per the FT24C64 datasheet.
const PAGE_SIZE: usize = 32;

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

    /// Poll the device with a zero-length write until it ACKs, indicating
    /// the internal write cycle has completed. Retries up to `max_attempts`
    /// times with a short yield between each attempt.
    ///
    /// Returns `Ok(())` as soon as the device acknowledges. Returns
    /// [`Error`] if the device does not become ready within `max_attempts`.
    async fn poll_until_ready(&mut self, max_attempts: u8) -> Result<(), Error> {
        let mut attempts = 0_u8;
        loop {
            let result = self.i2c.write(DEVICE_ADDR, &[]).await;
            if result.is_ok() {
                return Ok(());
            }
            attempts = attempts.saturating_add(1);
            if attempts >= max_attempts {
                return result;
            }
            Timer::after(Duration::from_micros(20)).await;
        }
    }

    /// Read `buf.len()` bytes starting at 16-bit word address `addr`.
    ///
    /// Polls the device until it acknowledges before issuing the read,
    /// accommodating the power-on reset delay without requiring a fixed
    /// worst-case wait from the caller.
    pub async fn read(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Error> {
        self.poll_until_ready(20).await?;
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
            let addr_bytes = addr.to_be_bytes();
            let chunk = &data[offset..offset.saturating_add(chunk_len)];
            result =
                self.i2c.transaction(DEVICE_ADDR, &mut [Operation::Write(&addr_bytes), Operation::Write(chunk)]).await;
            if result.is_err() {
                break;
            }

            result = self.poll_until_ready(255).await;
            if result.is_err() {
                break;
            }

            offset = offset.saturating_add(chunk_len);
        }

        // Restore to input pull-up: write-protect re-asserted.
        self.wp.set_as_input(Pull::Up);
        result
    }

    /// Erase the entire EEPROM by writing `0xFF` to all 8 192 bytes.
    ///
    /// Pages are written sequentially from address `0x0000` to `0x1FFF` (256
    /// pages × 32 bytes). After each page write, the driver waits for the
    /// device to become ready again using the polling-based readiness check
    /// before continuing with the next page.
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
            let addr = match u16::try_from(offset) {
                Ok(a) => a,
                Err(_) => return Err(Overrun),
            };
            let result = self.write(addr, &blank).await;
            if let Err(e) = result {
                return Err(e);
            }
            offset = offset.saturating_add(PAGE_SIZE);
        }
        Ok(())
    }
}
