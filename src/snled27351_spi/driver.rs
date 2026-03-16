//! SNLED27351 SPI driver implementation.

use crate::snled27351_spi::registers::{
    CURRENT_TUNE_REGISTER_COUNT,
    LED_CONTROL_REGISTER_COUNT,
    PAGE_CURRENT_TUNE,
    PAGE_FUNCTION,
    PAGE_LED_CONTROL,
    PAGE_PWM,
    PATTERN_CMD,
    PULLDOWNUP_ALL_ENABLED,
    PWM_REGISTER_COUNT,
    REG_PULLDOWNUP,
    REG_SCAN_PHASE,
    REG_SLEW_RATE_CONTROL_MODE_1,
    REG_SLEW_RATE_CONTROL_MODE_2,
    REG_SOFTWARE_SHUTDOWN,
    REG_SOFTWARE_SLEEP,
    SCAN_PHASE_12_CHANNEL,
    SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE,
    SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE,
    SOFTWARE_SHUTDOWN_SSD_NORMAL,
    SOFTWARE_SHUTDOWN_SSD_SHUTDOWN,
    SOFTWARE_SLEEP_DISABLE,
    WRITE_CMD,
};
use core::slice::from_ref;
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use embassy_time::Timer;

/// Number of SNLED27351 drivers chained on the SPI bus.
const DRIVER_COUNT: usize = 2;

/// Maximum SPI payload: 2-byte header + PWM register block.
const SPI_BUF_LEN: usize = PWM_REGISTER_COUNT.saturating_add(2);

#[derive(Copy, Clone)]
/// Mapping of an RGB LED to SNLED driver channels.
pub struct SnledLed {
    /// Blue color channel index on the SNLED driver.
    pub blue: u8,
    /// Index of the SNLED driver controlling this LED.
    pub driver: u8,
    /// Green color channel index on the SNLED driver.
    pub green: u8,
    /// Red color channel index on the SNLED driver.
    pub red: u8,
}

/// PWM shadow buffers and dirty flags, split from hardware handles so
/// they can be borrowed independently of `spi` and `cs`.
struct SnledBuffers {
    /// PWM shadow buffer per driver; index is the raw register address.
    pwm_buf: [[u8; PWM_REGISTER_COUNT]; DRIVER_COUNT],
    /// Whether each driver's buffer has unsent changes.
    pwm_dirty: [bool; DRIVER_COUNT],
}

impl SnledBuffers {
    /// Create zeroed buffers with no pending changes.
    const fn new() -> Self {
        Self { pwm_buf: [[0_u8; PWM_REGISTER_COUNT]; DRIVER_COUNT], pwm_dirty: [false; DRIVER_COUNT] }
    }

    /// Write scaled RGB values into the PWM shadow buffer for one LED.
    const fn stage_led(&mut self, led: SnledLed, red: u8, green: u8, blue: u8) {
        let drv = usize::from(led.driver);
        let Some(buf) = self.pwm_buf.get_mut(drv) else { return };

        if let Some(slot) = buf.get_mut(usize::from(led.red)) {
            *slot = red;
        }
        if let Some(slot) = buf.get_mut(usize::from(led.green)) {
            *slot = green;
        }
        if let Some(slot) = buf.get_mut(usize::from(led.blue)) {
            *slot = blue;
        }

        if let Some(dirty) = self.pwm_dirty.get_mut(drv) {
            *dirty = true;
        }
    }
}

/// SNLED27351 SPI bus handle — owns the hardware pins but not the buffers.
struct SnledBus<'peripherals> {
    /// Chip-select outputs for each SNLED driver instance.
    cs: [Output<'peripherals>; DRIVER_COUNT],
    /// Shutdown (SDB) control pin for enabling or disabling the driver.
    sdb: Output<'peripherals>,
    /// SPI peripheral used to communicate with the SNLED driver(s).
    spi: Spi<'peripherals, Async, spi::mode::Master>,
}

impl SnledBus<'_> {
    /// Flush one driver's dirty PWM buffer to hardware without copying it.
    ///
    /// Takes the buffer slice directly — no stack copy needed because
    /// `buf` and `spi`/`cs` are in separate structs.
    async fn flush_driver(&mut self, index: usize, pwm: &[u8; PWM_REGISTER_COUNT]) {
        let mut tx = [0_u8; SPI_BUF_LEN];

        let Some(cmd_slot) = tx.get_mut(0) else { return };
        *cmd_slot = WRITE_CMD | PATTERN_CMD | (PAGE_PWM & 0x0F);

        let Some(reg_slot) = tx.get_mut(1) else { return };
        *reg_slot = 0x00;

        let Some(payload_slot) = tx.get_mut(2..) else { return };
        payload_slot.copy_from_slice(pwm);

        let Some(cs) = self.cs.get_mut(index) else { return };
        cs.set_low();
        if self.spi.write(&tx).await.is_err() {
            cs.set_high();
            return;
        }
        cs.set_high();
    }

    /// Initialise a single SNLED27351 driver chip.
    async fn init_driver(&mut self, index: usize, chip_current_tune: u8) {
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await;
        self.write_register(index, PAGE_FUNCTION, REG_PULLDOWNUP, PULLDOWNUP_ALL_ENABLED).await;
        self.write_register(index, PAGE_FUNCTION, REG_SCAN_PHASE, SCAN_PHASE_12_CHANNEL).await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE)
            .await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_2, SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE)
            .await;
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SLEEP, SOFTWARE_SLEEP_DISABLE).await;

        let led_control = [0xFF_u8; LED_CONTROL_REGISTER_COUNT];
        self.write(index, PAGE_LED_CONTROL, 0x00, &led_control).await;

        let pwm = [0_u8; PWM_REGISTER_COUNT];
        self.write(index, PAGE_PWM, 0x00, &pwm).await;

        let current_tune = [chip_current_tune; CURRENT_TUNE_REGISTER_COUNT];
        self.write(index, PAGE_CURRENT_TUNE, 0x00, &current_tune).await;

        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_NORMAL).await;
    }

    /// Write a register page in a single CS-asserted SPI burst.
    ///
    /// Allocates a stack buffer combining the 2-byte header and `data`
    /// so the entire transfer is one DMA operation with no gap.
    async fn write(&mut self, index: usize, page: u8, reg: u8, data: &[u8]) {
        let Some(cs) = self.cs.get_mut(index) else { return };

        let mut buf = [0_u8; SPI_BUF_LEN];

        let Some(cmd_slot) = buf.get_mut(0) else { return };
        *cmd_slot = WRITE_CMD | PATTERN_CMD | (page & 0x0F);

        let Some(reg_slot) = buf.get_mut(1) else { return };
        *reg_slot = reg;

        let data_len = data.len().min(PWM_REGISTER_COUNT);
        let payload_end = data_len.saturating_add(2);

        let Some(payload_slot) = buf.get_mut(2..payload_end) else { return };
        let Some(data_slice) = data.get(..data_len) else { return };
        payload_slot.copy_from_slice(data_slice);

        cs.set_low();
        let Some(out) = buf.get(..payload_end) else {
            cs.set_high();
            return;
        };
        if self.spi.write(out).await.is_err() {
            cs.set_high();
            return;
        }
        cs.set_high();
    }

    /// Write a single-byte register value in one burst.
    async fn write_register(&mut self, index: usize, page: u8, reg: u8, data: u8) {
        self.write(index, page, reg, from_ref(&data)).await;
    }
}

/// SNLED27351 driver for RGB LED control.
pub struct Snled27351<'peripherals> {
    /// PWM shadow buffers and dirty tracking.
    bufs: SnledBuffers,
    /// SPI bus and hardware pin handles.
    bus: SnledBus<'peripherals>,
    /// Static mapping of logical LEDs to driver channels.
    leds: &'static [SnledLed],
}

impl<'peripherals> Snled27351<'peripherals> {
    /// Flush all dirty PWM shadow buffers to hardware.
    pub async fn flush(&mut self) {
        for drv in 0..DRIVER_COUNT {
            let dirty = self.bufs.pwm_dirty.get(drv).copied().unwrap_or(false);
            if !dirty {
                continue;
            }
            let Some(buf) = self.bufs.pwm_buf.get(drv) else { continue };
            // `buf` is a reference into `self.bufs`; `flush_driver` only
            // touches `self.bus` — the borrow checker sees these as disjoint.
            self.bus.flush_driver(drv, buf).await;
            if let Some(flag) = self.bufs.pwm_dirty.get_mut(drv) {
                *flag = false;
            }
        }
    }

    /// Initialize all SNLED27351 driver chips.
    pub async fn init(&mut self, chip_current_tune: u8) {
        for cs in &mut self.bus.cs {
            cs.set_high();
        }
        self.bus.sdb.set_low();
        Timer::after_millis(5).await;
        self.bus.sdb.set_high();
        Timer::after_millis(5).await;

        for index in 0..DRIVER_COUNT {
            self.bus.init_driver(index, chip_current_tune).await;
        }
    }

    /// Create a new SNLED27351 driver with the provided LED map.
    pub const fn new(
        spi: Spi<'peripherals, Async, spi::mode::Master>,
        cs: [Output<'peripherals>; DRIVER_COUNT],
        sdb: Output<'peripherals>,
        leds: &'static [SnledLed],
    ) -> Self {
        Self { bus: SnledBus { cs, sdb, spi }, bufs: SnledBuffers::new(), leds }
    }

    /// Write pre-corrected PWM values for all LEDs and flush immediately.
    pub async fn set_all_leds(&mut self, red: u8, green: u8, blue: u8) {
        let leds: &'static [SnledLed] = self.leds;
        for &led in leds {
            self.bufs.stage_led(led, red, green, blue);
        }
        self.flush().await;
    }

    /// Write pre-corrected PWM values for a single LED by index and flush
    /// immediately.
    pub async fn set_led(&mut self, led_index: usize, red: u8, green: u8, blue: u8) {
        self.stage_led(led_index, red, green, blue);
        self.flush().await;
    }

    /// Stage RGB values for one LED by index without flushing.
    pub const fn stage_led(&mut self, led_index: usize, red: u8, green: u8, blue: u8) {
        let Some(&led) = self.leds.get(led_index) else { return };
        self.bufs.stage_led(led, red, green, blue);
    }
}
