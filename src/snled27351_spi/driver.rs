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
    READ_CMD,
    REG_PULLDOWNUP,
    REG_SCAN_PHASE,
    REG_SLEW_RATE_CONTROL_MODE_1,
    REG_SLEW_RATE_CONTROL_MODE_2,
    REG_SOFTWARE_SHUTDOWN,
    REG_SOFTWARE_SLEEP,
    REG_THERMAL,
    SCAN_PHASE_12_CHANNEL,
    SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE,
    SLEW_RATE_CONTROL_MODE_2_DSL_ENABLE,
    SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE,
    SOFTWARE_SHUTDOWN_SSD_NORMAL,
    SOFTWARE_SHUTDOWN_SSD_SHUTDOWN,
    SOFTWARE_SLEEP_DISABLE,
    WRITE_CMD,
};
use core::{
    hint::{cold_path, likely},
    slice::from_ref,
};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use embassy_time::{Duration, Timer};

const DRIVER_INIT_DELAY: Duration = Duration::from_millis(500);

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

/// PWM shadow buffer and dirty flag for a single SNLED27351 driver.
struct SnledDriverBuf {
    /// Whether this driver's buffer has unsent changes.
    dirty: bool,
    /// PWM shadow buffer; index is the raw register address.
    pwm: [u8; PWM_REGISTER_COUNT],
}

impl SnledDriverBuf {
    /// Write scaled RGB values into the PWM shadow buffer for one LED.
    const fn stage_led(&mut self, led: SnledLed, red: u8, green: u8, blue: u8) {
        match self.pwm.get_mut(usize::from(led.red)) {
            Some(slot) => *slot = red,
            None => cold_path(),
        }
        match self.pwm.get_mut(usize::from(led.green)) {
            Some(slot) => *slot = green,
            None => cold_path(),
        }
        match self.pwm.get_mut(usize::from(led.blue)) {
            Some(slot) => *slot = blue,
            None => cold_path(),
        }
        self.dirty = true;
    }
}

/// PWM shadow buffers and dirty flags for all drivers, split from hardware
/// handles so they can be borrowed independently of `spi` and `cs`.
struct SnledBuffers<const DRIVER_COUNT: usize> {
    /// Per-driver PWM buffer and dirty flag.
    drivers: [SnledDriverBuf; DRIVER_COUNT],
}

impl<const DRIVER_COUNT: usize> SnledBuffers<DRIVER_COUNT> {
    /// Create zeroed buffers with no pending changes.
    const fn new() -> Self {
        Self { drivers: [const { SnledDriverBuf { pwm: [0_u8; PWM_REGISTER_COUNT], dirty: false } }; DRIVER_COUNT] }
    }

    /// Write scaled RGB values into the PWM shadow buffer for one LED.
    const fn stage_led(&mut self, led: SnledLed, red: u8, green: u8, blue: u8) {
        let drv = usize::from(led.driver);
        let Some(buf) = self.drivers.get_mut(drv) else { return };
        buf.stage_led(led, red, green, blue);
    }
}

/// SNLED27351 SPI bus handle.
struct SnledBus<'peripherals, const DRIVER_COUNT: usize> {
    /// Chip-select outputs for each SNLED driver instance.
    cs: [Output<'peripherals>; DRIVER_COUNT],
    /// Shutdown (SDB) control pin for enabling or disabling the driver.
    sdb: Output<'peripherals>,
    /// SPI peripheral used to communicate with the SNLED driver(s).
    spi: Spi<'peripherals, Async, spi::mode::Master>,
}

impl<const DRIVER_COUNT: usize> SnledBus<'_, DRIVER_COUNT> {
    /// Flush one driver's dirty PWM buffer to hardware without copying it.
    ///
    /// Takes the buffer slice directly — no stack copy needed because
    /// `buf` and `spi`/`cs` are in separate structs.
    /// Maximum SPI payload: 2-byte header + PWM register block.
    ///
    /// Returns `true` if succeeded, `false` on any error.
    async fn flush_driver(&mut self, index: usize, pwm: &[u8; PWM_REGISTER_COUNT]) -> bool {
        let mut tx = [0_u8; PWM_REGISTER_COUNT.saturating_add(2)];

        let Some(cmd_slot) = tx.get_mut(0) else { return false };
        *cmd_slot = WRITE_CMD | PATTERN_CMD | (PAGE_PWM & 0x0F);

        // tx[1] is the register address (0x00 = start of PWM page); already zero from
        // init.
        let Some(payload_slot) = tx.get_mut(2..) else { return false };
        payload_slot.copy_from_slice(pwm);

        let Some(cs) = self.cs.get_mut(index) else { return false };
        cs.set_low();
        let ok = self.spi.write(&tx).await.is_ok();
        cs.set_high();
        ok
    }

    /// Initialize a single SNLED27351 driver chip.
    async fn init_driver(&mut self, index: usize, chip_current_tune: u8) {
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await;
        self.write_register(index, PAGE_FUNCTION, REG_PULLDOWNUP, PULLDOWNUP_ALL_ENABLED).await;
        self.write_register(index, PAGE_FUNCTION, REG_SCAN_PHASE, SCAN_PHASE_12_CHANNEL).await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE)
            .await;
        self.write_register(
            index,
            PAGE_FUNCTION,
            REG_SLEW_RATE_CONTROL_MODE_2,
            SLEW_RATE_CONTROL_MODE_2_DSL_ENABLE | SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE,
        )
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

    /// Read a single register byte from a driver chip.
    ///
    /// Sends the read header (READ_CMD | PATTERN_CMD | page) followed by the
    /// register address, then receives one byte. Returns `None` on SPI error.
    async fn read_register(&mut self, index: usize, page: u8, reg: u8) -> Option<u8> {
        let tx: [u8; 3] = [READ_CMD | PATTERN_CMD | (page & 0x0F), reg, 0x00];
        let mut rx = [0_u8; 3];

        let Some(cs) = self.cs.get_mut(index) else { return None };
        cs.set_low();
        let ok = self.spi.transfer(&mut rx, &tx).await.is_ok();
        cs.set_high();

        if ok { rx.get(2).copied() } else { None }
    }

    /// Write a register page in a single CS-asserted SPI burst.
    ///
    /// Allocates a stack buffer combining the 2-byte header and `data`
    /// so the entire transfer is one DMA operation with no gap.
    async fn write(&mut self, index: usize, page: u8, reg: u8, data: &[u8]) {
        let Some(cs) = self.cs.get_mut(index) else { return };

        let mut buf = [0_u8; PWM_REGISTER_COUNT.saturating_add(2)];

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
pub struct Snled27351<'peripherals, const DRIVER_COUNT: usize> {
    /// PWM shadow buffers and dirty tracking.
    bufs: SnledBuffers<DRIVER_COUNT>,
    /// SPI bus and hardware pin handles.
    bus: SnledBus<'peripherals, DRIVER_COUNT>,
    /// Static mapping of logical LEDs to driver channels.
    leds: &'static [SnledLed],
}

impl<'peripherals, const DRIVER_COUNT: usize> Snled27351<'peripherals, DRIVER_COUNT> {
    /// Flush all dirty PWM shadow buffers to hardware.
    pub async fn flush(&mut self) {
        for (drv, buf) in self.bufs.drivers.iter_mut().enumerate() {
            if likely(!buf.dirty) {
                continue;
            }
            if self.bus.flush_driver(drv, &buf.pwm).await {
                buf.dirty = false;
            }
        }
    }

    /// Initialize all SNLED27351 driver chips.
    pub async fn init(&mut self, chip_current_tune: u8) {
        for cs in &mut self.bus.cs {
            cs.set_high();
        }
        self.bus.sdb.set_low();
        Timer::after(DRIVER_INIT_DELAY).await;
        self.bus.sdb.set_high();
        Timer::after(DRIVER_INIT_DELAY).await;

        for index in 0..DRIVER_COUNT {
            self.bus.init_driver(index, chip_current_tune).await;
        }
    }

    /// Read the thermal flag from driver `index`.
    ///
    /// Returns `true` if the chip reports temperature ≥ 70 °C (TDF bit set),
    /// `false` if below threshold or if the read fails.
    pub async fn check_thermal_flag_set(&mut self, index: usize) -> bool {
        self.bus.read_register(index, PAGE_FUNCTION, REG_THERMAL).await.map(|v| v & 0x01 != 0).unwrap_or(false)
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
