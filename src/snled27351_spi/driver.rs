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

/// SNLED27351 driver for RGB LED control.
pub struct Snled27351<'peripherals> {
    /// Chip-select outputs for each SNLED driver instance.
    cs: [Output<'peripherals>; DRIVER_COUNT],
    /// Global brightness scaling applied to all LEDs.
    global_brightness: u8,
    /// Static mapping of logical LEDs to driver channels.
    leds: &'static [SnledLed],
    /// Shutdown (SDB) control pin for enabling or disabling the driver.
    sdb: Output<'peripherals>,
    /// SPI peripheral used to communicate with the SNLED driver(s).
    spi: Spi<'peripherals, Async, spi::mode::Master>,
}

impl<'peripherals> Snled27351<'peripherals> {
    /// Initialize all SNLED27351 driver chips.
    pub async fn init(&mut self, chip_current_tune: u8) {
        // Keep CS inactive
        for cs in &mut self.cs {
            cs.set_high();
        }

        self.sdb.set_low();
        Timer::after_millis(5).await;
        self.sdb.set_high();
        Timer::after_millis(5).await;

        for index in 0..DRIVER_COUNT {
            self.init_driver(index, chip_current_tune).await;
        }
    }

    /// Initialize a single SNLED27351 driver.
    async fn init_driver(&mut self, index: usize, chip_current_tune: u8) {
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await;
        self.write_register(index, PAGE_FUNCTION, REG_PULLDOWNUP, PULLDOWNUP_ALL_ENABLED).await;
        self.write_register(index, PAGE_FUNCTION, REG_SCAN_PHASE, SCAN_PHASE_12_CHANNEL).await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE)
            .await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_2, SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE)
            .await;
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SLEEP, SOFTWARE_SLEEP_DISABLE).await;

        // Enable LED control (PWM mode)
        let led_control = [0xFF_u8; LED_CONTROL_REGISTER_COUNT];
        self.write(index, PAGE_LED_CONTROL, 0x00, &led_control).await;

        // Default brightness (off)
        let pwm = [0_u8; PWM_REGISTER_COUNT];
        self.write(index, PAGE_PWM, 0x00, &pwm).await;

        // Max current tune
        let current_tune = [chip_current_tune; CURRENT_TUNE_REGISTER_COUNT];
        self.write(index, PAGE_CURRENT_TUNE, 0x00, &current_tune).await;

        // Exit shutdown
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_NORMAL).await;
    }

    /// Create a new SNLED27351 driver with the provided LED map.
    pub const fn new(
        spi: Spi<'peripherals, Async, spi::mode::Master>,
        cs: [Output<'peripherals>; DRIVER_COUNT],
        sdb: Output<'peripherals>,
        leds: &'static [SnledLed],
    ) -> Self {
        Self { spi, cs, sdb, leds, global_brightness: 255 }
    }

    /// Prepare an RGB value for a given brightness.
    const fn prepare_color(&mut self, red: u8, green: u8, blue: u8, brightness: u8) -> (u8, u8, u8) {
        self.set_global_brightness_percent(brightness.clamp(0, 100));
        self.scaled_rgb(red, green, blue)
    }

    #[inline]
    /// Scale a value using the global brightness.
    const fn scale(&self, value: u8) -> u8 {
        u8::try_from(
            u16::from(value).saturating_mul(u16::from(self.global_brightness)).saturating_add(127).saturating_div(255),
        )
        .unwrap_or_default()
    }

    #[inline]
    /// Apply scaling and gamma to an RGB tuple.
    const fn scaled_rgb(&self, red: u8, green: u8, blue: u8) -> (u8, u8, u8) {
        let red_scaled = gamma2(self.scale(red));
        let green_scaled = gamma2(self.scale(green));
        let blue_scaled = gamma2(self.scale(blue));
        (red_scaled, green_scaled, blue_scaled)
    }

    /// Set the color for a single LED by index.
    pub async fn set_color(&mut self, led_index: usize, red: u8, green: u8, blue: u8, brightness: u8) {
        let Some(&led) = self.leds.get(led_index) else {
            return;
        };

        let (r_scaled, g_scaled, b_scaled) = self.prepare_color(red, green, blue, brightness);
        self.write_led_rgb(led, r_scaled, g_scaled, b_scaled).await;
    }

    /// Set the color for all LEDs.
    pub async fn set_color_all(&mut self, red: u8, green: u8, blue: u8, brightness: u8) {
        if self.leds.is_empty() {
            return;
        }
        let brightness_clamped = brightness.clamp(0, 100);

        let (r_scaled, g_scaled, b_scaled) = self.prepare_color(red, green, blue, brightness_clamped);

        for &led in self.leds {
            self.write_led_rgb(led, r_scaled, g_scaled, b_scaled).await;
        }
    }

    /// Set the color for all LEDs with a soft-start ramp.
    pub async fn set_color_all_softstart(
        &mut self,
        red: u8,
        green: u8,
        blue: u8,
        brightness: u8,
        steps: u8,
        ramp_time_ms: u32,
    ) {
        let target = brightness.clamp(0, 100);
        if target == 0 || self.leds.is_empty() {
            self.set_color_all(red, green, blue, target).await;
            return;
        }

        let steps_u32 = u32::from(steps.max(1));
        let delay_ms = match ramp_time_ms.checked_div(steps_u32) {
            Some(value) if value > 0 => u64::from(value),
            _ => 1,
        };

        for step in 0..=steps_u32 {
            let scaled = u32::from(target).saturating_mul(step);
            let percent = scaled.checked_div(steps_u32).map_or(0, |value| u8::try_from(value).unwrap_or_default());
            self.set_color_all(red, green, blue, percent).await;
            Timer::after_millis(delay_ms).await;
        }
    }

    /// Set the global brightness scale (0-255).
    const fn set_global_brightness(&mut self, brightness: u8) { self.global_brightness = brightness; }

    /// Set the global brightness as a percentage.
    pub const fn set_global_brightness_percent(&mut self, percent: u8) {
        let percent_parameter = percent.min(100);
        let brightness =
            u8::try_from(u16::from(percent_parameter).saturating_mul(255).saturating_div(100)).unwrap_or_default();
        self.set_global_brightness(brightness);
    }

    /// Write data to the specified register page.
    async fn write(&mut self, index: usize, page: u8, reg: u8, data: &[u8]) {
        let Some(cs) = self.cs.get_mut(index) else {
            return;
        };
        cs.set_low();
        let header = [WRITE_CMD | PATTERN_CMD | (page & 0x0F), reg];
        if self.spi.write(&header).await.is_err() {
            cs.set_high();
            return;
        }
        if self.spi.write(data).await.is_err() {
            cs.set_high();
            return;
        }
        cs.set_high();
    }

    /// Write an RGB value to a specific LED.
    async fn write_led_rgb(&mut self, led: SnledLed, red: u8, green: u8, blue: u8) {
        let drv = usize::from(led.driver);
        self.write_register(drv, PAGE_PWM, led.red, red).await;
        self.write_register(drv, PAGE_PWM, led.green, green).await;
        self.write_register(drv, PAGE_PWM, led.blue, blue).await;
    }

    /// Write a single-byte register value.
    async fn write_register(&mut self, index: usize, page: u8, reg: u8, data: u8) {
        self.write(index, page, reg, from_ref(&data)).await;
    }
}

/// Apply a gamma 2.0 correction curve.
#[inline]
const fn gamma2(value: u8) -> u8 {
    // gamma ~2.0
    let x = u16::from(value);
    u8::try_from(x.saturating_mul(x).saturating_add(127).saturating_div(255)).unwrap_or_default()
}
