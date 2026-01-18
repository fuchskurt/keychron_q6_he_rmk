use crate::snled27351_spi::registers::*;
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use embassy_time::Timer;

const DRIVER_COUNT: usize = 2;

#[derive(Copy, Clone)]
pub struct SnledLed {
    pub driver: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

pub struct Snled27351<'d> {
    spi: Spi<'d, Async, spi::mode::Master>,
    cs: [Output<'d>; DRIVER_COUNT],
    sdb: Output<'d>,
    leds: &'static [SnledLed],
    global_brightness: u8,
}

impl<'d> Snled27351<'d> {
    pub fn new(
        spi: Spi<'d, Async, spi::mode::Master>,
        cs: [Output<'d>; DRIVER_COUNT],
        sdb: Output<'d>,
        leds: &'static [SnledLed],
    ) -> Self {
        Self { spi, cs, sdb, leds, global_brightness: 255 }
    }

    pub async fn init(&mut self) {
        // Keep CS inactive
        for cs in &mut self.cs {
            cs.set_high();
        }

        self.sdb.set_low();
        Timer::after_millis(5).await;
        self.sdb.set_high();
        Timer::after_millis(5).await;

        for index in 0..DRIVER_COUNT {
            self.init_driver(index).await;
        }
    }

    async fn set_global_brightness(&mut self, b: u8) { self.global_brightness = b; }

    pub async fn set_global_brightness_percent(&mut self, percent: u8) {
        let p = percent.min(100);
        let b = (p as u16 * 255 / 100) as u8;
        self.set_global_brightness(b).await;
    }

    #[inline]
    fn gamma2(&self, v: u8) -> u8 {
        // gamma ~2.0
        let x = v as u16;
        ((x * x + 127) / 255) as u8
    }

    #[inline]
    fn scale(&self, v: u8) -> u8 { ((v as u16 * self.global_brightness as u16 + 127) / 255) as u8 }

    #[inline]
    fn scaled_rgb(&self, r: u8, g: u8, b: u8) -> (u8, u8, u8) {
        let r_scaled = self.gamma2(self.scale(r));
        let g_scaled = self.gamma2(self.scale(g));
        let b_scaled = self.gamma2(self.scale(b));
        (r_scaled, g_scaled, b_scaled)
    }

    async fn write_led_rgb(&mut self, led: SnledLed, r: u8, g: u8, b: u8) {
        let drv = led.driver as usize;
        self.write_register(drv, PAGE_PWM, led.r, r).await;
        self.write_register(drv, PAGE_PWM, led.g, g).await;
        self.write_register(drv, PAGE_PWM, led.b, b).await;
    }

    async fn prepare_color(&mut self, r: u8, g: u8, b: u8, brightness: u8) -> (u8, u8, u8) {
        self.set_global_brightness_percent(brightness).await;
        self.scaled_rgb(r, g, b)
    }

    pub async fn set_color_all(&mut self, r: u8, g: u8, b: u8, brightness: u8) {
        if self.leds.is_empty() {
            return;
        }

        let (r_scaled, g_scaled, b_scaled) = self.prepare_color(r, g, b, brightness).await;

        for &led in self.leds.iter() {
            self.write_led_rgb(led, r_scaled, g_scaled, b_scaled).await;
        }
    }

    async fn init_driver(&mut self, index: usize) {
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await;
        self.write_register(index, PAGE_FUNCTION, REG_PULLDOWNUP, PULLDOWNUP_ALL_ENABLED).await;
        self.write_register(index, PAGE_FUNCTION, REG_SCAN_PHASE, SCAN_PHASE_12_CHANNEL).await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE)
            .await;
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_2, SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE)
            .await;
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SLEEP, SOFTWARE_SLEEP_DISABLE).await;

        // Enable LED control (PWM mode)
        let led_control = [0xFFu8; LED_CONTROL_REGISTER_COUNT];
        self.write(index, PAGE_LED_CONTROL, 0, &led_control).await;

        // Default brightness (off)
        let pwm = [0u8; PWM_REGISTER_COUNT];
        self.write(index, PAGE_PWM, 0x00, &pwm).await;

        // Max current tune
        let current_tune = [0xFFu8; CURRENT_TUNE_REGISTER_COUNT];
        self.write(index, PAGE_CURRENT_TUNE, 0, &current_tune).await;

        // Exit shutdown
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_NORMAL).await;
    }

    async fn write_register(&mut self, index: usize, page: u8, reg: u8, data: u8) {
        self.write(index, page, reg, core::slice::from_ref(&data)).await;
    }

    async fn write(&mut self, index: usize, page: u8, reg: u8, data: &[u8]) {
        if index >= DRIVER_COUNT {
            return;
        }
        self.cs[index].set_low();
        let header = [WRITE_CMD | PATTERN_CMD | (page & 0x0F), reg];
        if self.spi.write(&header).await.is_err() {
            self.cs[index].set_high();
            return;
        }
        if self.spi.write(data).await.is_err() {
            self.cs[index].set_high();
            return;
        }
        self.cs[index].set_high();
    }
}
