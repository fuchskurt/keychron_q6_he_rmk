use embassy_stm32::{
    gpio::Output,
    mode::Blocking,
    spi::{self, Spi},
};
use embassy_time::Timer;

const DRIVER_COUNT: usize = 2;

const WRITE_CMD: u8 = 0 << 7;
const PATTERN_CMD: u8 = 2 << 4;
const PAGE_LED_CONTROL: u8 = 0x00;
const PAGE_PWM: u8 = 0x01;
const PAGE_FUNCTION: u8 = 0x03;
const PAGE_CURRENT_TUNE: u8 = 0x04;

const REG_SOFTWARE_SHUTDOWN: u8 = 0x00;
const SOFTWARE_SHUTDOWN_SSD_SHUTDOWN: u8 = 0x00;
const SOFTWARE_SHUTDOWN_SSD_NORMAL: u8 = 0x01;

const REG_PULLDOWNUP: u8 = 0x13;
const PULLDOWNUP_ALL_ENABLED: u8 = 0xAA;

const REG_SCAN_PHASE: u8 = 0x14;
const SCAN_PHASE_12_CHANNEL: u8 = 0x00;

const REG_SLEW_RATE_CONTROL_MODE_1: u8 = 0x15;
const SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE: u8 = 1 << 2;

const REG_SLEW_RATE_CONTROL_MODE_2: u8 = 0x16;
const SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE: u8 = 1 << 6;

const REG_SOFTWARE_SLEEP: u8 = 0x1A;
const SOFTWARE_SLEEP_DISABLE: u8 = 0x00;

const LED_CONTROL_REGISTER_COUNT: usize = 0x18;
const PWM_REGISTER_COUNT: usize = 0xC0;
const CURRENT_TUNE_REGISTER_COUNT: usize = 0x0C;

pub struct Snled27351<'d> {
    spi: Spi<'d, Blocking, spi::mode::Master>,
    cs: [Output<'d>; DRIVER_COUNT],
    sdb: Output<'d>,
}

impl<'d> Snled27351<'d> {
    pub fn new(spi: Spi<'d, Blocking, spi::mode::Master>, cs: [Output<'d>; DRIVER_COUNT], sdb: Output<'d>) -> Self {
        Self { spi, cs, sdb }
    }

    pub async fn init(&mut self, brightness: u8) {
        // Keep CS inactive
        for cs in &mut self.cs {
            cs.set_high();
        }

        // Hard reset the LED driver
        self.sdb.set_low();
        Timer::after_millis(5).await;
        self.sdb.set_high();
        Timer::after_millis(5).await;

        for index in 0..DRIVER_COUNT {
            self.init_driver(index, brightness);
        }
    }

    fn init_driver(&mut self, index: usize, brightness: u8) {
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN);
        self.write_register(index, PAGE_FUNCTION, REG_PULLDOWNUP, PULLDOWNUP_ALL_ENABLED);
        self.write_register(index, PAGE_FUNCTION, REG_SCAN_PHASE, SCAN_PHASE_12_CHANNEL);
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE);
        self.write_register(index, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_2, SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE);
        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SLEEP, SOFTWARE_SLEEP_DISABLE);

        let led_control = [0xFFu8; LED_CONTROL_REGISTER_COUNT];
        self.write(index, PAGE_LED_CONTROL, 0, &led_control);

        let pwm = [brightness; PWM_REGISTER_COUNT];
        self.write(index, PAGE_PWM, 0, &pwm);

        let current_tune = [0xFFu8; CURRENT_TUNE_REGISTER_COUNT];
        self.write(index, PAGE_CURRENT_TUNE, 0, &current_tune);

        self.write_register(index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_NORMAL);
    }

    fn write_register(&mut self, index: usize, page: u8, reg: u8, data: u8) {
        self.write(index, page, reg, core::slice::from_ref(&data));
    }

    fn write(&mut self, index: usize, page: u8, reg: u8, data: &[u8]) {
        if index >= DRIVER_COUNT {
            return;
        }

        self.cs[index].set_low();
        let header = [WRITE_CMD | PATTERN_CMD | (page & 0x0F), reg];

        let _ = self.spi.blocking_write(&header);
        let _ = self.spi.blocking_write(data);

        self.cs[index].set_high();
    }
}
