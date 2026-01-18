pub const WRITE_CMD: u8 = 0 << 7;
pub const PATTERN_CMD: u8 = 2 << 4;

pub const PAGE_LED_CONTROL: u8 = 0x00;
pub const PAGE_PWM: u8 = 0x01;
pub const PAGE_FUNCTION: u8 = 0x03;
pub const PAGE_CURRENT_TUNE: u8 = 0x04;

pub const REG_SOFTWARE_SHUTDOWN: u8 = 0x00;
pub const SOFTWARE_SHUTDOWN_SSD_SHUTDOWN: u8 = 0x00;
pub const SOFTWARE_SHUTDOWN_SSD_NORMAL: u8 = 0x01;

pub const REG_PULLDOWNUP: u8 = 0x13;
pub const PULLDOWNUP_ALL_ENABLED: u8 = 0xAA;

pub const REG_SCAN_PHASE: u8 = 0x14;
pub const SCAN_PHASE_12_CHANNEL: u8 = 0x00;

pub const REG_SLEW_RATE_CONTROL_MODE_1: u8 = 0x15;
pub const SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE: u8 = 1 << 2;

pub const REG_SLEW_RATE_CONTROL_MODE_2: u8 = 0x16;
pub const SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE: u8 = 1 << 6;

pub const REG_SOFTWARE_SLEEP: u8 = 0x1A;
pub const SOFTWARE_SLEEP_DISABLE: u8 = 0x00;

pub const LED_CONTROL_REGISTER_COUNT: usize = 0x18;
pub const PWM_REGISTER_COUNT: usize = 0xC0;
pub const CURRENT_TUNE_REGISTER_COUNT: usize = 0x0C;
