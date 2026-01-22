//! SNLED27351 register and command constants.
/// Register or command constant `WRITE_CMD`.
pub const WRITE_CMD: u8 = 0 << 7;
/// Register or command constant `PATTERN_CMD`.
pub const PATTERN_CMD: u8 = 2 << 4;

/// Register or command constant `PAGE_LED_CONTROL`.
pub const PAGE_LED_CONTROL: u8 = 0x00;
/// Register or command constant `PAGE_PWM`.
pub const PAGE_PWM: u8 = 0x01;
/// Register or command constant `PAGE_FUNCTION`.
pub const PAGE_FUNCTION: u8 = 0x03;
/// Register or command constant `PAGE_CURRENT_TUNE`.
pub const PAGE_CURRENT_TUNE: u8 = 0x04;

/// Register or command constant `REG_SOFTWARE_SHUTDOWN`.
pub const REG_SOFTWARE_SHUTDOWN: u8 = 0x00;
/// Register or command constant `SOFTWARE_SHUTDOWN_SSD_SHUTDOWN`.
pub const SOFTWARE_SHUTDOWN_SSD_SHUTDOWN: u8 = 0x00;
/// Register or command constant `SOFTWARE_SHUTDOWN_SSD_NORMAL`.
pub const SOFTWARE_SHUTDOWN_SSD_NORMAL: u8 = 0x01;

/// Register or command constant `REG_PULLDOWNUP`.
pub const REG_PULLDOWNUP: u8 = 0x13;
/// Register or command constant `PULLDOWNUP_ALL_ENABLED`.
pub const PULLDOWNUP_ALL_ENABLED: u8 = 0xAA;

/// Register or command constant `REG_SCAN_PHASE`.
pub const REG_SCAN_PHASE: u8 = 0x14;
/// Register or command constant `SCAN_PHASE_12_CHANNEL`.
pub const SCAN_PHASE_12_CHANNEL: u8 = 0x00;

/// Register or command constant `REG_SLEW_RATE_CONTROL_MODE_1`.
pub const REG_SLEW_RATE_CONTROL_MODE_1: u8 = 0x15;
/// Register or command constant `SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE`.
pub const SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE: u8 = 1 << 2;

/// Register or command constant `REG_SLEW_RATE_CONTROL_MODE_2`.
pub const REG_SLEW_RATE_CONTROL_MODE_2: u8 = 0x16;
/// Register or command constant `SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE`.
pub const SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE: u8 = 1 << 6;

/// Register or command constant `REG_SOFTWARE_SLEEP`.
pub const REG_SOFTWARE_SLEEP: u8 = 0x1A;
/// Register or command constant `SOFTWARE_SLEEP_DISABLE`.
pub const SOFTWARE_SLEEP_DISABLE: u8 = 0x00;

/// Register or command constant `LED_CONTROL_REGISTER_COUNT`.
pub const LED_CONTROL_REGISTER_COUNT: usize = 0x18;
/// Register or command constant `PWM_REGISTER_COUNT`.
pub const PWM_REGISTER_COUNT: usize = 0xC0;
/// Register or command constant `CURRENT_TUNE_REGISTER_COUNT`.
pub const CURRENT_TUNE_REGISTER_COUNT: usize = 0x0C;
