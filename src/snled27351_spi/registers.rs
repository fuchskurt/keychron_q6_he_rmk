//! SNLED27351 register addresses, page selectors, and configuration values.
//!
//! All constants are derived from the SNLED27351 datasheet.  The SPI frame
//! format is: `WRITE_CMD | PATTERN_CMD | (page & 0x0F)`, register address,
//! then the data payload.

/// SPI command byte for write operations (AD[7:6] = 0b00).
pub const WRITE_CMD: u8 = 0x00;

/// Pattern field of the SPI command byte: AD[5:4] = 0b10 selects the
/// auto-increment write pattern (address increments automatically across
/// the payload).
pub const PATTERN_CMD: u8 = 2 << 4;

/// Page address for the LED control registers.
///
/// Each of the 24 bytes enables or disables individual LED channels
/// (one bit per channel).
pub const PAGE_LED_CONTROL: u8 = 0x00;

/// Page address for the PWM duty-cycle registers.
///
/// 192 bytes, one per output channel, set the PWM brightness (0 = off,
/// 255 = full brightness).
pub const PAGE_PWM: u8 = 0x01;

/// Page address for the function configuration registers (shutdown, scan
/// phase, slew rate, etc.).
pub const PAGE_FUNCTION: u8 = 0x03;

/// Page address for the per-channel current-tuning registers.
///
/// 12 bytes scale the maximum drive current for each channel group.
pub const PAGE_CURRENT_TUNE: u8 = 0x04;

/// Function-page register address: software shutdown control.
pub const REG_SOFTWARE_SHUTDOWN: u8 = 0x00;

/// Software shutdown value: driver enters low-power shutdown mode.
/// All outputs are disabled.
pub const SOFTWARE_SHUTDOWN_SSD_SHUTDOWN: u8 = 0x00;

/// Software shutdown value: driver operates normally (shutdown released).
pub const SOFTWARE_SHUTDOWN_SSD_NORMAL: u8 = 0x01;

/// Function-page register address: pull-down / pull-up resistor
/// configuration for un-driven output pins.
pub const REG_PULLDOWNUP: u8 = 0x13;

/// Pull-down/pull-up value: all pull-down and pull-up resistors enabled
/// (0xAA = alternating PD/PU pattern per the datasheet recommendation).
pub const PULLDOWNUP_ALL_ENABLED: u8 = 0xAA;

/// Function-page register address: scan-phase configuration.
pub const REG_SCAN_PHASE: u8 = 0x14;

/// Scan-phase value: 12-channel scan (all 12 column phases active).
pub const SCAN_PHASE_12_CHANNEL: u8 = 0x00;

/// Function-page register address: slew-rate control mode 1.
pub const REG_SLEW_RATE_CONTROL_MODE_1: u8 = 0x15;

/// Slew-rate mode 1 value: pre-discharge pre-charge enable (PDP_ENABLE,
/// bit 2 set).  Reduces EMI by limiting current slew rate at turn-on.
pub const SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE: u8 = 1 << 2;

/// Function-page register address: slew-rate control mode 2.
pub const REG_SLEW_RATE_CONTROL_MODE_2: u8 = 0x16;

/// Slew-rate mode 2 value: slope slew-rate limit enable (SSL_ENABLE,
/// bit 6 set).  Further reduces switching-edge EMI.
pub const SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE: u8 = 1 << 6;

/// Function-page register address: software sleep control.
pub const REG_SOFTWARE_SLEEP: u8 = 0x1A;

/// Software sleep value: normal operation (sleep mode disabled).
pub const SOFTWARE_SLEEP_DISABLE: u8 = 0x00;

/// Number of LED-control registers per driver chip (24 = 0x18).
///
/// Each bit enables or disables one of the 192 output channels.
pub const LED_CONTROL_REGISTER_COUNT: usize = 0x18;

/// Number of PWM registers per driver chip (192 = 0xC0).
///
/// One byte per output channel; covers 12 scan phases × 16 sub-addresses.
pub const PWM_REGISTER_COUNT: usize = 0xC0;

/// Number of current-tuning registers per driver chip (12 = 0x0C).
///
/// One byte per channel group; scales the maximum LED drive current.
pub const CURRENT_TUNE_REGISTER_COUNT: usize = 0x0C;
