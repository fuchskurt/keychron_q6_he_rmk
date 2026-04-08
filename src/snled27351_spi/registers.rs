//! SNLED27351 register addresses, page selectors, and configuration values.
//!
//! All constants are derived from the SNLED27351 datasheet.  The SPI frame
//! format is: `WRITE_CMD | PATTERN_CMD | (page & 0x0F)`, register address,
//! then the data payload.
#![expect(dead_code, reason = "Register definitions kept for completeness; not all are used at runtime")]

/// SPI command byte for write operations (bit 7 = 0).
pub const WRITE_CMD: u8 = 0x00;
/// SPI command byte for read operations (bit 7 = 1).
pub const READ_CMD: u8 = 0x80;
/// Checking pattern field of the SPI command byte: bits \[6:4\] = 0b010 (fixed
/// value required by the protocol; must be combined with WRITE_CMD or
/// READ_CMD and the page nibble).
pub const PATTERN_CMD: u8 = 2 << 4;

/// Page address for the LED control, open, and short registers.
///
/// - 0x00–0x17: LED Control Register (on/off per channel, 24 bytes)
/// - 0x18–0x2F: LED Open Register   (open-detection status, 24 bytes)
/// - 0x30–0x47: LED Short Register  (short-detection status, 24 bytes)
pub const PAGE_LED_CONTROL: u8 = 0x00;
/// Page address for the PWM duty-cycle registers.
///
/// 192 bytes (0x00–0xBF), one per output channel; 0 = off, 255 = full
/// brightness.
pub const PAGE_PWM: u8 = 0x01;
/// Page address for the function configuration registers.
///
/// Contains shutdown, scan phase, slew rate, open/short detection, sleep, etc.
pub const PAGE_FUNCTION: u8 = 0x03;
/// Page address for the per-channel constant-current step registers.
///
/// 12 bytes (0x00–0x0B), one per CB channel.
pub const PAGE_CURRENT_TUNE: u8 = 0x04;

// ---------------------------------------------------------------------------
// Function page (PAGE_FUNCTION = 0x03) register addresses
// ---------------------------------------------------------------------------

/// Function-page register: software shutdown control (bit 0 = SSD).
pub const REG_SOFTWARE_SHUTDOWN: u8 = 0x00;
/// Software shutdown value: driver enters shutdown mode (SSD = 0).
pub const SOFTWARE_SHUTDOWN_SSD_SHUTDOWN: u8 = 0x00;
/// Software shutdown value: driver operates normally (SSD = 1).
pub const SOFTWARE_SHUTDOWN_SSD_NORMAL: u8 = 0x01;

/// Function-page register: LED driver ID (read-only, default 0x8A).
pub const REG_ID: u8 = 0x11;

/// Function-page register: thermal detector flag (bit 0 = TDF, read-only).
///
/// TDF = 0: system temperature is below 70 °C.
/// TDF = 1: system temperature has reached or exceeded 70 °C.
pub const REG_THERMAL: u8 = 0x12;

/// Function-page register: pull-down / pull-up resistor configuration.
///
/// Bit layout: \[CAPD, -, CAPD2, -, CBPU, -, CBPU2, -\]
pub const REG_PULLDOWNUP: u8 = 0x13;
/// Pull-down/pull-up value: all four control bits enabled
/// (CAPD | CAPD2 | CBPU | CBPU2 = 0xAA).
pub const PULLDOWNUP_ALL_ENABLED: u8 = 0xAA;
/// PDU bit: pull-down for CA1–CA16 on latch-data time (bit 7).
pub const PULLDOWNUP_CAPD: u8 = 1 << 7;
/// PDU bit: pull-down for CA1–CA16 on scan (PWM-off) time (bit 5).
pub const PULLDOWNUP_CAPD2: u8 = 1 << 5;
/// PDU bit: pull-up for CB1–CB12 on latch-data time (bit 3).
pub const PULLDOWNUP_CBPU: u8 = 1 << 3;
/// PDU bit: pull-up for CB1–CB12 on scan time (bit 1).
pub const PULLDOWNUP_CBPU2: u8 = 1 << 1;

/// Function-page register: scan-phase configuration (bits \[3:0\] = SP).
pub const REG_SCAN_PHASE: u8 = 0x14;
/// Scan-phase value: all 12 phases active (CB1–CB12, SP = 0b0000).
pub const SCAN_PHASE_12_CHANNEL: u8 = 0x00;

/// Function-page register: slew-rate control mode 1 (bit 2 = PDP_EN).
///
/// Note: the datasheet requires this register to be set to 0x00 or 0x04 only.
pub const REG_SLEW_RATE_CONTROL_MODE_1: u8 = 0x15;
/// Slew-rate mode 1 value: PWM delay-phase enable (PDP_EN, bit 2).
pub const SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE: u8 = 1 << 2;

/// Function-page register: slew-rate control mode 2 (bit 7 = DSL, bit 6 = SSL).
pub const REG_SLEW_RATE_CONTROL_MODE_2: u8 = 0x16;
/// Slew-rate mode 2 value: driving-channel (CA) slew-rate enable (DSL, bit 7).
pub const SLEW_RATE_CONTROL_MODE_2_DSL_ENABLE: u8 = 1 << 7;
/// Slew-rate mode 2 value: sinking-channel (CB) slew-rate enable (SSL, bit 6).
pub const SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE: u8 = 1 << 6;

/// Function-page register: open/short detection enable.
///
/// Bit 7 = ODS (start open detection), bit 6 = SDS (start short detection).
/// Both bits are write-only and cleared automatically by hardware.
pub const REG_OPEN_SHORT_ENABLE: u8 = 0x17;
/// Open/short enable bit: start open-circuit detection (ODS, bit 7).
pub const OPEN_SHORT_ENABLE_ODS: u8 = 1 << 7;
/// Open/short enable bit: start short-circuit detection (SDS, bit 6).
pub const OPEN_SHORT_ENABLE_SDS: u8 = 1 << 6;

/// Function-page register: open/short detection latched PWM duty (OSDD).
///
/// Valid range is 0x01–0xFA; 0x00 is reserved.  Writing this register
/// triggers a detection scan; hardware clears it automatically when done.
pub const REG_OPEN_SHORT_DUTY: u8 = 0x18;

/// Function-page register: open/short detection completion flags.
///
/// Bit 7 = ODINT (open done), bit 6 = SDINT (short done); set by hardware.
pub const REG_OPEN_SHORT_FLAG: u8 = 0x19;
/// Open/short flag bit: open-detection scan completed (ODINT, bit 7).
pub const OPEN_SHORT_FLAG_ODINT: u8 = 1 << 7;
/// Open/short flag bit: short-detection scan completed (SDINT, bit 6).
pub const OPEN_SHORT_FLAG_SDINT: u8 = 1 << 6;

/// Function-page register: software sleep control.
///
/// Bit 1 = SLEEP (write-only, cleared on wakeup), bit 0 = IREF_EN (keep 0).
pub const REG_SOFTWARE_SLEEP: u8 = 0x1A;
/// Software sleep value: normal operation (sleep disabled).
pub const SOFTWARE_SLEEP_DISABLE: u8 = 0x00;
/// Software sleep value: enter software sleep mode (SLEEP bit 1).
///
/// Wakeup sources: SDB pin, SYSRST pin, or SCL/SCK level change.
pub const SOFTWARE_SLEEP_ENABLE: u8 = 1 << 1;

// ---------------------------------------------------------------------------
// Register-block sizes
// ---------------------------------------------------------------------------

/// Number of LED-control registers per chip (24 = 0x18, addresses 0x00–0x17).
///
/// Each bit enables or disables one of the 192 output channels.
pub const LED_CONTROL_REGISTER_COUNT: usize = 0x18;
/// Number of PWM registers per chip (192 = 0xC0, addresses 0x00–0xBF).
///
/// One byte per output channel; covers 16 CA rows × 12 CB columns.
pub const PWM_REGISTER_COUNT: usize = 0xC0;
/// Number of current-tuning registers per chip (12 = 0x0C, addresses
/// 0x00–0x0B).
///
/// One byte per CB channel; scales the constant sink current.
pub const CURRENT_TUNE_REGISTER_COUNT: usize = 0x0C;
