//! Backlight control modules.

/// Gamma 2.2 correction lookup for 8-bit LED PWM values.
mod gamma;
/// Backlight task initialization.
pub mod init;
/// LED indicator backlight handling.
pub mod processor;
