//! Backlight control modules.

/// Gamma 2.2 correction lookup for 8-bit LED PWM values.
mod gamma;
/// LED indicator backlight handling.
pub mod processor;
/// Display state model, colors, and frame painting.
mod render;
/// Backlight task: command handling, thermal and power policy.
pub mod runner;
