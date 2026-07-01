//! Backlight painting: the display state model, color and brightness
//! constants, the gamma/brightness correction pipeline, and the frame
//! render functions.
//!
//! Everything here answers "what should the LEDs show for this state";
//! deciding *when* to repaint (command handling, thermal polling, USB
//! power transitions) lives in `task`.

use crate::backlight::gamma;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{Spi, mode::Master},
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;
use embedded_hal_async::spi::ErrorType;
use snled27351_driver::{
    driver::Driver,
    transport::spi::{Controller, TransportError},
};

/// LED index for the Caps Lock indicator key.
const CAPS_LOCK_LED_INDEX: usize = 62;
/// LED index for the Num Lock indicator key.
const NUM_LOCK_LED_INDEX: usize = 37;
/// Default global brightness percentage when no throttle is active.
pub(super) const FULL_BRIGHTNESS: u8 = 100;
/// Brightness percentage applied to indicator LEDs (0-100).
const INDICATOR_BRIGHTNESS: u8 = 100;
/// RGB color for the active-lock indicator state (red).
const INDICATOR_RED: (u8, u8, u8) = (255, 0, 0);
/// RGB color for the inactive-lock indicator state (white).
pub(super) const INDICATOR_WHITE: (u8, u8, u8) = (255, 255, 255);
/// RGB color for a disabled indicator (off).
pub(super) const INDICATOR_OFF: (u8, u8, u8) = (0, 0, 0);
/// Background color during the zero-travel calibration phase and the
/// EEPROM-write-failure signal (amber).
pub(super) const CALIB_AMBER: (u8, u8, u8) = (255, 120, 0);
/// Solid color used for the post-calibration success hold and for each
/// individually confirmed key during the full-travel pass (green).
pub(super) const CALIB_GREEN: (u8, u8, u8) = (0, 220, 80);
/// Number of brightness steps in the soft-start ramp.
const SOFTSTART_STEPS: u8 = 50;
/// Total duration of the soft-start ramp in milliseconds.
const SOFTSTART_RAMP_MS: u32 = 1000;

/// Concrete SPI device type wrapping one chip-select pin.
pub(super) type Device = SpiDevice<'static, ThreadModeRawMutex, Spi<'static, Async, Master>, Output<'static>>;
/// Concrete SPI transport type for this keyboard.
pub(super) type Transport = Controller<Device, Output<'static>, 2>;
/// Concrete driver type for this keyboard.
pub(super) type BacklightDriver = Driver<Transport, 2>;
/// Bus error type shared by all async driver helpers.
pub(super) type BusError = TransportError<<Device as ErrorType>::Error>;

/// Packed boolean state for [`BacklightState`], one bit per flag.
///
/// Holds the lock-indicator, host-connection, and calibration-in-progress
/// flags in a single byte so [`BacklightState`] stays within the bool-field
/// budget while keeping named, type-checked accessors.
#[derive(Clone, Copy, Default)]
pub(super) struct BacklightFlags {
    /// One bit per flag; see the `*_FLAG` masks.
    bits: u8,
}

impl BacklightFlags {
    /// Caps Lock is currently active.
    pub(super) const CAPS_LOCK: u8 = 0b0001;
    /// The USB host is currently connected and enumerated.
    pub(super) const HOST_CONNECTED: u8 = 0b0010;
    /// Num Lock is currently active.
    pub(super) const NUM_LOCK: u8 = 0b0100;

    /// Read the flag selected by `mask`.
    pub(super) const fn get(self, mask: u8) -> bool { self.bits & mask != 0 }

    /// Set or clear the flag selected by `mask`.
    pub(super) const fn set(&mut self, mask: u8, on: bool) {
        if on {
            self.bits |= mask;
        } else {
            self.bits &= !mask;
        }
    }
}

/// Which guided-calibration frame currently owns the display.
///
/// Tracked in [`BacklightState`] so any asynchronous repaint (thermal
/// throttle, USB connect/resume transitions) can redraw the frame the
/// calibration flow expects instead of clobbering it with the normal white
/// background. `Full` and `Zero` both mean a first-boot calibration is in
/// progress.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub(super) enum CalibDisplay {
    /// Full-travel pass; red-to-blue gradient plus per-key green overlays.
    Full,
    /// No calibration in progress; normal white background.
    #[default]
    None,
    /// Zero-travel pass (or EEPROM-write-failure signal); solid amber.
    Zero,
}

/// Full backlight state, kept in one place so any path can do a consistent
/// redraw without losing information.
#[derive(Clone, Copy, Default)]
pub(super) struct BacklightState {
    /// Global brightness percentage (0-100); reduced under thermal throttle.
    pub brightness:      u8 = FULL_BRIGHTNESS,
    /// The calibration frame (if any) that currently owns the display.
    ///
    /// Set to [`CalibDisplay::Zero`] when the zero-travel pass starts,
    /// [`CalibDisplay::Full`] for the full-travel pass, and back to
    /// [`CalibDisplay::None`] once calibration completes and the keyboard
    /// returns to normal white operation. Consulted by every asynchronous
    /// repaint path.
    pub calib_display:   CalibDisplay,
    /// Bitset of LED indices confirmed calibrated during the full-travel pass.
    ///
    /// Bit `i` set means LED `i` should be painted solid green by
    /// [`render_calib`]. Cleared to zero when calibration completes
    /// and the keyboard returns to normal white operation.
    pub calib_leds_done: u128,
    /// Whole-keyboard gradient percentage (0-100) during the full-travel pass.
    ///
    /// Drives the red-blue background interpolation in [`render_calib`].
    /// Cleared to zero alongside [`BacklightState::calib_leds_done`] on
    /// calibration completion.
    pub calib_pct:       u8,
    /// Packed lock / host-connection flags.
    ///
    /// `host_connected` is tracked across Power commands to detect
    /// connect/disconnect transitions. On disconnect all LEDs are turned
    /// off; on reconnect the backlight is restored.
    pub flags:           BacklightFlags,
}

/// Performs a brightness ramp on all LEDs.
///
/// Linearly steps brightness from 0 up to `target_brightness` over
/// [`SOFTSTART_STEPS`] increments across [`SOFTSTART_RAMP_MS`] milliseconds.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails during the ramp.
pub(super) async fn brightness_ramp(
    driver: &mut BacklightDriver,
    base_color: (u8, u8, u8),
    target_brightness: u8,
    ramp_up: bool,
    state: BacklightState,
) -> Result<(), BusError> {
    let (base_red, base_green, base_blue) = base_color;
    let target = u32::from(target_brightness.min(100));
    let steps = u32::from(SOFTSTART_STEPS.max(1));
    let delay_ms = u64::from(SOFTSTART_RAMP_MS.checked_div(steps).unwrap_or(0));

    for step in 0..=steps {
        let pct_step = if ramp_up { step } else { steps.saturating_sub(step) };
        let percent = u8::try_from(target.saturating_mul(pct_step).checked_div(steps).unwrap_or(0)).unwrap_or(0);

        let (red, green, blue) = correct(base_red, base_green, base_blue, percent);
        driver.stage_all_leds(red, green, blue);
        stage_indicator_leds(driver, state, percent.min(INDICATOR_BRIGHTNESS));

        if let Err(err) = driver.flush().await {
            return Err(err);
        }
        if step < steps {
            Timer::after_millis(delay_ms).await;
        }
    }
    Ok(())
}

/// Apply brightness scaling and gamma correction to an RGB colour.
///
/// Each channel is scaled by `brightness_percent` via [`scale`], then passed
/// through the gamma-2.2 LUT, producing values ready for the LED driver.
#[inline]
const fn correct(red: u8, green: u8, blue: u8, brightness_percent: u8) -> (u8, u8, u8) {
    (
        gamma::correct(scale(red, brightness_percent)),
        gamma::correct(scale(green, brightness_percent)),
        gamma::correct(scale(blue, brightness_percent)),
    )
}

/// Apply [`correct`] to a pre-packaged `(R, G, B)` colour tuple.
///
/// Avoids destructuring at every call site when working with the named colour
/// constants defined in this module.
#[inline]
const fn correct_color(color: (u8, u8, u8), brightness_percent: u8) -> (u8, u8, u8) {
    correct(color.0, color.1, color.2, brightness_percent)
}

/// Paint every LED with `color` at `brightness` and flush to hardware.
///
/// Combines the three-step `correct` -> `stage_all_leds` -> `flush` sequence
/// that appears in every solid-color calibration frame, so each call site is a
/// single expression.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails. See [`BacklightDriver::flush`].
pub(super) async fn fill_all_leds(
    driver: &mut BacklightDriver,
    color: (u8, u8, u8),
    brightness: u8,
) -> Result<(), BusError> {
    let (red, green, blue) = correct_color(color, brightness);
    driver.stage_all_leds(red, green, blue);
    driver.flush().await
}

/// Writes every LED to hardware according to `state`.
///
/// Paints the background white at the current brightness, then overlays the
/// indicator LEDs on top. Called after any state change requiring a full
/// redraw (e.g. thermal-throttle transitions during normal operation).
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails. See [`BacklightDriver::flush`].
pub(super) async fn render_all(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
    let (red, green, blue) = correct_color(INDICATOR_WHITE, state.brightness);
    driver.stage_all_leds(red, green, blue);
    render_indicators(driver, state).await
}

/// Render the full-travel calibration frame.
///
/// The background gradient starts red at 0 % progress and shifts toward blue
/// at 100 % via `state.calib_pct`. Every LED whose bit is set in
/// `state.calib_leds_done` is overlaid with solid green regardless of the
/// background, giving immediate per-key visual confirmation as each key is
/// pressed to the bottom.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails. See [`BacklightDriver::flush`].
pub(super) async fn render_calib(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
    let bg_red = scale(255, 100_u8.saturating_sub(state.calib_pct));
    let bg_blue = scale(220, state.calib_pct);
    let (bg_r, bg_g, bg_b) = correct(bg_red, 0, bg_blue, state.brightness);
    let (cal_r, cal_g, cal_b) = correct_color(CALIB_GREEN, state.brightness);

    driver.stage_all_leds(bg_r, bg_g, bg_b);

    let mut bits = state.calib_leds_done;
    while bits != 0 {
        let idx = usize::try_from(bits.trailing_zeros()).unwrap_or(usize::MAX);
        driver.stage_led(idx, cal_r, cal_g, cal_b);
        bits &= bits.saturating_sub(1);
    }

    driver.flush().await
}

/// Writes only the two indicator LEDs according to `state`.
///
/// Used when only indicator state has changed and the background is already
/// correct, avoiding a full redraw.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails. See [`BacklightDriver::flush`].
pub(super) async fn render_indicators(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
    stage_indicator_leds(driver, state, INDICATOR_BRIGHTNESS);
    driver.flush().await
}

/// Scale `value` by `brightness_percent` (0-100), rounding to nearest.
///
/// Computes `value × percent / 100` with half-up rounding by adding 50 before
/// the integer division. The intermediate value fits in [`u16`] because
/// `255 × 100 + 50 = 25 550 < 65 535`; neither saturating path can trigger.
#[inline]
const fn scale(value: u8, brightness_percent: u8) -> u8 {
    let value_u16 = u16::from(value);
    let percent_clamped = u16::from(brightness_percent.min(100));
    // +50 rounds to nearest; /100 cannot overflow u8 because v*p/100 <= 255.
    u8::try_from(value_u16.saturating_mul(percent_clamped).saturating_add(50).checked_div(100).unwrap_or_default())
        .unwrap_or(255)
}

/// Stage both indicator LEDs with the given `brightness` without flushing.
///
/// Caps Lock: red when active, white when inactive.
/// Num Lock: white when active, off when inactive.
///
/// Separated from the flush so callers in the brightness-ramp loop can batch
/// per-step indicator staging with the background colour before a single flush.
const fn stage_indicator_leds(driver: &mut BacklightDriver, state: BacklightState, brightness: u8) {
    let caps_color = if state.flags.get(BacklightFlags::CAPS_LOCK) { INDICATOR_RED } else { INDICATOR_WHITE };
    let (caps_red, caps_green, caps_blue) = correct_color(caps_color, brightness);
    driver.stage_led(CAPS_LOCK_LED_INDEX, caps_red, caps_green, caps_blue);

    let num_color = if state.flags.get(BacklightFlags::NUM_LOCK) { INDICATOR_WHITE } else { INDICATOR_OFF };
    let (num_red, num_green, num_blue) = correct_color(num_color, brightness);
    driver.stage_led(NUM_LOCK_LED_INDEX, num_red, num_green, num_blue);
}
