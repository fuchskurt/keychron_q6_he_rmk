//! Backlight task initialization and runtime loop.

use crate::{
    backlight::{
        gamma_correction::gamma_correction,
        lock_indicator::{BACKLIGHT_CH, BacklightCmd},
        mapping::LED_LAYOUT,
    },
    snled27351_spi::driver::Snled27351,
};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use embassy_time::{Duration, Ticker, Timer};
use rmk::embassy_futures::select::{Either, select};

/// LED index for the Caps Lock indicator key.
const CAPS_LOCK_LED_INDEX: usize = 62;
/// Number of SNLED27351 driver chips on this keyboard.
const DRIVER_COUNT: usize = 2;
/// LED index for the Num Lock indicator key.
const NUM_LOCK_LED_INDEX: usize = 37;
/// Brightness percentage applied to indicator LEDs (0–100).
const INDICATOR_BRIGHTNESS: u8 = 100;
/// RGB color for the active-lock indicator state (red).
const INDICATOR_RED: (u8, u8, u8) = (255, 0, 0);
/// RGB color for the inactive-lock indicator state (white).
const INDICATOR_WHITE: (u8, u8, u8) = (255, 255, 255);
/// RGB color for a disabled indicator (off).
const INDICATOR_OFF: (u8, u8, u8) = (0, 0, 0);

/// Number of brightness steps in the soft-start ramp.
const SOFTSTART_STEPS: u8 = 50;
/// Total duration of the soft-start ramp in milliseconds.
const SOFTSTART_RAMP_MS: u32 = 1000;
/// Brightness percentage applied when a driver chip reports temperature ≥ 70
/// °C.
const THERMAL_THROTTLE_BRIGHTNESS: u8 = 50;
/// Interval in milliseconds between thermal register polls.
const THERMAL_POLL: Duration = Duration::from_secs(5);

/// Scales a linear 0–255 value by a 0–100 brightness percentage.
///
/// Uses integer arithmetic with rounding to avoid floating point.
/// The result is clamped to `u8::MAX` on overflow.
#[inline]
const fn scale(value: u8, brightness_percent: u8) -> u8 {
    let v = u16::from(value);
    let p = u16::from(brightness_percent.min(100));
    u8::try_from(v.saturating_mul(p).saturating_add(50).checked_div(100).unwrap_or_default()).unwrap_or(255)
}

/// Applies brightness scaling and gamma 2.2 correction to an RGB triplet.
#[inline]
const fn correct(red: u8, green: u8, blue: u8, brightness_percent: u8) -> (u8, u8, u8) {
    (
        gamma_correction(scale(red, brightness_percent)),
        gamma_correction(scale(green, brightness_percent)),
        gamma_correction(scale(blue, brightness_percent)),
    )
}

/// Full backlight state, kept in one place so any path can do a consistent
/// redraw without losing information.
#[derive(Clone, Copy)]
struct BacklightState {
    /// Whether Caps Lock is currently active.
    caps_lock: bool,
    /// Whether Num Lock is currently active.
    num_lock: bool,
    /// Global brightness percentage (0–100); reduced under thermal throttle.
    brightness: u8,
}

impl BacklightState {
    const fn new() -> Self { Self { caps_lock: false, num_lock: false, brightness: 100 } }
}

/// Writes every LED to the hardware according to `state`.
///
/// Paints the background white at the current brightness, then overlays the
/// indicator LEDs on top. Called after any state change that requires a full
/// redraw (e.g. thermal-throttle transitions).
async fn render_all(backlight: &mut Snled27351<'static, DRIVER_COUNT>, state: BacklightState) {
    let (r, g, b) = correct(255, 255, 255, state.brightness);
    backlight.set_all_leds(r, g, b).await;
    render_indicators(backlight, state).await;
}

/// Writes only the two indicator LEDs according to `state`.
///
/// Used when only indicator state has changed and the background is
/// already correct, avoiding a full redraw.
async fn render_indicators(backlight: &mut Snled27351<'static, DRIVER_COUNT>, state: BacklightState) {
    // Caps Lock: red when active, white when inactive.
    let caps_color = if state.caps_lock { INDICATOR_RED } else { INDICATOR_WHITE };
    let (r, g, b) = correct(caps_color.0, caps_color.1, caps_color.2, INDICATOR_BRIGHTNESS);
    backlight.set_led(CAPS_LOCK_LED_INDEX, r, g, b).await;

    // Num Lock: white when active, off when inactive.
    let num_color = if state.num_lock { INDICATOR_WHITE } else { INDICATOR_OFF };
    let (r, g, b) = correct(num_color.0, num_color.1, num_color.2, INDICATOR_BRIGHTNESS);
    backlight.set_led(NUM_LOCK_LED_INDEX, r, g, b).await;
}

/// Performs a soft-start brightness ramp on all LEDs at initialization.
///
/// Linearly steps brightness from 0 up to `target_brightness` over
/// [`SOFTSTART_STEPS`] increments across [`SOFTSTART_RAMP_MS`] milliseconds.
async fn softstart(
    backlight: &mut Snled27351<'static, DRIVER_COUNT>,
    base_red: u8,
    base_green: u8,
    base_blue: u8,
    target_brightness: u8,
) {
    let target = u32::from(target_brightness.min(100));
    let steps = u32::from(SOFTSTART_STEPS.max(1));
    let delay_ms = u64::from(SOFTSTART_RAMP_MS.checked_div(steps).unwrap_or(0));

    for step in 0..=steps {
        let percent = u8::try_from(target.saturating_mul(step).checked_div(steps).unwrap_or(0)).unwrap_or(0);
        let (r, g, b) = correct(base_red, base_green, base_blue, percent);
        backlight.set_all_leds(r, g, b).await;
        if step < steps {
            Timer::after_millis(delay_ms).await;
        }
    }
}

/// Runs the backlight controller loop.
///
/// Initializes both SNLED27351 driver chips, performs a soft-start brightness
/// ramp, then enters an event loop that concurrently handles two concerns:
///
/// - **Indicator commands** arriving on [`BACKLIGHT_CH`]: updates Caps Lock and
///   Num Lock LEDs without touching the rest of the matrix.
/// - **Thermal polling** on a [`THERMAL_POLL`] ticker: reads the TDF register
///   from both chips and reduces overall brightness to
///   [`THERMAL_THROTTLE_BRIGHTNESS`] if either exceeds 70 °C, restoring it
///   automatically when both cool down. A full redraw is issued on each
///   transition so that indicator LEDs are never lost.
pub async fn backlight_runner(
    spi: Spi<'static, Async, spi::mode::Master>,
    cs0: Output<'static>,
    cs1: Output<'static>,
    sdb: Output<'static>,
) -> ! {
    let cs = [cs0, cs1];
    let mut backlight = Snled27351::new(spi, cs, sdb, LED_LAYOUT);

    backlight.init(0xFF).await;
    softstart(&mut backlight, 255, 255, 255, 100).await;

    let rx = BACKLIGHT_CH.receiver();
    let mut thermal_ticker = Ticker::every(Duration::from(THERMAL_POLL));
    let mut state = BacklightState::new();

    loop {
        match select(rx.receive(), thermal_ticker.next()).await {
            Either::First(cmd) => match cmd {
                BacklightCmd::Indicators { caps, num } => {
                    state.caps_lock = caps;
                    state.num_lock = num;
                    // Background is already correct; only repaint indicator LEDs.
                    render_indicators(&mut backlight, state).await;
                }
            },

            Either::Second(_) => {
                let hot = backlight.check_thermal_flag_set(0).await || backlight.check_thermal_flag_set(1).await;
                let new_brightness = if hot { THERMAL_THROTTLE_BRIGHTNESS } else { 100 };

                if new_brightness != state.brightness {
                    state.brightness = new_brightness;
                    // Full redraw required; restores indicator LEDs over the new background.
                    render_all(&mut backlight, state).await;
                }
            }
        }
    }
}
