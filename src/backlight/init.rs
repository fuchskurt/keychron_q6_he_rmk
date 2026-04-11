//! Backlight task initialization and runtime loop.

use crate::backlight::{
    gamma_correction::gamma_correction,
    lock_indicator::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
    mapping::LED_LAYOUT,
};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::spi::ErrorType;
use rmk::embassy_futures::select::{Either, select};
use snled27351_driver::{
    driver::Driver,
    transport::spi::{SpiTransport, SpiTransportError},
};

/// Concrete SPI transport type for this keyboard.
type Transport = SpiTransport<Spi<'static, Async, spi::mode::Master>, Output<'static>, Output<'static>, 2>;
/// Concrete driver type for this keyboard.
type BacklightDriver = Driver<Transport, 2>;

/// LED index for the Caps Lock indicator key.
const CAPS_LOCK_LED_INDEX: usize = 62;
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
/// Interval between thermal register polls.
const THERMAL_POLL: Duration = Duration::from_secs(5);

#[inline]
const fn scale(value: u8, brightness_percent: u8) -> u8 {
    let v = u16::from(value);
    let p = u16::from(brightness_percent.min(100));
    u8::try_from(v.saturating_mul(p).saturating_add(50).checked_div(100).unwrap_or_default()).unwrap_or(255)
}

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
    /// Whether a first-boot calibration pass is currently in progress.
    ///
    /// `true` from [`CalibPhase::Zero`] until [`CalibPhase::Done`] completes.
    /// Used by the thermal throttle path to call [`render_calib`] instead of
    /// [`render_all`] so a thermal event never clobbers the calibration
    /// display.
    in_calib: bool,
    /// Bitset of LED indices confirmed calibrated during the full-travel pass.
    ///
    /// Bit `i` set means LED `i` should be painted solid green by
    /// [`render_calib`]. Cleared to zero when [`CalibPhase::Done`] completes
    /// and the keyboard returns to normal white operation.
    calib_leds_done: u128,
    /// Whole-keyboard gradient percentage (0–100) during the full-travel pass.
    ///
    /// Drives the blue→green background interpolation in [`render_calib`].
    /// Cleared to zero alongside [`calib_leds_done`] on calibration completion.
    calib_pct: u8,
}

impl BacklightState {
    const fn new() -> Self {
        Self { caps_lock: false, num_lock: false, brightness: 100, in_calib: false, calib_leds_done: 0, calib_pct: 0 }
    }
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
async fn render_all(
    driver: &mut BacklightDriver,
    state: BacklightState,
) -> Result<(), SpiTransportError<<Spi<'static, Async, spi::mode::Master> as ErrorType>::Error>> {
    let (r, g, b) = correct(255, 255, 255, state.brightness);
    driver.stage_all_leds(r, g, b);
    render_indicators(driver, state).await
}

/// Render the full-travel calibration frame.
///
/// The background interpolates from pure blue (0 %) toward pure green (100 %)
/// via `state.calib_pct`. Every LED whose bit is set in `state.calib_leds_done`
/// is overlaid with solid green regardless of the background, giving immediate
/// per-key visual confirmation as each key is pressed to the bottom.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails. See [`BacklightDriver::flush`].
async fn render_calib(
    driver: &mut BacklightDriver,
    state: BacklightState,
) -> Result<(), SpiTransportError<<Spi<'static, Async, spi::mode::Master> as ErrorType>::Error>> {
    // Gradient background: blue fades out, green fades in.
    let bg_green = scale(220, state.calib_pct);
    let bg_blue = scale(255, 100_u8.saturating_sub(state.calib_pct));
    let (bg_r, bg_g, bg_b) = correct(0, bg_green, bg_blue, state.brightness);

    // Per-key confirmed color: solid green.
    let (cal_r, cal_g, cal_b) = correct(0, 220, 80, state.brightness);

    driver.stage_all_leds(bg_r, bg_g, bg_b);

    // Overlay individually confirmed LEDs in green.
    let mut bits = state.calib_leds_done;
    while bits != 0 {
        // Isolate and consume the lowest set bit.
        let idx = bits.trailing_zeros() as usize;
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
async fn render_indicators(
    driver: &mut BacklightDriver,
    state: BacklightState,
) -> Result<(), SpiTransportError<<Spi<'static, Async, spi::mode::Master> as ErrorType>::Error>> {
    // Caps Lock: red when active, white when inactive.
    let caps_color = if state.caps_lock { INDICATOR_RED } else { INDICATOR_WHITE };
    let (r, g, b) = correct(caps_color.0, caps_color.1, caps_color.2, INDICATOR_BRIGHTNESS);
    driver.stage_led(CAPS_LOCK_LED_INDEX, r, g, b);

    // Num Lock: white when active, off when inactive.
    let num_color = if state.num_lock { INDICATOR_WHITE } else { INDICATOR_OFF };
    let (r, g, b) = correct(num_color.0, num_color.1, num_color.2, INDICATOR_BRIGHTNESS);
    driver.stage_led(NUM_LOCK_LED_INDEX, r, g, b);

    driver.flush().await
}

/// Performs a soft-start brightness ramp on all LEDs at initialization.
///
/// Linearly steps brightness from 0 up to `target_brightness` over
/// [`SOFTSTART_STEPS`] increments across [`SOFTSTART_RAMP_MS`] milliseconds.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails during the ramp.
async fn softstart(
    driver: &mut BacklightDriver,
    base_red: u8,
    base_green: u8,
    base_blue: u8,
    target_brightness: u8,
) -> Result<(), SpiTransportError<<Spi<'static, Async, spi::mode::Master> as ErrorType>::Error>> {
    let target = u32::from(target_brightness.min(100));
    let steps = u32::from(SOFTSTART_STEPS.max(1));
    let delay_ms = u64::from(SOFTSTART_RAMP_MS.checked_div(steps).unwrap_or(0));

    for step in 0..=steps {
        let percent = u8::try_from(target.saturating_mul(step).checked_div(steps).unwrap_or(0)).unwrap_or(0);
        let (r, g, b) = correct(base_red, base_green, base_blue, percent);
        driver.set_all_leds(r, g, b).await?;
        if step < steps {
            Timer::after_millis(delay_ms).await;
        }
    }
    Ok(())
}

/// Runs the backlight controller loop.
///
/// Initializes both SNLED27351 driver chips, performs a soft-start brightness
/// ramp, then enters an event loop that concurrently handles two concerns:
///
/// - **Indicator and calibration commands** arriving on [`BACKLIGHT_CH`].
/// - **Thermal polling** on a [`THERMAL_POLL`] ticker: reads the TDF register
///   from both chips and reduces overall brightness to
///   [`THERMAL_THROTTLE_BRIGHTNESS`] if either exceeds 70 °C, restoring it
///   automatically when both cool down. During calibration [`render_calib`] is
///   used instead of [`render_all`] so the calibration display is never
///   clobbered by a thermal event.
pub async fn backlight_runner(
    spi: Spi<'static, Async, spi::mode::Master>,
    cs0: Output<'static>,
    cs1: Output<'static>,
    sdb: Output<'static>,
) -> ! {
    let transport = SpiTransport::new(spi, [cs0, cs1], sdb);
    let mut driver = BacklightDriver::new(transport, LED_LAYOUT);

    if driver.init(0xFF).await.is_ok() {
        let _ = softstart(&mut driver, 255, 255, 255, 100).await;
    }

    let rx = BACKLIGHT_CH.receiver();
    let mut thermal_ticker = Ticker::every(THERMAL_POLL);
    let mut state = BacklightState::new();

    loop {
        match select(rx.receive(), thermal_ticker.next()).await {
            Either::First(cmd) => match cmd {
                BacklightCmd::CalibPhase(phase) => match phase {
                    CalibPhase::Zero => {
                        // Amber: zero-travel pass, all keys should be fully released.
                        state.in_calib = true;
                        let (r, g, b) = correct(255, 120, 0, state.brightness);
                        driver.stage_all_leds(r, g, b);
                        let _ = driver.flush().await;
                    }
                    CalibPhase::Full => {
                        // Reset calib tracking and render the initial blue frame via
                        // render_calib so the full-travel phase starts consistently.
                        state.calib_leds_done = 0;
                        state.calib_pct = 0;
                        let _ = render_calib(&mut driver, state).await;
                    }
                    CalibPhase::Done => {
                        // Solid green hold for 2 s to confirm calibration is stored.
                        let (r, g, b) = correct(0, 220, 80, state.brightness);
                        driver.stage_all_leds(r, g, b);
                        let _ = driver.flush().await;
                        Timer::after_millis(2000).await;
                        // Clear all calibration state before returning to normal
                        // white so stale bits cannot bleed into the next render.
                        state.in_calib = false;
                        state.calib_leds_done = 0;
                        state.calib_pct = 0;
                        let _ = render_all(&mut driver, state).await;
                    }
                },
                BacklightCmd::CalibProgress(pct) => {
                    state.calib_pct = pct;
                    let _ = render_calib(&mut driver, state).await;
                }
                BacklightCmd::CalibKeyDone(led_idx) => {
                    // Mark this LED calibrated and repaint immediately so the
                    // key turns green the moment it crosses the threshold.
                    if usize::from(led_idx) < 128 {
                        state.calib_leds_done |= 1_u128 << led_idx;
                    }
                    let _ = render_calib(&mut driver, state).await;
                }
                BacklightCmd::Indicators { caps, num } => {
                    state.caps_lock = caps;
                    state.num_lock = num;
                    // Background is already correct; only repaint indicator LEDs.
                    let _ = render_indicators(&mut driver, state).await;
                }
            },
            Either::Second(_) => {
                let hot = driver.check_thermal_flag_set(0).await || driver.check_thermal_flag_set(1).await;
                let new_brightness = if hot { THERMAL_THROTTLE_BRIGHTNESS } else { 100 };
                if new_brightness != state.brightness {
                    state.brightness = new_brightness;
                    // During calibration use render_calib so the gradient and
                    // per-key green state are preserved after the brightness change.
                    // Outside calibration restore the normal white background.
                    if state.in_calib {
                        let _ = render_calib(&mut driver, state).await;
                    } else {
                        let _ = render_all(&mut driver, state).await;
                    }
                }
            }
        }
    }
}
