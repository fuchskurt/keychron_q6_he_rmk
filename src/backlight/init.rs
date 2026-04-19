//! Backlight task initialization and runtime loop.

use crate::{
    backlight::{
        gamma_correction::gamma_correction,
        led_processor::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
    },
    layout::LED_LAYOUT,
};
use CalibPhase::{AllAccepted, Done, Full, Zero};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::spi::ErrorType;
use rmk::{
    embassy_futures::select::{Either3, select3},
    hid::Report,
    state::{ConnectionState, get_connection_state},
};
use snled27351_driver::{
    driver::Driver,
    transport::spi::{SpiTransport, SpiTransportError},
};

/// Concrete SPI transport type for this keyboard.
type Transport = SpiTransport<Spi<'static, Async, spi::mode::Master>, Output<'static>, Output<'static>, 2>;
/// Concrete driver type for this keyboard.
type BacklightDriver = Driver<Transport, 2>;
/// Bus error type shared by all async driver helpers.
type BusError = SpiTransportError<<Spi<'static, Async, spi::mode::Master> as ErrorType>::Error>;

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
/// Background colour during the zero-travel calibration phase and the
/// EEPROM-write-failure signal (amber).
const CALIB_AMBER: (u8, u8, u8) = (255, 120, 0);
/// Solid colour used for the post-calibration success hold and for each
/// individually confirmed key during the full-travel pass (green).
const CALIB_GREEN: (u8, u8, u8) = (0, 220, 80);
/// Number of brightness steps in the soft-start ramp.
const SOFTSTART_STEPS: u8 = 50;
/// Total duration of the soft-start ramp in milliseconds.
const SOFTSTART_RAMP_MS: u32 = 1000;
/// Brightness percentage applied when a driver chip reports temperature ≥ 70
/// °C.
const THERMAL_THROTTLE_BRIGHTNESS: u8 = 50;
/// Interval between thermal register polls.
const THERMAL_POLL: Duration = Duration::from_secs(5);
/// Interval between host connection state polls.
///
/// Short enough to detect an unplug within half a second without spinning
/// at full speed. The connection check is a single atomic load so the cost
/// per tick is negligible.
const CONNECTION_POLL: Duration = Duration::from_millis(500);
/// Duration in milliseconds to hold the solid-green display after calibration
/// is successfully stored, before returning to normal white backlight.
const CALIB_DONE_HOLD_MS: u64 = 2000;
/// Number of on/off cycles in the "all keys accepted" blink animation.
const CALIB_ALL_DONE_BLINK_COUNT: u8 = 3;
/// Duration of each on and each off half-period of the blink (milliseconds).
const CALIB_ALL_DONE_BLINK_HALF_MS: u64 = 150;
/// Delay between USB enumeration and the initial LED-state nudge report.
///
/// Gives the RMK HID reader task time to start listening on the output
/// endpoint before the empty report is sent. Without this delay the host
/// response may arrive before RMK is ready and be silently dropped,
/// leaving Num Lock in the wrong state.
const CONNECTION_NUDGE_DELAY: Duration = Duration::from_millis(50);

/// Scale `value` by `brightness_percent` (0–100), rounding to nearest.
///
/// Computes `value × percent / 100` with half-up rounding by adding 50 before
/// the integer division. The intermediate value fits in [`u16`] because
/// `255 × 100 + 50 = 25 550 < 65 535`; neither saturating path can trigger.
#[inline]
const fn scale(value: u8, brightness_percent: u8) -> u8 {
    let value_u16 = u16::from(value);
    let percent_clamped = u16::from(brightness_percent.min(100));
    // +50 rounds to nearest; /100 cannot overflow u8 because v*p/100 ≤ 255.
    u8::try_from(value_u16.saturating_mul(percent_clamped).saturating_add(50).checked_div(100).unwrap_or_default()).unwrap_or(255)
}

/// Apply brightness scaling and gamma correction to an RGB colour.
///
/// Each channel is scaled by `brightness_percent` via [`scale`], then passed
/// through the gamma-2.2 LUT, producing values ready for the LED driver.
#[inline]
const fn correct(red: u8, green: u8, blue: u8, brightness_percent: u8) -> (u8, u8, u8) {
    (
        gamma_correction(scale(red, brightness_percent)),
        gamma_correction(scale(green, brightness_percent)),
        gamma_correction(scale(blue, brightness_percent)),
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
/// Combines the three-step correct → stage_all_leds → flush sequence that
/// appears in every solid-colour calibration frame, so each call site is a
/// single expression.
async fn fill_all_leds(driver: &mut BacklightDriver, color: (u8, u8, u8), brightness: u8) -> Result<(), BusError> {
    let (r, g, b) = correct_color(color, brightness);
    driver.stage_all_leds(r, g, b);
    driver.flush().await
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
    /// Whether the USB host is currently connected and enumerated.
    ///
    /// Tracked across [`CONNECTION_POLL`] ticks to detect connect and
    /// disconnect transitions. On disconnect all LEDs are turned off to avoid
    /// driving the backlight unnecessarily. On reconnect the backlight is
    /// restored and an empty keyboard report is sent to prompt the host to
    /// resend its current LED indicator state, ensuring Num Lock and Caps Lock
    /// reflect the correct state from the first moment the keyboard is ready.
    host_connected: bool,
    /// Whether a first-boot calibration pass is currently in progress.
    ///
    /// `true` from [`Zero`] until [`Done`] completes.
    /// Used by the thermal throttle path to call [`render_calib`] instead of
    /// [`render_all`] so a thermal event never clobbers the calibration
    /// display.
    in_calib: bool,
    /// Bitset of LED indices confirmed calibrated during the full-travel pass.
    ///
    /// Bit `i` set means LED `i` should be painted solid green by
    /// [`render_calib`]. Cleared to zero when [`Done`] completes
    /// and the keyboard returns to normal white operation.
    calib_leds_done: u128,
    /// Whole-keyboard gradient percentage (0–100) during the full-travel pass.
    ///
    /// Drives the red→blue background interpolation in [`render_calib`].
    /// Cleared to zero alongside [`self::BacklightState::calib_leds_done`] on
    /// calibration completion.
    calib_pct: u8,
}

impl BacklightState {
    const fn new() -> Self {
        Self {
            brightness: 100,
            calib_leds_done: 0,
            calib_pct: 0,
            caps_lock: false,
            host_connected: false,
            in_calib: false,
            num_lock: false,
        }
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
async fn render_all(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
    let (r, g, b) = correct_color(INDICATOR_WHITE, state.brightness);
    driver.stage_all_leds(r, g, b);
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
async fn render_calib(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
    // Gradient background: starts red (0 %), shifts toward blue (100 %).
    let bg_red = scale(255, 100_u8.saturating_sub(state.calib_pct));
    let bg_blue = scale(220, state.calib_pct);
    let (bg_r, bg_g, bg_b) = correct(bg_red, 0, bg_blue, state.brightness);

    // Per-key confirmed color: solid green.
    let (cal_r, cal_g, cal_b) = correct_color(CALIB_GREEN, state.brightness);

    driver.stage_all_leds(bg_r, bg_g, bg_b);

    // Overlay individually confirmed LEDs in green.
    let mut bits = state.calib_leds_done;
    while bits != 0 {
        // Isolate and consume the lowest set bit.
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
async fn render_indicators(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
    // Caps Lock: red when active, white when inactive.
    let caps_color = if state.caps_lock { INDICATOR_RED } else { INDICATOR_WHITE };
    let (r, g, b) = correct_color(caps_color, INDICATOR_BRIGHTNESS);
    driver.stage_led(CAPS_LOCK_LED_INDEX, r, g, b);

    // Num Lock: white when active, off when inactive.
    let num_color = if state.num_lock { INDICATOR_WHITE } else { INDICATOR_OFF };
    let (r, g, b) = correct_color(num_color, INDICATOR_BRIGHTNESS);
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
) -> Result<(), BusError> {
    let target = u32::from(target_brightness.min(100));
    // SOFTSTART_STEPS is a non-zero constant; .max(1) is a belt-and-suspenders
    // guard ensuring checked_div always returns Some.
    let steps = u32::from(SOFTSTART_STEPS.max(1));
    let delay_ms = u64::from(SOFTSTART_RAMP_MS.checked_div(steps).unwrap_or(0));

    // `0..=steps` produces SOFTSTART_STEPS + 1 iterations so that both the
    // 0 % start and the 100 % end are written. The final delay is skipped
    // (step == steps guard) so the function returns immediately after the
    // last write without an unnecessary pause.
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
/// ramp, waits for USB enumeration, then enters an event loop that
/// concurrently handles three concerns:
///
/// - **Indicator and calibration commands** arriving on [`BACKLIGHT_CH`].
/// - **Thermal polling** on a [`THERMAL_POLL`] ticker: reads the TDF register
///   from both chips and reduces overall brightness to
///   [`THERMAL_THROTTLE_BRIGHTNESS`] if either exceeds 70 °C, restoring it
///   automatically when both cool down. During calibration [`render_calib`] is
///   used instead of [`render_all`] so the calibration display is never
///   clobbered by a thermal event.
/// - **Connection polling** on a [`CONNECTION_POLL`] ticker: monitors the USB
///   host connection state. On disconnect all LEDs are turned off. On reconnect
///   the backlight is restored and an empty keyboard report is sent to nudge
///   the host into reporting its current indicator state, ensuring Num Lock and
///   Caps Lock are correct without requiring a key press.
pub async fn backlight_runner(
    spi: Spi<'static, Async, spi::mode::Master>,
    cs0: Output<'static>,
    cs1: Output<'static>,
    sdb: Output<'static>,
) -> ! {
    let transport = SpiTransport::new(spi, [cs0, cs1], sdb);
    let mut driver = BacklightDriver::new(transport, LED_LAYOUT);

    // Backlight failures are non-critical: the keyboard remains fully functional
    // without LEDs. All driver errors are intentionally ignored throughout
    // this task via `let _ = ...`.
    if driver.init(0xFF).await.is_ok() {
        let _ = softstart(&mut driver, 255, 255, 255, 100).await;
    }

    // Wait for USB enumeration before entering the event loop. The connection
    // ticker will handle the initial nudge and all subsequent transitions, so
    // host_connected starts false and the first CONNECTION_POLL tick after
    // enumeration triggers the connect path naturally.
    while get_connection_state() == ConnectionState::Disconnected {
        Timer::after_millis(10).await;
    }

    let rx = BACKLIGHT_CH.receiver();
    let mut thermal_ticker = Ticker::every(THERMAL_POLL);
    let mut connection_ticker = Ticker::every(CONNECTION_POLL);
    let mut state = BacklightState::new();

    loop {
        match select3(rx.receive(), thermal_ticker.next(), connection_ticker.next()).await {
            Either3::First(cmd) => match cmd {
                BacklightCmd::CalibPhase(phase) => match phase {
                    Zero => {
                        // Amber: zero-travel pass, all keys should be fully released.
                        state.in_calib = true;
                        let _ = fill_all_leds(&mut driver, CALIB_AMBER, state.brightness).await;
                    }
                    Full => {
                        // Reset calib tracking and render the initial red frame via
                        // render_calib so the full-travel phase starts consistently.
                        state.calib_leds_done = 0;
                        state.calib_pct = 0;
                        let _ = render_calib(&mut driver, state).await;
                    }
                    AllAccepted => {
                        // Blink solid green × CALIB_ALL_DONE_BLINK_COUNT to signal
                        // that every key has been recorded and keys may be released.
                        // Calibration is still in progress (in_calib stays true).
                        for i in 0..CALIB_ALL_DONE_BLINK_COUNT {
                            let _ = fill_all_leds(&mut driver, CALIB_GREEN, state.brightness).await;
                            Timer::after_millis(CALIB_ALL_DONE_BLINK_HALF_MS).await;
                            if i < CALIB_ALL_DONE_BLINK_COUNT.saturating_sub(1) {
                                let _ = fill_all_leds(&mut driver, INDICATOR_OFF, state.brightness).await;
                                Timer::after_millis(CALIB_ALL_DONE_BLINK_HALF_MS).await;
                            }
                        }
                        // Leave the keyboard solid green heading into Done.
                    }
                    Done => {
                        // Solid green hold for 2 s to confirm calibration is stored.
                        let _ = fill_all_leds(&mut driver, CALIB_GREEN, state.brightness).await;
                        Timer::after_millis(CALIB_DONE_HOLD_MS).await;
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
                    // The `< 128` guard matches the bit-width of calib_leds_done
                    // (u128); checked_shl is an additional defensive layer.
                    if usize::from(led_idx) < 128 {
                        state.calib_leds_done |= 1_u128.checked_shl(u32::from(led_idx)).unwrap_or(0);
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
            Either3::Second(_) => {
                // Read the thermal flag from both driver chips. If either reports
                // a temperature at or above 70 °C, reduce brightness to
                // THERMAL_THROTTLE_BRIGHTNESS to protect the hardware. Brightness
                // is restored to 100 % automatically once both chips cool down.
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
            Either3::Third(_) => {
                // Poll the USB connection state. On disconnect all LEDs are turned
                // off to avoid driving the backlight unnecessarily. On reconnect
                // the backlight is restored and an empty keyboard report is sent to
                // nudge the host into resending its current LED indicator state.
                let connected = get_connection_state() == ConnectionState::Connected;
                if connected != state.host_connected {
                    state.host_connected = connected;
                    if connected {
                        // Wait for the HID reader task to be ready before sending
                        // the nudge, otherwise the host response may arrive before
                        // RMK is listening and be silently dropped.
                        Timer::after(CONNECTION_NUDGE_DELAY).await;
                        rmk::channel::KEYBOARD_REPORT_CHANNEL
                            .send(Report::KeyboardReport(rmk::descriptor::KeyboardReport::default()))
                            .await;
                        let _ = render_all(&mut driver, state).await;
                    } else {
                        // Host disconnected: turn off all LEDs until the next
                        // connect event restores the backlight.
                        let _ = fill_all_leds(&mut driver, INDICATOR_OFF, state.brightness).await;
                    }
                }
            }
        }
    }
}
