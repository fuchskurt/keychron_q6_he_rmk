//! Backlight task initialization and runtime loop.

use crate::{
    backlight::processor::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
    layout::LED_LAYOUT,
};
use CalibPhase::{AllAccepted, Done, Full, Zero};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    pac::USB_OTG_FS,
    spi::{Spi, mode::Master},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::spi::ErrorType;
use q6_core::gamma;
use rmk::{
    core_traits::Runnable,
    embassy_futures::select::{Either3, select3},
};
use snled27351_driver::{
    driver::Driver,
    transport::spi::{Controller, TransportError},
};
use static_cell::StaticCell;

/// Shared SPI bus protected by an async-aware mutex.
static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Master>>> = StaticCell::new();

/// LED index for the Caps Lock indicator key.
const CAPS_LOCK_LED_INDEX: usize = 62;
/// LED index for the Num Lock indicator key.
const NUM_LOCK_LED_INDEX: usize = 37;
/// Default global brightness percentage when no throttle is active.
const FULL_BRIGHTNESS: u8 = 100;
/// Brightness percentage applied to indicator LEDs (0-100).
const INDICATOR_BRIGHTNESS: u8 = 100;
/// RGB color for the active-lock indicator state (red).
const INDICATOR_RED: (u8, u8, u8) = (255, 0, 0);
/// RGB color for the inactive-lock indicator state (white).
const INDICATOR_WHITE: (u8, u8, u8) = (255, 255, 255);
/// RGB color for a disabled indicator (off).
const INDICATOR_OFF: (u8, u8, u8) = (0, 0, 0);
/// Background color during the zero-travel calibration phase and the
/// EEPROM-write-failure signal (amber).
const CALIB_AMBER: (u8, u8, u8) = (255, 120, 0);
/// Solid color used for the post-calibration success hold and for each
/// individually confirmed key during the full-travel pass (green).
const CALIB_GREEN: (u8, u8, u8) = (0, 220, 80);
/// Number of brightness steps in the soft-start ramp.
const SOFTSTART_STEPS: u8 = 50;
/// Total duration of the soft-start ramp in milliseconds.
const SOFTSTART_RAMP_MS: u32 = 1000;
/// Brightness percentage applied when a driver chip reports temperature >= 70
/// °C.
const THERMAL_THROTTLE_BRIGHTNESS: u8 = 50;
/// Interval between thermal register polls.
const THERMAL_POLL: Duration = Duration::from_secs(5);
/// Interval between host connection state polls.
///
/// Short enough to detect an unplug within half a second without spinning
/// at full speed. The connection check is a single register read so the cost
/// per tick is negligible.
const CONNECTION_POLL: Duration = Duration::from_millis(500);
/// Duration in milliseconds to hold the solid-green display after calibration
/// is successfully stored, before returning to normal white backlight.
const CALIB_DONE_HOLD_MS: u64 = 2000;
/// Number of on/off cycles in the "all keys accepted" blink animation.
const CALIB_ALL_DONE_BLINK_COUNT: u8 = 3;
/// Duration of each on and each off half-period of the blink (milliseconds).
const CALIB_ALL_DONE_BLINK_HALF_MS: u64 = 150;

/// Concrete SPI device type wrapping one chip-select pin.
type Device = SpiDevice<'static, ThreadModeRawMutex, Spi<'static, Async, Master>, Output<'static>>;
/// Concrete SPI transport type for this keyboard.
type Transport = Controller<Device, Output<'static>, 2>;
/// Concrete driver type for this keyboard.
type BacklightDriver = Driver<Transport, 2>;
/// Bus error type shared by all async driver helpers.
type BusError = TransportError<<Device as ErrorType>::Error>;

/// Packed boolean state for [`BacklightState`], one bit per flag.
///
/// Holds the lock-indicator, host-connection, and calibration-in-progress
/// flags in a single byte so [`BacklightState`] stays within the bool-field
/// budget while keeping named, type-checked accessors.
#[derive(Clone, Copy, Default)]
struct BacklightFlags {
    /// One bit per flag; see the `*_FLAG` masks.
    bits: u8,
}

impl BacklightFlags {
    /// Caps Lock is currently active.
    const CAPS_LOCK: u8 = 0b0001;
    /// The USB host is currently connected and enumerated.
    const HOST_CONNECTED: u8 = 0b0010;
    /// A first-boot calibration pass is currently in progress.
    const IN_CALIB: u8 = 0b0100;
    /// Num Lock is currently active.
    const NUM_LOCK: u8 = 0b1000;

    /// Read the flag selected by `mask`.
    const fn get(self, mask: u8) -> bool { self.bits & mask != 0 }

    /// Set or clear the flag selected by `mask`.
    const fn set(&mut self, mask: u8, on: bool) {
        if on {
            self.bits |= mask;
        } else {
            self.bits &= !mask;
        }
    }
}

/// Full backlight state, kept in one place so any path can do a consistent
/// redraw without losing information.
#[derive(Clone, Copy, Default)]
struct BacklightState {
    /// Global brightness percentage (0-100); reduced under thermal throttle.
    brightness:      u8 = FULL_BRIGHTNESS,
    /// Bitset of LED indices confirmed calibrated during the full-travel pass.
    ///
    /// Bit `i` set means LED `i` should be painted solid green by
    /// [`render_calib`]. Cleared to zero when [`Done`] completes
    /// and the keyboard returns to normal white operation.
    calib_leds_done: u128,
    /// Whole-keyboard gradient percentage (0-100) during the full-travel pass.
    ///
    /// Drives the red-blue background interpolation in [`render_calib`].
    /// Cleared to zero alongside [`BacklightState::calib_leds_done`] on
    /// calibration completion.
    calib_pct:       u8,
    /// Packed lock / host-connection / calibration-in-progress flags.
    ///
    /// `host_connected` is tracked across [`CONNECTION_POLL`] ticks to detect
    /// connect and disconnect transitions. On disconnect all LEDs are turned
    /// off; on reconnect the backlight is restored. `in_calib` is `true` from
    /// [`Zero`] until [`Done`] completes and is used by the thermal-throttle
    /// path to call [`render_calib`] instead of [`render_all`] so a thermal
    /// event never clobbers the calibration display.
    flags:           BacklightFlags,
}

/// Backlight controller owning its SPI bus and LED driver chips.
///
/// Construct with [`BacklightRunner::new`] and hand to
/// `run_all!` alongside the other keyboard runnables. All
/// LED control goes through [`Runnable::run`].
pub struct BacklightRunner {
    /// LED driver chips behind the shared SPI bus.
    driver: BacklightDriver,
}

impl BacklightRunner {
    /// Apply a single first-boot calibration phase transition to the
    /// backlight: paint the amber/red-blue/blinking-green/solid-green frames
    /// described in [`CalibPhase`] and update the calibration tracking fields
    /// so subsequent renders behave consistently.
    async fn handle_calib_phase(&mut self, state: &mut BacklightState, phase: CalibPhase) {
        match phase {
            Zero => {
                // Amber: zero-travel pass, all keys should be fully released.
                state.flags.set(BacklightFlags::IN_CALIB, true);
                _ = fill_all_leds(&mut self.driver, CALIB_AMBER, state.brightness).await;
            },
            Full => {
                // Reset calib tracking and render the initial red frame via
                // render_calib so the full-travel phase starts consistently.
                state.calib_leds_done = 0;
                state.calib_pct = 0;
                _ = render_calib(&mut self.driver, *state).await;
            },
            AllAccepted => {
                // Blink solid green × CALIB_ALL_DONE_BLINK_COUNT to signal
                // that every key has been recorded and keys may be released.
                // Calibration is still in progress (in_calib stays true).
                for blink_index in 0..CALIB_ALL_DONE_BLINK_COUNT {
                    _ = fill_all_leds(&mut self.driver, CALIB_GREEN, state.brightness).await;
                    Timer::after_millis(CALIB_ALL_DONE_BLINK_HALF_MS).await;
                    if blink_index < CALIB_ALL_DONE_BLINK_COUNT.saturating_sub(1) {
                        _ = fill_all_leds(&mut self.driver, INDICATOR_OFF, state.brightness).await;
                        Timer::after_millis(CALIB_ALL_DONE_BLINK_HALF_MS).await;
                    }
                }
                // Leave the keyboard solid green heading into Done.
            },
            Done => {
                // Solid green hold for 2 s to confirm calibration is stored.
                _ = fill_all_leds(&mut self.driver, CALIB_GREEN, state.brightness).await;
                Timer::after_millis(CALIB_DONE_HOLD_MS).await;
                // Clear all calibration state before returning to normal white
                // so stale bits cannot bleed into the next render.
                state.flags.set(BacklightFlags::IN_CALIB, false);
                state.calib_leds_done = 0;
                state.calib_pct = 0;
                _ = render_all(&mut self.driver, *state).await;
            },
        }
    }

    /// Apply a single [`BacklightCmd`] arriving on [`BACKLIGHT_CH`].
    async fn handle_cmd(&mut self, state: &mut BacklightState, cmd: BacklightCmd) {
        match cmd {
            BacklightCmd::CalibPhase(phase) => self.handle_calib_phase(state, phase).await,
            BacklightCmd::CalibProgress(pct) => {
                state.calib_pct = pct;
                _ = render_calib(&mut self.driver, *state).await;
            },
            BacklightCmd::CalibKeyDone(led_idx) => {
                // Mark this LED calibrated and repaint immediately so the key
                // turns green the moment it crosses the threshold. checked_shl
                // returns None for any shift >= 128 (the bit-width of
                // calib_leds_done).
                if let Some(bit) = 1_u128.checked_shl(u32::from(led_idx)) {
                    state.calib_leds_done |= bit;
                }
                _ = render_calib(&mut self.driver, *state).await;
            },
            BacklightCmd::Indicators { caps, num } => {
                state.flags.set(BacklightFlags::CAPS_LOCK, caps);
                state.flags.set(BacklightFlags::NUM_LOCK, num);
                // Background is already correct; only repaint indicator LEDs.
                _ = render_indicators(&mut self.driver, *state).await;
            },
        }
    }

    /// Poll the USB host connection state via the OTG FS hardware registers
    /// and react to transitions.
    ///
    /// On disconnect the backlight is ramped down and the driver chips are
    /// shut down; on reconnect the chips are woken back up and the backlight
    /// is ramped to [`FULL_BRIGHTNESS`]. No-op while the connection state is
    /// unchanged.
    async fn handle_connection_tick(&mut self, state: &mut BacklightState) {
        let connected = usb_is_active();
        if connected == state.flags.get(BacklightFlags::HOST_CONNECTED) {
            return;
        }
        state.flags.set(BacklightFlags::HOST_CONNECTED, connected);
        if connected {
            _ = self.driver.wake().await;
            _ = brightness_ramp(&mut self.driver, INDICATOR_WHITE, FULL_BRIGHTNESS, true, *state).await;
        } else {
            // Host disconnected: turn off all LEDs until the next connect
            // event restores the backlight.
            _ = brightness_ramp(&mut self.driver, INDICATOR_WHITE, state.brightness, false, *state).await;
            _ = self.driver.shutdown().await;
        }
    }

    /// Poll both driver chips' thermal flags and adjust global brightness.
    ///
    /// If either chip reports a temperature at or above 70 °C, brightness
    /// drops to [`THERMAL_THROTTLE_BRIGHTNESS`]; once both chips cool down,
    /// brightness is restored to [`FULL_BRIGHTNESS`]. Re-renders via
    /// [`render_calib`] during a calibration pass so the gradient and per-key
    /// green state survive the brightness change, otherwise via
    /// [`render_all`].
    async fn handle_thermal_tick(&mut self, state: &mut BacklightState) {
        let hot = self.driver.check_thermal_flag_set(0).await || self.driver.check_thermal_flag_set(1).await;
        let new_brightness = if hot { THERMAL_THROTTLE_BRIGHTNESS } else { FULL_BRIGHTNESS };
        if new_brightness == state.brightness {
            return;
        }
        state.brightness = new_brightness;
        if state.flags.get(BacklightFlags::IN_CALIB) {
            _ = render_calib(&mut self.driver, *state).await;
        } else {
            _ = render_all(&mut self.driver, *state).await;
        }
    }

    /// Construct and take ownership of the SPI bus and all chip-select pins.
    ///
    /// Initialises the shared bus, wraps each CS in its own `SpiDevice`, and
    /// builds the `BacklightDriver`. All further interaction is through
    /// [`Runnable::run`].
    pub fn new(
        spi: Spi<'static, Async, Master>,
        cs0: Output<'static>,
        cs1: Output<'static>,
        sdb: Output<'static>,
    ) -> Self {
        let spi_bus = SPI_BUS.init(Mutex::new(spi));
        let device0 = SpiDevice::new(spi_bus, cs0);
        let device1 = SpiDevice::new(spi_bus, cs1);
        let transport = Controller::new([device0, device1], sdb);
        let driver = BacklightDriver::new(transport, LED_LAYOUT);
        Self { driver }
    }
}

impl Runnable for BacklightRunner {
    /// Initialises both SNLED27351 driver chips, waits for USB enumeration,
    /// then enters an event loop that concurrently dispatches three
    /// concerns to dedicated handlers:
    ///
    /// - **Indicator and calibration commands** arriving on [`BACKLIGHT_CH`]
    ///   are routed to [`BacklightRunner::handle_cmd`].
    /// - **Thermal polling** on a [`THERMAL_POLL`] ticker is handled by
    ///   [`BacklightRunner::handle_thermal_tick`].
    /// - **Connection polling** on a [`CONNECTION_POLL`] ticker is handled by
    ///   [`BacklightRunner::handle_connection_tick`].
    ///
    /// Backlight failures are non-critical: the keyboard remains fully
    /// functional without LEDs, so each handler ignores driver errors via
    /// `_ = …`.
    async fn run(&mut self) -> ! {
        _ = self.driver.init(0xFF).await;

        // Wait for USB enumeration before entering the event loop. The
        // connection ticker handles all subsequent transitions, so
        // host_connected starts false and the first CONNECTION_POLL tick
        // after enumeration triggers the connect path naturally.
        while !usb_is_active() {
            Timer::after_millis(10).await;
        }

        let rx = BACKLIGHT_CH.receiver();
        let mut thermal_ticker = Ticker::every(THERMAL_POLL);
        let mut connection_ticker = Ticker::every(CONNECTION_POLL);
        let mut state = BacklightState::default();

        loop {
            match select3(rx.receive(), thermal_ticker.next(), connection_ticker.next()).await {
                Either3::First(cmd) => self.handle_cmd(&mut state, cmd).await,
                Either3::Second(()) => self.handle_thermal_tick(&mut state).await,
                Either3::Third(()) => self.handle_connection_tick(&mut state).await,
            }
        }
    }
}

/// Performs a brightness ramp on all LEDs.
///
/// Linearly steps brightness from 0 up to `target_brightness` over
/// [`SOFTSTART_STEPS`] increments across [`SOFTSTART_RAMP_MS`] milliseconds.
///
/// # Errors
///
/// Returns `Err` if any bus transaction fails during the ramp.
async fn brightness_ramp(
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
async fn fill_all_leds(driver: &mut BacklightDriver, color: (u8, u8, u8), brightness: u8) -> Result<(), BusError> {
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
async fn render_all(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
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
async fn render_calib(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
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
async fn render_indicators(driver: &mut BacklightDriver, state: BacklightState) -> Result<(), BusError> {
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

/// Returns `true` when the USB host is present and the bus is not suspended.
#[inline]
fn usb_is_active() -> bool { usb_vbus_present() && !usb_is_suspended() }

/// Returns `true` when the OTG FS hardware reports the USB bus is suspended.
///
/// Reads `OTG_FS.DSTS.SUSPSTS` directly so suspend is detected regardless of
/// whether the embassy-usb handler fires.
#[inline]
fn usb_is_suspended() -> bool { USB_OTG_FS.dsts().read().suspsts() }

/// Returns `true` when VBUS is present (cable plugged in, host powered).
///
/// Reads `OTG_FS.GOTGCTL.BSVLD` directly.
#[inline]
fn usb_vbus_present() -> bool { USB_OTG_FS.gotgctl().read().bsvld() }
