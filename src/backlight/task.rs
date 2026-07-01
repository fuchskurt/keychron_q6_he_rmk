//! Backlight task: owns the SPI bus and LED driver chips, drains the
//! command channel, and decides *when* to repaint.
//!
//! The display state model and the frame painting functions live in
//! `render`; this module reacts to indicator commands, guided-calibration
//! phase transitions, thermal polling, and USB power edges, and calls into
//! `render` with a consistent [`BacklightState`].

use crate::{
    backlight::{
        processor::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
        render::{
            BacklightDriver,
            BacklightFlags,
            BacklightState,
            BusError,
            CALIB_AMBER,
            CALIB_GREEN,
            CalibDisplay,
            FULL_BRIGHTNESS,
            INDICATOR_OFF,
            INDICATOR_WHITE,
            brightness_ramp,
            fill_all_leds,
            render_all,
            render_calib,
            render_indicators,
        },
    },
    layout::LED_LAYOUT,
};
use CalibPhase::{AllAccepted, Done, Full, Zero};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{Spi, mode::Master},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use rmk::{
    core_traits::Runnable,
    embassy_futures::select::{Either, select},
};
use snled27351_driver::transport::spi::Controller;
use static_cell::StaticCell;

/// Shared SPI bus protected by an async-aware mutex.
static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Master>>> = StaticCell::new();

/// Brightness percentage applied when a driver chip reports temperature >= 70
/// °C.
const THERMAL_THROTTLE_BRIGHTNESS: u8 = 50;
/// Interval between thermal register polls.
const THERMAL_POLL: Duration = Duration::from_secs(5);
/// Duration in milliseconds to hold the solid-green display after calibration
/// is successfully stored, before returning to normal white backlight.
const CALIB_DONE_HOLD_MS: u64 = 2000;
/// Number of on/off cycles in the "all keys accepted" blink animation.
const CALIB_ALL_DONE_BLINK_COUNT: u8 = 3;
/// Duration of each on and each off half-period of the blink (milliseconds).
const CALIB_ALL_DONE_BLINK_HALF_MS: u64 = 150;

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
                state.calib_display = CalibDisplay::Zero;
                _ = fill_all_leds(&mut self.driver, CALIB_AMBER, state.brightness).await;
            },
            Full => {
                // Reset calib tracking and render the initial red frame via
                // render_calib so the full-travel phase starts consistently.
                state.calib_display = CalibDisplay::Full;
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
                state.calib_display = CalibDisplay::None;
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
            BacklightCmd::Power(on) => self.handle_power(state, on).await,
        }
    }

    /// React to a USB host connection transition.
    ///
    /// On disconnect the backlight is ramped down and the driver chips are
    /// shut down; on reconnect the chips are woken back up and the backlight
    /// is ramped to the current global brightness (which reflects any active
    /// thermal throttle). No-op while the connection state is unchanged.
    ///
    /// While a first-boot calibration owns the display, the connect path
    /// repaints the active calibration frame instead of running the white
    /// ramp. Without this, the first connection tick after boot (or a USB
    /// resume mid-calibration) would clobber the guided amber/gradient
    /// display with a one-second ramp to solid white.
    async fn handle_power(&mut self, state: &mut BacklightState, connected: bool) {
        if connected == state.flags.get(BacklightFlags::HOST_CONNECTED) {
            return;
        }
        state.flags.set(BacklightFlags::HOST_CONNECTED, connected);
        if connected {
            _ = self.driver.wake().await;
            if state.calib_display == CalibDisplay::None {
                // Ramp to the state-tracked brightness rather than a fixed
                // 100% so an active thermal throttle survives the reconnect;
                // the thermal ticker restores full brightness once cool.
                _ = brightness_ramp(&mut self.driver, INDICATOR_WHITE, state.brightness, true, *state).await;
            } else {
                _ = self.render_current(*state).await;
            }
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
    /// [`BacklightRunner::render_current`] so a thermal event during a
    /// calibration pass repaints the active calibration frame (amber or
    /// gradient) rather than clobbering it with the normal white display.
    async fn handle_thermal_tick(&mut self, state: &mut BacklightState) {
        let hot = self.driver.check_thermal_flag_set(0).await || self.driver.check_thermal_flag_set(1).await;
        let new_brightness = if hot { THERMAL_THROTTLE_BRIGHTNESS } else { FULL_BRIGHTNESS };
        if new_brightness == state.brightness {
            return;
        }
        state.brightness = new_brightness;
        _ = self.render_current(*state).await;
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

    /// Repaint whichever frame currently owns the display: the active
    /// guided-calibration frame, or the normal white background with the
    /// indicator overlays.
    ///
    /// Used by every asynchronous repaint path (thermal throttle, USB
    /// connect/resume) so an unrelated event can never replace the
    /// calibration display with the wrong frame.
    ///
    /// # Errors
    ///
    /// Returns `Err` if any bus transaction fails. See
    /// [`BacklightDriver::flush`].
    async fn render_current(&mut self, state: BacklightState) -> Result<(), BusError> {
        match state.calib_display {
            CalibDisplay::None => render_all(&mut self.driver, state).await,
            CalibDisplay::Zero => fill_all_leds(&mut self.driver, CALIB_AMBER, state.brightness).await,
            CalibDisplay::Full => render_calib(&mut self.driver, state).await,
        }
    }
}

impl Runnable for BacklightRunner {
    /// Initialises both SNLED27351 driver chips, waits for USB enumeration,
    /// then enters an event loop that concurrently dispatches two
    /// concerns to dedicated handlers:
    ///
    /// - **Indicator and calibration commands** arriving on [`BACKLIGHT_CH`]
    ///   are routed to [`BacklightRunner::handle_cmd`].
    /// - **Thermal polling** on a [`THERMAL_POLL`] ticker is handled by
    ///   [`BacklightRunner::handle_thermal_tick`].
    ///
    /// Backlight failures are non-critical: the keyboard remains fully
    /// functional without LEDs, so each handler ignores driver errors via
    /// `_ = …`.
    async fn run(&mut self) -> ! {
        for _ in 0_u8..3_u8 {
            if self.driver.init(0xFF).await.is_ok() {
                break;
            }
            Timer::after_millis(200).await;
        }

        let rx = BACKLIGHT_CH.receiver();
        let mut thermal_ticker = Ticker::every(THERMAL_POLL);
        let mut state = BacklightState::default();

        loop {
            match select(rx.receive(), thermal_ticker.next()).await {
                Either::First(cmd) => self.handle_cmd(&mut state, cmd).await,
                Either::Second(()) => self.handle_thermal_tick(&mut state).await,
            }
        }
    }
}
