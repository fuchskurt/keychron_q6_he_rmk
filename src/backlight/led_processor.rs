use rmk::{
    channel::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel},
    event::LedIndicatorEvent,
    macros::processor,
};

/// Channel used to send backlight indicator commands.
pub static BACKLIGHT_CH: Channel<CriticalSectionRawMutex, BacklightCmd, 8> = Channel::new();

/// Commands for lock indicator LEDs.
#[derive(Copy, Clone)]
pub enum BacklightCmd {
    /// Mark a single LED as fully calibrated during the full-travel pass.
    ///
    /// The backlight task immediately repaints that LED green, providing
    /// per-key visual confirmation independent of the gradient background.
    CalibKeyDone(u8),
    /// Signal a first-boot calibration phase transition via the backlight.
    CalibPhase(CalibPhase),
    /// Report full-travel calibration progress as a percentage (0–100).
    ///
    /// Drives the whole-keyboard blue→green gradient.
    CalibProgress(u8),
    /// Update Caps Lock / Num Lock indicator LED states.
    Indicators {
        /// Whether Caps Lock is currently active.
        caps: bool,
        /// Whether Num Lock is currently active.
        num:  bool,
    },
}

/// Calibration phase signaled to the backlight task during first-boot setup.
#[derive(Copy, Clone)]
pub enum CalibPhase {
    /// All keys accepted; the settle window is running.
    /// Backlight: three short green flashes - signals the user every key has
    /// been recorded and all keys may be released.
    AllAccepted,
    /// Calibration complete and persisted to EEPROM; keyboard is ready.
    /// Backlight color: green for 2 s, then restores normal white.
    Done,
    /// Full-travel pass in progress; user should press every key to the bottom.
    /// Backlight color: red→blue gradient (0 % → 100 % of keys accepted);
    /// individual accepted keys light up green immediately.
    Full,
    /// Zero-travel pass in progress; keys should be fully released.
    /// Backlight color: amber.
    Zero,
}

/// Controller that reacts to indicator events for the LED driver.
#[processor(subscribe = [LedIndicatorEvent])]
pub struct LedIndicatorProcessor;

impl LedIndicatorProcessor {
    /// Create a new indicator controller.
    #[must_use]
    pub const fn new() -> Self { Self }

    /// Handle incoming lock LED indicator events.
    async fn on_led_indicator_event(&self, event: LedIndicatorEvent) {
        let caps = event.caps_lock();
        let num = event.num_lock();

        BACKLIGHT_CH.sender().try_send(BacklightCmd::Indicators { caps, num }).ok();
    }
}
