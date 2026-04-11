use BacklightCmd::Indicators;
use rmk::{
    channel::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel},
    event::LedIndicatorEvent,
    macros::processor,
};

/// Channel used to send backlight indicator commands.
pub static BACKLIGHT_CH: Channel<CriticalSectionRawMutex, BacklightCmd, 8> = Channel::new();

#[derive(Copy, Clone)]
/// Commands for lock indicator LEDs.
pub enum BacklightCmd {
    /// Update Caps Lock / Num Lock indicator LED states.
    Indicators { caps: bool, num: bool },
    /// Signal a first-boot calibration phase transition via the backlight.
    CalibPhase(CalibPhase),
    /// Report full-travel calibration progress as a percentage (0–100).
    ///
    /// Drives the whole-keyboard blue→green gradient.
    CalibProgress(u8),
    /// Mark a single LED as fully calibrated during the full-travel pass.
    ///
    /// The backlight task immediately repaints that LED green, providing
    /// per-key visual confirmation independent of the gradient background.
    CalibKeyDone(u8),
}

/// Calibration phase signaled to the backlight task during first-boot setup.
#[derive(Copy, Clone)]
pub enum CalibPhase {
    /// Zero-travel pass in progress; keys should be fully released.
    /// Backlight color: amber.
    Zero,
    /// Full-travel pass in progress; user should press every key to the bottom.
    /// Backlight color: blue.
    Full,
    /// Calibration complete and persisted to EEPROM; keyboard is ready.
    /// Backlight color: green for 2 s, then restores normal white.
    Done,
}

#[processor(subscribe = [LedIndicatorEvent])]
/// Controller that reacts to indicator events for the LED driver.
pub struct LedIndicatorProcessor;

impl LedIndicatorProcessor {
    /// Create a new indicator controller.
    pub const fn new() -> Self { Self }

    /// Handle incoming lock LED indicator events.
    async fn on_led_indicator_event(&self, event: LedIndicatorEvent) {
        let caps = event.indicator.caps_lock();
        let num = event.indicator.num_lock();

        BACKLIGHT_CH.sender().send(Indicators { caps, num }).await;
    }
}
