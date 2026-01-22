//! Lock indicator backlight integration.

use rmk::{
    channel::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel},
    event::LedIndicatorEvent,
    macros::controller,
};

/// Channel used to send backlight indicator commands.
pub static BACKLIGHT_CH: Channel<CriticalSectionRawMutex, BacklightCmd, 4> = Channel::new();

#[derive(Copy, Clone)]
/// Commands for lock indicator LEDs.
pub enum BacklightCmd {
    /// Update caps/num lock indicator states.
    Indicators {
        /// Whether Caps Lock is active.
        caps: bool,

        /// Whether Num Lock is active.
        num: bool,
    },
    /// Trigger a panic blink sequence.
    Panic,
}

#[controller(subscribe = [LedIndicatorEvent])]
/// Controller that reacts to indicator events for the SNLED driver.
pub struct SnledIndicatorController;

impl SnledIndicatorController {
    /// Create a new indicator controller.
    pub const fn new() -> Self { Self }

    /// Handle incoming lock LED indicator events.
    async fn on_led_indicator_event(&self, event: LedIndicatorEvent) {
        let caps = event.indicator.caps_lock();
        let num = event.indicator.num_lock();

        BACKLIGHT_CH.sender().send(BacklightCmd::Indicators { caps, num }).await;
    }
}
