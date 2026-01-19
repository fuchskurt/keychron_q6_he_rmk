use rmk::{
    channel::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel},
    event::LedIndicatorEvent,
    macros::controller,
};

pub static BACKLIGHT_CH: Channel<CriticalSectionRawMutex, BacklightCmd, 4> = Channel::new();

#[derive(Copy, Clone)]
pub enum BacklightCmd {
    Indicators { caps: bool, num: bool },
}

#[controller(subscribe = [LedIndicatorEvent])]
pub struct SnledIndicatorController;

impl SnledIndicatorController {
    pub fn new() -> Self { Self }

    async fn on_led_indicator_event(&mut self, event: LedIndicatorEvent) {
        let caps = event.indicator.caps_lock();
        let num = event.indicator.num_lock();

        BACKLIGHT_CH.sender().send(BacklightCmd::Indicators { caps, num }).await;
    }
}
