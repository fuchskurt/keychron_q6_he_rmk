use crate::snled27351_spi::{
    driver::{Snled27351, SnledLed},
    led_address::*,
};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};
use rmk::{
    channel::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel},
    event::LedIndicatorEvent,
    macros::controller,
};
pub const LED_LAYOUT: &[SnledLed] = &[
    SnledLed { driver: 0, r: CB7_CA16, g: CB9_CA16, b: CB8_CA16 },
    SnledLed { driver: 0, r: CB7_CA15, g: CB9_CA15, b: CB8_CA15 },
    SnledLed { driver: 0, r: CB7_CA14, g: CB9_CA14, b: CB8_CA14 },
    SnledLed { driver: 0, r: CB7_CA13, g: CB9_CA13, b: CB8_CA13 },
    SnledLed { driver: 0, r: CB7_CA12, g: CB9_CA12, b: CB8_CA12 },
    SnledLed { driver: 0, r: CB7_CA11, g: CB9_CA11, b: CB8_CA11 },
    SnledLed { driver: 0, r: CB7_CA10, g: CB9_CA10, b: CB8_CA10 },
    SnledLed { driver: 0, r: CB7_CA9, g: CB9_CA9, b: CB8_CA9 },
    SnledLed { driver: 0, r: CB7_CA8, g: CB9_CA8, b: CB8_CA8 },
    SnledLed { driver: 0, r: CB7_CA7, g: CB9_CA7, b: CB8_CA7 },
    SnledLed { driver: 0, r: CB7_CA6, g: CB9_CA6, b: CB8_CA6 },
    SnledLed { driver: 0, r: CB7_CA5, g: CB9_CA5, b: CB8_CA5 },
    SnledLed { driver: 0, r: CB7_CA4, g: CB9_CA4, b: CB8_CA4 },
    SnledLed { driver: 0, r: CB7_CA2, g: CB9_CA2, b: CB8_CA2 },
    SnledLed { driver: 0, r: CB7_CA1, g: CB9_CA1, b: CB8_CA1 },
    SnledLed { driver: 1, r: CB1_CA3, g: CB3_CA3, b: CB2_CA3 },
    SnledLed { driver: 0, r: CB10_CA4, g: CB12_CA4, b: CB11_CA4 },
    SnledLed { driver: 0, r: CB10_CA3, g: CB12_CA3, b: CB11_CA3 },
    SnledLed { driver: 0, r: CB10_CA2, g: CB12_CA2, b: CB11_CA2 },
    SnledLed { driver: 0, r: CB10_CA1, g: CB12_CA1, b: CB11_CA1 },
    SnledLed { driver: 0, r: CB1_CA16, g: CB3_CA16, b: CB2_CA16 },
    SnledLed { driver: 0, r: CB1_CA15, g: CB3_CA15, b: CB2_CA15 },
    SnledLed { driver: 0, r: CB1_CA14, g: CB3_CA14, b: CB2_CA14 },
    SnledLed { driver: 0, r: CB1_CA13, g: CB3_CA13, b: CB2_CA13 },
    SnledLed { driver: 0, r: CB1_CA12, g: CB3_CA12, b: CB2_CA12 },
    SnledLed { driver: 0, r: CB1_CA11, g: CB3_CA11, b: CB2_CA11 },
    SnledLed { driver: 0, r: CB1_CA10, g: CB3_CA10, b: CB2_CA10 },
    SnledLed { driver: 0, r: CB1_CA9, g: CB3_CA9, b: CB2_CA9 },
    SnledLed { driver: 0, r: CB1_CA8, g: CB3_CA8, b: CB2_CA8 },
    SnledLed { driver: 0, r: CB1_CA7, g: CB3_CA7, b: CB2_CA7 },
    SnledLed { driver: 0, r: CB1_CA6, g: CB3_CA6, b: CB2_CA6 },
    SnledLed { driver: 0, r: CB1_CA5, g: CB3_CA5, b: CB2_CA5 },
    SnledLed { driver: 0, r: CB1_CA4, g: CB3_CA4, b: CB2_CA4 },
    SnledLed { driver: 0, r: CB1_CA3, g: CB3_CA3, b: CB2_CA3 },
    SnledLed { driver: 0, r: CB1_CA2, g: CB3_CA2, b: CB2_CA2 },
    SnledLed { driver: 0, r: CB1_CA1, g: CB3_CA1, b: CB2_CA1 },
    SnledLed { driver: 1, r: CB1_CA2, g: CB3_CA2, b: CB2_CA2 },
    SnledLed { driver: 0, r: CB10_CA8, g: CB12_CA8, b: CB11_CA8 },
    SnledLed { driver: 0, r: CB10_CA7, g: CB12_CA7, b: CB11_CA7 },
    SnledLed { driver: 0, r: CB10_CA6, g: CB12_CA6, b: CB11_CA6 },
    SnledLed { driver: 0, r: CB10_CA5, g: CB12_CA5, b: CB11_CA5 },
    SnledLed { driver: 0, r: CB4_CA16, g: CB6_CA16, b: CB5_CA16 },
    SnledLed { driver: 0, r: CB4_CA15, g: CB6_CA15, b: CB5_CA15 },
    SnledLed { driver: 0, r: CB4_CA14, g: CB6_CA14, b: CB5_CA14 },
    SnledLed { driver: 0, r: CB4_CA13, g: CB6_CA13, b: CB5_CA13 },
    SnledLed { driver: 0, r: CB4_CA12, g: CB6_CA12, b: CB5_CA12 },
    SnledLed { driver: 0, r: CB4_CA11, g: CB6_CA11, b: CB5_CA11 },
    SnledLed { driver: 0, r: CB4_CA10, g: CB6_CA10, b: CB5_CA10 },
    SnledLed { driver: 0, r: CB4_CA9, g: CB6_CA9, b: CB5_CA9 },
    SnledLed { driver: 0, r: CB4_CA8, g: CB6_CA8, b: CB5_CA8 },
    SnledLed { driver: 0, r: CB4_CA7, g: CB6_CA7, b: CB5_CA7 },
    SnledLed { driver: 0, r: CB4_CA6, g: CB6_CA6, b: CB5_CA6 },
    SnledLed { driver: 0, r: CB4_CA5, g: CB6_CA5, b: CB5_CA5 },
    SnledLed { driver: 0, r: CB4_CA4, g: CB6_CA4, b: CB5_CA4 },
    SnledLed { driver: 0, r: CB4_CA3, g: CB6_CA3, b: CB5_CA3 },
    SnledLed { driver: 0, r: CB4_CA2, g: CB6_CA2, b: CB5_CA2 },
    SnledLed { driver: 0, r: CB4_CA1, g: CB6_CA1, b: CB5_CA1 },
    SnledLed { driver: 1, r: CB1_CA1, g: CB3_CA1, b: CB2_CA1 },
    SnledLed { driver: 0, r: CB10_CA12, g: CB12_CA12, b: CB11_CA12 },
    SnledLed { driver: 0, r: CB10_CA11, g: CB12_CA11, b: CB11_CA11 },
    SnledLed { driver: 0, r: CB10_CA10, g: CB12_CA10, b: CB11_CA10 },
    SnledLed { driver: 0, r: CB10_CA9, g: CB12_CA9, b: CB11_CA9 },
    SnledLed { driver: 1, r: CB1_CA16, g: CB3_CA16, b: CB2_CA16 },
    SnledLed { driver: 1, r: CB1_CA15, g: CB3_CA15, b: CB2_CA15 },
    SnledLed { driver: 1, r: CB1_CA14, g: CB3_CA14, b: CB2_CA14 },
    SnledLed { driver: 1, r: CB1_CA13, g: CB3_CA13, b: CB2_CA13 },
    SnledLed { driver: 1, r: CB1_CA12, g: CB3_CA12, b: CB2_CA12 },
    SnledLed { driver: 1, r: CB1_CA11, g: CB3_CA11, b: CB2_CA11 },
    SnledLed { driver: 1, r: CB1_CA10, g: CB3_CA10, b: CB2_CA10 },
    SnledLed { driver: 1, r: CB1_CA9, g: CB3_CA9, b: CB2_CA9 },
    SnledLed { driver: 1, r: CB1_CA8, g: CB3_CA8, b: CB2_CA8 },
    SnledLed { driver: 1, r: CB1_CA7, g: CB3_CA7, b: CB2_CA7 },
    SnledLed { driver: 1, r: CB1_CA6, g: CB3_CA6, b: CB2_CA6 },
    SnledLed { driver: 1, r: CB1_CA5, g: CB3_CA5, b: CB2_CA5 },
    SnledLed { driver: 1, r: CB1_CA4, g: CB3_CA4, b: CB2_CA4 },
    SnledLed { driver: 1, r: CB10_CA9, g: CB12_CA9, b: CB11_CA9 },
    SnledLed { driver: 1, r: CB10_CA8, g: CB12_CA8, b: CB11_CA8 },
    SnledLed { driver: 1, r: CB10_CA7, g: CB12_CA7, b: CB11_CA7 },
    SnledLed { driver: 1, r: CB7_CA16, g: CB9_CA16, b: CB8_CA16 },
    SnledLed { driver: 1, r: CB7_CA14, g: CB9_CA14, b: CB8_CA14 },
    SnledLed { driver: 1, r: CB7_CA13, g: CB9_CA13, b: CB8_CA13 },
    SnledLed { driver: 1, r: CB7_CA12, g: CB9_CA12, b: CB8_CA12 },
    SnledLed { driver: 1, r: CB7_CA11, g: CB9_CA11, b: CB8_CA11 },
    SnledLed { driver: 1, r: CB7_CA10, g: CB9_CA10, b: CB8_CA10 },
    SnledLed { driver: 1, r: CB7_CA9, g: CB9_CA9, b: CB8_CA9 },
    SnledLed { driver: 1, r: CB7_CA8, g: CB9_CA8, b: CB8_CA8 },
    SnledLed { driver: 1, r: CB7_CA7, g: CB9_CA7, b: CB8_CA7 },
    SnledLed { driver: 1, r: CB7_CA6, g: CB9_CA6, b: CB8_CA6 },
    SnledLed { driver: 1, r: CB7_CA5, g: CB9_CA5, b: CB8_CA5 },
    SnledLed { driver: 1, r: CB7_CA3, g: CB9_CA3, b: CB8_CA3 },
    SnledLed { driver: 1, r: CB7_CA1, g: CB9_CA1, b: CB8_CA1 },
    SnledLed { driver: 1, r: CB10_CA6, g: CB12_CA6, b: CB11_CA6 },
    SnledLed { driver: 1, r: CB10_CA5, g: CB12_CA5, b: CB11_CA5 },
    SnledLed { driver: 1, r: CB10_CA4, g: CB12_CA4, b: CB11_CA4 },
    SnledLed { driver: 1, r: CB10_CA3, g: CB12_CA3, b: CB11_CA3 },
    SnledLed { driver: 1, r: CB4_CA16, g: CB6_CA16, b: CB5_CA16 },
    SnledLed { driver: 1, r: CB4_CA15, g: CB6_CA15, b: CB5_CA15 },
    SnledLed { driver: 1, r: CB4_CA14, g: CB6_CA14, b: CB5_CA14 },
    SnledLed { driver: 1, r: CB4_CA10, g: CB6_CA10, b: CB5_CA10 },
    SnledLed { driver: 1, r: CB4_CA7, g: CB6_CA7, b: CB5_CA7 },
    SnledLed { driver: 1, r: CB4_CA6, g: CB6_CA6, b: CB5_CA6 },
    SnledLed { driver: 1, r: CB4_CA5, g: CB6_CA5, b: CB5_CA5 },
    SnledLed { driver: 1, r: CB4_CA4, g: CB6_CA4, b: CB5_CA4 },
    SnledLed { driver: 1, r: CB4_CA2, g: CB6_CA2, b: CB5_CA2 },
    SnledLed { driver: 1, r: CB4_CA1, g: CB6_CA1, b: CB5_CA1 },
    SnledLed { driver: 1, r: CB7_CA2, g: CB9_CA2, b: CB8_CA2 },
    SnledLed { driver: 1, r: CB10_CA2, g: CB12_CA2, b: CB11_CA2 },
    SnledLed { driver: 1, r: CB10_CA1, g: CB12_CA1, b: CB11_CA1 },
];

pub static BACKLIGHT_CH: Channel<CriticalSectionRawMutex, BacklightCmd, 4> = Channel::new();

#[derive(Copy, Clone)]
pub enum BacklightCmd {
    Indicators { caps: bool, num: bool },
    Panic,
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

pub async fn backlight_runner(
    spi: Spi<'static, Async, spi::mode::Master>,
    cs0: Output<'static>,
    cs1: Output<'static>,
    sdb: Output<'static>,
) -> ! {
    let cs = [cs0, cs1];
    let mut backlight = Snled27351::new(spi, cs, sdb, LED_LAYOUT);

    backlight.init(0xFF).await;
    backlight.set_color_all_softstart(255, 255, 255, 100, 50, 1000).await;

    let rx = BACKLIGHT_CH.receiver();
    loop {
        match rx.receive().await {
            BacklightCmd::Indicators { caps, num } => {
                if caps {
                    backlight.set_color(62, 255, 0, 0, 100).await;
                } else {
                    backlight.set_color(62, 255, 255, 255, 100).await;
                }

                if num {
                    backlight.set_color(37, 255, 255, 255, 100).await;
                } else {
                    backlight.set_color(37, 0, 0, 0, 0).await;
                }
            }
            BacklightCmd::Panic => {
                loop {
                    // solid red or blinking red
                    backlight.set_color_all(255, 0, 0, 100).await;
                    embassy_time::Timer::after_millis(300).await;
                    backlight.set_color_all(0, 0, 0, 0).await;
                    embassy_time::Timer::after_millis(300).await;
                }
            }
        }
    }
}
