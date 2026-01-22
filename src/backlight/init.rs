//! Backlight task initialization and runtime loop.

use crate::{
    backlight::{
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

/// LED index for the caps lock indicator.
const CAPS_LOCK_LED_INDEX: usize = 62;
/// LED index for the num lock indicator.
const NUM_LOCK_LED_INDEX: usize = 37;
/// Brightness used for indicator LEDs.
const INDICATOR_BRIGHTNESS: u8 = 100;
/// Delay between panic blink toggles.
const PANIC_BLINK_DELAY_MS: u64 = 300;
/// RGB value for the red indicator state.
const INDICATOR_RED: (u8, u8, u8) = (255, 0, 0);
/// RGB value for the white indicator state.
const INDICATOR_WHITE: (u8, u8, u8) = (255, 255, 255);
/// RGB value for the off indicator state.
const INDICATOR_OFF: (u8, u8, u8) = (0, 0, 0);

/// Run the backlight controller loop.
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
                let (caps_r, caps_g, caps_b) = if caps { INDICATOR_RED } else { INDICATOR_WHITE };
                backlight
                    .set_color(CAPS_LOCK_LED_INDEX, caps_r, caps_g, caps_b, INDICATOR_BRIGHTNESS)
                    .await;
                let (num_r, num_g, num_b) = if num { INDICATOR_WHITE } else { INDICATOR_OFF };
                backlight
                    .set_color(NUM_LOCK_LED_INDEX, num_r, num_g, num_b, INDICATOR_BRIGHTNESS)
                    .await;
            }
            BacklightCmd::Panic => loop {
                let (panic_r, panic_g, panic_b) = INDICATOR_RED;
                backlight.set_color_all(panic_r, panic_g, panic_b, INDICATOR_BRIGHTNESS).await;
                embassy_time::Timer::after_millis(PANIC_BLINK_DELAY_MS).await;
                let (off_r, off_g, off_b) = INDICATOR_OFF;
                backlight.set_color_all(off_r, off_g, off_b, 0).await;
                embassy_time::Timer::after_millis(PANIC_BLINK_DELAY_MS).await;
            },
        }
    }
}
