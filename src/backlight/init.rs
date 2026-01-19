use crate::{
    backlight::{
        lock_indicator::{BACKLIGHT_CH, BacklightCmd},
        mapping::*,
    },
    snled27351_spi::driver::Snled27351,
};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{self, Spi},
};

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
            BacklightCmd::Panic => loop {
                backlight.set_color_all(255, 0, 0, 100).await;
                embassy_time::Timer::after_millis(300).await;
                backlight.set_color_all(0, 0, 0, 0).await;
                embassy_time::Timer::after_millis(300).await;
            },
        }
    }
}
