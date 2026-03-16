//! Backlight task initialization and runtime loop.

use crate::{
    backlight::{
        gamma_correction::gamma_correction,
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
use embassy_time::Timer;

/// LED index for the caps lock indicator.
const CAPS_LOCK_LED_INDEX: usize = 62;
/// Number of SNLED27351 drivers on this keyboard.
const DRIVER_COUNT: usize = 2;
/// LED index for the num lock indicator.
const NUM_LOCK_LED_INDEX: usize = 37;
/// Brightness used for indicator LEDs.
const INDICATOR_BRIGHTNESS: u8 = 100;
/// RGB value for the red indicator state.
const INDICATOR_RED: (u8, u8, u8) = (255, 0, 0);
/// RGB value for the white indicator state.
const INDICATOR_WHITE: (u8, u8, u8) = (255, 255, 255);
/// RGB value for the off indicator state.
const INDICATOR_OFF: (u8, u8, u8) = (0, 0, 0);

/// Number of steps in the soft-start ramp.
const SOFTSTART_STEPS: u8 = 50;
/// Duration of the soft-start ramp in milliseconds.
const SOFTSTART_RAMP_MS: u32 = 1000;

/// Scales a 0–255 linear value by a 0–100 brightness percentage.
///
/// Uses integer arithmetic with rounding to avoid floating point.
#[inline]
const fn scale(value: u8, brightness_percent: u8) -> u8 {
    let value_scaled = u16::from(value);
    let percent_scaled = u16::from(brightness_percent.min(100));
    u8::try_from(value_scaled.saturating_mul(percent_scaled).saturating_add(50).saturating_div(100)).unwrap_or(255)
}

/// Applies brightness scaling and gamma 2.2 correction to an RGB triplet.
///
/// Returns corrected `(red, green, blue)` ready to write directly to the
/// PWM shadow buffer via [`Snled27351::set_led`] or
/// [`Snled27351::set_all_leds`].
#[inline]
const fn correct(red: u8, green: u8, blue: u8, brightness_percent: u8) -> (u8, u8, u8) {
    (
        gamma_correction(scale(red, brightness_percent)),
        gamma_correction(scale(green, brightness_percent)),
        gamma_correction(scale(blue, brightness_percent)),
    )
}

/// Runs the soft-start brightness ramp at initialization.
///
/// Steps brightness from 0 up to `target_brightness` over
/// [`SOFTSTART_STEPS`] increments across [`SOFTSTART_RAMP_MS`]
/// milliseconds, writing corrected PWM values and flushing each step.
#[expect(clippy::arithmetic_side_effects, reason = "steps is guaranteed non-zero via .max(1)")]
async fn softstart(
    backlight: &mut Snled27351<'static, DRIVER_COUNT>,
    base_red: u8,
    base_green: u8,
    base_blue: u8,
    target_brightness: u8,
) {
    let target = u32::from(target_brightness.min(100));
    let steps = u32::from(SOFTSTART_STEPS.max(1));
    let delay_ms = u64::from(SOFTSTART_RAMP_MS.saturating_div(steps));

    let mut step = 0_u32;
    while step <= steps {
        let percent = u8::try_from(target.saturating_mul(step).saturating_div(steps)).unwrap_or(0);
        let (scaled_red, scaled_green, scaled_blue) = correct(base_red, base_green, base_blue, percent);
        backlight.set_all_leds(scaled_red, scaled_green, scaled_blue).await;
        Timer::after_millis(delay_ms).await;
        step = step.saturating_add(1);
    }
}

/// Runs the backlight controller loop.
///
/// Initializes the SNLED27351 drivers, performs a soft-start brightness ramp,
/// then enters the event loop responding to [`BacklightCmd`] messages.
pub async fn backlight_runner(
    spi: Spi<'static, Async, spi::mode::Master>,
    cs0: Output<'static>,
    cs1: Output<'static>,
    sdb: Output<'static>,
) -> ! {
    let cs = [cs0, cs1];
    let mut backlight = Snled27351::new(spi, cs, sdb, LED_LAYOUT);

    backlight.init(0xFF).await;
    softstart(&mut backlight, 255, 255, 255, 100).await;

    let rx = BACKLIGHT_CH.receiver();
    loop {
        match rx.receive().await {
            BacklightCmd::Indicators { caps, num } => {
                let (caps_r, caps_g, caps_b) = if caps { INDICATOR_RED } else { INDICATOR_WHITE };
                let (scaled_red_caps, scaled_green_caps, scaled_blue_caps) =
                    correct(caps_r, caps_g, caps_b, INDICATOR_BRIGHTNESS);
                backlight.set_led(CAPS_LOCK_LED_INDEX, scaled_red_caps, scaled_green_caps, scaled_blue_caps).await;

                let (num_r, num_g, num_b) = if num { INDICATOR_WHITE } else { INDICATOR_OFF };
                let (scaled_red_num, scaled_green_num, scaled_blue_num) =
                    correct(num_r, num_g, num_b, INDICATOR_BRIGHTNESS);
                backlight.set_led(NUM_LOCK_LED_INDEX, scaled_red_num, scaled_green_num, scaled_blue_num).await;
            }
        }
    }
}
