//! Default keymap definitions and sizes.

use cfg_if::cfg_if;
use rmk::types::action::KeyAction;
use shared::layout::{ENCODER_VOLUME, EncoderAction};
mod shared;

cfg_if! {
    if #[cfg(feature = "ansi_layout")] {
        mod ansi;
        use ansi as selected;
    } else if #[cfg(feature = "iso_layout")] {
        mod iso;
        use iso as selected;
    } else if #[cfg(feature = "jis_layout")] {
        mod jis;
        use jis as selected;
    } else {
        compile_error!("You must enable one of: ansi_layout, iso_layout, jis_layout");
    }
}

pub use selected::led::LED_LAYOUT;
use selected::{
    layout::{MAC_ROW5, ROW0, ROW1, ROW2, ROW3, ROW4, WIN_ROW5},
    led::{LED_MAPPING_ROW0, LED_MAPPING_ROW1, LED_MAPPING_ROW2, LED_MAPPING_ROW3, LED_MAPPING_ROW4, LED_MAPPING_ROW5},
    sensor::{SENSOR_ROW0, SENSOR_ROW1, SENSOR_ROW2, SENSOR_ROW3, SENSOR_ROW4, SENSOR_ROW5},
};

/// Number of columns in the key matrix.
pub const COL: usize = 21;
/// Number of rows in the key matrix.
pub const ROW: usize = 6;
/// Number of supported layers.
pub const NUM_LAYER: usize = 2;
/// Number of rotary encoders.
pub const NUM_ENCODER: usize = 1;

pub const MATRIX_TO_LED: [[Option<u8>; COL]; ROW] =
    [LED_MAPPING_ROW0, LED_MAPPING_ROW1, LED_MAPPING_ROW2, LED_MAPPING_ROW3, LED_MAPPING_ROW4, LED_MAPPING_ROW5];

pub const SENSOR_POSITIONS: [[bool; COL]; ROW] =
    [SENSOR_ROW0, SENSOR_ROW1, SENSOR_ROW2, SENSOR_ROW3, SENSOR_ROW4, SENSOR_ROW5];

/// Return the default keymap for all layers.
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        // Layer 0: MAC_BASE
        [ROW0, ROW1, ROW2, ROW3, ROW4, MAC_ROW5],
        // Layer 1: WIN_BASE
        [ROW0, ROW1, ROW2, ROW3, ROW4, WIN_ROW5],
    ]
}

/// Return the default encoder action map for each layer.
pub const fn get_default_encoder_map() -> [[EncoderAction; NUM_ENCODER]; NUM_LAYER] { [ENCODER_VOLUME, ENCODER_VOLUME] }

/// Compile-time guard: [`LED_LAYOUT`] must fit within the 128-bit bitset.
const _: () = assert!(LED_LAYOUT.len() <= 128, "LED_LAYOUT exceeds the 128-LED capacity of the calib_leds_done bitset");

/// Compile-time guard: MATRIX_TO_LED indices must be valid.
const _: () = {
    let led_count = LED_LAYOUT.len();
    let mut row = 0;
    while row < MATRIX_TO_LED.len() {
        let mut col = 0;
        while col < MATRIX_TO_LED[row].len() {
            if let Some(idx) = MATRIX_TO_LED[row][col] {
                assert!((idx as usize) < led_count, "MATRIX_TO_LED contains an out-of-bounds LED index");
            }
            col += 1;
        }
        row += 1;
    }
};
