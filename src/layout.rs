//! Default keymap definitions and sizes.

/// Layout-independent shared tables, encoder map, and compile-time guards.
mod shared;

::cfg_if::cfg_if! {
    if #[cfg(feature = "ansi_layout")] {
        /// ANSI physical layout tables.
        mod ansi;
        use ansi as selected;
    } else if #[cfg(feature = "iso_layout")] {
        /// ISO physical layout tables.
        mod iso;
        use iso as selected;
    } else if #[cfg(feature = "jis_layout")] {
        /// JIS physical layout tables.
        mod jis;
        use jis as selected;
    } else {
        compile_error!("You must enable one of: ansi_layout, iso_layout, jis_layout");
    }
}

use rmk::types::action::KeyAction;
pub use selected::led::LED_LAYOUT;
use selected::{
    layout::{MAC_ROW5, ROW0, ROW1, ROW2, ROW3, ROW4, WIN_ROW5},
    led::{LED_MAPPING_ROW0, LED_MAPPING_ROW1, LED_MAPPING_ROW2, LED_MAPPING_ROW3, LED_MAPPING_ROW4, LED_MAPPING_ROW5},
    sensor::{SENSOR_ROW0, SENSOR_ROW1, SENSOR_ROW2, SENSOR_ROW3, SENSOR_ROW4, SENSOR_ROW5},
};
use shared::layout::{ENCODER_VOLUME, EncoderAction};

/// Number of columns in the key matrix.
pub const COL: usize = 21;
/// Number of rows in the key matrix.
pub const ROW: usize = 6;
/// Number of supported layers.
pub const NUM_LAYER: usize = 2;
/// Number of rotary encoders.
pub const NUM_ENCODER: usize = 1;

/// Matrix-position-to-LED-index map, row-major `[[Option<u8>; COL]; ROW]`.
///
/// `None` marks a matrix position with no backlight LED.
pub const MATRIX_TO_LED: [[Option<u8>; COL]; ROW] =
    [LED_MAPPING_ROW0, LED_MAPPING_ROW1, LED_MAPPING_ROW2, LED_MAPPING_ROW3, LED_MAPPING_ROW4, LED_MAPPING_ROW5];

/// Per-position hall-effect sensor presence map, row-major
/// `[[bool; COL]; ROW]`. `true` where a physical sensor exists.
pub const SENSOR_POSITIONS: [[bool; COL]; ROW] =
    [SENSOR_ROW0, SENSOR_ROW1, SENSOR_ROW2, SENSOR_ROW3, SENSOR_ROW4, SENSOR_ROW5];

/// Per-column table of valid sensor positions, built at compile time from
/// [`SENSOR_POSITIONS`].
///
/// Only positions where a physical hall-effect sensor is present are stored;
/// the scan loop iterates exactly these positions and skips all others without
/// a branch or a runtime sensor check.
///
/// Stored as a `static` rather than `const` so the linker places exactly one
/// copy in `.rodata` at a fixed address; the STM32F4 ART prefetch buffer
/// then keeps repeated accesses cheap across scan iterations rather than
/// re-fetching the table from inlined copies at every call site.
pub static VALID_ROWS_BY_COL: [ColValidKeys; COL] = {
    const EMPTY_COL: ColValidKeys = ColValidKeys { count: 0, rows: [0; ROW] };

    let mut result = [EMPTY_COL; COL];
    let mut col = 0_usize;
    while col < COL {
        let mut count = 0_usize;
        let mut row = 0_usize;
        while row < ROW {
            // SENSOR_POSITIONS is row-major, so transpose the indices here to
            // walk it column-major without a separate intermediate table.
            let has_sensor = if let Some(sensor_row) = SENSOR_POSITIONS.get(row)
                && let Some(&val) = sensor_row.get(col)
            {
                val
            } else {
                false
            };

            if has_sensor {
                if let Some(col_entry) = result.get_mut(col)
                    && let Some(slot) = col_entry.rows.get_mut(count)
                {
                    *slot = u8::try_from(row).unwrap_or(u8::MAX);
                }
                count = count.saturating_add(1);
            }
            row = row.saturating_add(1);
        }
        if let Some(col_entry) = result.get_mut(col) {
            col_entry.count = count;
        }
        col = col.saturating_add(1);
    }
    result
};

/// Precomputed list of valid sensor rows for one HC164 column.
///
/// Only the first [`ColValidKeys::count`] entries of [`ColValidKeys::rows`]
/// are populated; the remainder are zeroed placeholders that are never read.
/// Each row value indexes both the ADC DMA buffer and the key-state matrix:
/// the ADC channel order matches the matrix row order on this hardware.
#[derive(Clone, Copy)]
pub struct ColValidKeys {
    /// Number of valid entries in [`ColValidKeys::rows`].
    pub count: usize,
    /// Matrix rows with a physical hall-effect sensor present, in ascending
    /// order.
    pub rows:  [u8; ROW],
}

/// Return the default encoder action map for each layer.
pub const fn get_default_encoder_map() -> [[EncoderAction; NUM_ENCODER]; NUM_LAYER] { [ENCODER_VOLUME, ENCODER_VOLUME] }

/// Return the default keymap for all layers.
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        // Layer 0: MAC_BASE
        [ROW0, ROW1, ROW2, ROW3, ROW4, MAC_ROW5],
        // Layer 1: WIN_BASE
        [ROW0, ROW1, ROW2, ROW3, ROW4, WIN_ROW5],
    ]
}
