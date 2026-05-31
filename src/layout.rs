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

/// Column-major transpose of [`SENSOR_POSITIONS`].
///
/// `SENSOR_BY_COL[col][row]` is `true` when position `(row, col)` has a
/// physical hall-effect sensor. The inner scan loop holds `col` fixed while
/// iterating over all `row` values, so the column-major layout makes all six
/// row lookups for a given column adjacent in flash and friendly to the
/// STM32F4 ART prefetch.
pub const SENSOR_BY_COL: [[bool; ROW]; COL] = {
    let mut out = [[false; ROW]; COL];
    let mut col = 0;
    while col < COL {
        let mut row = 0;
        while row < ROW {
            if let Some(sensor_row) = SENSOR_POSITIONS.get(row)
                && let Some(&val) = sensor_row.get(col)
                && let Some(out_row) = out.get_mut(col)
                && let Some(cell) = out_row.get_mut(row)
            {
                *cell = val;
            }
            row = row.saturating_add(1);
        }
        col = col.saturating_add(1);
    }
    out
};

/// Per-column table of valid sensor positions, built at compile time from
/// [`SENSOR_BY_COL`].
///
/// Replaces the per-iteration `SENSOR_BY_COL` Option-chain lookup in the hot
/// scan path. Only positions where a physical hall-effect sensor is present
/// are stored; the scan loop iterates exactly these positions and skips all
/// others without a branch or a runtime sensor check.
///
/// Stored as a `static` rather than `const` so the linker places exactly one
/// copy in `.rodata` at a fixed address; the STM32F4 ART prefetch buffer
/// then keeps repeated accesses cheap across scan iterations rather than
/// re-fetching the table from inlined copies at every call site.
pub static VALID_ROWS_BY_COL: [ColValidKeys; COL] = {
    const EMPTY_KEY: ValidKey = ValidKey { buf_row: 0, key_row: 0 };
    const EMPTY_COL: ColValidKeys = ColValidKeys { keys: [EMPTY_KEY; ROW], count: 0 };

    let mut result = [EMPTY_COL; COL];
    let mut col = 0_usize;
    while col < COL {
        let mut count = 0_usize;
        let mut row = 0_usize;
        while row < ROW {
            let has_sensor = if let Some(col_sensors) = SENSOR_BY_COL.get(col)
                && let Some(&val) = col_sensors.get(row)
            {
                val
            } else {
                false
            };

            if has_sensor {
                if let Some(col_entry) = result.get_mut(col)
                    && let Some(slot) = col_entry.keys.get_mut(count)
                {
                    let row_u8 = u8::try_from(row).unwrap_or(u8::MAX);
                    *slot = ValidKey { buf_row: row_u8, key_row: row_u8 };
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

/// Precomputed list of valid sensor positions for one HC164 column.
///
/// Only the first [`ColValidKeys::count`] entries of [`ColValidKeys::keys`]
/// are populated; the remainder are zeroed placeholders that are never read.
#[derive(Clone, Copy)]
pub struct ColValidKeys {
    /// Number of valid entries in [`ColValidKeys::keys`].
    pub count: usize,
    /// Sensor positions with physical hall-effect sensors present, in
    /// ascending row order.
    pub keys:  [ValidKey; ROW],
}

/// A single valid sensor position within a column, storing both the ADC
/// buffer index and the key-state matrix index for that sensor.
///
/// On this hardware the ADC channel order matches the matrix row order, so
/// [`ValidKey::buf_row`] and [`ValidKey::key_row`] are always equal. They are
/// kept separate so the mapping can be adjusted without changing the hot-path
/// scan logic if the hardware ever diverges.
#[derive(Clone, Copy)]
pub struct ValidKey {
    /// Index into the ADC DMA buffer for this sensor's row channel.
    pub buf_row: u8,
    /// Row index into the `[[KeyEntry; ROW]; COL]` key-state array (the
    /// inner dimension of the column-major matrix).
    pub key_row: u8,
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
