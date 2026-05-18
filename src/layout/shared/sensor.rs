use crate::layout::COL;

/// Hall-effect sensor presence for matrix row 0; the encoder-button column
/// has no sensor.
pub const SENSOR_ROW0: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, false, true, true, true, true, true,
    true, true,
];
/// Hall-effect sensor presence for matrix row 1; all positions populated.
pub const SENSOR_ROW1: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];
/// Hall-effect sensor presence for matrix row 2; all positions populated.
pub const SENSOR_ROW2: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];
/// Hall-effect sensor presence for matrix row 5; space bar, layer-toggle,
/// and keypad-enter positions leave gaps.
pub const SENSOR_ROW5: [bool; COL] = [
    true, true, true, false, false, false, true, false, false, true, true, true, true, false, true, true, true, true,
    true, false, false,
];
