use crate::layout::COL;

// Row 0: col 13 = AudioMute mapped to the encoder button; no hall switch.
pub const SENSOR_ROW0: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, false, true, true, true, true, true,
    true, true,
];
// Row 1: all 21 positions populated.
pub const SENSOR_ROW1: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];
// Row 2: all 21 positions populated.
pub const SENSOR_ROW2: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];
// Row 5: cols 3–5 = Space, cols 7–8 = layer toggle, col 13 = gap, cols 19–20 =
// KpEnter.
pub const SENSOR_ROW5: [bool; COL] = [
    true, true, true, false, false, false, true, false, false, true, true, true, true, false, true, true, true, true,
    true, false, false,
];
