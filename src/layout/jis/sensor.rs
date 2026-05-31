use crate::layout::COL;
pub use crate::layout::shared::sensor::{SENSOR_ROW1, SENSOR_ROW2};

/// Hall-effect sensor presence for matrix row 0 (JIS).
pub const SENSOR_ROW0: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];

/// Hall-effect sensor presence for matrix row 3 (JIS).
pub const SENSOR_ROW3: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, true,
    true, true, false,
];

/// Hall-effect sensor presence for matrix row 4 (JIS).
pub const SENSOR_ROW4: [bool; COL] = [
    true, false, true, true, true, true, true, true, true, true, true, true, true, true, false, true, false, true,
    true, true, true,
];

/// Hall-effect sensor presence for matrix row 5 (JIS).
pub const SENSOR_ROW5: [bool; COL] = [
    true, true, true, true, false, false, true, false, true, true, true, true, true, false, true, true, true, true,
    true, false, false,
];
