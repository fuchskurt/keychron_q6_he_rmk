use crate::layout::COL;
pub use crate::layout::shared::sensor::SENSOR_ROW1;

pub const SENSOR_ROW0: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];

pub const SENSOR_ROW2: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true,
];

pub const SENSOR_ROW3: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, true,
    true, true, false,
];

pub const SENSOR_ROW4: [bool; COL] = [
    true, false, true, true, true, true, true, true, true, true, true, true, true, true, false, true, false, true,
    true, true, true,
];

pub const SENSOR_ROW5: [bool; COL] = [
    true, true, true, true, false, false, true, false, true, true, true, true, true, false, true, true, true, true,
    true, false, false,
];
