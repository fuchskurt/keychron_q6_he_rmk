use crate::layout::COL;
pub use crate::layout::shared::sensor::{SENSOR_ROW0, SENSOR_ROW1, SENSOR_ROW2, SENSOR_ROW5};

pub const SENSOR_ROW3: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, true,
    true, true, false,
];

pub const SENSOR_ROW4: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, false, true, false, true, false, true,
    true, true, true,
];
