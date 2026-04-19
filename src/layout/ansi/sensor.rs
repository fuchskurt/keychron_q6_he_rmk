use crate::layout::COL;
pub use crate::layout::shared::sensor::{SENSOR_ROW0, SENSOR_ROW1, SENSOR_ROW2, SENSOR_ROW5};

// Row 3: cols 13–16 = a!(No); col 20 = KpPlus spans rows 2–3.
pub const SENSOR_ROW3: [bool; COL] = [
    true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, true,
    true, true, false,
];
// Row 4: col 1 = LShift 2.25U, col 12 = RShift 2.75U, cols 14/16 = Up arrow
// gaps.
pub const SENSOR_ROW4: [bool; COL] = [
    true, false, true, true, true, true, true, true, true, true, true, true, false, true, false, true, false, true,
    true, true, true,
];
