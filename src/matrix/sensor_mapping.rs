use crate::keymap::{COL, ROW};

/// Compile-time guards: if `ROW` or `COL` in `keymap.rs` ever change, the
/// hardcoded dimensions of [`SENSOR_POSITIONS`] must be updated to match.
const _: [(); ROW] = [(); 6];
const _: [(); COL] = [(); 21];

/// Whether each matrix position has a physical hall-effect sensor.
///
/// Excluding these positions from calibration prevents their floating ADC
/// readings from crossing
/// [`crate::matrix::analog_matrix::CALIB_PRESS_THRESHOLD`] spuriously and
/// advancing `calibrated_count` to `total_keys` prematurely, which would cut
/// the full-travel window short before all real keys are pressed.
pub const SENSOR_POSITIONS: [[bool; COL]; ROW] = [
    // Row 0: col 13 = AudioMute mapped to the encoder button; no hall switch.
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, false, true, true, true, true,
        true, true, true,
    ],
    // Row 1: all 21 positions populated.
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ],
    // Row 2: all 21 positions populated.
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ],
    // Row 3: cols 13–16 = a!(No) (gap right of Enter + left nav cluster gap);
    //         col 20 = a!(No) (KpPlus spans rows 2–3, no second sensor).
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, true,
        true, true, false,
    ],
    // Row 4: col 1  = a!(No) (LShift 2.25U wide, no second sensor),
    //         col 12 = a!(No) (RShift 2.75U wide, no second sensor),
    //         col 14 = a!(No) (gap left of Up arrow),
    //         col 16 = a!(No) (gap right of Up arrow).
    [
        true, false, true, true, true, true, true, true, true, true, true, true, false, true, false, true, false, true,
        true, true, true,
    ],
    // Row 5: cols 3–5  = a!(No) (Space 6.25U wide, no additional sensors),
    //         cols 7–8  = df!(0)/df!(1) layer-toggle virtual keys (PB12 GPIO, no ADC),
    //         col  13   = a!(No) (gap right of RCtrl),
    //         cols 19–20 = a!(No) (KpEnter spans rows 4–5, no second sensor).
    [
        true, true, true, false, false, false, true, false, false, true, true, true, true, false, true, true, true,
        true, true, false, false,
    ],
];
