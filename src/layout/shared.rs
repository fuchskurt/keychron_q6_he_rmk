#![cfg_attr(feature = "jis_layout", expect(dead_code, reason = "JIS layout only uses some of these constants"))]

/// Layout-specific backlight configuration modules.
pub mod layout;
/// LED driver initialization and runtime loop.
pub mod led;
/// Hall-effect sensor presence map.
pub mod sensor;

use crate::layout::{COL, ROW};

/// Compile-time guards: if `ROW` or `COL` in `keymap.rs` ever change, the
/// hardcoded dimensions of [`crate::layout::ansi::SENSOR_POSITIONS`]
/// must be updated to match.
const _: [(); ROW] = [(); 6];
const _: [(); COL] = [(); 21];
