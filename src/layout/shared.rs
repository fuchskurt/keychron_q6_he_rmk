#![cfg_attr(feature = "jis_layout", expect(dead_code, reason = "JIS layout only uses some of these constants"))]

/// Layout-specific backlight configuration modules.
pub mod layout;
/// LED driver initialization and runtime loop.
pub mod led;
/// Hall-effect sensor presence map.
pub mod sensor;
