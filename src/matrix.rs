//! Matrix scanning submodules.

/// Analog hall-effect matrix implementation.
pub mod analog_matrix;
/// Calibration EEPROM serialization.
pub mod calib_store;
/// Encoder switch input device.
pub mod encoder_switch;
/// HC164 column selector.
pub mod hc164_cols;
/// Layer toggle input handling.
pub mod layer_toggle;
/// Travel polynomial functions.
pub mod travel_helpers;
