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

use embassy_stm32::{exti::ExtiInput, mode::Async};
use embassy_time::{Duration, Timer};

/// Default debounce window shared by the discrete switch inputs (encoder
/// switch and layer toggle).
pub const INPUT_DEBOUNCE: Duration = Duration::from_millis(15);

/// Wait for the next edge on `pin`, sleep out the `debounce` window, then
/// sample and return the settled logic level (`true` = high).
///
/// The edge -> settle -> sample debounce protocol shared by the discrete
/// switch inputs, kept in one place so it cannot drift between them. Callers
/// loop until the settled level differs from their last recorded state,
/// which also absorbs bounce trains that settle back at the old level.
pub async fn debounced_level(pin: &mut ExtiInput<'_, Async>, debounce: Duration) -> bool {
    pin.wait_for_any_edge().await;
    Timer::after(debounce).await;
    pin.is_high()
}
