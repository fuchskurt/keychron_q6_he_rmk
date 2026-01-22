//! Encoder switch input device handling.

use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

/// Debounced encoder switch exposed as an input device.
pub struct EncoderSwitch<'d> {
    pin: ExtiInput<'d>,
    row: u8,
    col: u8,
    last_pressed: bool,
    debounce: Duration,
}

impl<'d> EncoderSwitch<'d> {
    /// Create a new encoder switch input wrapper.
    pub fn new(pin: ExtiInput<'d>, row: u8, col: u8) -> Self {
        Self { pin, row, col, last_pressed: false, debounce: Duration::from_millis(5) }
    }

    #[inline]
    /// Sample the switch state.
    fn sample_pressed(&self) -> bool { self.pin.is_low() }
}

impl<'d> InputDevice for EncoderSwitch<'d> {
    /// Wait for the next debounced encoder switch event.
    async fn read_event(&mut self) -> Event {
        loop {
            self.pin.wait_for_any_edge().await;
            Timer::after(self.debounce).await;
            let pressed = self.sample_pressed();

            if pressed != self.last_pressed {
                self.last_pressed = pressed;
                return Event::Key(KeyboardEvent::key(self.row, self.col, pressed));
            }
        }
    }
}
