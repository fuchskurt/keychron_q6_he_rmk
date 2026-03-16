use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};
use rmk::{event::KeyboardEvent, macros::input_device};

/// Debounced encoder switch exposed as an input device.
#[input_device(publish = KeyboardEvent)]
pub struct EncoderSwitch<'peripherals> {
    /// Logical column index reported to the input system.
    col: u8,
    /// Debounce duration applied to state changes.
    debounce: Duration,
    /// Last debounced logical state of the switch.
    last_pressed: bool,
    /// External interrupt–backed input pin for the switch.
    pin: ExtiInput<'peripherals>,
    /// Logical row index reported to the input system.
    row: u8,
}

impl<'peripherals> EncoderSwitch<'peripherals> {
    /// Create a new encoder switch input wrapper.
    pub const fn new(pin: ExtiInput<'peripherals>, row: u8, col: u8) -> Self {
        Self { pin, row, col, last_pressed: false, debounce: Duration::from_millis(1) }
    }

    /// Wait for the next debounced encoder switch event.
    async fn read_keyboard_event(&mut self) -> KeyboardEvent {
        loop {
            self.pin.wait_for_any_edge().await;
            Timer::after(self.debounce).await;

            let pressed = self.sample_pressed();
            if pressed != self.last_pressed {
                self.last_pressed = pressed;
                return KeyboardEvent::key(self.row, self.col, pressed);
            }
        }
    }

    #[inline]
    /// Sample the switch state.
    fn sample_pressed(&self) -> bool { self.pin.is_low() }
}
