// encoder_switch.rs
use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

pub struct EncoderSwitch<'d> {
    pin: ExtiInput<'d>,
    row: u8,
    col: u8,
    last_pressed: bool,
    debounce: Duration,
}

impl<'d> EncoderSwitch<'d> {
    pub fn new(pin: ExtiInput<'d>, row: u8, col: u8) -> Self {
        Self { pin, row, col, last_pressed: false, debounce: Duration::from_millis(5) }
    }

    #[inline]
    fn sample_pressed(&self) -> bool {
        // Pull-up in hardware/config: pressed reads LOW
        self.pin.is_low()
    }
}

impl<'d> InputDevice for EncoderSwitch<'d> {
    async fn read_event(&mut self) -> Event {
        loop {
            // Wait for *either* edge, then debounce and sample state.
            // ExtiInput supports waiting for edges asynchronously.
            self.pin.wait_for_any_edge().await;

            // Debounce: wait a bit and sample the stable level
            Timer::after(self.debounce).await;
            let pressed = self.sample_pressed();

            if pressed != self.last_pressed {
                self.last_pressed = pressed;
                return Event::Key(KeyboardEvent::key(self.row, self.col, pressed));
            }

            // If it was bounce/no-change, loop and wait for next edge.
        }
    }
}
