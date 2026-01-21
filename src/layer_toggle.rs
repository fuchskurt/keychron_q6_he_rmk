use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

#[derive(Copy, Clone)]
pub struct MatrixPos {
    pub row: u8,
    pub col: u8,
}

pub struct LayerToggle<'d> {
    pin: ExtiInput<'d>,
    high_pos: MatrixPos,
    low_pos: MatrixPos,

    last_level: Option<bool>,
    pending_release: Option<MatrixPos>,
    debounce: Duration,
}

impl<'d> LayerToggle<'d> {
    pub fn new(pin: ExtiInput<'d>, high_pos: MatrixPos, low_pos: MatrixPos, debounce: Duration) -> Self {
        Self { pin, high_pos, low_pos, last_level: None, pending_release: None, debounce }
    }

    pub fn new_with_default_debounce(pin: ExtiInput<'d>, high_pos: MatrixPos, low_pos: MatrixPos) -> Self {
        Self::new(pin, high_pos, low_pos, Duration::from_micros(20))
    }

    #[inline]
    fn pos_for_level(&self, level_high: bool) -> MatrixPos { if level_high { self.high_pos } else { self.low_pos } }

    #[inline]
    fn queue_tap(&mut self, pos: MatrixPos) -> Event {
        self.pending_release = Some(pos);
        Event::Key(KeyboardEvent::key(pos.row, pos.col, true))
    }

    async fn read_level_debounced(&mut self) -> bool {
        if self.debounce != Duration::from_millis(0) {
            Timer::after(self.debounce).await;
        }
        self.pin.is_high()
    }

    fn maybe_emit_for_level(&mut self, new_level: bool) -> Option<Event> {
        if self.last_level == Some(new_level) {
            return None;
        }
        self.last_level = Some(new_level);
        Some(self.queue_tap(self.pos_for_level(new_level)))
    }
}

impl<'d> InputDevice for LayerToggle<'d> {
    async fn read_event(&mut self) -> Event {
        if let Some(pos) = self.pending_release.take() {
            return Event::Key(KeyboardEvent::key(pos.row, pos.col, false));
        }

        if self.last_level.is_none() {
            let level = self.pin.is_high();
            self.last_level = Some(level);
            return self.queue_tap(self.pos_for_level(level));
        }

        loop {
            self.pin.wait_for_any_edge().await;
            let level = self.read_level_debounced().await;

            if let Some(evt) = self.maybe_emit_for_level(level) {
                return evt;
            }
        }
    }
}
