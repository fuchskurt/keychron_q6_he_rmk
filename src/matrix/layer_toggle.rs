//! Layer toggle input device handling.

use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

#[derive(Copy, Clone)]
/// Matrix coordinates for a key position.
pub struct MatrixPos {
    /// Column index within the matrix.
    pub col: u8,
    /// Row index within the matrix.
    pub row: u8,
}

/// Input device that toggles layers based on a switch position.
pub struct LayerToggle<'peripherals> {
    /// Debounce duration applied to input level changes.
    debounce: Duration,
    /// Matrix position activated when the input is at a high logic level.
    high_pos: MatrixPos,
    /// Last observed logic level of the input pin.
    last_level: Option<bool>,
    /// Matrix position activated when the input is at a low logic level.
    low_pos: MatrixPos,
    /// Matrix position pending release after a level change.
    pending_release: Option<MatrixPos>,
    /// External interrupt input pin used to read the switch state.
    pin: ExtiInput<'peripherals>,
}

impl<'peripherals> LayerToggle<'peripherals> {
    /// Emit an event if the level has changed.
    fn maybe_emit_for_level(&mut self, new_level: bool) -> Option<Event> {
        if self.last_level == Some(new_level) {
            return None;
        }
        self.last_level = Some(new_level);
        Some(self.queue_tap(self.pos_for_level(new_level)))
    }

    /// Create a new layer toggle with a provided debounce duration.
    pub const fn new(
        pin: ExtiInput<'peripherals>,
        high_pos: MatrixPos,
        low_pos: MatrixPos,
        debounce: Duration,
    ) -> Self {
        Self { pin, high_pos, low_pos, last_level: None, pending_release: None, debounce }
    }

    /// Create a new layer toggle with the default debounce.
    pub const fn new_with_default_debounce(
        pin: ExtiInput<'peripherals>,
        high_pos: MatrixPos,
        low_pos: MatrixPos,
    ) -> Self {
        Self::new(pin, high_pos, low_pos, Duration::from_micros(20))
    }

    #[inline]
    /// Select the matrix position for the provided level.
    const fn pos_for_level(&self, level_high: bool) -> MatrixPos {
        if level_high { self.high_pos } else { self.low_pos }
    }

    #[inline]
    /// Queue a tap event for the provided position.
    fn queue_tap(&mut self, pos: MatrixPos) -> Event {
        self.pending_release = Some(pos);
        Event::Key(KeyboardEvent::key(pos.row, pos.col, true))
    }

    /// Read the switch level after a debounce delay.
    async fn read_level_debounced(&self) -> bool {
        if self.debounce != Duration::from_millis(0) {
            Timer::after(self.debounce).await;
        }
        self.pin.is_high()
    }
}

impl InputDevice for LayerToggle<'_> {
    /// Read the next layer toggle event.
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
