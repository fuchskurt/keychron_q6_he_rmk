use embassy_stm32::{exti::ExtiInput, mode::Async};
use embassy_time::{Duration, Timer};
use rmk::{event::KeyboardEvent, macros::input_device};

/// Matrix coordinates for a key position.
#[derive(Copy, Clone)]
pub struct MatrixPos {
    /// Column index within the matrix.
    pub col: u8,
    /// Row index within the matrix.
    pub row: u8,
}

/// Input device that toggles layers based on a switch position.
#[input_device(publish = KeyboardEvent)]
pub struct LayerToggle<'peripherals> {
    /// Debounce duration applied to input level changes.
    debounce:        Duration,
    /// Matrix position activated when the input is at a high logic level.
    high_pos:        MatrixPos,
    /// Last observed logic level of the input pin.
    last_level:      Option<bool>,
    /// Matrix position activated when the input is at a low logic level.
    low_pos:         MatrixPos,
    /// Matrix position pending release after a level change.
    pending_release: Option<MatrixPos>,
    /// External interrupt input pin used to read the switch state.
    pin:             ExtiInput<'peripherals, Async>,
}

impl<'peripherals> LayerToggle<'peripherals> {
    /// Emit an event if the level has changed.
    fn maybe_emit_for_level(&mut self, new_level: bool) -> Option<KeyboardEvent> {
        if self.last_level == Some(new_level) {
            return None;
        }
        self.last_level = Some(new_level);
        Some(self.queue_tap(self.pos_for_level(new_level)))
    }

    /// Create a new layer toggle with a provided debounce duration.
    #[must_use]
    pub const fn new(
        pin: ExtiInput<'peripherals, Async>,
        high_pos: MatrixPos,
        low_pos: MatrixPos,
        debounce: Duration,
    ) -> Self {
        Self { pin, high_pos, low_pos, last_level: None, pending_release: None, debounce }
    }

    /// Create a new layer toggle with the default debounce.
    #[must_use]
    pub const fn new_with_default_debounce(
        pin: ExtiInput<'peripherals, Async>,
        high_pos: MatrixPos,
        low_pos: MatrixPos,
    ) -> Self {
        Self::new(pin, high_pos, low_pos, Duration::from_millis(15))
    }

    /// Select the matrix position for the provided level.
    #[inline]
    const fn pos_for_level(&self, level_high: bool) -> MatrixPos {
        if level_high { self.high_pos } else { self.low_pos }
    }

    /// Queue a tap event for the provided position.
    #[inline]
    fn queue_tap(&mut self, pos: MatrixPos) -> KeyboardEvent {
        self.pending_release = Some(pos);
        KeyboardEvent::key(pos.row, pos.col, true)
    }

    /// Read the next layer-toggle `KeyboardEvent`.
    async fn read_keyboard_event(&mut self) -> KeyboardEvent {
        if let Some(pos) = self.pending_release.take() {
            return KeyboardEvent::key(pos.row, pos.col, false);
        }

        if self.last_level.is_none() {
            let level = self.pin.is_high();
            self.last_level = Some(level);
            return self.queue_tap(self.pos_for_level(level));
        }

        loop {
            self.pin.wait_for_any_edge().await;
            Timer::after(self.debounce).await;
            let level = self.pin.is_high();

            if let Some(evt) = self.maybe_emit_for_level(level) {
                return evt;
            }
        }
    }
}
