// hc164_cols.rs
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

pub struct Hc164Cols<'d> {
    ds: Output<'d>,
    cp: Output<'d>,
    mr: Output<'d>,
    // QMK uses a small NOP delay; we approximate with a tiny async delay
    bit_delay: Duration,
}

impl<'d> Hc164Cols<'d> {
    pub fn new(ds: Output<'d>, cp: Output<'d>, mr: Output<'d>) -> Self {
        Self { ds, cp, mr, bit_delay: Duration::from_micros(1) }
    }

    async fn pulse_cp(&mut self) {
        self.cp.set_high();
        Timer::after(self.bit_delay).await;
        self.cp.set_low();
        Timer::after(self.bit_delay).await;
    }

    async fn reset(&mut self) {
        // MR is active-low in QMK code (they drive low then high)
        self.mr.set_low();
        Timer::after(Duration::from_micros(2)).await;
        self.mr.set_high();
        Timer::after(Duration::from_micros(2)).await;
    }

    /// Equivalent of QMK "select_col()":
    /// only does something for col == 0 (reset + shift in 1 once)
    pub async fn select(&mut self, col: usize) {
        if col == 0 {
            self.reset().await;
            // shift in a single '1'
            self.ds.set_high();
            Timer::after(self.bit_delay).await;
            self.pulse_cp().await;
            self.ds.set_low();
        }
        // for col > 0, do nothing: the "active 1" is already positioned
        // by the previous column's advance() call.
    }

    /// Equivalent of QMK "unselect_col()": shift in a single '0' once.
    /// This advances the active column by one.
    pub async fn advance(&mut self) {
        self.ds.set_low();
        Timer::after(self.bit_delay).await;
        self.pulse_cp().await;
    }

    pub async fn unselect_all(&mut self) {
        // Optional: you can also reset to a known state.
        // QMK does not strictly do this; it advances each column.
        self.ds.set_low();
    }
}
