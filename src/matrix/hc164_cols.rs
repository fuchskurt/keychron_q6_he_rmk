//! HC164 shift-register column selector.

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

/// Column selector driven by an HC164 shift register.
pub struct Hc164Cols<'peripherals> {
    /// Delay inserted between bit transitions.
    bit_delay: Duration,
    /// Clock input (`CP`) for shifting data into the register.
    cp: Output<'peripherals>,
    /// Serial data input (`DS`) for the shift register.
    ds: Output<'peripherals>,
    /// Master reset (`MR`) input for clearing the register.
    mr: Output<'peripherals>,
}

impl<'peripherals> Hc164Cols<'peripherals> {
    /// Advance to the next column.
    pub async fn advance(&mut self) {
        self.ds.set_low();
        Timer::after(self.bit_delay).await;
        self.pulse_cp().await;
    }

    /// Create a new column selector for the HC164.
    pub const fn new(
        ds: Output<'peripherals>,
        cp: Output<'peripherals>,
        mr: Output<'peripherals>,
    ) -> Self {
        Self { ds, cp, mr, bit_delay: Duration::from_micros(1) }
    }

    /// Pulse the clock pin for one shift step.
    async fn pulse_cp(&mut self) {
        self.cp.set_high();
        Timer::after(self.bit_delay).await;
        self.cp.set_low();
        Timer::after(self.bit_delay).await;
    }

    /// Reset the shift register to the initial state.
    async fn reset(&mut self) {
        self.mr.set_low();
        Timer::after(Duration::from_micros(2)).await;
        self.mr.set_high();
        Timer::after(Duration::from_micros(2)).await;
    }

    /// Select a specific column, resetting when column zero is requested.
    pub async fn select(&mut self, col: usize) {
        if col == 0 {
            self.reset().await;
            self.ds.set_high();
            Timer::after(self.bit_delay).await;
            self.pulse_cp().await;
            self.ds.set_low();
        }
    }
}
