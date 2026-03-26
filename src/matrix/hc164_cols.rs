//! HC164 shift-register column selector.
//!
//! The HC164 is driven as a walking-ones shift register. Column 0 is selected
//! by pulsing MR low (clearing the register) then clocking in a single high
//! bit. Each subsequent column is selected by clocking in a low bit, which
//! shifts the high bit one position along the chain.
//!
//! All column transitions go through
//! [`crate::matrix::hc164_cols::Hc164Cols::select`], which tracks the
//! last column and handles both cases transparently. The scan loop only needs
//! to call `select(col)` — there is no separate `advance` step.

use cortex_m::asm::delay;
use embassy_stm32::gpio::Output;

/// Column selector driven by an HC164 shift register.
pub struct Hc164Cols<'peripherals> {
    /// Delay inserted between bit transitions.
    /// The HC164 requires tsu(DS) ≥ 20ns and th(DS) ≥ 5ns before/after
    /// the CP rising edge. 1µs satisfies this with substantial margin.
    bit_delay_cycles: u32,
    /// Clock input (`CP`) for shifting data into the register.
    cp: Output<'peripherals>,
    /// Serial data input (`DS`) for the shift register.
    ds: Output<'peripherals>,
    /// Master reset (`MR`) input for clearing the register.
    mr: Output<'peripherals>,
}

impl<'peripherals> Hc164Cols<'peripherals> {
    /// Create a new column selector for the HC164.
    pub const fn new(ds: Output<'peripherals>, cp: Output<'peripherals>, mr: Output<'peripherals>) -> Self {
        Self { ds, cp, mr, bit_delay_cycles: 4 }
    }

    /// Pulse the clock pin for one shift step.
    fn pulse_cp(&mut self) {
        self.cp.set_high();
        delay(self.bit_delay_cycles);
        self.cp.set_low();
        delay(self.bit_delay_cycles);
    }

    /// Select the given column.
    ///
    /// For column 0: clears the register via MR then clocks in a high bit,
    /// placing the walking-one at position 0.
    ///
    /// For all other columns: clocks in a low bit, advancing the walking-one
    /// to the next position. Must be called in strictly ascending order
    /// starting from 0 each scan.
    pub fn select(&mut self, col: usize) {
        if col == 0 {
            // Clear all outputs by pulsing MR low. The HC164 datasheet
            // specifies a minimum MR low pulse width of 18 ns
            self.mr.set_low();
            delay(self.bit_delay_cycles);
            self.mr.set_high();
            delay(self.bit_delay_cycles);

            // Clock in the walking-one at position 0.
            self.ds.set_high();
            delay(self.bit_delay_cycles);
            self.pulse_cp();
            self.ds.set_low();
        } else {
            // Shift the walking-one to the next position.
            self.ds.set_low();
            delay(self.bit_delay_cycles);
            self.pulse_cp();
        }
    }
}
