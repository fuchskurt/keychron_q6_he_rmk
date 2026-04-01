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

use core::hint::unlikely;
use cortex_m::asm::delay;
use embassy_stm32::gpio::{Flex, Pull, Speed};

/// Cycle count for each bit-level delay inserted around clock transitions.
///
/// The HC164 requires tsu(DS) ≥ 20 ns and th(DS) ≥ 5 ns before/after the
/// CP rising edge. At 84 MHz, 4 cycles ≈ 48 ns.
const BIT_DELAY_CYCLES: u32 = 4;

/// Column selector driven by an HC164 shift register.
pub struct Hc164Cols<'peripherals> {
    /// Clock input (`CP`) for shifting data into the register.
    cp: Flex<'peripherals>,
    /// Serial data input (`DS`) for the shift register.
    ds: Flex<'peripherals>,
    /// Master reset (`MR`) input for clearing the register.
    mr: Flex<'peripherals>,
}

impl<'peripherals> Hc164Cols<'peripherals> {

    /// Float all shift-register pins as inputs with pull-down.
    pub fn enter_low_power(&mut self) {
        self.ds.set_as_input(Pull::Down);
        self.cp.set_as_input(Pull::Down);
        self.mr.set_as_input(Pull::Down);
    }

    /// Restore all shift-register pins as outputs (normal scan state).
    pub fn exit_low_power(&mut self) {
        self.ds.set_low();
        self.ds.set_as_output(Speed::VeryHigh);
        self.cp.set_low();
        self.cp.set_as_output(Speed::VeryHigh);
        self.mr.set_low();
        self.mr.set_as_output(Speed::VeryHigh);
    }

    /// Create a new column selector for the HC164.
    pub const fn new(ds: Flex<'peripherals>, cp: Flex<'peripherals>, mr: Flex<'peripherals>) -> Self {
        Self { ds, cp, mr }
    }

    /// Pulse the clock pin for one shift step.
    fn pulse_cp(&mut self) {
        self.cp.set_high();
        delay(BIT_DELAY_CYCLES);
        self.cp.set_low();
        delay(BIT_DELAY_CYCLES);
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
        if unlikely(col == 0) {
            // Clear all outputs by pulsing MR low. The HC164 datasheet
            // specifies a minimum MR low pulse width of 18 ns
            self.mr.set_low();
            delay(BIT_DELAY_CYCLES);
            self.mr.set_high();
            delay(BIT_DELAY_CYCLES);

            // Clock in the walking-one at position 0.
            self.ds.set_high();
            delay(BIT_DELAY_CYCLES);
            self.pulse_cp();
            self.ds.set_low();
        } else {
            // Shift the walking-one to the next position.
            self.ds.set_low();
            delay(BIT_DELAY_CYCLES);
            self.pulse_cp();
        }
    }
}
