use core::hint::unlikely;
use cortex_m::asm::delay;
use embassy_stm32::gpio::Output;

/// Column selector driven by an HC164 shift register.
pub struct Hc164Cols<'peripherals> {
    /// Clock input (`CP`) for shifting data into the register.
    cp: Output<'peripherals>,
    /// Serial data input (`DS`) for the shift register.
    ds: Output<'peripherals>,
    /// Master reset (`MR`) input for clearing the register.
    mr: Output<'peripherals>,
    /// CPU cycles between pin transitions.
    shifter_delay_cycles: u32,
}

impl<'peripherals> Hc164Cols<'peripherals> {
    /// Create a new column selector for the HC164.
    pub const fn new(
        ds: Output<'peripherals>,
        cp: Output<'peripherals>,
        mr: Output<'peripherals>,
        shifter_delay_cycles: u32,
    ) -> Self {
        Self { cp, ds, mr, shifter_delay_cycles }
    }

    /// Pulse the clock pin for one shift step.
    fn pulse_cp(&mut self) {
        self.cp.set_high();
        delay(self.shifter_delay_cycles);
        self.cp.set_low();
        delay(self.shifter_delay_cycles);
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
            // Clear all outputs by pulsing MR low.
            self.mr.set_low();
            delay(self.shifter_delay_cycles);
            self.mr.set_high();
            delay(self.shifter_delay_cycles);

            // Clock in the walking-one at position 0.
            self.ds.set_high();
            delay(self.shifter_delay_cycles);
            self.pulse_cp();
            self.ds.set_low();
        } else {
            // Clock the walking-one to the next position.
            self.pulse_cp();
        }
    }
}
