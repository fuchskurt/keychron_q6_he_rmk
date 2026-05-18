use embassy_stm32::gpio::Output;

/// Column selector driven by an HC164 shift register.
pub struct Hc164Cols<'peripherals> {
    /// Clock input (`CP`) for shifting data into the register.
    cp: Output<'peripherals>,
    /// Serial data input (`DS`) for the shift register.
    ds: Output<'peripherals>,
    /// Master reset (`MR`) input for clearing the register.
    mr: Output<'peripherals>,
}

impl<'peripherals> Hc164Cols<'peripherals> {
    /// Advance the walking-one to the next column.
    /// Call this for every column after the first.
    #[inline]
    pub fn advance(&mut self) {
        self.cp.set_high();
        self.cp.set_low();
    }

    /// Create a new column selector for the HC164.
    pub const fn new(ds: Output<'peripherals>, cp: Output<'peripherals>, mr: Output<'peripherals>) -> Self {
        Self { cp, ds, mr }
    }

    /// Reset the register and clock in the walking-one at position 0.
    /// Call this before the first column of every scan.
    #[inline]
    pub fn reset(&mut self) {
        self.mr.set_low();
        self.mr.set_high();
        self.ds.set_high();
        self.advance();
        self.ds.set_low();
    }
}
