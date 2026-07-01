use embassy_stm32::gpio::{Flex, Pull, Speed};

/// Column selector driven by an HC164 shift register.
///
/// The register holds a walking-one: exactly one column line is high at a
/// time. Every scan pass must follow the same protocol, which both the
/// calibration passes and the hot scan loop rely on:
///
/// 1. [`Hc164Cols::reset`] once before the pass, leaving column 0 selected.
/// 2. Read the rows for the selected column (after a settle delay).
/// 3. [`Hc164Cols::advance`] once after each read to select the next column.
///
/// After `COL - 1` advances the walking-one has moved past the last column;
/// the next pass starts with a fresh [`Hc164Cols::reset`]. Reordering reads
/// and advances desynchronises the selected column from the loop index for
/// the remainder of the pass.
pub struct Hc164Cols<'peripherals> {
    /// Clock input (`CP`) for shifting data into the register.
    cp: Flex<'peripherals>,
    /// Serial data input (`DS`) for the shift register.
    ds: Flex<'peripherals>,
    /// Master reset (`MR`) input for clearing the register.
    mr: Flex<'peripherals>,
}

impl<'peripherals> Hc164Cols<'peripherals> {
    /// Advance the walking-one to the next column.
    /// Call this for every column after the first.
    #[inline]
    pub fn advance(&mut self) {
        self.cp.set_high();
        self.cp.set_low();
    }

    /// Clear the register so no column is selected (all outputs low).
    ///
    /// Used by [`Hc164Cols::reset`] at the start of every pass and by
    /// [`Hc164Cols::set_low_power`] when parking the register for USB
    /// suspend, so no sensor column is left driven while scanning is idle.
    #[inline]
    pub fn clear(&mut self) {
        self.mr.set_low();
        self.mr.set_high();
    }

    /// Create a new column selector for the HC164.
    pub fn new(mut ds: Flex<'peripherals>, mut cp: Flex<'peripherals>, mut mr: Flex<'peripherals>) -> Self {
        ds.set_as_output(Speed::VeryHigh);
        cp.set_as_output(Speed::VeryHigh);
        mr.set_as_output(Speed::VeryHigh);
        ds.set_low();
        cp.set_low();
        mr.set_low();
        Self { cp, ds, mr }
    }

    /// Reset the register and clock in the walking-one at position 0.
    /// Call this before the first column of every scan.
    #[inline]
    pub fn reset(&mut self) {
        self.clear();
        self.ds.set_high();
        self.advance();
        self.ds.set_low();
    }

    /// Restore the control lines to push-pull outputs after a resume.
    #[inline]
    pub fn set_active(&mut self) {
        self.ds.set_as_output(Speed::VeryHigh);
        self.cp.set_as_output(Speed::VeryHigh);
        self.mr.set_as_output(Speed::VeryHigh);
    }

    /// Park the control lines for suspend, mirroring the stock firmware's
    /// `matrix_enter_low_power`: clear the register, then tristate DS/CP/MR to
    /// input-pulldown so the MCU does not backfeed the unpowered HC164. Pair
    /// with [`Hc164Cols::set_active`] on resume.
    #[inline]
    pub fn set_low_power(&mut self) {
        self.clear();
        self.ds.set_as_input(Pull::Down);
        self.cp.set_as_input(Pull::Down);
        self.mr.set_as_input(Pull::Down);
    }
}
