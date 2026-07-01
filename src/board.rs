//! Board-specific hardware description for the Keychron Q6 HE.
//!
//! Everything that is a fixed property of this PCB rather than firmware
//! behavior lives here: the analog row pin set and its suspend parking,
//! the STM32F401 clock tree, and the flash/ADC register tweaks applied at
//! boot. `main` does the task wiring; this module describes the board.

use crate::{layout::ROW, matrix::analog_matrix::RowChannels};
use core::mem::ManuallyDrop;
use embassy_stm32::{
    Config,
    Peri,
    adc::{AdcChannel as _, BorrowedAdcChannel},
    gpio::{Flex, Pin, Pull},
    pac,
    peripherals::{self, ADC1},
    rcc::{
        AHBPrescaler,
        APBPrescaler,
        Hse,
        HseMode,
        Pll,
        PllMul,
        PllPDiv,
        PllPreDiv,
        PllQDiv,
        PllSource,
        Sysclk,
        mux::Clk48sel,
    },
    time::Hertz,
};
use pac::{ADC1_COMMON, SYSCFG, adccommon::vals::Adcpre};

/// Owns the six analog row pins of the Q6 HE matrix.
///
/// The current embassy ADC API exposes channels only as transient
/// [`BorrowedAdcChannel`] borrows of an owned pin (the owned, type-erased
/// `AnyAdcChannel` was removed upstream), so the matrix scanner cannot store
/// pre-erased channels. This holder keeps the concrete pins alive and
/// re-borrows them on demand through [`RowChannels`].
pub struct Q6RowPins<'peripherals> {
    /// Row 4 analog input (`ADC1_IN0`).
    pub pa0: Peri<'peripherals, peripherals::PA0>,
    /// Row 5 analog input (`ADC1_IN1`).
    pub pa1: Peri<'peripherals, peripherals::PA1>,
    /// Row 0 analog input (`ADC1_IN10`).
    pub pc0: Peri<'peripherals, peripherals::PC0>,
    /// Row 1 analog input (`ADC1_IN11`).
    pub pc1: Peri<'peripherals, peripherals::PC1>,
    /// Row 2 analog input (`ADC1_IN12`).
    pub pc2: Peri<'peripherals, peripherals::PC2>,
    /// Row 3 analog input (`ADC1_IN13`).
    pub pc3: Peri<'peripherals, peripherals::PC3>,
}

impl RowChannels<ADC1, ROW> for Q6RowPins<'_> {
    fn reborrow_channels(&mut self) -> [BorrowedAdcChannel<'_, ADC1>; ROW] {
        [
            self.pc0.reborrow_adc(),
            self.pc1.reborrow_adc(),
            self.pc2.reborrow_adc(),
            self.pc3.reborrow_adc(),
            self.pa0.reborrow_adc(),
            self.pa1.reborrow_adc(),
        ]
    }

    fn set_active(&mut self) {
        release_row_pull(&mut self.pc0);
        release_row_pull(&mut self.pc1);
        release_row_pull(&mut self.pc2);
        release_row_pull(&mut self.pc3);
        release_row_pull(&mut self.pa0);
        release_row_pull(&mut self.pa1);
    }

    fn set_low_power(&mut self) {
        park_row_low(&mut self.pc0);
        park_row_low(&mut self.pc1);
        park_row_low(&mut self.pc2);
        park_row_low(&mut self.pc3);
        park_row_low(&mut self.pa0);
        park_row_low(&mut self.pa1);
    }
}

/// Drive one row pin to input-pull-down and leave it there.
///
/// A [`Flex`] resets its pin to disconnected (floating analog) on drop, so it
/// is wrapped in [`ManuallyDrop`] to persist the pull-down through suspend.
/// Nothing is leaked but the register state; the pin is reclaimed by
/// `reborrow_adc` (analog) or [`release_row_pull`] (no-pull) on resume.
fn park_row_low<P>(pin: &mut Peri<'_, P>)
where
    P: Pin,
{
    let mut flex = ManuallyDrop::new(Flex::new(pin.reborrow()));
    flex.set_as_input(Pull::Down);
}

/// Return one row pin to a no-pull input, clearing a suspend pull-down before
/// the ADC re-asserts analog mode (which only sets the mode register and would
/// otherwise leave the pull-down in place, biasing the reading).
fn release_row_pull<P>(pin: &mut Peri<'_, P>)
where
    P: Pin,
{
    let mut flex = ManuallyDrop::new(Flex::new(pin.reborrow()));
    flex.set_as_input(Pull::None);
}

/// Build the STM32F401 clock and bus configuration used at boot.
///
/// HSE 16 MHz -> PLL (/8 x168 -> 336 MHz VCO) -> SYSCLK 84 MHz via /P=4 and
/// USB clock 48 MHz via /Q=7. AHB runs at full SYSCLK; APB1 at /2 (42 MHz)
/// for the slower peripherals; APB2 at full speed for ADC and SPI.
pub fn stm32_config() -> Config {
    let mut config = Config::default();
    config.rcc.hse = Some(Hse { freq: Hertz(16_000_000), mode: HseMode::Oscillator });
    config.rcc.hsi = false;
    config.rcc.pll_src = PllSource::Hse;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::Div8,     // 16/8 = 2 MHz
        mul:    PllMul::Mul168,      // 2*168 = 336 MHz (VCO)
        divp:   Some(PllPDiv::Div4), // 336/4 = 84 MHz (SYSCLK)
        divq:   Some(PllQDiv::Div7), // 336/7 = 48 MHz
        divr:   None,
    });
    config.rcc.ahb_pre = AHBPrescaler::Div1; // 84 MHz
    config.rcc.apb1_pre = APBPrescaler::Div2; // 42 MHz
    config.rcc.apb2_pre = APBPrescaler::Div1; // 84 MHz
    config.rcc.sys = Sysclk::Pll1P;
    config.rcc.mux.clk48sel = Clk48sel::Pll1Q;
    config
}

/// Enable the FLASH instruction cache, data cache, and prefetch buffer to
/// keep wait-stated flash reads off the hot path.
pub fn enable_flash_acceleration() {
    pac::FLASH.acr().modify(|w| {
        w.set_icen(true);
        w.set_dcen(true);
        w.set_prften(true);
    });
}

/// Apply the board's ADC clocking and noise tweaks.
///
/// Sets the ADC prescaler to /2 (42 MHz, overclocked from the 36 MHz spec)
/// and enables the STM32 AN4073 Option 2 workaround (`ADC1DC2`) to reduce
/// ADC noise coupling from USB activity on the STM32F401.
pub fn tune_adc() {
    ADC1_COMMON.ccr().modify(|w| w.set_adcpre(Adcpre::Div2));
    SYSCFG.pmc().modify(|w| w.set_adc1dc2(true));
}
