//! Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
//! and continuous auto-calibration.

/// First-boot guided calibration and EEPROM persistence.
mod calibration;
/// Hot-path matrix scan loop.
mod scan;
/// Calibration types, constants, and per-key runtime state.
pub mod types;

use crate::{
    eeprom::Ft24c64,
    matrix::{
        analog_matrix::types::{AdcSampleTime, KeyEntry},
        calib_store::{CALIB_BUF_LEN, EEPROM_BASE_ADDR, try_deserialize},
        hc164_cols::Hc164Cols,
    },
};
use core::array::from_fn;
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, BasicInstance, ConfiguredSequence, Instance, RxDma},
    crc::Crc,
    dma::InterruptHandler,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    pac::adc,
};
use rmk::core_traits::Runnable;
pub use types::HallCfg;

/// ADC-related peripherals grouped to allow split borrows when constructing
/// a [`embassy_stm32::adc::ConfiguredSequence`].
pub struct AdcPart<'peripherals, ADC, D, const ROW: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripheral used for sampling hall sensors.
    pub adc:         Adc<'peripherals, ADC>,
    /// DMA channel used for non-blocking ADC sequence reads.
    pub dma:         Peri<'peripherals, D>,
    /// ADC channels corresponding to each matrix row.
    pub row_adc:     [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time applied to every channel in the sequence.
    pub sample_time: AdcSampleTime<ADC>,
}

impl<'peripherals, ADC, D, const ROW: usize> AdcPart<'peripherals, ADC, D, ROW>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a [`embassy_stm32::adc::ConfiguredSequence`] over all row
    /// channels.
    ///
    /// Programs the ADC sequence registers once; the returned reader can be
    /// triggered repeatedly without reprogramming.
    fn configured_sequence<'reader, IRQ2>(
        &'reader mut self,
        irq: IRQ2,
    ) -> ConfiguredSequence<'reader, adc::Adc>
    where
        IRQ2: Binding<D::Interrupt, InterruptHandler<D>> + 'reader + 'peripherals,
    {
        let st = self.sample_time;
        self.adc.configured_sequence(self.dma.reborrow(), self.row_adc.iter_mut().map(|ch| (ch, st)), irq)
    }

    /// Create a new [`AdcPart`] from the given peripherals.
    pub const fn new(
        adc: Adc<'peripherals, ADC>,
        row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
        dma: Peri<'peripherals, D>,
        sample_time: AdcSampleTime<ADC>,
    ) -> Self {
        Self { adc, dma, row_adc, sample_time }
    }
}

/// Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
/// and continuous auto-calibration.
///
/// On first boot (or after EEPROM corruption) the firmware performs a guided
/// two-phase calibration:
///
/// 1. **Zero-travel pass** - all keys fully released; the firmware averages
///    `HallCfg::calib_passes` reads per key. Backlight signals amber.
/// 2. **Full-travel pass** - user presses every key to the bottom within
///    `HallCfg::full_calib_duration`; each key must be held for
///    [`types::CALIB_HOLD_DURATION_MS`] before it is accepted and its LED turns
///    green.
///
/// After all keys are accepted a
/// [`types::CALIB_SETTLE_AFTER_ALL_DONE`]
/// window continues sampling to capture the true bottom-out ADC. Validated
/// entries are written to the FT24C64 EEPROM and verified by read-back. On all
/// subsequent boots only full-travel data is loaded from EEPROM; zero-travel
/// is re-measured fresh to compensate for temperature drift.
///
/// During normal operation the auto-calibrator silently refines both zero and
/// full-travel values on every press/release cycle, keeping the scanner
/// accurate as the sensor drifts over time without requiring user interaction.
pub struct AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripherals and channels grouped for split-borrow compatibility.
    adc_part: AdcPart<'peripherals, ADC, D, ROW>,
    /// Sensing and scanning configuration.
    cfg:      HallCfg,
    /// Column driver used to select the active column via the HC164.
    cols:     Hc164Cols<'peripherals>,
    /// Hardware CRC peripheral used for EEPROM calibration block checksums.
    crc:      Crc<'peripherals>,
    /// EEPROM driver for loading and persisting calibration data.
    eeprom:   Ft24c64<'peripherals, IM>,
    /// DMA interrupt binding reused for every ADC sequence read.
    irq:      IRQ,
    /// Per-key runtime state, calibration, and auto-calibration. Stored
    /// column-major (`[[KeyEntry; ROW]; COL]`) so the per-column inner scan
    /// loop walks one contiguous SRAM block per HC164 column, which the AHB
    /// load-store unit pipelines better than scattered indirect loads from a
    /// row-major layout.
    keys:     [[KeyEntry; ROW]; COL],
}

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a new matrix scanner.
    ///
    /// Calibration is deferred to [`Runnable::run`], which loads from EEPROM
    /// on subsequent boots or runs a full first-boot calibration pass.
    pub fn new(
        adc_part: AdcPart<'peripherals, ADC, D, ROW>,
        irq: IRQ,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
        eeprom: Ft24c64<'peripherals, IM>,
        crc: Crc<'peripherals>,
    ) -> Self {
        Self { adc_part, cfg, cols, crc, eeprom, irq, keys: from_fn(|_| from_fn(|_| KeyEntry::default())) }
    }
}

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize> Runnable
    for AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    async fn run(&mut self) -> ! {
        let mut eeprom_buf = [0_u8; CALIB_BUF_LEN];

        let loaded = self.eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
            && try_deserialize::<ROW, COL>(&eeprom_buf, &mut self.keys, &mut self.crc);
        let mut buf = [0_u16; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);
        if loaded {
            #[cfg(feature = "defmt")]
            defmt::info!("calibration loaded from EEPROM");
            // Re-measure zero travel on every boot to compensate for
            // temperature drift; full-travel data comes from EEPROM.
            let zero_raw = calibration::calibrate_zero_raw(&mut self.cols, &mut seq, &mut buf, self.cfg).await;
            calibration::apply_calib(&mut self.keys, &zero_raw);
        } else {
            #[cfg(feature = "defmt")]
            defmt::info!("no valid EEPROM calibration, running first-boot calibration");
            calibration::run_first_boot_calib(
                &mut self.cols,
                &mut seq,
                &mut buf,
                self.cfg,
                &mut self.eeprom,
                &mut self.crc,
                &mut self.keys,
            )
            .await;
        }

        scan::run_scan_loop(&mut self.cols, &mut self.keys, &mut seq, &mut buf, self.cfg).await;
    }
}
