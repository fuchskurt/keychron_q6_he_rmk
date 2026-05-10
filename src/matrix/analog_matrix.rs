//! Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
//! and continuous auto-calibration.

mod calibration;
mod lut;
mod scan;
pub mod types;

pub(crate) use crate::matrix::analog_matrix::types::AdcSampleTime;
use crate::{
    eeprom::Ft24c64,
    matrix::{
        analog_matrix::types::KeyEntry,
        calib_store::{CALIB_BUF_LEN, EEPROM_BASE_ADDR, try_deserialize},
        hc164_cols::Hc164Cols,
    },
};
use core::array::from_fn;
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, BasicInstance, Instance, RxDma},
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
    ) -> embassy_stm32::adc::ConfiguredSequence<'reader, adc::Adc>
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
    /// Per-key runtime state, calibration, and auto-calibration stored
    /// column-major for cache efficiency during column scans: all ROW entries
    /// for one HC164 column are contiguous, fitting in at most three 64-byte
    /// cache lines.
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
    /// Recompute [`KeyEntry::calib_used`] and the hot-path calibration fields
    /// for every key from the freshly measured zero-travel readings in
    /// `zero_raw`, using the full-travel value stored in each
    /// [`KeyEntry::entry_full`].
    ///
    /// Called after EEPROM load and after first-boot calibration to make the
    /// scan loop hot path purely arithmetic.
    fn apply_calib(keys: &mut [[KeyEntry; ROW]; COL], zero_raw: &[[u16; COL]; ROW]) {
        for (col, key_col) in keys.iter_mut().enumerate() {
            for (key, zero_row) in key_col.iter_mut().zip(zero_raw.iter()) {
                if let Some(&zero) = zero_row.get(col) {
                    key.apply_zero(zero);
                }
            }
        }
    }

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
            // Re-measure zero travel on every boot to compensate for
            // temperature drift; full-travel data comes from EEPROM.
            let zero_raw = Self::calibrate_zero_raw(&mut self.cols, &mut seq, &mut buf, self.cfg).await;
            Self::apply_calib(&mut self.keys, &zero_raw);
        } else {
            Self::run_first_boot_calib(
                &mut self.cols,
                &mut seq,
                &mut buf,
                self.cfg,
                &mut self.eeprom,
                &mut self.crc,
                &mut self.keys,
                &mut eeprom_buf,
            )
            .await;
        }

        Self::run_scan_loop(&mut self.cols, &mut self.keys, &mut seq, &mut buf, self.cfg).await;
    }
}
