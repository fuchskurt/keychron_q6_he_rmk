//! Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
//! and continuous auto-calibration.

mod calibration;
mod scan;
mod types;

pub(crate) use crate::matrix::analog_matrix::types::{AdcSampleTime, AutoCalib, KeyCalib, KeyState};
use crate::{
    eeprom::Ft24c64,
    matrix::{
        calib_store::{CALIB_BUF_LEN, CalibEntry, EEPROM_BASE_ADDR, try_deserialize},
        hc164_cols::Hc164Cols,
    },
};
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, BasicInstance, Instance, RxDma},
    crc::Crc,
    dma::InterruptHandler,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    pac::adc,
};
use rmk::input_device::Runnable;
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
    pub adc: Adc<'peripherals, ADC>,
    /// DMA channel used for non-blocking ADC sequence reads.
    pub dma: Peri<'peripherals, D>,
    /// ADC channels corresponding to each matrix row.
    pub row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
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
    /// triggered repeatedly without reprogramming, saving ~20 cycles per
    /// column per scan.
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
    /// Per-key auto-calibration state used to refine
    /// [`self::AnalogHallMatrix::calib`] during normal operation.
    auto_calib: [[AutoCalib; COL]; ROW],
    /// Per-key calibration data applied during the scan loop.
    calib: [[KeyCalib; COL]; ROW],
    /// Sensing and scanning configuration.
    cfg: HallCfg,
    /// Column driver used to select the active column via the HC164.
    cols: Hc164Cols<'peripherals>,
    /// Hardware CRC peripheral used for EEPROM calibration block checksums.
    crc: Crc<'peripherals>,
    /// EEPROM driver for loading and persisting calibration data.
    eeprom: Ft24c64<'peripherals, IM>,
    /// DMA interrupt binding reused for every ADC sequence read.
    irq: IRQ,
    /// Dynamic per-key runtime state for the scan loop.
    state: [[KeyState; COL]; ROW],
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
        Self {
            adc_part,
            auto_calib: [[AutoCalib::new(); COL]; ROW],
            calib: [[KeyCalib::uncalibrated(); COL]; ROW],
            cfg,
            cols,
            crc,
            eeprom,
            irq,
            state: [[KeyState::new(); COL]; ROW],
        }
    }

    /// Apply a row-major array of [`CalibEntry`] values to `calib`, pairing
    /// each stored full-travel reading with the corresponding live zero-travel
    /// reading from `zero_raw`.
    fn apply_calib(
        calib: &mut [[KeyCalib; COL]; ROW],
        entries: &[[CalibEntry; COL]; ROW],
        zero_raw: &[[u16; COL]; ROW],
    ) {
        for ((cal_row, entry_row), zero_row) in calib.iter_mut().zip(entries).zip(zero_raw) {
            for ((cal, entry), &zero) in cal_row.iter_mut().zip(entry_row).zip(zero_row) {
                *cal = KeyCalib::new(zero, entry.full);
            }
        }
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
        let mut entries = Self::default_entries();

        let loaded = self.eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
            && try_deserialize::<ROW, COL>(&eeprom_buf, &mut entries, &mut self.crc);
        let mut buf = [0_u16; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);
        if loaded {
            // Re-measure zero travel on every boot to compensate for
            // temperature drift; full-travel data comes from EEPROM.
            let zero_raw = Self::calibrate_zero_raw(&mut self.cols, &mut seq, &mut buf, self.cfg).await;
            Self::apply_calib(&mut self.calib, &entries, &zero_raw);
        } else {
            Self::run_first_boot_calib(
                &mut self.cols,
                &mut seq,
                &mut buf,
                self.cfg,
                &mut self.eeprom,
                &mut self.crc,
                &mut self.calib,
                &mut eeprom_buf,
                &mut entries,
            )
            .await;
        }

        Self::run_scan_loop(
            &mut self.cols,
            &mut self.state,
            &mut self.calib,
            &mut self.auto_calib,
            &mut seq,
            &mut buf,
            self.cfg,
        )
        .await;
    }
}

/// Copy the element at `(row, col)` from a 2-D array slice, returning `None`
/// if either index is out of bounds.
#[inline]
fn get2<T: Copy, const C: usize>(arr: &[[T; C]], row: usize, col: usize) -> Option<T> {
    arr.get(row).and_then(|r| r.get(col)).copied()
}

/// Return a mutable reference to `(row, col)` in a 2-D array slice, returning
/// `None` if either index is out of bounds.
#[inline]
fn get2_mut<T, const C: usize>(arr: &mut [[T; C]], row: usize, col: usize) -> Option<&mut T> {
    arr.get_mut(row).and_then(|r| r.get_mut(col))
}
