//! Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
//! and continuous auto-calibration.

/// First-boot guided calibration and EEPROM persistence.
mod calibration;
/// Sparse hall-sensor transfer-function table with linear interpolation.
mod lut;
/// Hot-path matrix scan loop.
mod scan;
/// Calibration types, constants, per-key runtime state, and the calibration
/// arithmetic that operates on it.
pub mod types;

use crate::{
    eeprom::Ft24c64,
    matrix::{
        analog_matrix::types::{AdcSampleTime, KeyEntry},
        calib_store::{CALIB_BUF_LEN, EEPROM_BASE_ADDR, try_deserialize},
        hc164_cols::Hc164Cols,
    },
    usb_state::USB_ACTIVE,
};
use core::{array::from_fn, future::pending};
use embassy_stm32::{
    Peri,
    adc::{Adc, BasicInstance, BorrowedAdcChannel, ConfiguredSequence, Instance, RxDma},
    crc::Crc,
    dma::InterruptHandler,
    exti::ExtiInput,
    gpio::Output,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    mode::Async,
    pac::adc,
};
use rmk::{core_traits::Runnable, embassy_futures::yield_now};
pub use types::HallCfg;

/// Run one full matrix pass: for each of the `col_count` columns, yield to
/// the executor (doubling as the column settle delay), read all row ADCs
/// into `buf`, advance the HC164 walking-one, and hand the readings to
/// `on_col` together with the column index.
///
/// Shared by every calibration pass and the sequential suspend/resume passes
/// in `scan`, so the column-sequencing protocol stays in one place and cannot
/// drift between them. Two passes intentionally do not use this helper: the
/// full-rate scan loop (`scan::active_scan`) pipelines processing into the
/// DMA window instead of running it after the read, and the post-wake
/// publishing pass (`scan::eval_pass`) must `await` per column, which a
/// synchronous `FnMut` callback cannot express.
async fn scan_pass<F, const ROW: usize>(
    cols: &mut Hc164Cols<'_>,
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    col_count: usize,
    mut on_col: F,
) where
    F: FnMut(usize, &[u16; ROW]),
{
    cols.reset();
    for col in 0..col_count {
        yield_now().await;
        seq.read(buf).await;
        cols.advance();
        on_col(col, buf);
    }
}

/// Supplies the per-row ADC channels for a single sequence read.
///
/// The current embassy ADC API exposes channels only as transient
/// [`BorrowedAdcChannel`] values produced by `AdcChannel::reborrow_adc` on an
/// *owned* pin. The previously available owned, type-erased channel
/// (`AnyAdcChannel`, which itself implemented `AdcChannel`) was removed
/// upstream, so a `BorrowedAdcChannel` can no longer be stored in a struct and
/// re-borrowed with a shorter lifetime — it borrows its pin for as long as it
/// lives and has no public way to shorten that borrow.
///
/// Implementors therefore own the concrete row pins (which do implement
/// `AdcChannel`) and re-borrow them fresh on every call. This keeps
/// [`AnalogHallMatrix`] generic over the board's pin set while matching the
/// borrow-at-the-call-site contract the new API requires — no `unsafe` or
/// lifetime transmutation needed.
pub trait RowChannels<ADC, const ROW: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
{
    /// Re-borrow every row pin into a fresh [`BorrowedAdcChannel`] for one
    /// [`ConfiguredSequence`] construction. The returned channels borrow `self`
    /// only for the duration of the sequence build, after which the pins are
    /// free to be re-borrowed again.
    fn reborrow_channels(&mut self) -> [BorrowedAdcChannel<'_, ADC>; ROW];

    /// Clear the suspend pull-down ahead of resuming analog sampling.
    ///
    /// The ADC's analog-mode setup (`reborrow_adc`) only writes the mode
    /// register and leaves the pull configuration untouched, so a pull-down
    /// left over from [`RowChannels::set_low_power`] would otherwise drag the
    /// hall-sensor reading low. This returns every row to a no-pull input
    /// before the sequence is rebuilt.
    fn set_active(&mut self);

    /// Park the row pins for suspend by driving each to input-pull-down,
    /// mirroring the stock firmware's `setPinInputLow` on every row. While the
    /// sensor rail is cut the analog row nodes would otherwise float (the ADC
    /// leaves them in high-impedance analog mode), and a floating node can
    /// couple noise into the PC5 key-wake comparator network; a defined low
    /// avoids that. Undone by [`RowChannels::set_active`] before the next
    /// sequence build re-asserts analog mode.
    fn set_low_power(&mut self);
}

/// ADC-related peripherals grouped to allow split borrows when constructing
/// a [`embassy_stm32::adc::ConfiguredSequence`].
pub struct AdcPart<'peripherals, ADC, D, R, IRQ, const ROW: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    R: RowChannels<ADC, ROW>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripheral used for sampling hall sensors.
    pub adc:         Adc<'peripherals, ADC>,
    /// DMA channel used for non-blocking ADC sequence reads.
    pub dma:         Peri<'peripherals, D>,
    /// DMA interrupt binding reused for every ADC sequence read.
    pub irq:         IRQ,
    /// Owned row pins, re-borrowed into ADC channels for each sequence read.
    pub rows:        R,
    /// ADC sample time applied to every channel in the sequence.
    pub sample_time: AdcSampleTime<ADC>,
}

impl<'peripherals, ADC, D, R, IRQ, const ROW: usize> AdcPart<'peripherals, ADC, D, R, IRQ, ROW>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    R: RowChannels<ADC, ROW>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a [`embassy_stm32::adc::ConfiguredSequence`] over all row
    /// channels.
    ///
    /// Programs the ADC sequence registers once; the returned reader can be
    /// triggered repeatedly without reprogramming. The row pins are re-borrowed
    /// only while the sequence is being built, so the returned reader borrows
    /// just the ADC and DMA channel.
    fn configure_sequence(&mut self) -> ConfiguredSequence<'_, adc::Adc> {
        let st = self.sample_time;
        self.adc.configure_sequence(
            self.dma.reborrow(),
            self.rows.reborrow_channels().into_iter().map(|ch| (ch, st)),
            self.irq,
        )
    }

    /// Create a new [`AdcPart`] from the given peripherals.
    pub const fn new(
        adc: Adc<'peripherals, ADC>,
        rows: R,
        dma: Peri<'peripherals, D>,
        irq: IRQ,
        sample_time: AdcSampleTime<ADC>,
    ) -> Self {
        Self { adc, dma, irq, rows, sample_time }
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
pub struct AnalogHallMatrix<'peripherals, ADC, D, R, IRQ, IM, const ROW: usize, const COL: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    R: RowChannels<ADC, ROW>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripherals and channels grouped for split-borrow compatibility;
    /// also owns the DMA interrupt binding used to build each sequence.
    adc_part: AdcPart<'peripherals, ADC, D, R, IRQ, ROW>,
    /// Sensing and scanning configuration.
    cfg:      HallCfg,
    /// Column driver used to select the active column via the HC164.
    cols:     Hc164Cols<'peripherals>,
    /// Hardware CRC peripheral used for EEPROM calibration block checksums.
    crc:      Crc<'peripherals>,
    /// EEPROM driver for loading and persisting calibration data.
    eeprom:   Ft24c64<'peripherals, IM>,
    /// Per-key runtime state, calibration, and auto-calibration. Stored
    /// column-major (`[[KeyEntry; ROW]; COL]`) so the per-column inner scan
    /// loop walks one contiguous SRAM block per HC164 column, which the AHB
    /// load-store unit pipelines better than scattered indirect loads from a
    /// row-major layout.
    keys:     [[KeyEntry; ROW]; COL],
    /// Hall-sensor power rail (PC13), held high during calibration and active
    /// scanning and cut between passes while the USB bus is suspended.
    power:    Output<'peripherals>,
    /// Hardware any-key wake line (PC5); parks the scanner during suspend.
    wake:     ExtiInput<'peripherals, Async>,
}

impl<'peripherals, ADC, D, R, IRQ, IM, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, R, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    R: RowChannels<ADC, ROW>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a new matrix scanner.
    ///
    /// Calibration is deferred to [`Runnable::run`], which loads from EEPROM
    /// on subsequent boots or runs a full first-boot calibration pass.
    pub fn new(
        adc_part: AdcPart<'peripherals, ADC, D, R, IRQ, ROW>,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
        eeprom: Ft24c64<'peripherals, IM>,
        crc: Crc<'peripherals>,
        power: Output<'peripherals>,
        wake: ExtiInput<'peripherals, Async>,
    ) -> Self {
        Self { adc_part, cfg, cols, crc, eeprom, keys: from_fn(|_| from_fn(|_| KeyEntry::default())), power, wake }
    }
}

impl<'peripherals, ADC, D, R, IRQ, IM, const ROW: usize, const COL: usize> Runnable
    for AnalogHallMatrix<'peripherals, ADC, D, R, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    R: RowChannels<ADC, ROW>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    async fn run(&mut self) -> ! {
        let mut eeprom_buf = [0_u8; CALIB_BUF_LEN];

        let loaded = self.eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
            && try_deserialize::<ROW, COL>(&eeprom_buf, &mut self.keys, &mut self.crc);
        let mut buf = [0_u16; ROW];

        // Scope the calibration sequence so it is dropped (stopping the ADC)
        // before `scan::run` takes over `adc_part` to build and tear down its
        // own sequences around each suspend.
        {
            let mut seq = self.adc_part.configure_sequence();
            if loaded {
                // Re-measure zero travel on every boot to compensate for
                // temperature drift; full-travel data comes from EEPROM.
                let zero_raw = calibration::calibrate_zero_raw(&mut self.cols, &mut seq, &mut buf, self.cfg).await;
                calibration::apply_calib(&mut self.keys, &zero_raw);
            } else {
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
        }

        let Some(mut usb) = USB_ACTIVE.receiver() else {
            loop {
                pending::<()>().await;
            }
        };
        scan::run(
            &mut self.cols,
            &mut self.keys,
            &mut self.adc_part,
            &mut self.power,
            &mut self.wake,
            &mut usb,
            self.cfg,
        )
        .await;
    }
}
