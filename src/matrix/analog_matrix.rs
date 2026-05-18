//! Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
//! and continuous auto-calibration.

/// Sparse Hall-sensor transfer-function lookup table.
mod lut;
/// Calibration types, constants, and per-key runtime state.
pub mod types;

use crate::{
    backlight::processor::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
    eeprom::Ft24c64,
    layout::{MATRIX_TO_LED, VALID_ROWS_BY_COL},
    matrix::{
        analog_matrix::types::{
            AdcSampleTime,
            CALIB_HOLD_DURATION_MS,
            CALIB_PRESS_THRESHOLD,
            CALIB_SETTLE_AFTER_ALL_DONE,
            CALIB_ZERO_TOLERANCE,
            DEFAULT_FULL_RANGE,
            KeyCalibState,
            KeyEntry,
            MIN_USEFUL_FULL_RANGE,
            REF_ZERO_TRAVEL,
            VALID_RAW_MAX,
            VALID_RAW_MIN,
            ZERO_TRAVEL_DEAD_ZONE,
            full_from_min,
        },
        calib_store::{self, CALIB_BUF_LEN, EEPROM_BASE_ADDR, try_deserialize},
        hc164_cols::Hc164Cols,
    },
};
use core::array::from_fn;
use core::hint::{cold_path, likely, unlikely};
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, BasicInstance, ConfiguredSequence, Instance, RxDma},
    crc::Crc,
    dma::InterruptHandler,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    pac::adc,
};
use embassy_time::{Duration, Instant};
use rmk::{
    core_traits::Runnable,
    embassy_futures::yield_now,
    event::{KeyboardEvent, publish_event_async},
};
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

    /// Average `cfg.calib_passes` full-matrix scans to establish per-key
    /// zero-travel (resting) ADC values.
    ///
    /// All keys must be fully released during this pass. Returns a
    /// `ROW × COL` array of raw ADC averages, each reduced by
    /// [`ZERO_TRAVEL_DEAD_ZONE`] so that the resting position sits cleanly
    /// below the measured average, preventing ADC noise from producing
    /// spurious non-zero travel readings.
    async fn calibrate_zero_raw(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> [[u16; COL]; ROW] {
        let mut acc = [[0_u32; COL]; ROW];
        for _ in 0..cfg.calib_passes {
            cols.reset();
            for col in 0..COL {
                yield_now().await;
                seq.read(buf).await;
                cols.advance();
                for (acc_row, &raw) in acc.iter_mut().zip(buf.iter()) {
                    if let Some(cell) = acc_row.get_mut(col) {
                        *cell = cell.saturating_add(u32::from(raw));
                    }
                }
            }
        }

        let mut result = [[REF_ZERO_TRAVEL; COL]; ROW];
        for (res_row, acc_row) in result.iter_mut().zip(acc.iter()) {
            for (res, &total) in res_row.iter_mut().zip(acc_row.iter()) {
                // Leave the REF_ZERO_TRAVEL initializer in place on any
                // arithmetic failure (e.g. calib_passes == 0).
                if let Some(avg) = total.checked_div(cfg.calib_passes) {
                    *res = u16::try_from(avg).unwrap_or(REF_ZERO_TRAVEL).saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                }
            }
        }
        result
    }

    /// Count sensor positions that both have a physical hall element and a
    /// plausible zero-travel reading.
    ///
    /// [`VALID_ROWS_BY_COL`] already filters to sensor-present positions; the
    /// additional [`CALIB_ZERO_TOLERANCE`] check discards any sensor whose
    /// resting ADC is too far from [`REF_ZERO_TRAVEL`] to produce reliable
    /// calibration data. The result drives both the Phase A early-exit
    /// condition and the displayed progress percentage so they agree on the
    /// denominator.
    fn count_real_sensors(zero_raw: &[[u16; COL]; ROW]) -> usize {
        let mut total: usize = 0;
        for (col_idx, valid) in VALID_ROWS_BY_COL.iter().enumerate() {
            for i in 0..valid.count {
                let Some(key) = valid.keys.get(i) else { continue };
                let in_range = zero_raw
                    .get(usize::from(key.key_row))
                    .and_then(|row_slice| row_slice.get(col_idx))
                    .copied()
                    .is_some_and(|zero| zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE);
                if in_range {
                    total = total.saturating_add(1);
                }
            }
        }
        total
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

    /// Phase A of full-travel calibration: drive the per-key
    /// Waiting → Holding → Accepted state machine, repaint each accepted key
    /// green, push gradient progress updates, and exit early once all real
    /// keys are accepted or the user-supplied `duration` elapses.
    ///
    /// Updates `min_raw` with the deepest ADC reading seen per key
    /// throughout the pass (even for keys that have already been accepted),
    /// and returns the final accepted count so the caller can decide whether
    /// to send the all-accepted backlight signal.
    async fn run_calib_press_phase(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        zero_raw: &[[u16; COL]; ROW],
        min_raw: &mut [[u16; COL]; ROW],
        total_keys: usize,
        duration: Duration,
    ) -> usize {
        let deadline = Instant::now().saturating_add(duration);
        let hold_duration = Duration::from_millis(CALIB_HOLD_DURATION_MS);
        let mut calib_state: [[KeyCalibState; COL]; ROW] = [[KeyCalibState::Waiting; COL]; ROW];
        let mut calibrated_count: usize = 0;
        let mut last_pct: u8 = 0;

        while Instant::now() < deadline && calibrated_count < total_keys {
            cols.reset();
            for col in 0..COL {
                yield_now().await;
                seq.read(buf).await;
                cols.advance();

                // Only valid sensor positions are stored in VALID_ROWS_BY_COL;
                // columns with no sensors produce an empty list and are skipped
                // entirely without touching the calibration state machine.
                let Some(valid) = VALID_ROWS_BY_COL.get(col) else { continue };
                for i in 0..valid.count {
                    let Some(key) = valid.keys.get(i) else { continue };
                    let key_row = usize::from(key.key_row);
                    let raw = buf.get(usize::from(key.buf_row)).copied().unwrap_or(0);

                    // Always track the deepest reading seen, regardless of
                    // whether the key has been accepted yet.
                    min_into(min_raw, key_row, col, raw);

                    let Some(key_state) = calib_state.get_mut(key_row).and_then(|row_slice| row_slice.get_mut(col))
                    else {
                        continue;
                    };

                    // Skip keys already accepted.
                    if likely(matches!(*key_state, KeyCalibState::Accepted)) {
                        continue;
                    }

                    let zero =
                        zero_raw.get(key_row).and_then(|row_slice| row_slice.get(col)).copied().unwrap_or(REF_ZERO_TRAVEL);
                    let pressed = zero.saturating_sub(raw) >= CALIB_PRESS_THRESHOLD;

                    match *key_state {
                        KeyCalibState::Waiting if pressed => {
                            // Start hold timer on first threshold crossing.
                            *key_state = KeyCalibState::Holding(Instant::now());
                        },
                        KeyCalibState::Holding(_) if !pressed => {
                            // Released before hold duration; reset so the user
                            // must press it all the way down again.
                            *key_state = KeyCalibState::Waiting;
                        },
                        KeyCalibState::Holding(first_seen) if unlikely(first_seen.elapsed() >= hold_duration) => {
                            // Hold duration satisfied; accept this key.
                            *key_state = KeyCalibState::Accepted;
                            calibrated_count = calibrated_count.saturating_add(1);
                            #[cfg(feature = "defmt")]
                            defmt::debug!("key accepted row={} col={} raw={}", key_row, col, raw);

                            if let Some(led_row) = MATRIX_TO_LED.get(key_row)
                                && let Some(&Some(led_idx)) = led_row.get(col)
                            {
                                BACKLIGHT_CH.sender().send(BacklightCmd::CalibKeyDone(led_idx)).await;
                            }

                            let pct = pct_done(calibrated_count, total_keys);
                            if pct != last_pct {
                                last_pct = pct;
                                BACKLIGHT_CH.sender().send(BacklightCmd::CalibProgress(pct)).await;
                            }
                        },
                        // Holding while still pressed but under the hold
                        // duration, Waiting while still released, or
                        // Accepted: no state change.
                        KeyCalibState::Accepted | KeyCalibState::Holding(_) | KeyCalibState::Waiting => {},
                    }
                }
            }
        }
        calibrated_count
    }

    /// Run the guided first-boot two-phase calibration, persist the result to
    /// EEPROM, and apply it to `keys`.
    ///
    /// Backlight signals during the process:
    /// - **Amber** - zero-travel pass, all keys must be released.
    /// - **Red → blue gradient with green per accepted key** - full-travel
    ///   press window; the background interpolates with progress while each
    ///   accepted key turns solid green individually.
    /// - **Green blink ×3** - all keys accepted; keys may be released.
    /// - **Green for 2 s** - calibration stored successfully.
    /// - **Amber** - EEPROM write-back verification failed; keyboard will
    ///   re-calibrate on the next boot.
    ///
    /// Keys not pressed during the full-travel window fall back to
    /// `zero - DEFAULT_FULL_RANGE` so the keyboard remains functional.
    async fn run_first_boot_calib(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
        eeprom: &mut Ft24c64<'_, IM>,
        crc: &mut Crc<'_>,
        keys: &mut [[KeyEntry; ROW]; COL],
    ) {
        let mut eeprom_buf = [0_u8; CALIB_BUF_LEN];

        BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Zero)).await;
        let zero_raw = Self::calibrate_zero_raw(cols, seq, buf, cfg).await;

        BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Full)).await;
        let full_raw = Self::sample_full_raw(cols, seq, buf, cfg, &zero_raw).await;

        // Compute entry_full for every key from the measured zero and the
        // minimum ADC seen during the full-travel press window. Keys that
        // never crossed the press threshold fall back to the synthetic
        // `zero - DEFAULT_FULL_RANGE` floor so the keyboard stays usable.
        for (col, key_col) in keys.iter_mut().enumerate() {
            for (row, key) in key_col.iter_mut().enumerate() {
                let zero = zero_raw.get(row).and_then(|row_slice| row_slice.get(col)).copied().unwrap_or(REF_ZERO_TRAVEL);
                let seen_min = full_raw.get(row).and_then(|row_slice| row_slice.get(col)).copied().unwrap_or(u16::MAX);
                key.entry_full = if zero.saturating_sub(seen_min) >= MIN_USEFUL_FULL_RANGE {
                    full_from_min(zero, seen_min)
                } else {
                    zero.saturating_sub(DEFAULT_FULL_RANGE).max(VALID_RAW_MIN)
                };
            }
        }

        Self::apply_calib(keys, &zero_raw);

        calib_store::serialize(keys, &mut eeprom_buf, crc);

        // Verify by reading back into the same buffer and re-deserializing.
        // Reusing the buffer avoids a second large stack allocation.
        let verified = eeprom.erase().await.is_ok()
            && eeprom.write(EEPROM_BASE_ADDR, &eeprom_buf).await.is_ok()
            && eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
            && try_deserialize::<ROW, COL>(&eeprom_buf, keys, crc);

        if !verified {
            #[cfg(feature = "defmt")]
            defmt::warn!("EEPROM write-back verification failed, will recalibrate on next boot");
            // Signal amber so the user knows calibration will repeat on the
            // next boot.
            BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Zero)).await;
            return;
        }
        #[cfg(feature = "defmt")]
        defmt::info!("calibration verified and persisted to EEPROM");
        BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Done)).await;
    }

    /// Hot-path matrix scan loop. Runs forever, publishing key state changes
    /// directly via [`publish_event_async`] as they are detected.
    ///
    /// The [`ConfiguredSequence`] is programmed once before entry and reused
    /// across all columns and passes. Both the column settle delay and the DMA
    /// transfer are fully async. For each reading that passes the noise gate
    /// the auto-calibrator is updated before the travel and rapid-trigger
    /// logic runs, so any calibration refinement takes effect within the same
    /// scan pass.
    #[optimize(speed)]
    async fn run_scan_loop(
        cols: &mut Hc164Cols<'peripherals>,
        keys: &mut [[KeyEntry; ROW]; COL],
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> ! {
        #[cfg(feature = "bench")] use crate::bench;
        #[cfg(feature = "bench")]
        let mut acc = bench::Accumulator::new();
        let act_threshold = cfg.actuation_pt;
        // Clamp to 1 so a zero config value never disables the dead-band.
        let sensitivity_press = cfg.rt_sensitivity_press.max(1);
        let sensitivity_release = cfg.rt_sensitivity_release.max(1);
        let trough_floor = act_threshold.saturating_sub(sensitivity_press);
        let noise_gate = cfg.noise_gate;
        loop {
            cols.reset();
            for col in 0..COL {
                yield_now().await;
                seq.read(buf).await;
                #[cfg(feature = "adc_debug")]
                defmt::debug!(
                    "raw col={} r0={} r1={} r2={} r3={} r4={} r5={}",
                    col,
                    buf[0],
                    buf[1],
                    buf[2],
                    buf[3],
                    buf[4],
                    buf[5]
                );
                cols.advance();
                #[cfg(feature = "bench")]
                let time = bench::cycles();
                // Only valid sensor positions are stored in VALID_ROWS_BY_COL;
                // columns with no sensors produce an empty list and are skipped
                // entirely without touching the key-state machine.
                // Slice to exactly the populated entries (one check instead of
                // one per key), and hoist keys[col] out of the inner loop.
                if let Some(valid) = VALID_ROWS_BY_COL.get(col)
                    && let Some(valid_keys) = valid.keys.get(..valid.count)
                    && let Some(key_col) = keys.get_mut(col)
                {
                    for key in valid_keys {
                        let buf_row = usize::from(key.buf_row);
                        let key_row = usize::from(key.key_row);

                        // Clamp raw ADC value to valid range to prevent out-of-bounds
                        // LUT access and ensure valid calibration updates.
                        let raw = buf.get(buf_row).copied().unwrap_or(0).clamp(VALID_RAW_MIN, VALID_RAW_MAX);

                        let Some(entry) = key_col.get_mut(key_row) else { continue };

                        // Skip if the reading has not changed beyond the noise gate.
                        if likely(entry.last_raw.abs_diff(raw) < noise_gate) {
                            continue;
                        }

                        // Record the raw value now so that repeated identical readings
                        // are filtered by the noise gate even when travel_from later
                        // returns None (uncalibrated or out-of-range position).
                        entry.last_raw = raw;

                        // Update the auto-calibrator with this reading before the
                        // travel computation so any refined calibration is used immediately.
                        entry.auto_calib_step(raw);

                        let Some(new_travel) = entry.travel_from(raw) else { continue };

                        // After noise gate, travel often resolves to the same
                        // quantized value; skip redundant RT logic.
                        if likely(new_travel == entry.travel) {
                            continue;
                        }
                        entry.travel = new_travel;

                        // Dynamic Rapid Trigger; only the transition path needs
                        // to publish, so the common no-transition case stays in
                        // the `None` arm.
                        if let Some(now_pressed) = entry.step_rapid_trigger(
                            new_travel,
                            act_threshold,
                            sensitivity_press,
                            sensitivity_release,
                            trough_floor,
                        ) {
                            cold_path();
                            publish_event_async(KeyboardEvent::key(
                                key.key_row,
                                // COL ≤ 255 is enforced by a compile-time assert in layout.rs;
                                // u8::MAX is used as sentinel so a failure is obviously wrong.
                                u8::try_from(col).unwrap_or(u8::MAX),
                                now_pressed,
                            ))
                            .await;
                            #[cfg(feature = "defmt")]
                            defmt::trace!("key row={} col={} pressed={}", key.key_row, col, now_pressed);
                        }
                    }
                }
                #[cfg(feature = "bench")]
                if let Some(avg) = acc.record(bench::elapsed(time)) {
                    #[cfg(feature = "defmt")]
                    defmt::debug!("scan avg={} cycles/col", avg);
                }
            }
        }
    }

    /// Sample the matrix for `cfg.full_calib_duration` (or until all real
    /// keys are accepted), recording the minimum ADC reading seen per key.
    ///
    /// Lower ADC = more magnet travel, so the minimum reading over the window
    /// is the deepest press seen. A key is accepted only after it has stayed
    /// continuously below [`CALIB_PRESS_THRESHOLD`] for
    /// [`CALIB_HOLD_DURATION_MS`]; releasing and re-pressing resets the timer.
    /// The LED turns green only at acceptance, not at first crossing.
    ///
    /// After all keys are accepted a [`CALIB_SETTLE_AFTER_ALL_DONE`]
    /// continuation window keeps updating `min_raw` so the stored value
    /// reflects the true bottom-out ADC, not merely the acceptance instant.
    async fn sample_full_raw(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
        zero_raw: &[[u16; COL]; ROW],
    ) -> [[u16; COL]; ROW] {
        let mut min_raw = [[u16::MAX; COL]; ROW];
        let total_keys = Self::count_real_sensors(zero_raw).max(1);

        let calibrated_count =
            Self::run_calib_press_phase(cols, seq, buf, zero_raw, &mut min_raw, total_keys, cfg.full_calib_duration)
                .await;

        // If all keys were accepted (not just a deadline timeout), signal the
        // backlight to blink green so the user knows to release their keys.
        // Best-effort: missing this signal only skips the animation.
        if calibrated_count >= total_keys {
            BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::AllAccepted)).await;
        }

        // Phase B: continue sampling for CALIB_SETTLE_AFTER_ALL_DONE regardless
        // of whether Phase A ended by full acceptance or timeout.  This gives
        // every accepted key time to reach its true bottom-out ADC rather than
        // storing the value at the moment of acceptance, and still captures the
        // deepest reading seen for any keys that ran out of time.
        Self::settle_min_raw(cols, seq, buf, &mut min_raw, CALIB_SETTLE_AFTER_ALL_DONE).await;

        min_raw
    }

    /// Phase B of full-travel calibration: continue updating `min_raw` for
    /// `duration` without any state-machine work or backlight signaling, so
    /// each key has time to settle to its true bottom-out ADC.
    async fn settle_min_raw(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        min_raw: &mut [[u16; COL]; ROW],
        duration: Duration,
    ) {
        let deadline = Instant::now().saturating_add(duration);
        while Instant::now() < deadline {
            cols.reset();
            for col in 0..COL {
                yield_now().await;
                seq.read(buf).await;
                cols.advance();
                let Some(valid) = VALID_ROWS_BY_COL.get(col) else { continue };
                for i in 0..valid.count {
                    let Some(key) = valid.keys.get(i) else { continue };
                    let raw = buf.get(usize::from(key.buf_row)).copied().unwrap_or(0);
                    min_into(min_raw, usize::from(key.key_row), col, raw);
                }
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

        let loaded = self.eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
            && try_deserialize::<ROW, COL>(&eeprom_buf, &mut self.keys, &mut self.crc);
        let mut buf = [0_u16; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);
        if loaded {
            #[cfg(feature = "defmt")]
            defmt::info!("calibration loaded from EEPROM");
            // Re-measure zero travel on every boot to compensate for
            // temperature drift; full-travel data comes from EEPROM.
            let zero_raw = Self::calibrate_zero_raw(&mut self.cols, &mut seq, &mut buf, self.cfg).await;
            Self::apply_calib(&mut self.keys, &zero_raw);
        } else {
            #[cfg(feature = "defmt")]
            defmt::info!("no valid EEPROM calibration, running first-boot calibration");
            Self::run_first_boot_calib(
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

        Self::run_scan_loop(&mut self.cols, &mut self.keys, &mut seq, &mut buf, self.cfg).await;
    }
}

/// Update the running per-key minimum at `(key_row, col)` with a new ADC
/// reading via safe indexing.
///
/// Out-of-bounds positions are silently skipped; `min_raw` is dimensioned to
/// match the matrix and the calling indices come from
/// [`crate::layout::VALID_ROWS_BY_COL`] which itself is bounded by `ROW`/`COL`,
/// so this only fires on a structural bug.
#[inline]
fn min_into<const ROW: usize, const COL: usize>(min_raw: &mut [[u16; COL]; ROW], key_row: usize, col: usize, raw: u16) {
    if let Some(cell) = min_raw.get_mut(key_row).and_then(|min_row| min_row.get_mut(col)) {
        *cell = (*cell).min(raw);
    }
}

/// Convert `done`/`total` into a percentage in 0..=100 using saturating
/// arithmetic at every step so a zero or overflowing `total` returns 0
/// instead of panicking.
#[inline]
const fn pct_done(done: usize, total: usize) -> u8 {
    u8::try_from(done.saturating_mul(100).checked_div(total).unwrap_or(0)).unwrap_or(100).min(100)
}
