//! First-boot guided calibration and EEPROM persistence.
//!
//! Contains the two-phase calibration sequence (zero-travel pass followed by
//! full-travel press pass) and the EEPROM read/write/verify logic.  The
//! continuous background auto-calibration that runs during normal scanning
//! lives in [`super::scan`] alongside the hot scan loop that calls it.

use super::{AdcSampleTime, AnalogHallMatrix, HallCfg, get2};
use crate::{
    backlight::{
        led_processor::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
        mapping::MATRIX_TO_LED,
    },
    eeprom::Ft24c64,
    matrix::{
        analog_matrix::types::{
            BOTTOM_JITTER,
            CALIB_HOLD_DURATION_MS,
            CALIB_PRESS_THRESHOLD,
            CALIB_SETTLE_AFTER_ALL_DONE,
            CALIB_ZERO_TOLERANCE,
            DEFAULT_FULL_RANGE,
            KeyCalib,
            KeyCalibState,
            MIN_USEFUL_FULL_RANGE,
            REF_ZERO_TRAVEL,
            UNCALIBRATED_ZERO,
            VALID_RAW_MIN,
            ZERO_TRAVEL_DEAD_ZONE,
        },
        calib_store,
        calib_store::{CALIB_BUF_LEN, CalibEntry, EEPROM_BASE_ADDR},
        hc164_cols::Hc164Cols,
        sensor_mapping::SENSOR_POSITIONS,
    },
};
use calib_store::try_deserialize;
use embassy_stm32::{
    adc::{BasicInstance, ConfiguredSequence, Instance, RxDma},
    crc::Crc,
    dma::InterruptHandler,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    pac::adc,
};
use embassy_time::{Duration, Instant, Timer};

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// Build the default calibration entries used when EEPROM is blank or
    /// invalid, giving a reasonable approximation of travel until real
    /// calibration runs.
    pub(super) const fn default_entries() -> [[CalibEntry; COL]; ROW] {
        [[CalibEntry { full: UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE) }; COL]; ROW]
    }

    /// Average `cfg.calib_passes` full-matrix scans to establish per-key
    /// zero-travel (resting) ADC values.
    ///
    /// All keys must be fully released during this pass. Returns a
    /// `ROW × COL` array of raw ADC averages, each reduced by
    /// [`ZERO_TRAVEL_DEAD_ZONE`] so that the resting position sits cleanly
    /// below the measured average, preventing ADC noise from producing
    /// spurious non-zero travel readings.
    pub(super) async fn calibrate_zero_raw(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> [[u16; COL]; ROW] {
        let mut acc = [[0_u32; COL]; ROW];

        for _ in 0..cfg.calib_passes {
            for col in 0..COL {
                cols.select(col);
                seq.read(buf).await;
                for (acc_row, &raw) in acc.iter_mut().zip(buf.iter()) {
                    if let Some(cell) = acc_row.get_mut(col) {
                        *cell = cell.saturating_add(u32::from(raw));
                    }
                }
            }
        }

        let mut result = [[UNCALIBRATED_ZERO; COL]; ROW];
        for (res_row, acc_row) in result.iter_mut().zip(acc.iter()) {
            for (res, &total) in res_row.iter_mut().zip(acc_row.iter()) {
                // Leave the UNCALIBRATED_ZERO initializer in place on any
                // arithmetic failure (e.g. calib_passes == 0).
                if let Some(avg) = total.checked_div(cfg.calib_passes) {
                    *res = u16::try_from(avg).unwrap_or(UNCALIBRATED_ZERO).saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                }
            }
        }
        result
    }

    /// Sample the matrix for `duration` (or until all real keys are accepted),
    /// recording the minimum ADC reading seen per key.
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
    pub(super) async fn sample_full_raw(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        duration: Duration,
        zero_raw: &[[u16; COL]; ROW],
    ) -> [[u16; COL]; ROW] {
        let deadline = Instant::now() + duration;
        let mut min_raw = [[u16::MAX; COL]; ROW];

        // Closure to update the running minimum for a single position.
        let mut update_min = |row: usize, col: usize, raw: u16| {
            if let Some(cell) = min_raw.get_mut(row).and_then(|r| r.get_mut(col)) {
                *cell = (*cell).min(raw);
            }
        };

        let mut calib_state: [[KeyCalibState; COL]; ROW] = [[KeyCalibState::Waiting; COL]; ROW];
        let mut calibrated_count: usize = 0;
        let hold_duration = Duration::from_millis(CALIB_HOLD_DURATION_MS);

        // Count only positions that have a physical sensor and a plausible
        // zero-travel reading so the denominator matches the detection loop.
        let total_keys = (0..ROW)
            .flat_map(|r| (0..COL).map(move |c| (r, c)))
            .filter(|&(r, c)| {
                SENSOR_POSITIONS.get(r).and_then(|row| row.get(c)).copied().unwrap_or(false)
                    && get2(zero_raw, r, c).is_some_and(|z| z.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE)
            })
            .count()
            .max(1);

        let mut last_pct: u8 = 0;

        // Phase A: sample until all real keys are accepted or the deadline expires.
        while Instant::now() < deadline && calibrated_count < total_keys {
            for col in 0..COL {
                cols.select(col);
                seq.read(buf).await;

                for (row, &raw) in buf.iter().enumerate() {
                    // Always track the deepest reading seen, regardless of
                    // whether the key has been accepted yet.
                    update_min(row, col, raw);

                    if !SENSOR_POSITIONS.get(row).and_then(|r| r.get(col)).copied().unwrap_or(false) {
                        continue;
                    }

                    let Some(key_state) = calib_state.get_mut(row).and_then(|r| r.get_mut(col)) else {
                        continue;
                    };

                    // Skip keys already accepted.
                    if matches!(*key_state, KeyCalibState::Accepted) {
                        continue;
                    }

                    let zero = get2(zero_raw, row, col).unwrap_or(UNCALIBRATED_ZERO);
                    let pressed = zero.saturating_sub(raw) >= CALIB_PRESS_THRESHOLD;

                    match *key_state {
                        KeyCalibState::Waiting => {
                            if pressed {
                                // Start hold timer on first threshold crossing.
                                *key_state = KeyCalibState::Holding(Instant::now());
                            }
                        }
                        KeyCalibState::Holding(first_seen) => {
                            if pressed {
                                if first_seen.elapsed() >= hold_duration {
                                    // Hold duration satisfied; accept this key.
                                    *key_state = KeyCalibState::Accepted;
                                    calibrated_count = calibrated_count.saturating_add(1);

                                    if let Some(Some(led_idx)) =
                                        MATRIX_TO_LED.get(row).and_then(|r| r.get(col)).copied()
                                    {
                                        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibKeyDone(led_idx)).ok();
                                    }

                                    let pct = u8::try_from(
                                        calibrated_count.saturating_mul(100).checked_div(total_keys).unwrap_or(0),
                                    )
                                    .unwrap_or(100)
                                    .min(100);

                                    if pct != last_pct {
                                        last_pct = pct;
                                        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibProgress(pct)).ok();
                                    }
                                }
                                // else: still within hold window; keep waiting.
                            } else {
                                // Released before hold duration; reset so the
                                // user must press it all the way down again.
                                *key_state = KeyCalibState::Waiting;
                            }
                        }
                        KeyCalibState::Accepted => {}
                    }
                }
            }
        }

        // If all keys were accepted (not just a deadline timeout), signal the
        // backlight to blink green so the user knows to release their keys.
        // Best-effort: missing this signal only skips the animation.
        if calibrated_count >= total_keys {
            BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::AllAccepted)).ok();
        }

        // Phase B: continue sampling for CALIB_SETTLE_AFTER_ALL_DONE regardless
        // of whether Phase A ended by full acceptance or timeout.  This gives
        // every accepted key time to reach its true bottom-out ADC rather than
        // storing the value at the moment of acceptance, and still captures the
        // deepest reading seen for any keys that ran out of time.
        let settle_deadline = Instant::now().saturating_add(CALIB_SETTLE_AFTER_ALL_DONE);
        while Instant::now() < settle_deadline {
            for col in 0..COL {
                cols.select(col);
                seq.read(buf).await;
                for (row, &raw) in buf.iter().enumerate() {
                    update_min(row, col, raw);
                }
            }
        }

        min_raw
    }

    /// Run the guided first-boot two-phase calibration, persist the result to
    /// EEPROM, and apply it to `calib`.
    ///
    /// Backlight signals during the process:
    /// - **Amber** - zero-travel pass, all keys must be released.
    /// - **Blue → green per key** - full-travel press window.
    /// - **Green blink ×3** - all keys accepted; keys may be released.
    /// - **Green for 2 s** - calibration stored successfully.
    /// - **Amber** - EEPROM write-back verification failed; keyboard will
    ///   re-calibrate on the next boot.
    ///
    /// Keys not pressed during the full-travel window fall back to
    /// `zero − DEFAULT_FULL_RANGE` so the keyboard remains functional.
    pub(super) async fn run_first_boot_calib(
        cols: &mut Hc164Cols<'_>,
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
        eeprom: &mut Ft24c64<'_, IM>,
        crc: &mut Crc<'_>,
        calib: &mut [[KeyCalib; COL]; ROW],
        eeprom_buf: &mut [u8; CALIB_BUF_LEN],
        entries: &mut [[CalibEntry; COL]; ROW],
    ) {
        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Zero)).ok();
        let zero_raw = Self::calibrate_zero_raw(cols, seq, buf, cfg).await;

        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Full)).ok();
        let full_raw = Self::sample_full_raw(cols, seq, buf, cfg.full_calib_duration, &zero_raw).await;

        for ((entry_row, zero_row), full_row) in entries.iter_mut().zip(zero_raw.iter()).zip(full_raw.iter()) {
            for ((entry, &zero), &seen_min) in entry_row.iter_mut().zip(zero_row.iter()).zip(full_row.iter()) {
                let full = if zero > seen_min && zero.saturating_sub(seen_min) >= MIN_USEFUL_FULL_RANGE {
                    seen_min.saturating_add(BOTTOM_JITTER).min(zero.saturating_sub(MIN_USEFUL_FULL_RANGE))
                } else {
                    zero.saturating_sub(DEFAULT_FULL_RANGE).max(VALID_RAW_MIN)
                };
                *entry = CalibEntry { full };
            }
        }

        Self::apply_calib(calib, entries, &zero_raw);

        // Allow the backlight channel to drain before starting the I²C write.
        Timer::after_micros(200).await;
        calib_store::serialize(entries, eeprom_buf, crc);
        let erase_ok = eeprom.erase().await.is_ok();
        let write_ok = eeprom.write(EEPROM_BASE_ADDR, eeprom_buf).await.is_ok();

        // Verify by reading back into the same buffer and re-deserializing.
        // Reusing the buffer avoids a second large stack allocation.
        let verified = erase_ok
            && write_ok
            && eeprom.read(EEPROM_BASE_ADDR, eeprom_buf).await.is_ok()
            && try_deserialize::<ROW, COL>(eeprom_buf, entries, crc);

        if !verified {
            // Signal amber so the user knows calibration will repeat on the
            // next boot.
            BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Zero)).ok();
            return;
        }

        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Done)).ok();
    }
}
