//! First-boot guided calibration and EEPROM persistence.
//!
//! Factored out of `analog_matrix` as free functions (none needed `self`)
//! so the scanner type keeps a single inherent impl. Contains the two-phase
//! calibration sequence (zero-travel pass followed by full-travel press
//! pass) and the EEPROM read/write/verify logic. The continuous background
//! auto-calibration that runs during normal scanning lives in
//! [`KeyEntry`] and is driven by [`scan`].

use crate::{
    backlight::processor::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
    eeprom::Ft24c64,
    layout::{MATRIX_TO_LED, VALID_ROWS_BY_COL},
    matrix::{
        analog_matrix::types::{
            CALIB_HOLD_DURATION_MS,
            CALIB_PRESS_THRESHOLD,
            CALIB_SETTLE_AFTER_ALL_DONE,
            CALIB_ZERO_TOLERANCE,
            DEFAULT_FULL_RANGE,
            HallCfg,
            KeyCalibState,
            KeyEntry,
            MIN_USEFUL_FULL_RANGE,
            REF_ZERO_TRAVEL,
            VALID_RAW_MIN,
            ZERO_TRAVEL_DEAD_ZONE,
            full_from_min,
        },
        calib_store::{self, CALIB_BUF_LEN, EEPROM_BASE_ADDR, try_deserialize},
        hc164_cols::Hc164Cols,
    },
};
use core::hint::{likely, unlikely};
use embassy_stm32::{adc::ConfiguredSequence, crc::Crc, i2c::mode::MasterMode, pac::adc};
use embassy_time::{Duration, Instant};
use rmk::embassy_futures::yield_now;

/// Recompute [`KeyEntry::calib_used`] and the hot-path calibration fields
/// for every key from the freshly measured zero-travel readings in
/// `zero_raw`, using the full-travel value stored in each
/// [`KeyEntry::entry_full`].
///
/// Called after EEPROM load and after first-boot calibration to make the
/// scan loop hot path purely arithmetic.
pub(super) fn apply_calib<const ROW: usize, const COL: usize>(
    keys: &mut [[KeyEntry; ROW]; COL],
    zero_raw: &[[u16; COL]; ROW],
) {
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
pub(super) async fn calibrate_zero_raw<const ROW: usize, const COL: usize>(
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
pub(super) fn count_real_sensors<const ROW: usize, const COL: usize>(zero_raw: &[[u16; COL]; ROW]) -> usize {
    let mut total: usize = 0;
    for (col_idx, valid) in VALID_ROWS_BY_COL.iter().enumerate() {
        for i in 0..valid.count {
            let Some(&row_u8) = valid.rows.get(i) else { continue };
            let in_range = zero_raw
                .get(usize::from(row_u8))
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

/// Run the guided first-boot two-phase calibration, persist the result to
/// EEPROM, and apply it to `keys`.
///
/// Backlight signals during the process:
/// - **Amber** - zero-travel pass, all keys must be released.
/// - **Red → blue gradient with green per accepted key** - full-travel press
///   window; the background interpolates with progress while each accepted key
///   turns solid green individually.
/// - **Green blink ×3** - all keys accepted; keys may be released.
/// - **Green for 2 s** - calibration stored successfully.
/// - **Amber** - EEPROM write-back verification failed; keyboard will
///   re-calibrate on the next boot.
///
/// Keys not pressed during the full-travel window fall back to
/// `zero - DEFAULT_FULL_RANGE` so the keyboard remains functional.
pub(super) async fn run_first_boot_calib<IM, const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    cfg: HallCfg,
    eeprom: &mut Ft24c64<'_, IM>,
    crc: &mut Crc<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
) where
    IM: MasterMode,
{
    let mut eeprom_buf = [0_u8; CALIB_BUF_LEN];

    BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Zero)).await;
    let zero_raw = calibrate_zero_raw(cols, seq, buf, cfg).await;

    BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Full)).await;
    let full_raw = sample_full_raw(cols, seq, buf, cfg, &zero_raw).await;

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

    apply_calib(keys, &zero_raw);

    calib_store::serialize(keys, &mut eeprom_buf, crc);

    // Verify by reading back into the same buffer and re-deserializing.
    // Reusing the buffer avoids a second large stack allocation.
    let verified = eeprom.erase().await.is_ok()
        && eeprom.write(EEPROM_BASE_ADDR, &eeprom_buf).await.is_ok()
        && eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
        && try_deserialize::<ROW, COL>(&eeprom_buf, keys, crc);

    if !verified {
        // Signal amber so the user knows calibration will repeat on the
        // next boot.
        BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Zero)).await;
        return;
    }
    BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Done)).await;
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
pub(super) async fn run_calib_press_phase<const ROW: usize, const COL: usize>(
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
                let Some(&row_u8) = valid.rows.get(i) else { continue };
                let key_row = usize::from(row_u8);
                let raw = buf.get(key_row).copied().unwrap_or(0);

                // Always track the deepest reading seen, regardless of
                // whether the key has been accepted yet.
                min_into(min_raw, key_row, col, raw);

                let Some(key_state) = calib_state.get_mut(key_row).and_then(|row_slice| row_slice.get_mut(col)) else {
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

                        if let Some(led_row) = MATRIX_TO_LED.get(key_row)
                            && let Some(&Some(led_idx)) = led_row.get(col)
                        {
                            BACKLIGHT_CH.sender().send(BacklightCmd::CalibKeyDone(led_idx)).await;
                        }

                        // Progress percentage in 0..=100; saturating at every
                        // step so a zero or overflowing total yields 0 rather
                        // than panicking.
                        let pct =
                            u8::try_from(calibrated_count.saturating_mul(100).checked_div(total_keys).unwrap_or(0))
                                .unwrap_or(100)
                                .min(100);
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
pub(super) async fn sample_full_raw<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    cfg: HallCfg,
    zero_raw: &[[u16; COL]; ROW],
) -> [[u16; COL]; ROW] {
    let mut min_raw = [[u16::MAX; COL]; ROW];
    let total_keys = count_real_sensors(zero_raw).max(1);

    let calibrated_count =
        run_calib_press_phase(cols, seq, buf, zero_raw, &mut min_raw, total_keys, cfg.full_calib_duration).await;

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
    settle_min_raw(cols, seq, buf, &mut min_raw, CALIB_SETTLE_AFTER_ALL_DONE).await;

    min_raw
}

/// Phase B of full-travel calibration: continue updating `min_raw` for
/// `duration` without any state-machine work or backlight signaling, so
/// each key has time to settle to its true bottom-out ADC.
pub(super) async fn settle_min_raw<const ROW: usize, const COL: usize>(
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
                let Some(&row_u8) = valid.rows.get(i) else { continue };
                let row = usize::from(row_u8);
                let raw = buf.get(row).copied().unwrap_or(0);
                min_into(min_raw, row, col, raw);
            }
        }
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
