use super::{AdcSampleTime, AnalogHallMatrix, HallCfg};
use crate::{
    layout::VALID_ROWS_BY_COL,
    matrix::{
        analog_matrix::types::{
            AUTO_CALIB_CONFIDENCE_THRESHOLD,
            AUTO_CALIB_FULL_TRAVEL_THRESHOLD,
            AUTO_CALIB_MIN_RANGE,
            AUTO_CALIB_RELEASE_THRESHOLD,
            AUTO_CALIB_ZERO_JITTER,
            AutoCalibPhase,
            BOTTOM_JITTER,
            KeyEntry,
            MIN_USEFUL_FULL_RANGE,
            VALID_RAW_MAX,
            VALID_RAW_MIN,
            ZERO_TRAVEL_DEAD_ZONE,
        },
        hc164_cols::Hc164Cols,
    },
};
use core::hint::{cold_path, likely, unlikely};
use embassy_stm32::{
    adc::{BasicInstance, ConfiguredSequence, Instance, RxDma},
    dma::InterruptHandler,
    interrupt::typelevel::Binding,
    pac::adc,
};
use embassy_time::Timer;
use rmk::event::{KeyboardEvent, publish_event_async};

/// Update the auto-calibration state machine for a single key.
///
/// Called on every scan reading that passes the noise gate, before the
/// travel computation and rapid-trigger logic. Watches complete
/// press/release cycles and refines the live calibration once
/// [`AUTO_CALIB_CONFIDENCE_THRESHOLD`] confident cycles have accumulated,
/// compensating for temperature and mechanical drift without requiring a
/// manual re-calibration.
///
/// A cycle is scored only when the ADC range (zero − full) meets
/// [`AUTO_CALIB_MIN_RANGE`]; partial presses or noisy readings are
/// discarded. Updated calibration takes effect immediately on the next
/// travel computation within the same scan pass.
#[inline]
#[optimize(speed)]
fn auto_calib_update(entry: &mut KeyEntry, raw: u16) {
    match entry.ac_phase {
        AutoCalibPhase::Idle => {
            if raw < AUTO_CALIB_FULL_TRAVEL_THRESHOLD {
                entry.ac_full_cand = raw;
                entry.ac_phase = AutoCalibPhase::Pressing;
            }
        },
        AutoCalibPhase::Pressing => {
            if raw < entry.ac_full_cand {
                // Key is pressing deeper; track the running minimum.
                entry.ac_full_cand = raw;
            } else if raw.saturating_sub(entry.ac_full_cand) >= AUTO_CALIB_RELEASE_THRESHOLD {
                // Key has lifted enough above the minimum to count as releasing.
                entry.ac_zero_cand = raw;
                entry.ac_phase = AutoCalibPhase::Releasing;
            } else {
                // Reading is between ac_full_cand and ac_full_cand +
                // AUTO_CALIB_RELEASE_THRESHOLD: bottom-of-travel jitter or
                // a partial early lift. Stay in Pressing until a decisive rise.
            }
        },
        AutoCalibPhase::Releasing => {
            if raw > entry.ac_zero_cand {
                // Key still rising; update the zero-travel peak.
                entry.ac_zero_cand = raw;
            } else {
                // ADC has stopped rising: either settled within jitter (score
                // the cycle) or dropped sharply (key re-pressed mid-release).
                // Both cases return to Idle; only the settled case also scores.
                if entry.ac_zero_cand.saturating_sub(raw) <= AUTO_CALIB_ZERO_JITTER {
                    // ADC has settled within jitter of the zero-travel peak;
                    // score the cycle if the range is plausible.
                    let range = entry.ac_zero_cand.saturating_sub(entry.ac_full_cand);
                    if range >= AUTO_CALIB_MIN_RANGE {
                        entry.ac_confidence = entry.ac_confidence.saturating_add(1);
                    }

                    if entry.ac_confidence >= AUTO_CALIB_CONFIDENCE_THRESHOLD {
                        cold_path();
                        entry.ac_confidence = 0;

                        let new_zero = entry.ac_zero_cand.saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                        let new_full = entry
                            .ac_full_cand
                            .saturating_add(BOTTOM_JITTER)
                            .clamp(VALID_RAW_MIN, new_zero.saturating_sub(MIN_USEFUL_FULL_RANGE));

                        // Only commit the update if the new zero differs
                        // meaningfully from the current one.
                        entry.update_calib_if_drifted(new_zero, new_full);
                    }
                }
                entry.ac_phase = AutoCalibPhase::Idle;
            }
        },
    }
}

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: embassy_stm32::i2c::mode::MasterMode,
    AdcSampleTime<ADC>: Clone,
{
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
    pub(super) async fn run_scan_loop(
        cols: &mut Hc164Cols<'peripherals>,
        keys: &mut [[KeyEntry; ROW]; COL],
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> ! {
        let act_threshold = cfg.actuation_pt;
        // Clamp to 1 so a zero config value never disables the dead-band.
        let sensitivity_press = cfg.rt_sensitivity_press.max(1);
        let sensitivity_release = cfg.rt_sensitivity_release.max(1);
        let trough_floor = act_threshold.saturating_sub(sensitivity_press);
        let col_settle_us = cfg.col_settle_us;
        let noise_gate = cfg.noise_gate;
        loop {
            cols.reset();
            for col in 0..COL {
                Timer::after(col_settle_us).await;
                seq.read(buf).await;

                // Only valid sensor positions are stored in VALID_ROWS_BY_COL;
                // columns with no sensors produce an empty list and are skipped
                // entirely without touching the key-state machine.
                let Some(valid) = VALID_ROWS_BY_COL.get(col) else {
                    cols.advance();
                    continue;
                };

                // Slice to exactly the populated entries (one check instead of
                // one per key), and hoist keys[col] out of the inner loop.
                let Some(valid_keys) = valid.keys.get(..valid.count) else {
                    cols.advance();
                    continue;
                };
                let Some(key_col) = keys.get_mut(col) else {
                    cols.advance();
                    continue;
                };

                for key in valid_keys {
                    let buf_row = usize::from(key.buf_row);
                    let key_row = usize::from(key.key_row);

                    // Clamp raw ADC value to valid range to prevent out-of-bounds
                    // LUT access and ensure valid calibration updates.
                    let raw = buf.get(buf_row).copied().unwrap_or(0).clamp(VALID_RAW_MIN, VALID_RAW_MAX);

                    let Some(entry) = key_col.get_mut(key_row) else { continue };

                    let last_raw = entry.last_raw;

                    // Skip if the reading has not changed beyond the noise gate.
                    if likely(last_raw.abs_diff(raw) < noise_gate) {
                        continue;
                    }

                    // Record the raw value now so that repeated identical readings
                    // are filtered by the noise gate even when travel_from later
                    // returns None (uncalibrated or out-of-range position).
                    entry.last_raw = raw;

                    // Update the auto-calibrator with this reading before the
                    // travel computation so any refined calibration is used immediately.
                    auto_calib_update(entry, raw);

                    let Some(new_travel) = entry.travel_from(raw) else { continue };

                    let prev_travel = entry.travel;
                    let was_pressed = entry.pressed;

                    // After noise gate, travel often resolves to the same
                    // quantized value; skip redundant RT logic.
                    if likely(new_travel == prev_travel) {
                        continue;
                    }

                    entry.travel = new_travel;

                    let below_act = new_travel < act_threshold;

                    // Dynamic Rapid Trigger
                    let now_pressed = if was_pressed {
                        // Track the peak while held; release when travel drops at
                        // least `sensitivity_release` below it. The actuation floor
                        // is a hard lower bound that forces an immediate release.
                        entry.extremum = entry.extremum.max(new_travel);
                        if below_act {
                            false
                        } else {
                            // Only compute when relevant.
                            new_travel > entry.extremum.saturating_sub(sensitivity_release)
                        }
                    } else {
                        // Track the trough while released; re-press when travel
                        // climbs at least `sensitivity_press` above it AND exceeds
                        // the actuation floor. The trough is not required to have
                        // dipped below the floor first, so a finger hovering
                        // mid-travel after an RT-release can re-fire immediately.
                        // When travel drops below the actuation floor the trough is
                        // additionally clamped so the key re-presses cleanly at
                        // `act_threshold` without requiring an extra delta above it.
                        entry.extremum = entry.extremum.min(new_travel);
                        if below_act {
                            entry.extremum = entry.extremum.min(trough_floor);
                            false
                        } else {
                            // Only compute when above floor.
                            new_travel >= entry.extremum.saturating_add(sensitivity_press)
                        }
                    };

                    if unlikely(now_pressed != entry.pressed) {
                        // Reset extremum so the next direction starts fresh from
                        // the transition point.
                        entry.extremum = new_travel;
                        entry.pressed = now_pressed;
                        publish_event_async(KeyboardEvent::key(
                            key.key_row,
                            u8::try_from(col).unwrap_or_default(),
                            now_pressed,
                        ))
                        .await;
                    }
                }
                cols.advance();
            }
        }
    }
}
