//! Hot-path matrix scanning: rapid-trigger logic and per-key travel
//! computation.
//!
//! [`auto_calib_update`] and [`travel_from`] are free functions so they can be
//! inlined into [`AnalogHallMatrix::scan_for_next_change`] without going
//! through an extra method dispatch.

use super::{AdcSampleTime, AnalogHallMatrix, AutoCalib, HallCfg, KeyCalib, KeyState, get2, get2_mut};
use crate::matrix::{
    analog_matrix::types::{
        AUTO_CALIB_CONFIDENCE_THRESHOLD,
        AUTO_CALIB_FULL_TRAVEL_THRESHOLD,
        AUTO_CALIB_MIN_RANGE,
        AUTO_CALIB_RELEASE_THRESHOLD,
        AUTO_CALIB_ZERO_JITTER,
        AutoCalibPhase,
        BOTTOM_JITTER,
        FULL_TRAVEL_UNIT,
        MIN_USEFUL_FULL_RANGE,
        VALID_RAW_MAX,
        VALID_RAW_MIN,
        ZERO_TRAVEL_DEAD_ZONE,
    },
    hc164_cols::Hc164Cols,
    sensor_mapping::SENSOR_POSITIONS,
};
use core::hint::{likely, unlikely};
use embassy_stm32::{
    adc::{BasicInstance, ConfiguredSequence, Instance, RxDma},
    dma::InterruptHandler,
    interrupt::typelevel::Binding,
    pac::adc,
};
use embassy_time::Timer;
use num_traits::ToPrimitive as _;
use rmk::event::KeyboardEvent;

/// Update the auto-calibration state machine for a single key.
///
/// Called on every scan reading that passes the noise gate, before the
/// travel computation and rapid-trigger logic. Watches complete
/// press/release cycles and refines `calib[row][col]` once
/// [`AUTO_CALIB_CONFIDENCE_THRESHOLD`] confident cycles have accumulated,
/// compensating for temperature and mechanical drift without requiring a
/// manual re-calibration.
///
/// A cycle is scored only when the ADC range (zero − full) meets
/// [`AUTO_CALIB_MIN_RANGE`]; partial presses or noisy readings are
/// discarded. Updated calibration takes effect immediately on the next
/// travel computation within the same scan pass.
fn auto_calib_update<const ROW: usize, const COL: usize>(
    auto_calib: &mut [[AutoCalib; COL]; ROW],
    calib: &mut [[KeyCalib; COL]; ROW],
    row: usize,
    col: usize,
    raw: u16,
) {
    let Some(ac) = get2_mut(auto_calib, row, col) else { return };

    match ac.phase {
        AutoCalibPhase::Idle => {
            if raw < AUTO_CALIB_FULL_TRAVEL_THRESHOLD {
                ac.full_candidate = raw;
                ac.phase = AutoCalibPhase::Pressing;
            }
        }
        AutoCalibPhase::Pressing => {
            if raw < ac.full_candidate {
                // Key is pressing deeper; track the running minimum.
                ac.full_candidate = raw;
            } else if raw.saturating_sub(ac.full_candidate) > AUTO_CALIB_RELEASE_THRESHOLD {
                // Key has lifted enough above the minimum to count as releasing.
                ac.zero_candidate = raw;
                ac.phase = AutoCalibPhase::Releasing;
            } else {
                // Reading is between full_candidate and full_candidate +
                // AUTO_CALIB_RELEASE_THRESHOLD: bottom-of-travel jitter or
                // a partial early lift. Stay in Pressing until a decisive rise.
            }
        }
        AutoCalibPhase::Releasing => {
            if raw > ac.zero_candidate {
                // Key still rising; update the zero-travel peak.
                ac.zero_candidate = raw;
            } else if ac.zero_candidate.saturating_sub(raw) <= AUTO_CALIB_ZERO_JITTER {
                // ADC has settled within jitter of the zero-travel peak;
                // score the cycle if the range is plausible.
                let range = ac.zero_candidate.saturating_sub(ac.full_candidate);
                if range >= AUTO_CALIB_MIN_RANGE {
                    ac.confidence = ac.confidence.saturating_add(1);
                }

                if ac.confidence >= AUTO_CALIB_CONFIDENCE_THRESHOLD {
                    ac.confidence = 0;

                    let new_zero = ac.zero_candidate.saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                    let new_full = ac
                        .full_candidate
                        .saturating_add(BOTTOM_JITTER)
                        .clamp(VALID_RAW_MIN, new_zero.saturating_sub(MIN_USEFUL_FULL_RANGE));

                    // Only commit the update if the new zero differs
                    // meaningfully from the current one, avoiding
                    // unnecessary churn in the precomputed polynomial
                    // calibration data derived by `KeyCalib::new`.
                    if let Some(cal) = get2_mut(calib, row, col) {
                        if cal.zero.abs_diff(new_zero) > 10 {
                            *cal = KeyCalib::new(new_zero, new_full);
                        }
                    }
                }

                ac.phase = AutoCalibPhase::Idle;
            } else {
                // ADC dropped significantly below zero_candidate - key was
                // re-pressed before fully releasing. Restart the machine.
                ac.phase = AutoCalibPhase::Idle;
            }
        }
    }
}

/// Convert a raw ADC reading into a travel value.
///
/// Returns `None` if the position is uncalibrated, if `raw` is outside
/// [`VALID_RAW_MIN`]..=[`VALID_RAW_MAX`], or if [`KeyCalib::inv_scale`] is
/// not positive (degenerate or uninitialised calibration).
#[inline]
#[optimize(speed)]
fn travel_from(cal: &KeyCalib, raw: u16) -> Option<u8> {
    if !cal.used {
        return None;
    }
    if unlikely(!(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw)) {
        return None;
    }
    if cal.inv_scale <= 0.0 {
        return None;
    }

    let scaled = ((KeyCalib::poly(f32::from(raw)).algebraic_sub(cal.poly_zero)).algebraic_mul(cal.inv_scale))
        .clamp(0.0, f32::from(FULL_TRAVEL_UNIT));
    Some(scaled.to_u8().unwrap_or(FULL_TRAVEL_UNIT))
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
    /// Scan the matrix once, returning the first key state change found.
    ///
    /// The [`ConfiguredSequence`] is programmed once per invocation and
    /// reused across all columns. Both the column settle delay and the DMA
    /// transfer are fully async. For each reading that passes the noise gate
    /// the auto-calibrator is updated before the travel and rapid-trigger
    /// logic runs, so any calibration refinement takes effect within the same
    /// scan pass.
    #[optimize(speed)]
    pub(super) async fn scan_for_next_change(
        cols: &mut Hc164Cols<'peripherals>,
        state: &mut [[KeyState; COL]; ROW],
        calib: &mut [[KeyCalib; COL]; ROW],
        auto_calib: &mut [[AutoCalib; COL]; ROW],
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> Option<KeyboardEvent> {
        let act_threshold = cfg.actuation_pt;
        // Clamp to 1 so a zero config value never disables the dead-band.
        let sensitivity_press = cfg.rt_sensitivity_press.max(1);
        let sensitivity_release = cfg.rt_sensitivity_release.max(1);
        for col in 0..COL {
            cols.select(col);
            Timer::after(cfg.col_settle_us).await;
            seq.read(buf).await;

            for (row, &raw) in buf.iter().enumerate() {
                // Skip positions with no physical hall-effect sensor. Their
                // ADC readings are undefined and must never feed the key-state
                // machine or auto-calibrator.
                if !SENSOR_POSITIONS.get(row).and_then(|r| r.get(col)).copied().unwrap_or(false) {
                    continue;
                }

                let Some(st) = get2_mut(state, row, col) else { continue };
                let last_raw = st.last_raw;

                // Skip if the reading has not changed beyond the noise gate.
                if likely(last_raw.abs_diff(raw) < cfg.noise_gate) {
                    continue;
                }

                // Record the raw value now so that repeated identical readings
                // are filtered by the noise gate even when travel_from later
                // returns None (uncalibrated or out-of-range position).
                st.last_raw = raw;

                // Update the auto-calibrator with this reading before the
                // travel computation so any refined KeyCalib is used immediately.
                auto_calib_update(auto_calib, calib, row, col, raw);

                let Some(cal) = get2(calib, row, col) else { continue };
                let prev_travel = st.travel;
                let was_pressed = st.pressed;
                let Some(new_travel) = travel_from(&cal, raw) else { continue };

                if new_travel == prev_travel {
                    continue;
                }

                st.travel = new_travel;

                // Dynamic Rapid Trigger
                let now_pressed = if was_pressed {
                    // Track the peak while held; release when travel drops at
                    // least `sensitivity_release` below it. The actuation floor
                    // is a hard lower bound that forces an immediate release.
                    st.extremum = st.extremum.max(new_travel);
                    new_travel >= act_threshold && new_travel > st.extremum.saturating_sub(sensitivity_release)
                } else {
                    // Track the trough while released; re-press when travel
                    // climbs at least `sensitivity_press` above it AND exceeds
                    // the actuation floor. The trough is not required to have
                    // dipped below the floor first, so a finger hovering
                    // mid-travel after an RT-release can re-fire immediately.
                    st.extremum = st.extremum.min(new_travel);
                    new_travel >= act_threshold && new_travel >= st.extremum.saturating_add(sensitivity_press)
                };

                if now_pressed != st.pressed {
                    // Reset extremum so the next direction starts fresh from
                    // the transition point.
                    st.extremum = new_travel;
                    st.pressed = now_pressed;
                    return Some(KeyboardEvent::key(
                        u8::try_from(row).unwrap_or_default(),
                        u8::try_from(col).unwrap_or_default(),
                        now_pressed,
                    ));
                }
            }
        }
        None
    }
}
