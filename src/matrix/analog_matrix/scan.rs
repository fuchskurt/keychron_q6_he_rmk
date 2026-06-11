//! Hot-path matrix scan loop.
//!
//! Factored out of `analog_matrix` as free functions (they never needed
//! `self`) so the scanner type keeps a single inherent impl while this code
//! lives in its own module.
//!
//! Two loop strategies share the same per-column processing in
//! [`process_column`]:
//!
//! - **Sequential** (default): sample a column, then process it while the next
//!   column's select line settles.
//! - **Pipelined** (`pipelined_scan` feature): double-buffered; the previous
//!   column is processed *while* the ADC converts the current one, hiding the
//!   processing window behind the DMA transfer. Opt-in until the ADC noise
//!   floor with concurrent CPU activity has been validated on hardware.

use crate::{
    layout::VALID_ROWS_BY_COL,
    matrix::{
        analog_matrix::types::{HallCfg, KeyEntry, VALID_RAW_MAX, VALID_RAW_MIN},
        hc164_cols::Hc164Cols,
    },
};
use core::hint::{cold_path, likely};
#[cfg(feature = "pipelined_scan")]
use core::mem::swap;
use embassy_stm32::{adc::ConfiguredSequence, pac::adc};
#[cfg(feature = "pipelined_scan")]
use rmk::embassy_futures::join::join;
use rmk::{
    embassy_futures::yield_now,
    event::{KeyboardEvent, publish_event_async},
};

/// Rapid-trigger tuning values derived once from [`HallCfg`] before the scan
/// loop starts, so the hot path reads pre-clamped constants instead of
/// re-deriving them on every pass.
#[derive(Clone, Copy)]
struct RtTuning {
    /// Minimum travel threshold before a key is considered actuated.
    act_threshold:       u8,
    /// Raw ADC delta below which readings are treated as noise.
    noise_gate:          u16,
    /// Minimum upward travel from the trough required to register a press.
    sensitivity_press:   u8,
    /// Minimum downward travel from the peak required to register a release.
    sensitivity_release: u8,
    /// Lower clamp applied to the released-side extremum so a key driven
    /// below the actuation point re-fires cleanly at the actuation floor.
    trough_floor:        u8,
}

impl RtTuning {
    /// Derive the tuning values from `cfg`.
    ///
    /// Both sensitivities are clamped to 1 so a zero config value never
    /// disables the dead-band.
    const fn from_cfg(cfg: HallCfg) -> Self {
        let act_threshold = cfg.actuation_pt;
        let sensitivity_press = cfg.rt_sensitivity_press.max(1);
        Self {
            act_threshold,
            noise_gate: cfg.noise_gate,
            sensitivity_press,
            sensitivity_release: cfg.rt_sensitivity_release.max(1),
            trough_floor: act_threshold.saturating_sub(sensitivity_press),
        }
    }
}

/// Process one column's ADC readings: noise-gate each populated row, advance
/// the auto-calibrator, recompute travel, run the rapid-trigger state
/// machine, and publish any press/release transitions via
/// [`publish_event_async`].
///
/// `buf` must hold the row readings sampled while `col` was selected.
/// Columns with no sensors produce an empty [`VALID_ROWS_BY_COL`] list and
/// return without touching the key-state machine.
#[optimize(speed)]
async fn process_column<const ROW: usize, const COL: usize>(
    keys: &mut [[KeyEntry; ROW]; COL],
    buf: &[u16; ROW],
    col: usize,
    tuning: RtTuning,
) {
    // Slice to exactly the populated entries (one check instead of
    // one per key), and hoist keys[col] out of the inner loop.
    if let Some(valid) = VALID_ROWS_BY_COL.get(col)
        && let Some(valid_rows) = valid.rows.get(..valid.count)
        && let Some(key_col) = keys.get_mut(col)
    {
        for &row_u8 in valid_rows {
            let row = usize::from(row_u8);

            // Clamp raw ADC value to valid range to prevent out-of-bounds
            // LUT access and ensure valid calibration updates.
            let raw = buf.get(row).copied().unwrap_or(0).clamp(VALID_RAW_MIN, VALID_RAW_MAX);

            let Some(entry) = key_col.get_mut(row) else { continue };

            // Skip if the reading has not changed beyond the noise gate.
            if likely(entry.last_raw.abs_diff(raw) < tuning.noise_gate) {
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
                tuning.act_threshold,
                tuning.sensitivity_press,
                tuning.sensitivity_release,
                tuning.trough_floor,
            ) {
                cold_path();
                publish_event_async(KeyboardEvent::key(
                    row_u8,
                    // The matrix has 21 columns, so `col` always fits
                    // in a u8; u8::MAX is a sentinel that makes any
                    // future overflow obviously wrong.
                    u8::try_from(col).unwrap_or(u8::MAX),
                    now_pressed,
                ))
                .await;
            }
        }
    }
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
#[cfg(not(feature = "pipelined_scan"))]
#[optimize(speed)]
pub(super) async fn run_scan_loop<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    cfg: HallCfg,
) -> ! {
    let tuning = RtTuning::from_cfg(cfg);
    loop {
        cols.reset();
        for col in 0..COL {
            yield_now().await;
            seq.read(buf).await;
            cols.advance();
            process_column(keys, buf, col, tuning).await;
        }
    }
}

/// Pipelined hot-path matrix scan loop. Runs forever, publishing key state
/// changes directly via [`publish_event_async`] as they are detected.
///
/// Double-buffered: the first poll of [`ConfiguredSequence::read`] inside
/// [`join`] arms the DMA transfer and starts the ADC sequence, then the
/// previous column's readings are processed on the CPU while the conversion
/// proceeds in hardware. This hides the per-column processing window behind
/// the DMA transfer, which the readme benchmarks show dominating the
/// per-column budget.
///
/// The last column of a pass is processed during the first conversion of the
/// next pass, so the pipeline never needs a separate drain step.
#[cfg(feature = "pipelined_scan")]
#[optimize(speed)]
pub(super) async fn run_scan_loop<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    cfg: HallCfg,
) -> ! {
    let tuning = RtTuning::from_cfg(cfg);
    let mut prev = [0_u16; ROW];
    let mut prev_col: Option<usize> = None;
    loop {
        cols.reset();
        for col in 0..COL {
            // Column settle delay; also the executor yield point.
            yield_now().await;
            join(seq.read(buf), async {
                if let Some(done_col) = prev_col {
                    process_column(keys, &prev, done_col, tuning).await;
                }
            })
            .await;
            cols.advance();
            swap(buf, &mut prev);
            prev_col = Some(col);
        }
    }
}
