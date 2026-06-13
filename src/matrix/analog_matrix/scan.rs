//! Hot-path matrix scan loop.
//!
//! Factored out of `analog_matrix` as free functions (they never needed
//! `self`) so the scanner type keeps a single inherent impl while this code
//! lives in its own module.
//!
//! The loop is double-buffered (pipelined): while the ADC+DMA converts the
//! currently selected column, the CPU processes the previous column's
//! readings in `process_column`, hiding the per-column processing window
//! behind the DMA transfer that dominates the per-column budget.

use crate::{
    layout::VALID_ROWS_BY_COL,
    matrix::{
        analog_matrix::types::{HallCfg, KeyEntry, RtTuning, VALID_RAW_MAX, VALID_RAW_MIN, coarse_ms_now},
        hc164_cols::Hc164Cols,
    },
    usb_state::usb_is_active,
};
use core::{
    hint::{cold_path, likely, unlikely},
    mem::swap,
};
use embassy_stm32::{adc::ConfiguredSequence, gpio::Output, pac::adc};
use embassy_time::{Duration, Timer};
use rmk::{
    embassy_futures::{join::join, yield_now},
    event::{KeyboardEvent, publish_event_async},
};

/// Interval between matrix passes while the USB bus is suspended or the
/// cable is unplugged.
///
/// Between passes the hall-sensor power rail is cut and the executor sleeps
/// the CPU (WFI), so the keyboard draws only the leakage of an idle MCU plus
/// the LED drivers (already shut down by the backlight task). One pass every
/// interval keeps a held key detectable so it can remote-wake the host, and
/// bounds the added wake-by-keypress latency at one interval.
const SUSPEND_SCAN_INTERVAL: Duration = Duration::from_millis(50);

/// Delay after re-powering the hall-sensor rail before its readings are
/// trusted, letting the sensor outputs settle.
///
/// Hardware-dependent; validate on the board and lengthen if a key ever
/// phantom-wakes the host on resume.
const SENSOR_SETTLE: Duration = Duration::from_millis(2);

/// Number of throwaway matrix passes after re-powering the rail, before the
/// one evaluated pass.
///
/// The first reads after power-up are unreliable (the stock firmware
/// discards several at boot for the same reason); flushing them keeps a
/// power-up transient from being mistaken for a keypress.
const SUSPEND_DISCARD_PASSES: u8 = 3;

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
    // valid_rows() slices to exactly the populated entries (one check
    // instead of one per key); hoist keys[col] out of the inner loop.
    if let Some(valid) = VALID_ROWS_BY_COL.get(col)
        && let Some(key_col) = keys.get_mut(col)
    {
        // One timestamp per column is plenty for the auto-calibrator's
        // ~1 s release-time bound; the whole column processes in microseconds.
        let now = coarse_ms_now();
        for &row_u8 in valid.valid_rows() {
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
            entry.auto_calib_step(raw, now);

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
            if let Some(now_pressed) = entry.step_rapid_trigger(new_travel, tuning) {
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

/// Pipelined hot-path matrix scan loop. Runs forever, publishing key state
/// changes directly via [`publish_event_async`] as they are detected.
///
/// The [`ConfiguredSequence`] is programmed once before entry and reused
/// across all columns and passes. Both the column settle delay and the DMA
/// transfer are fully async. For each reading that passes the noise gate
/// the auto-calibrator is updated before the travel and rapid-trigger
/// logic runs, so any calibration refinement takes effect within the same
/// scan pass.
///
/// Double-buffered: the first poll of [`ConfiguredSequence::read`] inside
/// [`join`] arms the DMA transfer and starts the ADC sequence, then the
/// previous column's readings are processed on the CPU while the conversion
/// proceeds in hardware. This hides the per-column processing window behind
/// the DMA transfer, which development benchmarks measured dominating the
/// per-column budget (~9.8 µs DMA versus ~3.5 µs processing).
///
/// The last column of a pass is processed during the first conversion of the
/// next pass, so the pipeline never needs a separate drain step.
///
/// While the USB bus is suspended (or the cable unplugged) the loop runs in
/// trickle mode: one pass every [`SUSPEND_SCAN_INTERVAL`] with the sensor
/// columns unpowered in between, so a keypress can still remote-wake the
/// host without the keyboard burning full scan power against a sleeping
/// machine.
#[optimize(speed)]
pub(super) async fn run_scan_loop<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    power: &mut Output<'_>,
    cfg: HallCfg,
) -> ! {
    let tuning = RtTuning::from_cfg(cfg);
    let mut prev = [0_u16; ROW];
    let mut prev_col: Option<usize> = None;
    loop {
        if unlikely(!usb_is_active()) {
            cold_path();
            suspend_trickle(cols, keys, seq, buf, power, tuning).await;
            // The rail was power-cycled, so the pipelined `prev` buffer is
            // stale; drop it so the resumed pass does not process a column
            // against pre-suspend readings.
            prev_col = None;
        }
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

/// Low-power suspend loop, entered when the USB bus is suspended or
/// unplugged.
///
/// Each iteration cuts the hall-sensor power rail and naps for
/// [`SUSPEND_SCAN_INTERVAL`] (the executor puts the CPU into WFI sleep while
/// the timer runs), then re-powers the rail, lets it settle, discards the
/// power-up transient, and runs one evaluated pass. A genuinely held key
/// publishes a press on that pass, which RMK turns into a USB remote-wakeup
/// request. Returns once the host is active again, leaving the rail powered
/// so the caller's full-speed loop resumes immediately.
#[optimize(speed)]
async fn suspend_trickle<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    power: &mut Output<'_>,
    tuning: RtTuning,
) {
    while !usb_is_active() {
        // No column selected and the rail off: nothing in the matrix draws
        // current while the CPU sleeps out the interval.
        cols.clear();
        power.set_low();
        Timer::after(SUSPEND_SCAN_INTERVAL).await;

        // Re-power, settle, and flush the power-up transient before trusting
        // a reading.
        power.set_high();
        Timer::after(SENSOR_SETTLE).await;
        for _ in 0..SUSPEND_DISCARD_PASSES {
            trickle_pass(cols, keys, seq, buf, tuning, false).await;
        }
        trickle_pass(cols, keys, seq, buf, tuning, true).await;
    }
}

/// Run one sequential (non-pipelined) matrix pass during suspend trickle.
///
/// Reads every column in turn; when `evaluate` is set, each column's
/// readings are run through [`process_column`] so a held key publishes a
/// press. Discard passes (`evaluate == false`) only flush the ADC and leave
/// the per-key `last_raw` untouched, so the evaluated pass still compares a
/// settled reading against the pre-suspend resting value and the noise gate
/// rejects an unchanged key.
#[optimize(speed)]
async fn trickle_pass<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    tuning: RtTuning,
    evaluate: bool,
) {
    cols.reset();
    for col in 0..COL {
        yield_now().await;
        seq.read(buf).await;
        cols.advance();
        if evaluate {
            process_column(keys, buf, col, tuning).await;
        }
    }
}
