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
    low_power::{arm_rtc_wakeup, stop_nap},
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
/// Between passes the hall-sensor power rail is cut and the core sleeps, so
/// the keyboard draws only the leakage of an idle MCU plus the LED drivers
/// (already shut down by the backlight task). One pass every interval keeps a
/// held key detectable so it can remote-wake the host; it also bounds both
/// the wake-by-keypress latency and the host-initiated resume latency at one
/// interval, so it trades sleep current against how snappy waking feels.
const SUSPEND_SCAN_INTERVAL: Duration = Duration::from_millis(500);

/// Delay after re-powering the hall-sensor rail before its readings are
/// trusted, letting the sensor outputs settle.
///
/// Hardware-dependent; validate on the board and lengthen if a key ever
/// phantom-wakes the host on resume.
const SENSOR_SETTLE: Duration = Duration::from_millis(2);

/// Number of throwaway matrix passes after re-powering the rail, before any
/// reading is trusted.
///
/// The first reads after power-up are unreliable (the stock firmware
/// discards several at boot for the same reason); flushing them keeps a
/// power-up transient from being mistaken for a keypress.
const SUSPEND_DISCARD_PASSES: u8 = 3;

/// Delay between the two press-detection passes that must agree before a
/// suspended keyboard wakes the host.
///
/// A power-up or sensor transient does not survive this gap, but a key a
/// user is actually holding does, so the host only wakes on a real press.
const SUSPEND_CONFIRM_DELAY: Duration = Duration::from_millis(8);

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
    // Arm the RTC wakeup timer that paces STOP-mode naps during suspend.
    arm_rtc_wakeup(SUSPEND_SCAN_INTERVAL);
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
/// the timer runs), then re-powers the rail, lets it settle, and discards the
/// power-up transient. It then looks for a genuinely pressed key, and only
/// when one is confirmed does it run a publishing pass so RMK issues a USB
/// remote-wakeup request. Returns once the host is active again, leaving the
/// rail powered so the caller's full-speed loop resumes immediately.
///
/// Wake detection is deliberately decoupled from the edge-triggered
/// [`process_column`] machine: it tests each key's *absolute* calibrated
/// travel against the actuation point and requires two reads
/// [`SUSPEND_CONFIRM_DELAY`] apart to agree. A settling artifact after the
/// rail powers up therefore cannot wake the host; only a key the user is
/// actually holding does.
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
        // current while the core sleeps in STOP mode (low-microamp) until the
        // RTC wakeup timer brings it back.
        cols.clear();
        power.set_low();
        stop_nap();

        // Re-power, settle, and flush the power-up transient before trusting
        // a reading.
        power.set_high();
        Timer::after(SENSOR_SETTLE).await;
        for _ in 0..SUSPEND_DISCARD_PASSES {
            read_pass::<ROW, COL>(cols, seq, buf).await;
        }

        // Wake only on a real, sustained press: two absolute-travel checks
        // SUSPEND_CONFIRM_DELAY apart must both see a pressed key.
        if !any_key_pressed(cols, keys, seq, buf, tuning.act_threshold).await {
            continue;
        }
        Timer::after(SUSPEND_CONFIRM_DELAY).await;
        if !any_key_pressed(cols, keys, seq, buf, tuning.act_threshold).await {
            continue;
        }

        // Confirmed: run one publishing pass so the held key's press reaches
        // RMK, which raises the remote-wakeup request and registers the key.
        eval_pass(cols, keys, seq, buf, tuning).await;
    }
}

/// Read every column once without touching key state, to flush the ADC and
/// sensor settling transient after the rail is re-powered.
#[optimize(speed)]
async fn read_pass<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
) {
    cols.reset();
    for _ in 0..COL {
        yield_now().await;
        seq.read(buf).await;
        cols.advance();
    }
}

/// Test whether any calibrated key is currently pressed past the actuation
/// point, by its absolute travel rather than any change from a prior reading.
///
/// Reads the whole matrix once without mutating key state, so it is safe to
/// call repeatedly for confirmation and leaves the edge-triggered
/// [`process_column`] machine free to publish the real transition afterwards.
#[optimize(speed)]
async fn any_key_pressed<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &[[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    act_threshold: u8,
) -> bool {
    let mut pressed = false;
    cols.reset();
    for col in 0..COL {
        yield_now().await;
        seq.read(buf).await;
        cols.advance();
        if let Some(valid) = VALID_ROWS_BY_COL.get(col)
            && let Some(key_col) = keys.get(col)
        {
            for &row_u8 in valid.valid_rows() {
                let row = usize::from(row_u8);
                let raw = buf.get(row).copied().unwrap_or(0).clamp(VALID_RAW_MIN, VALID_RAW_MAX);
                if let Some(entry) = key_col.get(row)
                    && let Some(travel) = entry.travel_from(raw)
                    && travel >= act_threshold
                {
                    pressed = true;
                }
            }
        }
    }
    pressed
}

/// Run one sequential (non-pipelined) publishing pass, used once a suspend
/// wake has been confirmed so the held key's press reaches RMK.
#[optimize(speed)]
async fn eval_pass<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    tuning: RtTuning,
) {
    cols.reset();
    for col in 0..COL {
        yield_now().await;
        seq.read(buf).await;
        cols.advance();
        process_column(keys, buf, col, tuning).await;
    }
}
