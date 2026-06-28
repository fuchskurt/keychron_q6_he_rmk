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
    usb_state::{UsbReceiver, wait_active},
};
use core::{
    hint::{cold_path, likely},
    mem::swap,
};
use embassy_stm32::{adc::ConfiguredSequence, exti::ExtiInput, gpio::Output, mode::Async, pac::adc};
use embassy_time::{Duration, Timer};
use rmk::{
    embassy_futures::{
        join::join,
        select::{Either, select},
        yield_now,
    },
    event::{KeyboardEvent, publish_event_async},
};

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

/// Pipelined full-rate scan body. Runs until cancelled by the supervisor's
/// `select`; `prev`/`prev_col` are local so each (re)entry after a resume
/// starts a fresh pipeline rather than processing a column against stale,
/// pre-suspend readings.
///
/// Double-buffered: the first poll of [`ConfiguredSequence::read`] inside
/// [`join`] arms the DMA transfer and starts the ADC sequence, then the
/// previous column's readings are processed on the CPU while the conversion
/// proceeds in hardware. This hides the per-column processing window behind
/// the DMA transfer, which development benchmarks measured dominating the
/// per-column budget (~9.8 µs DMA versus ~3.5 µs processing).
#[optimize(speed)]
async fn active_scan<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    tuning: RtTuning,
) -> ! {
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

/// Confirm a real, sustained press: two absolute-travel checks
/// [`SUSPEND_CONFIRM_DELAY`] apart must both see a pressed key, so a sensor
/// settling artifact after the rail powers up cannot wake the host.
#[optimize(speed)]
async fn confirmed_press<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &[[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    act_threshold: u8,
) -> bool {
    if !any_key_pressed(cols, keys, seq, buf, act_threshold).await {
        return false;
    }
    Timer::after(SUSPEND_CONFIRM_DELAY).await;
    any_key_pressed(cols, keys, seq, buf, act_threshold).await
}

/// Event-driven scan supervisor: full-rate scan while the host is awake, park
/// on the PC5 hardware key-wake interrupt while suspended. No USB-state
/// polling and no periodic trickle scan — the CPU sits in WFI through suspend
/// and wakes only on the resume event or a key-wake edge.
///
/// While suspended the sensor rail (PC13) is cut. This assumes the PC5 wake
/// line still asserts on a keypress with the rail unpowered; if your board's
/// detect needs the rail powered, hold `power` high through the suspend block
/// instead and drop the re-power / settle / discard steps.
#[optimize(speed)]
pub(super) async fn run<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
    keys: &mut [[KeyEntry; ROW]; COL],
    seq: &mut ConfiguredSequence<'_, adc::Adc>,
    buf: &mut [u16; ROW],
    power: &mut Output<'_>,
    wake: &mut ExtiInput<'_, Async>,
    usb: &mut UsbReceiver,
    cfg: HallCfg,
) -> ! {
    let tuning = RtTuning::from_cfg(cfg);

    // Hold off scanning until the host first configures the device.
    wait_active(usb, true).await;

    loop {
        // Awake: full-rate scan until the host suspends. Dropping the scan
        // future here is cancellation-safe — the in-flight ADC DMA is aborted
        // on drop, and `prev`/`prev_col` are reset on the next `active_scan`.
        match select(active_scan(cols, keys, seq, buf, tuning), wait_active(usb, false)).await {
            Either::First(never) => match never {},
            Either::Second(()) => {},
        }

        // Suspended: rail off, park on the PC5 rising edge.
        cols.set_low_power();
        power.set_low();
        loop {
            match select(wait_active(usb, true), wake.wait_for_falling_edge()).await {
                Either::First(()) => break, // host resumed on its own
                Either::Second(()) => {
                    cold_path();
                    // Re-arm the matrix: restore outputs, power, settle, flush.
                    power.set_high();
                    Timer::after(SENSOR_SETTLE).await;
                    cols.set_active();
                    for _ in 0..SUSPEND_DISCARD_PASSES {
                        read_pass::<ROW, COL>(cols, seq, buf).await;
                    }
                    if confirmed_press(cols, keys, seq, buf, tuning.act_threshold).await {
                        // Publishing pass raises RMK's remote-wakeup request;
                        // the host resumes and the outer wait_active(true)
                        // breaks us out. Leave the rail powered for it.
                        eval_pass(cols, keys, seq, buf, tuning).await;
                    } else {
                        // Spurious edge: park again.
                        cols.set_low_power();
                        power.set_low();
                    }
                },
            }
        }

        // Resumed: restore outputs and power before active_scan restarts.
        power.set_high();
        Timer::after(SENSOR_SETTLE).await;
        cols.set_active();
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
