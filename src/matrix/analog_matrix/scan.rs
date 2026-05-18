//! Hot-path matrix scan loop.
//!
//! Factored out of `analog_matrix` as a free function (it never needed
//! `self`) so the scanner type keeps a single inherent impl while this code
//! lives in its own module.

use crate::{
    layout::VALID_ROWS_BY_COL,
    matrix::{
        analog_matrix::types::{HallCfg, KeyEntry, VALID_RAW_MAX, VALID_RAW_MIN},
        hc164_cols::Hc164Cols,
    },
};
use core::hint::{cold_path, likely};
use embassy_stm32::{adc::ConfiguredSequence, pac::adc};
use rmk::{
    embassy_futures::yield_now,
    event::{KeyboardEvent, publish_event_async},
};

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
pub(super) async fn run_scan_loop<const ROW: usize, const COL: usize>(
    cols: &mut Hc164Cols<'_>,
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
