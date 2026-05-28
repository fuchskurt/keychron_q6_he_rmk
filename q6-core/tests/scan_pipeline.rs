//! End-to-end host tests for the firmware's hot scan path.
//!
//! Wires together the same q6-core pieces the firmware composes inside
//! `run_scan_loop`: [`calib::compute`] to derive per-key calibration from a
//! zero/full ADC pair, [`calib::travel_from`] to convert each raw reading to
//! a travel value, and [`rapid_trigger::step`] to detect press/release
//! transitions.
#![expect(
    clippy::tests_outside_test_module,
    reason = "integration test files are themselves the test module"
)]

use q6_core::{
    calib::{self, FULL_TRAVEL_UNIT, REF_ZERO_TRAVEL},
    lut::{VALID_RAW_MAX, VALID_RAW_MIN},
    rapid_trigger,
};

const TYPICAL_FULL_RANGE: u16 = 900;
const ACT_THRESHOLD: u8 = 10;
const PRESS_SENS: u8 = 5;
const RELEASE_SENS: u8 = 3;
const TROUGH_FLOOR: u8 = 8;

/// Walk a sequence of raw ADC readings through the same chain the firmware's
/// scan loop runs, collecting any (press/release) transitions reported by
/// the rapid-trigger stepper.
fn simulate(zero: u16, full: u16, readings: &[u16]) -> Vec<bool> {
    let calib_data = calib::compute(zero, full);
    let mut pressed = false;
    let mut extremum = u8::MAX;
    let mut events = Vec::new();
    for &raw in readings {
        let clamped = raw.clamp(VALID_RAW_MIN, VALID_RAW_MAX);
        let Some(travel) = calib::travel_from(clamped, calib_data.lut_zero, calib_data.inv_scale, calib_data.calib_used)
        else {
            continue;
        };
        if let Some(now_pressed) = rapid_trigger::step(
            &mut pressed,
            &mut extremum,
            travel,
            ACT_THRESHOLD,
            PRESS_SENS,
            RELEASE_SENS,
            TROUGH_FLOOR,
        ) {
            events.push(now_pressed);
        }
    }
    events
}

#[test]
fn slow_press_release_emits_one_press_and_one_release() {
    // Linear ramp from resting to full press and back.
    let mut readings: Vec<u16> = (0_u16..=TYPICAL_FULL_RANGE)
        .step_by(20)
        .map(|delta| REF_ZERO_TRAVEL.saturating_sub(delta))
        .collect();
    readings.extend(
        (0_u16..=TYPICAL_FULL_RANGE)
            .step_by(20)
            .map(|delta| REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE).saturating_add(delta)),
    );
    let events = simulate(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE), &readings);
    assert_eq!(events, vec![true, false], "events: {events:?}");
}

#[test]
fn full_travel_bottom_out_pins_to_full_travel_unit_throughout_hold() {
    let calib_data = calib::compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE));
    let deeper = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE.saturating_add(50));
    let at_full = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    for raw in [at_full, deeper, VALID_RAW_MIN] {
        let travel = calib::travel_from(raw, calib_data.lut_zero, calib_data.inv_scale, calib_data.calib_used);
        assert_eq!(travel, Some(FULL_TRAVEL_UNIT), "raw={raw}");
    }
}

#[test]
fn rapid_tap_emits_two_press_events_in_sequence() {
    // Two complete press/release cycles within the calibrated range.
    let pressed_depth = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let readings = [REF_ZERO_TRAVEL, pressed_depth, REF_ZERO_TRAVEL, pressed_depth, REF_ZERO_TRAVEL];
    let events = simulate(REF_ZERO_TRAVEL, pressed_depth, &readings);
    assert_eq!(events, vec![true, false, true, false], "events: {events:?}");
}

#[test]
fn micro_jitter_below_noise_does_not_change_pressed_state() {
    // Readings drifting within the noise band around resting must never
    // produce a press event, even with a fully calibrated key.
    let offsets: [i16; 9] = [-5, 5, -3, 3, -1, 1, 0, -2, 2];
    let readings: Vec<u16> = offsets.iter().map(|&off| REF_ZERO_TRAVEL.saturating_add_signed(off)).collect();
    let events = simulate(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE), &readings);
    assert!(events.is_empty(), "expected no events from micro-jitter, got {events:?}");
}

#[test]
fn uncalibrated_key_emits_no_events() {
    // A key whose resting reading is far outside CALIB_ZERO_TOLERANCE is
    // marked calib_used=false; travel_from returns None and the scan path
    // emits no events even on deep presses.
    let way_off_zero = REF_ZERO_TRAVEL.saturating_add(2000);
    let calib_data = calib::compute(way_off_zero, way_off_zero.saturating_sub(TYPICAL_FULL_RANGE));
    assert!(!calib_data.calib_used);
    let pressed = way_off_zero.saturating_sub(TYPICAL_FULL_RANGE);
    let events = simulate(way_off_zero, pressed, &[way_off_zero, pressed, way_off_zero]);
    assert!(events.is_empty());
}

#[test]
fn rt_release_then_repress_within_same_press_emits_three_events() {
    // Press deep, RT-release on a moderate drop without going below the
    // actuation threshold, then re-press by rising past the new trough +
    // sensitivity_press.
    let pressed_depth = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let partial_release_raw = REF_ZERO_TRAVEL.saturating_sub(400);
    let repress_raw = REF_ZERO_TRAVEL.saturating_sub(600);
    let readings = [REF_ZERO_TRAVEL, pressed_depth, partial_release_raw, repress_raw];
    let events = simulate(REF_ZERO_TRAVEL, pressed_depth, &readings);
    // Press → RT-release → re-press = three events.
    assert_eq!(events, vec![true, false, true], "events: {events:?}");
}
