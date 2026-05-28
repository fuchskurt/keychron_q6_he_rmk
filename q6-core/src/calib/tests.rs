use super::{
    BOTTOM_JITTER,
    CALIB_ZERO_TOLERANCE,
    FULL_TRAVEL_UNIT,
    MIN_USEFUL_FULL_RANGE,
    REF_ZERO_TRAVEL,
    compute,
    full_from_min,
    travel_from,
};
use crate::lut::{self, VALID_RAW_MAX, VALID_RAW_MIN};

const TYPICAL_FULL_RANGE: u16 = 900;

#[test]
fn full_from_min_adds_bottom_jitter() {
    let zero = REF_ZERO_TRAVEL;
    let observed = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let stored = full_from_min(zero, observed);
    // Lifts the stored floor `BOTTOM_JITTER` above the observed minimum.
    assert_eq!(stored, observed.saturating_add(BOTTOM_JITTER));
}

#[test]
fn full_from_min_clamps_to_min_useful_range_below_zero() {
    // An observed minimum suspiciously close to zero gets pulled down so the
    // travel range is at least MIN_USEFUL_FULL_RANGE counts wide.
    let zero = REF_ZERO_TRAVEL;
    let too_high = zero.saturating_sub(10);
    let stored = full_from_min(zero, too_high);
    assert_eq!(stored, zero.saturating_sub(MIN_USEFUL_FULL_RANGE));
    assert!(zero.saturating_sub(stored) >= MIN_USEFUL_FULL_RANGE);
}

#[test]
fn full_from_min_enforces_lut_min_floor() {
    // Pathological observed minimum below the LUT domain is clamped up.
    let zero = REF_ZERO_TRAVEL;
    let stored = full_from_min(zero, 0);
    assert!(stored >= VALID_RAW_MIN);
}

#[test]
fn full_from_min_degrades_without_panic_when_zero_below_min() {
    // If `zero` is below VALID_RAW_MIN + MIN_USEFUL_FULL_RANGE the upper bound
    // falls below the lower bound. `.clamp` would panic; the helper falls
    // back to VALID_RAW_MIN.
    let stored = full_from_min(VALID_RAW_MIN, VALID_RAW_MIN);
    assert_eq!(stored, VALID_RAW_MIN);
}

#[test]
fn compute_flags_valid_sensor_at_reference_zero() {
    let calib = compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE));
    assert!(calib.calib_used);
    assert_eq!(calib.calib_zero, REF_ZERO_TRAVEL);
    assert!(calib.inv_scale > 0);
}

#[test]
fn compute_rejects_zero_too_far_from_reference() {
    // A position whose resting reading sits well outside the tolerance band
    // is treated as a missing sensor.
    let bad_zero = REF_ZERO_TRAVEL.saturating_add(CALIB_ZERO_TOLERANCE).saturating_add(1);
    let calib = compute(bad_zero, bad_zero.saturating_sub(TYPICAL_FULL_RANGE));
    assert!(!calib.calib_used);
}

#[test]
fn compute_accepts_boundary_of_tolerance() {
    let calib = compute(REF_ZERO_TRAVEL.saturating_add(CALIB_ZERO_TOLERANCE), VALID_RAW_MIN);
    assert!(calib.calib_used);
}

#[test]
fn compute_returns_zero_inv_scale_for_degenerate_range() {
    // zero == full produces a zero LUT delta; ceil_div_or_zero returns 0,
    // which travel_from treats as "uncalibrated".
    let calib = compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL);
    assert_eq!(calib.inv_scale, 0);
}

#[test]
fn travel_from_returns_none_when_uncalibrated() {
    let calib = compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE));
    assert_eq!(travel_from(REF_ZERO_TRAVEL, calib.lut_zero, calib.inv_scale, false), None);
}

#[test]
fn travel_from_returns_none_when_inv_scale_zero() {
    assert_eq!(travel_from(REF_ZERO_TRAVEL, 0, 0, true), None);
}

#[test]
fn travel_from_at_zero_reading_returns_zero_travel() {
    let calib = compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE));
    let travel = travel_from(REF_ZERO_TRAVEL, calib.lut_zero, calib.inv_scale, calib.calib_used);
    assert_eq!(travel, Some(0));
}

#[test]
fn travel_from_at_full_reading_returns_full_travel() {
    let full = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let calib = compute(REF_ZERO_TRAVEL, full);
    let travel = travel_from(full, calib.lut_zero, calib.inv_scale, calib.calib_used);
    assert_eq!(travel, Some(FULL_TRAVEL_UNIT));
}

#[test]
fn travel_from_clamps_readings_past_full_to_full_travel() {
    let full = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let calib = compute(REF_ZERO_TRAVEL, full);
    // Deeper press than calibrated full: result must clamp, not overflow.
    let travel = travel_from(VALID_RAW_MIN, calib.lut_zero, calib.inv_scale, calib.calib_used);
    assert_eq!(travel, Some(FULL_TRAVEL_UNIT));
}

#[test]
fn travel_from_is_monotone_in_press_depth() {
    let full = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let calib = compute(REF_ZERO_TRAVEL, full);
    let mut prev: u8 = 0;
    let mut raw = REF_ZERO_TRAVEL;
    while raw > full {
        let result = travel_from(raw, calib.lut_zero, calib.inv_scale, calib.calib_used);
        assert!(result.is_some(), "travel_from returned None for a calibrated key at raw={raw}");
        let cur = result.unwrap_or(prev);
        assert!(cur >= prev, "non-monotone at raw={raw}: {cur} < {prev}");
        prev = cur;
        raw = raw.saturating_sub(10);
    }
}

#[test]
fn travel_from_respects_lut_bounds() {
    // Readings outside the LUT domain saturate cleanly via the LUT's own clamp.
    let calib = compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE));
    let above = travel_from(VALID_RAW_MAX.saturating_add(100), calib.lut_zero, calib.inv_scale, calib.calib_used);
    assert!(above.is_some());
}

#[test]
fn const_evaluable() {
    const CALIB: super::Calib = compute(REF_ZERO_TRAVEL, REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE));
    const TRAVEL: Option<u8> = travel_from(REF_ZERO_TRAVEL, CALIB.lut_zero, CALIB.inv_scale, CALIB.calib_used);
    assert_eq!(TRAVEL, Some(0));
}

#[test]
fn travel_from_round_trips_endpoints_across_range_of_calibrations() {
    // For any plausible (zero, full) pair, the calibration must round-trip
    // both endpoints: zero → 0 travel, full → FULL_TRAVEL_UNIT travel.
    let full_ranges: [u16; 4] = [200, 500, 900, 1500];
    let zero_offsets: [i16; 5] = [-200, -50, 0, 50, 200];
    for full_range in full_ranges {
        for zero_offset in zero_offsets {
            let zero = REF_ZERO_TRAVEL.saturating_add_signed(zero_offset);
            let full = zero.saturating_sub(full_range);
            if full < lut::VALID_RAW_MIN {
                continue;
            }
            let calib = compute(zero, full);
            if !calib.calib_used {
                continue;
            }
            let at_zero = travel_from(zero, calib.lut_zero, calib.inv_scale, calib.calib_used);
            let at_full = travel_from(full, calib.lut_zero, calib.inv_scale, calib.calib_used);
            assert_eq!(at_zero, Some(0), "zero={zero} full={full}");
            assert_eq!(at_full, Some(FULL_TRAVEL_UNIT), "zero={zero} full={full}");
        }
    }
}

#[test]
fn travel_from_in_bottom_jitter_band_pins_to_full_travel() {
    // A reading at observed_min (i.e. `full - BOTTOM_JITTER`) must produce
    // FULL_TRAVEL_UNIT travel rather than overshooting, since `full_from_min`
    // lifts the stored floor by exactly that amount.
    let observed_min = REF_ZERO_TRAVEL.saturating_sub(TYPICAL_FULL_RANGE);
    let stored_full = full_from_min(REF_ZERO_TRAVEL, observed_min);
    let calib = compute(REF_ZERO_TRAVEL, stored_full);
    // The deeper-than-stored reading must still clamp to FULL_TRAVEL_UNIT.
    let travel = travel_from(observed_min, calib.lut_zero, calib.inv_scale, calib.calib_used);
    assert_eq!(travel, Some(FULL_TRAVEL_UNIT));
}

#[test]
fn full_from_min_with_default_full_range_input_matches_calibrated_full() {
    // `full_from_min(zero, zero - DEFAULT_FULL_RANGE)` is the fallback used
    // when a key was never pressed during first-boot calibration. The
    // computed value plus a fresh zero must yield a calib_used=true entry.
    const DEFAULT_FULL_RANGE: u16 = 900;
    let zero = REF_ZERO_TRAVEL;
    let fallback_full = full_from_min(zero, zero.saturating_sub(DEFAULT_FULL_RANGE));
    let calib = compute(zero, fallback_full);
    assert!(calib.calib_used);
    assert!(calib.inv_scale > 0);
}
