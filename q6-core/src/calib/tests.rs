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
use crate::lut::{VALID_RAW_MAX, VALID_RAW_MIN};

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
