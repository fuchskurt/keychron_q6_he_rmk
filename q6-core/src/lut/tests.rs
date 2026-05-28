use super::{LAST_IDX, SPARSE_N, TRAVEL_LUT, VALID_RAW_MAX, VALID_RAW_MIN, lookup};

#[test]
fn clamps_below_min_to_first_entry() {
    let first = TRAVEL_LUT[0];
    assert_eq!(lookup(0), first);
    assert_eq!(lookup(VALID_RAW_MIN.saturating_sub(1)), first);
    assert_eq!(lookup(VALID_RAW_MIN), first);
}

#[test]
fn segment_sample_points_hit_table_entries() {
    // At every exact multiple of SPARSE_N from the origin, interpolation
    // has zero fractional part so the result is the table entry itself.
    for (i, &entry) in TRAVEL_LUT.iter().enumerate().take(LAST_IDX) {
        let offset = u16::try_from(i.saturating_mul(SPARSE_N)).unwrap_or(u16::MAX);
        let raw = VALID_RAW_MIN.saturating_add(offset);
        assert_eq!(lookup(raw), entry, "sample point {i} (raw={raw})");
    }
}

#[test]
fn above_range_saturates_to_endpoint_value() {
    // `lookup(VALID_RAW_MAX)` interpolates within the final (partial)
    // segment; anything at or above the valid maximum clamps to that
    // same endpoint value rather than panicking or wrapping.
    let endpoint = lookup(VALID_RAW_MAX);
    assert_eq!(lookup(VALID_RAW_MAX.saturating_add(1)), endpoint);
    assert_eq!(lookup(VALID_RAW_MAX.saturating_add(100)), endpoint);
    assert_eq!(lookup(u16::MAX), endpoint);
    // The endpoint must not exceed the deep-press maximum (monotonicity
    // sanity at the top of the curve).
    assert!(endpoint <= TRAVEL_LUT[0]);
}

#[test]
fn monotone_non_increasing_across_range() {
    let mut prev = lookup(VALID_RAW_MIN);
    let mut raw = VALID_RAW_MIN;
    while raw < VALID_RAW_MAX {
        raw += 1;
        let cur = lookup(raw);
        assert!(cur <= prev, "non-monotone at raw={raw}: {cur} > {prev}");
        prev = cur;
    }
}

#[test]
fn interpolation_is_between_neighbouring_samples() {
    // A point one count past a sample must lie between that sample and
    // the next (the table is monotone-decreasing).
    for i in 0..LAST_IDX {
        let offset = u16::try_from(i.saturating_mul(SPARSE_N)).unwrap_or(u16::MAX);
        let base = VALID_RAW_MIN.saturating_add(offset);
        if base >= VALID_RAW_MAX {
            break;
        }
        let lo = lookup(base);
        let mid = lookup(base.saturating_add(1));
        assert!(mid <= lo);
    }
}

#[test]
fn lookup_is_usable_in_const_context() {
    const AT_MIN: u16 = lookup(VALID_RAW_MIN);
    assert_eq!(AT_MIN, TRAVEL_LUT[0]);
}
