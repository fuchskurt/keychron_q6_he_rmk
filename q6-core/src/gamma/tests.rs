use super::correct;

#[test]
fn endpoints_match_linear_range() {
    assert_eq!(correct(0), 0);
    assert_eq!(correct(255), 255);
}

#[test]
fn monotone_non_decreasing() {
    let mut prev = correct(0);
    for value in 1_u8..=u8::MAX {
        let cur = correct(value);
        assert!(cur >= prev, "non-monotone at {value}: {cur} < {prev}");
        prev = cur;
    }
}

#[test]
fn dim_values_compress_toward_zero() {
    // A gamma 2.2 curve must darken the low end of the range significantly:
    // half-input should produce well under half-output.
    assert!(correct(64) < 32);
    assert!(correct(128) < 96);
}

#[test]
fn bright_values_expand_toward_full() {
    // Near the top of the range the curve is steeper than linear; the second
    // half of inputs should cover more than half of the output range.
    let mid = correct(128);
    let top = correct(255);
    assert!(top.saturating_sub(mid) > 128);
}

#[test]
fn is_usable_in_const_context() {
    const ZERO: u8 = correct(0);
    const MAX: u8 = correct(u8::MAX);
    assert_eq!(ZERO, 0);
    assert_eq!(MAX, 255);
}
