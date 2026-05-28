use super::{ceil_div_or_zero, pct_done};

#[test]
fn ceil_div_rounds_up_and_guards_zero() {
    assert_eq!(ceil_div_or_zero(0, 0), 0);
    assert_eq!(ceil_div_or_zero(10, 0), 0);
    assert_eq!(ceil_div_or_zero(0, 5), 0);
    assert_eq!(ceil_div_or_zero(10, 5), 2);
    assert_eq!(ceil_div_or_zero(11, 5), 3);
    assert_eq!(ceil_div_or_zero(9, 5), 2);
    assert_eq!(ceil_div_or_zero(1, 1), 1);
    // No overflow panic at the top of the range.
    assert_eq!(ceil_div_or_zero(u32::MAX, u32::MAX), 1);
}

#[test]
fn pct_done_is_clamped_and_zero_safe() {
    assert_eq!(pct_done(0, 0), 0);
    assert_eq!(pct_done(5, 0), 0);
    assert_eq!(pct_done(0, 10), 0);
    assert_eq!(pct_done(5, 10), 50);
    assert_eq!(pct_done(10, 10), 100);
    // More done than total never exceeds 100.
    assert_eq!(pct_done(25, 10), 100);
    assert_eq!(pct_done(1, 3), 33);
}

#[test]
fn helpers_work_in_const_context() {
    const RATIO: u32 = ceil_div_or_zero(7, 2);
    const PCT: u8 = pct_done(3, 4);
    assert_eq!(RATIO, 4);
    assert_eq!(PCT, 75);
}
