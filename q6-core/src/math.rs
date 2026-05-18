//! Small saturating arithmetic helpers used by calibration.

/// Ceiling division of `num` by `den`, returning 0 instead of panicking
/// when `den` is zero.
///
/// Used by the firmware's `KeyEntry::apply_calib` for the `inv_scale`
/// Q16.16 reciprocal: rounding up ensures the scaled travel at the
/// full-travel LUT delta hits exactly the full-travel unit after the
/// right-shift instead of one count short. A zero `den` only occurs for a
/// degenerate calibration (`full_lut == zero_lut`) and is treated as
/// "uncalibrated" by the firmware's travel hot path.
#[must_use]
#[inline]
pub const fn ceil_div_or_zero(num: u32, den: u32) -> u32 {
    num.saturating_add(den.saturating_sub(1)).checked_div(den).unwrap_or(0)
}

/// Convert `done`/`total` into a percentage in 0..=100 using saturating
/// arithmetic at every step so a zero or overflowing `total` returns 0
/// instead of panicking.
#[must_use]
#[inline]
pub const fn pct_done(done: usize, total: usize) -> u8 {
    u8::try_from(done.saturating_mul(100).checked_div(total).unwrap_or(0)).unwrap_or(100).min(100)
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
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
}
