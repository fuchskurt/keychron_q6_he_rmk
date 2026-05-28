//! Small saturating arithmetic helpers used by calibration.

#[cfg(test)] mod tests;

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
