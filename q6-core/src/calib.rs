//! Pure calibration arithmetic for the Hall-effect key matrix.
//!
//! The hot-path travel formula `(lookup(raw) - lut_zero) * inv_scale >>
//! INV_SCALE_FRAC_BITS` lives here along with the constants it consumes and
//! the helpers that derive `inv_scale` / `lut_zero` from a measured
//! zero/full-travel ADC pair. The firmware's `KeyEntry` is a thin wrapper
//! that stores these fields and delegates the maths to this module.

use crate::{lut, math::ceil_div_or_zero};
use core::hint::{cold_path, unlikely};

#[cfg(test)]
mod tests;

/// Maximum acceptable distance from [`REF_ZERO_TRAVEL`] for a key to be
/// considered to have a valid Hall-effect sensor at its position.
pub const CALIB_ZERO_TOLERANCE: u16 = 500;

/// Expected travel distance in physical units; represents 4.0 mm.
pub const FULL_TRAVEL_UNIT: u8 = 40;

/// Number of fractional bits in the Q16.16 `inv_scale`.
pub const INV_SCALE_FRAC_BITS: u32 = 16;

/// The value of `1.0` in the Q16.16 format used by `inv_scale`.
pub const INV_SCALE_ONE: u32 = 1_u32.checked_shl(INV_SCALE_FRAC_BITS).expect("INV_SCALE_FRAC_BITS < u32::BITS");

/// Precomputed numerator for the Q16.16 `inv_scale` field
/// (`FULL_TRAVEL_UNIT * INV_SCALE_ONE`).
pub const FULL_TRAVEL_SCALED: u32 = u32::from(FULL_TRAVEL_UNIT).saturating_mul(INV_SCALE_ONE);

/// Reference zero-travel ADC value used for calibration and used-sensor
/// validation.
pub const REF_ZERO_TRAVEL: u16 = 3121;

/// ADC counts added to the measured full-travel minimum before storing it as
/// the calibration floor; see [`full_from_min`].
pub const BOTTOM_JITTER: u16 = 80;

/// Minimum ADC delta (zero - full) required to treat a full-travel reading as
/// usable. Guards only against completely flat or absent sensor readings;
/// intentionally small so any genuine press is accepted.
pub const MIN_USEFUL_FULL_RANGE: u16 = 100;

/// Derived calibration fields produced by [`compute`].
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub struct Calib {
    /// Whether this matrix position has a valid Hall-effect sensor.
    pub calib_used: bool,
    /// Raw ADC at zero travel; stored for drift detection.
    pub calib_zero: u16,
    /// Q16.16 reciprocal of the calibrated travel range.
    pub inv_scale:  u32,
    /// LUT value at zero travel, precomputed for fast travel arithmetic.
    pub lut_zero:   u16,
}

/// Compute the derived calibration fields for a key from its measured `zero`
/// and `full`-travel ADC readings.
///
/// Uses ceiling division on the inverse scale so the scaled travel at exactly
/// the full-travel LUT delta reaches [`FULL_TRAVEL_UNIT`] after the
/// right-shift rather than [`FULL_TRAVEL_UNIT`] - 1. `calib_used` is set when
/// the resting reading is within [`CALIB_ZERO_TOLERANCE`] of
/// [`REF_ZERO_TRAVEL`]; positions outside that band are treated as missing
/// sensors and excluded from scanning.
#[must_use]
#[inline]
pub const fn compute(zero: u16, full: u16) -> Calib {
    let zero_lut = lut::lookup(zero);
    let full_lut = lut::lookup(full);
    let delta = u32::from(full_lut.saturating_sub(zero_lut));
    Calib {
        calib_used: zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE,
        calib_zero: zero,
        inv_scale:  ceil_div_or_zero(FULL_TRAVEL_SCALED, delta),
        lut_zero:   zero_lut,
    }
}

/// Convert a raw ADC reading into a travel value, given a key's precomputed
/// `lut_zero`, `inv_scale`, and `calib_used` flag.
///
/// Returns `None` if the position is uncalibrated or `inv_scale == 0`.
/// Otherwise: look up the per-reading LUT value, subtract `lut_zero`,
/// multiply by the Q16.16 `inv_scale`, and right-shift to drop the
/// fractional bits. The final clamp to [`FULL_TRAVEL_UNIT`] keeps
/// bottom-of-travel jitter pinned at the maximum.
#[must_use]
#[inline]
#[optimize(speed)]
pub const fn travel_from(raw: u16, lut_zero: u16, inv_scale: u32, calib_used: bool) -> Option<u8> {
    if unlikely(!calib_used || inv_scale == 0) {
        cold_path();
        return None;
    }
    let delta = lut::lookup(raw).saturating_sub(lut_zero);
    let scaled = u32::from(delta).saturating_mul(inv_scale);
    let travel = scaled.wrapping_shr(INV_SCALE_FRAC_BITS).min(u32::from(FULL_TRAVEL_UNIT));
    Some(u8::try_from(travel).unwrap_or(FULL_TRAVEL_UNIT))
}

/// Compute a calibrated full-travel ADC value from a measured minimum and
/// zero-travel reading.
///
/// Adds [`BOTTOM_JITTER`] to the measured minimum so jitter at the physical
/// bottom does not produce spurious rapid-trigger releases, caps the result
/// at least [`MIN_USEFUL_FULL_RANGE`] below `zero` so a non-trivial travel
/// range survives, then enforces a [`lut::VALID_RAW_MIN`] floor. Shared by
/// both the first-boot calibration (with a default-range fallback for
/// never-pressed keys) and the runtime auto-calibrator.
///
/// Uses `.min(hi).max(lo)` rather than `.clamp(lo, hi)` because the upper
/// bound can fall below the lower bound for pathological sensor values
/// (`zero < VALID_RAW_MIN + MIN_USEFUL_FULL_RANGE`); `clamp` panics in that
/// case, while this ordering safely degrades to [`lut::VALID_RAW_MIN`].
#[must_use]
#[inline]
pub const fn full_from_min(zero: u16, observed_min: u16) -> u16 {
    observed_min
        .saturating_add(BOTTOM_JITTER)
        .min(zero.saturating_sub(MIN_USEFUL_FULL_RANGE))
        .max(lut::VALID_RAW_MIN)
}
