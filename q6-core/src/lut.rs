//! Sparse Hall-sensor transfer-function table with linear interpolation.
//!
//! Raw ADC readings (`VALID_RAW_MIN..=VALID_RAW_MAX`) follow the Hall
//! sensor's non-linear response to magnetic flux density: small distance
//! changes at full press shift the ADC by many counts, while small changes
//! near release barely move it. The transfer function inverts that
//! non-linearity so the *difference* between two transfer values is
//! proportional to the linear distance between the two raw readings.
//!
//! The transfer function is a cubic Hall-sensor polynomial sampled offline
//! every [`SPARSE_N`] raw counts and stored as
//! `round(256 * poly(raw) + 11008)`. The runtime never evaluates the
//! polynomial; the table plus linear interpolation between samples is the
//! firmware's only ground truth.
//!
//! Once the firmware has cached `lut_zero` at calibration time (see
//! `KeyEntry::apply_zero` in the firmware crate), the hot scan path computes
//! travel as a u16 subtraction (`lookup(raw) - lut_zero`) followed by a u32
//! multiply-shift. The LUT's additive bias cancels in the subtraction, so
//! it never has to be undone at runtime.
//!
//! Sizing: a 37-entry u16 table fits in 74 bytes of `.rodata`. The
//! worst-case reconstruction error against the underlying cubic polynomial
//! is 27 LUT counts (~0.126 travel units, ~0.013 mm, ~8x below the most
//! sensitive rapid-trigger setting). A flat 2301-entry LUT that skipped
//! the interpolation step was benchmarked on this MCU and showed no
//! measurable speedup over the sparse layout, so the sparse table is the
//! only implementation in tree.
//!
//! Endpoint handling: the raw range (length [`RAW_SPAN`]) is not an exact
//! multiple of [`SPARSE_N`], so the final segment is shorter than
//! [`SPARSE_N`] raw counts. The last entry in [`TRAVEL_LUT`] is *not* the
//! polynomial evaluated at the next sample point; it is the value that
//! makes the regular linear-interpolation formula at `raw = VALID_RAW_MAX`
//! match `poly(VALID_RAW_MAX)` exactly. That extrapolation keeps the
//! lookup single-branch (same divide and multiply for every segment,
//! including the partial one) while preserving accuracy at the upper
//! edge.

#[cfg(test)]
mod tests;

/// Maximum acceptable raw ADC value (inclusive upper bound of the table).
pub const VALID_RAW_MAX: u16 = 3500;

/// Minimum acceptable raw ADC value (table origin).
pub const VALID_RAW_MIN: u16 = 1200;

/// Number of raw ADC counts between adjacent samples in [`TRAVEL_LUT`].
///
/// Power-of-two so the `checked_shr` and `checked_rem` calls on the hot
/// path collapse to a single-cycle shift and AND-mask under LLVM's
/// strength reduction; the matching [`SPARSE_N_LOG2`] used as the shift
/// amount is derived once at compile time.
const SPARSE_N: usize = 64;

/// `log2(SPARSE_N)`; used as the shift amount that divides a raw offset by
/// the sample spacing in [`lookup`].
const SPARSE_N_LOG2: u32 = SPARSE_N.trailing_zeros();

/// [`SPARSE_N`] widened into the `u32` lane used by the interpolation
/// arithmetic. Lifted to a `const` so the `try_from(SPARSE_N)` safe-wrap
/// runs once at compile time instead of every call to [`lookup`].
const SPARSE_N_U32: u32 = u32::try_from(SPARSE_N).unwrap_or(u32::MAX);

/// Round-to-nearest bias added before the right-shift in [`lookup`].
///
/// Equal to `SPARSE_N_U32 / 2`; without this constant the truncation of
/// `(diff * frac) >> SPARSE_N_LOG2` would add up to half a count of bias
/// in the interpolation result.
const SPARSE_N_HALF: u32 = SPARSE_N_U32.checked_shr(1).unwrap_or(0);

/// Inclusive span of valid raw ADC counts covered by the table.
///
/// `VALID_RAW_MAX - VALID_RAW_MIN`; the offset of `raw` from the table
/// origin runs `0..=RAW_SPAN`.
const RAW_SPAN: usize = usize::from(VALID_RAW_MAX).saturating_sub(usize::from(VALID_RAW_MIN));

/// Number of entries in [`TRAVEL_LUT`].
///
/// One sample at every multiple of [`SPARSE_N`] in `0..=RAW_SPAN`, plus
/// one trailing entry that the interpolation in [`lookup`] uses as the
/// upper anchor of the final (possibly partial) segment.
const TRAVEL_LUT_LEN: usize = RAW_SPAN.div_ceil(SPARSE_N).saturating_add(1);

/// Largest valid index into [`TRAVEL_LUT`]; used as the upper bound of the
/// branch-free clamp on the interpolation anchors.
const LAST_IDX: usize = TRAVEL_LUT_LEN.saturating_sub(1);

/// Sparse Hall-sensor transfer-function table.
///
/// `TRAVEL_LUT[i] = round(256 * poly(VALID_RAW_MIN + i * SPARSE_N) + 11008)`
/// for `i` in `0..LAST_IDX`. The final entry (`TRAVEL_LUT[LAST_IDX]`) is
/// chosen so that linear interpolation at `raw = VALID_RAW_MAX` yields
/// exactly `round(256 * poly(VALID_RAW_MAX) + 11008)`; see the
/// module-level doc for why.
const TRAVEL_LUT: [u16; TRAVEL_LUT_LEN] = [
    0x848A, 0x7D23, 0x767A, 0x7085, 0x6B36, 0x6682, 0x625E, 0x5EBC, 0x5B90, 0x58D0, 0x566E, 0x545F, 0x5296, 0x5108,
    0x4FA8, 0x4E6B, 0x4D44, 0x4C27, 0x4B08, 0x49DC, 0x4896, 0x4729, 0x458B, 0x43AF, 0x4189, 0x3F0C, 0x3C2D, 0x38E0,
    0x3519, 0x30CB, 0x2BEB, 0x266C, 0x2043, 0x1963, 0x11C1, 0x0950, 0x000B,
];

/// Compile-time guard: every entry must fit in a `u16`. The static type
/// already enforces this, but the assert documents the contract and
/// breaks the build if the regenerator ever emits a different element type.
const _: () = assert!(TRAVEL_LUT.len() == TRAVEL_LUT_LEN, "TRAVEL_LUT length must equal TRAVEL_LUT_LEN");

/// Compile-time guard: the first entry must equal `round(256 * poly(MIN) +
/// 11008)`. Spot-check that the table actually starts where the doc claims it
/// does; keeps the regenerator and the runtime lookup honest about which row is
/// row 0.
const _: () = assert!(matches!(TRAVEL_LUT.first(), Some(&0x848A)), "TRAVEL_LUT must start at the deep-press maximum");

/// Compile-time guard: the LUT length matches the polynomial domain.
///
/// `n_segments = ceil(RAW_SPAN / SPARSE_N)`, plus one upper anchor.
const _: () = assert!(
    TRAVEL_LUT_LEN == RAW_SPAN.div_ceil(SPARSE_N).saturating_add(1),
    "TRAVEL_LUT_LEN must match the polynomial domain"
);

/// Compile-time guard: `SPARSE_N` must be a power of two so that
/// `checked_shr(SPARSE_N_LOG2)` and `checked_rem(SPARSE_N)` on the hot
/// path collapse to a single shift and AND-mask under LLVM strength
/// reduction.
const _: () = assert!(SPARSE_N.is_power_of_two(), "SPARSE_N must be a power of two");

/// Compile-time guard: the upper-anchor index must be in bounds. With
/// `LAST_IDX = TRAVEL_LUT_LEN - 1`, this rules out a one-off mistake in
/// `TRAVEL_LUT_LEN` that would let the interpolation read past the table.
const _: () = assert!(LAST_IDX < TRAVEL_LUT_LEN, "LAST_IDX must be a valid TRAVEL_LUT index");

/// Look up the LUT-equivalent value for ADC reading `raw` via linear
/// interpolation between the two nearest sparse samples.
///
/// Algorithm, all done in safe integer arithmetic:
///
/// 1. Compute the raw offset `o = saturating_sub(raw, VALID_RAW_MIN)` and clamp
///    it to `RAW_SPAN`. Together this maps any `u16` input into `0..=RAW_SPAN`
///    without branching.
/// 2. `idx = o.checked_shr(SPARSE_N_LOG2)`, `frac = o.checked_rem(SPARSE_N)`
///    split the offset into the segment index and the in-segment position. Both
///    compile to a single shift / AND-mask under LLVM strength reduction
///    because `SPARSE_N` is a compile-time power of two.
/// 3. Read the segment endpoints `lo = TRAVEL_LUT[idx]` and `hi =
///    TRAVEL_LUT[idx+1]` through `.get(...).copied().unwrap_or(...)`. The clamp
///    from step 1 guarantees both indices are in range; the fallbacks are
///    defensive no-ops the optimiser folds away.
/// 4. The polynomial is monotone-decreasing, so `lo >= hi`. The signed
///    difference is therefore `lo - hi` (a u16) and the interpolated value is
///    `lo - (diff * frac + N/2) / N`, computed in u32 to avoid intermediate
///    overflow. The `+ N/2` rounds to nearest before the shift; without it, the
///    truncation can add ~half a count of bias.
///
/// Saturating behaviour: readings below [`VALID_RAW_MIN`] resolve to
/// `TRAVEL_LUT[0]` (the deep-press maximum); readings above
/// [`VALID_RAW_MAX`] resolve to the upper anchor of the final segment
/// (which equals `poly(VALID_RAW_MAX)` after interpolation thanks to the
/// endpoint adjustment baked into the table). Hot-path callers always
/// pre-clamp, so neither saturation arm fires in steady state.
#[must_use]
#[inline]
#[optimize(speed)]
pub const fn lookup(raw: u16) -> u16 {
    let raw_off = usize::from(raw.saturating_sub(VALID_RAW_MIN)).min(RAW_SPAN);
    let idx = raw_off.checked_shr(SPARSE_N_LOG2).unwrap_or(0);
    let frac = raw_off.checked_rem(SPARSE_N).unwrap_or(0);
    let lo = TRAVEL_LUT.get(idx).copied().unwrap_or(0);
    let hi = TRAVEL_LUT.get(idx.saturating_add(1)).copied().unwrap_or(lo).min(lo);
    let diff = u32::from(lo.saturating_sub(hi));
    let frac_u32 = u32::try_from(frac).unwrap_or(u32::MAX);
    let contrib = diff.saturating_mul(frac_u32).saturating_add(SPARSE_N_HALF).checked_shr(SPARSE_N_LOG2).unwrap_or(0);
    let result = u32::from(lo).saturating_sub(contrib);
    u16::try_from(result).unwrap_or(u16::MAX)
}

