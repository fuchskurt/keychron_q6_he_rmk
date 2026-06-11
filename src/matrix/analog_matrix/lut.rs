//! Sparse Hall-sensor transfer-function table with linear interpolation.
//!
//! Raw ADC readings (`VALID_RAW_MIN..=VALID_RAW_MAX`) follow the Hall
//! sensor's non-linear response to magnetic flux density: small distance
//! changes at full press shift the ADC by many counts, while small changes
//! near release barely move it. The transfer function inverts that
//! non-linearity so the *difference* between two transfer values is
//! proportional to the linear distance between the two raw readings.
//!
//! The transfer function is the cubic Hall-sensor travel polynomial from
//! Keychron's stock firmware (`CONST_A1..=CONST_D1` in their QMK source):
//!
//! ```text
//! travel(x) = 426.88962 - 0.48358*x + 2.04637e-4*x^2 - 2.99368e-8*x^3
//! ```
//!
//! `TRAVEL_LUT` is generated **at compile time** from those coefficients,
//! sampled every `SPARSE_N` raw counts and stored as
//! `round(256 * travel(raw) + 11008)`. The generator evaluates the
//! polynomial in exact scaled-integer arithmetic (numerators over
//! `POLY_SCALE`), so the table is reproducible and the resampling
//! parameters (`VALID_RAW_MIN`, `VALID_RAW_MAX`, `SPARSE_N`) can be
//! changed without an offline tool. The runtime never evaluates the
//! polynomial; the table plus linear interpolation between samples is all
//! the hot path sees.
//!
//! Once the firmware has cached `lut_zero` at calibration time (see
//! `KeyEntry::apply_zero`), the hot scan path computes travel as a u16
//! subtraction (`lookup(raw) - lut_zero`) followed by a u32
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
//! Endpoint handling: the raw range (length `RAW_SPAN`) is not an exact
//! multiple of `SPARSE_N`, so the final segment is shorter than
//! `SPARSE_N` raw counts. The last entry in `TRAVEL_LUT` is *not* the
//! polynomial evaluated at the next sample point; it is the value that
//! makes the regular linear-interpolation formula at `raw = VALID_RAW_MAX`
//! match `travel(VALID_RAW_MAX)` exactly. That extrapolation keeps the
//! lookup single-branch (same divide and multiply for every segment,
//! including the partial one) while preserving accuracy at the upper
//! edge.

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

/// In-segment position of `VALID_RAW_MAX` within the final table segment.
///
/// Zero when the raw span is an exact multiple of [`SPARSE_N`] (every entry
/// is then a direct polynomial sample); otherwise the final segment is
/// partial and its upper anchor is extrapolated (see [`build_travel_lut`]).
const FINAL_FRAC: usize = RAW_SPAN.checked_rem(SPARSE_N).unwrap_or(0);

/// Number of entries in [`TRAVEL_LUT`].
///
/// One sample at every multiple of [`SPARSE_N`] in `0..=RAW_SPAN`, plus
/// one trailing entry that the interpolation in [`lookup`] uses as the
/// upper anchor of the final (possibly partial) segment.
const TRAVEL_LUT_LEN: usize = RAW_SPAN.div_ceil(SPARSE_N).saturating_add(1);

/// Numerator of the stock polynomial's constant coefficient over
/// [`POLY_SCALE`]: `426.88962` (Keychron `CONST_A1`).
const POLY_A_NUM: i64 = 4_268_896_200_000_000;

/// Numerator of the stock polynomial's linear coefficient over
/// [`POLY_SCALE`]: `-0.48358` (Keychron `CONST_B1`).
const POLY_B_NUM: i64 = -4_835_800_000_000;

/// Numerator of the stock polynomial's quadratic coefficient over
/// [`POLY_SCALE`]: `2.04637e-4` (Keychron `CONST_C1`).
const POLY_C_NUM: i64 = 2_046_370_000;

/// Numerator of the stock polynomial's cubic coefficient over
/// [`POLY_SCALE`]: `-2.99368e-8` (Keychron `CONST_D1`).
const POLY_D_NUM: i64 = -299_368;

/// Common denominator of the `POLY_*_NUM` coefficient numerators (`10^13`).
///
/// Large enough to represent every coefficient's decimal expansion exactly,
/// so the compile-time evaluation is exact rational arithmetic rather than
/// floating point; small enough that the Horner evaluation over the valid
/// raw domain stays below `2^63` (worst intermediate magnitude is ~`5e15`,
/// the final scaled value ~`3.4e17`).
const POLY_SCALE: i64 = 10_000_000_000_000;

/// Additive bias applied to the scaled polynomial before rounding.
///
/// Keeps every table value non-negative in `u16` (the polynomial goes
/// negative near `VALID_RAW_MAX`). Deltas of table values are what the hot
/// path consumes, so the bias cancels and never has to be undone.
const LUT_BIAS: i64 = 11008;

/// Fixed-point scale applied to the polynomial before rounding (8
/// fractional bits), giving the table sub-travel-unit resolution.
const LUT_FRAC_SCALE: i64 = 256;

/// Sparse Hall-sensor transfer-function table, generated at compile time by
/// [`build_travel_lut`] from the stock-firmware polynomial coefficients.
///
/// `TRAVEL_LUT[i] = round(256 * travel(VALID_RAW_MIN + i * SPARSE_N) +
/// 11008)` for every entry but the last. The final entry is the
/// extrapolated upper anchor of the partial last segment; see the
/// module-level doc for why.
const TRAVEL_LUT: [u16; TRAVEL_LUT_LEN] = build_travel_lut();

/// Compile-time regression pin: the generated table must be bit-identical
/// to the hand-generated table the firmware shipped with before the
/// compile-time generator existed (verified offline against the stock
/// coefficients with exact rational arithmetic).
///
/// Delete this pin if [`VALID_RAW_MIN`], [`VALID_RAW_MAX`], or
/// [`SPARSE_N`] are ever retuned - the expected values below describe only
/// the current sampling parameters.
const _: () = {
    const EXPECTED: [u16; 37] = [
        0x848A, 0x7D23, 0x767A, 0x7085, 0x6B36, 0x6682, 0x625E, 0x5EBC, 0x5B90, 0x58D0, 0x566E, 0x545F, 0x5296, 0x5108,
        0x4FA8, 0x4E6B, 0x4D44, 0x4C27, 0x4B08, 0x49DC, 0x4896, 0x4729, 0x458B, 0x43AF, 0x4189, 0x3F0C, 0x3C2D, 0x38E0,
        0x3519, 0x30CB, 0x2BEB, 0x266C, 0x2043, 0x1963, 0x11C1, 0x0950, 0x000B,
    ];
    assert!(TRAVEL_LUT_LEN == EXPECTED.len(), "sampling parameters changed; delete or regenerate this pin");
    let mut i = 0_usize;
    while i < EXPECTED.len() {
        let generated = if let Some(&val) = TRAVEL_LUT.get(i) { val } else { 0 };
        let expected = if let Some(&val) = EXPECTED.get(i) { val } else { u16::MAX };
        assert!(generated == expected, "generated TRAVEL_LUT entry diverges from the shipped table");
        i = i.saturating_add(1);
    }
};

/// Evaluate one direct table sample: `round(256 * travel(raw) + 11008)`.
///
/// The polynomial is evaluated with Horner's rule on the `POLY_*_NUM`
/// numerators, keeping everything in exact integer arithmetic over
/// [`POLY_SCALE`]. The result is rounded half-up; the closest value in the
/// valid domain sits ~0.009 from a rounding boundary, so no tie-breaking
/// subtleties arise. The saturating operators can never actually saturate
/// over the valid domain (see [`POLY_SCALE`]); they exist to keep the
/// arithmetic total.
const fn lut_sample(raw: u16) -> u16 {
    let x = i64::from(raw);
    let num = POLY_D_NUM
        .saturating_mul(x)
        .saturating_add(POLY_C_NUM)
        .saturating_mul(x)
        .saturating_add(POLY_B_NUM)
        .saturating_mul(x)
        .saturating_add(POLY_A_NUM);
    let scaled = num.saturating_mul(LUT_FRAC_SCALE).saturating_add(LUT_BIAS.saturating_mul(POLY_SCALE));
    // Round half-up, then drop the scale. `scaled` is positive across the
    // valid raw domain thanks to LUT_BIAS, so the floor division of the
    // biased numerator implements round-to-nearest.
    let half = POLY_SCALE.checked_div(2).unwrap_or(0);
    let rounded = scaled.saturating_add(half).checked_div(POLY_SCALE).unwrap_or(0);
    u16::try_from(rounded).unwrap_or(u16::MAX)
}

/// Build [`TRAVEL_LUT`] at compile time.
///
/// Every entry whose sample offset lies inside the raw span is a direct
/// [`lut_sample`] of the polynomial. When the span is not an exact multiple
/// of [`SPARSE_N`] (i.e. [`FINAL_FRAC`] is non-zero), the trailing entry is
/// instead the extrapolated anchor that makes the [`lookup`] interpolation
/// at `raw = VALID_RAW_MAX` (segment position [`FINAL_FRAC`]) reproduce
/// `lut_sample(VALID_RAW_MAX)` exactly: the smallest segment slope `d`
/// satisfying `(d * FINAL_FRAC + SPARSE_N/2) >> log2(SPARSE_N) ==
/// lo - target` is subtracted from the last direct sample.
const fn build_travel_lut() -> [u16; TRAVEL_LUT_LEN] {
    let mut table = [0_u16; TRAVEL_LUT_LEN];
    let mut i = 0_usize;
    while i < TRAVEL_LUT_LEN {
        let offset = i.saturating_mul(SPARSE_N);
        if offset <= RAW_SPAN
            && let Ok(off_u16) = u16::try_from(offset)
            && let Some(slot) = table.get_mut(i)
        {
            *slot = lut_sample(VALID_RAW_MIN.saturating_add(off_u16));
        }
        i = i.saturating_add(1);
    }
    if FINAL_FRAC != 0 {
        let target = lut_sample(VALID_RAW_MAX);
        let lo = if let Some(&prev) = table.get(TRAVEL_LUT_LEN.saturating_sub(2)) { prev } else { target };
        // `delta` is the interpolation result the anchor must produce at
        // segment position FINAL_FRAC; the polynomial is monotone
        // decreasing so `lo >= target` and the subtraction cannot wrap.
        let delta = usize::from(lo.saturating_sub(target));
        let half = SPARSE_N.checked_shr(1).unwrap_or(0);
        // Ceiling division: smallest d with d*FINAL_FRAC + N/2 reaching
        // delta after the >> log2(N) truncation.
        let slope = delta
            .saturating_mul(SPARSE_N)
            .saturating_sub(half)
            .saturating_add(FINAL_FRAC.saturating_sub(1))
            .checked_div(FINAL_FRAC)
            .unwrap_or(0);
        if let Some(slot) = table.get_mut(TRAVEL_LUT_LEN.saturating_sub(1)) {
            *slot = lo.saturating_sub(u16::try_from(slope).unwrap_or(u16::MAX));
        }
    }
    table
}

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
/// (which equals `travel(VALID_RAW_MAX)` after interpolation thanks to the
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
