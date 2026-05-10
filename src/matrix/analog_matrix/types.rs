use crate::matrix::analog_matrix::lut::TRAVEL_LUT;
use core::hint::{cold_path, unlikely};
use embassy_stm32::adc::{BasicAdcRegs, BasicInstance};
use embassy_time::{Duration, Instant};

/// Number of confident press/release cycles required before the
/// auto-calibrator updates the live calibration data for a key.
///
/// Low values react quickly to genuine sensor drift but risk committing a
/// noisy or partial-press cycle; higher values are more conservative. Three
/// cycles gives a good balance between responsiveness and stability.
pub(crate) const AUTO_CALIB_CONFIDENCE_THRESHOLD: u8 = 3;

/// Bottom-of-travel ADC jitter tolerance subtracted from [`DEFAULT_FULL_RANGE`]
/// when computing [`AUTO_CALIB_RELEASE_THRESHOLD`].
///
/// A key in the pressing phase must rise at least
/// [`DEFAULT_FULL_RANGE`] − [`AUTO_CALIB_FULL_JITTER`] counts from its minimum
/// before release tracking begins. This margin allows for typical jitter at
/// the physical bottom without prematurely advancing to the releasing phase.
pub(crate) const AUTO_CALIB_FULL_JITTER: u16 = 100;

/// ADC value below which a reading is considered a genuine full-travel press
/// for auto-calibration purposes.
pub(crate) const AUTO_CALIB_FULL_TRAVEL_THRESHOLD: u16 =
    REF_ZERO_TRAVEL.saturating_sub(DEFAULT_FULL_RANGE).saturating_add(100);

/// Minimum ADC range (zero − full) for a complete press/release cycle to be
/// scored as confident during auto-calibration.
///
/// Cycles with a narrower range are likely partial presses or ADC noise and
/// are discarded without incrementing the confidence counter.
pub(crate) const AUTO_CALIB_MIN_RANGE: u16 = 900;

/// Minimum ADC rise from the full-travel minimum required to transition from
/// the pressing phase to the releasing phase during auto-calibration.
///
/// Set to [`DEFAULT_FULL_RANGE`] − [`AUTO_CALIB_FULL_JITTER`] so the key must
/// have risen most of the way back toward zero before release tracking begins,
/// preventing partial lifts from being counted.
pub(crate) const AUTO_CALIB_RELEASE_THRESHOLD: u16 = DEFAULT_FULL_RANGE.saturating_sub(AUTO_CALIB_FULL_JITTER);

/// Maximum ADC drop from the current zero-travel peak that is treated as
/// resting-position jitter during the releasing phase of auto-calibration.
///
/// Once the ADC has settled within this band below the tracked peak, the
/// press/release cycle is scored and the machine returns to idle.
pub(crate) const AUTO_CALIB_ZERO_JITTER: u16 = 50;

/// Minimum ADC difference between the current zero-travel calibration and a
/// newly observed zero candidate required before committing an update.
///
/// Prevents small fluctuations from repeatedly rewriting calibration constants
/// when the resting position shifts only within normal noise or drift bounds.
/// Acts as a hysteresis band on zero updates, reducing churn in the derived
/// constants and avoiding unnecessary recomputation.
pub(crate) const AUTO_CALIB_ZERO_UPDATE_THRESHOLD: u16 = 10;

/// ADC counts added to the measured full-travel minimum before storing it as
/// the calibration floor.
///
/// Without this offset the calibration floor sits at the true physical bottom.
/// ADC jitter that lifts the key slightly off the bottom raises the raw ADC
/// reading by a few counts, which causes `travel_from` to return a value just
/// below [`FULL_TRAVEL_UNIT`]. With `rt_sensitivity_release` at 1 that dip
/// satisfies the rapid-trigger release condition, firing a spurious release
/// while the key is still physically held down.
///
/// Raising the stored floor [`BOTTOM_JITTER`] ADC counts above the physical
/// minimum ensures any reading in that band produces travel
/// ≥ [`FULL_TRAVEL_UNIT`], which clamps to [`FULL_TRAVEL_UNIT`]. Bottom
/// jitter therefore leaves travel pinned at the maximum while the key is
/// bottomed out, preventing phantom RT releases.
pub(crate) const BOTTOM_JITTER: u16 = 80;

/// Duration a key must remain continuously pressed past
/// [`CALIB_PRESS_THRESHOLD`] before it is accepted as a valid full-travel
/// sample and its LED turns green.
pub(crate) const CALIB_HOLD_DURATION_MS: u64 = 1000;

/// Minimum ADC delta (zero − raw) required to consider a key genuinely pressed
/// during live full-travel calibration sampling.
///
/// A real full-travel press produces ~900 counts of delta, so 450 requires
/// roughly half travel, large enough to reject ADC noise (~100 counts) while
/// still triggering well before the physical bottom.
pub const CALIB_PRESS_THRESHOLD: u16 = 450;

/// Extra sampling time after all keys have been accepted during full-travel
/// calibration, allowing each key's minimum ADC to settle at its true
/// physical bottom rather than the first crossing of [`CALIB_PRESS_THRESHOLD`].
pub(crate) const CALIB_SETTLE_AFTER_ALL_DONE: Duration = Duration::from_millis(2000);

/// Maximum acceptable distance from [`REF_ZERO_TRAVEL`] for a key to be
/// considered to have a valid hall-effect sensor at its position.
pub(crate) const CALIB_ZERO_TOLERANCE: u16 = 500;

/// Default full-range calibration delta used when no better value is available.
pub(crate) const DEFAULT_FULL_RANGE: u16 = 900;

/// Precomputed numerator for the Q16.16 `inv_scale` field.
pub(crate) const FULL_TRAVEL_SCALED: u32 = u32::from(FULL_TRAVEL_UNIT).saturating_mul(INV_SCALE_ONE);

/// Expected travel distance in physical units, represents 4.0 mm.
pub(crate) const FULL_TRAVEL_UNIT: u8 = 40;

/// Number of bits in the Q16.16 representation of [`KeyEntry::inv_scale`].
pub(crate) const INV_SCALE_FRAC_BITS: u32 = 16;
/// The value of `1.0` in the Q16.16 format used by [`KeyEntry::inv_scale`].
pub(crate) const INV_SCALE_ONE: u32 = 1_u32.checked_shl(INV_SCALE_FRAC_BITS).expect("INV_SCALE_FRAC_BITS < u32::BITS");

/// Minimum ADC delta (zero − full) required to treat a full-travel reading as
/// usable. Guards only against completely flat or absent sensor readings;
/// intentionally small so any genuine press is accepted.
pub(crate) const MIN_USEFUL_FULL_RANGE: u16 = 100;

/// Reference zero-travel ADC value used for calibration and used-sensor
/// validation.
pub const REF_ZERO_TRAVEL: u16 = 3121;

/// Maximum acceptable raw ADC value.
pub(crate) const VALID_RAW_MAX: u16 = 3500;

/// Minimum acceptable raw ADC value.
pub(crate) const VALID_RAW_MIN: u16 = 1200;

/// ADC counts subtracted from the averaged zero-travel reading before storing
/// it as the calibration zero point.
///
/// Places the resting zero point slightly below the true ADC average so the
/// key sits cleanly at zero travel while fully released. Without this margin,
/// thermal or mechanical drift that nudges the resting ADC upward would
/// immediately produce a small non-zero travel reading, feeding spurious
/// input into the rapid-trigger extremum tracker.
pub(crate) const ZERO_TRAVEL_DEAD_ZONE: u16 = 20;

/// ADC sample-time type associated with the selected ADC peripheral.
pub(crate) type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

/// Phase the auto-calibrator is currently in for a single key.
///
/// The state machine follows `Idle → Pressing → Releasing → Idle`. A cycle
/// is scored only when the key completes a genuine full-press and a clean
/// release. A re-press that occurs during the releasing phase resets to `Idle`
/// without scoring; an insufficient lift from full travel stays in `Pressing`
/// until the key rises far enough to enter `Releasing`.
///
/// `repr(u8)` pins the discriminant to one byte, matching the role of this
/// field inside the `repr(C)` [`KeyEntry`] struct.
#[repr(u8)]
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub(crate) enum AutoCalibPhase {
    /// Waiting for the key to be pressed past
    /// [`AUTO_CALIB_FULL_TRAVEL_THRESHOLD`].
    #[default]
    Idle,
    /// Key is pressed; tracking the minimum ADC reading (deepest travel).
    Pressing,
    /// Key is releasing; tracking the maximum ADC reading (zero travel).
    Releasing,
}

/// Configuration parameters for hall-effect key sensing.
#[derive(Clone, Copy, Default)]
pub struct HallCfg {
    /// Minimum travel threshold in mm/10 units before a key is considered
    /// actuated.
    pub actuation_pt:           u8           = 8,
    /// Number of full-matrix passes averaged together during zero-travel
    /// calibration.
    pub calib_passes:           u32          = 512,
    /// Time to wait after selecting a new column before reading ADC values,
    /// allowing the analog bus to settle.
    pub col_settle_us:          Duration     = Duration::from_micros(1),
    /// Duration of the full-travel sampling window during first-boot
    /// calibration.
    pub full_calib_duration:    Duration     = Duration::from_secs(360),
    /// Raw ADC delta below which readings are treated as noise and discarded.
    pub noise_gate:             u16          = 10,
    /// Minimum upward travel from the trough required to register a new press,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_press:   u8           = 3,
    /// Minimum downward travel from the peak required to register a release,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_release: u8           = 3,
}

/// Hold-detection state for a single key during the full-travel calibration
/// pass.
///
/// The state machine transitions:
/// [`KeyCalibState::Waiting`] -> [`KeyCalibState::Holding`] (t) when the key
/// crosses [`CALIB_PRESS_THRESHOLD`]; [`KeyCalibState::Holding`] (t) ->
/// [`KeyCalibState::Waiting`] if the key is released before
/// [`CALIB_HOLD_DURATION_MS`]; [`KeyCalibState::Holding`] (t) ->
/// [`KeyCalibState::Accepted`] when the hold duration is satisfied.
#[derive(Clone, Copy)]
pub(crate) enum KeyCalibState {
    /// Hold duration satisfied; this key is fully calibrated.
    Accepted,
    /// Key crossed the threshold at the stored [`Instant`]; hold timer is
    /// running.
    Holding(Instant),
    /// Key has not yet crossed the press threshold, or was released and reset.
    Waiting,
}

/// All per-key data accessed on every scan iteration, in a single flat struct.
///
/// Fields are ordered hot-to-cold so the first 12 bytes (offsets 0–11) cover
/// every field touched on a noise-gated scan pass (`last_raw`, `lut_zero`,
/// `inv_scale`, `travel`, `extremum`, `pressed`, `calib_used`).  A single
/// 64-byte cache-line load therefore services the entire common path.
///
/// `repr(C)` preserves declaration order, guaranteeing the documented byte
/// offsets.  Without it the compiler is free to reorder fields for alignment,
/// which could scatter the hot fields across multiple cache lines.
///
/// The matrix stores entries column-major as `[[KeyEntry; ROW]; COL]` so all
/// row entries for one HC164 column are contiguous; a column scan needs at
/// most three cache lines for all six keys rather than six scattered loads.
///
/// Initialise via [`Default`]; call [`KeyEntry::apply_zero`] with the measured
/// resting ADC after loading EEPROM or completing first-boot calibration.
#[repr(C)]
#[derive(Default)]
pub(crate) struct KeyEntry {
    // ── hot: read/written on every noise-gated scan pass ─────────────────
    /// Raw ADC from the previous scan cycle (noise gate filter).
    /// `u16::MAX` on first boot so the first real reading always passes.
    pub last_raw:      u16 = u16::MAX,
    /// LUT value at zero travel, precomputed for fast travel arithmetic.
    pub lut_zero:      u16,
    /// Q16.16 reciprocal of the calibrated travel range.
    pub inv_scale:     u32,
    /// Quantised travel value from the previous scan cycle.
    pub travel:        u8,
    /// Local extremum for rapid-trigger (peak while pressed, trough while
    /// released). Reset to `new_travel` on every press↔release transition.
    pub extremum:      u8 = u8::MAX,
    /// Whether the key is currently considered pressed.
    pub pressed:       bool,
    /// Whether this matrix position has a valid hall-effect sensor.
    pub calib_used:    bool,
    // ── auto-calibration: updated per scored press/release cycle ──────────
    /// Candidate zero-travel ADC peak tracked during the releasing phase.
    pub ac_zero_cand:  u16,
    /// Candidate full-travel ADC minimum tracked during the pressing phase.
    /// Initialised to `u16::MAX` so the first genuine press overwrites it.
    pub ac_full_cand:  u16 = u16::MAX,
    /// Current phase of the auto-calibration state machine.
    pub ac_phase:      AutoCalibPhase,
    /// Confidence counter; incremented per scored cycle, reset after an update.
    pub ac_confidence: u8,
    // ── calibration anchors: touched only on calibration updates ─────────
    /// Raw ADC at zero travel; stored for drift detection in
    /// [`KeyEntry::update_calib_if_drifted`].
    pub calib_zero:    u16 = REF_ZERO_TRAVEL,
    /// Persistent full-travel ADC reading loaded from / stored to EEPROM.
    /// Combined with a freshly measured zero reading on each boot to derive
    /// the hot-path fields via [`KeyEntry::apply_zero`].
    pub entry_full:    u16 = REF_ZERO_TRAVEL.saturating_sub(DEFAULT_FULL_RANGE),
}

/// Compile-time verification that the `repr(C)` field order produces the
/// documented hot-field layout.  Breaks the build rather than silently
/// degrading cache behaviour if a field is added or reordered.
const _: () = {
    use core::mem::offset_of;
    assert!(offset_of!(KeyEntry, last_raw) == 0);
    assert!(offset_of!(KeyEntry, lut_zero) == 2);
    assert!(offset_of!(KeyEntry, inv_scale) == 4);
    assert!(offset_of!(KeyEntry, travel) == 8);
    assert!(offset_of!(KeyEntry, extremum) == 9);
    assert!(offset_of!(KeyEntry, pressed) == 10);
    assert!(offset_of!(KeyEntry, calib_used) == 11);
};

impl KeyEntry {
    /// Recompute the hot-path calibration fields from `zero` and `full` ADC
    /// readings.
    ///
    /// Updates `inv_scale`, `lut_zero`, `calib_zero`, and `calib_used` in
    /// place. Uses ceiling division so the scaled travel at exactly the
    /// full-travel LUT delta reaches [`FULL_TRAVEL_UNIT`] after the
    /// right-shift rather than [`FULL_TRAVEL_UNIT`] − 1.
    fn apply_calib(&mut self, zero: u16, full: u16) {
        let zero_lut = Self::get_lut_val(zero);
        let full_lut = Self::get_lut_val(full);
        let delta = u32::from(full_lut.saturating_sub(zero_lut));
        // Ceiling division: rounds inv_scale up so the max travel value hits
        // FULL_TRAVEL_UNIT exactly after the INV_SCALE_FRAC_BITS right-shift.
        self.inv_scale = FULL_TRAVEL_SCALED.saturating_add(delta.saturating_sub(1)).checked_div(delta).unwrap_or(0);
        self.lut_zero = zero_lut;
        self.calib_zero = zero;
        self.calib_used = zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE;
    }

    /// Recompute calibration from a freshly measured `zero`-travel reading
    /// paired with the full-travel stored in [`KeyEntry::entry_full`].
    pub(crate) fn apply_zero(&mut self, zero: u16) {
        let full = self.entry_full;
        self.apply_calib(zero, full);
    }

    /// Look up the precomputed LUT value for ADC reading `raw`.
    ///
    /// Inputs outside [`VALID_RAW_MIN`]..=[`VALID_RAW_MAX`] cannot occur on
    /// the hot path because `travel_from` validates the range first. If an
    /// out-of-range value is passed anyway, the saturating subtract clamps to
    /// index 0 and `.get` clamps the upper end, returning a value at the LUT
    /// edge instead of panicking.
    ///
    /// # Safety
    ///
    /// `dsp` is globally enabled via `target-cpu=cortex-m4`; the compiler uses
    /// DSP instructions wherever profitable without a per-function annotation.
    /// `#[inline(always)]` eliminates call overhead at the two hot-path sites.
    #[inline(always)]
    #[optimize(speed)]
    pub(crate) fn get_lut_val(raw: u16) -> u16 {
        let idx = usize::from(raw.saturating_sub(VALID_RAW_MIN));
        TRAVEL_LUT.get(idx).copied().unwrap_or_else(|| {
            cold_path();
            // Only reached for raw > VALID_RAW_MAX; clamp to the edge entry.
            TRAVEL_LUT.last().copied().unwrap_or(0)
        })
    }

    /// Convert a raw ADC reading into a travel value.
    ///
    /// Returns `None` if the position is uncalibrated or `inv_scale == 0`.
    #[inline]
    #[optimize(speed)]
    pub(crate) fn travel_from(&self, raw: u16) -> Option<u8> {
        if unlikely(!self.calib_used || self.inv_scale == 0) {
            cold_path();
            return None;
        }
        let raw_lut_val = Self::get_lut_val(raw);
        let delta = raw_lut_val.saturating_sub(self.lut_zero);
        let scaled: u32 = u32::from(delta).saturating_mul(self.inv_scale);
        let travel: u32 = scaled.checked_shr(INV_SCALE_FRAC_BITS).unwrap_or(0).min(u32::from(FULL_TRAVEL_UNIT));
        Some(u8::try_from(travel).unwrap_or(FULL_TRAVEL_UNIT))
    }

    /// Update calibration if `new_zero` differs meaningfully from the current
    /// zero-travel value, avoiding unnecessary recalculation due to noise.
    ///
    /// Prevents small fluctuations from repeatedly rewriting the derived
    /// constants when the resting position shifts only within normal drift
    /// bounds.
    pub(crate) fn update_calib_if_drifted(&mut self, new_zero: u16, new_full: u16) {
        if self.calib_zero.abs_diff(new_zero) > AUTO_CALIB_ZERO_UPDATE_THRESHOLD {
            self.apply_calib(new_zero, new_full);
        }
    }
}
