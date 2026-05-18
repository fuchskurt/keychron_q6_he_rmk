use core::hint::{cold_path, unlikely};
use embassy_stm32::adc::{BasicAdcRegs, BasicInstance};
use embassy_time::{Duration, Instant};
pub use q6_core::lut::{VALID_RAW_MAX, VALID_RAW_MIN};
use q6_core::{lut, math::ceil_div_or_zero};

/// Number of confident press/release cycles required before the
/// auto-calibrator updates the live calibration data for a key.
///
/// Low values react quickly to genuine sensor drift but risk committing a
/// noisy or partial-press cycle; higher values are more conservative. Three
/// cycles gives a good balance between responsiveness and stability.
pub const AUTO_CALIB_CONFIDENCE_THRESHOLD: u8 = 3;

/// Bottom-of-travel ADC jitter tolerance subtracted from [`DEFAULT_FULL_RANGE`]
/// when computing [`AUTO_CALIB_RELEASE_THRESHOLD`].
///
/// A key in the pressing phase must rise at least
/// [`DEFAULT_FULL_RANGE`] - [`AUTO_CALIB_FULL_JITTER`] counts from its minimum
/// before release tracking begins. This margin allows for typical jitter at
/// the physical bottom without prematurely advancing to the releasing phase.
pub const AUTO_CALIB_FULL_JITTER: u16 = 100;

/// ADC value below which a reading is considered a genuine full-travel press
/// for auto-calibration purposes.
///
/// Set [`AUTO_CALIB_FULL_JITTER`] counts above the expected physical bottom so
/// bottom-of-travel ADC jitter does not require exactly hitting the reference
/// minimum to count as fully pressed.
pub const AUTO_CALIB_FULL_TRAVEL_THRESHOLD: u16 =
    REF_ZERO_TRAVEL.saturating_sub(DEFAULT_FULL_RANGE).saturating_add(AUTO_CALIB_FULL_JITTER);

/// Minimum ADC range (zero - full) for a complete press/release cycle to be
/// scored as confident during auto-calibration.
///
/// Cycles with a narrower range are likely partial presses or ADC noise and
/// are discarded without incrementing the confidence counter.
pub const AUTO_CALIB_MIN_RANGE: u16 = 900;

/// Minimum ADC rise from the full-travel minimum required to transition from
/// the pressing phase to the releasing phase during auto-calibration.
///
/// Set to [`DEFAULT_FULL_RANGE`] - [`AUTO_CALIB_FULL_JITTER`] so the key must
/// have risen most of the way back toward zero before release tracking begins,
/// preventing partial lifts from being counted.
pub const AUTO_CALIB_RELEASE_THRESHOLD: u16 = DEFAULT_FULL_RANGE.saturating_sub(AUTO_CALIB_FULL_JITTER);

/// Maximum ADC drop from the current zero-travel peak that is treated as
/// resting-position jitter during the releasing phase of auto-calibration.
///
/// Once the ADC has settled within this band below the tracked peak, the
/// press/release cycle is scored and the machine returns to idle.
pub const AUTO_CALIB_ZERO_JITTER: u16 = 50;

/// Minimum ADC difference between the current zero-travel calibration and a
/// newly observed zero candidate required before committing an update.
///
/// Prevents small fluctuations from repeatedly rewriting calibration constants
/// when the resting position shifts only within normal noise or drift bounds.
/// Acts as a hysteresis band on zero updates, reducing churn in the derived
/// constants and avoiding unnecessary recomputation.
pub const AUTO_CALIB_ZERO_UPDATE_THRESHOLD: u16 = 10;

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
pub const BOTTOM_JITTER: u16 = 80;

/// Duration a key must remain continuously pressed past
/// [`CALIB_PRESS_THRESHOLD`] before it is accepted as a valid full-travel
/// sample and its LED turns green.
pub const CALIB_HOLD_DURATION_MS: u64 = 1000;

/// Minimum ADC delta (zero - raw) required to consider a key genuinely pressed
/// during live full-travel calibration sampling.
///
/// A real full-travel press produces ~900 counts of delta, so 450 requires
/// roughly half travel, large enough to reject ADC noise (~100 counts) while
/// still triggering well before the physical bottom.
pub const CALIB_PRESS_THRESHOLD: u16 = 450;

/// Extra sampling time after all keys have been accepted during full-travel
/// calibration, allowing each key's minimum ADC to settle at its true
/// physical bottom rather than the first crossing of [`CALIB_PRESS_THRESHOLD`].
pub const CALIB_SETTLE_AFTER_ALL_DONE: Duration = Duration::from_millis(2000);

/// Maximum acceptable distance from [`REF_ZERO_TRAVEL`] for a key to be
/// considered to have a valid hall-effect sensor at its position.
pub const CALIB_ZERO_TOLERANCE: u16 = 500;

/// Default full-range calibration delta used when no better value is available.
pub const DEFAULT_FULL_RANGE: u16 = 900;

/// Precomputed numerator for the Q16.16 `inv_scale` field.
pub const FULL_TRAVEL_SCALED: u32 = u32::from(FULL_TRAVEL_UNIT).saturating_mul(INV_SCALE_ONE);

/// Expected travel distance in physical units, represents 4.0 mm.
pub const FULL_TRAVEL_UNIT: u8 = 40;

/// Number of bits in the Q16.16 representation of [`KeyEntry::inv_scale`].
pub const INV_SCALE_FRAC_BITS: u32 = 16;
/// The value of `1.0` in the Q16.16 format used by [`KeyEntry::inv_scale`].
pub const INV_SCALE_ONE: u32 = 1_u32.checked_shl(INV_SCALE_FRAC_BITS).expect("INV_SCALE_FRAC_BITS < u32::BITS");

/// Minimum ADC delta (zero - full) required to treat a full-travel reading as
/// usable. Guards only against completely flat or absent sensor readings;
/// intentionally small so any genuine press is accepted.
pub const MIN_USEFUL_FULL_RANGE: u16 = 100;

/// Reference zero-travel ADC value used for calibration and used-sensor
/// validation.
pub const REF_ZERO_TRAVEL: u16 = 3121;

/// ADC counts subtracted from the averaged zero-travel reading before storing
/// it as the calibration zero point.
///
/// Places the resting zero point slightly below the true ADC average so the
/// key sits cleanly at zero travel while fully released. Without this margin,
/// thermal or mechanical drift that nudges the resting ADC upward would
/// immediately produce a small non-zero travel reading, feeding spurious
/// input into the rapid-trigger extremum tracker.
pub const ZERO_TRAVEL_DEAD_ZONE: u16 = 20;

/// ADC sample-time type associated with the selected ADC peripheral.
pub type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

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
pub enum AutoCalibPhase {
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
    pub actuation_pt:           u8           = 10,
    /// Number of full-matrix passes averaged together during zero-travel
    /// calibration.
    pub calib_passes:           u32          = 512,
    /// Duration of the full-travel sampling window during first-boot
    /// calibration.
    pub full_calib_duration:    Duration     = Duration::from_secs(180),
    /// Raw ADC delta below which readings are treated as noise and discarded.
    pub noise_gate:             u16          = 10,
    /// Minimum upward travel from the trough required to register a new press,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_press:   u8           = 5,
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
pub enum KeyCalibState {
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
/// Fields are ordered hot-to-cold so the first 12 bytes (offsets 0-11) cover
/// every field touched on a noise-gated scan pass (`last_raw`, `lut_zero`,
/// `inv_scale`, `travel`, `extremum`, `pressed`, `calib_used`). The
/// Cortex-M4 in the STM32F401 has no data cache, but the contiguous
/// hot-field block still helps in two ways: the compiler can schedule
/// adjacent SRAM loads back-to-back via the AHB matrix without inserting
/// address-computation bubbles, and a future port to an MCU with a D-cache
/// (or a tight DTCM region) would automatically get cache-line locality.
///
/// `repr(C)` preserves declaration order, guaranteeing the documented byte
/// offsets. Without it the compiler is free to reorder fields for alignment,
/// which could scatter the hot fields across the struct and undo the
/// contiguity benefit above.
///
/// The matrix stores entries column-major as `[[KeyEntry; ROW]; COL]` so all
/// row entries for one HC164 column live in one SRAM block; the column scan
/// walks them with a fixed stride, which the load-store unit can pipeline
/// better than scattered indirect loads from a row-major layout.
///
/// Initialise via [`Default`]; call [`KeyEntry::apply_zero`] with the measured
/// resting ADC after loading EEPROM or completing first-boot calibration.
#[repr(C)]
#[derive(Default)]
pub struct KeyEntry {
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
    /// Candidate zero-travel ADC peak tracked during the releasing phase.
    pub ac_zero_cand:  u16,
    /// Candidate full-travel ADC minimum tracked during the pressing phase.
    /// Initialised to `u16::MAX` so the first genuine press overwrites it.
    pub ac_full_cand:  u16 = u16::MAX,
    /// Current phase of the auto-calibration state machine.
    pub ac_phase:      AutoCalibPhase,
    /// Confidence counter; incremented per scored cycle, reset after an update.
    pub ac_confidence: u8,
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
    assert!(offset_of!(KeyEntry, last_raw) == 0, "last_raw must be at offset 0");
    assert!(offset_of!(KeyEntry, lut_zero) == 2, "lut_zero must be at offset 2");
    assert!(offset_of!(KeyEntry, inv_scale) == 4, "inv_scale must be at offset 4");
    assert!(offset_of!(KeyEntry, travel) == 8, "travel must be at offset 8");
    assert!(offset_of!(KeyEntry, extremum) == 9, "extremum must be at offset 9");
    assert!(offset_of!(KeyEntry, pressed) == 10, "pressed must be at offset 10");
    assert!(offset_of!(KeyEntry, calib_used) == 11, "calib_used must be at offset 11");
};

impl KeyEntry {
    /// Recompute the hot-path calibration fields from `zero` and `full` ADC
    /// readings.
    ///
    /// Updates `inv_scale`, `lut_zero`, `calib_zero`, and `calib_used` in
    /// place. Uses ceiling division so the scaled travel at exactly the
    /// full-travel LUT delta reaches [`FULL_TRAVEL_UNIT`] after the
    /// right-shift rather than [`FULL_TRAVEL_UNIT`] - 1.
    const fn apply_calib(&mut self, zero: u16, full: u16) {
        let zero_lut = lut::lookup(zero);
        let full_lut = lut::lookup(full);
        let delta = u32::from(full_lut.saturating_sub(zero_lut));
        self.inv_scale = ceil_div_or_zero(FULL_TRAVEL_SCALED, delta);
        self.lut_zero = zero_lut;
        self.calib_zero = zero;
        self.calib_used = zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE;
    }

    /// Recompute calibration from a freshly measured `zero`-travel reading
    /// paired with the full-travel stored in [`KeyEntry::entry_full`].
    pub const fn apply_zero(&mut self, zero: u16) {
        let full = self.entry_full;
        self.apply_calib(zero, full);
    }

    /// Advance the auto-calibration state machine with a new ADC reading.
    ///
    /// Called on every scan reading that passes the noise gate, before the
    /// travel computation and rapid-trigger logic. Watches complete
    /// press/release cycles and refines the live calibration once
    /// [`AUTO_CALIB_CONFIDENCE_THRESHOLD`] confident cycles have accumulated,
    /// compensating for temperature and mechanical drift without requiring a
    /// manual re-calibration.
    ///
    /// A cycle is scored only when the ADC range (zero - full) meets
    /// [`AUTO_CALIB_MIN_RANGE`]; partial presses or noisy readings are
    /// discarded. Updated calibration takes effect immediately on the next
    /// travel computation within the same scan pass.
    #[inline]
    #[optimize(speed)]
    pub const fn auto_calib_step(&mut self, raw: u16) {
        match self.ac_phase {
            AutoCalibPhase::Idle => {
                if raw < AUTO_CALIB_FULL_TRAVEL_THRESHOLD {
                    self.ac_full_cand = raw;
                    self.ac_phase = AutoCalibPhase::Pressing;
                }
            },
            AutoCalibPhase::Pressing => {
                if raw < self.ac_full_cand {
                    // Key is pressing deeper; track the running minimum.
                    self.ac_full_cand = raw;
                } else if raw.saturating_sub(self.ac_full_cand) >= AUTO_CALIB_RELEASE_THRESHOLD {
                    // Reading has risen decisively above the tracked minimum;
                    // bottom-of-travel jitter or a partial early lift stays in
                    // Pressing rather than advancing.
                    self.ac_zero_cand = raw;
                    self.ac_phase = AutoCalibPhase::Releasing;
                } else {
                    // Within jitter band of the current minimum; stay in
                    // Pressing.
                }
            },
            AutoCalibPhase::Releasing => {
                if raw > self.ac_zero_cand {
                    // Key still rising; update the zero-travel peak.
                    self.ac_zero_cand = raw;
                } else {
                    // ADC has stopped rising: either settled within jitter (score
                    // the cycle) or dropped sharply (key re-pressed mid-release).
                    // Both cases return to Idle; only the settled case also scores.
                    if self.ac_zero_cand.saturating_sub(raw) <= AUTO_CALIB_ZERO_JITTER {
                        let range = self.ac_zero_cand.saturating_sub(self.ac_full_cand);
                        if range >= AUTO_CALIB_MIN_RANGE {
                            self.ac_confidence = self.ac_confidence.saturating_add(1);
                        }

                        if self.ac_confidence >= AUTO_CALIB_CONFIDENCE_THRESHOLD {
                            cold_path();
                            self.ac_confidence = 0;

                            let new_zero = self.ac_zero_cand.saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                            let new_full = full_from_min(new_zero, self.ac_full_cand);

                            // Only commit the update if the new zero differs
                            // meaningfully from the current one.
                            self.update_calib_if_drifted(new_zero, new_full);
                        }
                    }
                    self.ac_phase = AutoCalibPhase::Idle;
                }
            },
        }
    }

    /// Apply rapid-trigger logic for a new travel reading and report a press
    /// state transition if one occurred.
    ///
    /// Tracks the per-key extremum (peak while pressed, trough while released)
    /// and decides whether the key has just toggled state. Returns `None` if
    /// no transition occurred (extremum is still updated in place); returns
    /// `Some(now_pressed)` and resets the extremum to `new_travel` on a press
    /// or release transition so the next direction starts fresh from the
    /// transition point.
    ///
    /// `trough_floor` clamps the released-side extremum so a key driven below
    /// `act_threshold` re-fires cleanly at the actuation floor without
    /// requiring an extra `sensitivity_press` delta above it.
    #[inline]
    #[optimize(speed)]
    pub fn step_rapid_trigger(
        &mut self,
        new_travel: u8,
        act_threshold: u8,
        sensitivity_press: u8,
        sensitivity_release: u8,
        trough_floor: u8,
    ) -> Option<bool> {
        let below_act = new_travel < act_threshold;
        let now_pressed = if self.pressed {
            // Track the peak while held; release when travel drops at least
            // `sensitivity_release` below it. The actuation floor is a hard
            // lower bound that forces an immediate release.
            self.extremum = self.extremum.max(new_travel);
            if below_act { false } else { new_travel > self.extremum.saturating_sub(sensitivity_release) }
        } else {
            // Track the trough while released; re-press when travel climbs at
            // least `sensitivity_press` above it AND exceeds the actuation
            // floor. The trough is not required to have dipped below the floor
            // first, so a finger hovering mid-travel after an RT-release can
            // re-fire immediately.
            self.extremum = self.extremum.min(new_travel);
            if below_act {
                self.extremum = self.extremum.min(trough_floor);
                false
            } else {
                new_travel >= self.extremum.saturating_add(sensitivity_press)
            }
        };
        let changed = now_pressed != self.pressed;
        self.pressed = now_pressed;
        if changed {
            // Reset extremum so the next direction starts fresh from the
            // transition point.
            self.extremum = new_travel;
        }
        changed.then_some(now_pressed)
    }

    /// Convert a raw ADC reading into a travel value.
    ///
    /// Returns `None` if the position is uncalibrated or `inv_scale == 0`.
    /// Otherwise: lookup the per-reading LUT value, subtract the cached
    /// `lut_zero`, multiply by the Q16.16 `inv_scale`, and right-shift to
    /// drop the fractional bits. The final clamp to [`FULL_TRAVEL_UNIT`]
    /// keeps bottom-of-travel jitter pinned at the maximum (see
    /// [`BOTTOM_JITTER`]).
    #[inline]
    #[optimize(speed)]
    pub const fn travel_from(&self, raw: u16) -> Option<u8> {
        if unlikely(!self.calib_used || self.inv_scale == 0) {
            cold_path();
            return None;
        }
        let delta = lut::lookup(raw).saturating_sub(self.lut_zero);
        let scaled = u32::from(delta).saturating_mul(self.inv_scale);
        let travel = scaled.wrapping_shr(INV_SCALE_FRAC_BITS).min(u32::from(FULL_TRAVEL_UNIT));
        Some(u8::try_from(travel).unwrap_or(FULL_TRAVEL_UNIT))
    }

    /// Update calibration if `new_zero` differs meaningfully from the current
    /// zero-travel value, avoiding unnecessary recalculation due to noise.
    ///
    /// Prevents small fluctuations from repeatedly rewriting the derived
    /// constants when the resting position shifts only within normal drift
    /// bounds.
    pub const fn update_calib_if_drifted(&mut self, new_zero: u16, new_full: u16) {
        if self.calib_zero.abs_diff(new_zero) > AUTO_CALIB_ZERO_UPDATE_THRESHOLD {
            #[cfg(feature = "defmt")]
            defmt::debug!("auto-calib update: zero {} -> {}, full {}", self.calib_zero, new_zero, new_full);
            self.apply_calib(new_zero, new_full);
        }
    }
}

/// Compute a calibrated full-travel ADC value from a measured minimum and
/// zero-travel reading.
///
/// Adds [`BOTTOM_JITTER`] to the measured minimum so jitter at the physical
/// bottom does not produce spurious RT releases, caps the result to at least
/// [`MIN_USEFUL_FULL_RANGE`] below `zero` so a non-trivial travel range
/// survives, then enforces a [`VALID_RAW_MIN`] floor. Shared by both the
/// first-boot calibration (with a [`DEFAULT_FULL_RANGE`] fallback for
/// never-pressed keys) and the runtime auto-calibrator.
///
/// Uses `.min(hi).max(lo)` rather than `.clamp(lo, hi)` because the upper
/// bound can fall below the lower bound for pathological sensor values
/// (`zero < VALID_RAW_MIN + MIN_USEFUL_FULL_RANGE`); `clamp` panics in that
/// case, while this ordering safely degrades to [`VALID_RAW_MIN`].
#[inline]
pub const fn full_from_min(zero: u16, observed_min: u16) -> u16 {
    observed_min.saturating_add(BOTTOM_JITTER).min(zero.saturating_sub(MIN_USEFUL_FULL_RANGE)).max(VALID_RAW_MIN)
}
