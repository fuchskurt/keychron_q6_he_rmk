use super::lut;
pub use super::lut::{VALID_RAW_MAX, VALID_RAW_MIN};
use core::hint::{cold_path, unlikely};
use embassy_stm32::adc::{BasicAdcRegs, BasicInstance};
use embassy_time::{Duration, Instant};

/// Confidence score required before the auto-calibrator updates the live
/// calibration data for a key.
///
/// Cycles are scored with graded weights (see the `AUTO_CALIB_WEIGHT_*`
/// constants, mirroring the stock firmware's scheme): one unambiguous
/// deep cycle reaches the threshold on its own, while marginal cycles need
/// four confirmations. This converges after a single clean press when the
/// evidence is strong without giving noisy or shallow cycles more trust.
pub const AUTO_CALIB_CONFIDENCE_THRESHOLD: u8 = 12;

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
/// scored at all during auto-calibration.
///
/// Cycles with a narrower range are likely partial presses or ADC noise and
/// are discarded without adding confidence; see [`cycle_weight`] for the
/// graded scoring above this floor.
pub const AUTO_CALIB_MIN_RANGE: u16 = 900;

/// ADC range (zero - full) from which a press/release cycle is scored with
/// [`AUTO_CALIB_WEIGHT_GOOD`]: deeper than a marginal cycle but short of an
/// unambiguous full-depth press.
pub const AUTO_CALIB_RANGE_GOOD: u16 = 1000;

/// ADC range (zero - full) from which a press/release cycle is scored with
/// [`AUTO_CALIB_WEIGHT_STRONG`]: an unambiguous full-depth press whose
/// evidence is trusted enough to commit a calibration update on its own.
pub const AUTO_CALIB_RANGE_STRONG: u16 = 1100;

/// Minimum ADC rise from the full-travel minimum required to transition from
/// the pressing phase to the releasing phase during auto-calibration.
///
/// Set to [`DEFAULT_FULL_RANGE`] - [`AUTO_CALIB_FULL_JITTER`] so the key must
/// have risen most of the way back toward zero before release tracking begins,
/// preventing partial lifts from being counted.
pub const AUTO_CALIB_RELEASE_THRESHOLD: u16 = DEFAULT_FULL_RANGE.saturating_sub(AUTO_CALIB_FULL_JITTER);

/// Maximum time between leaving the physical bottom and settling at zero
/// travel for a press/release cycle to be scored, in [`coarse_ms_now`]
/// units (~1.024 ms each, so roughly one second).
///
/// A genuine release flight takes tens of milliseconds. Cycles slower than
/// this window are long holds (the resting position may have drifted while
/// the key sat at the bottom) or slow drifts masquerading as cycles, and
/// must not feed the calibration. The bound matters most with graded
/// confidence weights, where a single deep cycle can commit an update.
pub const AUTO_CALIB_VALID_RELEASE_WINDOW: u32 = 1000;

/// Confidence points for a cycle with range >= [`AUTO_CALIB_RANGE_GOOD`].
pub const AUTO_CALIB_WEIGHT_GOOD: u8 = 4;

/// Confidence points for a cycle with range >= [`AUTO_CALIB_MIN_RANGE`].
pub const AUTO_CALIB_WEIGHT_MIN: u8 = 3;

/// Confidence points for a cycle with range >= [`AUTO_CALIB_RANGE_STRONG`].
/// Equal to [`AUTO_CALIB_CONFIDENCE_THRESHOLD`], so one such cycle commits.
pub const AUTO_CALIB_WEIGHT_STRONG: u8 = 12;

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
/// the calibration floor; see [`full_from_min`].
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

/// Expected travel distance in fine travel units
/// ([`FULL_TRAVEL_UNIT`] × [`TRAVEL_SCALE`]); represents 4.0 mm.
pub const FULL_TRAVEL_FINE: u8 = FULL_TRAVEL_UNIT.saturating_mul(TRAVEL_SCALE);

/// Precomputed numerator for the Q16.16 `inv_scale` field
/// (`FULL_TRAVEL_FINE * INV_SCALE_ONE`).
pub const FULL_TRAVEL_SCALED: u32 = u32::from(FULL_TRAVEL_FINE).saturating_mul(INV_SCALE_ONE);

/// Expected travel distance in configuration units (mm/20); represents
/// 4.0 mm.
pub const FULL_TRAVEL_UNIT: u8 = 80;

/// Number of fractional bits in the Q16.16 `inv_scale`.
pub const INV_SCALE_FRAC_BITS: u32 = 16;

/// The value of `1.0` in the Q16.16 format used by `inv_scale`.
pub const INV_SCALE_ONE: u32 = 1_u32.checked_shl(INV_SCALE_FRAC_BITS).expect("INV_SCALE_FRAC_BITS < u32::BITS");

/// Minimum ADC delta (zero - full) required to treat a full-travel reading as
/// usable. Guards only against completely flat or absent sensor readings;
/// intentionally small so any genuine press is accepted.
pub const MIN_USEFUL_FULL_RANGE: u16 = 100;

/// Reference zero-travel ADC value used for calibration and used-sensor
/// validation.
pub const REF_ZERO_TRAVEL: u16 = 3121;

/// Fine travel quanta per 0.05 mm configuration unit.
///
/// Travel is tracked internally at 1/60 mm (the same fine unit the stock
/// firmware uses), three quanta per 0.05 mm configuration step, so
/// rapid-trigger comparisons are exact instead of carrying up to half a
/// configuration step of quantisation slop. [`HallCfg`] stays in 0.05 mm
/// units; [`RtTuning::from_cfg`] converts to fine units once at startup.
///
/// 0.05 mm is the finest honest configuration step for this sensor: the
/// ±10-count noise gate already spans ~0.045 mm of travel, so finer steps
/// would be indistinguishable in practice.
pub const TRAVEL_SCALE: u8 = 3;

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
    /// Minimum travel threshold in mm/20 units (1 = 0.05 mm) before a key is
    /// considered actuated.
    pub actuation_pt:           u8           = 20,
    /// Number of full-matrix passes averaged together during zero-travel
    /// calibration.
    pub calib_passes:           u32          = 512,
    /// Duration of the full-travel sampling window during first-boot
    /// calibration.
    pub full_calib_duration:    Duration     = Duration::from_secs(180),
    /// Raw ADC delta below which readings are treated as noise and discarded.
    pub noise_gate:             u16          = 10,
    /// Minimum upward travel from the trough required to register a new press,
    /// in mm/20 units (e.g. 1 = 0.05 mm).
    pub rt_sensitivity_press:   u8           = 10,
    /// Minimum downward travel from the peak required to register a release,
    /// in mm/20 units (e.g. 1 = 0.05 mm).
    pub rt_sensitivity_release: u8           = 6,
}

/// Rapid-trigger tuning values derived once from [`HallCfg`] before the scan
/// loop starts, so the hot path reads pre-clamped constants instead of
/// re-deriving them on every pass. All travel values are in fine travel
/// units (1/60 mm each, [`TRAVEL_SCALE`] quanta per configuration step).
#[derive(Clone, Copy)]
pub struct RtTuning {
    /// Minimum travel threshold before a key is considered actuated, in
    /// fine travel units.
    pub act_threshold:       u8,
    /// Raw ADC delta below which readings are treated as noise.
    pub noise_gate:          u16,
    /// Minimum upward travel from the trough required to register a press,
    /// in fine travel units.
    pub sensitivity_press:   u8,
    /// Minimum downward travel from the peak required to register a
    /// release, in fine travel units.
    pub sensitivity_release: u8,
    /// Lower clamp applied to the released-side extremum so a key driven
    /// below the actuation point re-fires cleanly at the actuation floor.
    pub trough_floor:        u8,
}

impl RtTuning {
    /// Derive the tuning values from `cfg`, converting the 0.05 mm
    /// configuration units into fine travel units.
    ///
    /// Both sensitivities are clamped to 1 so a zero config value never
    /// disables the dead-band.
    #[must_use]
    pub const fn from_cfg(cfg: HallCfg) -> Self {
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        let sensitivity_press = cfg.rt_sensitivity_press.max(1).saturating_mul(TRAVEL_SCALE);
        Self {
            act_threshold,
            noise_gate: cfg.noise_gate,
            sensitivity_press,
            sensitivity_release: cfg.rt_sensitivity_release.max(1).saturating_mul(TRAVEL_SCALE),
            trough_floor: act_threshold.saturating_sub(sensitivity_press),
        }
    }
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
/// Fields are listed alphabetically. The STM32F401's Cortex-M4 has no data
/// cache and no tightly-coupled memory, so field order has no cache-line
/// effect and the exact layout is left to the compiler.
///
/// The matrix stores entries column-major as `[[KeyEntry; ROW]; COL]` so all
/// row entries for one HC164 column are contiguous in SRAM; the column scan
/// walks them with a fixed stride.
///
/// Initialise via [`Default`]; call [`KeyEntry::apply_zero`] with the measured
/// resting ADC after loading EEPROM or completing first-boot calibration.
#[derive(Default)]
pub struct KeyEntry {
    /// Confidence score; raised per scored cycle by the graded
    /// `AUTO_CALIB_WEIGHT_*` values, reset after an update.
    pub ac_confidence: u8,
    /// Timestamp ([`coarse_ms_now`] units) of the last reading at the
    /// physical bottom, used by the release-time bound.
    pub ac_full_at:    u32,
    /// Candidate full-travel ADC minimum tracked during the pressing phase.
    /// Initialised to `u16::MAX` so the first genuine press overwrites it.
    pub ac_full_cand:  u16 = u16::MAX,
    /// Current phase of the auto-calibration state machine.
    pub ac_phase:      AutoCalibPhase,
    /// Candidate zero-travel ADC peak tracked during the releasing phase.
    pub ac_zero_cand:  u16,
    /// Whether this matrix position has a valid hall-effect sensor.
    pub calib_used:    bool,
    /// Raw ADC at zero travel; stored for drift detection in
    /// [`KeyEntry::update_calib_if_drifted`].
    pub calib_zero:    u16 = REF_ZERO_TRAVEL,
    /// Persistent full-travel ADC reading loaded from / stored to EEPROM.
    /// Combined with a freshly measured zero reading on each boot to derive
    /// the hot-path fields via [`KeyEntry::apply_zero`].
    pub entry_full:    u16 = REF_ZERO_TRAVEL.saturating_sub(DEFAULT_FULL_RANGE),
    /// Local extremum for rapid-trigger in fine travel units (peak while
    /// pressed, trough while released). Reset to `new_travel` on every
    /// press↔release transition.
    pub extremum:      u8 = u8::MAX,
    /// Q16.16 reciprocal of the calibrated travel range.
    pub inv_scale:     u32,
    /// Raw ADC from the previous scan cycle (noise gate filter).
    /// `u16::MAX` on first boot so the first real reading always passes.
    pub last_raw:      u16 = u16::MAX,
    /// LUT value at zero travel, precomputed for fast travel arithmetic.
    pub lut_zero:      u16,
    /// Whether the key is currently considered pressed.
    pub pressed:       bool,
    /// Quantised travel value from the previous scan cycle, in fine travel
    /// units (1/60 mm each).
    pub travel:        u8,
}

impl KeyEntry {
    /// Recompute the hot-path calibration fields from `zero` and `full` ADC
    /// readings.
    ///
    /// Updates `inv_scale`, `lut_zero`, `calib_zero`, and `calib_used` in
    /// place. Uses ceiling division on the inverse scale so the scaled travel
    /// at exactly the full-travel LUT delta reaches [`FULL_TRAVEL_FINE`] after
    /// the right-shift rather than [`FULL_TRAVEL_FINE`] - 1. `calib_used` is
    /// set when the resting reading is within [`CALIB_ZERO_TOLERANCE`] of
    /// [`REF_ZERO_TRAVEL`]; positions outside that band are treated as missing
    /// sensors and excluded from scanning.
    const fn apply_calib(&mut self, zero: u16, full: u16) {
        let zero_lut = lut::lookup(zero);
        let full_lut = lut::lookup(full);
        let delta = u32::from(full_lut.saturating_sub(zero_lut));
        // Ceiling division of `FULL_TRAVEL_SCALED` by `delta` (0 when
        // `delta == 0`, i.e. a degenerate calibration that the travel hot
        // path then treats as uncalibrated). Rounding up makes the scaled
        // travel at the full-travel LUT delta reach exactly the full-travel
        // unit after the right-shift rather than one count short.
        self.inv_scale = FULL_TRAVEL_SCALED.saturating_add(delta.saturating_sub(1)).checked_div(delta).unwrap_or(0);
        self.lut_zero = zero_lut;
        self.calib_zero = zero;
        self.calib_used = zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE;
    }

    /// Recompute calibration from a freshly measured `zero`-travel reading
    /// paired with the full-travel stored in [`KeyEntry::entry_full`].
    ///
    /// A reading more than [`CALIB_ZERO_TOLERANCE`] *below*
    /// [`REF_ZERO_TRAVEL`] means the key was almost certainly held down while
    /// the boot zero pass sampled it, so instead of letting
    /// [`KeyEntry::apply_calib`] disable the position, fall back to
    /// [`REF_ZERO_TRAVEL`]: the key registers as pressed until released and
    /// the auto-calibrator replaces the approximate zero after the first few
    /// genuine press/release cycles. Readings far *above* the reference still
    /// disable the position; that side indicates a missing or faulty sensor
    /// rather than a held key.
    pub const fn apply_zero(&mut self, zero: u16) {
        let resting = if zero.saturating_add(CALIB_ZERO_TOLERANCE) < REF_ZERO_TRAVEL { REF_ZERO_TRAVEL } else { zero };
        let full = self.entry_full;
        self.apply_calib(resting, full);
    }

    /// Advance the auto-calibration state machine with a new ADC reading.
    ///
    /// Called on every scan reading that passes the noise gate, before the
    /// travel computation and rapid-trigger logic; `now` is the current
    /// [`coarse_ms_now`] timestamp. Watches complete press/release cycles
    /// and refines the live calibration once the graded confidence score
    /// reaches [`AUTO_CALIB_CONFIDENCE_THRESHOLD`], compensating for
    /// temperature and mechanical drift without requiring a manual
    /// re-calibration.
    ///
    /// A cycle is scored only when the ADC range (zero - full) meets
    /// [`AUTO_CALIB_MIN_RANGE`] *and* the release settled within
    /// [`AUTO_CALIB_VALID_RELEASE_WINDOW`] of last touching the physical
    /// bottom; partial presses, noisy readings, and drawn-out releases are
    /// discarded. Deeper cycles score more (`AUTO_CALIB_WEIGHT_*`), so one
    /// unambiguous full-depth press is enough to commit. Updated calibration
    /// takes effect immediately on the next travel computation within the
    /// same scan pass.
    #[inline]
    #[optimize(speed)]
    pub const fn auto_calib_step(&mut self, raw: u16, now: u32) {
        match self.ac_phase {
            AutoCalibPhase::Idle => {
                if raw < AUTO_CALIB_FULL_TRAVEL_THRESHOLD {
                    self.ac_full_cand = raw;
                    self.ac_full_at = now;
                    self.ac_phase = AutoCalibPhase::Pressing;
                }
            },
            AutoCalibPhase::Pressing => {
                if raw < self.ac_full_cand {
                    // Key is pressing deeper; track the running minimum.
                    self.ac_full_cand = raw;
                    self.ac_full_at = now;
                } else if raw.saturating_sub(self.ac_full_cand) >= AUTO_CALIB_RELEASE_THRESHOLD {
                    // Reading has risen decisively above the tracked minimum;
                    // bottom-of-travel jitter or a partial early lift stays in
                    // Pressing rather than advancing.
                    self.ac_zero_cand = raw;
                    self.ac_phase = AutoCalibPhase::Releasing;
                } else if raw.saturating_sub(self.ac_full_cand) <= AUTO_CALIB_FULL_JITTER {
                    // Still at the physical bottom (within jitter of the
                    // tracked minimum). Refresh the bottom timestamp so the
                    // release-time bound measures the release flight, not
                    // how long the key was held down.
                    self.ac_full_at = now;
                } else {
                    // Partial lift between the jitter band and the release
                    // threshold; stay in Pressing.
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
                        // Score the cycle only if the release was quick
                        // (wrapping_sub handles timestamp wraparound), with
                        // deeper cycles earning more confidence.
                        if now.wrapping_sub(self.ac_full_at) < AUTO_CALIB_VALID_RELEASE_WINDOW {
                            self.ac_confidence = self.ac_confidence.saturating_add(cycle_weight(range));
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
    /// [`RtTuning::trough_floor`] clamps the released-side extremum so a key
    /// driven below the actuation threshold re-fires cleanly at the actuation
    /// floor without requiring an extra `sensitivity_press` delta above it.
    #[inline]
    #[optimize(speed)]
    pub fn step_rapid_trigger(&mut self, new_travel: u8, tuning: RtTuning) -> Option<bool> {
        let below_act = new_travel < tuning.act_threshold;
        let now_pressed = if self.pressed {
            // Track the peak while held; release when travel drops at least
            // `sensitivity_release` below it. The actuation floor is a hard
            // lower bound that forces an immediate release.
            self.extremum = self.extremum.max(new_travel);
            if below_act { false } else { new_travel > self.extremum.saturating_sub(tuning.sensitivity_release) }
        } else {
            // Track the trough while released; re-press when travel climbs at
            // least `sensitivity_press` above it AND exceeds the actuation
            // floor. The trough is not required to have dipped below the floor
            // first, so a finger hovering mid-travel after an RT-release can
            // re-fire immediately.
            self.extremum = self.extremum.min(new_travel);
            if below_act {
                self.extremum = self.extremum.min(tuning.trough_floor);
                false
            } else {
                new_travel >= self.extremum.saturating_add(tuning.sensitivity_press)
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
    /// Otherwise: look up the per-reading LUT value, subtract `lut_zero`,
    /// multiply by the Q16.16 `inv_scale`, and right-shift to drop the
    /// fractional bits. The result is in fine travel units; the final clamp
    /// to [`FULL_TRAVEL_FINE`] keeps bottom-of-travel jitter pinned at the
    /// maximum.
    #[inline]
    #[optimize(speed)]
    pub const fn travel_from(&self, raw: u16) -> Option<u8> {
        if unlikely(!self.calib_used || self.inv_scale == 0) {
            cold_path();
            return None;
        }
        let delta = lut::lookup(raw).saturating_sub(self.lut_zero);
        let scaled = u32::from(delta).saturating_mul(self.inv_scale);
        let travel = scaled.wrapping_shr(INV_SCALE_FRAC_BITS).min(u32::from(FULL_TRAVEL_FINE));
        Some(u8::try_from(travel).unwrap_or(FULL_TRAVEL_FINE))
    }

    /// Update calibration if `new_zero` differs meaningfully from the current
    /// zero-travel value, avoiding unnecessary recalculation due to noise.
    ///
    /// Prevents small fluctuations from repeatedly rewriting the derived
    /// constants when the resting position shifts only within normal drift
    /// bounds.
    pub const fn update_calib_if_drifted(&mut self, new_zero: u16, new_full: u16) {
        if self.calib_zero.abs_diff(new_zero) > AUTO_CALIB_ZERO_UPDATE_THRESHOLD {
            self.apply_calib(new_zero, new_full);
        }
    }
}

/// Coarse millisecond-scale timestamp for the auto-calibrator's
/// release-time bound.
///
/// Derived from the 1 MHz embassy tick counter by a right-shift (one unit
/// = 1.024 ms) so the scan hot path never pays for a 64-bit division. The
/// low 32 bits wrap roughly every 50 days; elapsed times are computed with
/// wrapping subtraction, so a wrap mid-cycle merely mis-scores a single
/// auto-calibration cycle.
#[must_use]
#[inline]
pub fn coarse_ms_now() -> u32 {
    u32::try_from(Instant::now().as_ticks().wrapping_shr(10) & u64::from(u32::MAX)).unwrap_or(0)
}

/// Grade a press/release cycle's confidence weight from its observed ADC
/// range (zero - full).
///
/// Deeper cycles are stronger evidence of the key's true endpoints:
/// [`AUTO_CALIB_RANGE_STRONG`] earns enough to commit an update on its own,
/// [`AUTO_CALIB_RANGE_GOOD`] needs three confirmations,
/// [`AUTO_CALIB_MIN_RANGE`] four, and anything narrower scores nothing.
#[must_use]
#[inline]
pub const fn cycle_weight(range: u16) -> u8 {
    if range >= AUTO_CALIB_RANGE_STRONG {
        AUTO_CALIB_WEIGHT_STRONG
    } else if range >= AUTO_CALIB_RANGE_GOOD {
        AUTO_CALIB_WEIGHT_GOOD
    } else if range >= AUTO_CALIB_MIN_RANGE {
        AUTO_CALIB_WEIGHT_MIN
    } else {
        0
    }
}

/// Compute a calibrated full-travel ADC value from a measured minimum and
/// zero-travel reading.
///
/// Adds [`BOTTOM_JITTER`] to the measured minimum so jitter at the physical
/// bottom does not produce spurious rapid-trigger releases, caps the result
/// at least [`MIN_USEFUL_FULL_RANGE`] below `zero` so a non-trivial travel
/// range survives, then enforces a [`VALID_RAW_MIN`] floor. Shared by both
/// the first-boot calibration (with a default-range fallback for never-pressed
/// keys) and the runtime auto-calibrator.
///
/// Uses `.min(hi).max(lo)` rather than `.clamp(lo, hi)` because the upper
/// bound can fall below the lower bound for pathological sensor values
/// (`zero < VALID_RAW_MIN + MIN_USEFUL_FULL_RANGE`); `clamp` panics in that
/// case, while this ordering safely degrades to [`VALID_RAW_MIN`].
#[must_use]
#[inline]
pub const fn full_from_min(zero: u16, observed_min: u16) -> u16 {
    observed_min.saturating_add(BOTTOM_JITTER).min(zero.saturating_sub(MIN_USEFUL_FULL_RANGE)).max(VALID_RAW_MIN)
}
