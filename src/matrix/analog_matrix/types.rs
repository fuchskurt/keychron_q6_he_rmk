use crate::matrix::analog_matrix::lut::TRAVEL_LUT;
use core::hint::{cold_path, unlikely};
use embassy_stm32::adc::{BasicAdcRegs, BasicInstance};
use embassy_time::{Duration, Instant};

/// Number of confident press/release cycles required before the
/// auto-calibrator updates the live [`KeyCalib`] data for a key.
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
/// Prevents small fluctuations from repeatedly rewriting [`KeyCalib`] when the
/// resting position shifts only within normal noise or drift bounds. Acts as a
/// hysteresis band on zero updates, reducing churn in the derived [`KeyCalib`]
/// data and avoiding unnecessary recomputation.
///
/// Set well above typical ADC noise but small relative to real zero-travel
/// drift so genuine changes are still captured.
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

/// Expected travel distance in physical units, represents 4.0 mm.
pub(crate) const FULL_TRAVEL_UNIT: u8 = 40;

/// Precomputed numerator for [`KeyCalib::inv_scale`]
pub(crate) const FULL_TRAVEL_SCALED: u32 = u32::from(FULL_TRAVEL_UNIT).saturating_mul(INV_SCALE_ONE);

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
/// Number of bits in the Q16.16 representation of [`KeyCalib::inv_scale`].
pub(crate) const INV_SCALE_FRAC_BITS: u32 = 16;
/// The value of `1.0` in the Q16.16 format used by [`KeyCalib::inv_scale`].
pub(crate) const INV_SCALE_ONE: u32 = 1_u32.checked_shl(INV_SCALE_FRAC_BITS).expect("INV_SCALE_FRAC_BITS < u32::BITS");
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
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum AutoCalibPhase {
    /// Waiting for the key to be pressed past
    /// [`AUTO_CALIB_FULL_TRAVEL_THRESHOLD`].
    Idle,
    /// Key is pressed; tracking the minimum ADC reading (deepest travel).
    Pressing,
    /// Key is releasing; tracking the maximum ADC reading (zero travel).
    Releasing,
}

/// Per-key state for the continuous auto-calibration that runs during normal
/// operation, refining zero and full-travel ADC values on every press/release
/// cycle to compensate for temperature and mechanical drift.
///
/// Once [`AUTO_CALIB_CONFIDENCE_THRESHOLD`] scored cycles accumulate the live
/// [`KeyCalib`] for that key is updated in place, no reboot required and no
/// user interaction needed.
#[derive(Clone, Copy)]
pub(crate) struct AutoCalib {
    /// Accumulated confidence score. Incremented by one per scored cycle;
    /// reset to zero after a calibration update is applied.
    pub confidence:     u8,
    /// Deepest (lowest) ADC reading seen during the current press, i.e. the
    /// candidate full-travel value.
    pub full_candidate: u16,
    /// Current phase of the press/release state machine.
    pub phase:          AutoCalibPhase,
    /// Highest ADC reading seen while the key was releasing after the most
    /// recent press, i.e. the candidate zero-travel value.
    pub zero_candidate: u16,
}

impl AutoCalib {
    /// Create a new idle auto-calibration state.
    pub(crate) const fn new() -> Self {
        Self { confidence: 0, full_candidate: u16::MAX, phase: AutoCalibPhase::Idle, zero_candidate: 0 }
    }
}

impl Default for AutoCalib {
    fn default() -> Self { Self::new() }
}

/// Per-key full-travel calibration value stored in EEPROM.
#[derive(Clone, Copy)]
pub(crate) struct CalibEntry {
    /// Raw ADC reading at full travel (key fully depressed to the PCB).
    pub full: u16,
}

/// Configuration parameters for hall-effect key sensing.
#[derive(Clone, Copy)]
pub struct HallCfg {
    /// Minimum travel threshold in mm/10 units before a key is considered
    /// actuated.
    pub actuation_pt:           u8,
    /// Number of full-matrix passes averaged together during zero-travel
    /// calibration.
    pub calib_passes:           u32,
    /// Time to wait after selecting a new column before reading ADC values,
    /// allowing the analog bus to settle.
    pub col_settle_us:          Duration,
    /// Duration of the full-travel sampling window during first-boot
    /// calibration.
    pub full_calib_duration:    Duration,
    /// Raw ADC delta below which readings are treated as noise and discarded.
    pub noise_gate:             u16,
    /// Minimum upward travel from the trough required to register a new press,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_press:   u8,
    /// Minimum downward travel from the peak required to register a release,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_release: u8,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self {
        Self {
            actuation_pt:           8,
            calib_passes:           512,
            col_settle_us:          Duration::from_micros(1),
            full_calib_duration:    Duration::from_secs(360),
            noise_gate:             10,
            rt_sensitivity_press:   3,
            rt_sensitivity_release: 3,
        }
    }
}

/// Calibration values for a single key.
#[derive(Clone, Copy)]
pub(crate) struct KeyCalib {
    /// Precomputed reciprocal of the calibrated travel range.
    pub inv_scale: u32,
    /// LUT value corresponding to zero travel (key fully released).
    pub lut_zero:  u16,
    /// Whether this matrix position has a valid hall-effect sensor.
    pub used:      bool,
    /// Raw ADC value corresponding to zero travel (key fully released).
    pub zero:      u16,
}

impl KeyCalib {
    /// Build calibration data from zero and full-travel ADC readings.
    ///
    /// Precomputes [`KeyCalib::lut_zero`]
    /// and [`KeyCalib::inv_scale`] so the
    /// hot scan path needs only a LUT lookup and a multiply-shift.
    pub(crate) fn new(zero: u16, full: u16) -> Self {
        let zero_travel = Self::get_lut_val(zero);
        let full_travel = Self::get_lut_val(full);
        let delta_full = u32::from(full_travel.saturating_sub(zero_travel));
        // Ceiling division: travel_from right-shifts by [`INV_SCALE_FRAC_BITS`]
        // (truncating), so rounding inv_scale up guarantees the result at
        // exactly the full-travel LUT delta reaches FULL_TRAVEL_UNIT after
        // the shift. A floor inv_scale would sometimes give FULL_TRAVEL_UNIT - 1.
        let inv_scale =
            FULL_TRAVEL_SCALED.saturating_add(delta_full.saturating_sub(1)).checked_div(delta_full).unwrap_or(0);
        Self { inv_scale, lut_zero: zero_travel, used: zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE, zero }
    }

    /// Look up the precomputed LUT value for ADC reading `raw`.
    ///
    /// Inputs outside [`VALID_RAW_MIN`]..=[`VALID_RAW_MAX`] cannot occur on
    /// the hot path because `travel_from` validates the range first. If an
    /// out-of-range value is passed anyway, the saturating subtract clamps to
    /// index 0 and `.get` clamps the upper end, returning a value at the LUT
    /// edge instead of panicking.
    #[inline]
    pub(crate) fn get_lut_val(raw: u16) -> u16 {
        let idx = usize::from(raw.saturating_sub(VALID_RAW_MIN));
        TRAVEL_LUT.get(idx).copied().unwrap_or_else(|| {
            cold_path();
            // Only reached for raw > VALID_RAW_MAX; clamp to the edge entry.
            TRAVEL_LUT.last().copied().unwrap_or(0)
        })
    }

    /// Build an uncalibrated placeholder.
    ///
    /// [`KeyCalib::used`] is `false` so the scanner ignores this position
    /// entirely until real calibration data is applied.
    pub(crate) const fn uncalibrated() -> Self {
        Self { inv_scale: 0, lut_zero: 0, used: false, zero: REF_ZERO_TRAVEL }
    }
}

impl Default for KeyCalib {
    fn default() -> Self { Self::uncalibrated() }
}

/// Dynamic state tracked per key across scan iterations.
#[derive(Clone, Copy)]
pub(crate) struct KeyState {
    /// Local extremum for rapid-trigger calculations.
    ///
    /// Tracks the peak (highest travel) while pressed and the trough (lowest
    /// travel) while released. Reset to `new_travel` on every state transition
    /// so each direction starts tracking from the transition point.
    pub extremum: u8,
    /// Raw ADC reading from the previous scan cycle.
    ///
    /// Used by the noise gate to skip processing when the reading has not
    /// changed meaningfully. Initialised to [`u16::MAX`] so the first real
    /// reading always passes the gate.
    pub last_raw: u16,
    /// Whether the key is currently considered pressed.
    pub pressed:  bool,
    /// Travel value from the previous scan cycle.
    ///
    /// Used to skip redundant RT computations when travel has not changed
    /// after the noise gate passes.
    pub travel:   u8,
}

impl KeyState {
    /// Create a new default key state.
    ///
    /// [`KeyState::last_raw`] starts at [`u16::MAX`] so that the first real ADC
    /// reading always produces a diff larger than any noise gate, ensuring
    /// every key is evaluated on the very first scan pass.
    pub(crate) const fn new() -> Self { Self { extremum: u8::MAX, last_raw: u16::MAX, travel: 0, pressed: false } }
}

impl Default for KeyState {
    fn default() -> Self { Self::new() }
}

/// All per-key data accessed on every scan iteration, grouped into a single
/// allocation for cache locality.
///
/// Initialise with [`KeyCalib::uncalibrated`] until EEPROM or factory
/// calibration data is available; [`KeyState`] and [`AutoCalib`] default to
/// their idle states automatically. [`CalibEntry`] is populated from EEPROM
/// on boot or written during first-boot calibration, and is the source of
/// truth for [`KeyEntry::calib`] across reboots.
pub(crate) struct KeyEntry {
    /// Continuous auto-calibration state machine, observing press/release
    /// cycles and refining [`KeyEntry::calib`] to compensate for drift.
    pub auto_calib: AutoCalib,
    /// Calibration constants derived from zero and full-travel ADC readings.
    /// Updated in place by the auto-calibrator without requiring a reboot.
    pub calib:      KeyCalib,
    /// Persistent full-travel ADC reading loaded from or written to EEPROM.
    /// Combined with a freshly measured zero-travel reading on each boot to
    /// derive [`KeyEntry::calib`] via [`KeyCalib::new`].
    pub entry:      CalibEntry,
    /// Runtime press/release tracking state for rapid-trigger logic.
    pub state:      KeyState,
}

impl KeyEntry {
    /// Recompute [`KeyEntry::calib`] from a freshly measured `zero`-travel
    /// reading paired with the full-travel value stored in [`KeyEntry::entry`].
    pub(crate) fn apply_zero(&mut self, zero: u16) { self.calib = KeyCalib::new(zero, self.entry.full); }

    /// Update [`KeyEntry::calib`] if `new_zero` differs meaningfully from the
    /// current zero-travel calibration, avoiding unnecessary churn in the
    /// LUT-based constants derived by [`KeyCalib::new`].
    pub(crate) fn update_calib_if_drifted(&mut self, new_zero: u16, new_full: u16) {
        if self.calib.zero.abs_diff(new_zero) > AUTO_CALIB_ZERO_UPDATE_THRESHOLD {
            self.calib = KeyCalib::new(new_zero, new_full);
        }
    }

    /// Convert a raw ADC reading into a travel value.
    ///
    /// Returns `None` if the position is uncalibrated or `cal.inv_scale == 0`.
    #[inline]
    pub(crate) fn travel_from(&self, raw: u16) -> Option<u8> {
        if unlikely(!self.calib.used || self.calib.inv_scale == 0) {
            cold_path();
            return None;
        }
        let raw_lut_val = KeyCalib::get_lut_val(raw);
        let delta = raw_lut_val.saturating_sub(self.calib.lut_zero);
        let scaled: u32 = u32::from(delta).saturating_mul(self.calib.inv_scale);
        let travel: u32 = scaled.checked_shr(INV_SCALE_FRAC_BITS).unwrap_or(0).min(u32::from(FULL_TRAVEL_UNIT));
        Some(u8::try_from(travel).unwrap_or(FULL_TRAVEL_UNIT))
    }
}

impl Default for KeyEntry {
    fn default() -> Self {
        Self {
            state:      KeyState::new(),
            calib:      KeyCalib::uncalibrated(),
            auto_calib: AutoCalib::new(),
            entry:      CalibEntry { full: REF_ZERO_TRAVEL.saturating_sub(DEFAULT_FULL_RANGE) },
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
pub(crate) enum KeyCalibState {
    /// Hold duration satisfied; this key is fully calibrated.
    Accepted,
    /// Key crossed the threshold at the stored [`Instant`]; hold timer is
    /// running.
    Holding(Instant),
    /// Key has not yet crossed the press threshold, or was released and reset.
    Waiting,
}
