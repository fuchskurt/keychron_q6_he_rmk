use embassy_stm32::adc::{BasicAdcRegs, BasicInstance};
use embassy_time::{Duration, Instant};
use num_traits::float::Float as _;

/// Number of confident press/release cycles required before the
/// auto-calibrator updates the live [`KeyCalib`] data for a key.
///
/// Low values react quickly to genuine sensor drift but risk committing a
/// noisy or partial-press cycle; higher values are more conservative. Three
/// cycles gives a good balance between responsiveness and stability.
pub(crate) const AUTO_CALIB_CONFIDENCE_THRESHOLD: u8 = 3;

/// Maximum ADC rise from the current full-travel minimum that is still treated
/// as bottom-of-travel jitter during the pressing phase of auto-calibration.
///
/// Readings that rise by more than this amount from the minimum but less than
/// [`AUTO_CALIB_RELEASE_THRESHOLD`] are treated as noise and do not advance
/// the phase.
pub(crate) const AUTO_CALIB_FULL_JITTER: u16 = 100;

/// ADC value below which a reading is considered a genuine full-travel press
/// for auto-calibration purposes.
pub(crate) const AUTO_CALIB_FULL_TRAVEL_THRESHOLD: u16 =
    UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE).saturating_add(100);

/// Minimum ADC range (zero − full) for a complete press/release cycle to be
/// scored as confident during auto-calibration.
///
/// Cycles with a narrower range are likely partial presses or ADC noise and
/// are discarded without incrementing the confidence counter.
pub(crate) const AUTO_CALIB_MIN_RANGE: u16 = 900;

/// Minimum ADC rise from the full-travel minimum required to transition from
/// the pressing phase to the releasing phase during auto-calibration.
///
/// Set to `DEFAULT_FULL_RANGE − AUTO_CALIB_FULL_JITTER` so the key must have
/// risen most of the way back toward zero before release tracking begins,
/// preventing partial lifts from being counted.
pub(crate) const AUTO_CALIB_RELEASE_THRESHOLD: u16 = DEFAULT_FULL_RANGE.saturating_sub(AUTO_CALIB_FULL_JITTER);

/// Maximum ADC drop from the current zero-travel peak that is treated as
/// resting-position jitter during the releasing phase of auto-calibration.
///
/// Once the ADC has settled within this band below the tracked peak, the
/// press/release cycle is scored and the machine returns to idle.
pub(crate) const AUTO_CALIB_ZERO_JITTER: u16 = 50;

/// ADC counts added to the measured full-travel minimum before storing it as
/// the calibration floor.
///
/// The physical bottom-out reading is the true minimum ADC value a key can
/// produce, so without this offset `travel_from` can return exactly
/// [`FULL_TRAVEL_UNIT`] while the key is held down. Any upward jitter then
/// advances `extremum`, and a subsequent reading that dips back to the true
/// floor satisfies the rapid-trigger release condition, firing a spurious
/// release even though the key never moved.
///
/// By raising the stored floor 80 counts above the physical minimum the scale
/// factor is anchored to a point the key cannot reach in normal use, so
/// bottom-of-travel travel values stay comfortably below [`FULL_TRAVEL_UNIT`]
/// and RT jitter cannot accumulate a phantom peak.
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

/// Minimum ADC delta (zero − full) required to accept a full-travel sample
/// when validating stored EEPROM entries. Guards only against completely flat
/// or absent readings, intentionally small.
pub(crate) const MIN_USEFUL_FULL_RANGE: u16 = 100;

/// Cubic coefficient of the Hall sensor transfer polynomial `f(x) = A3·x³ +
/// A2·x² + A1·x + A0`.
///
/// Maps a raw ADC value to a distance-from-zero in mm, derived by least-squares
/// fit to empirical sensor data. Evaluated via Horner's method in
/// [`KeyCalib::poly`].
pub(crate) const POLY_A0: f32 = 426.88962;
/// Linear coefficient of the Hall sensor transfer polynomial.
pub(crate) const POLY_A1: f32 = -0.48358;
/// Quadratic coefficient of the Hall sensor transfer polynomial.
pub(crate) const POLY_A2: f32 = 2.04637e-4;
/// Cubic coefficient of the Hall sensor transfer polynomial.
pub(crate) const POLY_A3: f32 = -2.99368e-8;

/// Reference zero-travel ADC value used for calibration and used-sensor
/// validation.
pub const REF_ZERO_TRAVEL: u16 = 3121;

/// Fallback zero-travel ADC value used before any calibration has run.
pub(crate) const UNCALIBRATED_ZERO: u16 = 3000;

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
/// release; partial presses or re-presses mid-release reset to `Idle`.
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
    pub confidence: u8,
    /// Deepest (lowest) ADC reading seen during the current press, i.e. the
    /// candidate full-travel value.
    pub full_candidate: u16,
    /// Current phase of the press/release state machine.
    pub phase: AutoCalibPhase,
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

/// Configuration parameters for hall-effect key sensing.
#[derive(Clone, Copy)]
pub struct HallCfg {
    /// Minimum travel threshold in mm/10 units before a key is considered
    /// actuated.
    pub actuation_pt: u8,
    /// Number of full-matrix passes averaged together during zero-travel
    /// calibration.
    pub calib_passes: u32,
    /// Duration of the full-travel sampling window during first-boot
    /// calibration.
    pub full_calib_duration: Duration,
    /// Raw ADC delta below which back-to-back readings from the same key are
    /// treated as noise and discarded.
    pub noise_gate: u16,
    /// Minimum upward travel from the trough required to register a new press,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_press: u8,
    /// Minimum downward travel from the peak required to register a release,
    /// in mm/10 units (e.g. 1 = 0.1 mm).
    pub rt_sensitivity_release: u8,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self {
        Self {
            actuation_pt: 8,
            calib_passes: 512,
            full_calib_duration: Duration::from_secs(360),
            noise_gate: 10,
            rt_sensitivity_press: 3,
            rt_sensitivity_release: 3,
        }
    }
}

/// Calibration values for a single key.
#[derive(Clone, Copy)]
pub(crate) struct KeyCalib {
    /// Reciprocal scale factor precomputed on construction:
    pub inv_scale: f32,
    /// Polynomial value at zero travel (key fully released)
    pub poly_zero: f32,
    /// Whether this matrix position has a valid hall-effect sensor.
    ///
    /// `false` for unpopulated positions or those whose zero-travel reading
    /// is too far from [`REF_ZERO_TRAVEL`] to be a real key.
    pub used: bool,
    /// Raw ADC value corresponding to zero travel (key fully released).
    pub zero: u16,
}

impl KeyCalib {
    /// Build calibration data from zero and full-travel ADC readings.
    ///
    /// Precomputes [`Self::poly_zero`] and [`Self::inv_scale`] so the hot
    /// scan path only needs a polynomial evaluation and a multiply.
    pub(crate) fn new(zero: u16, full: u16) -> Self {
        let poly_zero = Self::poly(f32::from(zero));
        let delta = Self::poly(f32::from(full)).algebraic_sub(poly_zero);
        Self {
            inv_scale: (delta > 0.0).then(|| f32::from(FULL_TRAVEL_UNIT).algebraic_div(delta)).unwrap_or(0.0),
            poly_zero,
            used: zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE,
            zero,
        }
    }

    /// Evaluate the Hall sensor transfer polynomial at `x` using Horner's
    /// method.
    ///
    /// Implements `f(x) = A3·x³ + A2·x² + A1·x + A0` in Horner form to
    /// minimize floating-point rounding and use the VFMA instruction on
    /// Cortex-M4.
    pub(crate) fn poly(x: f32) -> f32 { POLY_A3.mul_add(x, POLY_A2).mul_add(x, POLY_A1).mul_add(x, POLY_A0) }

    /// Build an uncalibrated placeholder.
    ///
    /// `used` is `false` so the scanner ignores this position entirely until
    /// real calibration data is applied.
    pub(crate) const fn uncalibrated() -> Self {
        Self { inv_scale: 0.0, poly_zero: 0.0, used: false, zero: UNCALIBRATED_ZERO }
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
    /// changed meaningfully. Initialised to `u16::MAX` so the first real
    /// reading always passes the gate.
    pub last_raw: u16,
    /// Whether the key is currently considered pressed.
    pub pressed: bool,
    /// Travel value from the previous scan cycle.
    ///
    /// Used to skip redundant RT computations when travel has not changed
    /// after the noise gate passes.
    pub travel: u8,
}

impl KeyState {
    /// Create a new default key state.
    ///
    /// `last_raw` starts at [`u16::MAX`] so that the first real ADC reading
    /// always produces a diff larger than any noise gate, ensuring every key
    /// is evaluated on the very first scan pass.
    pub(crate) const fn new() -> Self { Self { extremum: u8::MAX, last_raw: u16::MAX, travel: 0, pressed: false } }
}

impl Default for KeyState {
    fn default() -> Self { Self::new() }
}

/// Hold-detection state for a single key during the full-travel calibration
/// pass.
///
/// The state machine transitions:
/// `Waiting → Holding(t)` when the key crosses [`CALIB_PRESS_THRESHOLD`];
/// `Holding(t) → Waiting` if the key is released before
/// [`CALIB_HOLD_DURATION_MS`]; `Holding(t) → Accepted` when the hold duration
/// is satisfied.
#[derive(Clone, Copy)]
pub(crate) enum KeyCalibState {
    /// Hold duration satisfied; this key is fully calibrated.
    Accepted,
    /// Key crossed the threshold at the stored instant; hold timer is running.
    Holding(Instant),
    /// Key has not yet crossed the press threshold, or was released and reset.
    Waiting,
}
