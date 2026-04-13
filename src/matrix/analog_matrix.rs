use crate::{
    backlight::{
        lock_indicator::{BACKLIGHT_CH, BacklightCmd, CalibPhase},
        mapping::MATRIX_TO_LED,
    },
    eeprom::Ft24c64,
    matrix::{calib_store, calib_store::CALIB_BUF_LEN, hc164_cols::Hc164Cols, sensor_mapping::SENSOR_POSITIONS},
};
use calib_store::EEPROM_BASE_ADDR;
use core::{
    future::pending,
    hint::{likely, unlikely},
};
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, BasicAdcRegs, BasicInstance, ConfiguredSequence, Instance, RxDma},
    dma::InterruptHandler,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    pac::adc,
};
use embassy_time::{Duration, Instant, Timer};
use num_traits::float::Float;
use rmk::{
    event::{KeyboardEvent, publish_event_async},
    input_device::{InputDevice, Runnable},
};

/// Copy the element at `(row, col)` from a 2-D array slice, returning `None`
/// if either index is out of bounds.
#[inline]
fn get2<T: Copy, const C: usize>(arr: &[[T; C]], row: usize, col: usize) -> Option<T> {
    arr.get(row)?.get(col).copied()
}

/// Return a mutable reference to `(row, col)` in a 2-D array slice, returning
/// `None` if either index is out of bounds.
#[inline]
fn get2_mut<T, const C: usize>(arr: &mut [[T; C]], row: usize, col: usize) -> Option<&mut T> {
    arr.get_mut(row)?.get_mut(col)
}

/// Number of confident press/release cycles required before the
/// auto-calibrator updates the live [`KeyCalib`] data for a key.
///
/// Low values react quickly to genuine sensor drift but risk committing a
/// noisy or partial-press cycle; higher values are more conservative. Three
/// cycles gives a good balance between responsiveness and stability.
const AUTO_CALIB_CONFIDENCE_THRESHOLD: u8 = 3;

/// Maximum ADC rise from the current full-travel minimum that is still treated
/// as bottom-of-travel jitter during the pressing phase of auto-calibration.
///
/// Readings that rise by more than this amount from the minimum but less than
/// [`AUTO_CALIB_RELEASE_THRESHOLD`] are treated as noise and do not advance
/// the phase.
const AUTO_CALIB_FULL_JITTER: u16 = 100;

/// ADC value below which a reading is considered a genuine full-travel press
/// for auto-calibration purposes.
const AUTO_CALIB_FULL_TRAVEL_THRESHOLD: u16 = UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE).saturating_add(100);

/// Minimum ADC range (zero − full) for a complete press/release cycle to be
/// scored as confident during auto-calibration.
///
/// Cycles with a narrower range are likely partial presses or ADC noise and
/// are discarded without incrementing the confidence counter.
const AUTO_CALIB_MIN_RANGE: u16 = 900;

/// Minimum ADC rise from the full-travel minimum required to transition from
/// the pressing phase to the releasing phase during auto-calibration.
///
/// Set to `DEFAULT_FULL_RANGE − AUTO_CALIB_FULL_JITTER` so the key must have
/// risen most of the way back toward zero before release tracking begins,
/// preventing partial lifts from being counted.
const AUTO_CALIB_RELEASE_THRESHOLD: u16 = DEFAULT_FULL_RANGE.saturating_sub(AUTO_CALIB_FULL_JITTER);

/// Maximum ADC drop from the current zero-travel peak that is treated as
/// resting-position jitter during the releasing phase of auto-calibration.
///
/// Once the ADC has settled within this band below the tracked peak, the
/// press/release cycle is scored and the machine returns to idle.
const AUTO_CALIB_ZERO_JITTER: u16 = 50;

/// ADC counts added to the measured full-travel minimum before storing it as
/// the calibration floor.
///
/// The physical bottom-out reading is the true minimum ADC value a key can
/// produce, so without this offset `travel_scaled_from` can return exactly
/// [`FULL_TRAVEL_SCALED`] while the key is held down. Any upward jitter then
/// advances `extremum`, and a subsequent reading that dips back to the true
/// floor satisfies the rapid-trigger release condition, firing a spurious
/// release even though the key never moved.
///
/// By raising the stored floor 80 counts above the physical minimum the scale
/// factor is anchored to a point the key cannot reach in normal use, so
/// bottom-of-travel travel values stay comfortably below [`FULL_TRAVEL_SCALED`]
/// and RT jitter cannot accumulate a phantom peak.
const BOTTOM_JITTER: u16 = 80;

/// Duration a key must remain continuously pressed past
/// [`CALIB_PRESS_THRESHOLD`] before it is accepted as a valid full-travel
/// sample and its LED turns green.
const CALIB_HOLD_DURATION_MS: u64 = 1000;

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
const CALIB_SETTLE_AFTER_ALL_DONE_MS: u64 = 1000;

/// Maximum acceptable distance from [`REF_ZERO_TRAVEL`] for a key to be
/// considered to have a valid hall-effect sensor at its position.
const CALIB_ZERO_TOLERANCE: u16 = 500;

/// Number of additional ADC reads used to confirm a column reading is stable
/// before processing any key state changes within it.
///
/// After the initial column read, the column is re-sampled this many times
/// immediately with no settle delay. If any re-sample differs from the first
/// read by more than [`HallCfg::noise_gate`] on any row, the entire column
/// pass is discarded and no key state is mutated.
const DEBOUNCE_PASSES: u8 = 2;

/// Default full-range calibration delta used when no better value is available.
const DEFAULT_FULL_RANGE: u16 = 900;

/// Time the EEPROM requires after power-on before acknowledging I²C
/// transactions.
const EEPROM_POWER_ON_DELAY: Duration = Duration::from_micros(300);

/// Full travel value expressed in internal scaled travel units.
const FULL_TRAVEL_SCALED: u8 = FULL_TRAVEL_UNIT.saturating_mul(TRAVEL_SCALE);

/// Compile-time guard: the product must fit in a `u8` without saturating so
/// that [`FULL_TRAVEL_SCALED`] is exact and travel clamping in
/// [`AnalogHallMatrix::travel_scaled_from`] is correct.
const _: () = assert!(
    u32::from(FULL_TRAVEL_UNIT).saturating_mul(u32::from(TRAVEL_SCALE)) < 256,
    "FULL_TRAVEL_UNIT * TRAVEL_SCALE overflows u8; reduce one of the constants"
);

/// Expected travel distance in physical units, represents 4.0 mm.
const FULL_TRAVEL_UNIT: u8 = 40;

/// Minimum ADC delta (zero − full) required to accept a full-travel sample
/// when validating stored EEPROM entries. Guards only against completely flat
/// or absent readings, intentionally small.
const MIN_USEFUL_FULL_RANGE: u16 = 100;

/// Reference zero-travel ADC value used as the LUT origin.
pub const REF_ZERO_TRAVEL: u16 = 3121;

/// Scale factor converting physical travel units into internal scaled travel
/// units.
const TRAVEL_SCALE: u8 = 6;

/// Fallback zero-travel ADC value used before any calibration has run.
const UNCALIBRATED_ZERO: u16 = 3000;

/// Maximum acceptable raw ADC value.
const VALID_RAW_MAX: u16 = 3500;

/// Minimum acceptable raw ADC value.
const VALID_RAW_MIN: u16 = 1200;

/// ADC counts subtracted from the averaged zero-travel reading before storing
/// it as the calibration zero point.
///
/// Places the resting zero point slightly below the true ADC average so the
/// key sits cleanly at zero travel while fully released. Without this margin,
/// thermal or mechanical drift that nudges the resting ADC upward would
/// immediately produce a small non-zero travel reading, feeding spurious
/// input into the rapid-trigger extremum tracker.
const ZERO_TRAVEL_DEAD_ZONE: u16 = 20;

/// ADC sample-time type associated with the selected ADC peripheral.
type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

/// Phase the auto-calibrator is currently in for a single key.
///
/// The state machine follows `Idle → Pressing → Releasing → Idle`. A cycle
/// is scored only when the key completes a genuine full-press and a clean
/// release; partial presses or re-presses mid-release reset to `Idle`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum AutoCalibPhase {
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
struct AutoCalib {
    /// Accumulated confidence score. Incremented by one per scored cycle;
    /// reset to zero after a calibration update is applied.
    confidence: u8,
    /// Deepest (lowest) ADC reading seen during the current press, i.e. the
    /// candidate full-travel value.
    full_candidate: u16,
    /// Current phase of the press/release state machine.
    phase: AutoCalibPhase,
    /// Highest ADC reading seen while the key was releasing after the most
    /// recent press, i.e. the candidate zero-travel value.
    zero_candidate: u16,
}

impl AutoCalib {
    /// Create a new idle auto-calibration state.
    const fn new() -> Self {
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
    /// Time to wait after selecting a new column before reading ADC values,
    /// allowing the analog bus to settle.
    pub col_settle_us: Duration,
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
    /// CPU cycles to hold each HC164 pin transition stable.
    pub shifter_delay_cycles: u32,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self {
        Self {
            actuation_pt: 8,
            calib_passes: 512,
            col_settle_us: Duration::from_micros(35),
            full_calib_duration: Duration::from_secs(360),
            noise_gate: 5,
            rt_sensitivity_press: 3,
            rt_sensitivity_release: 3,
            shifter_delay_cycles: 8,
        }
    }
}

/// Calibration values for a single key.
#[derive(Clone, Copy)]
struct KeyCalib {
    /// Whether this matrix position has a valid hall-effect sensor.
    ///
    /// `false` for unpopulated positions or those whose zero-travel reading
    /// is too far from [`REF_ZERO_TRAVEL`] to be a real key.
    used: bool,
    /// Polynomial Value at Pfull pressed point, precomputed on construction.
    poly_delta_full: f32,
    /// Polynomial Value at Zero point, precomputed on construction.
    poly_zero: f32,
    /// Raw ADC value corresponding to zero travel (key fully released).
    zero: u16,
}

impl KeyCalib {
    fn poly(x: f32) -> f32 {
        (-2.99368e-8_f32).mul_add(x, 2.04637e-4_f32).mul_add(x, -0.48358_f32).mul_add(x, 426.88962_f32)
    }

    /// Build calibration data from zero and full-travel ADC readings.
    fn new(zero: u16, full: u16) -> Self {
        let poly_zero = Self::poly(f32::from(zero));
        let poly_full = Self::poly(f32::from(full));
        Self {
            poly_delta_full: poly_full - poly_zero,
            poly_zero,
            used: zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE,
            zero,
        }
    }

    /// Build an uncalibrated placeholder.
    ///
    /// `used` is `false` so the scanner ignores this position entirely until
    /// real calibration data is applied.
    pub const fn uncalibrated() -> Self {
        Self { used: false, poly_delta_full: 1.0, poly_zero: 0.0, zero: UNCALIBRATED_ZERO }
    }
}

impl Default for KeyCalib {
    fn default() -> Self { Self::uncalibrated() }
}

/// Dynamic state tracked per key across scan iterations.
#[derive(Clone, Copy)]
struct KeyState {
    /// Local extremum for rapid-trigger calculations.
    ///
    /// Tracks the peak (highest travel) while pressed and the trough (lowest
    /// travel) while released. Reset to `new_travel` on every state transition
    /// so each direction starts tracking from the transition point.
    extremum: u8,
    /// Raw ADC reading from the previous scan cycle.
    ///
    /// Used by the noise gate to skip processing when the reading has not
    /// changed meaningfully. Initialised to `u16::MAX` so the first real
    /// reading always passes the gate.
    last_raw: u16,
    /// Whether the key is currently considered pressed.
    pressed: bool,
    /// Travel value from the previous scan cycle, in internal scaled units.
    ///
    /// Used to skip redundant RT computations when travel has not changed
    /// after the noise gate passes.
    travel_scaled: u8,
}

impl KeyState {
    /// Create a new default key state.
    ///
    /// `last_raw` starts at [`u16::MAX`] so that the first real ADC reading
    /// always produces a diff larger than any noise gate, ensuring every key
    /// is evaluated on the very first scan pass.
    pub const fn new() -> Self { Self { extremum: u8::MAX, last_raw: u16::MAX, travel_scaled: 0, pressed: false } }
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
enum KeyCalibState {
    /// Key has not yet crossed the press threshold, or was released and reset.
    Waiting,
    /// Key crossed the threshold at the stored instant; hold timer is running.
    Holding(Instant),
    /// Hold duration satisfied; this key is fully calibrated.
    Accepted,
}

/// ADC-related peripherals grouped to allow split borrows when constructing
/// a [`ConfiguredSequence`].
pub struct AdcPart<'peripherals, ADC, D, const ROW: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripheral used for sampling hall sensors.
    pub adc: Adc<'peripherals, ADC>,
    /// DMA channel used for non-blocking ADC sequence reads.
    pub dma: Peri<'peripherals, D>,
    /// ADC channels corresponding to each matrix row.
    pub row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time applied to every channel in the sequence.
    pub sample_time: AdcSampleTime<ADC>,
}

impl<'peripherals, ADC, D, const ROW: usize> AdcPart<'peripherals, ADC, D, ROW>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a [`ConfiguredSequence`] over all row channels.
    ///
    /// Programs the ADC sequence registers once; the returned reader can be
    /// triggered repeatedly without reprogramming, saving ~20 cycles per
    /// column per scan.
    fn configured_sequence<'reader, IRQ2>(&'reader mut self, irq: IRQ2) -> ConfiguredSequence<'reader, adc::Adc>
    where
        IRQ2: Binding<D::Interrupt, InterruptHandler<D>> + 'reader + 'peripherals,
    {
        let st = self.sample_time;
        self.adc.configured_sequence(self.dma.reborrow(), self.row_adc.iter_mut().map(|ch| (ch, st)), irq)
    }

    /// Create a new [`AdcPart`] from the given peripherals.
    pub fn new(
        adc: Adc<'peripherals, ADC>,
        row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
        dma: Peri<'peripherals, D>,
        sample_time: AdcSampleTime<ADC>,
    ) -> Self {
        Self { adc, dma, row_adc, sample_time }
    }
}

/// Hall-effect analog matrix scanner with EEPROM-backed per-key calibration
/// and continuous auto-calibration.
///
/// On first boot (or after EEPROM corruption) the firmware performs a guided
/// two-phase calibration:
///
/// 1. **Zero-travel pass** - all keys fully released; the firmware averages
///    `HallCfg::calib_passes` reads per key. Backlight signals amber.
/// 2. **Full-travel pass** - user presses every key to the bottom within
///    `HallCfg::full_calib_duration`; each key must be held for
///    [`CALIB_HOLD_DURATION_MS`] before it is accepted and its LED turns green.
///    Only positions in [`SENSOR_POSITIONS`] are counted.
///
/// After all keys are accepted a [`CALIB_SETTLE_AFTER_ALL_DONE_MS`] window
/// continues sampling to capture the true bottom-out ADC. Validated entries
/// are written to the FT24C64 EEPROM and verified by read-back. On all
/// subsequent boots only full-travel data is loaded from EEPROM; zero-travel
/// is re-measured fresh to compensate for temperature drift.
///
/// During normal operation the auto-calibrator silently refines both zero and
/// full-travel values on every press/release cycle, keeping the scanner
/// accurate as the sensor drifts over time without requiring user interaction.
pub struct AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripherals and channels grouped for split-borrow compatibility.
    adc_part: AdcPart<'peripherals, ADC, D, ROW>,
    /// Per-key auto-calibration state used to refine
    /// [`self::AnalogHallMatrix::calib`] during normal operation.
    auto_calib: [[AutoCalib; COL]; ROW],
    /// Per-key calibration data applied during the scan loop.
    calib: [[KeyCalib; COL]; ROW],
    /// Sensing and scanning configuration.
    cfg: HallCfg,
    /// Column driver used to select the active column via the HC164.
    cols: Hc164Cols<'peripherals>,
    /// EEPROM driver for loading and persisting calibration data.
    eeprom: Ft24c64<'peripherals, IM>,
    /// DMA interrupt binding reused for every ADC sequence read.
    irq: IRQ,
    /// Dynamic per-key runtime state for the scan loop.
    state: [[KeyState; COL]; ROW],
}

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a new matrix scanner.
    ///
    /// Calibration is deferred to [`Runnable::run`], which loads from EEPROM
    /// on subsequent boots or runs a full first-boot calibration pass.
    pub fn new(
        adc_part: AdcPart<'peripherals, ADC, D, ROW>,
        irq: IRQ,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
        eeprom: Ft24c64<'peripherals, IM>,
    ) -> Self {
        Self {
            adc_part,
            auto_calib: [[AutoCalib::new(); COL]; ROW],
            calib: [[KeyCalib::uncalibrated(); COL]; ROW],
            cfg,
            cols,
            eeprom,
            irq,
            state: [[KeyState::new(); COL]; ROW],
        }
    }

    /// Build the default calibration entries used when EEPROM is blank or
    /// invalid, giving a reasonable approximation of travel until real
    /// calibration runs.
    const fn default_entries() -> [[calib_store::CalibEntry; COL]; ROW] {
        [[calib_store::CalibEntry { full: UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE) }; COL]; ROW]
    }

    /// Apply a row-major array of [`calib_store::CalibEntry`] values to
    /// `self.calib`, pairing each stored full-travel reading with the
    /// corresponding live zero-travel reading from `zero_raw`.
    fn apply_calib(&mut self, entries: &[[calib_store::CalibEntry; COL]; ROW], zero_raw: &[[u16; COL]; ROW]) {
        for row in 0..ROW {
            for col in 0..COL {
                let Some(cal) = get2_mut(&mut self.calib, row, col) else { continue };
                let Some(zero) = get2(zero_raw, row, col) else { continue };
                let Some(entry) = get2(entries, row, col) else { continue };
                *cal = KeyCalib::new(zero, entry.full);
            }
        }
    }

    /// Average `cfg.calib_passes` full-matrix scans to establish per-key
    /// zero-travel (resting) ADC values.
    ///
    /// All keys must be fully released during this pass. Returns a
    /// `ROW × COL` array of raw ADC averages, each reduced by
    /// [`ZERO_TRAVEL_DEAD_ZONE`] so that the resting position sits cleanly
    /// below the measured average, preventing ADC noise from producing
    /// spurious non-zero travel readings. Does not modify `self.calib`.
    async fn calibrate_zero_raw(&mut self) -> [[u16; COL]; ROW] {
        let mut acc = [[0_u32; COL]; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);
        let mut buf = [0_u16; ROW];

        for _ in 0..self.cfg.calib_passes {
            for col in 0..COL {
                self.cols.select(col);
                Timer::after(self.cfg.col_settle_us).await;
                seq.read(&mut buf).await;
                for (row, &raw) in buf.iter().enumerate() {
                    if let Some(cell) = get2_mut(&mut acc, row, col) {
                        *cell = cell.saturating_add(u32::from(raw));
                    }
                }
            }
        }

        let mut result = [[UNCALIBRATED_ZERO; COL]; ROW];
        for (res_row, acc_row) in result.iter_mut().zip(acc.iter()) {
            for (res, &total) in res_row.iter_mut().zip(acc_row.iter()) {
                // Leave the UNCALIBRATED_ZERO initializer in place on any
                // arithmetic failure (e.g. calib_passes == 0).
                if let Some(avg) = total.checked_div(self.cfg.calib_passes) {
                    *res = u16::try_from(avg).unwrap_or(UNCALIBRATED_ZERO).saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                }
            }
        }
        result
    }

    /// Sample the matrix for `duration` (or until all real keys are accepted),
    /// recording the minimum ADC reading seen per key.
    ///
    /// Lower ADC = more magnet travel, so the minimum reading over the window
    /// is the deepest press seen. A key is accepted only after it has stayed
    /// continuously below [`CALIB_PRESS_THRESHOLD`] for
    /// [`CALIB_HOLD_DURATION_MS`]; releasing and re-pressing resets the timer.
    /// The LED turns green only at acceptance, not at first crossing.
    ///
    /// After all keys are accepted a [`CALIB_SETTLE_AFTER_ALL_DONE_MS`]
    /// continuation window keeps updating `min_raw` so the stored value
    /// reflects the true bottom-out ADC, not merely the acceptance instant.
    async fn sample_full_raw(&mut self, duration: Duration, zero_raw: &[[u16; COL]; ROW]) -> [[u16; COL]; ROW] {
        let deadline = Instant::now() + duration;
        let mut min_raw = [[u16::MAX; COL]; ROW];

        // Closure to update the running minimum for a single position.
        let mut update_min = |row: usize, col: usize, raw: u16| {
            if let Some(cell) = get2_mut(&mut min_raw, row, col) {
                if raw < *cell {
                    *cell = raw;
                }
            }
        };

        let mut calib_state: [[KeyCalibState; COL]; ROW] = [[KeyCalibState::Waiting; COL]; ROW];
        let mut calibrated_count: usize = 0;
        let hold_duration = Duration::from_millis(CALIB_HOLD_DURATION_MS);

        // Count only positions that have a physical sensor and a plausible
        // zero-travel reading so the denominator matches the detection loop.
        let total_keys = (0..ROW)
            .flat_map(|r| (0..COL).map(move |c| (r, c)))
            .filter(|&(r, c)| {
                SENSOR_POSITIONS.get(r).and_then(|row| row.get(c)).copied().unwrap_or(false)
                    && get2(zero_raw, r, c).map_or(false, |z| z.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE)
            })
            .count()
            .max(1);

        let mut seq = self.adc_part.configured_sequence(self.irq);
        let mut buf = [0_u16; ROW];
        let mut last_pct: u8 = 0;

        // Phase A: sample until all real keys are accepted or the deadline expires.
        while Instant::now() < deadline && calibrated_count < total_keys {
            for col in 0..COL {
                self.cols.select(col);
                Timer::after(self.cfg.col_settle_us).await;
                seq.read(&mut buf).await;

                for (row, &raw) in buf.iter().enumerate() {
                    // Always track the deepest reading seen, regardless of
                    // whether the key has been accepted yet.
                    update_min(row, col, raw);

                    if !SENSOR_POSITIONS.get(row).and_then(|r| r.get(col)).copied().unwrap_or(false) {
                        continue;
                    }

                    let Some(key_state) = get2_mut(&mut calib_state, row, col) else {
                        continue;
                    };

                    // Skip keys already accepted.
                    if matches!(*key_state, KeyCalibState::Accepted) {
                        continue;
                    }

                    let zero = get2(zero_raw, row, col).unwrap_or(UNCALIBRATED_ZERO);
                    let pressed = zero.saturating_sub(raw) >= CALIB_PRESS_THRESHOLD;

                    match *key_state {
                        KeyCalibState::Waiting => {
                            if pressed {
                                // Start hold timer on first threshold crossing.
                                *key_state = KeyCalibState::Holding(Instant::now());
                            }
                        }
                        KeyCalibState::Holding(first_seen) => {
                            if pressed {
                                if first_seen.elapsed() >= hold_duration {
                                    // Hold duration satisfied; accept this key.
                                    *key_state = KeyCalibState::Accepted;
                                    calibrated_count = calibrated_count.saturating_add(1);

                                    if let Some(Some(led_idx)) =
                                        MATRIX_TO_LED.get(row).and_then(|r| r.get(col)).copied()
                                    {
                                        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibKeyDone(led_idx)).ok();
                                    }

                                    let pct = u8::try_from(
                                        calibrated_count.saturating_mul(100).checked_div(total_keys).unwrap_or(0),
                                    )
                                    .unwrap_or(100)
                                    .min(100);

                                    if pct != last_pct {
                                        last_pct = pct;
                                        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibProgress(pct)).ok();
                                    }
                                }
                                // else: still within hold window; keep waiting.
                            } else {
                                // Released before hold duration; reset so the
                                // user must press it all the way down again.
                                *key_state = KeyCalibState::Waiting;
                            }
                        }
                        KeyCalibState::Accepted => {}
                    }
                }
            }
        }

        // If all keys were accepted (not just a deadline timeout), signal the
        // backlight to blink green so the user knows to release their keys.
        // Best-effort: missing this signal only skips the animation.
        if calibrated_count >= total_keys {
            BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::AllAccepted)).ok();
        }

        // Phase B: continue sampling for CALIB_SETTLE_AFTER_ALL_DONE_MS
        // regardless of whether Phase A ended by full acceptance or timeout.
        // This gives every accepted key time to reach its true bottom-out ADC
        // rather than storing the value at the moment of acceptance, and still
        // captures the deepest reading seen for any keys that ran out of time.
        let settle_deadline = Instant::now() + Duration::from_millis(CALIB_SETTLE_AFTER_ALL_DONE_MS);
        while Instant::now() < settle_deadline {
            for col in 0..COL {
                self.cols.select(col);
                Timer::after(self.cfg.col_settle_us).await;
                seq.read(&mut buf).await;
                for (row, &raw) in buf.iter().enumerate() {
                    update_min(row, col, raw);
                }
            }
        }

        min_raw
    }

    /// Run the guided first-boot two-phase calibration, persist the result to
    /// EEPROM, and apply it to `self.calib`.
    ///
    /// Backlight signals during the process:
    /// - **Amber** - zero-travel pass, all keys must be released.
    /// - **Blue → green per key** - full-travel press window.
    /// - **Green blink ×3** - all keys accepted; keys may be released.
    /// - **Green for 2 s** - calibration stored successfully.
    /// - **Amber** - EEPROM write-back verification failed; keyboard will
    ///   re-calibrate on the next boot.
    ///
    /// Keys not pressed during the full-travel window fall back to
    /// `zero − DEFAULT_FULL_RANGE` so the keyboard remains functional.
    async fn run_first_boot_calib(
        &mut self,
        eeprom_buf: &mut [u8; CALIB_BUF_LEN],
        entries: &mut [[calib_store::CalibEntry; COL]; ROW],
    ) {
        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Zero)).ok();
        let zero_raw = self.calibrate_zero_raw().await;

        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Full)).ok();
        let full_raw = self.sample_full_raw(self.cfg.full_calib_duration, &zero_raw).await;

        for row in 0..ROW {
            for col in 0..COL {
                let zero = get2(&zero_raw, row, col).unwrap_or(UNCALIBRATED_ZERO);
                let seen_min = get2(&full_raw, row, col).unwrap_or(u16::MAX);
                let full = if zero > seen_min && zero.saturating_sub(seen_min) >= MIN_USEFUL_FULL_RANGE {
                    seen_min.saturating_add(BOTTOM_JITTER).min(zero.saturating_sub(MIN_USEFUL_FULL_RANGE))
                } else {
                    zero.saturating_sub(DEFAULT_FULL_RANGE)
                };
                if let Some(entry) = get2_mut(entries, row, col) {
                    *entry = calib_store::CalibEntry { full };
                }
            }
        }

        self.apply_calib(entries, &zero_raw);

        // Allow the backlight channel to drain before starting the I²C write.
        Timer::after_millis(50).await;
        calib_store::serialize(entries, eeprom_buf);

        let write_ok = self.eeprom.write(EEPROM_BASE_ADDR, eeprom_buf).await.is_ok();

        // Verify by reading back into the same buffer and re-deserializing.
        // Reusing the buffer avoids a second large stack allocation.
        let verified = write_ok
            && self.eeprom.read(EEPROM_BASE_ADDR, eeprom_buf).await.is_ok()
            && calib_store::try_deserialize::<ROW, COL>(eeprom_buf, entries);

        if !verified {
            // Signal amber so the user knows calibration will repeat on the
            // next boot.
            BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Zero)).await;
            return;
        }

        BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Done)).await;
    }

    /// Update the auto-calibration state machine for a single key.
    ///
    /// Called on every scan reading that passes the noise gate, before the
    /// travel computation and rapid-trigger logic. Watches complete
    /// press/release cycles and refines `calib[row][col]` once
    /// [`AUTO_CALIB_CONFIDENCE_THRESHOLD`] confident cycles have accumulated,
    /// compensating for temperature and mechanical drift without requiring a
    /// manual re-calibration.
    ///
    /// A cycle is scored only when the ADC range (zero − full) meets
    /// [`AUTO_CALIB_MIN_RANGE`]; partial presses or noisy readings are
    /// discarded. Updated calibration takes effect immediately on the next
    /// travel computation within the same scan pass.
    fn auto_calib_update(
        auto_calib: &mut [[AutoCalib; COL]; ROW],
        calib: &mut [[KeyCalib; COL]; ROW],
        row: usize,
        col: usize,
        raw: u16,
    ) {
        let Some(ac) = get2_mut(auto_calib, row, col) else { return };

        match ac.phase {
            AutoCalibPhase::Idle => {
                if raw < AUTO_CALIB_FULL_TRAVEL_THRESHOLD {
                    ac.full_candidate = raw;
                    ac.phase = AutoCalibPhase::Pressing;
                }
            }
            AutoCalibPhase::Pressing => {
                if raw < ac.full_candidate {
                    // Key is pressing deeper; track the running minimum.
                    ac.full_candidate = raw;
                } else if raw.saturating_sub(ac.full_candidate) > AUTO_CALIB_RELEASE_THRESHOLD {
                    // Key has lifted enough above the minimum to count as releasing.
                    ac.zero_candidate = raw;
                    ac.phase = AutoCalibPhase::Releasing;
                } else {
                    // Reading is between full_candidate and full_candidate +
                    // AUTO_CALIB_RELEASE_THRESHOLD: bottom-of-travel jitter or
                    // a partial early lift. Stay in
                    // Pressing until a decisive rise.
                }
            }
            AutoCalibPhase::Releasing => {
                if raw > ac.zero_candidate {
                    // Key still rising; update the zero-travel peak.
                    ac.zero_candidate = raw;
                } else if ac.zero_candidate.saturating_sub(raw) <= AUTO_CALIB_ZERO_JITTER {
                    // ADC has settled within jitter of the zero-travel peak;
                    // score the cycle if the range is plausible.
                    let range = ac.zero_candidate.saturating_sub(ac.full_candidate);
                    if range >= AUTO_CALIB_MIN_RANGE {
                        ac.confidence = ac.confidence.saturating_add(1);
                    }

                    if ac.confidence >= AUTO_CALIB_CONFIDENCE_THRESHOLD {
                        ac.confidence = 0;

                        let new_zero = ac.zero_candidate.saturating_sub(ZERO_TRAVEL_DEAD_ZONE);
                        let new_full = ac
                            .full_candidate
                            .saturating_add(BOTTOM_JITTER)
                            .min(new_zero.saturating_sub(MIN_USEFUL_FULL_RANGE));

                        // Only commit the update if the new zero differs
                        // meaningfully from the current one, avoiding
                        // unnecessary scale-factor recomputation.
                        if let Some(cal) = get2_mut(calib, row, col) {
                            if cal.zero.abs_diff(new_zero) > 10 {
                                *cal = KeyCalib::new(new_zero, new_full);
                            }
                        }
                    }

                    ac.phase = AutoCalibPhase::Idle;
                } else {
                    // ADC dropped significantly below zero_candidate - key was
                    // re-pressed before fully releasing. Restart the machine.
                    ac.phase = AutoCalibPhase::Idle;
                }
            }
        }
    }

    /// Scan the matrix once, returning the first key state change found.
    ///
    /// The [`ConfiguredSequence`] is programmed once per invocation and
    /// reused across all columns. Both the column settle delay and the DMA
    /// transfer are fully async. For each reading that passes the noise gate
    /// the auto-calibrator is updated before the travel and rapid-trigger
    /// logic runs, so any calibration refinement takes effect within the same
    /// scan pass.
    #[optimize(speed)]
    async fn scan_for_next_change(
        cols: &mut Hc164Cols<'peripherals>,
        state: &mut [[KeyState; COL]; ROW],
        calib: &mut [[KeyCalib; COL]; ROW],
        auto_calib: &mut [[AutoCalib; COL]; ROW],
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> Option<KeyboardEvent> {
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        // Clamp to 1 so a zero config value never disables the dead-band.
        let sensitivity_press = cfg.rt_sensitivity_press.saturating_mul(TRAVEL_SCALE).max(1);
        let sensitivity_release = cfg.rt_sensitivity_release.saturating_mul(TRAVEL_SCALE).max(1);

        for col in 0..COL {
            cols.select(col);
            Timer::after(cfg.col_settle_us).await;
            seq.read(buf).await;

            // Column stability check: re-read the same column DEBOUNCE_PASSES
            // times and compare each re-read to the first. If any key's
            // pressed state (above / below the actuation threshold) disagrees
            // between the first and a subsequent read, the entire column is
            // skipped for this scan pass. One noisy key invalidates the whole
            // column so no state is mutated on a transient spike.
            //
            // `calib` is read-only here - `auto_calib_update` only runs later,
            // after the stability gate passes, so the calibration data used for
            // both reads is identical.
            //
            // `buf` is restored to `first_read` unconditionally so the per-key
            // noise gate always compares against the same initial reading,
            // regardless of the stability outcome.
            let first_read = *buf;
            // Pre-compute each row's pressed state from the first reading.
            // first_read is constant across all debounce passes, so computing
            // it here avoids repeating the LUT lookup and multiply inside the
            // pass loop for the baseline side of the comparison.
            let first_pressed: [bool; ROW] = core::array::from_fn(|row| {
                get2(calib, row, col)
                    .and_then(|c| Self::travel_scaled_from(&c, first_read[row]))
                    .map_or(false, |t| t >= act_threshold)
            });
            let mut stable = true;
            for _ in 0..DEBOUNCE_PASSES {
                seq.read(buf).await;
                let state_changed =
                    buf.iter().zip(first_pressed.iter()).enumerate().any(|(row, (&recheck, &was_pressed))| {
                        get2(calib, row, col)
                            .and_then(|c| Self::travel_scaled_from(&c, recheck))
                            .map_or(false, |t| t >= act_threshold)
                            != was_pressed
                    });
                if state_changed {
                    stable = false;
                    break;
                }
            }
            *buf = first_read;
            if !stable {
                continue;
            }

            for (row, &raw) in buf.iter().enumerate() {
                let Some(st) = get2_mut(state, row, col) else { continue };
                let last_raw = st.last_raw;

                // Skip if the reading has not changed beyond the noise gate.
                if likely(last_raw.abs_diff(raw) < cfg.noise_gate) {
                    continue;
                }

                // Update the auto-calibrator with this reading before the
                // travel computation so any refined KeyCalib is used immediately.
                Self::auto_calib_update(auto_calib, calib, row, col, raw);

                let Some(cal) = get2(calib, row, col) else { continue };
                let prev_travel = st.travel_scaled;
                let was_pressed = st.pressed;
                let Some(new_travel) = Self::travel_scaled_from(&cal, raw) else { continue };

                st.last_raw = raw;

                if new_travel == prev_travel {
                    continue;
                }

                st.travel_scaled = new_travel;

                // Dynamic Rapid Trigger
                let now_pressed = if was_pressed {
                    // Track the peak while held; release when travel drops at
                    // least `sensitivity_release` below it. The actuation floor
                    // is a hard lower bound that forces an immediate release.
                    st.extremum = st.extremum.max(new_travel);
                    new_travel >= act_threshold && new_travel > st.extremum.saturating_sub(sensitivity_release)
                } else {
                    // Track the trough while released; re-press when travel
                    // climbs at least `sensitivity_press` above it AND exceeds
                    // the actuation floor. The trough is not required to have
                    // dipped below the floor first, so a finger hovering
                    // mid-travel after an RT-release can re-fire immediately.
                    st.extremum = st.extremum.min(new_travel);
                    new_travel >= act_threshold && new_travel >= st.extremum.saturating_add(sensitivity_press)
                };

                if now_pressed != st.pressed {
                    // Reset extremum so the next direction starts fresh from
                    // the transition point.
                    st.extremum = new_travel;
                    st.pressed = now_pressed;
                    return Some(KeyboardEvent::key(
                        u8::try_from(row).unwrap_or_default(),
                        u8::try_from(col).unwrap_or_default(),
                        now_pressed,
                    ));
                }
            }
        }
        None
    }

    /// Convert a raw ADC reading into a scaled travel value.
    ///
    /// Returns `None` if the position is uncalibrated, if `raw` is outside
    /// [`VALID_RAW_MIN`]..=[`VALID_RAW_MAX`], or if the computation overflows.
    #[inline]
    #[optimize(speed)]
    fn travel_scaled_from(cal: &KeyCalib, raw: u16) -> Option<u8> {
        if !cal.used {
            return None;
        }
        if unlikely(!(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw)) {
            return None;
        }
        if cal.poly_delta_full <= 0.0 {
            return None;
        }

        let delta_raw = KeyCalib::poly(f32::from(raw)) - cal.poly_zero;

        Some(
            (delta_raw / cal.poly_delta_full * f32::from(FULL_TRAVEL_SCALED)).clamp(0.0, f32::from(FULL_TRAVEL_SCALED))
                as u8,
        )
    }
}

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize> InputDevice
    for AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    type Event = KeyboardEvent;

    /// Not used - events are published directly via [`publish_event_async`] in
    /// [`Runnable::run`]. Returns [`pending`] to satisfy the trait bound
    /// without competing with the scan loop.
    async fn read_event(&mut self) -> Self::Event { pending().await }
}

impl<'peripherals, ADC, D, IRQ, IM, const ROW: usize, const COL: usize> Runnable
    for AnalogHallMatrix<'peripherals, ADC, D, IRQ, IM, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    IM: MasterMode,
    AdcSampleTime<ADC>: Clone,
{
    async fn run(&mut self) -> ! {
        let mut eeprom_buf = [0_u8; CALIB_BUF_LEN];
        let mut entries = Self::default_entries();

        // Wait for the EEPROM to complete its power-on reset before issuing
        // the first I²C transaction.
        Timer::after(EEPROM_POWER_ON_DELAY).await;

        let loaded = self.eeprom.read(EEPROM_BASE_ADDR, &mut eeprom_buf).await.is_ok()
            && calib_store::try_deserialize::<ROW, COL>(&eeprom_buf, &mut entries);

        if loaded {
            // Re-measure zero travel on every boot to compensate for
            // temperature drift; full-travel data comes from EEPROM.
            let zero_raw = self.calibrate_zero_raw().await;
            self.apply_calib(&entries, &zero_raw);
        } else {
            self.run_first_boot_calib(&mut eeprom_buf, &mut entries).await;
        }

        let mut buf = [0_u16; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);
        loop {
            if let Some(ev) = Self::scan_for_next_change(
                &mut self.cols,
                &mut self.state,
                &mut self.calib,
                &mut self.auto_calib,
                &mut seq,
                &mut buf,
                self.cfg,
            )
            .await
            {
                publish_event_async(ev).await;
            }
        }
    }
}
