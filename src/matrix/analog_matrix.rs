//! Analog hall-effect matrix scanner with EEPROM-backed per-key calibration.

use crate::{
    backlight::mapping::MATRIX_TO_LED,
    eeprom::Ft24c64,
    matrix::{
        calib_store,
        hc164_cols::Hc164Cols,
        travel_helpers::{delta_from_ref, SCALE_Q_FACTOR, SCALE_SHIFT},
    },
};
use core::{
    future::pending,
    hint::{likely, unlikely},
};
use embassy_stm32::{
    adc::{Adc, AnyAdcChannel, BasicAdcRegs, BasicInstance, ConfiguredSequence, Instance, RxDma},
    dma::InterruptHandler,
    i2c::mode::MasterMode,
    interrupt::typelevel::Binding,
    pac::adc,
    Peri,
};
use embassy_time::{Duration, Instant, Timer};
use rmk::{
    event::KeyboardEvent,
    input_device::{InputDevice, Runnable},
};

/// Expected travel distance in physical units, represents 4.0 mm.
const FULL_TRAVEL_UNIT: u8 = 40;
/// Scale factor converting physical travel units into internal scaled travel
/// units.
const TRAVEL_SCALE: u8 = 6;
/// `i64` version of `TRAVEL_SCALE`.
const TRAVEL_SCALE_I64: i64 = i64::from(TRAVEL_SCALE);
/// Full travel value expressed in internal scaled travel units.
const FULL_TRAVEL_SCALED: u8 = FULL_TRAVEL_UNIT.saturating_mul(TRAVEL_SCALE);
/// Default full-range calibration delta used when no better value is available.
const DEFAULT_FULL_RANGE: u16 = 900;
/// Maximum acceptable distance from [`REF_ZERO_TRAVEL`] for a key to be
/// considered to have a valid hall-effect sensor at its position.
const CALIB_ZERO_TOLERANCE: u16 = 500;
/// Minimum acceptable raw ADC value.
const VALID_RAW_MIN: u16 = 1200;
/// Maximum acceptable raw ADC value.
const VALID_RAW_MAX: u16 = 3500;
/// Reference zero-travel ADC value used as the LUT origin.
const REF_ZERO_TRAVEL: u16 = 3121;
/// Fallback zero-travel ADC value used before any calibration has run.
const UNCALIBRATED_ZERO: u16 = 3000;
/// Minimum ADC delta (zero − full) required to accept a full-travel sample
/// when validating stored EEPROM entries. A small value is intentional here
/// since this only guards against completely flat or absent readings.
const MIN_USEFUL_FULL_RANGE: u16 = 100;
/// Minimum ADC delta (zero − raw) required to consider a key genuinely pressed
/// during live full-travel calibration sampling.
///
/// Must be large enough to reject ADC noise and resting variation (which can
/// produce ~100 count swings from the averaged zero) while still triggering
/// before the key reaches absolute bottom. Half of [`DEFAULT_FULL_RANGE`] is
/// a reliable margin — a real full-travel press produces ~900 counts of delta,
/// so 450 requires roughly half travel before the key is counted as pressed.
const CALIB_PRESS_THRESHOLD: u16 = 450;
/// Extra sampling time after all keys have been detected as pressed during
/// full-travel calibration.
///
/// Gives the user time to bottom out every key fully before the minimum ADC
/// values are frozen, ensuring the stored full-travel reading reflects the
/// true physical bottom rather than the first crossing of
/// [`CALIB_PRESS_THRESHOLD`].
const CALIB_SETTLE_AFTER_ALL_DONE_MS: u64 = 500;
/// Minimum time the FT24C64 requires between VCC reaching operating voltage
/// and its first valid I²C response (datasheet §Power-On Reset, t_POR = 5 ms).
/// Using 10 ms gives comfortable margin for slow power ramps.
const EEPROM_POWER_ON_DELAY_MS: u64 = 10;

/// Whether each matrix position has a physical hall-effect sensor.
///
/// `false` for `a!(No)` keymap positions (wide-key extensions, empty cells)
/// and virtual positions driven by separate input devices. Specifically:
///
/// - The layer-toggle switch uses `df!()` at row 5 cols 7–8 but is wired to the
///   `PB12` GPIO pin via [`crate::matrix::layer_toggle::LayerToggle`], not to
///   the analog matrix. Its ADC readings are floating garbage.
/// - All `a!(No)` positions in the keymap correspond to the second half of wide
///   keys (2U Backspace, 2.25U LShift, 2.75U RShift, 6.25U Space, etc.) or
///   genuinely empty matrix cells. None of these have hall sensors.
///
/// Excluding these positions from calibration prevents their floating ADC
/// readings from triggering [`CALIB_PRESS_THRESHOLD`] spuriously, which would
/// advance `calibrated_count` to `total_keys` prematurely and cut the
/// full-travel window short before all real keys have been pressed.
const SENSOR_POSITIONS: [[bool; 21]; 6] = [
    // Row 0: all 21 positions populated.
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ],
    // Row 1: all 21 positions populated.
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ],
    // Row 2: all 21 positions populated.
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ],
    // Row 3: cols 13–16 are a!(No) (Enter-right gap + nav gap), col 20 is a!(No)
    //         (KpPlus spans rows 2–3, no second sensor).
    [
        true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, true,
        true, true, false,
    ],
    // Row 4: col 1  = a!(No) (LShift 2.25U extension),
    //         col 12 = a!(No) (RShift 2.75U extension),
    //         col 14 = a!(No) (gap left of Up arrow),
    //         col 16 = a!(No) (gap right of Up arrow).
    [
        true, false, true, true, true, true, true, true, true, true, true, true, false, true, false, true, false, true,
        true, true, true,
    ],
    // Row 5: cols 3–5  = a!(No) (Space 6.25U extension),
    //         cols 7–8  = df!(0)/df!(1) layer-toggle virtual keys, no sensor (PB12 GPIO),
    //         col  13   = a!(No) (RCtrl-right gap),
    //         cols 19–20 = a!(No) (KpEnter spans rows 4–5, no second sensor).
    [
        true, true, true, false, false, false, true, false, false, true, true, true, true, false, true, true, true,
        true, true, false, false,
    ],
];

/// ADC sample-time type associated with the selected ADC peripheral.
type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

#[derive(Clone, Copy)]
/// Configuration parameters for hall-effect key sensing.
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
    /// calibration. The user must press every key to the bottom within this
    /// window for accurate full-travel values to be recorded.
    pub full_calib_duration: Duration,
    /// Raw ADC delta below which back-to-back readings from the same key are
    /// considered noise and discarded without further processing.
    pub noise_gate: u16,
    /// Minimum upward travel from the trough required to register a new press,
    /// expressed in internal scaled travel units.
    pub rt_sensitivity_press: u8,
    /// Minimum downward travel from the peak required to register a release,
    /// expressed in internal scaled travel units.
    pub rt_sensitivity_release: u8,
    /// CPU cycles to hold each HC164 pin transition stable before moving to
    /// the next state.
    pub shifter_delay_cycles: u32,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self {
        Self {
            actuation_pt: 6,
            calib_passes: 512,
            col_settle_us: Duration::from_micros(35),
            full_calib_duration: Duration::from_secs(360),
            noise_gate: 2,
            rt_sensitivity_press: 8,
            rt_sensitivity_release: 6,
            shifter_delay_cycles: 8,
        }
    }
}

#[derive(Clone, Copy)]
/// Calibration values for a single key.
struct KeyCalib {
    /// Fixed-point scale factor used to convert a raw LUT delta into internal
    /// scaled travel units. Derived from the measured zero-to-full ADC range
    /// at calibration time.
    scale_factor: i32,
    /// Whether this matrix position has a valid hall-effect sensor.
    ///
    /// `false` for positions that are unpopulated or whose zero-travel reading
    /// is too far from [`REF_ZERO_TRAVEL`] to be a real key.
    used: bool,
    /// Raw ADC value corresponding to zero travel (key fully released).
    zero: u16,
}

impl KeyCalib {
    /// Build calibration data from zero and full-travel ADC readings.
    const fn new(zero: u16, full: u16) -> Self {
        Self {
            scale_factor: compute_scale_factor(zero, full),
            used: zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE,
            zero,
        }
    }

    /// Build an uncalibrated placeholder calibration.
    ///
    /// Used to fill the matrix before any calibration data is available.
    /// `used` is `false` so the scanner ignores this position entirely.
    pub const fn uncalibrated() -> Self { Self { scale_factor: SCALE_Q_FACTOR, used: false, zero: UNCALIBRATED_ZERO } }
}

impl Default for KeyCalib {
    /// Provide a default uncalibrated calibration record.
    fn default() -> Self { Self::uncalibrated() }
}

#[derive(Clone, Copy)]
/// Dynamic state tracked per key across scan iterations.
struct KeyState {
    /// Local extremum used for rapid-trigger calculations.
    ///
    /// While pressed: tracks the highest travel reached (peak).
    /// While released: tracks the lowest travel reached (trough).
    /// Reset to `new_travel` on every press/release transition so each new
    /// direction starts tracking from the transition point.
    extremum: u8,
    /// Raw ADC reading from the previous scan cycle.
    ///
    /// Compared against the new reading via the noise gate; the inner scan
    /// loop is skipped when the delta is below `HallCfg::noise_gate`.
    last_raw: u16,
    /// Whether the key is currently considered pressed.
    pressed: bool,
    /// Travel value from the previous scan cycle, in internal scaled units.
    ///
    /// Used to detect whether travel has actually changed after the noise
    /// gate passes, avoiding redundant RT computations.
    travel_scaled: u8,
}

impl KeyState {
    /// Create a new default key state.
    ///
    /// `extremum` is initialised to `u8::MAX` so the first real reading
    /// immediately establishes a valid trough without a false press.
    /// `last_raw` is `u16::MAX` so the first ADC reading always passes the
    /// noise gate regardless of its value.
    pub const fn new() -> Self { Self { extremum: u8::MAX, last_raw: u16::MAX, travel_scaled: 0, pressed: false } }
}

impl Default for KeyState {
    /// Provide a default key state.
    fn default() -> Self { Self::new() }
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
    /// Create a [`ConfiguredSequence`] over all row channels, programming the
    /// ADC sequence registers once.
    ///
    /// The returned reader can be triggered repeatedly without reprogramming
    /// the sequence registers, saving ~20 cycles per column per scan.
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

/// Hall-effect analog matrix scanner with EEPROM-backed per-key calibration.
///
/// On first boot (or after EEPROM corruption) the firmware performs a guided
/// two-phase calibration:
///
/// 1. **Zero-travel pass** — user keeps all keys fully released; the firmware
///    averages `HallCfg::calib_passes` reads per key to establish resting ADC
///    values. Backlight signals amber.
/// 2. **Full-travel pass** — user presses every key to the bottom within
///    `HallCfg::full_calib_duration`; the firmware records the minimum ADC
///    reading seen per key. Backlight signals blue→green per key as each is
///    confirmed pressed. Only positions listed as `true` in
///    [`SENSOR_POSITIONS`] are counted, ensuring `a!(No)` cells and virtual
///    layer-toggle positions cannot trigger false completions.
///
/// After all real keys are detected a [`CALIB_SETTLE_AFTER_ALL_DONE_MS`] window
/// continues sampling so the stored value reflects true bottom-out ADC, not
/// merely the threshold-crossing instant. Validated entries are written to the
/// FT24C64 EEPROM and verified by read-back before the keyboard enters normal
/// operation. Backlight turns green for 2 s on success or amber on write
/// failure.
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
    /// Per-key calibration data applied during the scan loop.
    calib: [[KeyCalib; COL]; ROW],
    /// Sensing and scanning configuration.
    cfg: HallCfg,
    /// Column driver used to select the active column via the HC164.
    cols: Hc164Cols<'peripherals>,
    /// FT24C64 EEPROM driver for loading and persisting calibration data.
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
    /// Calibration is deferred to [`Runnable::run`], which attempts to load
    /// stored data from EEPROM and falls back to a guided first-boot
    /// calibration pass if no valid data is found.
    pub fn new(
        adc_part: AdcPart<'peripherals, ADC, D, ROW>,
        irq: IRQ,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
        eeprom: Ft24c64<'peripherals, IM>,
    ) -> Self {
        Self {
            adc_part,
            irq,
            cols,
            cfg,
            eeprom,
            calib: [[KeyCalib::uncalibrated(); COL]; ROW],
            state: [[KeyState::new(); COL]; ROW],
        }
    }

    /// Build the default calibration entries used when EEPROM is blank or
    /// invalid.
    ///
    /// Populates every key with [`UNCALIBRATED_ZERO`] as the zero-travel value
    /// and `UNCALIBRATED_ZERO − DEFAULT_FULL_RANGE` as the full-travel value,
    /// giving a reasonable approximation of travel until real calibration runs.
    const fn default_entries() -> [[calib_store::CalibEntry; COL]; ROW] {
        [[calib_store::CalibEntry {
            zero: UNCALIBRATED_ZERO,
            full: UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE),
        }; COL]; ROW]
    }

    /// Apply a row-major array of [`calib_store::CalibEntry`] values to
    /// `self.calib`, converting each pair of ADC readings into a
    /// [`KeyCalib`] with a pre-computed scale factor.
    fn apply_calib(&mut self, entries: &[[calib_store::CalibEntry; COL]; ROW]) {
        for (calib_row, entry_row) in self.calib.iter_mut().zip(entries.iter()) {
            for (cal, entry) in calib_row.iter_mut().zip(entry_row.iter()) {
                *cal = KeyCalib::new(entry.zero, entry.full);
            }
        }
    }

    /// Average `cfg.calib_passes` full-matrix scans to establish per-key
    /// zero-travel (resting) ADC values.
    ///
    /// All keys must be fully released during this pass. Returns a
    /// `ROW × COL` array of raw ADC averages without modifying `self.calib`.
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
                    if let Some(cell) = acc.get_mut(row).and_then(|r| r.get_mut(col)) {
                        *cell = cell.saturating_add(u32::from(raw));
                    }
                }
            }
        }

        let mut result = [[UNCALIBRATED_ZERO; COL]; ROW];
        for (res_row, acc_row) in result.iter_mut().zip(acc.iter()) {
            for (res, &total) in res_row.iter_mut().zip(acc_row.iter()) {
                *res = u16::try_from(total.checked_div(self.cfg.calib_passes).unwrap_or_default())
                    .unwrap_or(UNCALIBRATED_ZERO);
            }
        }
        result
    }

    /// Continuously scan the matrix for `duration` (or until all real keys are
    /// confirmed pressed, whichever comes first), recording the minimum ADC
    /// reading seen per key.
    ///
    /// Since lower ADC = more magnet travel for this sensor type, the minimum
    /// reading over the window corresponds to the deepest press seen.
    ///
    /// Only positions marked `true` in [`SENSOR_POSITIONS`] are eligible for
    /// detection, preventing `a!(No)` cells and virtual layer-toggle positions
    /// (df!, driven by PB12 GPIO not the analog matrix) from contributing
    /// spurious detections that would advance `calibrated_count` prematurely
    /// and cut the full-travel window short.
    ///
    /// After `calibrated_count` reaches `total_keys`, a
    /// [`CALIB_SETTLE_AFTER_ALL_DONE_MS`] continuation window keeps sampling
    /// so the stored full-travel value reflects each key's true physical
    /// bottom-out ADC rather than the instant it crossed the detection
    /// threshold. No new backlight signals are sent during the settle window.
    async fn sample_full_raw(&mut self, duration: Duration, zero_raw: &[[u16; COL]; ROW]) -> [[u16; COL]; ROW] {
        use crate::backlight::lock_indicator::{BacklightCmd, BACKLIGHT_CH};

        let deadline = Instant::now() + duration;
        let mut min_raw = [[u16::MAX; COL]; ROW];
        let mut calibrated = [[false; COL]; ROW];
        let mut calibrated_count: usize = 0;

        // Count only positions that both have a physical sensor (per
        // SENSOR_POSITIONS) and have a plausible zero-travel reading (within
        // CALIB_ZERO_TOLERANCE of REF_ZERO_TRAVEL). The denominator must
        // match exactly what gets counted in the detection loop below.
        let total_keys = (0..ROW)
            .flat_map(|r| (0..COL).map(move |c| (r, c)))
            .filter(|&(r, c)| {
                SENSOR_POSITIONS.get(r).and_then(|row| row.get(c)).copied().unwrap_or(false)
                    && zero_raw
                        .get(r)
                        .and_then(|row| row.get(c))
                        .map_or(false, |&z| z.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE)
            })
            .count()
            .max(1);

        let mut seq = self.adc_part.configured_sequence(self.irq);
        let mut buf = [0_u16; ROW];
        let mut last_pct: u8 = 0;

        // Phase A: sample until every real key has been detected as pressed
        // OR the safety deadline expires. The SENSOR_POSITIONS gate ensures
        // that a!(No) cells and virtual layer-toggle positions cannot trigger
        // false completions regardless of their floating ADC values.
        while Instant::now() < deadline && calibrated_count < total_keys {
            for col in 0..COL {
                self.cols.select(col);
                Timer::after(self.cfg.col_settle_us).await;
                seq.read(&mut buf).await;

                for (row, &raw) in buf.iter().enumerate() {
                    // Always track the running minimum — even for positions
                    // not yet confirmed pressed — so we record the deepest
                    // reading seen over the whole window.
                    if let Some(cell) = min_raw.get_mut(row).and_then(|r| r.get_mut(col)) {
                        if raw < *cell {
                            *cell = raw;
                        }
                    }

                    // Skip positions without a physical sensor. This is the
                    // primary guard against df!() layer-toggle columns (7, 8
                    // in row 5) whose floating ADC can exceed CALIB_PRESS_THRESHOLD
                    // and would otherwise cause premature early exit.
                    if !SENSOR_POSITIONS.get(row).and_then(|r| r.get(col)).copied().unwrap_or(false) {
                        continue;
                    }

                    let done_flag = calibrated.get_mut(row).and_then(|r| r.get_mut(col));
                    let zero = zero_raw.get(row).and_then(|r| r.get(col)).copied().unwrap_or(0);

                    if let Some(flag) = done_flag {
                        if !*flag && zero > raw && zero.saturating_sub(raw) >= CALIB_PRESS_THRESHOLD {
                            *flag = true;
                            calibrated_count = calibrated_count.saturating_add(1);

                            // Immediately light this key green in the backlight.
                            if let Some(Some(led_idx)) = MATRIX_TO_LED.get(row).and_then(|r| r.get(col)) {
                                BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibKeyDone(*led_idx)).ok();
                            }

                            let pct =
                                u8::try_from(calibrated_count.saturating_mul(100).checked_div(total_keys).unwrap_or(0))
                                    .unwrap_or(100)
                                    .min(100);

                            // Only send when the integer percentage changes;
                            // drops under load are acceptable since this is
                            // display-only.
                            if pct != last_pct {
                                last_pct = pct;
                                BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibProgress(pct)).ok();
                            }
                        }
                    }
                }
            }
        }

        // Phase B: continue sampling for CALIB_SETTLE_AFTER_ALL_DONE_MS after
        // all keys are detected. This captures the true bottom-out ADC value
        // for every key, including the last one pressed whose min_raw would
        // otherwise be frozen at its threshold-crossing value rather than its
        // physical bottom. No backlight signals are sent; the display is
        // already fully green.
        let settle_deadline = Instant::now() + Duration::from_millis(CALIB_SETTLE_AFTER_ALL_DONE_MS);
        while Instant::now() < settle_deadline {
            for col in 0..COL {
                self.cols.select(col);
                Timer::after(self.cfg.col_settle_us).await;
                seq.read(&mut buf).await;
                for (row, &raw) in buf.iter().enumerate() {
                    if let Some(cell) = min_raw.get_mut(row).and_then(|r| r.get_mut(col)) {
                        if raw < *cell {
                            *cell = raw;
                        }
                    }
                }
            }
        }

        min_raw
    }

    /// Run the guided first-boot two-phase calibration, persist the result to
    /// EEPROM, and apply it to `self.calib`.
    ///
    /// The backlight task is signalled at each phase transition via
    /// [`BACKLIGHT_CH`](crate::backlight::lock_indicator::BACKLIGHT_CH):
    /// - **Amber** during the zero-travel pass (all keys released).
    /// - **Blue → green** during the full-travel press window; individual keys
    ///   turn green as they are confirmed pressed.
    /// - **Green** for 2 s after the data is successfully verified in EEPROM.
    /// - **Amber** again if the EEPROM write-back verification fails,
    ///   signalling that the keyboard will re-calibrate on the next boot.
    ///
    /// Keys not pressed during the full-travel window fall back to
    /// `zero − DEFAULT_FULL_RANGE` so they remain functional without a manual
    /// re-calibration cycle. The EEPROM write is verified by reading the data
    /// back and re-running the CRC check before the Done signal is sent.
    ///
    /// The verification reuses `eeprom_buf` (already 511 bytes) rather than
    /// allocating a second large buffer, keeping peak stack usage in this
    /// function to a minimum.
    async fn run_first_boot_calib(
        &mut self,
        eeprom_buf: &mut [u8],
        entries: &mut [[calib_store::CalibEntry; COL]; ROW],
    ) {
        use crate::backlight::lock_indicator::{BacklightCmd, CalibPhase, BACKLIGHT_CH};

        // Phase 1: zero-travel — all keys must be fully released.
        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Zero)).ok();
        let zero_raw = self.calibrate_zero_raw().await;

        // Phase 2: full-travel — user presses every key to the bottom.
        BACKLIGHT_CH.sender().try_send(BacklightCmd::CalibPhase(CalibPhase::Full)).ok();
        let full_raw = self.sample_full_raw(self.cfg.full_calib_duration, &zero_raw).await;

        // Validate and build entries. Keys whose full-travel delta is too
        // small (never pressed or sensor absent) retain the default range
        // so the matrix stays functional even if the user skips some keys.
        for row in 0..ROW {
            for col in 0..COL {
                let zero = zero_raw[row][col];
                let seen_min = full_raw[row][col];
                let full = if zero > seen_min && zero.saturating_sub(seen_min) >= MIN_USEFUL_FULL_RANGE {
                    seen_min
                } else {
                    // Key was not pressed or delta too small to trust.
                    zero.saturating_sub(DEFAULT_FULL_RANGE)
                };
                if let Some(entry) = entries.get_mut(row).and_then(|r| r.get_mut(col)) {
                    *entry = calib_store::CalibEntry { zero, full };
                }
            }
        }

        self.apply_calib(entries);

        // Serialize into the shared buffer and write to EEPROM.
        Timer::after_millis(50).await;
        calib_store::serialize(entries, eeprom_buf);
        let write_ok = self.eeprom.write(calib_store::EEPROM_BASE_ADDR, eeprom_buf).await.is_ok();

        // Verify by reading back into the same buffer and deserializing into
        // `entries` (which already holds the correct values). Reusing
        // `eeprom_buf` avoids a second 511-byte stack allocation that would
        // otherwise push total stack usage in this function over ~2 KB and
        // risk corrupting the read via stack overflow. If deserialization
        // succeeds the CRC and magic byte are valid and the write is confirmed.
        let verified = write_ok
            && matches!(self.eeprom.read(calib_store::EEPROM_BASE_ADDR, eeprom_buf).await, Ok(()))
            && calib_store::try_deserialize::<ROW, COL>(eeprom_buf, entries);

        if !verified {
            // Write-back verification failed — signal amber so the user knows
            // calibration will repeat on the next boot.
            BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Zero)).await;
            return;
        }

        // Blocking send so the Done signal is never dropped even if the
        // backlight channel is momentarily full.
        BACKLIGHT_CH.sender().send(BacklightCmd::CalibPhase(CalibPhase::Done)).await;
    }

    /// Scan the matrix once and return the first key state change found.
    ///
    /// The [`ConfiguredSequence`] is programmed once per invocation and
    /// reused across all columns, avoiding per-column sequence register
    /// writes. Both the column settle delay and the DMA transfer are fully
    /// async.
    #[optimize(speed)]
    async fn scan_for_next_change(
        cols: &mut Hc164Cols<'peripherals>,
        state: &mut [[KeyState; COL]; ROW],
        calib: &[[KeyCalib; COL]; ROW],
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        cfg: HallCfg,
    ) -> Option<KeyboardEvent> {
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        // Clamp to 1 so a zero config value never disables the dead-band entirely.
        let sensitivity_press = cfg.rt_sensitivity_press.max(1);
        let sensitivity_release = cfg.rt_sensitivity_release.max(1);

        for col in 0..COL {
            cols.select(col);
            Timer::after(cfg.col_settle_us).await;
            seq.read(buf).await;

            for (row, ((state_row, calib_row), &raw)) in state.iter_mut().zip(calib.iter()).zip(buf.iter()).enumerate()
            {
                let Some(st) = state_row.get_mut(col) else { continue };
                let last_raw = st.last_raw;

                // Fast path: skip processing if the reading has not changed
                // beyond the noise gate threshold.
                if likely(last_raw.abs_diff(raw) < cfg.noise_gate) {
                    continue;
                }

                let Some(&cal) = calib_row.get(col) else { continue };
                let prev_travel = st.travel_scaled;
                let was_pressed = st.pressed;
                let Some(new_travel) = Self::travel_scaled_from(&cal, raw) else {
                    continue;
                };

                st.last_raw = raw;

                if new_travel == prev_travel {
                    continue;
                }

                st.travel_scaled = new_travel;

                // Dynamic Rapid Trigger — always active.
                let now_pressed = if was_pressed {
                    // Track the highest point reached while the key is held so
                    // we can measure downward travel from the peak.
                    if new_travel > st.extremum {
                        st.extremum = new_travel;
                    }
                    // Release when the key has dropped at least
                    // `sensitivity_release` units from the recorded peak. The
                    // actuation floor is a hard lower bound: if travel falls
                    // below it the key always releases immediately regardless
                    // of the RT delta.
                    new_travel >= act_threshold && new_travel > st.extremum.saturating_sub(sensitivity_release)
                } else {
                    // Track the lowest point reached while the key is up so we
                    // can measure upward travel from the trough.
                    if new_travel < st.extremum {
                        st.extremum = new_travel;
                    }
                    // Re-press when travel has climbed at least
                    // `sensitivity_press` units from the trough AND is above
                    // the actuation floor. Matching QMK behaviour: the trough
                    // is not required to have dipped below the floor first, so
                    // a finger hovering mid-travel after an RT-release can
                    // re-fire as soon as it rises by the press sensitivity.
                    new_travel >= act_threshold && new_travel >= st.extremum.saturating_add(sensitivity_press)
                };

                if now_pressed != st.pressed {
                    // Reset extremum so the next direction track starts fresh
                    // from the transition point rather than inheriting a stale
                    // peak or trough from the previous direction.
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
    /// Returns `None` if the key position is uncalibrated (`cal.used ==
    /// false`), if `raw` is outside [`VALID_RAW_MIN`]..=[`VALID_RAW_MAX`], or
    /// if the travel computation overflows, allowing the caller to decide the
    /// fallback behaviour.
    #[inline]
    #[optimize(speed)]
    fn travel_scaled_from(cal: &KeyCalib, raw: u16) -> Option<u8> {
        if !cal.used {
            return None;
        }
        if unlikely(!(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw)) {
            return None;
        }
        // Shift the raw reading into the LUT's reference frame, then clamp to
        // the zero-travel reference so upward travel (key lifting above rest)
        // always maps to zero rather than a negative delta.
        let x = apply_ref_offset(cal.zero, raw).min(REF_ZERO_TRAVEL);

        let delta = i64::from(delta_from_ref(x));
        let prod = delta.saturating_mul(i64::from(cal.scale_factor)).saturating_mul(TRAVEL_SCALE_I64);
        let travel = prod.checked_shr(SCALE_SHIFT).unwrap_or(0);
        Some(u8::try_from(travel.clamp(0_i64, i64::from(FULL_TRAVEL_SCALED))).unwrap_or_default())
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
        use ::rmk::event::publish_event_async;

        // Stack-allocate the EEPROM scratch buffer and entry table once; both
        // are reused for the load path and, if calibration runs, the store and
        // verify paths. Keeping a single 511-byte buffer (rather than two)
        // prevents the combined stack frame from exceeding safe limits on the
        // 64 K RAM STM32F401RC.
        let mut eeprom_buf = [0_u8; calib_store::CALIB_BUF_LEN];
        let mut entries = Self::default_entries();

        // Wait for the FT24C64 to complete its internal power-on reset
        // sequence before issuing the first I²C transaction. The MCU boots
        // significantly faster than the EEPROM becomes ready; without this
        // delay the first read NAKs, `loaded` is false, and calibration
        // reruns on every power cycle.
        Timer::after_millis(EEPROM_POWER_ON_DELAY_MS).await;

        // Attempt to load previously stored calibration from EEPROM.
        // Both the I²C read and the deserialization (magic + version + CRC)
        // must succeed for the data to be considered valid; any failure falls
        // through to first-boot calibration.
        let loaded = matches!(self.eeprom.read(calib_store::EEPROM_BASE_ADDR, &mut eeprom_buf).await, Ok(()))
            && calib_store::try_deserialize::<ROW, COL>(&eeprom_buf, &mut entries);

        if loaded {
            // Known-good EEPROM data — apply immediately and skip calibration.
            self.apply_calib(&entries);
        } else {
            // First boot or corrupted EEPROM — run the guided two-phase
            // calibration with backlight feedback and persist the result.
            self.run_first_boot_calib(&mut eeprom_buf, &mut entries).await;
        }

        // Main scan loop — calibration is guaranteed complete by this point.
        let mut buf = [0_u16; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);
        loop {
            if let Some(ev) =
                Self::scan_for_next_change(&mut self.cols, &mut self.state, &self.calib, &mut seq, &mut buf, self.cfg)
                    .await
            {
                publish_event_async(ev).await;
            }
        }
    }
}

/// Adjust a raw ADC value by the per-key offset between `zero` and
/// [`REF_ZERO_TRAVEL`].
///
/// Keys whose resting ADC differs from the LUT reference point are shifted
/// into the LUT's reference frame before lookup, compensating for
/// manufacturing variation in magnet strength and sensor placement.
#[inline]
const fn apply_ref_offset(zero: u16, raw: u16) -> u16 {
    if zero >= REF_ZERO_TRAVEL {
        raw.saturating_sub(zero.saturating_sub(REF_ZERO_TRAVEL))
    } else {
        raw.saturating_add(REF_ZERO_TRAVEL.saturating_sub(zero)).min(REF_ZERO_TRAVEL)
    }
}

/// Compute a fixed-point scale factor from zero and full-travel ADC readings.
///
/// The scale factor normalizes the LUT delta (in Q8) to the known physical
/// full-travel distance ([`FULL_TRAVEL_UNIT`]), producing a per-key Q16
/// multiplier stored in [`KeyCalib::scale_factor`]. If the full-travel delta
/// is zero (sensor absent or identical readings) the identity factor
/// [`SCALE_Q_FACTOR`] is returned as a safe fallback.
#[inline]
const fn compute_scale_factor(zero: u16, full: u16) -> i32 {
    let x_full = apply_ref_offset(zero, full);
    let full_travel: i64 = i64::from(delta_from_ref(x_full));
    if full_travel == 0 {
        return SCALE_Q_FACTOR;
    }
    let num = i64::from(FULL_TRAVEL_UNIT).saturating_mul(256_i64).saturating_mul(i64::from(SCALE_Q_FACTOR));
    match num.checked_div(full_travel) {
        Some(quotient) => i32::try_from(quotient).unwrap_or(SCALE_Q_FACTOR),
        None => SCALE_Q_FACTOR,
    }
}
