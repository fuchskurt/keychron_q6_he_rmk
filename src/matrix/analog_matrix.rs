//! Analog hall-effect matrix scanning and calibration.

use crate::matrix::{
    hc164_cols::Hc164Cols,
    travel_lut::{SCALE_Q_FACTOR, SCALE_SHIFT, delta_from_ref},
};
use core::array::from_fn;
use embassy_stm32::adc::{Adc, AnyAdcChannel, BasicAdcRegs, Instance, SampleTime};
use embassy_time::{Duration, Timer};
use heapless::Deque;
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

/// Expected travel distance in physical units.
const FULL_TRAVEL_UNIT: u16 = 40;
/// Scale factor applied to travel computations.
const TRAVEL_SCALE: u16 = 6;
/// `i64` version of `TRAVEL_SCALE`.
const TRAVEL_SCALE_I64: i64 = i64::from(TRAVEL_SCALE);
/// Full travel value after scaling.
const FULL_TRAVEL_SCALED: i64 = i64::from(FULL_TRAVEL_UNIT.saturating_mul(TRAVEL_SCALE));
/// Default full-range calibration delta.
const DEFAULT_FULL_RANGE: u16 = 900;
/// Minimum acceptable raw ADC value.
const VALID_RAW_MIN: u16 = 1200;
/// Maximum acceptable raw ADC value.
const VALID_RAW_MAX: u16 = 3500;
/// Reference zero-travel value for the LUT.
const REF_ZERO_TRAVEL: u16 = 3121;
/// Fallback zero value when uncalibrated.
const UNCALIBRATED_ZERO: u16 = 3000;
/// Raw ADC delta below which noise is ignored.
const NOISE_GATE: u16 = 5;
/// Number of passes used during calibration.
const CALIB_PASSES: u32 = 8;

#[derive(Clone, Copy)]
/// Configuration parameters for hall-effect key sensing.
pub struct HallCfg {
    /// Actuation point in scaled travel units.
    pub actuation_pt: u16,
    /// Offset from the actuation point required for de-activation.
    pub deact_offset: u16,
    /// Delay after switching a column before sampling the ADC.
    pub settle_after_col: Duration,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self {
        Self { settle_after_col: Duration::from_micros(40), actuation_pt: 20, deact_offset: 3 }
    }
}

#[derive(Clone, Copy)]
/// Calibration values for a single key.
struct KeyCalib {
    /// Fixed-point scale factor used to convert raw delta to travel units.
    scale_factor: i32,
    /// Raw ADC value corresponding to zero travel.
    zero: u16,
}

impl KeyCalib {
    /// Build calibration data from zero and full travel values.
    const fn new(zero: u16, full: u16) -> Self {
        let scale_factor = compute_scale_factor(zero, full);
        Self { scale_factor, zero }
    }

    /// Build an uncalibrated placeholder calibration.
    pub const fn uncalibrated() -> Self {
        Self::new(UNCALIBRATED_ZERO, UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE))
    }
}

impl Default for KeyCalib {
    /// Provide a default uncalibrated calibration record.
    fn default() -> Self { Self::uncalibrated() }
}

#[derive(Clone, Copy)]
/// Dynamic state tracked per key.
struct KeyState {
    /// Last raw ADC reading for this key.
    last_raw: u16,
    /// Whether the key is currently considered pressed.
    pressed: bool,
    /// Current travel value scaled to internal units.
    travel_scaled: u16,
}

impl KeyState {
    /// Create a new default key state.
    pub const fn new() -> Self { Self { last_raw: 0, travel_scaled: 0, pressed: false } }
}

impl Default for KeyState {
    /// Provide a default key state.
    fn default() -> Self { Self::new() }
}

/// Two-dimensional grid for matrix data.
struct MatrixGrid<T, const ROW: usize, const COL: usize> {
    /// Cell storage indexed as `[row][col]`.
    cells: [[T; COL]; ROW],
}

impl<T: Copy, const ROW: usize, const COL: usize> MatrixGrid<T, ROW, COL> {
    #[inline]
    #[expect(clippy::return_and_then, reason = "Needed for this function.")]
    /// Get a reference to a cell if it exists.
    fn get(&self, row: usize, col: usize) -> Option<&T> {
        self.cells.get(row).and_then(|row_cells| row_cells.get(col))
    }

    #[inline]
    #[expect(clippy::return_and_then, reason = "Needed for this function.")]
    /// Get a mutable reference to a cell if it exists.
    fn get_mut(&mut self, row: usize, col: usize) -> Option<&mut T> {
        self.cells.get_mut(row).and_then(|row_cells| row_cells.get_mut(col))
    }

    #[inline]
    /// Iterate over all grid cells with their coordinates.
    fn iter_cells(&self) -> impl Iterator<Item = ((usize, usize), &T)> {
        self.cells.iter().enumerate().flat_map(|(row_iterator, row)| {
            row.iter()
                .enumerate()
                .map(move |(col_iterator, cell)| ((row_iterator, col_iterator), cell))
        })
    }

    /// Create a new grid filled using the provided initializer.
    fn new(mut function: impl FnMut() -> T) -> Self {
        Self { cells: from_fn(|_| from_fn(|_| function())) }
    }
}

/// Hall-effect analog matrix scanner.
pub struct AnalogHallMatrix<'peripherals, ADC, const ROW: usize, const COL: usize>
where
    ADC: Instance,
    ADC::Regs: BasicAdcRegs,
{
    /// Actuation threshold in scaled travel units.
    act_threshold: u16,
    /// ADC peripheral used for sampling hall sensors.
    adc: Adc<'peripherals, ADC>,
    /// Calibration data for each key in the matrix.
    calib: MatrixGrid<KeyCalib, ROW, COL>,
    /// Whether calibration has been completed.
    calibrated: bool,
    /// Column driver used to select the active column.
    cols: Hc164Cols<'peripherals>,
    /// De-actuation threshold in scaled travel units.
    deact_threshold: u16,
    /// Queue of pending keyboard events.
    pending: Deque<KeyboardEvent, 32>,
    /// ADC channels corresponding to each row.
    row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time configuration.
    sample_time: SampleTime,
    /// Delay after switching a column before sampling.
    settle_after_col: Duration,
    /// Dynamic per-key state for the matrix.
    state: MatrixGrid<KeyState, ROW, COL>,
}

impl<'peripherals, ADC, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, ROW, COL>
where
    ADC: Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = SampleTime>,
{
    #[inline]
    /// Apply a zero offset to a raw reading and clamp to a valid range.
    const fn apply_zero_offset(zero: u16, raw: u16) -> Option<u16> {
        if zero >= REF_ZERO_TRAVEL {
            match raw.checked_sub(zero.saturating_sub(REF_ZERO_TRAVEL)) {
                Some(value) => Some(value),
                None => None,
            }
        } else {
            match raw.checked_add(REF_ZERO_TRAVEL.saturating_sub(zero)) {
                Some(value) => Some(value),
                None => None,
            }
        }
    }

    /// Collect calibration samples to determine zero-travel values.
    async fn calibrate_zero_travel(&mut self) {
        let mut acc: MatrixGrid<u32, ROW, COL> = MatrixGrid::new(|| 0_u32);

        let sample_time = self.sample_time;
        let adc = &mut self.adc;
        let row_adc = &mut self.row_adc;

        for _ in 0..CALIB_PASSES {
            for col in 0..COL {
                self.cols.select(col).await;
                Timer::after(self.settle_after_col).await;

                for (row, ch) in row_adc.iter_mut().enumerate() {
                    let value = adc.blocking_read(ch, sample_time);

                    if let Some(cell) = acc.get_mut(row, col) {
                        (*cell) = (*cell).saturating_add(u32::from(value));
                    }
                }

                self.cols.advance().await;
            }
        }

        for ((row, col), acc_cell) in acc.iter_cells() {
            let avg = u16::try_from((*acc_cell).saturating_div(CALIB_PASSES)).unwrap_or_default();

            if let Some(cal_cell) = self.calib.get_mut(row, col) {
                *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
        self.calibrated = true;
    }

    /// Create a new hall matrix scanner instance.
    pub fn new(
        adc: Adc<'peripherals, ADC>,
        row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
        sample_time: SampleTime,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
    ) -> Self {
        let settle_after_col = cfg.settle_after_col;
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        let deact_threshold =
            cfg.actuation_pt.saturating_sub(cfg.deact_offset).saturating_mul(TRAVEL_SCALE);
        Self {
            adc,
            row_adc,
            sample_time,
            cols,
            settle_after_col,
            act_threshold,
            deact_threshold,
            calib: MatrixGrid::new(KeyCalib::default),
            state: MatrixGrid::new(KeyState::default),
            calibrated: false,
            pending: Deque::new(),
        }
    }

    /// Scan the matrix and enqueue any key state changes.
    async fn scan_and_enqueue_changes(&mut self) {
        let sample_time = self.sample_time;
        let adc = &mut self.adc;
        let row_adc = &mut self.row_adc;

        for col in 0..COL {
            self.cols.select(col).await;
            Timer::after(self.settle_after_col).await;

            for (row, ch) in row_adc.iter_mut().enumerate() {
                let raw = adc.blocking_read(ch, sample_time);

                let Some(&cal) = self.calib.get(row, col) else {
                    continue;
                };
                let Some(st) = self.state.get_mut(row, col) else {
                    continue;
                };

                let last_raw = st.last_raw;
                let prev_travel = st.travel_scaled;
                let was_pressed = st.pressed;

                if last_raw != 0 && last_raw.abs_diff(raw) < NOISE_GATE {
                    continue;
                }

                let new_travel = Self::travel_scaled_from(&cal, prev_travel, raw);

                if new_travel == prev_travel {
                    st.last_raw = raw;
                    continue;
                }

                let now_pressed = if was_pressed {
                    new_travel >= self.deact_threshold
                } else {
                    new_travel >= self.act_threshold
                };

                st.last_raw = raw;
                st.travel_scaled = new_travel;

                if now_pressed != st.pressed {
                    st.pressed = now_pressed;

                    let ev = KeyboardEvent::key(
                        u8::try_from(row).unwrap_or_default(),
                        u8::try_from(col).unwrap_or_default(),
                        now_pressed,
                    );
                    if self.pending.push_back(ev).is_err() {
                        if self.pending.pop_front().is_some() {
                        } else {
                            // Queue was unexpectedly empty; nothing to drop
                        }
                        match self.pending.push_back(ev) {
                            Ok(()) => {}
                            Err(_ev) => {
                                // Queue still full (or some invariant broke).
                                // Drop the event.
                                // Intentionally ignore.
                            }
                        }
                    }
                }
            }

            self.cols.advance().await;
        }
    }

    #[inline]
    /// Convert a raw reading into a scaled travel value.
    fn travel_scaled_from(cal: &KeyCalib, prev: u16, raw: u16) -> u16 {
        if !(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw) {
            return prev;
        }

        let x = match Self::apply_zero_offset(cal.zero, raw) {
            Some(x) => x,
            None => return prev,
        };

        if x > REF_ZERO_TRAVEL {
            return 0;
        }

        let delta = i64::from(delta_from_ref(x));
        let prod =
            delta.saturating_mul(i64::from(cal.scale_factor)).saturating_mul(TRAVEL_SCALE_I64);
        let travel = prod.checked_shr(SCALE_SHIFT).unwrap_or(0);

        u16::try_from(travel.clamp(0, FULL_TRAVEL_SCALED)).unwrap_or_default()
    }
}

impl<ADC, const ROW: usize, const COL: usize> InputDevice for AnalogHallMatrix<'_, ADC, ROW, COL>
where
    ADC: Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = SampleTime>,
{
    /// Read the next key event, calibrating on first use.
    async fn read_event(&mut self) -> Event {
        if !self.calibrated {
            self.calibrate_zero_travel().await;
        }

        loop {
            if let Some(ev) = self.pending.pop_front() {
                return Event::Key(ev);
            }

            self.scan_and_enqueue_changes().await;

            if self.pending.is_empty() {
                Timer::after(Duration::from_micros(50)).await;
            }
        }
    }
}

#[inline]
/// Compute a scale factor from zero and full-travel calibration values.
const fn compute_scale_factor(zero: u16, full: u16) -> i32 {
    let x_full: u16 = if zero >= REF_ZERO_TRAVEL {
        match full.checked_sub(zero.saturating_sub(REF_ZERO_TRAVEL)) {
            Some(value) => value,
            None => return SCALE_Q_FACTOR,
        }
    } else {
        match full.checked_add(REF_ZERO_TRAVEL.saturating_sub(zero)) {
            Some(value) => value,
            None => return SCALE_Q_FACTOR,
        }
    };

    let full_travel: i64 = i64::from(delta_from_ref(x_full));
    if full_travel == 0 {
        return SCALE_Q_FACTOR;
    }

    let num = i64::from(FULL_TRAVEL_UNIT)
        .saturating_mul(256_i64)
        .saturating_mul(i64::from(SCALE_Q_FACTOR));
    let Some(quotient) = num.checked_div(full_travel) else {
        return SCALE_Q_FACTOR;
    };
    i32::try_from(quotient).unwrap_or(SCALE_Q_FACTOR)
}
