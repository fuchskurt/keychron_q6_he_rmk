use crate::matrix::{
    hc164_cols::Hc164Cols,
    travel_helpers::{SCALE_Q_FACTOR, SCALE_SHIFT, delta_from_ref},
};
use core::array::from_fn;
use cortex_m::asm::delay;
use embassy_stm32::adc::{Adc, AnyAdcChannel, BasicAdcRegs, BasicInstance, Instance};
use rmk::{embassy_futures::yield_now, event::KeyboardEvent, macros::input_device};

/// Expected travel distance in physical units, represents 4.0 mm.
const FULL_TRAVEL_UNIT: u8 = 40;
/// Scale factor applied to travel computations.
const TRAVEL_SCALE: u8 = 6;
/// `i64` version of `TRAVEL_SCALE`.
const TRAVEL_SCALE_I64: i64 = i64::from(TRAVEL_SCALE);
/// Full travel value after scaling.
const FULL_TRAVEL_SCALED: u8 = FULL_TRAVEL_UNIT.saturating_mul(TRAVEL_SCALE);
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
const CALIB_PASSES: u16 = 8;

/// ADC sample-time type associated with the selected ADC peripheral.
type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

#[derive(Clone, Copy)]
/// Configuration parameters for hall-effect key sensing.
pub struct HallCfg {
    /// Actuation point in scaled travel units, where value is mm/10.
    pub actuation_pt: u8,
    /// Offset from the actuation point required for de-activation in 0.x mm .
    pub deact_offset: u8,
    /// Delay after switching a column before sampling the ADC.
    pub settle_after_col_cycles: u32,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self { Self { settle_after_col_cycles: 16, actuation_pt: 15, deact_offset: 3 } }
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
    travel_scaled: u8,
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
    /// Cell Storage.
    cells: [[T; COL]; ROW],
}

impl<T: Copy, const ROW: usize, const COL: usize> MatrixGrid<T, ROW, COL> {
    /// Get a reference to a cell if it exists.
    #[inline]
    const fn get(&self, row: usize, col: usize) -> Option<&T> {
        match self.cells.get(row) {
            Some(row_cells) => row_cells.get(col),
            None => None,
        }
    }

    /// Get a mutable reference to a cell if it exists.
    #[inline]
    const fn get_mut(&mut self, row: usize, col: usize) -> Option<&mut T> {
        match self.cells.get_mut(row) {
            Some(row_cells) => row_cells.get_mut(col),
            None => None,
        }
    }

    /// Iterate over all grid cells with their coordinates.
    fn iter_cells(&self) -> impl Iterator<Item = ((usize, usize), &T)> {
        self.cells
            .iter()
            .enumerate()
            .flat_map(|(row_idx, row)| row.iter().enumerate().map(move |(col_idx, cell)| ((row_idx, col_idx), cell)))
    }

    /// Create a new grid filled using the provided initializer.
    fn new(init: impl Fn() -> T) -> Self { Self { cells: from_fn(|_| from_fn(|_| init())) } }
}

/// Hall-effect analog matrix scanner.
#[input_device(publish = KeyboardEvent)]
pub struct AnalogHallMatrix<'peripherals, ADC, const ROW: usize, const COL: usize>
where
    ADC: Instance + BasicInstance,
    ADC::Regs: BasicAdcRegs,
    AdcSampleTime<ADC>: Clone,
{
    /// Actuation threshold in scaled travel units.
    act_threshold: u8,
    /// ADC peripheral used for sampling hall sensors.
    adc: Adc<'peripherals, ADC>,
    /// Calibration data for each key in the matrix.
    calib: MatrixGrid<KeyCalib, ROW, COL>,
    /// Whether calibration has been completed.
    calibrated: bool,
    /// Column driver used to select the active column.
    cols: Hc164Cols<'peripherals>,
    /// De-actuation threshold in scaled travel units.
    deact_threshold: u8,
    /// ADC channels corresponding to each row.
    row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time configuration.
    sample_time: AdcSampleTime<ADC>,
    /// Delay after switching a column before sampling.
    settle_after_col_cycles: u32,
    /// Dynamic per-key state for the matrix.
    state: MatrixGrid<KeyState, ROW, COL>,
}

impl<'peripherals, ADC, const ROW: usize, const COL: usize> AnalogHallMatrix<'peripherals, ADC, ROW, COL>
where
    ADC: Instance + BasicInstance,
    ADC::Regs: BasicAdcRegs,
    AdcSampleTime<ADC>: Clone,
{
    /// Apply a zero offset to a raw reading and clamp to a valid range.
    #[inline]
    const fn apply_zero_offset(zero: u16, raw: u16) -> Option<u16> {
        match apply_ref_offset(zero, raw) {
            Some(x) if x <= REF_ZERO_TRAVEL => Some(x),
            _ => None,
        }
    }

    /// Run the calibration ADC sweep immediately and mark as calibrated.
    ///
    /// Call this during initialization when no stored calibration is available.
    /// After calling, use [`Self::get_zeros`] to retrieve values for EEPROM.
    pub fn calibrate_now(&mut self) { self.calibrate_zero_travel(); }

    /// Collect calibration samples to determine zero-travel values.
    fn calibrate_zero_travel(&mut self) {
        let mut acc: MatrixGrid<u16, ROW, COL> = MatrixGrid::new(|| 0_u16);

        for _ in 0..CALIB_PASSES {
            for col in 0..COL {
                self.cols.select(col);
                delay(self.settle_after_col_cycles);
                let samples = Self::read_row_samples(&mut self.adc, &mut self.row_adc, &self.sample_time);
                for (row, value) in samples.iter().enumerate() {
                    if let Some(cell) = acc.get_mut(row, col) {
                        *cell = cell.saturating_add(*value);
                    }
                }
            }
        }

        for ((row, col), acc_cell) in acc.iter_cells() {
            let avg = (*acc_cell).saturating_div(CALIB_PASSES);
            if let Some(cal_cell) = self.calib.get_mut(row, col) {
                *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
        self.calibrated = true;
    }

    /// Return per-key zero-travel ADC values for EEPROM storage.
    ///
    /// Returns `None` if the matrix has not yet been calibrated.
    pub fn get_zeros(&self) -> Option<[[u16; COL]; ROW]> {
        if !self.calibrated {
            return None;
        }
        Some(from_fn(|row| from_fn(|col| self.calib.get(row, col).map_or(0, |cal| cal.zero))))
    }

    /// Load calibration from previously stored zero-travel ADC values.
    ///
    /// Call this during initialisation if
    /// [`crate::eeprom_storage::EepromStorage`] returned valid data. Sets
    /// `calibrated = true` so the boot-time ADC sweep is skipped entirely.
    pub fn load_from_zeros(&mut self, zeros: &[[u16; COL]; ROW]) {
        for (row, row_slice) in zeros.iter().enumerate() {
            for (col, &zero) in row_slice.iter().enumerate() {
                if let Some(cal) = self.calib.get_mut(row, col) {
                    let full = zero.saturating_sub(DEFAULT_FULL_RANGE);
                    *cal = KeyCalib::new(zero, full);
                }
            }
        }
        self.calibrated = true;
    }

    /// Create a new hall matrix scanner instance.
    pub fn new(
        adc: Adc<'peripherals, ADC>,
        row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
        sample_time: AdcSampleTime<ADC>,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
    ) -> Self {
        let settle_after_col_cycles = cfg.settle_after_col_cycles;
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        let deact_threshold = cfg.actuation_pt.saturating_sub(cfg.deact_offset).saturating_mul(TRAVEL_SCALE);
        Self {
            adc,
            row_adc,
            sample_time,
            cols,
            settle_after_col_cycles,
            act_threshold,
            deact_threshold,
            calib: MatrixGrid::new(KeyCalib::default),
            state: MatrixGrid::new(KeyState::default),
            calibrated: false,
        }
    }

    /// Read the next debounced `KeyboardEvent` from the analog hall matrix.
    async fn read_keyboard_event(&mut self) -> KeyboardEvent {
        if !self.calibrated {
            self.calibrate_zero_travel();
        }
        loop {
            if let Some(ev) = self.scan_for_next_change() {
                return ev;
            }
            yield_now().await;
        }
    }

    /// Read one ADC sample per row for the currently selected column.
    fn read_row_samples(
        adc: &mut Adc<'peripherals, ADC>,
        row_adc: &mut [AnyAdcChannel<'peripherals, ADC>; ROW],
        sample_time: &AdcSampleTime<ADC>,
    ) -> [u16; ROW] {
        from_fn(|row| row_adc.get_mut(row).map_or(0, |ch| adc.blocking_read(ch, sample_time.clone())))
    }

    /// Scan the matrix and return the first key state change found, if any.
    fn scan_for_next_change(&mut self) -> Option<KeyboardEvent> {
        for col in 0..COL {
            self.cols.select(col);
            delay(self.settle_after_col_cycles);
            let samples = Self::read_row_samples(&mut self.adc, &mut self.row_adc, &self.sample_time);

            for (row, &raw) in samples.iter().enumerate() {
                let Some(st) = self.state.get_mut(row, col) else { continue };
                let last_raw = st.last_raw;

                if last_raw != 0 && last_raw.abs_diff(raw) < NOISE_GATE {
                    continue;
                }

                let Some(&cal) = self.calib.get(row, col) else { continue };
                let prev_travel = st.travel_scaled;
                let was_pressed = st.pressed;
                let new_travel = Self::travel_scaled_from(&cal, prev_travel, raw);

                st.last_raw = raw;

                if new_travel == prev_travel {
                    continue;
                }

                st.travel_scaled = new_travel;
                let now_pressed =
                    if was_pressed { new_travel >= self.deact_threshold } else { new_travel >= self.act_threshold };

                if now_pressed != st.pressed {
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

    #[inline]
    /// Convert a raw reading into a scaled travel value.
    fn travel_scaled_from(cal: &KeyCalib, prev: u8, raw: u16) -> u8 {
        if !(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw) {
            return prev;
        }
        let Some(x) = Self::apply_zero_offset(cal.zero, raw) else {
            return prev;
        };
        let delta = i64::from(delta_from_ref(x));
        let prod = delta.saturating_mul(i64::from(cal.scale_factor)).saturating_mul(TRAVEL_SCALE_I64);
        let travel = prod.checked_shr(SCALE_SHIFT).unwrap_or(0);
        u8::try_from(travel.clamp(0_i64, i64::from(FULL_TRAVEL_SCALED))).unwrap_or_default()
    }
}

/// Adjust a raw ADC value by the offset between `zero` and [`REF_ZERO_TRAVEL`].
///
/// Returns `None` if the adjustment overflows or the result exceeds
/// [`REF_ZERO_TRAVEL`].
#[inline]
const fn apply_ref_offset(zero: u16, raw: u16) -> Option<u16> {
    if zero >= REF_ZERO_TRAVEL {
        raw.checked_sub(zero.saturating_sub(REF_ZERO_TRAVEL))
    } else {
        raw.checked_add(REF_ZERO_TRAVEL.saturating_sub(zero))
    }
}

#[inline]
/// Compute a scale factor from zero and full-travel calibration values.
const fn compute_scale_factor(zero: u16, full: u16) -> i32 {
    let x_full: u16 = match apply_ref_offset(zero, full) {
        Some(value) => value,
        None => return SCALE_Q_FACTOR,
    };

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
