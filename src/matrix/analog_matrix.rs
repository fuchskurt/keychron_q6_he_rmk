use crate::matrix::{
    hc164_cols::Hc164Cols,
    travel_helpers::{SCALE_Q_FACTOR, SCALE_SHIFT, delta_from_ref},
};
use core::array::from_fn;
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
const NOISE_GATE: u16 = 2;
/// Number of passes used during calibration.
const CALIB_PASSES: u32 = 64;

/// ADC sample-time type associated with the selected ADC peripheral.
type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

#[derive(Clone, Copy)]
/// Configuration parameters for hall-effect key sensing.
pub struct HallCfg {
    /// Actuation point in scaled travel units, where value is mm/10.
    pub actuation_pt: u8,
    /// Offset from the actuation point required for de-activation in 0.x mm.
    pub deact_offset: u8,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self { Self { actuation_pt: 15, deact_offset: 3 } }
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
    pub const fn new() -> Self { Self { last_raw: u16::MAX, travel_scaled: 0, pressed: false } }
}

impl Default for KeyState {
    /// Provide a default key state.
    fn default() -> Self { Self::new() }
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
    calib: [[KeyCalib; COL]; ROW],
    /// Column driver used to select the active column.
    cols: Hc164Cols<'peripherals>,
    /// De-actuation threshold in scaled travel units.
    deact_threshold: u8,
    /// ADC channels corresponding to each row.
    row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time configuration.
    sample_time: AdcSampleTime<ADC>,
    /// Dynamic per-key state for the matrix.
    state: [[KeyState; COL]; ROW],
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

    /// Collect calibration samples to determine zero-travel values.
    ///
    /// Called once synchronously inside [`Self::new`] before the async
    /// executor starts, so blocking here is intentional and safe.
    fn calibrate_zero_travel(&mut self) {
        let mut acc: [[u32; COL]; ROW] = [[0_u32; COL]; ROW];

        for _ in 0..CALIB_PASSES {
            for col in 0..COL {
                self.cols.select(col);
                let samples = Self::read_row_samples(&mut self.adc, &mut self.row_adc, self.sample_time.clone());
                for (row, value) in samples.iter().enumerate() {
                    if let Some(cell) = acc.get_mut(row).and_then(|row_slice| row_slice.get_mut(col)) {
                        *cell = cell.saturating_add(u32::from(*value));
                    }
                }
            }
        }

        for row in 0..ROW {
            for col in 0..COL {
                let Some(acc_cell) = acc.get(row).and_then(|row_slice| row_slice.get(col)) else { continue };
                let avg = u16::try_from(acc_cell.saturating_div(CALIB_PASSES)).unwrap_or(UNCALIBRATED_ZERO);
                if let Some(cal_cell) = self.calib.get_mut(row).and_then(|row_slice| row_slice.get_mut(col)) {
                    *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
                }
            }
        }
    }

    /// Create a new hall matrix scanner instance.
    ///
    /// Performs a blocking zero-travel calibration pass before returning.
    /// This must be called before the async executor is running.
    pub fn new(
        adc: Adc<'peripherals, ADC>,
        row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
        sample_time: AdcSampleTime<ADC>,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
    ) -> Self {
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        let deact_threshold = cfg.actuation_pt.saturating_sub(cfg.deact_offset).saturating_mul(TRAVEL_SCALE);

        let mut matrix = Self {
            adc,
            row_adc,
            sample_time,
            cols,
            act_threshold,
            deact_threshold,
            calib: [[KeyCalib::uncalibrated(); COL]; ROW],
            state: [[KeyState::new(); COL]; ROW],
        };

        matrix.calibrate_zero_travel();
        matrix
    }

    /// Read the next `KeyboardEvent` from the analog hall matrix.
    async fn read_keyboard_event(&mut self) -> KeyboardEvent {
        loop {
            if let Some(ev) = self.scan_for_next_change() {
                return ev;
            }
            yield_now().await;
        }
    }

    /// Read one ADC sample per row for the currently selected column.
    ///
    /// Performs a single dummy read on the first row before the real pass to
    /// allow the analog lines to settle after column selection. The ADC sample
    /// time provides the active settling window; one read is sufficient since
    /// all rows share the same column bus.
    fn read_row_samples(
        adc: &mut Adc<'peripherals, ADC>,
        row_adc: &mut [AnyAdcChannel<'peripherals, ADC>; ROW],
        sample_time: AdcSampleTime<ADC>,
    ) -> [u16; ROW] {
        if let Some(ch) = row_adc.get_mut(0) {
            let _: u16 = adc.blocking_read(ch, sample_time.clone());
        }
        from_fn(|row| row_adc.get_mut(row).map_or(0, |ch| adc.blocking_read(ch, sample_time.clone())))
    }

    /// Scan the matrix and return the first key state change found, if any.
    fn scan_for_next_change(&mut self) -> Option<KeyboardEvent> {
        for col in 0..COL {
            self.cols.select(col);
            let samples = Self::read_row_samples(&mut self.adc, &mut self.row_adc, self.sample_time.clone());

            for (row, &raw) in samples.iter().enumerate() {
                let Some(st) = self.state.get_mut(row).and_then(|row_slice| row_slice.get_mut(col)) else { continue };
                let last_raw = st.last_raw;

                if last_raw.abs_diff(raw) < NOISE_GATE {
                    continue;
                }

                let Some(&cal) = self.calib.get(row).and_then(|row_slice| row_slice.get(col)) else { continue };
                let prev_travel = st.travel_scaled;
                let was_pressed = st.pressed;
                let new_travel = Self::travel_scaled_from(&cal, raw).unwrap_or(prev_travel);

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
    /// Convert a raw reading into a scaled travel value, returning `None` on
    /// out-of-range or offset failure so the caller decides the fallback.
    fn travel_scaled_from(cal: &KeyCalib, raw: u16) -> Option<u8> {
        if !(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw) {
            return None;
        }
        let x = match Self::apply_zero_offset(cal.zero, raw) {
            Some(x) => x,
            None => return None,
        };
        let delta = i64::from(delta_from_ref(x));
        let prod = delta.saturating_mul(i64::from(cal.scale_factor)).saturating_mul(TRAVEL_SCALE_I64);
        let travel = prod.checked_shr(SCALE_SHIFT).unwrap_or(0);
        Some(u8::try_from(travel.clamp(0_i64, i64::from(FULL_TRAVEL_SCALED))).unwrap_or_default())
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
