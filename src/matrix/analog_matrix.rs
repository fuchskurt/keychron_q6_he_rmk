use crate::matrix::{
    hc164_cols::Hc164Cols,
    travel_helpers::{SCALE_Q_FACTOR, SCALE_SHIFT, delta_from_ref},
};
use core::hint::{cold_path, likely, unlikely};
use embassy_stm32::{
    adc::{Adc, AnyAdcChannel, BasicAdcRegs, BasicInstance, Instance, RxDma},
    dma,
    interrupt::typelevel,
    pac::adc,
};
use rmk::{event::KeyboardEvent, macros::input_device};

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

/// ADC sample-time type associated with the selected ADC peripheral.
type AdcSampleTime<ADC> = <<ADC as BasicInstance>::Regs as BasicAdcRegs>::SampleTime;

#[derive(Clone, Copy)]
/// Configuration parameters for hall-effect key sensing.
pub struct HallCfg {
    /// Actuation point in scaled travel units, where value is mm/10.
    pub actuation_pt: u8,
    /// Number of passes used during calibration.
    pub calib_passes: u32,
    /// Offset from the actuation point required for de-activation in 0.x mm.
    pub deact_offset: u8,
    /// Raw ADC delta below which readings are treated as noise (default: 2).
    pub noise_gate: u16,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self { Self { actuation_pt: 15, deact_offset: 3, noise_gate: 2, calib_passes: 64 } }
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
    const fn new(zero: u16, full: u16) -> Self { Self { scale_factor: compute_scale_factor(zero, full), zero } }

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
///
/// Uses DMA-backed ADC reads for all sampling including the initial
/// zero-travel calibration, keeping the firmware fully non-blocking
/// from construction onwards.
#[input_device(publish = KeyboardEvent)]
pub struct AnalogHallMatrix<'peripherals, ADC, D, IRQ, const ROW: usize, const COL: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: typelevel::Binding<D::Interrupt, dma::InterruptHandler<D>> + Copy,
    AdcSampleTime<ADC>: Clone,
{
    /// Actuation threshold in scaled travel units.
    act_threshold: u8,
    /// ADC peripheral used for sampling hall sensors.
    adc: Adc<'peripherals, ADC>,
    /// Calibration data for each key in the matrix.
    calib: [[KeyCalib; COL]; ROW],
    /// Number of passes used during calibration.
    calib_passes: u32,
    /// Column driver used to select the active column.
    cols: Hc164Cols<'peripherals>,
    /// De-actuation threshold in scaled travel units.
    deact_threshold: u8,
    /// DMA channel used for non-blocking ADC sequence reads.
    dma: embassy_stm32::Peri<'peripherals, D>,
    /// DMA interrupt binding used for ADC sequence reads.
    irq: IRQ,
    /// Raw ADC delta below which readings are treated as noise.
    noise_gate: u16,
    /// ADC channels corresponding to each row.
    row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time configuration.
    sample_time: AdcSampleTime<ADC>,
    /// Dynamic per-key state for the matrix.
    state: [[KeyState; COL]; ROW],
}

impl<'peripherals, ADC, D, IRQ, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, IRQ, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: typelevel::Binding<D::Interrupt, dma::InterruptHandler<D>> + Copy,
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

    /// Perform zero-travel calibration using DMA-backed ADC reads.
    ///
    /// Samples each key `calib_passes` times and averages the results to
    /// establish a per-key resting ADC value. Called once inside [`Self::new`]
    /// before the matrix enters the scan loop.
    async fn calibrate_zero_travel(&mut self) {
        let mut acc: [[u32; COL]; ROW] = [[0_u32; COL]; ROW];

        for _ in 0..self.calib_passes {
            for col in 0..COL {
                self.cols.select(col);
                let buf = self.read_row_samples().await;
                for (row, &sample) in buf.iter().enumerate() {
                    if let Some(cell) = acc.get_mut(row).and_then(|r| r.get_mut(col)) {
                        *cell = cell.saturating_add(u32::from(sample));
                    }
                }
            }
        }

        for (calib_row, acc_row) in self.calib.iter_mut().zip(acc.iter()) {
            for (cal_cell, &acc_cell) in calib_row.iter_mut().zip(acc_row.iter()) {
                let avg = u16::try_from(acc_cell.checked_div(self.calib_passes).unwrap_or_default())
                    .unwrap_or(UNCALIBRATED_ZERO);
                *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
    }

    /// Create a new matrix scanner and perform zero-travel calibration.
    ///
    /// Calibration uses DMA-backed ADC reads and runs asynchronously.
    /// The calibration takes approximately 7.7 ms and is invisible behind
    /// USB enumeration time. After this returns the matrix is ready to scan.
    pub async fn new(
        adc: Adc<'peripherals, ADC>,
        row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
        dma: embassy_stm32::Peri<'peripherals, D>,
        irq: IRQ,
        sample_time: AdcSampleTime<ADC>,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
    ) -> Self {
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        let deact_threshold = cfg.actuation_pt.saturating_sub(cfg.deact_offset).saturating_mul(TRAVEL_SCALE);

        let mut matrix = Self {
            adc,
            row_adc,
            dma,
            irq,
            sample_time,
            cols,
            act_threshold,
            deact_threshold,
            noise_gate: cfg.noise_gate,
            calib_passes: cfg.calib_passes,
            calib: [[KeyCalib::uncalibrated(); COL]; ROW],
            state: [[KeyState::new(); COL]; ROW],
        };

        matrix.calibrate_zero_travel().await;
        matrix
    }

    /// Read the next `KeyboardEvent` from the analog hall matrix.
    async fn read_keyboard_event(&mut self) -> KeyboardEvent {
        loop {
            if let Some(ev) = self.scan_for_next_change().await {
                return ev;
            }
        }
    }

    /// Read all ROW channels via DMA for the currently selected column.
    ///
    /// The ADC sample time (15 cycles at 42 MHz ≈ 357 ns) exceeds the
    /// HC164 propagation delay (~20–50 ns), so no separate analog settling
    /// step is required before the DMA transfer.
    async fn read_row_samples(&mut self) -> [u16; ROW] {
        let mut buf = [0_u16; ROW];
        let st = self.sample_time.clone();
        let _ = self
            .adc
            .read_seq(self.dma.reborrow(), self.irq, self.row_adc.iter_mut().map(|ch| (ch, st.clone())), &mut buf)
            .await;
        buf
    }

    /// Scan the matrix and return the first key state change found, if any.
    #[optimize(speed)]
    async fn scan_for_next_change(&mut self) -> Option<KeyboardEvent> {
        for col in 0..COL {
            self.cols.select(col);
            let samples = self.read_row_samples().await;

            for (row, &raw) in samples.iter().enumerate() {
                let Some(st) = self.state.get_mut(row).and_then(|row_slice| row_slice.get_mut(col)) else {
                    continue;
                };
                let last_raw = st.last_raw;

                if likely(last_raw.abs_diff(raw) < self.noise_gate) {
                    continue;
                }

                let Some(&cal) = self.calib.get(row).and_then(|row_slice| row_slice.get(col)) else {
                    continue;
                };
                let prev_travel = st.travel_scaled;
                let was_pressed = st.pressed;
                let Some(new_travel) = Self::travel_scaled_from(&cal, raw) else {
                    // Raw reading is out of the valid ADC range. Do not update
                    // last_raw so the noise gate baseline stays at the last
                    // known-good value.
                    cold_path();
                    continue;
                };

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

    /// Convert a raw reading into a scaled travel value, returning `None` on
    /// out-of-range or offset failure so the caller decides the fallback.
    #[inline]
    #[optimize(speed)]
    fn travel_scaled_from(cal: &KeyCalib, raw: u16) -> Option<u8> {
        if unlikely(!(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw)) {
            return None;
        }
        let Some(x) = Self::apply_zero_offset(cal.zero, raw) else {
            cold_path();
            return None;
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

/// Compute a scale factor from zero and full-travel calibration values.
#[inline]
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
