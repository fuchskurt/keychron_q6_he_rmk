use crate::hc164_cols::Hc164Cols;
use embassy_stm32::adc::{Adc, AnyAdcChannel, BasicAdcRegs, SampleTime};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

// Constants
pub const FULL_TRAVEL_UNIT: u16 = 40;
pub const TRAVEL_SCALE: u16 = 6;
pub const FULL_TRAVEL_SCALED: u16 = FULL_TRAVEL_UNIT * TRAVEL_SCALE;
pub const DEFAULT_FULL_RANGE: u16 = 900;
pub const VALID_RAW_MIN: u16 = 1200;
pub const VALID_RAW_MAX: u16 = 3500;
pub const REF_ZERO_TRAVEL: u16 = 3121;
pub const UNCALIBRATED_ZERO: u16 = 3000;
pub const CONST_A1: f32 = 426.88962;
pub const CONST_B1: f32 = -0.48358;
pub const CONST_C1: f32 = 2.04637e-4;
pub const CONST_D1: f32 = -2.99368e-8;
pub const NOISE_GATE: u16 = 5;
pub const CALIB_PASSES: usize = 8;
pub const POLY_AT_REFZ: f32 = travel_poly(REF_ZERO_TRAVEL as f32);

#[inline]
const fn travel_poly(x: f32) -> f32 { (((CONST_D1 * x) + CONST_C1) * x + CONST_B1) * x + CONST_A1 }

#[inline]
const fn travel_poly_delta_from_ref(x: f32) -> f32 { travel_poly(x) - POLY_AT_REFZ }

#[derive(Clone, Copy)]
pub struct HallCfg {
    pub settle_after_col: Duration,
    pub actuation_pt: u16,
    pub deact_offset: u16,
}

impl Default for HallCfg {
    fn default() -> Self { Self { settle_after_col: Duration::from_micros(40), actuation_pt: 20, deact_offset: 3 } }
}

#[derive(Clone, Copy)]
struct KeyCalib {
    zero: u16,
    scale_factor: f32,
}

impl KeyCalib {
    const fn new(zero: u16, full: u16) -> Self {
        let scale_factor = compute_scale_factor(zero, full);
        Self { zero, scale_factor }
    }
}

#[derive(Clone, Copy)]
struct KeyState {
    last_raw: u16,
    travel_scaled: u16,
    pressed: bool,
}

impl KeyState {
    pub const fn new() -> Self { Self { last_raw: 0, travel_scaled: 0, pressed: false } }
}

impl Default for KeyState {
    fn default() -> Self { Self::new() }
}

impl KeyCalib {
    const fn is_uncalibrated(&self) -> bool { self.zero == UNCALIBRATED_ZERO }

    pub const fn uncalibrated() -> Self {
        Self::new(UNCALIBRATED_ZERO, UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE))
    }
}
impl Default for KeyCalib {
    fn default() -> Self { Self::uncalibrated() }
}

pub struct AnalogHallMatrix<'d, ADC, const ROW: usize, const COL: usize>
where
    ADC: embassy_stm32::adc::Instance,
    ADC::Regs: BasicAdcRegs,
{
    adc: Adc<'d, ADC>,
    row_adc: [AnyAdcChannel<'d, ADC>; ROW],
    sample_time: SampleTime,
    cols: Hc164Cols<'d>,
    cfg: HallCfg,
    calib: [[KeyCalib; COL]; ROW],
    state: [[KeyState; COL]; ROW],
    pending: Vec<KeyboardEvent, 32>,
}

impl<'d, ADC, const ROW: usize, const COL: usize> AnalogHallMatrix<'d, ADC, ROW, COL>
where
    ADC: embassy_stm32::adc::Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = SampleTime>,
{
    pub fn new(
        adc: Adc<'d, ADC>,
        row_adc: [AnyAdcChannel<'d, ADC>; ROW],
        sample_time: SampleTime,
        cols: Hc164Cols<'d>,
        cfg: HallCfg,
    ) -> Self {
        let calib = [[KeyCalib::default(); COL]; ROW];
        let state = [[KeyState::default(); COL]; ROW];

        Self { adc, row_adc, sample_time, cols, cfg, calib, state, pending: Vec::new() }
    }

    #[inline]
    fn travel_scaled_from(cal: &KeyCalib, prev: u16, raw: u16) -> u16 {
        if !(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw) {
            return prev;
        }

        let offset = (cal.zero as i32) - REF_ZERO_TRAVEL as i32;
        let x = (raw as i32 - offset) as f32;

        if x > REF_ZERO_TRAVEL as f32 {
            return 0;
        }

        let delta = travel_poly_delta_from_ref(x);
        let t = delta * cal.scale_factor * (TRAVEL_SCALE as f32);
        t.clamp(0.0, FULL_TRAVEL_SCALED as f32) as u16
    }

    fn needs_zero_calibration(&self) -> bool {
        self.calib.first().and_then(|row| row.first()).is_some_and(KeyCalib::is_uncalibrated)
    }

    #[inline]
    fn read_row_blocking(&mut self, row: usize) -> u16 {
        self.adc.blocking_read(&mut self.row_adc[row], self.sample_time)
    }

    async fn calibrate_zero_travel(&mut self) {
        let mut acc = [[0u32; COL]; ROW];

        for _ in 0..CALIB_PASSES {
            for col in 0..COL {
                self.cols.select(col).await;
                Timer::after(self.cfg.settle_after_col).await;

                for (row, row_acc) in acc.iter_mut().enumerate() {
                    if let Some(cell) = row_acc.get_mut(col) {
                        *cell += self.read_row_blocking(row) as u32;
                    }
                }
                self.cols.advance().await;
            }
        }

        for (acc_row, calib_row) in acc.iter().zip(&mut self.calib) {
            for (acc_cell, calib_cell) in acc_row.iter().zip(calib_row.iter_mut()) {
                let avg = (*acc_cell / CALIB_PASSES as u32) as u16;
                *calib_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
    }

    async fn scan_and_enqueue_changes(&mut self) {
        let act = self.cfg.actuation_pt * TRAVEL_SCALE;
        let deact = self.cfg.actuation_pt.saturating_sub(self.cfg.deact_offset) * TRAVEL_SCALE;

        for col in 0..COL {
            self.cols.select(col).await;
            Timer::after(self.cfg.settle_after_col).await;

            for row in 0..ROW {
                let raw = self.read_row_blocking(row);
                let (last_raw, prev_travel, was_pressed, cal) = {
                    let st = &self.state[row][col];
                    let cal = &self.calib[row][col];
                    (st.last_raw, st.travel_scaled, st.pressed, *cal)
                };

                if last_raw != 0 && last_raw.abs_diff(raw) < NOISE_GATE {
                    continue;
                }
                let new_travel = Self::travel_scaled_from(&cal, prev_travel, raw);

                let st = &mut self.state[row][col];
                if new_travel == prev_travel {
                    st.last_raw = raw;
                    continue;
                }

                let now_pressed = if was_pressed { new_travel >= deact } else { new_travel >= act };

                st.last_raw = raw;
                st.travel_scaled = new_travel;

                if now_pressed != st.pressed {
                    st.pressed = now_pressed;
                    let _ = self.pending.push(KeyboardEvent::key(row as u8, col as u8, now_pressed));
                }
            }

            self.cols.advance().await;
        }
    }
}

impl<'d, ADC, const ROW: usize, const COL: usize> InputDevice for AnalogHallMatrix<'d, ADC, ROW, COL>
where
    ADC: embassy_stm32::adc::Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = SampleTime>,
{
    async fn read_event(&mut self) -> Event {
        if self.needs_zero_calibration() {
            self.calibrate_zero_travel().await;
        }

        loop {
            if let Some(ev) = self.pending.pop() {
                return Event::Key(ev);
            }

            self.scan_and_enqueue_changes().await;

            if self.pending.is_empty() {
                Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

#[inline]
const fn compute_scale_factor(zero: u16, full: u16) -> f32 {
    let offset = zero as i32 - REF_ZERO_TRAVEL as i32;
    let x_full = (full as i32 - offset) as f32;
    let full_travel = travel_poly(x_full) - POLY_AT_REFZ;

    if full_travel.abs() < 1e-6 { 1.0 } else { (FULL_TRAVEL_UNIT as f32) / full_travel }
}
