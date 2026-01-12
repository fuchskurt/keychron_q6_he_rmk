// analog_matrix.rs
use crate::hc164_cols::Hc164Cols;
use embassy_stm32::adc::{Adc, AnyAdcChannel, BasicAdcRegs, SampleTime};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

pub const FULL_TRAVEL_UNIT: u16 = 40;
pub const TRAVEL_SCALE: u16 = 6;

// QMK-ish defaults / limits
pub const DEFAULT_FULL_RANGE: u16 = 900;
pub const VALID_RAW_MIN: u16 = 1200;
pub const VALID_RAW_MAX: u16 = 3500;

// Polynomial reference points (from Keychron/QMK logic)
pub const REF_ZERO_TRAVEL: i32 = 3121;
pub const REF_FULL_TRAVEL: i32 = 1940;

// Polynomial coeffs
pub const CONST_A1: f32 = 426.88962;
pub const CONST_B1: f32 = -0.48358;
pub const CONST_C1: f32 = 2.04637e-4;
pub const CONST_D1: f32 = -2.99368e-8;

#[inline]
fn travel_poly(x: f32) -> f32 { CONST_A1 + CONST_B1 * x + CONST_C1 * x * x + CONST_D1 * x * x * x }

#[derive(Clone, Copy)]
pub struct HallCfg {
    pub settle_after_col: Duration, // QMK ~40us
    pub actuation_pt: u16,          // 0..40 (QMK default 20)
    pub deact_offset: u16,          // QMK ~3
}

impl Default for HallCfg {
    fn default() -> Self { Self { settle_after_col: Duration::from_micros(40), actuation_pt: 20, deact_offset: 3 } }
}

#[derive(Clone, Copy)]
struct KeyCalib {
    zero: u16,
    full: u16,
    scale_factor: f32,
}

#[derive(Clone, Copy)]
struct KeyState {
    last_raw: u16,
    travel_scaled: u16,
    pressed: bool,
}

pub struct AnalogHallMatrix<'d, ADC, const ROW: usize, const COL: usize>
where
    ADC: embassy_stm32::adc::Instance,
    ADC::Regs: embassy_stm32::adc::BasicAdcRegs,
{
    adc: Adc<'d, ADC>,
    row_adc: [AnyAdcChannel<'d, ADC>; ROW],
    sample_time: SampleTime,

    cols: Hc164Cols<'d>,
    cfg: HallCfg,

    calib: [[KeyCalib; COL]; ROW],
    state: [[KeyState; COL]; ROW],

    // pending key events (RMK read_event returns 1 Event at a time)
    pending: Vec<KeyboardEvent, 32>,
}

impl<'d, ADC, const ROW: usize, const COL: usize> AnalogHallMatrix<'d, ADC, ROW, COL>
where
    ADC: embassy_stm32::adc::Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = embassy_stm32::adc::SampleTime>,
{
    pub fn new(
        adc: Adc<'d, ADC>,
        row_adc: [AnyAdcChannel<'d, ADC>; ROW],
        sample_time: SampleTime,
        cols: Hc164Cols<'d>,
        cfg: HallCfg,
    ) -> Self {
        let mut calib =
            [[KeyCalib { zero: 3000, full: 3000u16.saturating_sub(DEFAULT_FULL_RANGE), scale_factor: 1.0 }; COL]; ROW];

        for r in 0..ROW {
            for c in 0..COL {
                calib[r][c].scale_factor = compute_scale_factor(calib[r][c].zero, calib[r][c].full);
            }
        }

        let state = [[KeyState { last_raw: 0, travel_scaled: 0, pressed: false }; COL]; ROW];

        Self { adc, row_adc, sample_time, cols, cfg, calib, state, pending: Vec::new() }
    }

    #[inline]
    fn read_row_blocking(&mut self, r: usize) -> u16 {
        // blocking_read takes sample_time by value; SampleTime isn't Copy on your
        // target.
        let st = self.sample_time;
        self.adc.blocking_read(&mut self.row_adc[r], st)
    }

    fn convert_to_travel_scaled(&self, r: usize, c: usize, raw: u16) -> u16 {
        if raw < VALID_RAW_MIN || raw > VALID_RAW_MAX {
            return self.state[r][c].travel_scaled;
        }

        // Keychron/QMK hall boards: pressing lowers ADC value.
        // QMK effectively uses (zero_travel - value) as “more pressed”.
        //
        // Their polynomial mapping does:
        // offset = zero - REF_ZERO_TRAVEL
        // x = raw - offset
        // travel = (poly(x) - poly(REF_ZERO)) * scale_factor * TRAVEL_SCALE
        let z = self.calib[r][c].zero as i32;
        let offset = z - REF_ZERO_TRAVEL;

        let x = (raw as i32 - offset) as f32;
        let refz = REF_ZERO_TRAVEL as f32;

        // if raw is “above” rest (should not happen much), clamp to 0 travel
        if x > refz {
            return 0;
        }

        let mut t = (travel_poly(x) - travel_poly(refz)) * self.calib[r][c].scale_factor * (TRAVEL_SCALE as f32);
        if t < 0.0 {
            t = 0.0;
        }
        let max_t = ((FULL_TRAVEL_UNIT + 1) * TRAVEL_SCALE - 1) as f32;
        if t > max_t {
            t = max_t;
        }
        t as u16
    }

    async fn calibrate_zero_travel(&mut self) {
        const PASSES: usize = 8;
        let mut acc = [[0u32; COL]; ROW];

        for _ in 0..PASSES {
            for col in 0..COL {
                self.cols.select(col).await;
                Timer::after(self.cfg.settle_after_col).await;

                for row in 0..ROW {
                    acc[row][col] += self.read_row_blocking(row) as u32;
                }

                self.cols.advance().await;
            }
        }

        for row in 0..ROW {
            for col in 0..COL {
                let avg = (acc[row][col] / PASSES as u32) as u16;
                self.calib[row][col].zero = avg;
                self.calib[row][col].full = avg.saturating_sub(DEFAULT_FULL_RANGE);
                self.calib[row][col].scale_factor =
                    compute_scale_factor(self.calib[row][col].zero, self.calib[row][col].full);
            }
        }
    }

    async fn scan_and_enqueue_changes(&mut self) {
        let act = self.cfg.actuation_pt * TRAVEL_SCALE;
        let deact = (self.cfg.actuation_pt.saturating_sub(self.cfg.deact_offset)) * TRAVEL_SCALE;

        for col in 0..COL {
            self.cols.select(col).await;
            Timer::after(self.cfg.settle_after_col).await;

            for row in 0..ROW {
                let raw = self.read_row_blocking(row);

                // small noise gate (like QMK abs(last-raw) < 5)
                let last = self.state[row][col].last_raw;
                if last != 0 && last.abs_diff(raw) < 5 {
                    continue;
                }
                self.state[row][col].last_raw = raw;
                if row == 0 && col == 0 {
                    defmt::info!("raw[0,0]={}", raw);
                }
                let new_travel = self.convert_to_travel_scaled(row, col, raw);
                if new_travel == self.state[row][col].travel_scaled {
                    continue;
                }
                self.state[row][col].travel_scaled = new_travel;

                let was = self.state[row][col].pressed;
                let now = if was { new_travel >= deact } else { new_travel >= act };

                if now != was {
                    self.state[row][col].pressed = now;
                    let ev = KeyboardEvent::key(row as u8, col as u8, now);
                    let _ = self.pending.push(ev);
                }
            }

            self.cols.advance().await;
        }
    }
}

impl<'d, ADC, const ROW: usize, const COL: usize> InputDevice for AnalogHallMatrix<'d, ADC, ROW, COL>
where
    ADC: embassy_stm32::adc::Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = embassy_stm32::adc::SampleTime>,
{
    async fn read_event(&mut self) -> Event {
        // One-time calibration on first call
        if self.calib[0][0].zero == 3000 {
            self.calibrate_zero_travel().await;
        }

        loop {
            if let Some(ev) = self.pending.pop() {
                return Event::Key(ev);
            }

            self.scan_and_enqueue_changes().await;

            if let Some(ev) = self.pending.pop() {
                return Event::Key(ev);
            }

            // no change → yield a bit
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}

fn compute_scale_factor(zero: u16, full: u16) -> f32 {
    let offset = zero as i32 - REF_ZERO_TRAVEL;
    let x_full = (full as i32 - offset) as f32;

    let refz = REF_ZERO_TRAVEL as f32;
    let full_travel = travel_poly(x_full) - travel_poly(refz);

    if full_travel.abs() < 1e-6 { 1.0 } else { (FULL_TRAVEL_UNIT as f32) / full_travel }
}
