use crate::hc164_cols::Hc164Cols;
use embassy_stm32::adc::{Adc, AnyAdcChannel, BasicAdcRegs, SampleTime};
use embassy_time::{Duration, Timer};
use heapless::Deque;
use rmk::{
    event::{Event, KeyboardEvent},
    input_device::InputDevice,
};

pub const FULL_TRAVEL_UNIT: u16 = 40;
pub const TRAVEL_SCALE: u16 = 6;
pub const FULL_TRAVEL_SCALED: u16 = FULL_TRAVEL_UNIT * TRAVEL_SCALE;
pub const DEFAULT_FULL_RANGE: u16 = 900;
pub const VALID_RAW_MIN: u16 = 1200;
pub const VALID_RAW_MAX: u16 = 3500;
pub const REF_ZERO_TRAVEL: u16 = 3121;
pub const UNCALIBRATED_ZERO: u16 = 3000;
pub const NOISE_GATE: u16 = 5;
pub const CALIB_PASSES: usize = 8;
pub const POLY_AT_REFZ: f32 = travel_poly(REF_ZERO_TRAVEL as f32);

// Original calibration polynomial:
//
// f(x) = A + B·x + C·x² + D·x³
//
// f(x) = 426.88962 - 0.48358 · x + 2.04637e-4 · x² - 2.99368e-8 · x³
//
// Chebyshev approximation coefficients
pub const CHEB_C0: f32 = 27.825;
pub const CHEB_C1: f32 = -54.576;
pub const CHEB_C2: f32 = -4.2435;
pub const CHEB_C3: f32 = -11.383;
pub const CHEB_RANGE: f32 = VALID_RAW_MAX as f32 - VALID_RAW_MIN as f32;
pub const CHEB_MUL: f32 = 2.0 / CHEB_RANGE;
pub const CHEB_BIAS: f32 = -(VALID_RAW_MAX as f32 + VALID_RAW_MIN as f32) / CHEB_RANGE;

#[inline]
const fn x_to_t_pm1(x: f32) -> f32 { x * CHEB_MUL + CHEB_BIAS }

#[inline]
const fn travel_poly(x: f32) -> f32 {
    let x = x.clamp(VALID_RAW_MIN as f32, VALID_RAW_MAX as f32);
    let t = x_to_t_pm1(x);

    // Clenshaw (degree 3)
    let b3 = CHEB_C3;
    let b2 = CHEB_C2 + 2.0 * t * b3;
    let b1 = CHEB_C1 + 2.0 * t * b2 - b3;

    t * b1 - b2 + CHEB_C0
}

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

    const fn is_uncalibrated(&self) -> bool { self.zero == UNCALIBRATED_ZERO }

    pub const fn uncalibrated() -> Self {
        Self::new(UNCALIBRATED_ZERO, UNCALIBRATED_ZERO.saturating_sub(DEFAULT_FULL_RANGE))
    }
}

impl Default for KeyCalib {
    fn default() -> Self { Self::uncalibrated() }
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

struct Grid2D<T, const ROW: usize, const COL: usize> {
    cells: [[T; COL]; ROW],
}

impl<T: Copy, const ROW: usize, const COL: usize> Grid2D<T, ROW, COL> {
    fn new(mut f: impl FnMut() -> T) -> Self { Self { cells: core::array::from_fn(|_| core::array::from_fn(|_| f())) } }

    #[inline]
    fn get(&self, row: usize, col: usize) -> Option<&T> { self.cells.get(row).and_then(|r| r.get(col)) }

    #[inline]
    fn get_mut(&mut self, row: usize, col: usize) -> Option<&mut T> {
        self.cells.get_mut(row).and_then(|r| r.get_mut(col))
    }

    #[inline]
    fn iter_cells(&self) -> impl Iterator<Item = ((usize, usize), &T)> {
        self.cells.iter().enumerate().flat_map(|(r, row)| row.iter().enumerate().map(move |(c, cell)| ((r, c), cell)))
    }
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

    calib: Grid2D<KeyCalib, ROW, COL>,
    state: Grid2D<KeyState, ROW, COL>,

    pending: Deque<KeyboardEvent, 32>,
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
        Self {
            adc,
            row_adc,
            sample_time,
            cols,
            cfg,
            calib: Grid2D::new(KeyCalib::default),
            state: Grid2D::new(KeyState::default),
            pending: Deque::new(),
        }
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

    fn needs_zero_calibration(&self) -> bool { self.calib.get(0, 0).is_some_and(|c| c.is_uncalibrated()) }

    async fn calibrate_zero_travel(&mut self) {
        let mut acc: Grid2D<u32, ROW, COL> = Grid2D::new(|| 0u32);

        let cfg = self.cfg;
        let sample_time = self.sample_time;
        let adc = &mut self.adc;
        let row_adc = &mut self.row_adc;

        for _ in 0..CALIB_PASSES {
            for col in 0..COL {
                self.cols.select(col).await;
                Timer::after(cfg.settle_after_col).await;

                for (row, ch) in row_adc.iter_mut().enumerate() {
                    let v = adc.blocking_read(ch, sample_time);

                    if let Some(cell) = acc.get_mut(row, col) {
                        *cell += v as u32;
                    }
                }

                self.cols.advance().await;
            }
        }

        for ((row, col), acc_cell) in acc.iter_cells() {
            let avg = (*acc_cell / CALIB_PASSES as u32) as u16;

            if let Some(cal_cell) = self.calib.get_mut(row, col) {
                *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
    }

    async fn scan_and_enqueue_changes(&mut self) {
        let cfg = self.cfg;
        let sample_time = self.sample_time;

        let act = cfg.actuation_pt * TRAVEL_SCALE;
        let deact = cfg.actuation_pt.saturating_sub(cfg.deact_offset) * TRAVEL_SCALE;

        let adc = &mut self.adc;
        let row_adc = &mut self.row_adc;

        for col in 0..COL {
            self.cols.select(col).await;
            Timer::after(cfg.settle_after_col).await;

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

                let now_pressed = if was_pressed { new_travel >= deact } else { new_travel >= act };

                st.last_raw = raw;
                st.travel_scaled = new_travel;

                if now_pressed != st.pressed {
                    st.pressed = now_pressed;

                    let ev = KeyboardEvent::key(row as u8, col as u8, now_pressed);
                    if self.pending.push_back(ev).is_err() {
                        let _ = self.pending.pop_front();
                        let _ = self.pending.push_back(ev);
                    }
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
            if let Some(ev) = self.pending.pop_front() {
                return Event::Key(ev);
            }

            self.scan_and_enqueue_changes().await;

            if self.pending.is_empty() {
                Timer::after(Duration::from_micros(10)).await;
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
