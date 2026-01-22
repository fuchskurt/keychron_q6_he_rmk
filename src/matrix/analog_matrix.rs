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

const FULL_TRAVEL_UNIT: u16 = 40;
const TRAVEL_SCALE: u16 = 6;
const TRAVEL_SCALE_I64: i64 = i64::from(TRAVEL_SCALE);
const FULL_TRAVEL_SCALED: i64 = i64::from(FULL_TRAVEL_UNIT.saturating_mul(TRAVEL_SCALE));
const DEFAULT_FULL_RANGE: u16 = 900;
const VALID_RAW_MIN: u16 = 1200;
const VALID_RAW_MAX: u16 = 3500;
const REF_ZERO_TRAVEL: u16 = 3121;
const UNCALIBRATED_ZERO: u16 = 3000;
const NOISE_GATE: u16 = 5;
const CALIB_PASSES: usize = 8;

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
    scale_factor: i32,
}

impl KeyCalib {
    const fn new(zero: u16, full: u16) -> Self {
        let scale_factor = compute_scale_factor(zero, full);
        Self { zero, scale_factor }
    }

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

struct MatrixGrid<T, const ROW: usize, const COL: usize> {
    cells: [[T; COL]; ROW],
}

impl<T: Copy, const ROW: usize, const COL: usize> MatrixGrid<T, ROW, COL> {
    fn new(mut f: impl FnMut() -> T) -> Self { Self { cells: from_fn(|_| from_fn(|_| f())) } }

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
    ADC: Instance,
    ADC::Regs: BasicAdcRegs,
{
    adc: Adc<'d, ADC>,
    row_adc: [AnyAdcChannel<'d, ADC>; ROW],
    sample_time: SampleTime,

    cols: Hc164Cols<'d>,
    settle_after_col: Duration,
    act_threshold: u16,
    deact_threshold: u16,

    calib: MatrixGrid<KeyCalib, ROW, COL>,
    state: MatrixGrid<KeyState, ROW, COL>,
    calibrated: bool,
    pending: Deque<KeyboardEvent, 32>,
}

impl<'d, ADC, const ROW: usize, const COL: usize> AnalogHallMatrix<'d, ADC, ROW, COL>
where
    ADC: Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = SampleTime>,
{
    pub fn new(
        adc: Adc<'d, ADC>,
        row_adc: [AnyAdcChannel<'d, ADC>; ROW],
        sample_time: SampleTime,
        cols: Hc164Cols<'d>,
        cfg: HallCfg,
    ) -> Self {
        let settle_after_col = cfg.settle_after_col;
        let act_threshold = cfg.actuation_pt.saturating_mul(TRAVEL_SCALE);
        let deact_threshold = cfg.actuation_pt.saturating_sub(cfg.deact_offset).saturating_mul(TRAVEL_SCALE);
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

    #[inline]
    const fn apply_zero_offset(zero: u16, raw: u16) -> Option<u16> {
        if zero >= REF_ZERO_TRAVEL {
            raw.checked_sub(zero - REF_ZERO_TRAVEL)
        } else {
            raw.checked_add(REF_ZERO_TRAVEL - zero)
        }
    }

    #[inline]
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
        let prod = delta.saturating_mul(i64::from(cal.scale_factor)).saturating_mul(TRAVEL_SCALE_I64);
        let travel = prod.checked_shr(SCALE_SHIFT).unwrap_or(0);

        u16::try_from(travel.clamp(0, FULL_TRAVEL_SCALED)).unwrap_or_default()
    }

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
            let avg = u16::try_from((*acc_cell).saturating_div(u32::try_from(CALIB_PASSES).unwrap_or_default()))
                .unwrap_or_default();

            if let Some(cal_cell) = self.calib.get_mut(row, col) {
                *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
        self.calibrated = true;
    }

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

                let now_pressed =
                    if was_pressed { new_travel >= self.deact_threshold } else { new_travel >= self.act_threshold };

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
    ADC: Instance,
    ADC::Regs: BasicAdcRegs<SampleTime = SampleTime>,
{
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

    let num = i64::from(FULL_TRAVEL_UNIT).saturating_mul(256_i64).saturating_mul(i64::from(SCALE_Q_FACTOR));
    i32::try_from(num.saturating_div(full_travel)).unwrap_or_default()
}
