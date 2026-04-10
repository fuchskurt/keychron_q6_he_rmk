use crate::matrix::{
    hc164_cols::Hc164Cols,
    travel_helpers::{delta_from_ref, SCALE_Q_FACTOR, SCALE_SHIFT},
};
use core::{
    future::pending,
    hint::{likely, unlikely},
};
use embassy_stm32::{
    adc::{Adc, AnyAdcChannel, BasicAdcRegs, BasicInstance, ConfiguredSequence, Instance, RxDma},
    dma::InterruptHandler,
    interrupt::typelevel::Binding,
    pac::adc,
    Peri,
};
use embassy_time::{Duration, Timer};
use rmk::{
    event::KeyboardEvent,
    input_device::{InputDevice, Runnable},
};

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
/// Maximum acceptable distance from Ref Zero Travel on calibration.
const CALIB_ZERO_TOLERANCE: u16 = 500;
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
    /// Minimum travel floor in mm/10 units.
    pub actuation_pt: u8,
    /// Number of passes used during calibration.
    pub calib_passes: u32,
    /// Column settle time in microseconds after selecting a new column.
    pub col_settle_us: Duration,
    /// Raw ADC delta below which readings are treated as noise (default: 2).
    pub noise_gate: u16,
    /// Minimum directional travel change required to register a press or
    /// release.
    pub rt_sensitivity: u8,
    /// CPU cycles between HC164 pin transitions.
    pub shifter_delay_cycles: u32,
}

impl Default for HallCfg {
    /// Provide default hall configuration values.
    fn default() -> Self {
        Self {
            actuation_pt: 6,
            calib_passes: 512,
            col_settle_us: Duration::from_micros(35),
            noise_gate: 2,
            rt_sensitivity: 6,
            shifter_delay_cycles: 8,
        }
    }
}

#[derive(Clone, Copy)]
/// Calibration values for a single key.
struct KeyCalib {
    /// Fixed-point scale factor used to convert raw delta to travel units.
    scale_factor: i32,
    /// Whether this position has a valid hall effect sensor.
    used: bool,
    /// Raw ADC value corresponding to zero travel.
    zero: u16,
}

impl KeyCalib {
    /// Build calibration data from zero and full travel values.
    const fn new(zero: u16, full: u16) -> Self {
        Self {
            scale_factor: compute_scale_factor(zero, full),
            used: zero.abs_diff(REF_ZERO_TRAVEL) <= CALIB_ZERO_TOLERANCE,
            zero,
        }
    }

    /// Build an uncalibrated placeholder calibration.
    pub const fn uncalibrated() -> Self { Self { scale_factor: SCALE_Q_FACTOR, used: false, zero: UNCALIBRATED_ZERO } }
}

impl Default for KeyCalib {
    /// Provide a default uncalibrated calibration record.
    fn default() -> Self { Self::uncalibrated() }
}

#[derive(Clone, Copy)]
/// Dynamic state tracked per key.
struct KeyState {
    /// Tracks the local extremum for rapid-trigger calculations.
    extremum: u8,
    /// Last raw ADC reading for this key.
    last_raw: u16,
    /// Whether the key is currently considered pressed.
    pressed: bool,
    /// Current travel value scaled to internal units.
    travel_scaled: u8,
}

impl KeyState {
    /// Create a new default key state.
    pub const fn new() -> Self { Self { extremum: 0, last_raw: u16::MAX, travel_scaled: 0, pressed: false } }
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
    /// ADC channels corresponding to each row.
    pub row_adc: [AnyAdcChannel<'peripherals, ADC>; ROW],
    /// ADC sample time configuration.
    pub sample_time: AdcSampleTime<ADC>,
}

impl<'peripherals, ADC, D, const ROW: usize> AdcPart<'peripherals, ADC, D, ROW>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    AdcSampleTime<ADC>: Clone,
{
    /// Create a [`ConfiguredSequence`] over all row channels, programming the
    /// ADC sequence registers once. The returned reader can be triggered
    /// repeatedly without reprogramming the sequence registers.
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

/// Hall-effect analog matrix scanner.
///
/// Uses DMA-backed ADC reads for all sampling including the initial
/// zero-travel calibration, keeping the firmware fully non-blocking
/// from construction onwards.
pub struct AnalogHallMatrix<'peripherals, ADC, D, IRQ, const ROW: usize, const COL: usize>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    AdcSampleTime<ADC>: Clone,
{
    /// ADC peripherals and channels grouped for split-borrow compatibility.
    adc_part: AdcPart<'peripherals, ADC, D, ROW>,
    /// Calibration data for each key in the matrix.
    calib: [[KeyCalib; COL]; ROW],
    /// Sensing and scanning configuration.
    cfg: HallCfg,
    /// Column driver used to select the active column.
    cols: Hc164Cols<'peripherals>,
    /// DMA interrupt binding used for ADC sequence reads.
    irq: IRQ,
    /// Dynamic per-key state for the matrix.
    state: [[KeyState; COL]; ROW],
}

impl<'peripherals, ADC, D, IRQ, const ROW: usize, const COL: usize>
    AnalogHallMatrix<'peripherals, ADC, D, IRQ, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    AdcSampleTime<ADC>: Clone,
{
    /// Perform zero-travel calibration using a shared [`ConfiguredSequence`].
    ///
    /// Uses the same sequence as scanning to avoid ADC/DMA reconfiguration
    /// between calibration and the scan loop.
    async fn calibrate_zero_travel(
        cols: &mut Hc164Cols<'peripherals>,
        calib: &mut [[KeyCalib; COL]; ROW],
        seq: &mut ConfiguredSequence<'_, adc::Adc>,
        buf: &mut [u16; ROW],
        col_settle_us: Duration,
        calib_passes: u32,
    ) {
        let mut acc: [[u32; COL]; ROW] = [[0_u32; COL]; ROW];

        for _ in 0..calib_passes {
            for col in 0..COL {
                cols.select(col);
                Timer::after(col_settle_us).await;
                seq.read(buf).await;
                for (acc_row, &sample) in acc.iter_mut().zip(buf.iter()) {
                    if let Some(cell) = acc_row.get_mut(col) {
                        *cell = cell.saturating_add(u32::from(sample));
                    }
                }
            }
        }

        for (calib_row, acc_row) in calib.iter_mut().zip(acc.iter()) {
            for (cal_cell, &acc_cell) in calib_row.iter_mut().zip(acc_row.iter()) {
                let avg =
                    u16::try_from(acc_cell.checked_div(calib_passes).unwrap_or_default()).unwrap_or(UNCALIBRATED_ZERO);
                *cal_cell = KeyCalib::new(avg, avg.saturating_sub(DEFAULT_FULL_RANGE));
            }
        }
    }

    /// Create a new matrix scanner without calibration.
    ///
    /// Calibration runs inside [`Runnable::run`] using the same
    /// [`ConfiguredSequence`] as the scan loop, avoiding any ADC/DMA
    /// reconfiguration between calibration and scanning.
    pub const fn new(
        adc_part: AdcPart<'peripherals, ADC, D, ROW>,
        irq: IRQ,
        cols: Hc164Cols<'peripherals>,
        cfg: HallCfg,
    ) -> Self {
        Self {
            adc_part,
            irq,
            cols,
            cfg,
            calib: [[KeyCalib::uncalibrated(); COL]; ROW],
            state: [[KeyState::new(); COL]; ROW],
        }
    }

    /// Scan the matrix and return the first key state change found, if any.
    ///
    /// The ADC sequence is programmed once per scan pass and reused across
    /// all columns, avoiding per-column sequence register writes.
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
        let sensitivity = cfg.rt_sensitivity.max(1);

        for col in 0..COL {
            cols.select(col);
            Timer::after(cfg.col_settle_us).await;
            seq.read(buf).await;

            for (row, ((state_row, calib_row), &raw)) in state.iter_mut().zip(calib.iter()).zip(buf.iter()).enumerate()
            {
                let Some(st) = state_row.get_mut(col) else { continue };
                let last_raw = st.last_raw;

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
                    // Track peak (maximum travel) while the key is held.
                    if new_travel > st.extremum {
                        st.extremum = new_travel;
                    }
                    // Stay pressed while above the floor and within sensitivity
                    // units of the peak. Dropping below the floor forces an
                    // immediate release so the key never stays stuck.
                    new_travel >= act_threshold && new_travel > st.extremum.saturating_sub(sensitivity)
                } else {
                    // Track trough (minimum travel) while the key is up.
                    if new_travel < st.extremum {
                        st.extremum = new_travel;
                    }
                    // Reset zone: the trough must first drop back below the
                    // actuation floor before a new press is allowed. Without
                    // this, a finger hovering mid-travel after release can
                    // re-fire on a tiny 1mm wobble, causing random keypresses.
                    st.extremum < act_threshold
                        && new_travel >= act_threshold
                        && new_travel >= st.extremum.saturating_add(sensitivity)
                };

                if now_pressed != st.pressed {
                    // Reset extremum so the next direction track starts fresh.
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

    /// Convert a raw reading into a scaled travel value, returning `None` on
    /// out-of-range or offset failure so the caller decides the fallback.
    #[inline]
    #[optimize(speed)]
    fn travel_scaled_from(cal: &KeyCalib, raw: u16) -> Option<u8> {
        if !cal.used {
            return None;
        }

        if unlikely(!(VALID_RAW_MIN..=VALID_RAW_MAX).contains(&raw)) {
            return None;
        }
        let x = apply_ref_offset(cal.zero, raw).min(REF_ZERO_TRAVEL);

        let delta = i64::from(delta_from_ref(x));
        let prod = delta.saturating_mul(i64::from(cal.scale_factor)).saturating_mul(TRAVEL_SCALE_I64);
        let travel = prod.checked_shr(SCALE_SHIFT).unwrap_or(0);
        Some(u8::try_from(travel.clamp(0_i64, i64::from(FULL_TRAVEL_SCALED))).unwrap_or_default())
    }
}

impl<'peripherals, ADC, D, IRQ, const ROW: usize, const COL: usize> InputDevice
    for AnalogHallMatrix<'peripherals, ADC, D, IRQ, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    AdcSampleTime<ADC>: Clone,
{
    type Event = KeyboardEvent;

    async fn read_event(&mut self) -> Self::Event { pending().await }
}

impl<'peripherals, ADC, D, IRQ, const ROW: usize, const COL: usize> Runnable
    for AnalogHallMatrix<'peripherals, ADC, D, IRQ, ROW, COL>
where
    ADC: Instance<Regs = adc::Adc> + BasicInstance,
    D: RxDma<ADC>,
    IRQ: Binding<D::Interrupt, InterruptHandler<D>> + Copy + 'peripherals,
    AdcSampleTime<ADC>: Clone,
{
    async fn run(&mut self) -> ! {
        use ::rmk::event::publish_event_async;

        let mut buf = [0_u16; ROW];
        let mut seq = self.adc_part.configured_sequence(self.irq);

        Self::calibrate_zero_travel(
            &mut self.cols,
            &mut self.calib,
            &mut seq,
            &mut buf,
            self.cfg.col_settle_us,
            self.cfg.calib_passes,
        )
        .await;

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

/// Adjust a raw ADC value by the offset between `zero` and [`REF_ZERO_TRAVEL`].
#[inline]
const fn apply_ref_offset(zero: u16, raw: u16) -> u16 {
    if zero >= REF_ZERO_TRAVEL {
        raw.saturating_sub(zero.saturating_sub(REF_ZERO_TRAVEL))
    } else {
        raw.saturating_add(REF_ZERO_TRAVEL.saturating_sub(zero)).min(REF_ZERO_TRAVEL)
    }
}

/// Compute a scale factor from zero and full-travel calibration values.
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
