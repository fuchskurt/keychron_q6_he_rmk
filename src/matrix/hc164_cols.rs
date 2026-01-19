use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

pub struct Hc164Cols<'d> {
    ds: Output<'d>,
    cp: Output<'d>,
    mr: Output<'d>,
    bit_delay: Duration,
}

impl<'d> Hc164Cols<'d> {
    pub fn new(ds: Output<'d>, cp: Output<'d>, mr: Output<'d>) -> Self {
        Self { ds, cp, mr, bit_delay: Duration::from_micros(1) }
    }

    async fn pulse_cp(&mut self) {
        self.cp.set_high();
        Timer::after(self.bit_delay).await;
        self.cp.set_low();
        Timer::after(self.bit_delay).await;
    }

    async fn reset(&mut self) {
        self.mr.set_low();
        Timer::after(Duration::from_micros(2)).await;
        self.mr.set_high();
        Timer::after(Duration::from_micros(2)).await;
    }

    pub async fn select(&mut self, col: usize) {
        if col == 0 {
            self.reset().await;
            self.ds.set_high();
            Timer::after(self.bit_delay).await;
            self.pulse_cp().await;
            self.ds.set_low();
        }
    }

    pub async fn advance(&mut self) {
        self.ds.set_low();
        Timer::after(self.bit_delay).await;
        self.pulse_cp().await;
    }
}
