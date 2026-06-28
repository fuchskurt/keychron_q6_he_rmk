#![no_main]
#![no_std]
#![feature(
    const_convert,
    const_trait_impl,
    const_cmp,
    const_index,
    const_option_ops,
    const_result_trait_fn,
    impl_trait_in_assoc_type,
    optimize_attribute,
    likely_unlikely,
    default_field_values
)]
extern crate cortex_m as _;

/// Backlight driver integration.
mod backlight;
/// EEPROM I²C driver.
mod eeprom;
/// Default layout definitions.
mod layout;
/// Matrix scanning components.
mod matrix;
/// USB host connection state helpers shared across tasks.
mod usb_state;

use crate::{
    backlight::{init::BacklightRunner, processor::LedIndicator},
    eeprom::Ft24c64,
    layout::{COL, ROW},
    matrix::{
        analog_matrix::{AdcPart, AnalogHallMatrix, HallCfg, RowChannels},
        encoder_switch,
        hc164_cols::Hc164Cols,
        layer_toggle::{LayerToggle, MatrixPos},
    },
};
use core::mem::ManuallyDrop;
use embassy_executor::{Spawner, main};
use embassy_stm32::{
    Config,
    Peri,
    adc::{Adc, AdcChannel as _, BorrowedAdcChannel, SampleTime},
    bind_interrupts,
    crc::Crc,
    dma,
    exti::{self, ExtiInput},
    gpio::{Flex, Level, Output, Pin, Pull, Speed},
    i2c::{self, I2c},
    init,
    interrupt::typelevel,
    pac,
    peripherals::{self, ADC1},
    rcc::{
        AHBPrescaler,
        APBPrescaler,
        Hse,
        HseMode,
        Pll,
        PllMul,
        PllPDiv,
        PllPreDiv,
        PllQDiv,
        PllSource,
        Sysclk,
        mux::Clk48sel,
    },
    spi::{self},
    time::Hertz,
    usb::{self, Driver},
};
use encoder_switch::EncoderSwitch;
use layout::{get_default_encoder_map, get_default_keymap};
use pac::{ADC1_COMMON, SYSCFG, adccommon::vals::Adcpre};
use rmk::{
    KeymapData,
    config::{BehaviorConfig, DeviceConfig, PositionalConfig, RmkConfig},
    initialize_keymap,
    input_device::rotary_encoder::RotaryEncoder,
    keyboard::Keyboard,
    run_all,
    usb::UsbTransport,
};
use static_cell::ConstStaticCell;

bind_interrupts!(struct Irqs {
    DMA2_STREAM0 => dma::InterruptHandler<peripherals::DMA2_CH0>;
    DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
    DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
    DMA1_STREAM4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_STREAM2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    EXTI3 => exti::InterruptHandler<typelevel::EXTI3>;
    EXTI9_5 => exti::InterruptHandler<typelevel::EXTI9_5>;
    EXTI15_10 => exti::InterruptHandler<typelevel::EXTI15_10>;
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
});

/// Owns the six analog row pins of the Q6 HE matrix.
///
/// The current embassy ADC API exposes channels only as transient
/// [`BorrowedAdcChannel`] borrows of an owned pin (the owned, type-erased
/// `AnyAdcChannel` was removed upstream), so the matrix scanner cannot store
/// pre-erased channels. This holder keeps the concrete pins alive and
/// re-borrows them on demand through [`RowChannels`].
struct Q6RowPins<'peripherals> {
    /// Row 4 analog input (`ADC1_IN0`).
    pa0: Peri<'peripherals, peripherals::PA0>,
    /// Row 5 analog input (`ADC1_IN1`).
    pa1: Peri<'peripherals, peripherals::PA1>,
    /// Row 0 analog input (`ADC1_IN10`).
    pc0: Peri<'peripherals, peripherals::PC0>,
    /// Row 1 analog input (`ADC1_IN11`).
    pc1: Peri<'peripherals, peripherals::PC1>,
    /// Row 2 analog input (`ADC1_IN12`).
    pc2: Peri<'peripherals, peripherals::PC2>,
    /// Row 3 analog input (`ADC1_IN13`).
    pc3: Peri<'peripherals, peripherals::PC3>,
}

impl RowChannels<ADC1, ROW> for Q6RowPins<'_> {
    fn reborrow_channels(&mut self) -> [BorrowedAdcChannel<'_, ADC1>; ROW] {
        [
            self.pc0.reborrow_adc(),
            self.pc1.reborrow_adc(),
            self.pc2.reborrow_adc(),
            self.pc3.reborrow_adc(),
            self.pa0.reborrow_adc(),
            self.pa1.reborrow_adc(),
        ]
    }

    fn set_active(&mut self) {
        release_row_pull(&mut self.pc0);
        release_row_pull(&mut self.pc1);
        release_row_pull(&mut self.pc2);
        release_row_pull(&mut self.pc3);
        release_row_pull(&mut self.pa0);
        release_row_pull(&mut self.pa1);
    }

    fn set_low_power(&mut self) {
        park_row_low(&mut self.pc0);
        park_row_low(&mut self.pc1);
        park_row_low(&mut self.pc2);
        park_row_low(&mut self.pc3);
        park_row_low(&mut self.pa0);
        park_row_low(&mut self.pa1);
    }
}

/// Drive one row pin to input-pull-down and leave it there.
///
/// A [`Flex`] resets its pin to disconnected (floating analog) on drop, so it
/// is wrapped in [`ManuallyDrop`] to persist the pull-down through suspend.
/// Nothing is leaked but the register state; the pin is reclaimed by
/// `reborrow_adc` (analog) or [`release_row_pull`] (no-pull) on resume.
fn park_row_low<P>(pin: &mut Peri<'_, P>)
where
    P: Pin,
{
    let mut flex = ManuallyDrop::new(Flex::new(pin.reborrow()));
    flex.set_as_input(Pull::Down);
}

/// Return one row pin to a no-pull input, clearing a suspend pull-down before
/// the ADC re-asserts analog mode (which only sets the mode register and would
/// otherwise leave the pull-down in place, biasing the reading).
fn release_row_pull<P>(pin: &mut Peri<'_, P>)
where
    P: Pin,
{
    let mut flex = ManuallyDrop::new(Flex::new(pin.reborrow()));
    flex.set_as_input(Pull::None);
}

/// Build the STM32F401 clock and bus configuration used at boot.
///
/// HSE 16 MHz → PLL (÷8 ×168 → 336 MHz VCO) → SYSCLK 84 MHz via /P=4 and
/// USB clock 48 MHz via /Q=7. AHB runs at full SYSCLK; APB1 at /2 (42 MHz)
/// for the slower peripherals; APB2 at full speed for ADC and SPI.
fn stm32_config() -> Config {
    let mut config = Config::default();
    config.rcc.hse = Some(Hse { freq: Hertz(16_000_000), mode: HseMode::Oscillator });
    config.rcc.hsi = false;
    config.rcc.pll_src = PllSource::Hse;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::Div8,     // 16/8 = 2 MHz
        mul:    PllMul::Mul168,      // 2*168 = 336 MHz (VCO)
        divp:   Some(PllPDiv::Div4), // 336/4 = 84 MHz (SYSCLK)
        divq:   Some(PllQDiv::Div7), // 336/7 = 48 MHz
        divr:   None,
    });
    config.rcc.ahb_pre = AHBPrescaler::Div1; // 84 MHz
    config.rcc.apb1_pre = APBPrescaler::Div2; // 42 MHz
    config.rcc.apb2_pre = APBPrescaler::Div1; // 84 MHz
    config.rcc.sys = Sysclk::Pll1P;
    config.rcc.mux.clk48sel = Clk48sel::Pll1Q;
    config
}

/// Enable the FLASH instruction cache, data cache, and prefetch buffer to
/// keep wait-stated flash reads off the hot path.
fn enable_flash_acceleration() {
    pac::FLASH.acr().modify(|w| {
        w.set_icen(true);
        w.set_dcen(true);
        w.set_prften(true);
    });
}

/// Entry point for the firmware.
#[main]
async fn main(spawner: Spawner) {
    // Tasks run inline via run_all!; the executor's spawner is unused by our
    // code but is part of the signature the `#[main]` macro requires.
    _ = spawner;
    let peripheral = init(stm32_config());
    enable_flash_acceleration();

    // Usb config
    static EP_OUT_BUFFER: ConstStaticCell<[u8; 1024]> = ConstStaticCell::new([0; 1024]);
    let usb_config = {
        let mut usb_config = usb::Config::default();
        usb_config.vbus_detection = false;
        usb_config
    };
    let driver = Driver::new_fs(
        peripheral.USB_OTG_FS,
        Irqs,
        peripheral.PA12,
        peripheral.PA11,
        &mut EP_OUT_BUFFER.take()[..],
        usb_config,
    );

    // Keyboard config
    let rmk_config = RmkConfig {
        device_config: DeviceConfig {
            manufacturer:  "Keychron",
            product_name:  "Q6 HE",
            vid:           0x3434,
            pid:           0x0B60,
            serial_number: "rmk:q6he:000006",
        },
    };
    // PC13 powers the hall-sensor rail; driven high here and handed to the
    // matrix scanner, which keeps it high during calibration and active
    // scanning and cuts it between passes while the USB bus is suspended.
    let matrix_power = Output::new(peripheral.PC13, Level::High, Speed::Low);
    // PC5: hardware any-key wake line, pulled down so it idles low; the matrix
    // parks on its rising edge during USB suspend.
    let wake = ExtiInput::new(peripheral.PC5, peripheral.EXTI5, Pull::Up, Irqs);

    // HC164 columns
    let ds = Flex::new(peripheral.PB3);
    let cp = Flex::new(peripheral.PB5);
    let mr = Flex::new(peripheral.PD2);
    let cols = Hc164Cols::new(ds, cp, mr);

    // ADC matrix (rows are ADC pins)
    let adc: Adc<'_, ADC1> = Adc::new(peripheral.ADC1);
    // Set ADC prescaler to /2 (42MHz, overclocked from 36MHz spec)
    ADC1_COMMON.ccr().modify(|w| w.set_adcpre(Adcpre::Div2));
    // Apply STM32 AN4073 Option 2 workaround to reduce ADC noise coupling from USB
    // activity on STM32F401.
    SYSCFG.pmc().modify(|w| w.set_adc1dc2(true));
    // Own the row pins so they can be re-borrowed into fresh ADC channels for
    // each sequence read. The new embassy ADC API no longer provides an owned,
    // type-erased channel; channels are transient borrows of their pins.
    let row_pins = Q6RowPins {
        pc0: peripheral.PC0,
        pc1: peripheral.PC1,
        pc2: peripheral.PC2,
        pc3: peripheral.PC3,
        pa0: peripheral.PA0,
        pa1: peripheral.PA1,
    };

    let i2c_config = {
        let mut cfg = i2c::Config::default();
        cfg.frequency = Hertz(850_000);
        cfg
    };
    let i2c3 = I2c::new(
        peripheral.I2C3,
        peripheral.PA8,      // SCL (PA8 = I2C3_SCL)
        peripheral.PC9,      // SDA (PC9 = I2C3_SDA)
        peripheral.DMA1_CH4, // TX DMA
        peripheral.DMA1_CH2, // RX DMA
        Irqs,
        i2c_config,
    );
    let eeprom_wp = Flex::new(peripheral.PB10);
    let eeprom = Ft24c64::new(i2c3, eeprom_wp);

    // Hardware CRC peripheral for EEPROM calibration block checksums.
    let crc = Crc::new(peripheral.CRC);

    let adc_part = AdcPart::new(adc, row_pins, peripheral.DMA2_CH0, Irqs, SampleTime::Cycles56);
    let mut matrix = AnalogHallMatrix::<_, _, _, _, _, ROW, COL>::new(
        adc_part,
        cols,
        HallCfg::default(),
        eeprom,
        crc,
        matrix_power,
        wake,
    );
    // Rotary encoder
    let pin_a = ExtiInput::new(peripheral.PB14, peripheral.EXTI14, Pull::None, Irqs);
    let pin_b = ExtiInput::new(peripheral.PB15, peripheral.EXTI15, Pull::None, Irqs);
    let mut encoder = RotaryEncoder::with_resolution(pin_a, pin_b, 4, true, 0);
    let enc_sw_pin = ExtiInput::new(peripheral.PA3, peripheral.EXTI3, Pull::Up, Irqs);
    let mut enc_switch = EncoderSwitch::new(enc_sw_pin, 0, 13);

    // Layer Toggle Switch
    let layer_toggle_pin = ExtiInput::new(peripheral.PB12, peripheral.EXTI12, Pull::Up, Irqs);
    let mut layer_toggle = LayerToggle::new_with_default_debounce(
        layer_toggle_pin,
        MatrixPos { row: 5, col: 7 }, // HIGH taps this
        MatrixPos { row: 5, col: 8 }, // LOW taps this
    );

    // Initialize the storage and keymap
    let mut keymap_data = KeymapData::new_with_encoder(get_default_keymap(), get_default_encoder_map());
    let mut behavior_config = BehaviorConfig::default();
    let key_config = PositionalConfig::default();
    let keymap = initialize_keymap(&mut keymap_data, &mut behavior_config, &key_config).await;

    // Initialize the keyboard
    let mut keyboard = Keyboard::new(&keymap);
    let mut usb_transport = UsbTransport::new(driver, rmk_config.device_config);

    // LED backlight
    let spi_config = {
        let mut spi_config = spi::Config::default();
        spi_config.frequency = Hertz(3_500_000);
        spi_config.mode = spi::MODE_0;
        spi_config
    };
    let spi_backlight = spi::Spi::new(
        peripheral.SPI1,     // SPI1
        peripheral.PA5,      // SCK
        peripheral.PA7,      // MOSI
        peripheral.PA6,      // MISO
        peripheral.DMA2_CH3, // TX DMA
        peripheral.DMA2_CH2, // RX DMA
        Irqs,
        spi_config,
    );
    let cs0 = Output::new(peripheral.PB8, Level::High, Speed::VeryHigh);
    let cs1 = Output::new(peripheral.PB9, Level::High, Speed::VeryHigh);
    let sdb = Output::new(peripheral.PB7, Level::Low, Speed::VeryHigh);
    let mut led_indicator = LedIndicator::new();
    let mut backlight = BacklightRunner::new(spi_backlight, cs0, cs1, sdb);
    let mut usb_state_task = usb_state::UsbStateTask::new();

    // Start.
    //
    // All tasks intentionally share this single thread-mode executor: RMK
    // selects ThreadModeRawMutex for its internal event/report channels on
    // cortex-m targets, so every publisher and consumer - including the
    // matrix scanner's publish_event_async - must stay in thread mode.
    // Moving the scanner to a higher-priority InterruptExecutor would lock
    // those mutexes from handler mode, which is unsound; that optimisation
    // is blocked until RMK uses CriticalSectionRawMutex on this target.
    run_all!(
        keyboard,
        usb_transport,
        matrix,
        encoder,
        enc_switch,
        layer_toggle,
        led_indicator,
        usb_state_task,
        backlight
    )
    .await;
}
