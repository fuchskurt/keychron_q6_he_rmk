#![no_main]
#![no_std]

mod analog_matrix;
mod encoder_switch;
mod hc164_cols;
mod keymap;
mod vial;

use crate::{
    analog_matrix::{AnalogHallMatrix, HallCfg},
    hc164_cols::Hc164Cols,
    keymap::{COL, ROW},
};
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel as _, AnyAdcChannel, SampleTime},
    bind_interrupts,
    exti::{self, ExtiInput},
    gpio::{Level, Output, Pull, Speed},
    interrupt::typelevel,
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
    time::Hertz,
    usb::{self, Driver},
};
use rmk::{
    channel::EVENT_CHANNEL,
    config::{BehaviorConfig, DeviceConfig, PositionalConfig, RmkConfig, VialConfig},
    futures::future::join3,
    initialize_encoder_keymap,
    input_device::{Runnable, rotary_encoder::RotaryEncoder},
    keyboard::Keyboard,
    run_devices,
    run_rmk,
};
use static_cell::StaticCell;
use vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    EXTI3 => exti::InterruptHandler<typelevel::EXTI3>;
    EXTI15_10 => exti::InterruptHandler<typelevel::EXTI15_10>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // RCC config
    let mut config = Config::default();

    config.rcc.hse = Some(Hse { freq: Hertz(16_000_000), mode: HseMode::Oscillator });
    config.rcc.hsi = false;
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV8,   // 16/8 = 2 MHz
        mul: PllMul::MUL168,       // 2*168 = 336 MHz (VCO)
        divp: Some(PllPDiv::DIV4), // 336/4 = 84 MHz (SYSCLK)
        divq: Some(PllQDiv::DIV7), // 336/7 = 48 MHz
        divr: None,
    });

    config.rcc.ahb_pre = AHBPrescaler::DIV1; // 84
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 36
    config.rcc.apb2_pre = APBPrescaler::DIV1; // 84
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.mux.clk48sel = Clk48sel::PLL1_Q;

    // Initialize peripherals
    let p = embassy_stm32::init(config);

    // Usb config
    static EP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, &mut EP_OUT_BUFFER.init([0; 1024])[..], usb_config);
    // Use internal flash to emulate eeprom

    // Keyboard config
    let rmk_config = RmkConfig {
        vial_config: VialConfig::new(VIAL_KEYBOARD_ID, VIAL_KEYBOARD_DEF, &[(5, 0), (3, 1)]),
        device_config: DeviceConfig {
            manufacturer: "Keychron",
            product_name: "Q6 HE",
            vid: 0x3434,
            pid: 0x0B60,
            serial_number: "vial:f64c2b3c:000001",
        },
    };
    let _analog_matrix_power = Output::new(p.PC13, Level::High, Speed::Low);
    let _analog_matrix_wakeup = embassy_stm32::gpio::Input::new(p.PC5, Pull::Up);

    // HC164 columns
    let ds = Output::new(p.PB3, Level::Low, Speed::VeryHigh);
    let cp = Output::new(p.PB5, Level::Low, Speed::VeryHigh);
    let mr = Output::new(p.PD2, Level::Low, Speed::VeryHigh);
    let cols = Hc164Cols::new(ds, cp, mr);

    // ADC matrix (rows are ADC pins)
    let adc: Adc<'_, ADC1> = Adc::new(p.ADC1);
    let row_channels: [AnyAdcChannel<'_, ADC1>; 6] = [
        p.PC0.degrade_adc(),
        p.PC1.degrade_adc(),
        p.PC2.degrade_adc(),
        p.PC3.degrade_adc(),
        p.PA0.degrade_adc(),
        p.PA1.degrade_adc(),
    ];

    let mut matrix =
        AnalogHallMatrix::<_, ROW, COL>::new(adc, row_channels, SampleTime::CYCLES28, cols, HallCfg::default());

    // Rotary enoder
    let pin_a = ExtiInput::new(p.PB14, p.EXTI14, Pull::None, Irqs);
    let pin_b = ExtiInput::new(p.PB15, p.EXTI15, Pull::None, Irqs);
    let mut encoder = RotaryEncoder::with_resolution(pin_a, pin_b, 4, true, 0);
    let enc_sw_pin = ExtiInput::new(p.PA3, p.EXTI3, Pull::Up, Irqs);
    let mut enc_switch = encoder_switch::EncoderSwitch::new(enc_sw_pin, 0, 13);

    // Initialize the storage and keymap
    let mut default_keymap = keymap::get_default_keymap();
    let mut default_encoder = keymap::get_default_encoder_map();
    let mut behavior_config = BehaviorConfig::default();
    let mut per_key_config = PositionalConfig::default();
    let keymap =
        initialize_encoder_keymap(&mut default_keymap, &mut default_encoder, &mut behavior_config, &mut per_key_config)
            .await;

    // Initialize the  keyboard
    let mut keyboard = Keyboard::new(&keymap);

    // Start
    join3(
        run_devices!(
            (matrix, encoder, enc_switch) => EVENT_CHANNEL,
        ),
        keyboard.run(),
        run_rmk(&keymap, driver, rmk_config),
    )
    .await;
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! { loop {} }
