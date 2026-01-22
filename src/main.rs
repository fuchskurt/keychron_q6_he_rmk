#![no_main]
#![no_std]
#![feature(const_trait_impl)]
#![feature(const_cmp)]
#![feature(const_option_ops)]
#![feature(const_index)]
#![feature(const_convert)]
#![feature(const_result_trait_fn)]
#![warn(clippy::all, clippy::pedantic, clippy::restriction, clippy::nursery)]
#![deny(warnings)]
#![expect(
    clippy::implicit_return,
    clippy::blanket_clippy_restriction_lints,
    clippy::separated_literal_suffix,
    clippy::single_call_fn,
    clippy::self_named_module_files,
    reason = "Cleaner code"
)]
#![expect(clippy::future_not_send, reason = "Embassy is designed that way")]

/// Backlight driver integration.
mod backlight;
/// Flash storage wrapper types.
mod flash;
/// Default keymap definitions.
mod keymap;
/// Layer toggle input handling.
mod layer_toggle;
/// Matrix scanning components.
mod matrix;
/// SNLED27351 driver support.
mod snled27351_spi;
/// Vial configuration constants.
mod vial;

use crate::{
    backlight::{
        init::backlight_runner,
        lock_indicator::{BACKLIGHT_CH, BacklightCmd, SnledIndicatorController},
    },
    flash::Flash16K,
    keymap::{COL, ROW},
    layer_toggle::{LayerToggle, MatrixPos},
    matrix::{
        analog_matrix::{AnalogHallMatrix, HallCfg},
        encoder_switch,
        hc164_cols::Hc164Cols,
    },
};
use core::panic::PanicInfo;
use cortex_m::asm::wfi;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel as _, AnyAdcChannel, SampleTime},
    bind_interrupts,
    exti::{self, ExtiInput},
    flash::Flash,
    gpio::{Input, Level, Output, Pull, Speed},
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
    spi::{self},
    time::Hertz,
    usb::{self, Driver},
};
use embassy_time::Duration;
use rmk::{
    channel::EVENT_CHANNEL,
    config::{
        BehaviorConfig,
        DeviceConfig,
        PositionalConfig,
        RmkConfig,
        StorageConfig,
        VialConfig,
    },
    controller::EventController as _,
    futures::future::join5,
    initialize_encoder_keymap_and_storage,
    input_device::{Runnable as _, rotary_encoder::RotaryEncoder},
    keyboard::Keyboard,
    run_devices,
    run_rmk,
    storage::async_flash_wrapper,
};
use static_cell::StaticCell;
use vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    EXTI3 => exti::InterruptHandler<typelevel::EXTI3>;
    EXTI15_10 => exti::InterruptHandler<typelevel::EXTI15_10>;
});

#[embassy_executor::main]
/// Entry point for the firmware.
async fn main(spawner: Spawner) {
    // Explicitly drop spawner.
    let _: Spawner = spawner;
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
    let peripheral = embassy_stm32::init(config);

    // Usb config
    static EP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let mut usb_config = usb::Config::default();
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(
        peripheral.USB_OTG_FS,
        Irqs,
        peripheral.PA12,
        peripheral.PA11,
        &mut EP_OUT_BUFFER.init([0; 1024])[..],
        usb_config,
    );

    // Use internal flash to emulate eeprom
    let storage_config = StorageConfig {
        // Start at sector 1, 0x4000 from the start of the FLASH region
        start_addr: 0x4000,
        num_sectors: 2,
        ..Default::default()
    };
    let flash = async_flash_wrapper(Flash16K(Flash::new_blocking(peripheral.FLASH)));

    // Keyboard config
    let rmk_config = RmkConfig {
        vial_config: VialConfig::new(VIAL_KEYBOARD_ID, VIAL_KEYBOARD_DEF, &[(5, 0), (5, 1)]),
        device_config: DeviceConfig {
            manufacturer: "Keychron",
            product_name: "Q6 HE",
            vid: 0x3434,
            pid: 0x0B60,
            serial_number: "vial:f64c2b3c:000001",
        },
        ..Default::default()
    };
    let _analog_matrix_power = Output::new(peripheral.PC13, Level::High, Speed::Low);
    let _analog_matrix_wakeup = Input::new(peripheral.PC5, Pull::Up);

    // HC164 columns
    let ds = Output::new(peripheral.PB3, Level::Low, Speed::VeryHigh);
    let cp = Output::new(peripheral.PB5, Level::Low, Speed::VeryHigh);
    let mr = Output::new(peripheral.PD2, Level::Low, Speed::VeryHigh);
    let cols = Hc164Cols::new(ds, cp, mr);

    // ADC matrix (rows are ADC pins)
    let adc: Adc<'_, ADC1> = Adc::new(peripheral.ADC1);
    let row_channels: [AnyAdcChannel<'_, ADC1>; ROW] = [
        peripheral.PC0.degrade_adc(),
        peripheral.PC1.degrade_adc(),
        peripheral.PC2.degrade_adc(),
        peripheral.PC3.degrade_adc(),
        peripheral.PA0.degrade_adc(),
        peripheral.PA1.degrade_adc(),
    ];

    let mut matrix = AnalogHallMatrix::<_, ROW, COL>::new(
        adc,
        row_channels,
        SampleTime::CYCLES15,
        cols,
        HallCfg { settle_after_col: Duration::from_micros(10), ..HallCfg::default() },
    );

    // Rotary encoder
    let pin_a = ExtiInput::new(peripheral.PB14, peripheral.EXTI14, Pull::None, Irqs);
    let pin_b = ExtiInput::new(peripheral.PB15, peripheral.EXTI15, Pull::None, Irqs);
    let mut encoder = RotaryEncoder::with_resolution(pin_a, pin_b, 4, true, 0);
    let enc_sw_pin = ExtiInput::new(peripheral.PA3, peripheral.EXTI3, Pull::Up, Irqs);
    let mut enc_switch = encoder_switch::EncoderSwitch::new(enc_sw_pin, 0, 13);

    // Layer Toggle Switch
    let layer_toggle_pin = ExtiInput::new(peripheral.PB12, peripheral.EXTI12, Pull::Up, Irqs);
    let mut layer_toggle = LayerToggle::new_with_default_debounce(
        layer_toggle_pin,
        MatrixPos { row: 5, col: 7 }, // HIGH taps this
        MatrixPos { row: 5, col: 8 }, // LOW taps this
    );

    // Initialize the storage and keymap
    let mut default_keymap = keymap::get_default_keymap();
    let mut default_encoder = keymap::get_default_encoder_map();
    let mut behavior_config = BehaviorConfig::default();
    let mut per_key_config = PositionalConfig::default();
    let (keymap, mut storage) = initialize_encoder_keymap_and_storage(
        &mut default_keymap,
        &mut default_encoder,
        flash,
        &storage_config,
        &mut behavior_config,
        &mut per_key_config,
    )
    .await;

    // Initialize the  keyboard
    let mut keyboard = Keyboard::new(&keymap);

    // LED backlight (SNLED27351)
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode = spi::MODE_0;
    let spi_backlight = spi::Spi::new(
        peripheral.SPI1,     // SPI1
        peripheral.PA5,      // SCK
        peripheral.PA7,      // MOSI
        peripheral.PA6,      // MISO
        peripheral.DMA2_CH3, // TX DMA
        peripheral.DMA2_CH0, // RX DMA
        spi_config,
    );
    let cs0 = Output::new(peripheral.PB8, Level::High, Speed::VeryHigh);
    let cs1 = Output::new(peripheral.PB9, Level::High, Speed::VeryHigh);
    let sdb = Output::new(peripheral.PB7, Level::Low, Speed::VeryHigh);
    let mut snled_indicator = SnledIndicatorController::new();

    // Start
    join5(
        run_devices!((matrix, encoder, enc_switch, layer_toggle) => EVENT_CHANNEL),
        keyboard.run(),
        run_rmk(&keymap, driver, &mut storage, rmk_config),
        backlight_runner(spi_backlight, cs0, cs1, sdb),
        snled_indicator.event_loop(),
    )
    .await;
}

#[panic_handler]
/// Panic handler that triggers a backlight blink loop.
fn panic(_info: &PanicInfo) -> ! {
    // Blink Backlight LEDs
    if BACKLIGHT_CH.try_send(BacklightCmd::Panic).is_err() {
        // channel full or disconnected
    }
    // Stop everything else
    loop {
        wfi();
    }
}
