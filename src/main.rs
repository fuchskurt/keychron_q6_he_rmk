#![no_main]
#![no_std]
#![feature(const_trait_impl)]
#![feature(const_cmp)]
#![feature(const_option_ops)]
#![feature(const_index)]
#![feature(const_convert)]
#![feature(const_result_trait_fn)]
#![feature(const_ops)]
#![feature(generic_const_exprs)]
#![warn(clippy::all, clippy::pedantic, clippy::restriction, clippy::nursery)]
#![deny(warnings)]
#![expect(
    clippy::implicit_return,
    clippy::blanket_clippy_restriction_lints,
    clippy::separated_literal_suffix,
    clippy::single_call_fn,
    clippy::self_named_module_files,
    clippy::future_not_send,
    incomplete_features,
    reason = "Implementation specific ignored lints"
)]
/// Backlight driver integration.
mod backlight;
/// External I2C EEPROM calibration storage.
mod eeprom_storage;
/// Flash storage wrapper types.
mod flash_wrapper;
/// Default keymap definitions.
mod keymap;
/// Matrix scanning components.
mod matrix;
/// SNLED27351 driver support.
mod snled27351_spi;
/// Vial configuration constants.
mod vial;

use crate::{
    backlight::{init::backlight_runner, lock_indicator::SnledIndicatorProcessor},
    eeprom_storage::EepromStorage,
    flash_wrapper::Flash16K,
    keymap::{COL, ROW},
    matrix::{
        analog_matrix::{AnalogHallMatrix, HallCfg},
        encoder_switch,
        hc164_cols::Hc164Cols,
        layer_toggle::{LayerToggle, MatrixPos},
    },
    vial::VIAL_SERIAL,
};
use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::SCB};
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel as _, AnyAdcChannel, SampleTime},
    bind_interrupts,
    exti::{self, ExtiInput},
    flash,
    flash::Flash,
    gpio::{Input, Level, Output, Pull, Speed},
    i2c::{self, I2c},
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
use encoder_switch::EncoderSwitch;
use rmk::{
    KeymapData,
    config::{BehaviorConfig, DeviceConfig, PositionalConfig, RmkConfig, StorageConfig, VialConfig},
    futures::future::join4,
    initialize_keymap_and_storage,
    input_device::{Runnable as _, rotary_encoder::RotaryEncoder},
    keyboard::Keyboard,
    run_all,
    run_rmk,
    storage::async_flash_wrapper,
};
use static_cell::StaticCell;
use vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    EXTI3 => exti::InterruptHandler<typelevel::EXTI3>;
    EXTI15_10 => exti::InterruptHandler<typelevel::EXTI15_10>;
    FLASH => flash::InterruptHandler;
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
    let flash = async_flash_wrapper(Flash16K(Flash::new(peripheral.FLASH, Irqs)));

    // Keyboard config
    let rmk_config = RmkConfig {
        vial_config: VialConfig::new(VIAL_KEYBOARD_ID, VIAL_KEYBOARD_DEF, &[(5, 0), (5, 1)]),
        device_config: DeviceConfig {
            manufacturer: "Keychron",
            product_name: "Q6 HE",
            vid: 0x3434,
            pid: 0x0B60,
            serial_number: VIAL_SERIAL,
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
        SampleTime::CYCLES3,
        cols,
        HallCfg { settle_after_col_cycles: 84, ..HallCfg::default() },
    );

    // External EEPROM (I2C3): load calibration if present, otherwise calibrate
    // and persist so subsequent boots skip the ADC sweep.
    //
    // PA8 = I2C3_SCL (AF4), PC9 = I2C3_SDA (AF4), PB10 = WP (HIGH = protected)
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz(400_000);
    let i2c3 = I2c::new_blocking(peripheral.I2C3, peripheral.PA8, peripheral.PC9, i2c_config);
    let eeprom_wp = Output::new(peripheral.PB10, Level::High, Speed::Low);
    let mut eeprom = EepromStorage::new(i2c3, eeprom_wp);

    if let Some(zeros) = eeprom.load_calibration::<ROW, COL>() {
        // Stored calibration found: skip the 8-pass ADC sweep on boot.
        matrix.load_from_zeros(&zeros);
    } else {
        // No valid calibration in EEPROM: run the sweep and save the result.
        matrix.calibrate_now();
        if let Some(zeros) = matrix.get_zeros() {
            match eeprom.save_calibration::<ROW, COL>(&zeros) {
                Ok(()) | Err(_) => {}
            }
        }
    }

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
    let mut keymap_data = KeymapData::new_with_encoder(keymap::get_default_keymap(), keymap::get_default_encoder_map());
    let mut behavior_config = BehaviorConfig::default();
    let key_config = PositionalConfig::default();
    let (keymap, mut storage) =
        initialize_keymap_and_storage(&mut keymap_data, flash, &storage_config, &mut behavior_config, &key_config)
            .await;

    // Initialize the keyboard
    let mut keyboard = Keyboard::new(&keymap);

    // LED backlight (SNLED27351)
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(3_500_000);
    spi_config.mode = spi::MODE_0;
    let spi_backlight = spi::Spi::new(
        peripheral.SPI1,     // SPI1
        peripheral.PA5,      // SCK
        peripheral.PA7,      // MOSI
        peripheral.PA6,      // MISO
        peripheral.DMA2_CH3, // TX DMA
        peripheral.DMA2_CH2, // RX DMA
        spi_config,
    );
    let cs0 = Output::new(peripheral.PB8, Level::High, Speed::VeryHigh);
    let cs1 = Output::new(peripheral.PB9, Level::High, Speed::VeryHigh);
    let sdb = Output::new(peripheral.PB7, Level::Low, Speed::VeryHigh);
    let mut snled_indicator = SnledIndicatorProcessor::new();

    // Start
    join4(
        run_all!(matrix, encoder, enc_switch, layer_toggle, snled_indicator),
        keyboard.run(),
        run_rmk(&keymap, driver, &mut storage, rmk_config),
        backlight_runner(spi_backlight, cs0, cs1, sdb),
    )
    .await;
}

#[panic_handler]
/// Panic handler that triggers a restart.
fn panic(_info: &PanicInfo) -> ! {
    asm::delay(10_000);
    SCB::sys_reset();
}
