# Keychron Q6 HE ANSI - Custom RMK Firmware

[![Docs](https://img.shields.io/badge/docs-latest-blue?style=flat-square)](https://fuchskurt.github.io)
[![Release](https://img.shields.io/github/v/release/fuchskurt/keychron_q6_he_rmk?style=flat-square)](https://github.com/fuchskurt/keychron_q6_he_rmk/releases)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue?style=flat-square)](#license)

![Keychron Q6 HE](https://cdn.shopify.com/s/files/1/0059/0630/1017/files/Keychron-Q6-HE-Wireless-QMK-Custom-Magnetic-Switch-Keyboard-White.jpg)

Open-source replacement firmware in `RUST` based on [RMK](https://github.com/HaoboGu/rmk) for
the [Keychron Q6 HE ANSI](https://www.keychron.com/products/keychron-q6-he-qmk-wireless-custom-keyboard).
Compatible with [Vial](https://get.vial.today) for live keymap editing without reflashing.

---

## Features

- **Analog actuation point**:  Keys actuate at 0.8 mm by default. Rapid Trigger re-actuates a key as soon as it moves
  0.3 mm in the opposite direction, with no fixed reset point.
- **Two hardware layers**: Mac and Windows base layers, switchable via the physical toggle on the side of the keyboard.
- **Vial keymap editing**: Remap any key, encoder action, or layer live from [Vial](https://get.vial.today)
  or [vial.rocks](https://vial.rocks) without reflashing.
- **RGB backlight**: Full per-key RGB. Caps Lock lights red when active; Num Lock lights white when active.
- **Rotary encoder**: Volume up/down by default; remappable via Vial.
- **Automatic calibration**: Guided first-boot calibration with backlight feedback. On subsequent boots the keyboard
  re-measures key resting positions automatically and refines calibration silently in the background, so drift is
  corrected without any user action.
- **Thermal protection**: LED brightness is automatically reduced if the backlight driver chip gets too hot, and
  restored when it cools down.

---

## Flashing

### One-time setup

Install [Rust nightly](https://rustup.rs) and the required tools:

```sh
rustup toolchain install nightly
rustup target add thumbv7em-none-eabihf
rustup component add rust-src llvm-tools
cargo make install
```

### Build and flash

1. Enter DFU mode: hold the reset button on the underside of the keyboard, then plug it in (or press reset while plugged
   in).

2. Flash:

   ```sh
   cargo make flash
   ```

   The keyboard reboots automatically when flashing completes.

### Pre-built binary

Download a `.bin` from [Releases](https://github.com/fuchskurt/keychron_q6_he_rmk/releases) and flash it with any STM32
DFU tool (e.g. `dfu-util`).

---

## First-boot calibration

The first time the firmware runs, or whenever saved calibration is missing or corrupt, the backlight guides you through
a two-step setup. Follow the colors:

| Backlight               | What to do                                                                                                                                                                            |
|-------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Solid amber**         | Leave all keys fully released                                                                                                                                                         |
| **Red → blue gradient** | Press every key all the way to the bottom and hold it for 1 second. Each key lights up **green** when accepted. The background shifts from red toward blue as more keys are accepted. |
| **Green blink × 3**     | All keys accepted, you may release them now                                                                                                                                           |
| **Solid green for 2 s** | Calibration saved, keyboard is ready                                                                                                                                                  |

> **If the backlight returns to amber** after the green blink, saving to the EEPROM failed. Power-cycle the keyboard and
> repeat calibration on the next boot.

On every subsequent boot the keyboard briefly re-measures key resting positions to compensate for temperature changes,
then starts normally.

---

## Keymap editing

Connect the keyboard and open [Vial](https://get.vial.today) (desktop app or [vial.rocks](https://vial.rocks) in a
browser). Select the keyboard, then remap keys, layers, or encoder actions. Changes take effect immediately without
reflashing.

---

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE) at your option.
