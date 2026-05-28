# Keychron Q6 HE - Custom RMK Firmware

[![Docs](https://img.shields.io/badge/docs-latest-blue?style=flat-square)](https://fuchskurt.github.io)
[![Release](https://img.shields.io/github/v/release/fuchskurt/keychron_q6_he_rmk?style=flat-square)](https://github.com/fuchskurt/keychron_q6_he_rmk/releases)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue?style=flat-square)](#license)

![Keychron Q6 HE](https://cdn.shopify.com/s/files/1/0059/0630/1017/files/Keychron-Q6-HE-Wireless-QMK-Custom-Magnetic-Switch-Keyboard-White.jpg)

Open-source replacement firmware in `RUST` based on [RMK](https://github.com/HaoboGu/rmk) for
the [Keychron Q6 HE](https://www.keychron.com/products/keychron-q6-he-qmk-wireless-custom-keyboard).
Compatible with [Vial](https://get.vial.today) for live keymap editing without reflashing.

Supports **ANSI**, **ISO**, and **JIS** layouts, select the right build for your board variant.

---

## Features

- **Analog actuation point**: Keys actuate at 1.0 mm by default and Rapid Trigger sensitivity starts at 0.3 mm, both
  adjustable in 0.1 mm increments, with no fixed reset point.
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

## Layout variants

| Layout | Feature flag  | Cargo profile                      |
| ------ | ------------- | ---------------------------------- |
| ANSI   | `ansi_layout` | `--features ansi_layout` (default) |
| ISO    | `iso_layout`  | `--features iso_layout`            |
| JIS    | `jis_layout`  | `--features jis_layout`            |

Exactly one layout feature must be enabled at a time. Each layout ships its own `vial.json`, LED mapping, and sensor
presence map.

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

### Build and flash (ANSI)

1. Enter DFU mode: hold the reset button on the underside of the keyboard, then plug it in (or press reset while
   plugged in).

2. Flash:

   ```sh
   cargo make flash
   ```

   The keyboard reboots automatically when flashing completes.

> `cargo make flash` defaults to the ANSI layout. To flash a different layout, pass the feature flag directly:
>
> ```sh
> cargo dfu --vid 0x483 --pid 0xdf11 --release --features iso_layout
> cargo dfu --vid 0x483 --pid 0xdf11 --release --features jis_layout
> ```

### Pre-built binaries

Download the `.bin` matching your layout from
[Releases](https://github.com/fuchskurt/keychron_q6_he_rmk/releases) and flash with any STM32 DFU tool:

```sh
dfu-util -a 0 -s 0x08000000:mass-erase:force:leave \
  -D keychron-q6-he-rmk-<layout>-<version>.bin \
  -S <serial number>
```

Replace `<layout>` with `ansi`, `iso`, or `jis`.

---

## First-boot calibration

The first time the firmware runs, or whenever saved calibration is missing or corrupt, the backlight guides you through
a two-step setup. Follow the colors:

| Backlight                | What to do                                                                                                                                                                            |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Solid amber**          | Leave all keys fully released                                                                                                                                                         |
| **Red -> blue gradient** | Press every key all the way to the bottom and hold it for 1 second. Each key lights up **green** when accepted. The background shifts from red toward blue as more keys are accepted. |
| **Green blink × 3**      | All keys accepted, you may release them now                                                                                                                                           |
| **Solid green for 2 s**  | Calibration saved, keyboard is ready                                                                                                                                                  |

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

## Scan loop performance

Benchmarked on STM32F401RC at 84 MHz, 21 columns × 6 rows, measured with the DWT cycle counter
(`cargo make bench`, `firmware-debug` profile: `lto = "fat"`, `codegen-units = 1` applied to all crates). Cycle counts cover the synchronous per-column processing
window: noise gate, auto-calibration, travel computation, and rapid-trigger logic. ADC DMA
(~9.8 µs/col) and `yield_now` (~1 µs/col) are outside the measurement window but included in
the end-to-end figures.

### Configurations tested

| Configuration                        | Cycles/col | Notes                            |
| ------------------------------------ | :--------: | -------------------------------- |
| LUT in flash, `lto = false`          |  299-301   | `firmware-debug` baseline        |
| Polynomial approximation (2nd order) |  310-312   | slower than LUT - not used       |
| LUT in flash, `lto = "fat"`          |  291-293   | ✓ production configuration       |
| LUT in RAM (`.data` section)         |  290-292   | no measurable benefit - reverted |
| LUT + `target-cpu=cortex-m4` flag    |  312-313   | scheduling regression - dropped  |

### End-to-end figures (production configuration)

| Metric          | Value     |
| --------------- | --------- |
| Time per column | ~14.3 µs  |
| Time per pass   | ~299.8 µs |
| Scan rate       | ~3,336 Hz |
| Avg key latency | ~150 µs   |

### Findings

- **Fat LTO** saves ~8 cycles/col (~2 µs/pass) over a non-LTO build through cross-crate
  inlining of the hot path. This same production configuration is also used for flashing.

- **Polynomial approximation is slower** than the LUT by ~19 cycles/col. The STM32F401's
  ART prefetch keeps LUT accesses cheap; two 32-bit multiplies cost more than the table lookup
  at this access pattern.

- **Sparse LUT**: ships a 37-entry u16 LUT (74 B) with linear interpolation between samples.
  A flat 2301-entry LUT (4602 B) with a one-load lookup was A/B benchmarked against it on
  this MCU and produced no measurable speedup, so the interpolated table is the only
  implementation in tree.

- **ADC DMA dominates** at ~9.8 µs/col (68% of per-column budget). All processing
  optimisations combined move average keypress latency by ~1 µs. The ADC sample time is
  already at the hardware minimum; the scan rate ceiling is the ADC, not the firmware logic.

- **USB HID poll rate** (1,000 Hz, 1 ms) is the practical latency floor for the host.
  The 3,336 Hz scan rate means the keyboard scans more than 3× per USB frame.

---

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE) at your option.
