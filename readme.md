# Keychron Q6 HE - Custom RMK Firmware

[![Docs](https://img.shields.io/badge/docs-latest-blue?style=flat-square)](https://fuchskurt.github.io)
[![Release](https://img.shields.io/github/v/release/fuchskurt/keychron_q6_he_rmk?style=flat-square)](https://github.com/fuchskurt/keychron_q6_he_rmk/releases)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue?style=flat-square)](#license)

![Keychron Q6 HE](https://cdn.shopify.com/s/files/1/0059/0630/1017/files/Keychron-Q6-HE-Wireless-QMK-Custom-Magnetic-Switch-Keyboard-White.jpg)

Open-source replacement firmware written in Rust, based on [RMK](https://github.com/HaoboGu/rmk),
for the [Keychron Q6 HE](https://www.keychron.com/products/keychron-q6-he-qmk-wireless-custom-keyboard).
Keys can be remapped live, without reflashing, over the experimental
[rynk protocol](https://github.com/HaoboGu/rmk/tree/feat/rynk_protocol) — RMK's native remapping
protocol.

Supports **ANSI**, **ISO**, and **JIS** layouts; pick the build that matches your board variant.

---

## Features

- **Analog actuation point**: Keys actuate at 1.0 mm by default and Rapid Trigger sensitivity starts at 0.3 mm, both
  adjustable in 0.1 mm increments, with no fixed reset point.
- **Two hardware layers**: Mac and Windows base layers, switchable via the physical toggle on the side of the keyboard.
- **Live keymap editing**: Remap any key, encoder action, or layer on the fly over the
  [rynk protocol](https://github.com/HaoboGu/rmk/tree/feat/rynk_protocol) (RMK), without reflashing.
- **RGB backlight**: Full per-key RGB. Caps Lock lights red when active; Num Lock lights white when active.
- **Rotary encoder**: Volume up/down by default; remappable over rynk.
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

Exactly one layout feature must be enabled at a time. Each layout ships its own LED mapping and sensor presence map.

---

## Flashing

### One-time setup

Install [rustup](https://rustup.rs) and the required host tools. The pinned
toolchain, target, and components (`rust-src`, `llvm-tools`, …) are declared in
`rust-toolchain.toml` and installed automatically by rustup on the first build.

```sh
rustup toolchain install nightly
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

> `cargo make flash` builds the ANSI layout. For the other layouts use:
>
> ```sh
> cargo make flash-iso
> cargo make flash-jis
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

Keys, layers, and encoder actions are remapped live over the experimental
[rynk protocol](https://github.com/HaoboGu/rmk/tree/feat/rynk_protocol), RMK's native remapping
protocol. Connect the keyboard to a rynk-compatible client and edit; changes take effect
immediately without reflashing.

---

## Scan loop performance

Benchmarked during development on STM32F401RC at 84 MHz, 21 columns × 6 rows, measured with the
DWT cycle counter (`lto = "fat"`, `codegen-units = 1`). Cycle counts cover the synchronous
per-column processing window: noise gate, auto-calibration, travel computation, and
rapid-trigger logic. ADC DMA
(~9.8 µs/col) and `yield_now` (~1 µs/col) are outside the measurement window but included in
the end-to-end figures.

### Configurations tested

| Configuration                        | Cycles/col | Notes                            |
| ------------------------------------ | :--------: | -------------------------------- |
| LUT in flash, `lto = false`          |  299-301   | non-LTO baseline                 |
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

- **Fat LTO** saves ~8 cycles/col (~2 µs/pass) over a non-LTO build through aggressive
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

- **Pipelined scan loop**: the scan loop is double-buffered — the first poll of the
  sequence read arms the DMA and starts the ADC, then the previous column's readings are
  processed on the CPU while the conversion proceeds in hardware. Based on the budget
  above (~9.8 µs DMA + ~3.5 µs processing per column) this hides the entire processing
  window, for a theoretical pass time of ~230 µs and a scan rate around 4.3 kHz (+30%
  over the sequential figures in the tables, which predate the pipelining). Note that the
  CPU now executes during ADC conversions, which can couple digital switching noise into
  the hall-sensor readings (the same coupling class as the AN4073 workaround applied for
  USB). The noise gate (±10 counts) is the first line of defense; if a unit shows
  resting-position jitter, compare against a build with the pipelining reverted.

- **Binary size flags**: the nightly `-Zfmt-debug=none` and `-Zlocation-detail=none`
  rustflags were measured to save exactly 0 bytes — `panic = "immediate-abort"` plus fat
  LTO already eliminates all formatting and panic-location machinery, and the stripped
  binary embeds no source paths (so `trim-paths` would be a no-op as well). None of them
  are enabled.

---

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE) at your option.
