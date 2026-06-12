# Keychron Q6 HE - Custom RMK Firmware

[![Docs](https://img.shields.io/badge/docs-latest-blue?style=flat-square)](https://fuchskurt.github.io/keychron_q6_he_rmk/)
[![Release](https://img.shields.io/github/v/release/fuchskurt/keychron_q6_he_rmk?style=flat-square)](https://github.com/fuchskurt/keychron_q6_he_rmk/releases)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue?style=flat-square)](#license)

![Keychron Q6 HE](https://cdn.shopify.com/s/files/1/0059/0630/1017/files/Keychron-Q6-HE-Wireless-QMK-Custom-Magnetic-Switch-Keyboard-White.jpg)

Open-source replacement firmware for the
[Keychron Q6 HE](https://www.keychron.com/products/keychron-q6-he-qmk-wireless-custom-keyboard), written in Rust and
built on [RMK](https://github.com/HaoboGu/rmk). Builds are available for the ANSI, ISO, and JIS variants of the board.

## Features

- **Analog keys with Rapid Trigger**: Keys actuate at 1.0 mm of travel. A held key releases after rising just 0.3 mm
  and re-fires after 0.5 mm of downward travel, wherever in the stroke that happens, with no fixed reset point. All
  three distances are firmware settings in 0.1 mm steps.
- **Live keymap editing**: Remap any key, layer, or encoder action on the fly through
  [rynk](https://github.com/HaoboGu/rmk/tree/feat/rynk_protocol), RMK's experimental remapping protocol. No reflashing
  needed.
- **Mac and Windows layers**: Two base layers, switched with the physical toggle on the side of the keyboard.
- **White backlight with status colors**: The backlight glows white during normal use. Caps Lock turns red while
  active and Num Lock lights up while active. The per-key RGB hardware also guides you through calibration.
- **Rotary encoder**: Volume up and down out of the box, press the knob to mute. Remappable like any key.
- **Automatic calibration**: A guided one-time calibration on first boot, with the backlight walking you through it.
  After that the keyboard re-checks itself on every boot and quietly keeps its calibration fresh while you type, so
  sensor drift never becomes your problem.
- **Thermal protection**: The backlight dims itself if the LED driver chips run hot and returns to full brightness once
  they cool down.

## Layouts

| Layout | Feature flag  |
| ------ | ------------- |
| ANSI   | `ansi_layout` |
| ISO    | `iso_layout`  |
| JIS    | `jis_layout`  |

Exactly one layout must be chosen per build; there is no default. The `cargo make` flash tasks below pick the right
flag for you.

## Flashing

### Pre-built binaries

Download the `.bin` matching your layout from the
[Releases](https://github.com/fuchskurt/keychron_q6_he_rmk/releases) page, then:

1. Put the keyboard into DFU mode: pop off the space bar keycap and hold the reset button underneath it while
   plugging in the cable.
2. Find the keyboard's serial number with `dfu-util -l`.
3. Flash:

   ```sh
   dfu-util -a 0 -s 0x08000000:mass-erase:force:leave \
     -D keychron-q6-he-rmk-<layout>-<version>.bin \
     -S <serial number>
   ```

   Replace `<layout>` with `ansi`, `iso`, or `jis`, and `<version>` with the release version.

### Building from source

Install [rustup](https://rustup.rs), then set up the build tools once:

```sh
cargo install cargo-make
cargo make install
```

rustup installs the pinned nightly toolchain and its components automatically on the first build.

To build and flash, put the keyboard into DFU mode the same way as described above, then run the task for your layout:

```sh
cargo make flash-ansi   # ANSI
cargo make flash-iso    # ISO
cargo make flash-jis    # JIS
```

The keyboard reboots on its own once flashing finishes.

## First-boot calibration

The first time the firmware starts, or if the saved calibration ever goes missing or gets corrupted, the keyboard walks
you through a short setup. Just follow the backlight:

| Backlight                | What to do                                                                                                                                                  |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Solid amber**          | Leave all keys fully released                                                                                                                               |
| **Red to blue gradient** | Press every key all the way down and hold it for a second. Each accepted key turns green, and the background shifts from red toward blue as you make progress. |
| **Three green blinks**   | Every key has been recorded, you can let go now                                                                                                             |
| **Solid green for 2 s**  | Calibration saved, the keyboard is ready                                                                                                                    |

The press-every-key step ends once all keys are in, or after 3 minutes. Any key you did not get to still works with a
sensible default range and fine-tunes itself automatically as you type.

If the backlight goes back to amber after the green blinks, saving the calibration failed. Unplug the keyboard, plug it
back in, and run the calibration again.

On every later boot the keyboard briefly re-measures each key's resting position to account for temperature changes,
then starts up normally.

## Keymap editing

Keys, layers, and the encoder can be remapped live through
[rynk](https://github.com/HaoboGu/rmk/tree/feat/rynk_protocol), RMK's experimental remapping protocol. Connect the
keyboard to a rynk-compatible client, edit your layout, and the changes apply immediately. No reflash, no reboot.

## Scan rate

The firmware scans the whole key matrix at about 3,300 Hz, more than three full passes per USB poll (the host polls at
1,000 Hz). The average key latency added by scanning is roughly 150 microseconds.

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE) at your option.
