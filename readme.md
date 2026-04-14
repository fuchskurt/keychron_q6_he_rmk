# Keychron Q6 HE ANSI - RMK Firmware

[![Docs](https://img.shields.io/badge/docs-latest-blue?style=flat-square)](https://fuchskurt.github.io/keychron_q6_he_rmk/rmk_q6_he_ansi/)
[![Release](https://img.shields.io/github/v/release/fuchskurt/keychron_q6_he_rmk?style=flat-square)](https://github.com/fuchskurt/keychron_q6_he_rmk/releases)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue?style=flat-square)](#license)

![Keychron Q6 HE](https://cdn.shopify.com/s/files/1/0059/0630/1017/files/Keychron-Q6-HE-Wireless-QMK-Custom-Magnetic-Switch-Keyboard-White.jpg)

Custom firmware for the Keychron Q6 HE ANSI, written in Rust using the
[RMK](https://github.com/HaoboGu/rmk) keyboard framework and
[Embassy](https://embassy.dev) async runtime. Replaces the stock QMK firmware
with a fully open, Rust-native implementation that retains Vial compatibility
for keymap editing.

|                         |                                                                                                                                             |
|-------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| **Keyboard Maintainer** | [Kurt Fuchs](https://github.com/fuchskurt)                                                                                                  |
| **Hardware**            | Keychron Q6 HE ANSI                                                                                                                         |
| **MCU**                 | STM32F401RC (Cortex-M4F, 84 MHz, 256K Flash, 64K RAM)                                                                                       |
| **Availability**        | [Keychron Q6 HE QMK/VIA Wireless Custom Mechanical Keyboard](https://www.keychron.com/products/keychron-q6-he-qmk-wireless-custom-keyboard) |

---

## Table of Contents

- [How it works](#how-it-works)
    - [Hall effect matrix scanning](#hall-effect-matrix-scanning)
  - [Sensor transfer polynomial and travel computation](#sensor-transfer-polynomial-and-travel-computation)
  - [First-boot calibration](#first-boot-calibration)
  - [Continuous auto-calibration](#continuous-auto-calibration)
  - [EEPROM persistence](#eeprom-persistence)
    - [STM32F401 errata workarounds](#stm32f401-errata-workarounds)
    - [Backlight](#backlight)
    - [Layers and input devices](#layers-and-input-devices)
- [Building](#building)
- [Flashing](#flashing)
- [Keymap editing](#keymap-editing)
- [Memory layout](#memory-layout)
- [Project structure](#project-structure)
- [License](#license)

---

## How it works

### Hall effect matrix scanning

Unlike traditional keyboards that use mechanical contacts, the Q6 HE uses
hall effect magnetic switches. Each key contains a magnet whose field strength
changes as the key is pressed, and the MCU measures this via ADC rather than
detecting a simple on/off contact closure. This enables analog actuation point
control and rapid trigger functionality.

The key matrix is scanned using an HC164 shift register as a walking-ones
column selector. For each column the firmware:

1. Selects the column via the HC164
2. Waits a configurable column settle time (`HallCfg::col_settle_us`, default 35 µs)
3. Reads all 6 row ADC channels in a single non-blocking DMA burst
4. Runs a column stability check: the column is re-sampled `DEBOUNCE_PASSES` (2)
   times immediately with no settle delay. If the pressed/released state of any
   key disagrees between the first and any subsequent read, the entire column
   pass is discarded and no key state is mutated.
5. For each reading that passes the noise gate, the auto-calibrator is updated
   before the travel and rapid-trigger logic runs, so any calibration refinement
   takes effect within the same scan pass.
6. Computes travel distance and checks for actuation threshold crossings using
   dynamic rapid trigger.

All ADC sampling is performed via `ConfiguredSequence`. The ADC channel sequence
registers are programmed once at construction and reused across all columns and
calibration passes, avoiding the per-call overhead of reprogramming them on every
read. Both the settle delay and the DMA transfer are fully async: the executor is
free to run other tasks (USB HID, RMK event processing) during both waits.

### Sensor transfer polynomial and travel computation

The raw ADC reading from a hall effect key is not linearly proportional to
physical travel. The relationship between ADC counts and millimetres of travel
is described by a cubic polynomial derived by least-squares fit to empirical
sensor data:

```
f(x) = A3·x³ + A2·x² + A1·x + A0
```

with the following coefficients:

| Coefficient | Value         |
|-------------|---------------|
| `A0`        | `426.88962`   |
| `A1`        | `-0.48358`    |
| `A2`        | `2.04637e-4`  |
| `A3`        | `-2.99368e-8` |

The polynomial is evaluated using Horner's method to minimize floating-point
rounding and avoid redundant multiplications:

```
f(x) = ((A3·x + A2)·x + A1)·x + A0
```

For each key, two polynomial values are precomputed at construction time using
the calibrated zero-travel ADC value and the calibrated full-travel ADC value:
`poly_zero` and `poly_delta_full`. At scan time the travel fraction is then a
simple linear interpolation:

```
travel = (f(raw) − poly_zero) / poly_delta_full × FULL_TRAVEL_SCALED
```

The result is clamped to `[0, FULL_TRAVEL_SCALED]` and converted to an integer
scaled travel unit (`FULL_TRAVEL_UNIT = 40` representing 4.0 mm, scaled by
`TRAVEL_SCALE = 6`, giving `FULL_TRAVEL_SCALED = 240`).

### First-boot calibration

On the very first boot (or whenever EEPROM data is absent or corrupt), the
firmware performs a guided two-phase calibration before entering the normal
scan loop. The backlight provides real-time visual feedback throughout.

#### Phase 1 — Zero-travel pass (backlight: amber)

All keys must be fully released. The firmware performs 512 complete matrix
scans (configurable via `HallCfg::calib_passes`) and averages the ADC readings
for every key. Each average is reduced by `ZERO_TRAVEL_DEAD_ZONE` (20 counts)
before being stored as the per-key resting ADC reference. This dead-zone
ensures that the resting position sits cleanly below the measured average,
so that thermal or mechanical drift that nudges the ADC upward cannot
immediately produce a spurious non-zero travel reading.

#### Phase 2 — Full-travel press pass (backlight: red → blue gradient, per-key green on acceptance)

The user must press every key to the physical bottom. The backlight background
shifts from red toward blue as more keys are accepted, giving a whole-keyboard
progress indicator. Individual keys light up green the moment they are accepted.

A key is accepted only after it satisfies two conditions simultaneously:

1. **Depth**: the raw ADC reading must drop at least `CALIB_PRESS_THRESHOLD`
   (450 counts) below its calibrated zero — roughly half-travel — ensuring the
   key is genuinely pressed and not just resting on noise.
2. **Duration**: the key must remain continuously below that threshold for
   `CALIB_HOLD_DURATION_MS` (1 000 ms). Releasing and re-pressing resets the
   hold timer, so bouncy or partial presses cannot accidentally satisfy the
   requirement.

Only positions listed in `SENSOR_POSITIONS` (which excludes virtual matrix
cells, wide-key gaps, and the rotary encoder button) count toward the
completion total, preventing unpopulated positions from blocking calibration
from completing.

#### Settle window

Once all keys are accepted the backlight blinks solid green three times to
signal that keys may be released. A `CALIB_SETTLE_AFTER_ALL_DONE_MS` (2 000 ms)
continuation window then keeps sampling every key to capture each key's true
physical bottom-out ADC rather than the value at the moment of acceptance.
During this window the running minimum per key is still updated, so the stored
full-travel value reflects the deepest reading seen across the entire phase.

#### Storage

After the settle window, each key's stored full-travel value is computed as:

```
full = min(min_raw + BOTTOM_JITTER, zero − MIN_USEFUL_FULL_RANGE)
```

`BOTTOM_JITTER` (80 counts) is added to the physical bottom-out reading before
storing. Without this offset the scale factor would be anchored to a point the
key can reach in normal use; any upward jitter then advances the rapid-trigger
extremum and a subsequent dip back to the true floor fires a spurious release
even though the key never moved. By raising the stored floor 80 counts above
the physical minimum, bottom-of-travel travel values stay comfortably below
`FULL_TRAVEL_SCALED` and RT jitter cannot accumulate a phantom peak.

The calibration block is serialized and written to the FT24C64 EEPROM (see
[EEPROM persistence](#eeprom-persistence)), then verified by an immediate
read-back and CRC check. A successful write is confirmed with a 2-second
solid-green hold before the keyboard enters normal operation. If write-back
verification fails the backlight returns to amber and calibration will repeat
on the next boot.

Keys that were not pressed during the full-travel window fall back to
`zero − DEFAULT_FULL_RANGE` (900 counts), so the keyboard remains functional
even if calibration was incomplete.

### Continuous auto-calibration

During normal operation the firmware silently refines each key's zero and
full-travel calibration values in the background, compensating for temperature
drift and mechanical changes without requiring a manual re-calibration.

For every ADC reading that passes the noise gate, the auto-calibrator runs a
three-state machine before the rapid-trigger logic:

```
Idle → Pressing → Releasing → Idle
```

- **Idle**: waits for the raw ADC to drop below `AUTO_CALIB_FULL_TRAVEL_THRESHOLD`,
  indicating the start of a full-travel press.
- **Pressing**: tracks the running ADC minimum (deepest travel). Transitions to
  **Releasing** only when the reading has risen at least `AUTO_CALIB_RELEASE_THRESHOLD`
  above that minimum, preventing a partial lift from being mistaken for a release.
- **Releasing**: tracks the running ADC maximum (zero travel). Once the reading
  settles within `AUTO_CALIB_ZERO_JITTER` (50 counts) of the tracked peak, the
  cycle is scored if the range `(zero_candidate − full_candidate)` meets
  `AUTO_CALIB_MIN_RANGE` (900 counts). Partial presses or re-presses mid-release
  reset the machine to **Idle**.

Each scored cycle increments a per-key confidence counter. Once
`AUTO_CALIB_CONFIDENCE_THRESHOLD` (3) confident cycles accumulate, the live
`KeyCalib` for that key is updated in place — no reboot required — using:

```
new_zero = zero_candidate − ZERO_TRAVEL_DEAD_ZONE
new_full = min(full_candidate + BOTTOM_JITTER, new_zero − MIN_USEFUL_FULL_RANGE)
```

The update is committed only if the new zero differs from the current one by
more than 10 ADC counts, avoiding unnecessary scale-factor recomputation for
insignificant drift.

### EEPROM persistence

Calibration data is stored on an **FT24C64** 64-Kbit (8 K × 8) I²C EEPROM
connected on I2C3. The driver in `src/eeprom.rs` provides page-aligned writes
(32 bytes per page per the FT24C64 datasheet), I²C acknowledgement polling, and
hardware write-protection management.

**Write-protect pin.** The WP pin is held at input pull-up (write-protect
asserted) whenever the driver is idle. Before each write sequence it is driven
output-low (write-protect deasserted) and restored to input pull-up afterward,
minimizing the window during which accidental writes are possible.

**Page write protocol.** The write method splits the data into page-aligned
chunks, issues a two-byte address + data `I2cTransaction` for each chunk, then
polls the device with zero-length writes (ACK polling) until the internal write
cycle completes before advancing to the next page. Up to 30 polling attempts
are made with a 200 µs yield between each.

**Serialisation format.** The calibration block written by `src/matrix/calib_store.rs`
has the following layout:

| Offset | Size                  | Field                                                                  |
|--------|-----------------------|------------------------------------------------------------------------|
| 0      | 4 bytes               | Magic number `0x5136_4845` ("Q6HE"), LE                                |
| 4      | 1 byte                | Format version (`1`)                                                   |
| 5      | `ROW × COL × 2` bytes | Full-travel ADC values, row-major, LE u16                              |
| end−2  | 2 bytes               | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over all preceding bytes |

Deserialisation validates the magic number, version byte, and CRC before
accepting any data. A version mismatch rejects data written by an incompatible
layout and triggers a fresh first-boot calibration. A CRC mismatch — caused by
power loss during a write, hardware faults, or a blank device — does the same.

**First boot detection.** On every boot, the firmware attempts to read and
deserialise the EEPROM block. If it succeeds, only the zero-travel pass is
repeated (to compensate for temperature drift since the last boot); full-travel
data is loaded from EEPROM. If deserialization fails for any reason the full
two-phase guided calibration is triggered.

### STM32F401 errata workarounds

The firmware applies STM32 AN4073 Option 2 (`ADC1DC2`) to reduce ADC noise
coupling from simultaneous USB activity. On the STM32F401, digital supply
noise from the USB peripheral can couple into ADC readings when both are
active. Setting bit 16 of `SYSCFG_PMC` enables an internal decoupling
capacitor that suppresses this, improving reading stability.

### Backlight

RGB backlight is driven by two SNLED27351 LED driver chips over SPI, each
controlling half the keyboard. The firmware applies gamma 2.2 correction and
brightness scaling to all LED writes. Lock indicator LEDs (Caps Lock, Num Lock)
are driven as distinct colors via an async channel from the key event
processor to the backlight task.

**Thermal throttle.** A ticker polls the TDF (thermal flag) register from both
driver chips every 5 seconds. If either chip reports a die temperature ≥ 70 °C,
global brightness is immediately reduced to 50 %. It is restored automatically
when both chips cool down. During a calibration pass `render_calib` is used
instead of `render_all` so a thermal event never clobbers the calibration
display.

**Calibration backlight states.**

| Phase                | Colour                                                      |
|----------------------|-------------------------------------------------------------|
| Zero-travel pass     | Solid amber                                                 |
| Full-travel pass     | Red → blue gradient (progress); per-key green on acceptance |
| All keys accepted    | Three short green flashes; keys may be released             |
| Calibration stored   | Solid green for 2 s, then normal white                      |
| EEPROM write failure | Returns to amber; calibration repeats on next boot          |

### Layers and input devices

The firmware supports two layers (Mac and Windows base layers) switchable via
a physical panel toggle switch. A rotary encoder with a push button provides
volume control. Both use async edge-interrupt input with debouncing.

---

## Building

Requires a nightly Rust toolchain and the `thumbv7em-none-eabihf` target.

```sh
rustup toolchain install nightly
rustup target add thumbv7em-none-eabihf
rustup component add rust-src
cargo install flip-link
```

Build the firmware:

```sh
cargo build --release
```

Produce a `.bin` artifact:

```sh
cargo make objcopy
```

## Flashing

Install all required host tools (only needed once):

```sh
cargo make install
```

Put the keyboard into DFU mode by holding the reset button, then flash:

```sh
cargo make flash
```

---

## Keymap editing

The firmware is Vial-compatible. Download [Vial](https://get.vial.today) or
use the web app at [vial.rocks](https://vial.rocks) to remap keys.

---

## Memory layout

The STM32F401RC has 256K of flash. The firmware uses the following layout:

| Region      | Start        | Size | Purpose                     |
|-------------|--------------|------|-----------------------------|
| Sector 0    | `0x08000000` | 16K  | Bootloader (DFU)            |
| Sectors 1–2 | `0x08004000` | 32K  | Keymap and settings storage |
| Sectors 3–7 | `0x0800C000` | 208K | Firmware                    |

The linker script places firmware text at `0x0800C000` to avoid overwriting
the storage sectors used for keymap persistence.

---

## Project structure

```
src/
├── main.rs                  # Entry point, peripheral init, task startup
├── keymap.rs                # Default keymap and encoder map
├── eeprom.rs                # FT24C64 I²C EEPROM driver (page writes, WP pin, ACK polling)
├── flash_wrapper_async.rs   # Asynchronous 16KB erase-size wrapper for STM32F4 flash sectors
├── vial.rs                  # Generated Vial keyboard definition
├── matrix/
│   ├── analog_matrix.rs     # Hall effect ADC matrix scanner (DMA-backed, calibration, auto-calib, RT)
│   ├── calib_store.rs       # EEPROM serialisation: magic + version + CRC-16 framing
│   ├── hc164_cols.rs        # HC164 shift register column driver
│   ├── sensor_mapping.rs    # Bitmask of populated sensor positions used during calibration
│   ├── encoder_switch.rs    # Rotary encoder push button
│   └── layer_toggle.rs      # Physical layer toggle switch
└── backlight/
    ├── init.rs              # Backlight task, soft-start ramp, thermal throttle, calibration display
    ├── gamma_correction.rs  # Gamma 2.2 LUT
    ├── led_processor.rs     # Caps/Num lock indicator processor and backlight command channel
    └── mapping.rs           # LED index to SNLED channel mapping; MATRIX_TO_LED table
```

---

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE) at your option.