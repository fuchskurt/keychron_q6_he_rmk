# Keychron Q1 Pro

![Keychron Q1 Pro](https://cdn.shopify.com/s/files/1/0059/0630/1017/t/5/assets/keychronq1proqmkviacustommechanicalkeyboard--edited-1669962623486.jpg)

A customizable 81 keys TKL keyboard.

* Keyboard Maintainer: [Keychron](https://github.com/fuchskurt)
* Hardware Supported: Keychron Q1 Pro
* Hardware Availability: [Keychron Q1 Pro Wireless, Customizeable, QMK/VIA Mechanical Keyboard](https://www.keychron.com/products/keychron-q1-pro-qmk-custom-mechanical-keyboard-iso-layout-collection)

Make example for this keyboard (after setting up your build environment):
```
    cargo make objcopy
```

Flashing example for this keyboard:

```
    dfu-util -l # Note the serial Number of the MCU.
    dfu-util -a 0 -s 0x08000000:mass-erase:force:leave -D rmk.bin -S <SerialNumber>
```
