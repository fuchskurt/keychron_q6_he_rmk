# Keychron Q6 HE

![Keychron Q6 HE](https://cdn.shopify.com/s/files/1/0059/0630/1017/files/Keychron-Q6-HE-Wireless-QMK-Custom-Magnetic-Switch-Keyboard-White.jpg)

A customizable 100% hall effect keyboard.

* Keyboard Maintainer: [Kurt Fuchs](https://github.com/fuchskurt)
* Hardware Supported: Keychron Q6 HE
* Hardware
  Availability: [Keychron Q6 HE QMK/VIA Wireless Custom Mechanical Keyboard](https://www.keychron.com/products/keychron-q6-he-qmk-wireless-custom-keyboard)

Make example for this keyboard (after setting up your build environment):

```
    cargo make objcopy
```

Flashing example for this keyboard:

```
    dfu-util -l # Note the serial Number of the MCU.
    dfu-util -a 0 -s 0x08000000:mass-erase:force:leave -D rmk.bin -S <SerialNumber>
```
