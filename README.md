# Negicon v3

This repository contains the firmware for my custom DJ controller, the Negicon.
This is the third major redesign, using RP2040 microcontrollers.

## Major features
### Fully Digital
From the beginning, the Negicon is designed to use fully digital inputs. This allows all controls to be reset from software without going out of sync with the physical state of the controller. Simply put, you can't forget to put any knobs back to 0.

### Modular
Version 3 is not just a single controller, but a modular platform that allows for flexible controller designs of any form factor without firmware changes. All inputs are dynamically scanned and identified, so hardware-specific configuration needed.

### Composable
The firmware supports both direct USB HID connections to the host PC, as well as connecting to upstream RP2040 chips via SPI. This allows for chaining multiple RP2040s for larger controllers with more inputs than a single chip could handle.

### Hotswappable
Input modules are continuously scanned and initialized, allowing for full hotplug support. Rip out a buttion in the middle of a set. Change the layout on the fly. The controller will recognize it and set it up automatically.