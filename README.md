# AgIsoAuxInputs

Open-source ISOBUS **auxiliary input devices** (ISO 11783-6 AUX-N): joysticks, keypads, switch boxes. They plug into any Virtual Terminal and are assigned to implement functions through the VT's standard aux-assignment screen.

Built on [AgIsoStack++](https://github.com/Open-Agriculture/AgIsoStack-plus-plus). Not affiliated with Open-Agriculture.

![Joystick](images/3D_joystick.png)

## Status

| Device | State |
|---|---|
| Joystick (4 analog axes + buttons) | Working end-to-end on real hardware (ESP32-S3 + PCAN-USB + AgIsoVirtualTerminal): address claim, VT handshake, object pool upload and live aux input assignment |
| Keypads, switch boxes, ... | Planned, same board |

## Hardware

Every device runs on the same board: the **ESP32S3R8N8 CAN Board V1.0.0**. It has its own power supply with reverse-polarity protection, a CAN transceiver, an ADC and I/O expanders on board. A device can be built on a breadboard with nothing more than buttons, switches and potentiometers. If a device needs more inputs, add more I2C chips.

| Function | Part | Connection |
|---|---|---|
| MCU | ESP32-S3 (ESP32S3R8N8, 8 MB flash) | |
| CAN | ESP32 TWAI + transceiver | TX GPIO10, RX GPIO11, 250 kbit/s |
| I2C bus | | SDA GPIO39, SCL GPIO38, 400 kHz |
| Analog inputs | ADS1115 | 0x48, 4 channels |
| Digital I/O (main) | PCA9555 | 0x20 |
| Digital I/O (extender) | PCA9555 | 0x21-0x23 (auto-detected), 16 pins |
| Status LEDs | | GPIO48, GPIO47 |
| Optocoupler output | | GPIO18 |

[hardware/](hardware/) contains the schematic, Gerbers, BOM and pick-and-place files. [hardware/3d/](hardware/3d/) contains the joystick housing and button models.

## Devices

### Joystick

- 4 ADS1115 channels are sent as analog AUX-N inputs (function type "analogue, maintains position"), labelled `A1`-`A4`.
- 16 extender PCA9555 pins are sent as momentary boolean AUX-N inputs, labelled after where the button sits on the joystick:

| Bit | Label | Bit | Label |
|---|---|---|---|
| 0 | `X0` (spare) | 8 | `DR-` |
| 1 | `DL+` | 9 | `DR+` |
| 2 | `DL-` | 10 | `T4` |
| 3 | `ML` | 11 | `T3` |
| 4 | `MD` | 12 | `T2` |
| 5 | `MR` | 13 | `T1` |
| 6 | `MC` | 14 | `X14` (spare) |
| 7 | `MU` | 15 | `X15` (spare) |

The main PCA9555 and the optocoupler output are not used by this firmware yet.

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```sh
cd firmware
pio run -e joystick -t upload
pio device monitor
```

`joystick_ota` builds the same firmware with a Wi-Fi access point for updates: SSID `joystick-auxn`, password `12345678`. Open `http://192.168.4.1/` for live input values and `/update` to upload a new firmware.

### Changing the object pool

Edit the labels or IDs in [firmware/tools/build_aux_pool.py](firmware/tools/build_aux_pool.py), then run it from `firmware/`:

```sh
python tools/build_aux_pool.py
```

The pool version string sent to the VT is a hash of the pool content, so VTs re-upload a changed pool automatically.

## Before use on a real machine

The ISOBUS NAME uses the placeholder manufacturer code `1407` (the one AgIsoStack's examples use). That is fine for bench testing against AgIsoVirtualTerminal. Get a real assigned manufacturer code before deploying on a customer's ISOBUS network.

## Roadmap

- **Device profiles**: describe each device's inputs (source chip/pin, AUX-N function type, label) in one file per device. The object pool and the input mapping are generated from it, so a new keypad only needs a new profile.
- Use the main PCA9555 and skip unwired inputs.
- Configurable NAME function instance, so several devices can share one bus.

## License

[WTFPL](LICENSE). AgIsoStack++ is MIT licensed.
