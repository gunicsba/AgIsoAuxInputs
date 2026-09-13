# Brain board: ESP32S3R8N8 CAN Board V1.0

The central board that every AgIsoAuxInputs device runs on. It connects to the tractor's power and ISOBUS. Keypads and joysticks connect to it.

## Features

- **Power**: 9-16 V input with reverse-polarity protection (MOSFET ideal diode, NCV68061). A TPS5430 buck converter makes 5 V and an AMS1117 makes 3.3 V. The board can also be powered from USB-C.
- **CAN**: MCP2562 transceiver with PESD1CAN ESD protection (ESP32 TWAI on GPIO10/GPIO11).
- **On-board inputs**: ADS1115 4-channel ADC and a PCA9555 I/O expander (0x20). A device can be built on a breadboard with only buttons, switches and potentiometers.
- **I2C to keypads**: PCA9306 level shifter between the 3.3 V ESP32 and the 5 V keypad bus.
- **Switch output**: optocoupler (EL3H7) driven by GPIO18.

## Connectors

| Connector | Pins |
|---|---|
| WAGO terminals | 9-16V, GND, CAN_H, CAN_L, SWITCH, SWITCH_GND |
| USB-C | Power, programming, serial log |
| H4 (pots) | 5V, GND, POT1-POT4 |
| H5 (buttons) | BTN0-BTN3, LED1, GND |
| H6 (buttons) | BTN4-BTN7, LED2, GND |
| H1 (I2C) | 5V, SDA, SCL |
| H7 | GPIO7, GPIO8, GPIO9 (220 Ω series) |

## Files

| File | Use |
|---|---|
| `Gerber_brain.zip` | PCB fabrication |
| `BOM_brain.xlsx` | Bill of materials (LCSC part numbers) |
| `PickAndPlace_brain.xlsx` | Component placement for assembly |
| `schematic.pdf` | Pages 3-7 are this board (in the repo: `hardware/schematic.pdf`) |

To order: upload the Gerber ZIP to JLCPCB. For assembly, add the BOM and pick-and-place files.
