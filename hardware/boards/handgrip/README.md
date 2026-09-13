# Handgrip keypad V1.0

A 13-button keypad for a handheld implement control grip. It has its own PCA9555 and connects to the [brain board](../brain/) over I2C.

The schematic and EasyEDA project call this board **Joystick Front**. The design source is on OSHWLab, in the same project as the brain board: [ESP32S3R8N8 CAN Board](https://oshwlab.com/gunicsba/esp32s3r8n8-can-board).

<img src="handgrip.jpg" alt="Assembled Handgrip keypad PCB with numbered buttons" width="480">

> The button layout is inspired by the HARDI NOVA sprayer grip. This project is not affiliated with or endorsed by HARDI. HARDI and NOVA are trademarks of their respective owners.

## Connector

| H3 pin | Signal |
|---|---|
| 1 | 5V |
| 2 | SDA |
| 3 | SCL |
| 4 | GND |

## I2C address: set the jumpers

The PCA9555 address is set by the **ADDR selector** solder jumpers (A2 is tied to GND). With both jumpers open the address is **0x20**. That clashes with the brain board's own PCA9555, so bridge at least one jumper:

| JP7 (A0) | JP6 (A1) | Address |
|---|---|---|
| open | open | 0x20 (conflicts with the brain board) |
| bridged | open | 0x21 |
| open | bridged | 0x22 |
| bridged | bridged | 0x23 |

The firmware auto-detects the keypad at 0x21-0x23.

## Buttons

The number printed next to each button on the silkscreen is its PCA9555 bit (silkscreen `n` = net `BTNn` = bit `n`). Bits 0, 14 and 15 are not connected to a button.

| Silkscreen / bit | Position | Firmware label |
|---|---|---|
| 1, 2 | Left diagonal (outer, inner) | `DL+`, `DL-` |
| 3 | Cross left | `ML` |
| 4 | Cross down | `MD` |
| 5 | Cross right | `MR` |
| 6 | Cross center | `MC` |
| 7 | Cross up | `MU` |
| 8, 9 | Right diagonal (inner, outer) | `DR-`, `DR+` |
| 10-13 | Top edge, right to left | `T4`-`T1` |

## Files

| File | Use |
|---|---|
| `Gerber_handgrip.zip` | PCB fabrication |
| `BOM_handgrip.xlsx` | Bill of materials (LCSC part numbers) |
| `PickAndPlace_handgrip.xlsx` | Component placement for assembly |
| `schematic.pdf` | Pages 1-2 are this board (in the repo: `hardware/schematic.pdf`) |

The matching housing is in [enclosures/handgrip](../../enclosures/handgrip/).
