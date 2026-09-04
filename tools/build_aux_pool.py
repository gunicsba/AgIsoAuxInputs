#!/usr/bin/env python3
"""
Builds the minimal ISO 11783-6 AUX-N object pool (.iop) for the joystick_aux_n
firmware: one Working Set, one Data Mask showing "TEST", and one
Auxiliary Input Type 2 object per physical input (4 analog channels + 16
extender buttons) - each with a short text designator (e.g. "B1", "A1") so
the VT's aux-assignment screen shows something distinguishable per input
instead of generic/blank entries.

The byte layout below is not guesswork: it was reverse-engineered from the
actual object-pool *parser* in the AgIsoStack++ library
(isobus_virtual_terminal_working_set_base.cpp, VirtualTerminalWorkingSetBase::
parse_next_object) - i.e. the same code a real VT (including AgIsoVirtualTerminal)
uses to decode an incoming pool - and cross-checked against ISO 11783-6:2014/2018.
Per ISO 11783-6:2014 Table J.4 ("Number of objects to follow... used as the
Auxiliary Input designator... shall fit inside a Soft Key designator"), the
designator child isn't restricted to picture graphics - a small OutputString
is valid and much simpler than encoding bitmap icons.

Run this script whenever object IDs/count/text need to change:
    python tools/build_aux_pool.py
It (re)writes src/object_pool/aux_n_pool.iop and
src/object_pool/object_pool_ids.h.
"""
import pathlib
import struct

OUT_DIR = pathlib.Path(__file__).resolve().parent.parent / "src" / "object_pool"

NULL_OBJECT_ID = 0xFFFF

# --- Object IDs -------------------------------------------------------------
WORKING_SET = 0
MAIN_MASK = 1
TEST_STRING = 2
TEST_FONT = 3
DESIGNATOR_FONT = 4

NUM_ANALOG = 4
NUM_BUTTONS = 16
AUX_ANALOG_BASE = 100          # 100..103
AUX_BUTTON_BASE = 200          # 200..215
DESIGNATOR_ANALOG_BASE = 300   # 300..303
DESIGNATOR_BUTTON_BASE = 400   # 400..415

AUX_ANALOG_IDS = [AUX_ANALOG_BASE + i for i in range(NUM_ANALOG)]
AUX_BUTTON_IDS = [AUX_BUTTON_BASE + i for i in range(NUM_BUTTONS)]
DESIGNATOR_ANALOG_IDS = [DESIGNATOR_ANALOG_BASE + i for i in range(NUM_ANALOG)]
DESIGNATOR_BUTTON_IDS = [DESIGNATOR_BUTTON_BASE + i for i in range(NUM_BUTTONS)]

ANALOG_LABELS = [f"A{i + 1}" for i in range(NUM_ANALOG)]

# Mapped by physically pressing each button and reading which bit (0-15) of
# the extender PCA9555 changed in the firmware's "[BTN] <16 bits>" serial
# log - see the button-mapping session. Bits 0, 14, 15 aren't wired to a
# labelled physical button on this joystick.
BUTTON_LABELS = [
    "X0",   # 0 - unused/spare
    "DL+",  # 1
    "DL-",  # 2
    "ML",   # 3
    "MD",   # 4
    "MR",   # 5
    "MC",   # 6
    "MU",   # 7
    "DR-",  # 8
    "DR+",  # 9
    "T4",   # 10
    "T3",   # 11
    "T2",   # 12
    "T1",   # 13
    "X14",  # 14 - unused/spare
    "X15",  # 15 - unused/spare
]

# --- AuxiliaryInputType2 function types (ISO 11783-6:2018 Table J.5) --------
FUNCTION_TYPE_ANALOGUE_MAINTAINS_POSITION = 1   # potentiometer / joystick axis
FUNCTION_TYPE_BOOLEAN_MOMENTARY = 2             # push button, returns to off

# --- VT object type numbers (isobus_virtual_terminal_objects.hpp) ----------
TYPE_WORKING_SET = 0
TYPE_DATA_MASK = 1
TYPE_OUTPUT_STRING = 11
TYPE_FONT_ATTRIBUTES = 23
TYPE_AUXILIARY_INPUT_TYPE_2 = 32

# Designator box is a fixed 60x60, matching the CCI reference pool's sizing.
# FontSize::Size16x16 (16x16px/char) keeps the longest label ("X14"/"X15"/
# "DL+"/"DL-"/"DR+"/"DR-", 3 chars = 48px wide) well clear of the 60px
# edges - a bigger font would get clipped by the VT (objects/parts outside
# the designator area are clipped per ISO 11783-6 Table J.4).
DESIGNATOR_FONT_SIZE_ENUM = 4  # FontSize::Size16x16
DESIGNATOR_CHAR_W = 16
DESIGNATOR_BOX = 60
# justification bitfield: bits0-1 horizontal (1=middle), bits2-3 vertical (1=middle)
JUSTIFY_CENTERED = 0b0101


def u16(v):
    return struct.pack("<H", v & 0xFFFF)


def i16(v):
    return struct.pack("<h", v)


def u8(v):
    return struct.pack("<B", v & 0xFF)


def build_working_set():
    # header: ID(2) type(1) bg(1) selectable(1) activeMask(2) nChildren(1) nMacros(1) nLanguages(1) = 10 bytes
    data = u16(WORKING_SET) + u8(TYPE_WORKING_SET)
    data += u8(0)              # background colour
    data += u8(0)              # selectable = false (only one working set)
    data += u16(MAIN_MASK)     # active mask
    data += u8(1)              # 1 child: the data mask
    data += u8(0)              # 0 macros
    data += u8(0)              # 0 languages
    # child: (objectID, x, y)
    data += u16(MAIN_MASK) + i16(0) + i16(0)
    return data


def build_data_mask():
    # header: ID(2) type(1) bg(1) softKeyMask(2) nChildren(1) nMacros(1) = 8 bytes
    data = u16(MAIN_MASK) + u8(TYPE_DATA_MASK)
    data += u8(1)                    # background colour (white-ish index)
    data += u16(NULL_OBJECT_ID)      # no soft key mask
    data += u8(1)                    # 1 child: the TEST string
    data += u8(0)                    # 0 macros
    data += u16(TEST_STRING) + i16(10) + i16(10)
    return data


def build_output_string(object_id: int, width: int, height: int, font_id: int,
                         bg_colour: int, text: bytes, justify: int = 0):
    # header: ID(2) type(1) width(2) height(2) bg(1) font(2) options(1) varRef(2) justify(1) strLen(2) = 16 bytes
    data = u16(object_id) + u8(TYPE_OUTPUT_STRING)
    data += u16(width)
    data += u16(height)
    data += u8(bg_colour)
    data += u16(font_id)             # font attributes object
    data += u8(0)                    # options
    data += u16(NULL_OBJECT_ID)      # no string variable -> use inline value below
    data += u8(justify)
    data += u16(len(text))
    data += text
    data += u8(0)                    # 0 macros
    return data


def build_font_attributes(object_id: int, size_enum: int):
    # header: ID(2) type(1) colour(1) size(1) type(1) style(1) nMacros(1) = 8 bytes
    data = u16(object_id) + u8(TYPE_FONT_ATTRIBUTES)
    data += u8(0)          # colour (black)
    data += u8(size_enum)
    data += u8(0)           # FontType::ISO8859_1
    data += u8(0)           # style bitfield: none
    data += u8(0)           # 0 macros
    return data


def build_aux_input_type2(object_id: int, function_type: int, designator_id: int):
    # header: ID(2) type(1) bg(1) bitfield(1) nObjects(1) = 6 bytes, then 1
    # designator child: (objectID, x, y) = 6 bytes.
    data = u16(object_id) + u8(TYPE_AUXILIARY_INPUT_TYPE_2)
    data += u8(0)                 # background colour (not rendered)
    data += u8(function_type & 0x1F)  # bits 5-7 = 0: not critical, not single-assignment
    data += u8(1)                 # 1 linked object: the designator
    data += u16(designator_id) + i16(0) + i16(0)
    return data


def main():
    pool = bytearray()
    pool += build_working_set()
    pool += build_data_mask()
    pool += build_output_string(TEST_STRING, 120, 24, TEST_FONT, 1, b"TEST")
    pool += build_font_attributes(TEST_FONT, 3)          # FontSize::Size12x16
    pool += build_font_attributes(DESIGNATOR_FONT, DESIGNATOR_FONT_SIZE_ENUM)

    for oid, did, label in zip(AUX_ANALOG_IDS, DESIGNATOR_ANALOG_IDS, ANALOG_LABELS):
        pool += build_output_string(did, DESIGNATOR_BOX, DESIGNATOR_BOX,
                                     DESIGNATOR_FONT, 1, label.encode("ascii"),
                                     justify=JUSTIFY_CENTERED)
        pool += build_aux_input_type2(oid, FUNCTION_TYPE_ANALOGUE_MAINTAINS_POSITION, did)

    for oid, did, label in zip(AUX_BUTTON_IDS, DESIGNATOR_BUTTON_IDS, BUTTON_LABELS):
        pool += build_output_string(did, DESIGNATOR_BOX, DESIGNATOR_BOX,
                                     DESIGNATOR_FONT, 1, label.encode("ascii"),
                                     justify=JUSTIFY_CENTERED)
        pool += build_aux_input_type2(oid, FUNCTION_TYPE_BOOLEAN_MOMENTARY, did)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    iop_path = OUT_DIR / "aux_n_pool.iop"
    iop_path.write_bytes(bytes(pool))

    header_path = OUT_DIR / "object_pool_ids.h"
    lines = [
        "// Auto-generated by tools/build_aux_pool.py - do not edit by hand.",
        "#pragma once",
        "",
        f"#define WORKING_SET {WORKING_SET}",
        f"#define MAIN_MASK {MAIN_MASK}",
        f"#define TEST_STRING {TEST_STRING}",
        f"#define TEST_FONT {TEST_FONT}",
        f"#define DESIGNATOR_FONT {DESIGNATOR_FONT}",
        f"#define NUM_AUX_ANALOG {NUM_ANALOG}",
        f"#define NUM_AUX_BUTTONS {NUM_BUTTONS}",
        "",
        "static const uint16_t AUX_ANALOG_IDS[NUM_AUX_ANALOG] = {"
        + ", ".join(str(i) for i in AUX_ANALOG_IDS) + "};",
        "static const uint16_t AUX_BUTTON_IDS[NUM_AUX_BUTTONS] = {"
        + ", ".join(str(i) for i in AUX_BUTTON_IDS) + "};",
        "",
    ]
    header_path.write_text("\n".join(lines), encoding="ascii")

    # Embedded as a plain C byte array (not a linker-embedded binary): far
    # simpler and more portable across PlatformIO's various framework/build
    # backends than relying on board_build.embed_files's linker-symbol
    # mechanism, which turned out to be fragile under the mixed
    # arduino+espidf framework this firmware uses.
    data_header_path = OUT_DIR / "aux_n_pool_data.h"
    byte_lines = []
    for i in range(0, len(pool), 16):
        chunk = pool[i:i + 16]
        byte_lines.append("    " + ", ".join(f"0x{b:02X}" for b in chunk) + ",")
    data_lines = [
        "// Auto-generated by tools/build_aux_pool.py - do not edit by hand.",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        f"static const uint32_t AUX_N_POOL_SIZE = {len(pool)};",
        "static const uint8_t AUX_N_POOL_DATA[] = {",
        *byte_lines,
        "};",
        "",
    ]
    data_header_path.write_text("\n".join(data_lines), encoding="ascii")

    print(f"Wrote {iop_path} ({len(pool)} bytes)")
    print(f"Wrote {header_path}")
    print(f"Wrote {data_header_path}")


if __name__ == "__main__":
    main()
