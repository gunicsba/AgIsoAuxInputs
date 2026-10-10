#!/usr/bin/env python3
"""
Builds the minimal ISO 11783-6 AUX-N object pool (.iop) for the joystick
firmware: one Working Set, a main Data Mask drawing the handgrip's button and
LED layout, a "Lights" Data
Mask (mode button, R/G/B and brightness inputs for the LEDs on GPIO7), soft
keys to toggle the lights and switch between the two masks, and one Auxiliary
Input Type 2 object per physical input (4 analog channels + 16
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

Run this script (from firmware/) whenever object IDs/count/text need to change:
    python tools/build_aux_pool.py
It (re)writes src/object_pool/aux_n_pool.iop, src/object_pool/object_pool_ids.h
and src/object_pool/aux_n_pool_data.h.
"""
import pathlib
import struct

OUT_DIR = pathlib.Path(__file__).resolve().parent.parent / "src" / "object_pool"

NULL_OBJECT_ID = 0xFFFF

# --- Object IDs -------------------------------------------------------------
WORKING_SET = 0
MAIN_MASK = 1
TEST_FONT = 3               # 12x16 font for all mask labels
DESIGNATOR_FONT = 4
LIGHTS_MASK = 5
WS_DESIGNATOR = 6
MAIN_SOFT_KEY_MASK = 7
LIGHTS_SOFT_KEY_MASK = 8
KEY_FONT = 9

# Lights UI (see src/lights.cpp). Keys and buttons are reported by object ID
# in VT Soft Key / Button Activation messages; the inputs carry their value
# inline (no number variable), so the firmware updates them with Change
# Numeric Value on the input object itself.
BTN_LIGHTS_MODE = 12
KEY_LIGHTS_TOGGLE = 14     # on both soft key masks
KEY_LIGHTS_PAGE = 15       # main mask -> lights mask
KEY_LIGHTS_BACK = 16       # lights mask -> main mask
STR_LIGHTS_MODE = 22
STR_KEY_TOGGLE_TITLE = 24
STR_KEY_TOGGLE_STATE = 25
STR_KEY_PAGE = 26
STR_KEY_BACK = 27
STR_LIGHTS_TITLE = 30
STR_RED = 31
STR_GREEN = 32
STR_BLUE = 33
STR_BRIGHTNESS = 34
INPUT_RED = 40
INPUT_GREEN = 41
INPUT_BLUE = 42
INPUT_BRIGHTNESS = 43
# Yellow inputs with a black frame mark what the operator can edit. One
# rectangle object is drawn behind every input.
INPUT_FRAME_LINE = 35
INPUT_FRAME = 36

# Handgrip picture on the main mask: one outlined rectangle + label per
# button, one small square per LED. Each rectangle has its own fill
# attributes so the firmware can later recolour it (pressed button, live LED
# colour) with Change Fill Attributes.
JOY_BUTTON_RECT_BASE = 500     # + PCA9555 bit
JOY_BUTTON_FILL_BASE = 520     # + PCA9555 bit
JOY_BUTTON_LABEL_BASE = 540    # + PCA9555 bit
JOY_LED_RECT_BASE = 560        # + LED index (0..8, order on the GPIO7 chain)
JOY_LED_FILL_BASE = 580        # + LED index
JOY_LINE = 37

# Main mask header/footer. The commit is filled in at runtime from
# src/git_version.h (see tools/git_version.py), so a new commit doesn't
# change the pool and force a VT re-upload.
STR_MAIN_TITLE = 50
STR_REPO_URL = 51
STR_COMMIT = 52
MAIN_TITLE_TEXT = b"AgIsoAuxInputs"
REPO_URL_TEXT = b"github.com/gunicsba/AgIsoAuxInputs"
COMMIT_PLACEHOLDER = b"--------"   # 7-char hash + "+" for uncommitted changes

# Initial texts. The firmware overwrites the toggle/mode labels with
# send_change_string_value, padding every variant to these same lengths -
# the pool string length is what the VT has room for.
TOGGLE_STATE_TEXT = b"OFF"
MODE_TEXT = b"MODE: RAINBOW"

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
TYPE_SOFT_KEY_MASK = 4
TYPE_KEY = 5
TYPE_BUTTON = 6
TYPE_INPUT_NUMBER = 9
TYPE_OUTPUT_RECTANGLE = 14
TYPE_LINE_ATTRIBUTES = 24
TYPE_FILL_ATTRIBUTES = 25
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
JUSTIFY_RIGHT_MIDDLE = 0b0110

# VT standard palette indices (ISO 11783-6 Table A.3)
COLOUR_BLACK = 0
COLOUR_WHITE = 1
COLOUR_SILVER = 7
COLOUR_YELLOW = 14

# Light page layout is drawn for a 480x480 data mask (AgIsoVirtualTerminal's
# default); a VT with a smaller mask will clip the lower rows.
BUTTON_W = 220
BUTTON_H = 50
LABEL_FONT_SIZE_ENUM = 3   # FontSize::Size12x16
KEY_FONT_SIZE_ENUM = 2     # FontSize::Size8x12: "LIGHTS" = 48px in a 60px key
INPUT_W = 100
INPUT_H = 34
FRAME_MARGIN = 3

# Handgrip layout (centre points on the 480x480 mask), mirrored around
# x=240. The keypad is a V whose point sits between MC and MD: DL+/DR+ at
# the top of the arms, DL-/DR- lower, slightly above MU and outside ML/MR.
# T1-T4 run across the top. Buttons are square so an assigned AUX-N
# function's designator can be drawn into them later.
JOY_BUTTON_W = 50
JOY_BUTTON_H = 50
JOY_LED_SIZE = 12
JOY_CENTRE_X = 240
T_ROW_Y = 110
M_CENTRE_Y = 335
M_PITCH = 70       # button + 20px gap for an LED


def mirror(x: int, y: int):
    return 2 * JOY_CENTRE_X - x, y


def between(a, b):
    return (a[0] + b[0]) // 2, (a[1] + b[1]) // 2


JOY_BUTTON_CENTRES = {
    "T1": (128, T_ROW_Y), "T2": (203, T_ROW_Y),
    "T3": (278, T_ROW_Y), "T4": (353, T_ROW_Y),
    "DL+": (50, 150), "DR+": mirror(50, 150),
    "DL-": (100, 240), "DR-": mirror(100, 240),
    "MU": (JOY_CENTRE_X, M_CENTRE_Y - M_PITCH),
    "ML": (JOY_CENTRE_X - M_PITCH, M_CENTRE_Y),
    "MC": (JOY_CENTRE_X, M_CENTRE_Y),
    "MR": (JOY_CENTRE_X + M_PITCH, M_CENTRE_Y),
    "MD": (JOY_CENTRE_X, M_CENTRE_Y + M_PITCH),
}
_b = JOY_BUTTON_CENTRES
# The T-row LEDs sit in the gaps, flush with the buttons' bottom edge.
_T_LED_Y = T_ROW_Y + JOY_BUTTON_H // 2 - JOY_LED_SIZE // 2
# LED marks from the handgrip drawing. Chain order isn't known yet - these
# are numbered top to bottom, left to right.
JOY_LED_CENTRES = [
    (between(_b["T1"], _b["T2"])[0], _T_LED_Y),
    (between(_b["T2"], _b["T3"])[0], _T_LED_Y),
    (between(_b["T3"], _b["T4"])[0], _T_LED_Y),
    between(_b["DL+"], _b["DL-"]), between(_b["DR+"], _b["DR-"]),
    between(_b["MU"], _b["MC"]),
    between(_b["ML"], _b["MC"]), between(_b["MC"], _b["MR"]),
    between(_b["MC"], _b["MD"]),          # the point of the V
]


def u16(v):
    return struct.pack("<H", v & 0xFFFF)


def i16(v):
    return struct.pack("<h", v)


def u8(v):
    return struct.pack("<B", v & 0xFF)


def build_working_set():
    # header: ID(2) type(1) bg(1) selectable(1) activeMask(2) nChildren(1) nMacros(1) nLanguages(1) = 10 bytes
    data = u16(WORKING_SET) + u8(TYPE_WORKING_SET)
    data += u8(COLOUR_WHITE)   # background colour
    data += u8(1)              # selectable: the operator can open our masks
    data += u16(MAIN_MASK)     # active mask
    data += u8(1)              # 1 child: the designator shown in the VT's WS list
    data += u8(0)              # 0 macros
    data += u8(0)              # 0 languages
    # child: (objectID, x, y)
    data += u16(WS_DESIGNATOR) + i16(0) + i16(0)
    return data


def build_data_mask(object_id: int, soft_key_mask_id: int, children):
    # header: ID(2) type(1) bg(1) softKeyMask(2) nChildren(1) nMacros(1) = 8 bytes
    data = u16(object_id) + u8(TYPE_DATA_MASK)
    data += u8(COLOUR_WHITE)         # background colour
    data += u16(soft_key_mask_id)
    data += u8(len(children))
    data += u8(0)                    # 0 macros
    for child_id, x, y in children:
        data += u16(child_id) + i16(x) + i16(y)
    return data


def build_soft_key_mask(object_id: int, key_ids):
    # header: ID(2) type(1) bg(1) nObjects(1) nMacros(1) = 6 bytes, then one
    # object ID per key (keys are placed in order, no x/y).
    data = u16(object_id) + u8(TYPE_SOFT_KEY_MASK)
    data += u8(COLOUR_SILVER)        # background colour
    data += u8(len(key_ids))
    data += u8(0)                    # 0 macros
    for key_id in key_ids:
        data += u16(key_id)
    return data


def build_key(object_id: int, children):
    # header: ID(2) type(1) bg(1) keyCode(1) nObjects(1) nMacros(1) = 7 bytes,
    # then (objectID, x, y) per child inside the soft key designator.
    data = u16(object_id) + u8(TYPE_KEY)
    data += u8(COLOUR_SILVER)        # background colour
    data += u8(object_id & 0xFF)     # key code (unused - firmware matches on object ID)
    data += u8(len(children))
    data += u8(0)                    # 0 macros
    for child_id, x, y in children:
        data += u16(child_id) + i16(x) + i16(y)
    return data


def build_key_label(object_id: int, height: int, font_id: int, text: bytes):
    return build_output_string(object_id, DESIGNATOR_BOX, height, font_id,
                                COLOUR_SILVER, text, justify=JUSTIFY_CENTERED,
                                transparent=True)


def build_line_attributes(object_id: int, colour: int, width: int):
    # ID(2) type(1) colour(1) width(1) lineArt(2) nMacros(1) = 8 bytes
    data = u16(object_id) + u8(TYPE_LINE_ATTRIBUTES)
    data += u8(colour)
    data += u8(width)
    data += u16(0xFFFF)              # solid line
    data += u8(0)                    # 0 macros
    return data


def build_fill_attributes(object_id: int, colour: int):
    # ID(2) type(1) fillType(1) colour(1) pattern(2) nMacros(1) = 8 bytes
    data = u16(object_id) + u8(TYPE_FILL_ATTRIBUTES)
    data += u8(2)                    # fill with the colour below
    data += u8(colour)
    data += u16(NULL_OBJECT_ID)      # no pattern
    data += u8(0)                    # 0 macros
    return data


def build_output_rectangle(object_id: int, line_id: int, width: int, height: int,
                           fill_id: int = NULL_OBJECT_ID):
    # ID(2) type(1) lineAttr(2) width(2) height(2) lineSuppression(1)
    # fillAttr(2) nMacros(1) = 13 bytes. No fill attributes = outline only.
    data = u16(object_id) + u8(TYPE_OUTPUT_RECTANGLE)
    data += u16(line_id)
    data += u16(width)
    data += u16(height)
    data += u8(0)                    # draw all four sides
    data += u16(fill_id)
    data += u8(0)                    # 0 macros
    return data


def build_button(object_id: int, label_id: int):
    # header: ID(2) type(1) width(2) height(2) bg(1) border(1) keyCode(1)
    # options(1) nChildren(1) nMacros(1) = 13 bytes, then (objectID, x, y)
    # per child. Options 0 = momentary, VT-drawn border, enabled.
    data = u16(object_id) + u8(TYPE_BUTTON)
    data += u16(BUTTON_W)
    data += u16(BUTTON_H)
    data += u8(COLOUR_SILVER)        # background colour
    data += u8(COLOUR_BLACK)         # border colour
    data += u8(object_id & 0xFF)     # key code (unused - firmware matches on object ID)
    data += u8(0)                    # options
    data += u8(1)                    # 1 child: the label string
    data += u8(0)                    # 0 macros
    # Label fills the button face minus a 5px margin for the VT's border.
    data += u16(label_id) + i16(5) + i16(5)
    return data


def build_button_label(object_id: int, text: bytes):
    return build_output_string(object_id, BUTTON_W - 10, BUTTON_H - 10,
                                TEST_FONT, COLOUR_SILVER, text,
                                justify=JUSTIFY_CENTERED, transparent=True)


def build_input_number(object_id: int, value: int, maximum: int):
    # ID(2) type(1) width(2) height(2) bg(1) font(2) options(1) varRef(2)
    # value(4) min(4) max(4) offset(4) scale(float 4) decimals(1) format(1)
    # justify(1) options2(1) nMacros(1) = 38 bytes
    data = u16(object_id) + u8(TYPE_INPUT_NUMBER)
    data += u16(INPUT_W)
    data += u16(INPUT_H)
    data += u8(COLOUR_YELLOW)        # background colour: marks it editable
    data += u16(TEST_FONT)           # font attributes object
    data += u8(0)                    # options
    data += u16(NULL_OBJECT_ID)      # no number variable -> inline value
    data += struct.pack("<I", value)
    data += struct.pack("<I", 0)     # minimum
    data += struct.pack("<I", maximum)
    data += struct.pack("<i", 0)     # offset
    data += struct.pack("<f", 1.0)   # scale
    data += u8(0)                    # 0 decimals
    data += u8(0)                    # fixed (not exponential) format
    data += u8(JUSTIFY_RIGHT_MIDDLE)
    data += u8(1)                    # options2: enabled, no real-time editing
    data += u8(0)                    # 0 macros
    return data


def build_output_string(object_id: int, width: int, height: int, font_id: int,
                         bg_colour: int, text: bytes, justify: int = 0,
                         transparent: bool = False):
    # header: ID(2) type(1) width(2) height(2) bg(1) font(2) options(1) varRef(2) justify(1) strLen(2) = 16 bytes
    data = u16(object_id) + u8(TYPE_OUTPUT_STRING)
    data += u16(width)
    data += u16(height)
    data += u8(bg_colour)
    data += u16(font_id)             # font attributes object
    data += u8(1 if transparent else 0)  # options bit 0: transparent
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
    main_children = [
        (STR_MAIN_TITLE, 0, 20),
        (STR_REPO_URL, 10, 452),
        (STR_COMMIT, 390, 452),
    ]
    for label, (cx, cy) in JOY_BUTTON_CENTRES.items():
        bit = BUTTON_LABELS.index(label)
        x, y = cx - JOY_BUTTON_W // 2, cy - JOY_BUTTON_H // 2
        main_children += [
            (JOY_BUTTON_RECT_BASE + bit, x, y),
            (JOY_BUTTON_LABEL_BASE + bit, x, y),
        ]
    for led, (cx, cy) in enumerate(JOY_LED_CENTRES):
        main_children.append((JOY_LED_RECT_BASE + led, cx - JOY_LED_SIZE // 2,
                              cy - JOY_LED_SIZE // 2))
    pool += build_data_mask(MAIN_MASK, MAIN_SOFT_KEY_MASK, main_children)
    lights_children = [
        (STR_LIGHTS_TITLE, 10, 10),
        (BTN_LIGHTS_MODE, 10, 50),
    ]
    for row, (label_id, input_id) in enumerate([
            (STR_RED, INPUT_RED), (STR_GREEN, INPUT_GREEN),
            (STR_BLUE, INPUT_BLUE), (STR_BRIGHTNESS, INPUT_BRIGHTNESS)]):
        y = 130 + row * 50
        lights_children += [
            (label_id, 10, y + 5),
            # Frame first so the input is drawn on top of it.
            (INPUT_FRAME, 170 - FRAME_MARGIN, y - FRAME_MARGIN),
            (input_id, 170, y),
        ]
    pool += build_data_mask(LIGHTS_MASK, LIGHTS_SOFT_KEY_MASK, lights_children)
    pool += build_soft_key_mask(MAIN_SOFT_KEY_MASK,
                                [KEY_LIGHTS_TOGGLE, KEY_LIGHTS_PAGE])
    pool += build_soft_key_mask(LIGHTS_SOFT_KEY_MASK,
                                [KEY_LIGHTS_TOGGLE, KEY_LIGHTS_BACK])
    pool += build_font_attributes(TEST_FONT, LABEL_FONT_SIZE_ENUM)
    pool += build_font_attributes(DESIGNATOR_FONT, DESIGNATOR_FONT_SIZE_ENUM)
    pool += build_output_string(WS_DESIGNATOR, DESIGNATOR_BOX, DESIGNATOR_BOX,
                                 DESIGNATOR_FONT, COLOUR_WHITE, b"JOY",
                                 justify=JUSTIFY_CENTERED)

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

    pool += build_output_string(STR_MAIN_TITLE, 480, 40, DESIGNATOR_FONT,
                                 COLOUR_WHITE, MAIN_TITLE_TEXT,
                                 justify=JUSTIFY_CENTERED)
    pool += build_output_string(STR_REPO_URL, 330, 20, KEY_FONT, COLOUR_WHITE,
                                 REPO_URL_TEXT)
    pool += build_output_string(STR_COMMIT, 80, 20, KEY_FONT, COLOUR_WHITE,
                                 COMMIT_PLACEHOLDER,
                                 justify=JUSTIFY_RIGHT_MIDDLE)

    # Handgrip picture
    pool += build_line_attributes(JOY_LINE, COLOUR_BLACK, 2)
    for label in JOY_BUTTON_CENTRES:
        bit = BUTTON_LABELS.index(label)
        pool += build_fill_attributes(JOY_BUTTON_FILL_BASE + bit, COLOUR_WHITE)
        pool += build_output_rectangle(JOY_BUTTON_RECT_BASE + bit, JOY_LINE,
                                       JOY_BUTTON_W, JOY_BUTTON_H,
                                       JOY_BUTTON_FILL_BASE + bit)
        pool += build_output_string(JOY_BUTTON_LABEL_BASE + bit, JOY_BUTTON_W,
                                     JOY_BUTTON_H, TEST_FONT, COLOUR_WHITE,
                                     label.encode("ascii"),
                                     justify=JUSTIFY_CENTERED, transparent=True)
    for led in range(len(JOY_LED_CENTRES)):
        pool += build_fill_attributes(JOY_LED_FILL_BASE + led, COLOUR_SILVER)
        pool += build_output_rectangle(JOY_LED_RECT_BASE + led, JOY_LINE,
                                       JOY_LED_SIZE, JOY_LED_SIZE,
                                       JOY_LED_FILL_BASE + led)

    # Soft keys. The toggle key shows "LIGHT" over the current state.
    pool += build_font_attributes(KEY_FONT, KEY_FONT_SIZE_ENUM)
    pool += build_key(KEY_LIGHTS_TOGGLE, [
        (STR_KEY_TOGGLE_TITLE, 0, 6),
        (STR_KEY_TOGGLE_STATE, 0, 26),
    ])
    pool += build_key_label(STR_KEY_TOGGLE_TITLE, 16, KEY_FONT, b"LIGHT")
    pool += build_key_label(STR_KEY_TOGGLE_STATE, 24, DESIGNATOR_FONT,
                            TOGGLE_STATE_TEXT)
    pool += build_key(KEY_LIGHTS_PAGE, [(STR_KEY_PAGE, 0, 0)])
    pool += build_key_label(STR_KEY_PAGE, DESIGNATOR_BOX, KEY_FONT, b"LIGHTS")
    pool += build_key(KEY_LIGHTS_BACK, [(STR_KEY_BACK, 0, 0)])
    pool += build_key_label(STR_KEY_BACK, DESIGNATOR_BOX, KEY_FONT, b"BACK")

    # Lights page
    pool += build_button(BTN_LIGHTS_MODE, STR_LIGHTS_MODE)
    pool += build_button_label(STR_LIGHTS_MODE, MODE_TEXT)
    pool += build_line_attributes(INPUT_FRAME_LINE, COLOUR_BLACK, 2)
    pool += build_output_rectangle(INPUT_FRAME, INPUT_FRAME_LINE,
                                   INPUT_W + 2 * FRAME_MARGIN,
                                   INPUT_H + 2 * FRAME_MARGIN)
    for string_id, text in [
            (STR_LIGHTS_TITLE, b"LIGHTS"),
            (STR_RED, b"RED"),
            (STR_GREEN, b"GREEN"),
            (STR_BLUE, b"BLUE"),
            (STR_BRIGHTNESS, b"BRIGHTNESS %")]:
        pool += build_output_string(string_id, 150, 24, TEST_FONT,
                                     COLOUR_WHITE, text)
    pool += build_input_number(INPUT_RED, 255, 255)
    pool += build_input_number(INPUT_GREEN, 255, 255)
    pool += build_input_number(INPUT_BLUE, 255, 255)
    pool += build_input_number(INPUT_BRIGHTNESS, 50, 100)

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
        f"#define TEST_FONT {TEST_FONT}",
        f"#define DESIGNATOR_FONT {DESIGNATOR_FONT}",
        f"#define LIGHTS_MASK {LIGHTS_MASK}",
        f"#define BTN_LIGHTS_MODE {BTN_LIGHTS_MODE}",
        f"#define KEY_LIGHTS_TOGGLE {KEY_LIGHTS_TOGGLE}",
        f"#define KEY_LIGHTS_PAGE {KEY_LIGHTS_PAGE}",
        f"#define KEY_LIGHTS_BACK {KEY_LIGHTS_BACK}",
        f"#define STR_KEY_TOGGLE_STATE {STR_KEY_TOGGLE_STATE}",
        f"#define STR_LIGHTS_MODE {STR_LIGHTS_MODE}",
        f"#define INPUT_RED {INPUT_RED}",
        f"#define INPUT_GREEN {INPUT_GREEN}",
        f"#define INPUT_BLUE {INPUT_BLUE}",
        f"#define INPUT_BRIGHTNESS {INPUT_BRIGHTNESS}",
        f"#define JOY_BUTTON_FILL_BASE {JOY_BUTTON_FILL_BASE}",
        f"#define STR_COMMIT {STR_COMMIT}",
        f"#define COMMIT_TEXT_LENGTH {len(COMMIT_PLACEHOLDER)}",
        "// Bit n set = PCA9555 bit n is drawn on the main mask (has a fill object)",
        f"#define JOY_BUTTONS_ON_SCREEN 0x{sum(1 << BUTTON_LABELS.index(l) for l in JOY_BUTTON_CENTRES):04X}",
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
