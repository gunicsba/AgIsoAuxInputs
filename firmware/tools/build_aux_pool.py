#!/usr/bin/env python3
"""
Builds the minimal ISO 11783-6 AUX-N object pool (.iop) for the joystick
firmware: one Working Set, a main Data Mask drawing the handgrip's button and
LED layout (with a pointer per button that shows the assigned AUX function's
designator), a "Lights" Data Mask (mode button, R/G/B and brightness inputs
for the LEDs on GPIO7), a "Settings" Data Mask, soft keys to toggle the
lights and switch between the masks, and Auxiliary Input Type 2 objects for
the physical inputs - each with a short text designator (e.g. "MC", "A1") so
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

Pools are built for two page modes: "single" (one AUX input per button) and
"multi" (T1-T4 select a page; the other buttons have an input per page plus
an all-pages "*" input), the firmware picking one at boot from its settings.
Each comes in two data mask layouts, "full" (480x480) and "compact"
(200x200), which the firmware scales to the connected VT (see "Scaling").

Run this script (from firmware/) whenever object IDs/count/text need to change:
    python tools/build_aux_pool.py
It (re)writes src/object_pool/aux_n_pool_{single,multi}_{full,compact}.iop,
src/object_pool/object_pool_ids.h and src/object_pool/aux_n_pool_data.h.
After a layout change, run tools/check_pool_scaling.py: it scales the pools
for a range of VTs like the firmware does and checks that everything fits.
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
# Data mask fonts. Scaled with the compact layout, unlike the designator and
# soft key fonts (designators and soft keys keep their 60x32 on every VT).
MASK_TITLE_FONT = 10        # 16x16 main title
MASK_SMALL_FONT = 11        # 8x12 repo URL and commit
PAGED_DESIGNATOR_FONT = 13  # 12x16 multi-page designators ("MC*")

# Lights UI (see src/lights.cpp). Keys and buttons are reported by object ID
# in VT Soft Key / Button Activation messages; the inputs carry their value
# inline (no number variable), so the firmware updates them with Change
# Numeric Value on the input object itself.
BTN_LIGHTS_MODE = 12
KEY_LIGHTS_TOGGLE = 14     # on the main and lights soft key masks
KEY_LIGHTS_PAGE = 15       # main mask -> lights mask
KEY_BACK = 16              # lights / settings mask -> main mask
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

# Settings mask (see src/settings.cpp). Button labels are rewritten at runtime
# and padded to the lengths below.
SETTINGS_MASK = 60
SETTINGS_SOFT_KEY_MASK = 61
KEY_SETTINGS = 62
STR_KEY_SETTINGS = 63
BTN_PAGE_MODE = 64
STR_PAGE_MODE = 65
BTN_RESTART = 66
STR_RESTART = 67
STR_RESTART_HINT = 68
BTN_PAGE_COLOUR_BASE = 70      # + page index 0..3
STR_PAGE_COLOUR_BASE = 74      # + page index 0..3
BTN_PRESS_FLASH = 78
STR_PRESS_FLASH = 79
STR_SETTINGS_TITLE = 80
PAGE_MODE_TEXT = b"PAGES: SINGLE"
RESTART_HINT_TEXT = b"                        "   # "RESTART TO APPLY" when pending
PAGE_COLOUR_TEXT = b"PAGE 1: MAGENTA"
PRESS_FLASH_TEXT = b"PRESS FLASH: ON "

# --- Single vs multi-page mode ----------------------------------------------
# Single: every button is one AUX-N input. Multi: T1-T4 select page 1-4
# (they're not AUX inputs), and every other button has five inputs - one per
# page plus a "*" input used on all pages unless the current page's own input
# is assigned. Each mode has its own model identification code so the VT
# keeps separate preferred assignments for the two input sets.
MODEL_ID_SINGLE = 1
MODEL_ID_MULTI = 2
NUM_PAGES = 4
PAGED_LAYERS = NUM_PAGES + 1   # layer 4 = all pages ("*")
PAGE_SELECT_LABELS = ["T1", "T2", "T3", "T4"]   # page 1..4
AUX_PAGED_BASE = 600           # + bit * NUM_PAGES + page ("*" = single-mode ID)
DESIGNATOR_PAGED_BASE = 700    # + bit * NUM_PAGES + page
# Auxiliary Control Designator Type 2 Object Pointers: one per on-screen AUX
# button, pointer type 1 = the VT draws the designator of the function
# assigned to the referenced input. Multi-page mode retargets them at runtime
# (Change Attribute) to the input that's active on the current page.
AUX_POINTER_BASE = 800         # + bit
POINTER_TYPE_ASSIGNED_FUNCTION = 1
# The pointers stay in the pool (the firmware retargets them) but are only
# placed on the main mask when this is set. Off while investigating
# AgIsoVirtualTerminal 1.5.1-mods hanging once they were shown.
SHOW_AUX_POINTERS = False
# Multi-page mode: 4 small squares at the bottom of each paged button show
# which page's input is active (X---, -X--, XXXX for "*").
PAGE_IND_RECT_BASE = 900       # + bit * 4 + page
PAGE_IND_FILL_BASE = 1000      # + bit * 4 + page
JOY_THIN_LINE = 38

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
TYPE_AUXILIARY_CONTROL_DESIGNATOR_TYPE_2 = 33
TYPE_STRING_VARIABLE = 22

# --- Scaling ------------------------------------------------------------------
# ISO 11783-6 (4.6.16) leaves scaling to the Working Set: it reads the VT's
# Data Mask and Soft Key designator sizes and adapts positions, sizes and
# fonts itself (only Picture Graphic bitmaps are scaled by the VT, from the
# object's width attribute). So the pools are built for a reference size and
# every number that depends on it is recorded as a scale field (offset, kind,
# reference value). The firmware rewrites those fields for the connected VT
# before the upload - the pool's bytes otherwise stay as built here.
#
# Data masks: two hand-made layouts, each scaled to the VT's exact mask
# size. Below COMPACT_BELOW_MASK the full layout's text would end up in the
# 6x8 font and clip, so smaller masks (200x200 AMATRON 3, 240, ...) get the
# compact one.
FULL_MASK = 480
COMPACT_MASK = 200
COMPACT_BELOW_MASK = 320

# Soft keys (keys, the working set designator, AUX input designators): two
# shapes, picked by the VT's designator aspect ratio and scaled to its size
# on each axis. Square for keys at least 3/4 as high as wide (60x60, 60x47),
# wide for flatter ones (120x80, 60x32). The pool bytes as built carry the
# wide values, which fit the smallest designator ISO allows (60x32) - an
# AMATRON 3 rejected the pool outright while an AUX input's designator was
# larger than its soft keys ("Faulty Object 100").
KEY_SQUARE = (60, 60)
KEY_WIDE = (60, 32)

# Scale field kinds - shared with the firmware (src/object_pool_adapt.cpp)
SCALE_MASK_POS = 0    # int16, x mask scale
SCALE_MASK_SIZE = 1   # uint16, x mask scale, at least 1
SCALE_MASK_LINE = 2   # uint8 line width, x mask scale, at least 1
SCALE_MASK_FONT = 3   # uint8 font size enum, best fit for the scaled size
SCALE_KEY_X = 4       # int16, x key width scale
SCALE_KEY_Y = 5       # int16, x key height scale
SCALE_KEY_W = 6       # uint16, x key width scale, at least 1
SCALE_KEY_H = 7       # uint16, x key height scale, at least 1
SCALE_KEY_FONT = 8    # uint8 font size enum, best fit for the scaled size
SCALE_KIND_FORMAT = {SCALE_MASK_POS: "<h", SCALE_MASK_SIZE: "<H", SCALE_MASK_LINE: "<B",
                     SCALE_MASK_FONT: "<B", SCALE_KEY_X: "<h", SCALE_KEY_Y: "<h",
                     SCALE_KEY_W: "<H", SCALE_KEY_H: "<H", SCALE_KEY_FONT: "<B"}

# ISO 11783-6 font size enum -> (width, height) in px
FONT_SIZES = [(6, 8), (8, 8), (8, 12), (12, 16), (16, 16), (16, 24), (24, 32),
              (32, 32), (32, 48), (48, 64), (64, 64), (64, 96), (96, 128),
              (128, 128), (128, 192)]
FONT_6X8, FONT_8X12, FONT_12X16, FONT_16X16 = 0, 2, 3, 4

# Soft key fonts (fixed object IDs, same in both layouts)
KEY_FONTS = {
    KEY_FONT: FONT_8X12,               # "LIGHTS" = 48px in a 60px key
    DESIGNATOR_FONT: FONT_16X16,       # "X14", "DL+" = 48px
    PAGED_DESIGNATOR_FONT: FONT_12X16,  # "DL+*" = 48px
}

# Where a one-line label sits in a designator, per shape: (x, y, w, h). VT3
# has no vertical justification, so the text box itself is centred.
DESIGNATOR_BOX = ((0, 18, 60, 24), (0, 6, 60, 20))
KEY_LABEL_BOX = ((0, 22, 60, 16), (0, 8, 60, 16))
# The light toggle key: "LIGHT" over the current state
KEY_TOGGLE_TITLE_BOX = ((0, 10, 60, 14), (0, 0, 60, 14))
KEY_TOGGLE_STATE_BOX = ((0, 28, 60, 20), (0, 15, 60, 16))

# AUX-N (the type 2 AUX objects) arrived with VT version 3. A VT2 - e.g. the
# AMATRON 3 that reports version byte 2 - rejects the pool at the first of
# them ("Faulty Object 100", other error). The pool size is already sent by
# the time the VT's version is known, so the firmware can't drop them;
# instead it rewrites each one in place into a String Variable of the same
# length (ID(2) type(1) length(2) value), which nothing references.
VT3_OBJECT_TYPES = (TYPE_AUXILIARY_INPUT_TYPE_2, TYPE_AUXILIARY_CONTROL_DESIGNATOR_TYPE_2)
STRING_VARIABLE_HEADER = 5
# Pads the smaller of a mode's two layouts to the larger one's size: the
# pool size goes to the VT before its mask size picks the layout.
FILLER_OBJECT = 65000

# justification bitfield: bits0-1 horizontal (0=left, 1=middle, 2=right),
# bits2-3 vertical (0=top, 1=middle). These are the values the UI is drawn
# for, but vertical justification is a VT4+ feature: the bits are reserved on
# VT3 and older, and a Mueller VT3 rejected the whole pool with them set. So
# the pools ship with only the horizontal bits (JUSTIFY_HORIZONTAL_MASK), and
# every byte that lost a vertical bit is recorded for the firmware to patch
# back once the connected VT reports version 4+.
JUSTIFY_CENTERED = 0b0101
JUSTIFY_RIGHT_MIDDLE = 0b0110
JUSTIFY_HORIZONTAL_MASK = 0b0011

# VT standard palette indices (ISO 11783-6 Table A.3)
COLOUR_BLACK = 0
COLOUR_WHITE = 1
COLOUR_SILVER = 7
COLOUR_YELLOW = 14

# --- Layouts ------------------------------------------------------------------
# Handgrip picture (centre points on the 480x480 mask), mirrored around
# x=240. The keypad is a V whose point sits between MC and MD: DL+/DR+ at
# the top of the arms, DL-/DR- lower, slightly above MU and outside ML/MR.
# T1-T4 run across the top. Buttons are square so an assigned AUX-N
# function's designator can be drawn into them later.
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


def joy_led_centres(centres, button: int, led: int):
    """LED marks from the handgrip drawing, numbered top to bottom, left to
    right (chain order isn't known yet). The T-row LEDs sit in the gaps,
    flush with the buttons' bottom edge."""
    b = centres
    t_led_y = b["T1"][1] + button // 2 - led // 2
    return [
        (between(b["T1"], b["T2"])[0], t_led_y),
        (between(b["T2"], b["T3"])[0], t_led_y),
        (between(b["T3"], b["T4"])[0], t_led_y),
        between(b["DL+"], b["DL-"]), between(b["DR+"], b["DR-"]),
        between(b["MU"], b["MC"]),
        between(b["ML"], b["MC"]), between(b["MC"], b["MR"]),
        between(b["MC"], b["MD"]),          # the point of the V
    ]


class Layout:
    """Data mask geometry for one reference mask size. Texts the firmware
    rewrites at runtime keep their length in every layout; only static texts
    (repo URL) may differ."""

    def __init__(self, **values):
        self.__dict__.update(values)


FULL = Layout(
    name="full", mask=FULL_MASK,
    fonts={TEST_FONT: FONT_12X16, MASK_TITLE_FONT: FONT_16X16, MASK_SMALL_FONT: FONT_8X12},
    line_width=2,
    title=(0, 20, 480, 40), repo_url=(10, 452, 330, 20), commit=(390, 452, 80, 20),
    repo_url_text=REPO_URL_TEXT,
    joy_centres=JOY_BUTTON_CENTRES, joy_button=50, joy_led=12,
    page_ind=(9, 6, 11, 3),            # w, h, pitch, margin
    button=(220, 50, 5),               # w, h, label margin
    lights_title=(10, 10, 150, 24), lights_mode=(10, 50),
    lights_rows=(130, 50, 10, 5, 170),  # first y, pitch, label x, label dy, input x
    lights_label=(150, 24), input=(100, 34), frame_margin=3,
    settings_title=(10, 10, 150, 24),
    settings_buttons={BTN_PAGE_MODE: (10, 50), BTN_RESTART: (250, 50),
                      BTN_PAGE_COLOUR_BASE + 0: (10, 140), BTN_PAGE_COLOUR_BASE + 1: (250, 140),
                      BTN_PAGE_COLOUR_BASE + 2: (10, 200), BTN_PAGE_COLOUR_BASE + 3: (250, 200),
                      BTN_PRESS_FLASH: (10, 260)},
    restart_hint=(10, 108, 460, 24),
)

# 200x200 (e.g. AMATRON 3): everything in 6x8 except the title, one settings
# button per row, the handgrip shrunk to fit.
_COMPACT_SCALE = COMPACT_MASK / FULL_MASK
COMPACT = Layout(
    name="compact", mask=COMPACT_MASK,
    fonts={TEST_FONT: FONT_6X8, MASK_TITLE_FONT: FONT_8X12, MASK_SMALL_FONT: FONT_6X8},
    line_width=1,
    title=(0, 3, 200, 14), repo_url=(2, 190, 144, 8), commit=(150, 190, 48, 8),
    repo_url_text=b"gunicsba/AgIsoAuxInputs",
    joy_centres={label: (round(x * _COMPACT_SCALE), round(y * _COMPACT_SCALE))
                 for label, (x, y) in JOY_BUTTON_CENTRES.items()},
    joy_button=22, joy_led=6,
    page_ind=(4, 3, 5, 1),
    button=(192, 20, 3),
    lights_title=(4, 3, 100, 10), lights_mode=(4, 18),
    lights_rows=(48, 26, 4, 4, 110),
    lights_label=(100, 10), input=(60, 16), frame_margin=2,
    settings_title=(4, 3, 100, 10),
    settings_buttons={BTN_PAGE_MODE: (4, 16), BTN_RESTART: (4, 38),
                      BTN_PAGE_COLOUR_BASE + 0: (4, 72), BTN_PAGE_COLOUR_BASE + 1: (4, 94),
                      BTN_PAGE_COLOUR_BASE + 2: (4, 116), BTN_PAGE_COLOUR_BASE + 3: (4, 138),
                      BTN_PRESS_FLASH: (4, 160)},
    restart_hint=(4, 60, 192, 10),
)
LAYOUTS = (FULL, COMPACT)


# --- Pool building ------------------------------------------------------------

class PoolObject(bytes):
    """Object bytes plus what the firmware patches in it (offsets within the
    object)."""
    justify_upgrades = ()   # (offset, VT4 value)
    scale_fields = ()       # (offset, kind, square value, wide value)
    size = None             # (w, h) on a data mask
    mask_children = ()      # (child ID, x, y) of a data mask


class Pool(bytearray):
    """Object pool that collects its objects' patches at pool offsets, and the
    AUX-N objects a VT2 doesn't know."""

    def __init__(self):
        super().__init__()
        self.justify_upgrades = []   # (offset, VT4 value)
        self.vt3_objects = []        # (offset, object length)
        self.scale_fields = []       # (offset, kind, square value, wide value)
        self.sizes = {}              # object ID -> (w, h), for check_mask_bounds()
        self.placements = []         # (mask ID, child ID, x, y)

    def __iadd__(self, obj):
        base = len(self)
        object_id = obj[0] | (obj[1] << 8)
        if getattr(obj, "size", None):
            self.sizes[object_id] = obj.size
        self.placements += [(object_id, *child) for child in getattr(obj, "mask_children", ())]
        self.justify_upgrades += [(base + o, v) for o, v in getattr(obj, "justify_upgrades", ())]
        self.scale_fields += [(base + o, k, s, w) for o, k, s, w in getattr(obj, "scale_fields", ())]
        if obj[2] in VT3_OBJECT_TYPES:
            assert len(obj) >= STRING_VARIABLE_HEADER, "too short to blank out"
            self.vt3_objects.append((base, len(obj)))
        return super().__iadd__(obj)


class Obj:
    """Builds one object, recording the fields the firmware patches."""

    def __init__(self, object_id: int, object_type: int):
        self.data = bytearray()
        self.justify = []
        self.fields = []
        self.size = None
        self.mask_children = []
        self.u16(object_id)
        self.u8(object_type)

    def u8(self, v):
        self.data += struct.pack("<B", v & 0xFF)

    def u16(self, v):
        self.data += struct.pack("<H", v & 0xFFFF)

    def i16(self, v):
        self.data += struct.pack("<h", v)

    def raw(self, b: bytes):
        self.data += b

    def scaled(self, kind: int, value, wide=None):
        """A field the firmware scales. Key fields take the square value and
        the wide one; the wide one goes into the pool bytes."""
        if wide is None:
            wide = value
        self.fields.append((len(self.data), kind, value, wide))
        self.data += struct.pack(SCALE_KIND_FORMAT[kind], wide)

    def mask_xy(self, x: int, y: int):
        self.scaled(SCALE_MASK_POS, x)
        self.scaled(SCALE_MASK_POS, y)

    def mask_wh(self, w: int, h: int):
        self.size = (w, h)
        self.scaled(SCALE_MASK_SIZE, w)
        self.scaled(SCALE_MASK_SIZE, h)

    def key_xy(self, box):
        (xs, ys, _, _), (xw, yw, _, _) = box
        self.scaled(SCALE_KEY_X, xs, xw)
        self.scaled(SCALE_KEY_Y, ys, yw)

    def key_wh(self, box):
        (_, _, ws, hs), (_, _, ww, hw) = box
        self.scaled(SCALE_KEY_W, ws, ww)
        self.scaled(SCALE_KEY_H, hs, hw)

    def justify_byte(self, justify: int):
        """VT3-safe justify; the full value is patched in for VT4+."""
        safe = justify & JUSTIFY_HORIZONTAL_MASK
        if safe != justify:
            self.justify.append((len(self.data), justify))
        self.u8(safe)

    def done(self) -> PoolObject:
        obj = PoolObject(bytes(self.data))
        obj.justify_upgrades = tuple(self.justify)
        obj.scale_fields = tuple(self.fields)
        obj.size = self.size
        obj.mask_children = tuple(self.mask_children)
        return obj


def check_text_fits(text: bytes, font_enum: int, w: int, h: int, what: str):
    fw, fh = FONT_SIZES[font_enum]
    assert len(text) * fw <= w and fh <= h, (
        f"{what}: {text!r} in {FONT_SIZES[font_enum]} doesn't fit {w}x{h}")


def check_key_box(box, what: str):
    for (x, y, w, h), (kw, kh) in zip(box, (KEY_SQUARE, KEY_WIDE)):
        assert x >= 0 and y >= 0 and x + w <= kw and y + h <= kh, (
            f"{what}: {(x, y, w, h)} outside the {kw}x{kh} designator")


def build_working_set():
    # header: ID(2) type(1) bg(1) selectable(1) activeMask(2) nChildren(1) nMacros(1) nLanguages(1) = 10 bytes
    o = Obj(WORKING_SET, TYPE_WORKING_SET)
    o.u8(COLOUR_WHITE)   # background colour
    o.u8(1)              # selectable: the operator can open our masks
    o.u16(MAIN_MASK)     # active mask
    o.u8(1)              # 1 child: the designator shown in the VT's WS list
    o.u8(0)              # 0 macros
    o.u8(0)              # 0 languages
    o.u16(WS_DESIGNATOR)
    o.key_xy(DESIGNATOR_BOX)
    return o.done()


def build_data_mask(object_id: int, soft_key_mask_id: int, children):
    # header: ID(2) type(1) bg(1) softKeyMask(2) nChildren(1) nMacros(1) = 8 bytes
    o = Obj(object_id, TYPE_DATA_MASK)
    o.u8(COLOUR_WHITE)         # background colour
    o.u16(soft_key_mask_id)
    o.u8(len(children))
    o.u8(0)                    # 0 macros
    for child_id, x, y in children:
        o.u16(child_id)
        o.mask_xy(x, y)
    o.mask_children = list(children)
    return o.done()


def build_soft_key_mask(object_id: int, key_ids):
    # header: ID(2) type(1) bg(1) nObjects(1) nMacros(1) = 6 bytes, then one
    # object ID per key (keys are placed in order, no x/y).
    o = Obj(object_id, TYPE_SOFT_KEY_MASK)
    o.u8(COLOUR_SILVER)        # background colour
    o.u8(len(key_ids))
    o.u8(0)                    # 0 macros
    for key_id in key_ids:
        o.u16(key_id)
    return o.done()


def build_key(object_id: int, children):
    # header: ID(2) type(1) bg(1) keyCode(1) nObjects(1) nMacros(1) = 7 bytes,
    # then (objectID, x, y) per child inside the soft key designator.
    o = Obj(object_id, TYPE_KEY)
    o.u8(COLOUR_SILVER)        # background colour
    o.u8(object_id & 0xFF)     # key code (unused - firmware matches on object ID)
    o.u8(len(children))
    o.u8(0)                    # 0 macros
    for child_id, box in children:
        o.u16(child_id)
        o.key_xy(box)
    return o.done()


def build_line_attributes(object_id: int, colour: int, width: int):
    # ID(2) type(1) colour(1) width(1) lineArt(2) nMacros(1) = 8 bytes
    o = Obj(object_id, TYPE_LINE_ATTRIBUTES)
    o.u8(colour)
    o.scaled(SCALE_MASK_LINE, width)
    o.u16(0xFFFF)              # solid line
    o.u8(0)                    # 0 macros
    return o.done()


def build_fill_attributes(object_id: int, colour: int):
    # ID(2) type(1) fillType(1) colour(1) pattern(2) nMacros(1) = 8 bytes
    o = Obj(object_id, TYPE_FILL_ATTRIBUTES)
    o.u8(2)                    # fill with the colour below
    o.u8(colour)
    o.u16(NULL_OBJECT_ID)      # no pattern
    o.u8(0)                    # 0 macros
    return o.done()


def build_output_rectangle(object_id: int, line_id: int, width: int, height: int,
                           fill_id: int = NULL_OBJECT_ID):
    # ID(2) type(1) lineAttr(2) width(2) height(2) lineSuppression(1)
    # fillAttr(2) nMacros(1) = 13 bytes. No fill attributes = outline only.
    o = Obj(object_id, TYPE_OUTPUT_RECTANGLE)
    o.u16(line_id)
    o.mask_wh(width, height)
    o.u8(0)                    # draw all four sides
    o.u16(fill_id)
    o.u8(0)                    # 0 macros
    return o.done()


def build_button(object_id: int, label_id: int, layout: Layout):
    # header: ID(2) type(1) width(2) height(2) bg(1) border(1) keyCode(1)
    # options(1) nChildren(1) nMacros(1) = 13 bytes, then (objectID, x, y)
    # per child. Options 0 = momentary, VT-drawn border, enabled.
    w, h, margin = layout.button
    o = Obj(object_id, TYPE_BUTTON)
    o.mask_wh(w, h)
    o.u8(COLOUR_SILVER)        # background colour
    o.u8(COLOUR_BLACK)         # border colour
    o.u8(object_id & 0xFF)     # key code (unused - firmware matches on object ID)
    o.u8(0)                    # options
    o.u8(1)                    # 1 child: the label string
    o.u8(0)                    # 0 macros
    # Label fills the button face minus a margin for the VT's border.
    o.u16(label_id)
    o.mask_xy(margin, margin)
    return o.done()


def build_button_label(object_id: int, text: bytes, layout: Layout):
    w, h, margin = layout.button
    return build_output_string(object_id, w - 2 * margin, h - 2 * margin, TEST_FONT,
                               COLOUR_SILVER, text, layout, justify=JUSTIFY_CENTERED,
                               transparent=True)


def build_input_number(object_id: int, value: int, maximum: int, layout: Layout):
    # ID(2) type(1) width(2) height(2) bg(1) font(2) options(1) varRef(2)
    # value(4) min(4) max(4) offset(4) scale(float 4) decimals(1) format(1)
    # justify(1) options2(1) nMacros(1) = 38 bytes
    w, h = layout.input
    check_text_fits(b"255", layout.fonts[TEST_FONT], w, h, f"{layout.name} input {object_id}")
    o = Obj(object_id, TYPE_INPUT_NUMBER)
    o.mask_wh(w, h)
    o.u8(COLOUR_YELLOW)        # background colour: marks it editable
    o.u16(TEST_FONT)           # font attributes object
    o.u8(0)                    # options
    o.u16(NULL_OBJECT_ID)      # no number variable -> inline value
    o.raw(struct.pack("<I", value))
    o.raw(struct.pack("<I", 0))     # minimum
    o.raw(struct.pack("<I", maximum))
    o.raw(struct.pack("<i", 0))     # offset
    o.raw(struct.pack("<f", 1.0))   # scale
    o.u8(0)                    # 0 decimals
    o.u8(0)                    # fixed (not exponential) format
    o.justify_byte(JUSTIFY_RIGHT_MIDDLE)
    o.u8(1)                    # options2: enabled, no real-time editing
    o.u8(0)                    # 0 macros
    return o.done()


def build_output_string(object_id: int, width: int, height: int, font_id: int,
                        bg_colour: int, text: bytes, layout: Layout, justify: int = 0,
                        transparent: bool = False):
    """An Output String on a data mask."""
    check_text_fits(text, layout.fonts[font_id], width, height,
                    f"{layout.name} string {object_id}")
    o = Obj(object_id, TYPE_OUTPUT_STRING)
    o.mask_wh(width, height)
    return finish_output_string(o, font_id, bg_colour, text, justify, transparent)


def build_key_string(object_id: int, box, font_id: int, bg_colour: int, text: bytes,
                     transparent: bool = False):
    """An Output String in a soft key designator; box = (square, wide)."""
    check_key_box(box, f"key string {object_id}")
    for _, _, w, h in box:
        check_text_fits(text, KEY_FONTS[font_id], w, h, f"key string {object_id}")
    o = Obj(object_id, TYPE_OUTPUT_STRING)
    o.key_wh(box)
    return finish_output_string(o, font_id, bg_colour, text, JUSTIFY_CENTERED, transparent)


def finish_output_string(o: Obj, font_id: int, bg_colour: int, text: bytes, justify: int,
                         transparent: bool):
    # ID(2) type(1) width(2) height(2) bg(1) font(2) options(1) varRef(2)
    # justify(1) strLen(2) = 16 bytes, then the text and nMacros(1)
    o.u8(bg_colour)
    o.u16(font_id)             # font attributes object
    o.u8(1 if transparent else 0)  # options bit 0: transparent
    o.u16(NULL_OBJECT_ID)      # no string variable -> use inline value below
    o.justify_byte(justify)
    o.u16(len(text))
    o.raw(text)
    o.u8(0)                    # 0 macros
    return o.done()


def build_font_attributes(object_id: int, size_enum: int, kind: int):
    # ID(2) type(1) colour(1) size(1) type(1) style(1) nMacros(1) = 8 bytes
    o = Obj(object_id, TYPE_FONT_ATTRIBUTES)
    o.u8(0)                    # colour (black)
    o.scaled(kind, size_enum)
    o.u8(0)                    # FontType::ISO8859_1
    o.u8(0)                    # style bitfield: none
    o.u8(0)                    # 0 macros
    return o.done()


def build_aux_input_type2(object_id: int, function_type: int, designator_id: int):
    # header: ID(2) type(1) bg(1) bitfield(1) nObjects(1) = 6 bytes, then 1
    # designator child: (objectID, x, y) = 6 bytes.
    o = Obj(object_id, TYPE_AUXILIARY_INPUT_TYPE_2)
    o.u8(0)                    # background colour (not rendered)
    o.u8(function_type & 0x1F)  # bits 5-7 = 0: not critical, not single-assignment
    o.u8(1)                    # 1 linked object: the designator
    o.u16(designator_id)
    o.key_xy(DESIGNATOR_BOX)
    return o.done()


def build_aux_designator_pointer(object_id: int, aux_object_id: int):
    # ID(2) type(1) pointerType(1) auxObjectID(2) = 6 bytes, no macros
    o = Obj(object_id, TYPE_AUXILIARY_CONTROL_DESIGNATOR_TYPE_2)
    o.u8(POINTER_TYPE_ASSIGNED_FUNCTION)
    o.u16(aux_object_id)
    return o.done()


def build_filler(length: int):
    # String Variable: ID(2) type(1) length(2) value
    o = Obj(FILLER_OBJECT, TYPE_STRING_VARIABLE)
    o.u16(length - STRING_VARIABLE_HEADER)
    o.raw(b" " * (length - STRING_VARIABLE_HEADER))
    return o.done()


def build_designator(object_id: int, font_id: int, bg_colour: int, text: bytes):
    return build_key_string(object_id, DESIGNATOR_BOX, font_id, bg_colour, text)


def build_key_label(object_id: int, box, font_id: int, text: bytes):
    return build_key_string(object_id, box, font_id, COLOUR_SILVER, text, transparent=True)


PAGED_LABELS = [l for l in JOY_BUTTON_CENTRES if l not in PAGE_SELECT_LABELS]
PAGED_BITS = [BUTTON_LABELS.index(l) for l in PAGED_LABELS]
PAGE_SELECT_BITS = [BUTTON_LABELS.index(l) for l in PAGE_SELECT_LABELS]


ALL_PAGES_LAYER = NUM_PAGES


def paged_aux_id(bit: int, layer: int):
    """AUX input for a paged button on page 0..3, or ALL_PAGES_LAYER ("*")."""
    if layer == ALL_PAGES_LAYER:
        return AUX_BUTTON_BASE + bit      # same ID as in single mode
    return AUX_PAGED_BASE + bit * NUM_PAGES + layer


def paged_designator_id(bit: int, page: int):
    return DESIGNATOR_PAGED_BASE + bit * NUM_PAGES + page


def paged_label(label: str, layer: int):
    return label + ("*" if layer == NUM_PAGES else str(layer + 1))


def build_pool(multi: bool, L: Layout):
    pool = Pool()
    pool += build_working_set()
    button = L.joy_button
    ind_w, ind_h, ind_pitch, ind_margin = L.page_ind
    main_children = [
        (STR_MAIN_TITLE, *L.title[:2]),
        (STR_REPO_URL, *L.repo_url[:2]),
        (STR_COMMIT, *L.commit[:2]),
    ]
    for label, (cx, cy) in L.joy_centres.items():
        bit = BUTTON_LABELS.index(label)
        x, y = cx - button // 2, cy - button // 2
        main_children += [
            (JOY_BUTTON_RECT_BASE + bit, x, y),
            (JOY_BUTTON_LABEL_BASE + bit, x, y),
        ]
        if multi and label in PAGE_SELECT_LABELS:
            continue  # page selectors: no AUX input, no designator
        # Drawn after the label: once a function is assigned, its designator
        # covers the label.
        if SHOW_AUX_POINTERS:
            main_children.append((AUX_POINTER_BASE + bit, x, y))
        if multi:
            for page in range(NUM_PAGES):
                main_children.append((
                    PAGE_IND_RECT_BASE + bit * NUM_PAGES + page,
                    x + ind_margin + page * ind_pitch,
                    y + button - ind_h - ind_margin))
    led_centres = joy_led_centres(L.joy_centres, button, L.joy_led)
    for led, (cx, cy) in enumerate(led_centres):
        main_children.append((JOY_LED_RECT_BASE + led, cx - L.joy_led // 2,
                              cy - L.joy_led // 2))
    pool += build_data_mask(MAIN_MASK, MAIN_SOFT_KEY_MASK, main_children)

    first_y, pitch, label_x, label_dy, input_x = L.lights_rows
    margin = L.frame_margin
    lights_children = [
        (STR_LIGHTS_TITLE, *L.lights_title[:2]),
        (BTN_LIGHTS_MODE, *L.lights_mode),
    ]
    for row, (label_id, input_id) in enumerate([
            (STR_RED, INPUT_RED), (STR_GREEN, INPUT_GREEN),
            (STR_BLUE, INPUT_BLUE), (STR_BRIGHTNESS, INPUT_BRIGHTNESS)]):
        y = first_y + row * pitch
        lights_children += [
            (label_id, label_x, y + label_dy),
            # Frame first so the input is drawn on top of it.
            (INPUT_FRAME, input_x - margin, y - margin),
            (input_id, input_x, y),
        ]
    pool += build_data_mask(LIGHTS_MASK, LIGHTS_SOFT_KEY_MASK, lights_children)
    pool += build_data_mask(SETTINGS_MASK, SETTINGS_SOFT_KEY_MASK, [
        (STR_SETTINGS_TITLE, *L.settings_title[:2]),
        (STR_RESTART_HINT, *L.restart_hint[:2]),
        *((button_id, x, y) for button_id, (x, y) in L.settings_buttons.items()),
    ])
    pool += build_soft_key_mask(MAIN_SOFT_KEY_MASK,
                                [KEY_LIGHTS_TOGGLE, KEY_LIGHTS_PAGE, KEY_SETTINGS])
    pool += build_soft_key_mask(LIGHTS_SOFT_KEY_MASK,
                                [KEY_LIGHTS_TOGGLE, KEY_BACK])
    pool += build_soft_key_mask(SETTINGS_SOFT_KEY_MASK, [KEY_BACK])
    for font_id, size in L.fonts.items():
        pool += build_font_attributes(font_id, size, SCALE_MASK_FONT)
    for font_id, size in KEY_FONTS.items():
        pool += build_font_attributes(font_id, size, SCALE_KEY_FONT)
    pool += build_designator(WS_DESIGNATOR, DESIGNATOR_FONT, COLOUR_WHITE, b"JOY")

    for oid, did, label in zip(AUX_ANALOG_IDS, DESIGNATOR_ANALOG_IDS, ANALOG_LABELS):
        pool += build_designator(did, DESIGNATOR_FONT, COLOUR_WHITE, label.encode("ascii"))
        pool += build_aux_input_type2(oid, FUNCTION_TYPE_ANALOGUE_MAINTAINS_POSITION, did)

    # The single-mode inputs exist in both pools: in multi mode a paged
    # button's "*" input reuses its single-mode ID, and T1-T4/spares keep
    # theirs (never pressed). A VT or implement restoring a single-mode
    # assignment then always finds the input - AgIsoVirtualTerminal hangs on
    # one that doesn't exist (Open-Agriculture/AgIsoVirtualTerminal#252) - and
    # an assignment made in single mode carries over as "all pages".
    for oid, did, label in zip(AUX_BUTTON_IDS, DESIGNATOR_BUTTON_IDS, BUTTON_LABELS):
        if multi and label in PAGED_LABELS:
            label = paged_label(label, ALL_PAGES_LAYER)   # "MC*", "DL+*"
            font = PAGED_DESIGNATOR_FONT
        else:
            font = DESIGNATOR_FONT
        pool += build_designator(did, font, COLOUR_WHITE, label.encode("ascii"))
        pool += build_aux_input_type2(oid, FUNCTION_TYPE_BOOLEAN_MOMENTARY, did)
    for label in JOY_BUTTON_CENTRES:
        if multi and label in PAGE_SELECT_LABELS:
            continue
        bit = BUTTON_LABELS.index(label)
        # In multi mode this starts on "*"; the firmware retargets it per page.
        pool += build_aux_designator_pointer(AUX_POINTER_BASE + bit,
                                             AUX_BUTTON_BASE + bit)

    if multi:
        for bit, label in zip(PAGED_BITS, PAGED_LABELS):
            for page in range(NUM_PAGES):
                pool += build_designator(
                    paged_designator_id(bit, page), PAGED_DESIGNATOR_FONT, COLOUR_WHITE,
                    paged_label(label, page).encode("ascii"))
                pool += build_aux_input_type2(paged_aux_id(bit, page),
                                              FUNCTION_TYPE_BOOLEAN_MOMENTARY,
                                              paged_designator_id(bit, page))
                fill_id = PAGE_IND_FILL_BASE + bit * NUM_PAGES + page
                pool += build_fill_attributes(fill_id, COLOUR_WHITE)
                pool += build_output_rectangle(
                    PAGE_IND_RECT_BASE + bit * NUM_PAGES + page, JOY_THIN_LINE,
                    ind_w, ind_h, fill_id)
        pool += build_line_attributes(JOY_THIN_LINE, COLOUR_BLACK, 1)

    # Settings page
    pool += build_output_string(STR_SETTINGS_TITLE, *L.settings_title[2:], TEST_FONT,
                                COLOUR_WHITE, b"SETTINGS", L)
    for button_id, label_id, text in [
            (BTN_PAGE_MODE, STR_PAGE_MODE, PAGE_MODE_TEXT),
            (BTN_RESTART, STR_RESTART, b"RESTART"),
            (BTN_PRESS_FLASH, STR_PRESS_FLASH, PRESS_FLASH_TEXT),
            *[(BTN_PAGE_COLOUR_BASE + p, STR_PAGE_COLOUR_BASE + p, PAGE_COLOUR_TEXT)
              for p in range(NUM_PAGES)]]:
        pool += build_button(button_id, label_id, L)
        pool += build_button_label(label_id, text, L)
    pool += build_output_string(STR_RESTART_HINT, *L.restart_hint[2:], TEST_FONT,
                                COLOUR_WHITE, RESTART_HINT_TEXT, L)
    pool += build_key(KEY_SETTINGS, [(STR_KEY_SETTINGS, KEY_LABEL_BOX)])
    pool += build_key_label(STR_KEY_SETTINGS, KEY_LABEL_BOX, KEY_FONT, b"SETUP")

    pool += build_output_string(STR_MAIN_TITLE, *L.title[2:], MASK_TITLE_FONT,
                                COLOUR_WHITE, MAIN_TITLE_TEXT, L, justify=JUSTIFY_CENTERED)
    pool += build_output_string(STR_REPO_URL, *L.repo_url[2:], MASK_SMALL_FONT,
                                COLOUR_WHITE, L.repo_url_text, L)
    pool += build_output_string(STR_COMMIT, *L.commit[2:], MASK_SMALL_FONT, COLOUR_WHITE,
                                COMMIT_PLACEHOLDER, L, justify=JUSTIFY_RIGHT_MIDDLE)

    # Handgrip picture
    pool += build_line_attributes(JOY_LINE, COLOUR_BLACK, L.line_width)
    for label in JOY_BUTTON_CENTRES:
        bit = BUTTON_LABELS.index(label)
        pool += build_fill_attributes(JOY_BUTTON_FILL_BASE + bit, COLOUR_WHITE)
        pool += build_output_rectangle(JOY_BUTTON_RECT_BASE + bit, JOY_LINE,
                                       button, button, JOY_BUTTON_FILL_BASE + bit)
        pool += build_output_string(JOY_BUTTON_LABEL_BASE + bit, button, button, TEST_FONT,
                                    COLOUR_WHITE, label.encode("ascii"), L,
                                    justify=JUSTIFY_CENTERED, transparent=True)
    for led in range(len(led_centres)):
        pool += build_fill_attributes(JOY_LED_FILL_BASE + led, COLOUR_SILVER)
        pool += build_output_rectangle(JOY_LED_RECT_BASE + led, JOY_LINE,
                                       L.joy_led, L.joy_led, JOY_LED_FILL_BASE + led)

    # Soft keys. The toggle key shows "LIGHT" over the current state.
    pool += build_key(KEY_LIGHTS_TOGGLE, [
        (STR_KEY_TOGGLE_TITLE, KEY_TOGGLE_TITLE_BOX),
        (STR_KEY_TOGGLE_STATE, KEY_TOGGLE_STATE_BOX),
    ])
    pool += build_key_label(STR_KEY_TOGGLE_TITLE, KEY_TOGGLE_TITLE_BOX, KEY_FONT, b"LIGHT")
    pool += build_key_label(STR_KEY_TOGGLE_STATE, KEY_TOGGLE_STATE_BOX, DESIGNATOR_FONT,
                            TOGGLE_STATE_TEXT)
    pool += build_key(KEY_LIGHTS_PAGE, [(STR_KEY_PAGE, KEY_LABEL_BOX)])
    pool += build_key_label(STR_KEY_PAGE, KEY_LABEL_BOX, KEY_FONT, b"LIGHTS")
    pool += build_key(KEY_BACK, [(STR_KEY_BACK, KEY_LABEL_BOX)])
    pool += build_key_label(STR_KEY_BACK, KEY_LABEL_BOX, KEY_FONT, b"BACK")

    # Lights page
    pool += build_button(BTN_LIGHTS_MODE, STR_LIGHTS_MODE, L)
    pool += build_button_label(STR_LIGHTS_MODE, MODE_TEXT, L)
    pool += build_line_attributes(INPUT_FRAME_LINE, COLOUR_BLACK, L.line_width)
    input_w, input_h = L.input
    pool += build_output_rectangle(INPUT_FRAME, INPUT_FRAME_LINE,
                                   input_w + 2 * margin, input_h + 2 * margin)
    pool += build_output_string(STR_LIGHTS_TITLE, *L.lights_title[2:], TEST_FONT,
                                COLOUR_WHITE, b"LIGHTS", L)
    for string_id, text in [
            (STR_RED, b"RED"),
            (STR_GREEN, b"GREEN"),
            (STR_BLUE, b"BLUE"),
            (STR_BRIGHTNESS, b"BRIGHTNESS %")]:
        pool += build_output_string(string_id, *L.lights_label, TEST_FONT,
                                    COLOUR_WHITE, text, L)
    pool += build_input_number(INPUT_RED, 255, 255, L)
    pool += build_input_number(INPUT_GREEN, 255, 255, L)
    pool += build_input_number(INPUT_BLUE, 255, 255, L)
    pool += build_input_number(INPUT_BRIGHTNESS, 50, 100, L)
    check_mask_bounds(pool, L)
    return pool


def check_mask_bounds(pool: Pool, L: Layout):
    """Every data mask child has to sit inside the reference mask."""
    for mask_id, child_id, x, y in pool.placements:
        w, h = pool.sizes.get(child_id, (0, 0))
        assert 0 <= x and 0 <= y and x + w <= L.mask and y + h <= L.mask, (
            f"{L.name}: object {child_id} at {x},{y} ({w}x{h}) outside mask {mask_id}")


def build_layouts(multi: bool):
    """Both layouts of a mode, padded to the same size."""
    pools = [build_pool(multi, layout) for layout in LAYOUTS]
    target = max(len(p) for p in pools) + STRING_VARIABLE_HEADER + 1
    for pool in pools:
        pool += build_filler(target - len(pool))
    return pools


def c_array(name: str, pool: bytes):
    lines = [f"static const uint8_t {name}[] = {{"]
    for i in range(0, len(pool), 16):
        lines.append("    " + ", ".join(f"0x{b:02X}" for b in pool[i:i + 16]) + ",")
    return lines + ["};", ""]


def main():
    modes = {"single": build_layouts(multi=False), "multi": build_layouts(multi=True)}

    # A layout change in a builder must fail here, not patch a wrong byte at
    # runtime.
    for pools in modes.values():
        for pool in pools:
            for offset, value in pool.justify_upgrades:
                assert pool[offset] == value & JUSTIFY_HORIZONTAL_MASK, (
                    f"offset {offset} is not a justify byte (0x{pool[offset]:02X})")
            for offset, kind, _, wide in pool.scale_fields:
                fmt = SCALE_KIND_FORMAT[kind]
                assert struct.unpack_from(fmt, pool, offset)[0] == wide, offset

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, pools in modes.items():
        for layout, pool in zip(LAYOUTS, pools):
            iop_path = OUT_DIR / f"aux_n_pool_{name}_{layout.name}.iop"
            iop_path.write_bytes(bytes(pool))
            print(f"Wrote {iop_path} ({len(pool)} bytes)")

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
        f"#define KEY_BACK {KEY_BACK}",
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
        "// Settings mask",
        f"#define SETTINGS_MASK {SETTINGS_MASK}",
        f"#define KEY_SETTINGS {KEY_SETTINGS}",
        f"#define BTN_PAGE_MODE {BTN_PAGE_MODE}",
        f"#define STR_PAGE_MODE {STR_PAGE_MODE}",
        f"#define BTN_RESTART {BTN_RESTART}",
        f"#define STR_RESTART_HINT {STR_RESTART_HINT}",
        f"#define BTN_PAGE_COLOUR_BASE {BTN_PAGE_COLOUR_BASE}",
        f"#define STR_PAGE_COLOUR_BASE {STR_PAGE_COLOUR_BASE}",
        f"#define BTN_PRESS_FLASH {BTN_PRESS_FLASH}",
        f"#define STR_PRESS_FLASH {STR_PRESS_FLASH}",
        f"#define PAGE_MODE_TEXT_LENGTH {len(PAGE_MODE_TEXT)}",
        f"#define RESTART_HINT_TEXT_LENGTH {len(RESTART_HINT_TEXT)}",
        f"#define PAGE_COLOUR_TEXT_LENGTH {len(PAGE_COLOUR_TEXT)}",
        f"#define PRESS_FLASH_TEXT_LENGTH {len(PRESS_FLASH_TEXT)}",
        "",
        "// Single / multi-page mode",
        f"#define MODEL_ID_SINGLE {MODEL_ID_SINGLE}",
        f"#define MODEL_ID_MULTI {MODEL_ID_MULTI}",
        f"#define AUX_POINTER_BASE {AUX_POINTER_BASE}",
        f"#define NUM_PAGES {NUM_PAGES}",
        f"#define PAGED_LAYERS {PAGED_LAYERS}",
        f"#define PAGE_IND_FILL_BASE {PAGE_IND_FILL_BASE}",
        f"#define NUM_PAGED_BUTTONS {len(PAGED_BITS)}",
        "// PCA9555 bits of the buttons that have per-page inputs in multi mode",
        "static const uint8_t PAGED_BUTTON_BITS[NUM_PAGED_BUTTONS] = {"
        + ", ".join(str(b) for b in PAGED_BITS) + "};",
        "// Multi mode AUX input per paged button: pages 1..4, then \"*\" (which",
        "// reuses the button's single-mode ID from AUX_BUTTON_IDS)",
        "static const uint16_t PAGED_AUX_IDS[NUM_PAGED_BUTTONS][PAGED_LAYERS] = {",
        *["    {" + ", ".join(str(paged_aux_id(b, l)) for l in range(PAGED_LAYERS)) + "},"
          for b in PAGED_BITS],
        "};",
        "// PCA9555 bits of T1..T4, which select page 1..4 in multi mode",
        "static const uint8_t PAGE_SELECT_BITS[NUM_PAGES] = {"
        + ", ".join(str(b) for b in PAGE_SELECT_BITS) + "};",
        "",
    ]
    header_path.write_text("\n".join(lines), encoding="ascii")

    # Embedded as plain C byte arrays (not linker-embedded binaries): far
    # simpler and more portable across PlatformIO's various framework/build
    # backends than relying on board_build.embed_files's linker-symbol
    # mechanism, which turned out to be fragile under the mixed
    # arduino+espidf framework this firmware uses.
    data_header_path = OUT_DIR / "aux_n_pool_data.h"
    data_lines = [
        "// Auto-generated by tools/build_aux_pool.py - do not edit by hand.",
        "// The object pools, one per page mode and layout, and what the firmware",
        "// patches in them for the connected VT (see src/object_pool_adapt.cpp).",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "// Justify byte shipped VT3-safe (horizontal only), with the full value",
        "// to patch in once the VT reports version 4+",
        "struct AuxPoolJustifyUpgrade {",
        "  uint16_t offset;",
        "  uint8_t value;",
        "};",
        "",
        "// AUX-N (VT3+) object, rewritten into a String Variable for a VT2",
        "struct AuxPoolVt3Object {",
        "  uint16_t offset;",
        "  uint16_t length;",
        "};",
        "",
        "// A field scaled to the VT's data mask or soft key designator: its value",
        "// at the layout's reference size, for square and for wide soft keys",
        "struct AuxPoolScaleField {",
        "  uint16_t offset;",
        "  uint8_t kind;",
        "  int16_t square;",
        "  int16_t wide;",
        "};",
        "",
        "struct AuxPoolLayout {",
        "  const char *name;",
        "  uint16_t maskSize;",
        "  const uint8_t *data;",
        "  uint32_t size;",
        "  const AuxPoolJustifyUpgrade *justify;",
        "  uint32_t justifyCount;",
        "  const AuxPoolVt3Object *vt3Objects;",
        "  uint32_t vt3ObjectCount;",
        "  const AuxPoolScaleField *scale;",
        "  uint32_t scaleCount;",
        "};",
        "",
        "enum AuxPoolScaleKind : uint8_t {",
        f"  AUX_SCALE_MASK_POS = {SCALE_MASK_POS},",
        f"  AUX_SCALE_MASK_SIZE = {SCALE_MASK_SIZE},",
        f"  AUX_SCALE_MASK_LINE = {SCALE_MASK_LINE},",
        f"  AUX_SCALE_MASK_FONT = {SCALE_MASK_FONT},",
        f"  AUX_SCALE_KEY_X = {SCALE_KEY_X},",
        f"  AUX_SCALE_KEY_Y = {SCALE_KEY_Y},",
        f"  AUX_SCALE_KEY_W = {SCALE_KEY_W},",
        f"  AUX_SCALE_KEY_H = {SCALE_KEY_H},",
        f"  AUX_SCALE_KEY_FONT = {SCALE_KEY_FONT},",
        "};",
        "",
        f"#define AUX_POOL_COMPACT_BELOW_MASK {COMPACT_BELOW_MASK}",
        f"#define AUX_POOL_KEY_SQUARE_W {KEY_SQUARE[0]}",
        f"#define AUX_POOL_KEY_SQUARE_H {KEY_SQUARE[1]}",
        f"#define AUX_POOL_KEY_WIDE_W {KEY_WIDE[0]}",
        f"#define AUX_POOL_KEY_WIDE_H {KEY_WIDE[1]}",
        f"#define AUX_POOL_TYPE_STRING_VARIABLE {TYPE_STRING_VARIABLE}",
        f"#define AUX_POOL_STRING_VARIABLE_HEADER {STRING_VARIABLE_HEADER}",
        "",
        "// ISO 11783-6 font size enum -> width, height",
        "static const uint8_t AUX_POOL_FONT_SIZES[][2] = {"
        + ", ".join(f"{{{w}, {h}}}" for w, h in FONT_SIZES) + "};",
        "",
    ]
    layout_rows = {}
    for name, pools in modes.items():
        for layout, pool in zip(LAYOUTS, pools):
            prefix = f"AUX_N_POOL_{name.upper()}_{layout.name.upper()}"
            data_lines += c_array(f"{prefix}_DATA", pool)
            data_lines += [
                f"static const AuxPoolJustifyUpgrade {prefix}_JUSTIFY[] = {{",
                *[f"    {{{o}, 0x{v:02X}}}," for o, v in pool.justify_upgrades],
                "};",
                f"static const AuxPoolVt3Object {prefix}_VT3_OBJECTS[] = {{",
                *[f"    {{{o}, {n}}}," for o, n in pool.vt3_objects],
                "};",
                f"static const AuxPoolScaleField {prefix}_SCALE[] = {{",
                *[f"    {{{o}, {k}, {s}, {w}}}," for o, k, s, w in pool.scale_fields],
                "};",
                "",
            ]
            layout_rows.setdefault(name, []).append(
                f"    {{\"{layout.name}\", {layout.mask}, {prefix}_DATA, {len(pool)}, "
                f"{prefix}_JUSTIFY, {len(pool.justify_upgrades)}, "
                f"{prefix}_VT3_OBJECTS, {len(pool.vt3_objects)}, "
                f"{prefix}_SCALE, {len(pool.scale_fields)}}},")
    data_lines += [
        "// [multi-page][compact]; both layouts of a mode have the same size",
        "static const AuxPoolLayout AUX_N_POOL_LAYOUTS[2][2] = {",
        "  {", *layout_rows["single"], "  },",
        "  {", *layout_rows["multi"], "  },",
        "};",
        f"#define AUX_N_POOL_MAX_SIZE {max(len(p) for pools in modes.values() for p in pools)}",
        "",
    ]
    data_header_path.write_text("\n".join(data_lines), encoding="ascii")

    print(f"Wrote {header_path}")
    print(f"Wrote {data_header_path}")


if __name__ == "__main__":
    main()
