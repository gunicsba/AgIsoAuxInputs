"""Applies src/object_pool_adapt.cpp's scaling to the built pools for a set
of VT profiles, parses the result and checks that every text fits its box
and every child sits inside its parent (mask, button, soft key designator).
Run after changing a layout in build_aux_pool.py:
    python tools/check_pool_scaling.py
Keep scale() in step with the firmware."""
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import build_aux_pool as b  # noqa: E402

PROFILES = [  # name, mask, key w, key h
    ("AgIsoVT", 480, 60, 60),
    ("Mueller BASIC", 400, 120, 80),
    ("AMATRON 3", 200, 60, 47),
    ("240 mask / 60x32 keys", 240, 60, 32),
    ("320 mask", 320, 64, 48),
    ("800 mask / 80x80 keys", 800, 80, 80),
]
ALL_FONTS = (1 << 15) - 1
EPS = 0.001


def best_font(w, h):
    for size in range(14, 0, -1):
        fw, fh = b.FONT_SIZES[size]
        if fw <= w + EPS and fh <= h + EPS:
            return size
    return 0


def scale(pool, layout, mask, kw, kh):
    data = bytearray(pool)
    m = mask / layout.mask
    square = kh * 4 >= kw * 3
    rw, rh = b.KEY_SQUARE if square else b.KEY_WIDE
    sx, sy = kw / rw, kh / rh
    for off, kind, sq, wide in pool.scale_fields:
        v = sq if square else wide
        if kind == b.SCALE_MASK_POS:
            val = int(v * m)
        elif kind in (b.SCALE_MASK_SIZE, b.SCALE_MASK_LINE):
            val = max(1, int(v * m))
        elif kind == b.SCALE_MASK_FONT:
            val = best_font(b.FONT_SIZES[v][0] * m, b.FONT_SIZES[v][1] * m)
        elif kind == b.SCALE_KEY_X:
            val = int(v * sx)
        elif kind == b.SCALE_KEY_Y:
            val = int(v * sy)
        elif kind == b.SCALE_KEY_W:
            val = max(1, int(v * sx))
        elif kind == b.SCALE_KEY_H:
            val = max(1, int(v * sy))
        elif kind == b.SCALE_KEY_FONT:
            val = best_font(b.FONT_SIZES[v][0] * sx, b.FONT_SIZES[v][1] * sy)
        struct.pack_into(b.SCALE_KIND_FORMAT[kind], data, off, val)
    return bytes(data), ("square" if square else "wide"), sx, sy


def parse(data):
    objs = {}
    i = 0
    while i < len(data):
        oid, typ = struct.unpack_from("<HB", data, i)
        o = {"type": typ}
        if typ == b.TYPE_WORKING_SET:
            n = data[i + 7]
            o["children"] = [struct.unpack_from("<Hhh", data, i + 10 + 6 * k) for k in range(n)]
            size = 10 + 6 * n
        elif typ == b.TYPE_DATA_MASK:
            n = data[i + 6]
            o["children"] = [struct.unpack_from("<Hhh", data, i + 8 + 6 * k) for k in range(n)]
            size = 8 + 6 * n
        elif typ == b.TYPE_SOFT_KEY_MASK:
            size = 6 + 2 * data[i + 4]
        elif typ == b.TYPE_KEY:
            n = data[i + 5]
            o["children"] = [struct.unpack_from("<Hhh", data, i + 7 + 6 * k) for k in range(n)]
            size = 7 + 6 * n
        elif typ == b.TYPE_BUTTON:
            o["w"], o["h"] = struct.unpack_from("<HH", data, i + 3)
            n = data[i + 11]
            o["children"] = [struct.unpack_from("<Hhh", data, i + 13 + 6 * k) for k in range(n)]
            size = 13 + 6 * n
        elif typ == b.TYPE_INPUT_NUMBER:
            o["w"], o["h"] = struct.unpack_from("<HH", data, i + 3)
            o["font"] = struct.unpack_from("<H", data, i + 8)[0]
            o["text"] = b"255"
            size = 38
        elif typ == b.TYPE_OUTPUT_STRING:
            o["w"], o["h"] = struct.unpack_from("<HH", data, i + 3)
            o["font"] = struct.unpack_from("<H", data, i + 8)[0]
            n = struct.unpack_from("<H", data, i + 14)[0]
            o["text"] = data[i + 16:i + 16 + n].rstrip(b" ")
            size = 17 + n
        elif typ == b.TYPE_OUTPUT_RECTANGLE:
            o["w"], o["h"] = struct.unpack_from("<HH", data, i + 5)
            size = 13
        elif typ == b.TYPE_FONT_ATTRIBUTES:
            o["size"] = data[i + 4]
            size = 8
        elif typ in (b.TYPE_LINE_ATTRIBUTES, b.TYPE_FILL_ATTRIBUTES):
            size = 8
        elif typ == b.TYPE_STRING_VARIABLE:
            size = 5 + struct.unpack_from("<H", data, i + 3)[0]
        elif typ == b.TYPE_AUXILIARY_INPUT_TYPE_2:
            n = data[i + 5]
            o["children"] = [struct.unpack_from("<Hhh", data, i + 6 + 6 * k) for k in range(n)]
            size = 6 + 6 * n
        elif typ == b.TYPE_AUXILIARY_CONTROL_DESIGNATOR_TYPE_2:
            size = 6
        else:
            raise ValueError(f"unknown type {typ} at {i}")
        objs[oid] = o
        i += size
    assert i == len(data)
    return objs


def check(objs, mask, kw, kh, where):
    problems = []

    def text_fits(cid, o):
        if "text" not in o:
            return
        fw, fh = b.FONT_SIZES[objs[o["font"]]["size"]]
        if len(o["text"]) * fw > o["w"] or fh > o["h"]:
            problems.append(f"{where}: text {o['text']!r} ({fw}x{fh}) in {o['w']}x{o['h']} "
                            f"(object {cid})")

    def inside(parent, pw, ph):
        for cid, x, y in objs[parent].get("children", ()):
            c = objs[cid]
            w, h = c.get("w", 0), c.get("h", 0)
            if x < 0 or y < 0 or x + w > pw or y + h > ph:
                problems.append(f"{where}: {cid} at {x},{y} {w}x{h} outside {parent} ({pw}x{ph})")

    for oid, o in objs.items():
        t = o["type"]
        if t == b.TYPE_DATA_MASK:
            inside(oid, mask, mask)
        elif t in (b.TYPE_KEY, b.TYPE_AUXILIARY_INPUT_TYPE_2, b.TYPE_WORKING_SET):
            inside(oid, kw, kh)
        elif t == b.TYPE_BUTTON:
            inside(oid, o["w"], o["h"])
        text_fits(oid, o)
    return problems


def main():
    failed = False
    for multi in (False, True):
        pools = b.build_layouts(multi)
        for name, mask, kw, kh in PROFILES:
            layout_index = 1 if mask < b.COMPACT_BELOW_MASK else 0
            layout, pool = b.LAYOUTS[layout_index], pools[layout_index]
            data, shape, sx, sy = scale(pool, layout, mask, kw, kh)
            objs = parse(data)
            where = f"{'multi' if multi else 'single'} {name}"
            problems = check(objs, mask, kw, kh, where)
            fonts = {fid: b.FONT_SIZES[objs[fid]["size"]] for fid in
                     (b.TEST_FONT, b.MASK_TITLE_FONT, b.MASK_SMALL_FONT, b.KEY_FONT,
                      b.DESIGNATOR_FONT, b.PAGED_DESIGNATOR_FONT)}
            print(f"{where}: {layout.name} x{mask / layout.mask:.2f}, {shape} keys "
                  f"x{sx:.2f}/{sy:.2f}; fonts label {fonts[b.TEST_FONT]} title "
                  f"{fonts[b.MASK_TITLE_FONT]} small {fonts[b.MASK_SMALL_FONT]} key "
                  f"{fonts[b.KEY_FONT]} designator {fonts[b.DESIGNATOR_FONT]}"
                  f" -> {'OK' if not problems else f'{len(problems)} problems'}")
            for problem in problems[:8]:
                print("   ", problem)
            failed |= bool(problems)
    sys.exit(1 if failed else 0)


main()
