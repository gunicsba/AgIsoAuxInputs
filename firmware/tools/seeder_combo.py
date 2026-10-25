"""PlatformIO pre-build script for the joystick_seeder test env: a second
working set on the same board, the seeder controller from
github.com/davidbalogh92/Isobus_ESP_seeder_V2 (a separate checkout, see
custom_seeder_dir in platformio.ini).

- Exports SEEDER_DIR, so src/CMakeLists.txt compiles the seeder's core and
  ISOBUS app (VT UI, TC client) into this firmware.
- Splits the seeder's generated object pool at object boundaries into
  custom_seeder_pool_parts parts of about equal size and writes them to
  src/object_pool/seeder_pool_parts.h as C arrays (and each part as
  seeder_pool_part_<n>.iop next to it). The VT client uploads
  each part as its own Object Pool Transfer, then one End of Object Pool -
  to test VTs on chunked uploads.

Standalone check: python tools/seeder_combo.py <pool.iop> <parts>
"""
import os
import pathlib
import struct
import sys
import zlib


# Fixed bytes per object type (after ID + type), and where its variable
# lengths are: ("n6", offset) a 1-byte count of 6-byte child refs,
# ("n2", offset) of 2-byte refs (macros, list items, soft keys...),
# ("n4", offset) of 4-byte points, ("len1"/"len2"/"len4", offset) a byte
# count of raw data. Offsets are from the start of the object.
OBJECT_LAYOUTS = {
    0: (10, [("n6", 7), ("n2", 8), ("n2", 9)]),   # Working Set (languages are 2 bytes)
    1: (8, [("n6", 6), ("n2", 7)]),               # Data Mask
    2: (10, [("n6", 8), ("n2", 9)]),              # Alarm Mask
    3: (10, [("n6", 8), ("n2", 9)]),              # Container
    4: (6, [("n2", 4), ("n2", 5)]),               # Soft Key Mask
    5: (7, [("n6", 5), ("n2", 6)]),               # Key
    6: (13, [("n6", 11), ("n2", 12)]),            # Button
    7: (13, [("n2", 12)]),                        # Input Boolean
    8: (19, [("len1", 16), ("n2", 18)]),          # Input String
    9: (38, [("n2", 37)]),                        # Input Number
    10: (13, [("n2", 10), ("n2", 12)]),           # Input List
    11: (17, [("len2", 14), ("n2", 16)]),         # Output String
    12: (29, [("n2", 28)]),                       # Output Number
    13: (11, [("n2", 10)]),                       # Line
    14: (13, [("n2", 12)]),                       # Rectangle
    15: (15, [("n2", 14)]),                       # Ellipse
    16: (14, [("n4", 12), ("n2", 13)]),           # Polygon
    17: (21, [("n2", 20)]),                       # Meter
    18: (24, [("n2", 23)]),                       # Linear Bar Graph
    19: (27, [("n2", 26)]),                       # Arched Bar Graph
    20: (17, [("len4", 12), ("n2", 16)]),         # Picture Graphic
    21: (7, []),                                  # Number Variable
    22: (5, [("len2", 3)]),                       # String Variable
    23: (8, [("n2", 7)]),                         # Font Attributes
    24: (8, [("n2", 7)]),                         # Line Attributes
    25: (8, [("n2", 7)]),                         # Fill Attributes
    26: (6, [("len1", 4), ("n2", 5)]),            # Input Attributes
    27: (5, []),                                  # Object Pointer
    28: (5, [("len2", 3)]),                       # Macro
}
COUNT_SIZES = {"n6": 6, "n2": 2, "n4": 4}


def object_size(pool, start):
    obj_type = pool[start + 2]
    if obj_type not in OBJECT_LAYOUTS:
        raise ValueError(f"object at offset {start}: unsupported type {obj_type}")
    fixed, variable = OBJECT_LAYOUTS[obj_type]
    # Variable parts follow the fixed part in the order listed, so the
    # offsets of fields after a raw data block (Input String, Output String,
    # Picture, Input Attributes) shift by its length.
    size = fixed
    shift = 0
    for kind, offset in variable:
        pos = start + offset + shift
        if kind in COUNT_SIZES:
            size += pool[pos] * COUNT_SIZES[kind]
        else:
            n = {"len1": 1, "len2": 2, "len4": 4}[kind]
            length = int.from_bytes(pool[pos:pos + n], "little")
            size += length
            # The raw data sits right after this field's fixed position for
            # Input/Output String and Input Attributes, and after the whole
            # fixed part for a Picture, with the macro count before it
            if obj_type != 20:
                shift += length
    return size


def object_offsets(pool):
    offsets = []
    pos = 0
    while pos < len(pool):
        offsets.append(pos)
        pos += object_size(pool, pos)
    if pos != len(pool):
        raise ValueError(f"last object runs past the end ({pos} > {len(pool)})")
    return offsets


def split_pool(pool, parts):
    offsets = object_offsets(pool)
    cuts = [0]
    for k in range(1, parts):
        target = len(pool) * k // parts
        cut = min((o for o in offsets if o > cuts[-1]), key=lambda o: abs(o - target), default=None)
        if cut is not None and cut not in cuts:
            cuts.append(cut)
    cuts.append(len(pool))
    return [pool[a:b] for a, b in zip(cuts, cuts[1:])], len(offsets)


def version_label(pool, parts):
    # 7 chars (VT version labels are 32 bytes on VT5, 7 before): the part
    # count first, so a VT that cached the seeder pool under another split
    # (or unsplit, under its own label) gets a fresh, chunked upload.
    return f"P{parts}{zlib.crc32(pool) & 0xFFFFF:05X}"


def write_header(path, chunks, label, source):
    lines = [
        "// Generated by tools/seeder_combo.py at build time - do not edit.",
        f"// {source.name} split into {len(chunks)} parts at object boundaries.",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        f'#define SEEDER_POOL_VERSION "{label}"',
        f"#define SEEDER_POOL_PART_COUNT {len(chunks)}",
        "",
    ]
    for i, chunk in enumerate(chunks):
        lines.append(f"static const uint8_t SEEDER_POOL_PART_{i}[{len(chunk)}] = {{")
        for row in range(0, len(chunk), 24):
            lines.append("  " + ",".join(str(b) for b in chunk[row:row + 24]) + ",")
        lines.append("};")
    lines.append("")
    lines.append("static const uint8_t *const SEEDER_POOL_PARTS[SEEDER_POOL_PART_COUNT] = {")
    lines.append("  " + ", ".join(f"SEEDER_POOL_PART_{i}" for i in range(len(chunks))) + ",")
    lines.append("};")
    lines.append("static const uint32_t SEEDER_POOL_PART_SIZES[SEEDER_POOL_PART_COUNT] = {")
    lines.append("  " + ", ".join(str(len(c)) for c in chunks) + ",")
    lines.append("};")
    content = "\n".join(lines) + "\n"
    if not path.exists() or path.read_text() != content:
        path.write_text(content)


def main():
    pool = pathlib.Path(sys.argv[1]).read_bytes()
    chunks, objects = split_pool(pool, int(sys.argv[2]))
    assert b"".join(chunks) == pool
    print(f"{objects} objects, parts {[len(c) for c in chunks]}, version {version_label(pool, len(chunks))}")


if __name__ == "__main__":
    main()
else:
    Import("env")  # noqa: F821 - provided by PlatformIO

    project_dir = pathlib.Path(env["PROJECT_DIR"])  # noqa: F821
    seeder_dir = (project_dir / env.GetProjectOption("custom_seeder_dir")).resolve()  # noqa: F821
    parts = int(env.GetProjectOption("custom_seeder_pool_parts", "4"))  # noqa: F821
    source = seeder_dir / "generated" / "object_pool.iop"
    if not source.exists():
        sys.exit(f"seeder_combo: {source} not found - clone "
                 "github.com/davidbalogh92/Isobus_ESP_seeder_V2 there or set custom_seeder_dir")
    os.environ["SEEDER_DIR"] = seeder_dir.as_posix()
    pool = source.read_bytes()
    chunks, objects = split_pool(pool, parts)
    label = version_label(pool, len(chunks))
    out_dir = project_dir / "src" / "object_pool"
    write_header(out_dir / "seeder_pool_parts.h", chunks, label, source)
    for old in out_dir.glob("seeder_pool_part_*.iop"):
        old.unlink()
    for i, chunk in enumerate(chunks):
        (out_dir / f"seeder_pool_part_{i}.iop").write_bytes(chunk)
    print(f"seeder pool: {objects} objects in {len(chunks)} parts {[len(c) for c in chunks]}, "
          f"version {label}, from {seeder_dir}")
