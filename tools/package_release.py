#!/usr/bin/env python3
"""
Packages every hardware/boards/* and hardware/enclosures/* folder into its
own release ZIP, so people can download just the parts they want:

    dist/AgIsoAuxInputs-<version>-board-<name>.zip
    dist/AgIsoAuxInputs-<version>-enclosure-<name>.zip

Board ZIPs also get the shared hardware/schematic.pdf.

Used by .github/workflows/release.yml; runs locally the same way:
    python tools/package_release.py v1.0.0
"""
import pathlib
import sys
import zipfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
HARDWARE = ROOT / "hardware"
SCHEMATIC = HARDWARE / "schematic.pdf"
DIST = ROOT / "dist"

KINDS = (("boards", "board"), ("enclosures", "enclosure"))


def main():
    version = sys.argv[1] if len(sys.argv) > 1 else "dev"
    DIST.mkdir(exist_ok=True)

    for folder_name, prefix in KINDS:
        for folder in sorted(p for p in (HARDWARE / folder_name).iterdir() if p.is_dir()):
            out = DIST / f"AgIsoAuxInputs-{version}-{prefix}-{folder.name}.zip"
            # Top-level folder carries the kind too, so a board and an
            # enclosure sharing a name don't overwrite each other when unzipped.
            top = f"{prefix}-{folder.name}"
            with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
                for f in sorted(folder.rglob("*")):
                    if f.is_file():
                        zf.write(f, f"{top}/{f.relative_to(folder).as_posix()}")
                if prefix == "board":
                    zf.write(SCHEMATIC, f"{top}/schematic.pdf")
            print(f"Wrote {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
