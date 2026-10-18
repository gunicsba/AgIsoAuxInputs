#!/usr/bin/env python3
"""
Packages the released firmware build (joystick_canlink, the one with the web
pages, Wi-Fi updates and the CANlink bridge) into two files:

    dist/AgIsoAuxInputs-<version>-<env>-full.bin    blank board, flash at 0x0
    dist/AgIsoAuxInputs-<version>-<env>-update.bin  the /update web page

The full image merges the bootloader, partition table, OTA data and app, so
a browser flasher (https://espressif.github.io/esptool-js/) can write it in
one go without PlatformIO.

Build first, then run from the repo root (needs `pip install "esptool>=5,<6"`):
    pio run -d firmware -e joystick_canlink
    python tools/package_firmware.py v1.0.0
"""
import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
BUILD = ROOT / "firmware" / ".pio" / "build"
DIST = ROOT / "dist"

ENVS = ("joystick_canlink",)

# Offsets from ESP-IDF's ESP32-S3 defaults and firmware/partitions.csv.
# The OTA data points the board at app0, so a board that ran an OTA update
# from app1 boots the freshly flashed app.
IMAGES = (
    ("0x0", "bootloader.bin"),
    ("0x8000", "partitions.bin"),
    ("0xe000", "ota_data_initial.bin"),
    ("0x10000", "firmware.bin"),
)


def main():
    version = sys.argv[1] if len(sys.argv) > 1 else "dev"
    DIST.mkdir(exist_ok=True)

    for env in ENVS:
        build = BUILD / env
        full = DIST / f"AgIsoAuxInputs-{version}-{env}-full.bin"
        update = DIST / f"AgIsoAuxInputs-{version}-{env}-update.bin"

        files = []
        for offset, name in IMAGES:
            files += [offset, str(build / name)]
        subprocess.run([sys.executable, "-m", "esptool", "--chip", "esp32s3",
                        "merge-bin", "-o", str(full),
                        "--flash-mode", "dio", "--flash-freq", "80m",
                        "--flash-size", "8MB", *files], check=True)
        shutil.copyfile(build / "firmware.bin", update)
        print(f"Wrote {full.relative_to(ROOT)}")
        print(f"Wrote {update.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
