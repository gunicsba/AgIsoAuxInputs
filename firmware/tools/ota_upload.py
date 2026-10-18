"""PlatformIO script adding an "ota" target: upload the firmware over Wi-Fi.

    pio run -e joystick_canlink -t ota [--upload-port HOST]

Builds, POSTs the .bin to the board's /update page (ESP2SOTA), then waits
for the board to come back and checks (from /debug) that it runs the commit
just built.
HOST defaults to the env's custom_ota_host (its mDNS name); use the AP
address (192.168.4.1 / 192.168.0.34) when connected to the board's own AP.
"""
import pathlib
import re
import socket
import time
import urllib.error
import urllib.request
import uuid
from http.client import RemoteDisconnected

Import("env")  # noqa: F821 - provided by PlatformIO

REBOOT_WAIT_S = 60
UPLOAD_TIMEOUT_S = 60


def upload_host(env):
    port = env.subst("$UPLOAD_PORT")
    # --upload-port, unless PlatformIO filled in a serial port by itself
    if port and not port.upper().startswith("COM") and not port.startswith("/dev/"):
        return port
    return env.GetProjectOption("custom_ota_host")


def built_commit(env):
    """GIT_COMMIT_SHORT from the src/git_version.h this build used."""
    header = pathlib.Path(env.subst("$PROJECT_SRC_DIR")) / "git_version.h"
    match = re.search(r'GIT_COMMIT_SHORT "([^"]*)"', header.read_text())
    return match.group(1) if match else None


def ota_upload(source, target, env):
    host = upload_host(env)
    firmware = open(str(source[0]), "rb").read()
    boundary = uuid.uuid4().hex
    body = (f"--{boundary}\r\n"
            'Content-Disposition: form-data; name="update"; filename="firmware.bin"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n").encode() + firmware + \
        f"\r\n--{boundary}--\r\n".encode()
    request = urllib.request.Request(
        f"http://{host}/update", data=body,
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})

    print(f"Uploading {len(firmware)} bytes to http://{host}/update ...")
    started = time.time()
    try:
        with urllib.request.urlopen(request, timeout=UPLOAD_TIMEOUT_S) as response:
            reply = response.read().decode(errors="replace").strip()
    except (RemoteDisconnected, ConnectionResetError, socket.timeout, TimeoutError):
        # ESP2SOTA reboots right after replying, so the reply can be cut off
        # or never arrive; the commit check below tells whether it worked.
        reply = "OK?"
    except urllib.error.URLError as error:
        if isinstance(error.reason, (socket.timeout, TimeoutError)):
            reply = "OK?"
        else:
            print(f"OTA upload to {host} failed: {error}")
            env.Exit(1)
    except OSError as error:
        print(f"OTA upload to {host} failed: {error}")
        env.Exit(1)
    if reply not in ("OK", "OK?"):
        print(f"Board rejected the firmware: {reply!r} (it reboots into the old one)")
        env.Exit(1)

    expected = built_commit(env)
    print("Uploaded" if reply == "OK" else "Upload reply missing", "- waiting for the board ...")
    time.sleep(3)
    deadline = time.time() + REBOOT_WAIT_S
    status = None
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"http://{host}/debug", timeout=3) as response:
                status = response.readline().decode(errors="replace").strip()
            # The commit alone doesn't prove the update: a rebuild with the same
            # uncommitted changes reports the same one. The uptime has to
            # show a reboot since the upload started.
            uptime = re.search(r", up (\d+) s,", status)
            rebooted = uptime is not None and int(uptime.group(1)) <= time.time() - started
            if rebooted and (expected is None or f" {expected}," in status):
                print("Board is back: " + status)
                return
        except (urllib.error.URLError, OSError):
            pass
        time.sleep(2)
    if status:
        print(f"Board answers but does not run {expected}: {status}")
    else:
        print(f"Board not reachable at {host} after {REBOOT_WAIT_S} s")
    env.Exit(1)


env.AddCustomTarget(  # noqa: F821
    name="ota",
    dependencies="$BUILD_DIR/${PROGNAME}.bin",
    actions=ota_upload,
    title="OTA upload",
    description="Upload the firmware over Wi-Fi to the board's /update page",
)
