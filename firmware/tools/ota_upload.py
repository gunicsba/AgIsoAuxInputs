"""PlatformIO script adding an "ota" target: upload the firmware over Wi-Fi.

    pio run -e joystick_viewt -t ota [--upload-port HOST]

Builds, POSTs the .bin to the board's /update page (ESP2SOTA), then waits
for the board to come back and prints the commit it now runs (from /debug).
HOST defaults to the env's custom_ota_host (its mDNS name); use the AP
address (192.168.4.1 / 192.168.0.34) when connected to the board's own AP.
"""
import time
import urllib.error
import urllib.request
import uuid
from http.client import RemoteDisconnected

Import("env")  # noqa: F821 - provided by PlatformIO

REBOOT_WAIT_S = 60


def upload_host(env):
    port = env.subst("$UPLOAD_PORT")
    # --upload-port, unless PlatformIO filled in a serial port by itself
    if port and not port.upper().startswith("COM") and not port.startswith("/dev/"):
        return port
    return env.GetProjectOption("custom_ota_host")


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
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            reply = response.read().decode(errors="replace").strip()
    except (RemoteDisconnected, ConnectionResetError):
        reply = "OK?"  # the board may reboot before the reply gets out
    except (urllib.error.URLError, OSError) as error:
        print(f"OTA upload to {host} failed: {error}")
        env.Exit(1)
    if reply not in ("OK", "OK?"):
        print(f"Board rejected the firmware: {reply!r} (it reboots into the old one)")
        env.Exit(1)

    print("Uploaded, waiting for the board to reboot ...")
    time.sleep(3)
    deadline = time.time() + REBOOT_WAIT_S
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"http://{host}/debug", timeout=3) as response:
                print("Board is back: " + response.readline().decode(errors="replace").strip())
                return
        except (urllib.error.URLError, OSError):
            time.sleep(2)
    print(f"Board not reachable at {host} after {REBOOT_WAIT_S} s")
    env.Exit(1)


env.AddCustomTarget(  # noqa: F821
    name="ota",
    dependencies="$BUILD_DIR/${PROGNAME}.bin",
    actions=ota_upload,
    title="OTA upload",
    description="Upload the firmware over Wi-Fi to the board's /update page",
)
