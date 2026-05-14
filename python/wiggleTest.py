"""Wiggle all three motors both ways, then exit."""
import signal
import sys
import time
from arduino.app_utils import App, Bridge

signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

WIGGLE_SECONDS = 0.35
WIGGLE_MS = int(WIGGLE_SECONDS * 1000)
POWER = 100


def drive_motor(motor, direction, duration_ms):
    power = POWER if direction else 0
    Bridge.call("drive", motor, direction, power, duration_ms)


def call_mast(direction, duration_ms):
    Bridge.call("mast", direction, duration_ms)


def command_all(direction, duration_ms):
    drive_motor(0, direction, duration_ms)
    drive_motor(1, direction, duration_ms)
    call_mast(direction, duration_ms)


def loop():
    had_error = False
    try:
        command_all(1, WIGGLE_MS)
        time.sleep(WIGGLE_SECONDS)

        command_all(-1, WIGGLE_MS)
        time.sleep(WIGGLE_SECONDS)
    except BaseException:
        had_error = True
        raise
    finally:
        if had_error:
            try:
                command_all(0, 0)
            except Exception:
                pass
        else:
            command_all(0, 0)
    sys.exit(0)

App.run(user_loop=loop)
