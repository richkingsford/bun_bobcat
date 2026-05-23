#!/usr/bin/env python3
"""Move Bun's continuous-rotation mast servo both ways."""

import argparse
import time

from arduino.app_utils import Bridge


DEFAULT_SECONDS = 0.5
DEFAULT_POWER = 100
COMMAND_PERIOD_S = 0.05


def hold(speed, seconds):
    end_t = time.monotonic() + seconds
    while True:
        Bridge.notify("drive_mast", speed)
        remaining = end_t - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(COMMAND_PERIOD_S, remaining))


def coast():
    Bridge.notify("drive_mast", 0)


def main():
    parser = argparse.ArgumentParser(
        description="Move the D10 mast servo one way, then the other."
    )
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--power", type=int, default=DEFAULT_POWER)
    parser.add_argument("--pause", type=float, default=0.2)
    args = parser.parse_args()

    power = max(1, min(100, args.power))

    try:
        Bridge.call("drive_mast", 0, timeout=2)
    except ValueError:
        print("[mast] loaded sketch does not expose drive_mast")
        print("[mast] upload the updated sketch/sketch.ino, then retry")
        return 2

    try:
        print(f"[mast] servo one way for {args.seconds:.2f}s", flush=True)
        hold(power, args.seconds)
        coast()
        time.sleep(args.pause)

        print(f"[mast] servo other way for {args.seconds:.2f}s", flush=True)
        hold(-power, args.seconds)
        coast()
    finally:
        coast()

    print("[mast] done; servo neutral", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
