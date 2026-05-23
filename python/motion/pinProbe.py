#!/usr/bin/env python3
"""Raw D6-D10 pin probe for Bun motor-driver diagnostics."""

import argparse
from pathlib import Path
import sys
import time


DEFAULT_ROUTER_SOCKET = "/var/run/arduino-router.sock"
DEFAULT_SECONDS = 0.5
COMMAND_PERIOD_S = 0.05
RPC_TIMEOUT_S = 2.0


def bit(value):
    return 1 if int(value) else 0


def require_bridge(router_socket):
    if not Path(router_socket).exists():
        print(f"[pins] router socket not found at {router_socket}", file=sys.stderr)
        return None

    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:
        print(f"[pins] arduino.app_utils is unavailable: {exc}", file=sys.stderr)
        return None

    try:
        Bridge.call("drive_pins", 0, 0, 0, 0, 0, timeout=RPC_TIMEOUT_S)
    except ValueError:
        print("[pins] loaded sketch does not expose drive_pins.", file=sys.stderr)
        print("[pins] upload sketch/sketch.ino, then retry.", file=sys.stderr)
        return None
    except (TimeoutError, RuntimeError, OSError) as exc:
        print(f"[pins] bridge check failed: {exc}", file=sys.stderr)
        return None

    return Bridge


def hold_pins(bridge, d6, d7, d8, d9, d10, seconds):
    end_t = time.monotonic() + seconds
    while True:
        bridge.notify("drive_pins", d6, d7, d8, d9, d10)
        remaining = end_t - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(COMMAND_PERIOD_S, remaining))


def coast(bridge):
    try:
        bridge.notify("drive_pins", 0, 0, 0, 0, 0)
    except Exception:
        pass


def named_pattern(name):
    patterns = {
        "left-r-plus": (0, 0, 1, 0, 0),
        "left-r-minus": (0, 0, 0, 1, 0),
        "left-r-minus-enable": (0, 0, 0, 1, 1),
        "left-r-plus-enable": (0, 0, 1, 0, 1),
        "left-both-enable": (0, 0, 1, 1, 1),
        "right-l-plus": (1, 0, 0, 0, 0),
        "right-l-minus": (0, 1, 0, 0, 0),
        "all-low": (0, 0, 0, 0, 0),
    }
    return patterns[name]


def main():
    parser = argparse.ArgumentParser(description="Hold raw D6-D10 states briefly.")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--pattern", choices=(
        "left-r-plus",
        "left-r-minus",
        "left-r-minus-enable",
        "left-r-plus-enable",
        "left-both-enable",
        "right-l-plus",
        "right-l-minus",
        "all-low",
    ))
    parser.add_argument("--d6", type=bit, default=0)
    parser.add_argument("--d7", type=bit, default=0)
    parser.add_argument("--d8", type=bit, default=0)
    parser.add_argument("--d9", type=bit, default=0)
    parser.add_argument("--d10", type=bit, default=0)
    parser.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET)
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    if args.pattern:
        d6, d7, d8, d9, d10 = named_pattern(args.pattern)
    else:
        d6, d7, d8, d9, d10 = args.d6, args.d7, args.d8, args.d9, args.d10

    bridge = require_bridge(args.router_socket)
    if bridge is None:
        return 1

    print(
        f"[pins] holding D6={d6} D7={d7} D8={d8} D9={d9} D10={d10} "
        f"for {args.seconds:.2f}s"
    )
    try:
        hold_pins(bridge, d6, d7, d8, d9, d10, args.seconds)
    except KeyboardInterrupt:
        print("\n[pins] interrupted; all low")
        return 130
    finally:
        coast(bridge)

    print("[pins] done; all low")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
