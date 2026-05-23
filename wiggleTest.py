#!/usr/bin/env python3
"""Wiggle Bun's treads and mast servo through the UNO Q bridge."""

import argparse
from pathlib import Path
import sys
import time


DEFAULT_ROUTER_SOCKET = "/var/run/arduino-router.sock"
DEFAULT_SECONDS = 0.5
DEFAULT_POWER = 100
RPC_TIMEOUT_S = 2.0
COMMAND_PERIOD_S = 0.05


def clamp_power(power):
    return max(0, min(100, int(power)))


def hold_drive(bridge, left, right, third, seconds):
    end_t = time.monotonic() + seconds
    while True:
        bridge.notify("drive_triple", left, right, third)
        remaining = end_t - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(COMMAND_PERIOD_S, remaining))


def coast(bridge):
    try:
        bridge.notify("drive_triple", 0, 0, 0)
    except Exception:
        pass


def pause_after_coast(seconds):
    if seconds > 0:
        time.sleep(seconds)


def main():
    parser = argparse.ArgumentParser(
        description="Wiggle Bun's treads forward/back, then D10 mast servo up/down."
    )
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS,
                        help="seconds to hold each direction (default 0.5 = 500 ms)")
    parser.add_argument("--power", type=int, default=DEFAULT_POWER,
                        help="speed magnitude 0-100 (default 100)")
    parser.add_argument("--cycles", type=int, default=1,
                        help="number of forward/back wiggle cycles")
    parser.add_argument("--pause", type=float, default=0.1,
                        help="coast pause between direction changes")
    parser.add_argument("--mast-only", action="store_true",
                        help="only move the D10 mast servo up/down")
    parser.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET,
                        help=f"App Lab router socket (default {DEFAULT_ROUTER_SOCKET})")
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.cycles <= 0:
        parser.error("--cycles must be positive")
    if args.pause < 0:
        parser.error("--pause must be non-negative")

    power = clamp_power(args.power)
    if power == 0:
        parser.error("--power must be greater than 0")

    if not Path(args.router_socket).exists():
        print(f"[wiggle] router socket not found at {args.router_socket}", file=sys.stderr)
        print("[wiggle] run this on the UNO Q/App Lab host with the router active.", file=sys.stderr)
        return 1

    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:
        print(f"[wiggle] arduino.app_utils is unavailable: {exc}", file=sys.stderr)
        return 1

    try:
        Bridge.call("drive_triple", 0, 0, 0, timeout=RPC_TIMEOUT_S)
    except ValueError:
        print("[wiggle] loaded sketch does not expose drive_triple.", file=sys.stderr)
        print("[wiggle] upload sketch/sketch.ino from this repo, then retry.", file=sys.stderr)
        return 2
    except (TimeoutError, RuntimeError, OSError) as exc:
        print(f"[wiggle] bridge check failed: {exc}", file=sys.stderr)
        return 1

    try:
        for cycle in range(args.cycles):
            if args.cycles > 1:
                print(f"[wiggle] cycle {cycle + 1}/{args.cycles}")

            if not args.mast_only:
                print(f"[wiggle] treads forward for {args.seconds:.2f}s")
                hold_drive(Bridge, power, power, 0, args.seconds)
                coast(Bridge)
                pause_after_coast(args.pause)

                print(f"[wiggle] treads backward for {args.seconds:.2f}s")
                hold_drive(Bridge, -power, -power, 0, args.seconds)
                coast(Bridge)
                pause_after_coast(args.pause)

            print(f"[wiggle] mast up for {args.seconds:.2f}s")
            hold_drive(Bridge, 0, 0, power, args.seconds)
            coast(Bridge)
            pause_after_coast(args.pause)

            print(f"[wiggle] mast down for {args.seconds:.2f}s")
            hold_drive(Bridge, 0, 0, -power, args.seconds)
            coast(Bridge)
            if cycle != args.cycles - 1:
                pause_after_coast(args.pause)
    finally:
        coast(Bridge)

    print("[wiggle] done; motors coasting and mast neutral")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
