#!/usr/bin/env python3
"""Wiggle Bun's treads and mast servo through the UNO Q bridge."""

import argparse
from pathlib import Path
import sys
import time

from host_controller import DEFAULT_PORT, SERIAL_BAUD, SerialLink


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


def wiggle_steps(power, mast_only):
    steps = []
    if not mast_only:
        steps.extend([
            ("left tread forward", power, 0, 0),
            ("left tread backward", -power, 0, 0),
            ("right tread forward", 0, power, 0),
            ("right tread backward", 0, -power, 0),
        ])
    steps.extend([
        ("mast positive", 0, 0, power),
        ("mast negative", 0, 0, -power),
    ])
    return steps


def run_serial_wiggle(args, power):
    link = SerialLink(args.port, args.baud)
    if not link.connect():
        print(f"[wiggle] serial transport is not ready at {args.port}", file=sys.stderr)
        return 1

    def hold(left, right, mast, seconds):
        end_t = time.monotonic() + seconds
        while True:
            link.send(left, right, mast)
            remaining = end_t - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(COMMAND_PERIOD_S, remaining))

    try:
        for cycle in range(args.cycles):
            if args.cycles > 1:
                print(f"[wiggle] cycle {cycle + 1}/{args.cycles}")

            steps = wiggle_steps(power, args.mast_only)

            for index, (label, left, right, mast) in enumerate(steps):
                print(f"[wiggle] {label} for {args.seconds:.2f}s")
                hold(left, right, mast, args.seconds)
                link.coast()
                if cycle != args.cycles - 1 or index != len(steps) - 1:
                    pause_after_coast(args.pause)
    finally:
        link.coast()
        link.close()

    print("[wiggle] done; motors coasting and mast neutral")
    return 0


def run_rpc_wiggle(args, power):
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

            steps = wiggle_steps(power, args.mast_only)
            for index, (label, left, right, mast) in enumerate(steps):
                print(f"[wiggle] {label} for {args.seconds:.2f}s")
                hold_drive(Bridge, left, right, mast, args.seconds)
                coast(Bridge)
                if cycle != args.cycles - 1 or index != len(steps) - 1:
                    pause_after_coast(args.pause)
    finally:
        coast(Bridge)

    print("[wiggle] done; motors coasting and mast neutral")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Wiggle Bun's treads forward/back, then D10 mast servo up/down."
    )
    parser.add_argument("--transport", choices=("serial", "rpc"), default="rpc",
                        help="movement transport (default rpc)")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port for --transport serial (default {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS,
                        help=f"seconds to hold each direction (default {DEFAULT_SECONDS:g})")
    parser.add_argument("--power", type=int, default=DEFAULT_POWER,
                        help=f"speed magnitude 0-100 (default {DEFAULT_POWER})")
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

    if args.transport == "serial":
        return run_serial_wiggle(args, power)
    return run_rpc_wiggle(args, power)


if __name__ == "__main__":
    raise SystemExit(main())
