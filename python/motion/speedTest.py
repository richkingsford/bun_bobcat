#!/usr/bin/env python3
"""Both-tread raw pin speed probe for Bun."""

import argparse
from pathlib import Path
import sys
import time


DEFAULT_ROUTER_SOCKET = "/var/run/arduino-router.sock"
DEFAULT_SECONDS = 0.5
DEFAULT_PAUSE = 1.0
DEFAULT_FLOOR_POWER = 37.5
DEFAULT_CEILING_POWER = 70.0
DEFAULT_PWM_PERIOD = 0.10
RPC_TIMEOUT_S = 2.0


FORWARD_PINS = (0, 1, 0, 1, 1)   # D6/D7 inverted, D8/D9/D10 unchanged
BACKWARD_PINS = (1, 0, 1, 0, 1)
COAST_PINS = (0, 0, 0, 0, 0)


def require_bridge(router_socket):
    if not Path(router_socket).exists():
        print(f"[speed] router socket not found at {router_socket}", file=sys.stderr)
        return None

    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:
        print(f"[speed] arduino.app_utils is unavailable: {exc}", file=sys.stderr)
        return None

    try:
        Bridge.call("drive_pins", *COAST_PINS, timeout=RPC_TIMEOUT_S)
    except ValueError:
        print("[speed] loaded sketch does not expose drive_pins.", file=sys.stderr)
        print("[speed] upload sketch/sketch.ino, then retry.", file=sys.stderr)
        return None
    except (TimeoutError, RuntimeError, OSError) as exc:
        print(f"[speed] bridge check failed: {exc}", file=sys.stderr)
        return None

    return Bridge


def hold_pins(bridge, pins, seconds, power, pwm_period):
    on_s = pwm_period * power / 100.0
    off_s = pwm_period - on_s
    end_t = time.monotonic() + seconds
    while True:
        remaining = end_t - time.monotonic()
        if remaining <= 0:
            break

        cycle_s = min(pwm_period, remaining)
        cycle_on_s = min(on_s, cycle_s)
        cycle_off_s = max(0.0, cycle_s - cycle_on_s)

        if cycle_on_s > 0:
            bridge.notify("drive_pins", *pins)
            time.sleep(cycle_on_s)
        if cycle_off_s > 0:
            bridge.notify("drive_pins", *COAST_PINS)
            time.sleep(cycle_off_s)


def coast(bridge):
    try:
        bridge.notify("drive_pins", *COAST_PINS)
    except Exception:
        pass


def describe(label, pins, seconds, power):
    d6, d7, d8, d9, d10 = pins
    print(
        f"[speed] {label}: D6={d6} D7={d7} D8={d8} D9={d9} D10={d10} "
        f"at {power:.1f}% for {seconds:.2f}s"
    )


def run_pair(bridge, label, power, args):
    print(f"[speed] {label} pass")
    describe("both forward", FORWARD_PINS, args.seconds, power)
    hold_pins(bridge, FORWARD_PINS, args.seconds, power, args.pwm_period)
    coast(bridge)
    time.sleep(args.pause)

    describe("both backward", BACKWARD_PINS, args.seconds, power)
    hold_pins(bridge, BACKWARD_PINS, args.seconds, power, args.pwm_period)
    coast(bridge)


def main():
    parser = argparse.ArgumentParser(
        description="Move both treads forward, pause, then backward."
    )
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--pause", type=float, default=DEFAULT_PAUSE)
    parser.add_argument("--floor-power", type=float, default=DEFAULT_FLOOR_POWER)
    parser.add_argument("--ceiling-power", type=float, default=DEFAULT_CEILING_POWER)
    parser.add_argument("--pwm-period", type=float, default=DEFAULT_PWM_PERIOD)
    parser.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET)
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.pause < 0:
        parser.error("--pause must be non-negative")
    if not 0 < args.floor_power <= 100:
        parser.error("--floor-power must be in 0..100")
    if not 0 < args.ceiling_power <= 100:
        parser.error("--ceiling-power must be in 0..100")
    if args.floor_power > args.ceiling_power:
        parser.error("--floor-power must be <= --ceiling-power")
    if args.pwm_period <= 0:
        parser.error("--pwm-period must be positive")

    bridge = require_bridge(args.router_socket)
    if bridge is None:
        return 1

    try:
        run_pair(bridge, "floor", args.floor_power, args)
        time.sleep(args.pause)
        run_pair(bridge, "ceiling", args.ceiling_power, args)
    except KeyboardInterrupt:
        print("\n[speed] interrupted; all low")
        return 130
    finally:
        coast(bridge)

    print("[speed] done; all low")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
