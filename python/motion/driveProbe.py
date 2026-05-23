#!/usr/bin/env python3
"""Raw tread channel probe for Bun."""

import argparse
from pathlib import Path
import sys
import time


DEFAULT_ROUTER_SOCKET = "/var/run/arduino-router.sock"
DEFAULT_SECONDS = 1.0
DEFAULT_PAUSE = 0.5
DEFAULT_COMMAND_PERIOD_S = 0.05
DEFAULT_DUTY_PERCENT = 100.0
DEFAULT_LEFT_DUTY_PERCENT = None
DEFAULT_RIGHT_DUTY_PERCENT = None
DEFAULT_PWM_PERIOD_S = 0.25
DEFAULT_MIN_ON_S = 0.015
RPC_TIMEOUT_S = 2.0


def require_bridge(router_socket):
    if not Path(router_socket).exists():
        print(f"[probe] router socket not found at {router_socket}", file=sys.stderr)
        return None

    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:
        print(f"[probe] arduino.app_utils is unavailable: {exc}", file=sys.stderr)
        return None

    try:
        Bridge.call("drive_triple", 0, 0, 0, timeout=RPC_TIMEOUT_S)
    except ValueError:
        print("[probe] loaded sketch does not expose drive_triple.", file=sys.stderr)
        return None
    except (TimeoutError, RuntimeError, OSError) as exc:
        print(f"[probe] bridge check failed: {exc}", file=sys.stderr)
        return None

    return Bridge


def coast(bridge, dry_run=False):
    if dry_run:
        return
    try:
        bridge.notify("drive_triple", 0, 0, 0)
    except Exception:
        pass


def duty_for_channel(channel_duty, default_duty):
    return default_duty if channel_duty is None else channel_duty


def send_raw_pwm_cycle(bridge, left, right, left_duty, right_duty, cycle_s, min_on_s):
    left_on = left != 0 and left_duty > 0
    right_on = right != 0 and right_duty > 0
    left_off_t = min(cycle_s, max(min_on_s, cycle_s * left_duty / 100.0)) if left_on else 0.0
    right_off_t = min(cycle_s, max(min_on_s, cycle_s * right_duty / 100.0)) if right_on else 0.0

    events = sorted({0.0, left_off_t, right_off_t, cycle_s})
    for start, end in zip(events, events[1:]):
        if end <= start:
            continue
        active_left = left if left_on and start < left_off_t else 0
        active_right = right if right_on and start < right_off_t else 0
        bridge.notify("drive_triple", int(active_left), int(active_right), 0)
        time.sleep(end - start)


def hold_raw(bridge, left, right, seconds, args):
    left_duty = duty_for_channel(args.left_duty_percent, args.duty_percent)
    right_duty = duty_for_channel(args.right_duty_percent, args.duty_percent)
    print(
        f"[probe] raw L={left:+d} R={right:+d} for {seconds:.2f}s "
        f"at L-duty={left_duty:.1f}% R-duty={right_duty:.1f}%"
    )
    dry_run = args.dry_run
    if dry_run:
        time.sleep(min(seconds, 0.1))
        return

    end_t = time.monotonic() + seconds
    try:
        while True:
            remaining = end_t - time.monotonic()
            if remaining <= 0:
                break
            if left_duty >= 99.9 and right_duty >= 99.9:
                on_s = min(args.command_period, remaining)
                bridge.notify("drive_triple", int(left), int(right), 0)
                time.sleep(on_s)
            else:
                cycle_s = min(args.pwm_period, remaining)
                send_raw_pwm_cycle(
                    bridge,
                    left,
                    right,
                    left_duty,
                    right_duty,
                    cycle_s,
                    args.min_on,
                )
                coast(bridge)
    finally:
        coast(bridge)


def matrix_steps(seconds):
    return [
        ("raw R- only: expected physical left one way", 0, -100, seconds),
        ("raw R+ only: expected physical left other way", 0, +100, seconds),
        ("raw L- only: expected physical right one way", -100, 0, seconds),
        ("raw L+ only: expected physical right other way", +100, 0, seconds),
        ("raw L-/R- together", -100, -100, seconds),
        ("raw L-/R+ together", -100, +100, seconds),
        ("raw L+/R- together", +100, -100, seconds),
        ("raw L+/R+ together", +100, +100, seconds),
    ]


def main():
    parser = argparse.ArgumentParser(description="Send exact raw tread commands.")
    parser.add_argument("--left", type=int, default=None,
                        help="raw left command channel, -100..100")
    parser.add_argument("--right", type=int, default=None,
                        help="raw right command channel, -100..100")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--pause", type=float, default=DEFAULT_PAUSE)
    parser.add_argument("--command-period", type=float, default=DEFAULT_COMMAND_PERIOD_S,
                        help="seconds between repeated raw commands at 100% duty")
    parser.add_argument("--duty-percent", type=float, default=DEFAULT_DUTY_PERCENT,
                        help="host-side duty cycle for the raw command")
    parser.add_argument("--left-duty-percent", type=float, default=DEFAULT_LEFT_DUTY_PERCENT,
                        help="host-side duty cycle for raw left channel")
    parser.add_argument("--right-duty-percent", type=float, default=DEFAULT_RIGHT_DUTY_PERCENT,
                        help="host-side duty cycle for raw right channel")
    parser.add_argument("--pwm-period", type=float, default=DEFAULT_PWM_PERIOD_S,
                        help="host-side PWM cycle length below 100% duty")
    parser.add_argument("--min-on", type=float, default=DEFAULT_MIN_ON_S,
                        help="shortest on-pulse below 100% duty")
    parser.add_argument("--matrix", action="store_true",
                        help="run a labeled raw command matrix")
    parser.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.pause < 0:
        parser.error("--pause must be non-negative")
    if args.command_period <= 0:
        parser.error("--command-period must be positive")
    if not 0 < args.duty_percent <= 100:
        parser.error("--duty-percent must be in 0..100")
    for label, value in (
        ("--left-duty-percent", args.left_duty_percent),
        ("--right-duty-percent", args.right_duty_percent),
    ):
        if value is not None and not 0 <= value <= 100:
            parser.error(f"{label} must be in 0..100")
    if args.pwm_period <= 0 or args.min_on <= 0 or args.min_on > args.pwm_period:
        parser.error("--pwm-period and --min-on must be positive, with min <= period")

    if args.matrix:
        steps = matrix_steps(args.seconds)
    else:
        if args.left is None or args.right is None:
            parser.error("provide --left and --right, or use --matrix")
        if not -100 <= args.left <= 100 or not -100 <= args.right <= 100:
            parser.error("--left and --right must be in -100..100")
        steps = [("single raw command", args.left, args.right, args.seconds)]

    bridge = None if args.dry_run else require_bridge(args.router_socket)
    if bridge is None and not args.dry_run:
        return 1

    if args.dry_run:
        print("[probe] dry run; no commands will be sent")

    try:
        for index, (label, left, right, seconds) in enumerate(steps):
            if index and args.pause > 0:
                print(f"[probe] coast pause {args.pause:.2f}s")
                coast(bridge, dry_run=args.dry_run)
                time.sleep(args.pause)
            print(f"[probe] {label}")
            hold_raw(bridge, left, right, seconds, args)
    except KeyboardInterrupt:
        print("\n[probe] interrupted; coasting")
        return 130
    finally:
        coast(bridge, dry_run=args.dry_run)

    print("[probe] done; treads coasting")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
