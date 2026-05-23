#!/usr/bin/env python3
"""Slow in-place turn test for Bun's treads."""

import argparse
from pathlib import Path
import sys
import time


DEFAULT_ROUTER_SOCKET = "/var/run/arduino-router.sock"
DEFAULT_SECONDS = 3.0
DEFAULT_FLOOR_PERCENT = 1.0
DEFAULT_CEILING_PERCENT = 50.0
DEFAULT_ACTIVE_POWER = 100
DEFAULT_FORWARD_COMMAND_SIGN = -1
DEFAULT_SWAP_LEFT_RIGHT = True
DEFAULT_PERIOD_MS = 250
DEFAULT_MIN_ON_MS = 15
DEFAULT_PAUSE_S = 1.0
RPC_TIMEOUT_S = 2.0


def clamp(value, low, high):
    return max(low, min(high, value))


def require_bridge(router_socket):
    if not Path(router_socket).exists():
        print(f"[turn] router socket not found at {router_socket}", file=sys.stderr)
        return None

    try:
        from arduino.app_utils import Bridge
    except ImportError as exc:
        print(f"[turn] arduino.app_utils is unavailable: {exc}", file=sys.stderr)
        return None

    try:
        Bridge.call("drive_triple", 0, 0, 0, timeout=RPC_TIMEOUT_S)
    except ValueError:
        print("[turn] loaded sketch does not expose drive_triple.", file=sys.stderr)
        return None
    except (TimeoutError, RuntimeError, OSError) as exc:
        print(f"[turn] bridge check failed: {exc}", file=sys.stderr)
        return None

    return Bridge


def coast(bridge, dry_run=False):
    if dry_run:
        return
    try:
        bridge.notify("drive_triple", 0, 0, 0)
    except Exception:
        pass


def command_channels(physical_left, physical_right, swap_left_right):
    if swap_left_right:
        return physical_right, physical_left
    return physical_left, physical_right


def send_pair(bridge, physical_left, physical_right, swap_left_right, dry_run=False):
    if dry_run:
        return
    raw_left, raw_right = command_channels(physical_left, physical_right, swap_left_right)
    bridge.notify("drive_triple", int(raw_left), int(raw_right), 0)


def effective_floor(args):
    period_s = args.period_ms / 1000.0
    min_on_s = args.min_on_ms / 1000.0
    return max(args.floor_percent, 100.0 * min_on_s / period_s)


def duty_at(elapsed_s, duration_s, floor_percent, ceiling_percent):
    ratio = clamp(elapsed_s / duration_s, 0.0, 1.0)
    return floor_percent + (ceiling_percent - floor_percent) * ratio


def physical_turn_speeds(turn, args):
    forward = args.forward_command_sign * args.active_power
    backward = -forward
    if turn == "left":
        return backward, forward
    return forward, backward


def run_turn(bridge, turn, args):
    floor_percent = effective_floor(args)
    period_s = args.period_ms / 1000.0
    min_on_s = args.min_on_ms / 1000.0
    physical_left, physical_right = physical_turn_speeds(turn, args)
    raw_left, raw_right = command_channels(
        physical_left,
        physical_right,
        args.swap_left_right,
    )

    print(
        f"[turn] {turn}: duty {floor_percent:.1f}% -> "
        f"{args.ceiling_percent:.1f}% over {args.seconds:.2f}s "
        f"(physical L={physical_left:+d} R={physical_right:+d}; "
        f"raw L={raw_left:+d} R={raw_right:+d})"
    )

    start_t = time.monotonic()
    next_log_step = -1
    try:
        while True:
            elapsed = time.monotonic() - start_t
            remaining = args.seconds - elapsed
            if remaining <= 0:
                break

            duty = duty_at(elapsed, args.seconds, floor_percent, args.ceiling_percent)
            cycle_s = min(period_s, remaining)
            on_s = clamp(cycle_s * duty / 100.0, min_on_s, cycle_s)
            off_s = max(0.0, cycle_s - on_s)

            log_step = int(elapsed)
            if log_step != next_log_step:
                print(f"[turn] {turn} t={elapsed:4.1f}s duty={duty:4.1f}%")
                next_log_step = log_step

            send_pair(
                bridge,
                physical_left,
                physical_right,
                args.swap_left_right,
                dry_run=args.dry_run,
            )
            time.sleep(on_s)
            coast(bridge, dry_run=args.dry_run)
            if off_s > 0:
                time.sleep(off_s)
    finally:
        coast(bridge, dry_run=args.dry_run)


def main():
    parser = argparse.ArgumentParser(
        description="Slowly ramp Bun through left and right in-place turns."
    )
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--floor-percent", type=float, default=DEFAULT_FLOOR_PERCENT)
    parser.add_argument("--ceiling-percent", type=float, default=DEFAULT_CEILING_PERCENT)
    parser.add_argument("--active-power", type=int, default=DEFAULT_ACTIVE_POWER)
    parser.add_argument("--forward-command-sign", type=int, choices=(-1, 1),
                        default=DEFAULT_FORWARD_COMMAND_SIGN)
    parser.add_argument("--period-ms", type=int, default=DEFAULT_PERIOD_MS)
    parser.add_argument("--min-on-ms", type=int, default=DEFAULT_MIN_ON_MS)
    parser.add_argument("--pause", type=float, default=DEFAULT_PAUSE_S)
    parser.add_argument("--only", choices=("both", "left", "right"), default="both",
                        help="which turn direction to test")
    parser.add_argument("--no-swap-left-right", action="store_true",
                        help="disable the learned command-channel swap")
    parser.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if not 0 < args.floor_percent <= args.ceiling_percent <= 100:
        parser.error("duty percents must satisfy 0 < floor <= ceiling <= 100")
    if not 1 <= args.active_power <= 100:
        parser.error("--active-power must be in 1..100")
    if args.period_ms <= 0 or args.min_on_ms <= 0 or args.min_on_ms > args.period_ms:
        parser.error("--period-ms and --min-on-ms must be positive, with min <= period")
    if args.pause < 0:
        parser.error("--pause must be non-negative")

    args.swap_left_right = DEFAULT_SWAP_LEFT_RIGHT and not args.no_swap_left_right

    bridge = None if args.dry_run else require_bridge(args.router_socket)
    if bridge is None and not args.dry_run:
        return 1

    floor_percent = effective_floor(args)
    if floor_percent > args.ceiling_percent:
        print(
            f"[turn] effective floor {floor_percent:.1f}% exceeds ceiling "
            f"{args.ceiling_percent:.1f}%; lower --min-on-ms or raise --period-ms",
            file=sys.stderr,
        )
        return 2

    print(f"[turn] physical forward sign is {args.forward_command_sign:+d}")
    print(f"[turn] command channels swapped: {args.swap_left_right}")
    if args.dry_run:
        print("[turn] dry run; no commands will be sent")

    directions = ["left", "right"] if args.only == "both" else [args.only]
    try:
        for index, turn in enumerate(directions):
            if index and args.pause > 0:
                print(f"[turn] coast pause {args.pause:.2f}s")
                coast(bridge, dry_run=args.dry_run)
                time.sleep(args.pause)
            run_turn(bridge, turn, args)
    except KeyboardInterrupt:
        print("\n[turn] interrupted; coasting")
        return 130
    finally:
        coast(bridge, dry_run=args.dry_run)

    print("[turn] done; treads coasting")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
