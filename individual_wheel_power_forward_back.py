#!/usr/bin/env python3
"""Run each raw tread channel forward and backward at a requested PWM."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from host_controller import RpcLink


def ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--power", type=int, required=True)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--pause", type=float, default=0.5)
    args = parser.parse_args()

    power = max(0, min(100, int(args.power)))
    tests = (
        ("raw_left_forward", [-power, 0]),
        ("raw_left_backward", [power, 0]),
        ("raw_right_forward", [0, -power]),
        ("raw_right_backward", [0, power]),
    )

    out_dir = Path("logs/motor_diagnostic")
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / (
        f"individual_wheel_{power}_forward_back_"
        + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + ".jsonl"
    )

    def write(event: str, **fields: object) -> None:
        row = {"ts": ts(), "event": event, **fields}
        print(json.dumps(row, sort_keys=True), flush=True)
        with log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")

    link = RpcLink()
    if not link.connect():
        write("link_failed")
        return 1

    try:
        link.send(0, 0)
        write(
            "test_start",
            power=power,
            duration_s=args.duration,
            tests=[
                {"label": label, "raw_command": command}
                for label, command in tests
            ],
        )
        time.sleep(args.pause)

        for label, command in tests:
            write("active", label=label, raw_command=command, duration_s=args.duration)
            link.send(command[0], command[1])
            time.sleep(args.duration)
            link.send(0, 0)
            write("coast", label=label, raw_command=[0, 0])
            time.sleep(args.pause)
    finally:
        try:
            link.send(0, 0)
            write("final_coast", raw_command=[0, 0], log_path=str(log_path))
        finally:
            link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
