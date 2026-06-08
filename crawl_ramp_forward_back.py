#!/usr/bin/env python3
"""Slow capped ramp crawl for Bun's treads.

Validation-only: no 100% kick, no controller changes. Starts at 20% and ramps
to a 50% cap over 2 seconds, then repeats in reverse.
"""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

from host_controller import RpcLink


START_POWER = 12
MAX_POWER = 20
DURATION_S = 1.5
STEP_S = 0.10
PAUSE_S = 0.50


def ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def main() -> int:
    out_dir = Path("logs/motor_diagnostic")
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / (
        "crawl_ramp_forward_back_"
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
        return 1

    def ramp(label: str, sign: int) -> None:
        start = time.monotonic()
        write(
            "ramp_start",
            label=label,
            start_power=START_POWER,
            max_power=MAX_POWER,
            duration_s=DURATION_S,
        )
        try:
            while True:
                elapsed = time.monotonic() - start
                if elapsed >= DURATION_S:
                    break
                frac = max(0.0, min(1.0, elapsed / DURATION_S))
                power = round(START_POWER + (MAX_POWER - START_POWER) * frac)
                left = sign * power
                right = sign * power
                link.send(left, right)
                write(
                    "active",
                    label=label,
                    elapsed_s=round(elapsed, 3),
                    power=power,
                    command=[left, right],
                )
                time.sleep(min(STEP_S, DURATION_S - elapsed))
        finally:
            link.coast()
            write("coast", label=label, command=[0, 0])

    try:
        link.coast()
        time.sleep(PAUSE_S)
        write("test_start", start_power=START_POWER, max_power=MAX_POWER)
        ramp("forward", -1)
        time.sleep(PAUSE_S)
        ramp("backward", 1)
    finally:
        link.coast()
        write("final_coast", command=[0, 0], log_path=str(log_path))
        link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
