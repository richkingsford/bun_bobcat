#!/usr/bin/env python3
"""Run Bun's straight crawl forward and backward at raw PWM 13."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

from host_controller import RpcLink


POWER = 13
DURATION_S = 1.5
PAUSE_S = 0.5


def ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def main() -> int:
    out_dir = Path("logs/motor_diagnostic")
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / (
        "crawl_constant13_forward_back_"
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
        write("test_start", power=POWER, duration_s=DURATION_S)
        time.sleep(PAUSE_S)

        for label, command in (
            ("forward", [-POWER, -POWER]),
            ("backward", [POWER, POWER]),
        ):
            write("active", label=label, raw_command=command, duration_s=DURATION_S)
            link.send(command[0], command[1])
            time.sleep(DURATION_S)
            link.send(0, 0)
            write("coast", label=label, raw_command=[0, 0])
            time.sleep(PAUSE_S)
    finally:
        try:
            link.send(0, 0)
            write("final_coast", raw_command=[0, 0], log_path=str(log_path))
        finally:
            link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
