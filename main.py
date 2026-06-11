#!/usr/bin/env python3
"""Step 1 master orchestrator for Bun.

Runs host_controller.py as the continuous PD loop, monitors its telemetry for
the Step 1 gate, and performs a bounded reset between successful attempts.
All reset motion uses Bun's validated raw crawl command vocabulary:
forward [-13,-13], backward [13,13], turn frames [23,0] or [0,23].
"""

from __future__ import annotations

import argparse
import json
import os
import random
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

from host_controller import RpcLink


TARGET_DIST_MM = 170.0
X_TOL_MM = 5.0
DIST_TOL_MM = 20.0
ATTEMPTS = 20
MAX_ATTEMPT_S = 30.0
SUCCESS_SAMPLES = 2

CRAWL_PWM = 13
TURN_PWM = 23
FRAME_S = 0.150
RESET_BACKWARD_MIN_S = 0.5
RESET_BACKWARD_MAX_S = 1.5

VISION_URL = "http://127.0.0.1:8080/status"
TELEMETRY_RE = re.compile(
    r"\[tracking\]\s+d=\s*(?P<dist>[+-]?\d+(?:\.\d+)?)mm\s+"
    r"x=\s*(?P<x>[+-]?\d+(?:\.\d+)?)"
)


def utc_ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class JsonlLog:
    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields: object) -> None:
        row = {"ts": utc_ts(), "event": event, **fields}
        print(json.dumps(row, sort_keys=True), flush=True)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")


def read_vision_status(vision_url: str, timeout_s: float = 0.5) -> dict | None:
    try:
        with urlopen(vision_url, timeout=timeout_s) as response:
            return json.loads(response.read().decode("utf-8"))
    except (OSError, URLError, json.JSONDecodeError, ValueError):
        return None


def vision_is_running(vision_url: str) -> bool:
    return read_vision_status(vision_url, timeout_s=0.5) is not None


def launch_stream(log: JsonlLog, vision_url: str) -> subprocess.Popen | None:
    if vision_is_running(vision_url):
        log.write("stream_already_running", vision_url=vision_url)
        return None

    stream_log = Path("logs/step1_master/stream_latest.log")
    stream_log.parent.mkdir(parents=True, exist_ok=True)
    f = stream_log.open("a", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, "-u", "python/brick_vision/stream.py"],
        stdout=f,
        stderr=subprocess.STDOUT,
        text=True,
    )
    log.write("stream_launch", pid=proc.pid, log_path=str(stream_log))

    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if vision_is_running(vision_url):
            log.write("stream_ready", vision_url=vision_url)
            return proc
        if proc.poll() is not None:
            log.write("stream_exited", returncode=proc.returncode, log_path=str(stream_log))
            return proc
        time.sleep(0.25)

    log.write("stream_not_ready", pid=proc.pid, log_path=str(stream_log))
    return proc


def terminate_process(proc: subprocess.Popen | None, timeout_s: float = 2.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout_s)


def launch_host(args: argparse.Namespace) -> subprocess.Popen:
    cmd = [
        sys.executable,
        "-u",
        "host_controller.py",
        "--transport",
        "rpc",
        "--vision-url",
        args.vision_url,
        "--stop-offset",
        str(args.target_dist),
        "--command-policy",
        "step",
        "--crawl-pwm",
        str(args.crawl_pwm),
        "--turn-pwm",
        str(args.turn_pwm),
        "--crawl-frame-ms",
        str(args.frame_ms),
    ]
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
    )


def parse_telemetry(line: str) -> tuple[float, float] | None:
    match = TELEMETRY_RE.search(line)
    if not match:
        return None
    return float(match.group("dist")), float(match.group("x"))


def in_success_gate(dist_mm: float, x_mm: float, target_dist: float,
                    dist_tol: float, x_tol: float) -> bool:
    return abs(dist_mm - target_dist) <= dist_tol and abs(x_mm) <= x_tol


def coast(link: RpcLink, log: JsonlLog, reason: str) -> None:
    link.send(0, 0)
    log.write("coast", reason=reason, raw_command=[0, 0])


def reset_field(link: RpcLink, log: JsonlLog, args: argparse.Namespace) -> None:
    duration_s = random.uniform(args.reset_min_s, args.reset_max_s)
    pivot_cmd = random.choice(([args.turn_pwm, 0], [0, args.turn_pwm]))

    log.write(
        "reset_backward_start",
        duration_s=round(duration_s, 3),
        raw_command=[args.crawl_pwm, args.crawl_pwm],
    )
    link.send(args.crawl_pwm, args.crawl_pwm)
    time.sleep(duration_s)
    coast(link, log, "reset_backward_done")

    before_pivot = read_vision_status(args.vision_url)
    log.write("reset_pre_pivot_vision", status=compact_vision(before_pivot))

    log.write(
        "reset_pivot_start",
        duration_s=args.frame_ms / 1000.0,
        raw_command=pivot_cmd,
    )
    link.send(pivot_cmd[0], pivot_cmd[1])
    time.sleep(args.frame_ms / 1000.0)
    coast(link, log, "reset_pivot_done")

    after_pivot = read_vision_status(args.vision_url)
    log.write("reset_post_pivot_vision", status=compact_vision(after_pivot))


def compact_vision(payload: dict | None) -> dict:
    det = (payload or {}).get("detection") or {}
    spatial = det.get("spatial") or {}
    return {
        "found": bool(det.get("found")),
        "confidence": det.get("confidence"),
        "x_mm": spatial.get("x_mm"),
        "y_mm": spatial.get("y_mm"),
        "dist_mm": spatial.get("dist_mm"),
        "valid": spatial.get("valid"),
    }


def run_attempt(attempt: int, log: JsonlLog, args: argparse.Namespace) -> dict:
    proc = launch_host(args)
    log.write(
        "attempt_start",
        attempt=attempt,
        host_pid=proc.pid,
        target_dist=args.target_dist,
        x_tol=args.x_tol,
        dist_tol=args.dist_tol,
        command_policy="crawl",
        crawl_pwm=args.crawl_pwm,
        turn_pwm=args.turn_pwm,
        frame_ms=args.frame_ms,
    )

    start = time.monotonic()
    gate_hits = 0
    best: dict | None = None

    try:
        assert proc.stdout is not None
        while True:
            if time.monotonic() - start > args.max_attempt_s:
                log.write("attempt_timeout", attempt=attempt, best=best)
                return {"success": False, "reason": "timeout", "best": best}

            line = proc.stdout.readline()
            if line == "":
                if proc.poll() is not None:
                    log.write("host_exit", attempt=attempt, returncode=proc.returncode)
                    return {"success": False, "reason": "host_exit", "best": best}
                time.sleep(0.02)
                continue

            line = line.rstrip()
            log.write("host_stdout", attempt=attempt, line=line)
            parsed = parse_telemetry(line)
            if parsed is None:
                continue

            dist_mm, x_mm = parsed
            dist_err = dist_mm - args.target_dist
            x_err = x_mm
            row = {
                "dist_mm": round(dist_mm, 2),
                "x_mm": round(x_mm, 2),
                "dist_err": round(dist_err, 2),
                "x_err": round(x_err, 2),
            }
            if best is None or (
                abs(dist_err) + abs(x_err)
                < abs(best["dist_err"]) + abs(best["x_err"])
            ):
                best = row

            if in_success_gate(dist_mm, x_mm, args.target_dist,
                               args.dist_tol, args.x_tol):
                gate_hits += 1
                log.write("gate_hit", attempt=attempt, hits=gate_hits, **row)
                if gate_hits >= args.success_samples:
                    log.write("success", attempt=attempt, **row)
                    return {"success": True, "reason": "gate", "best": best, **row}
            else:
                gate_hits = 0
    finally:
        terminate_process(proc)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--attempts", type=int, default=ATTEMPTS)
    parser.add_argument("--target-dist", type=float, default=TARGET_DIST_MM)
    parser.add_argument("--x-tol", type=float, default=X_TOL_MM)
    parser.add_argument("--dist-tol", type=float, default=DIST_TOL_MM)
    parser.add_argument("--success-samples", type=int, default=SUCCESS_SAMPLES)
    parser.add_argument("--max-attempt-s", type=float, default=MAX_ATTEMPT_S)
    parser.add_argument("--vision-url", default=VISION_URL)
    parser.add_argument("--crawl-pwm", type=int, default=CRAWL_PWM)
    parser.add_argument("--turn-pwm", type=int, default=TURN_PWM)
    parser.add_argument("--frame-ms", type=int, default=int(FRAME_S * 1000))
    parser.add_argument("--reset-min-s", type=float, default=RESET_BACKWARD_MIN_S)
    parser.add_argument("--reset-max-s", type=float, default=RESET_BACKWARD_MAX_S)
    parser.add_argument("--no-start-stream", action="store_true")
    args = parser.parse_args()

    log_path = Path("logs/step1_master") / (
        "step1_master_"
        + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + ".jsonl"
    )
    log = JsonlLog(log_path)
    random.seed()

    log.write(
        "run_start",
        attempts=args.attempts,
        target_dist=args.target_dist,
        x_tol=args.x_tol,
        dist_tol=args.dist_tol,
        y_ignored=True,
        crawl_pwm=args.crawl_pwm,
        turn_pwm=args.turn_pwm,
        frame_ms=args.frame_ms,
        official_commands={
            "forward": [-args.crawl_pwm, -args.crawl_pwm],
            "backward": [args.crawl_pwm, args.crawl_pwm],
            "turn_left_or_right": [[args.turn_pwm, 0], [0, args.turn_pwm]],
        },
    )

    stream_proc = None
    if not args.no_start_stream:
        stream_proc = launch_stream(log, args.vision_url)

    link = RpcLink()
    if not link.connect():
        log.write("reset_link_failed")
        terminate_process(stream_proc)
        return 1

    successes = 0
    results = []
    try:
        for attempt in range(1, args.attempts + 1):
            result = run_attempt(attempt, log, args)
            results.append(result)
            if result.get("success"):
                successes += 1
                reset_field(link, log, args)
            else:
                coast(link, log, f"attempt_{attempt}_{result.get('reason')}")

        success_pct = (100.0 * successes / args.attempts) if args.attempts else 0.0
        log.write(
            "run_complete",
            successes=successes,
            attempts=args.attempts,
            success_pct=round(success_pct, 2),
            results=results,
            log_path=str(log_path),
        )
        return 0
    except KeyboardInterrupt:
        log.write("interrupted", successes=successes, attempts_completed=len(results))
        return 130
    finally:
        coast(link, log, "run_final")
        link.close()
        terminate_process(stream_proc)


if __name__ == "__main__":
    raise SystemExit(main())
