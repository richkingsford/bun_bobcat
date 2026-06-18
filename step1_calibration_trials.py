#!/usr/bin/env python3
"""Run repeated Step 1 calibration attempts and summarize the evidence."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from host_controller import RpcLink
from vision_autostart import ensure_stream


def ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_jsonl(path: Path, event: str, **fields: object) -> None:
    row = {"ts": ts(), "event": event, **fields}
    print(json.dumps(row, sort_keys=True), flush=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True) + "\n")


def reading_dict(reading) -> dict | None:
    if not reading:
        return None
    try:
        return {
            "x_mm": round(float(reading[0]), 2),
            "dist_mm": round(float(reading[2]), 2),
        }
    except (TypeError, ValueError, IndexError):
        return None


def command_key(raw_command) -> str:
    if not isinstance(raw_command, list) or len(raw_command) < 2:
        return "?"
    return f"{raw_command[0]},{raw_command[1]}"


def classify(summary: dict, args: argparse.Namespace) -> str:
    if summary.get("success"):
        return "success"
    if summary.get("aborted_no_vision"):
        return "no_vision_acquire"

    total = max(1, summary.get("total_ticks", 0))
    lost_pct = summary.get("lost_ticks", 0) / total
    best = summary.get("best") or {}
    before = summary.get("before") or {}

    if summary.get("tracking_ticks", 0) == 0:
        return "no_tracking"
    if lost_pct >= 0.65:
        return "vision_drop"
    if abs(best.get("x_mm", 0.0)) > args.far_x_tol:
        return "x_not_centered"

    best_dist = best.get("dist_mm")
    start_dist = before.get("dist_mm")
    if best_dist is None or start_dist is None:
        return "incomplete_data"

    low = args.target_dist - args.dist_tol
    high = args.target_dist + args.dist_tol
    if best_dist < low:
        return "overshot_close"
    if start_dist > high and best_dist > start_dist - args.min_closure_mm:
        return "too_slow"
    if best_dist > high:
        return "timeout_far"
    return "missed_gate"


def summarize_attempt(attempt: int, child_events: list[dict],
                      returncode: int, args: argparse.Namespace) -> dict:
    ticks = [e for e in child_events if e.get("event") == "tick"]
    tracking = [e for e in ticks if e.get("state") == "tracking"]
    lost = [e for e in ticks if e.get("state") == "lost"]
    run_start = next((e for e in child_events if e.get("event") == "run_start"), {})
    run_complete = next(
        (e for e in reversed(child_events) if e.get("event") == "run_complete"),
        {},
    )
    stop_gate = next(
        (e for e in reversed(child_events) if e.get("event") == "stop_gate"),
        {},
    )

    commands = Counter(
        command_key(e.get("raw_command"))
        for e in tracking
        if e.get("raw_command") is not None
    )

    summary = {
        "attempt": attempt,
        "returncode": returncode,
        "success": bool(run_complete.get("success")),
        "before": reading_dict(run_start.get("before")),
        "after": reading_dict(run_complete.get("after")),
        "best": run_complete.get("best"),
        "stop_reason": run_complete.get("stop_reason") or stop_gate.get("reason"),
        "child_log_path": run_complete.get("log_path"),
        "total_ticks": len(ticks),
        "tracking_ticks": len(tracking),
        "lost_ticks": len(lost),
        "commands": dict(commands),
        "aborted_no_vision": any(
            e.get("event") == "run_aborted_no_vision" for e in child_events
        ),
    }
    summary["classification"] = classify(summary, args)
    return summary


def run_child_attempt(attempt: int, args: argparse.Namespace,
                      log_path: Path) -> dict:
    cmd = [
        sys.executable,
        "-u",
        "align_bounded_once.py",
        "--duration", str(args.duration),
        "--target-dist", str(args.target_dist),
        "--x-tol", str(args.x_tol),
        "--dist-tol", str(args.dist_tol),
        "--vision-url", args.vision_url,
        "--mid-x-tol", str(args.mid_x_tol),
        "--far-x-tol", str(args.far_x_tol),
        "--mid-dist-mm", str(args.mid_dist_mm),
        "--final-dist-mm", str(args.final_dist_mm),
        "--crawl-pwm", str(args.crawl_pwm),
        "--turn-pwm", str(args.turn_pwm),
        "--frame-ms", str(args.frame_ms),
        "--min-confidence", str(args.min_confidence),
        "--vision-timeout-ms", str(args.vision_timeout_ms),
        "--acquire-samples", str(args.acquire_samples),
        "--acquire-timeout", str(args.acquire_timeout),
        "--acquire-stable-ms", str(args.acquire_stable_ms),
        "--acquire-dist-spread-mm", str(args.acquire_dist_spread_mm),
        "--acquire-x-spread-mm", str(args.acquire_x_spread_mm),
        "--predictive-stop-s", str(args.predictive_stop_s),
        "--settle-ms", str(args.settle_ms),
    ]
    if not args.per_attempt_stream:
        cmd.append("--reuse-stream")
    write_jsonl(log_path, "attempt_start", attempt=attempt, cmd=cmd)
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    child_events = []
    assert proc.stdout is not None
    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            write_jsonl(log_path, "child_stdout", attempt=attempt, line=line)
            continue
        child_events.append(event)
        write_jsonl(log_path, "child_event", attempt=attempt, child=event)

    returncode = proc.wait()
    summary = summarize_attempt(attempt, child_events, returncode, args)
    write_jsonl(log_path, "attempt_complete", **summary)
    return summary


def reset_after_win(args: argparse.Namespace, log_path: Path,
                    attempt: int) -> bool:
    link = RpcLink()
    if not link.connect():
        write_jsonl(log_path, "reset_failed", attempt=attempt,
                    reason="rpc_connect_failed")
        return False
    try:
        write_jsonl(
            log_path,
            "reset_backward_start",
            attempt=attempt,
            duration_s=args.reset_back_s,
            raw_command=[args.crawl_pwm, args.crawl_pwm],
        )
        link.send(args.crawl_pwm, args.crawl_pwm)
        time.sleep(args.reset_back_s)
        link.send(0, 0)
        write_jsonl(log_path, "reset_backward_done", attempt=attempt)
        return True
    finally:
        try:
            link.send(0, 0)
        finally:
            link.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--target-dist", type=float, default=170.0)
    parser.add_argument("--x-tol", type=float, default=8.0)
    parser.add_argument("--dist-tol", type=float, default=20.0)
    parser.add_argument("--vision-url", default="http://127.0.0.1:8080/status")
    parser.add_argument(
        "--per-attempt-stream",
        action="store_true",
        help="Use the older single-attempt camera lifecycle for debugging.",
    )
    parser.add_argument("--stream-warmup-s", type=float, default=2.0)
    parser.add_argument("--mid-x-tol", type=float, default=12.0)
    parser.add_argument("--far-x-tol", type=float, default=25.0)
    parser.add_argument("--mid-dist-mm", type=float, default=250.0)
    parser.add_argument("--final-dist-mm", type=float, default=190.0)
    parser.add_argument("--crawl-pwm", type=int, default=13)
    parser.add_argument("--turn-pwm", type=int, default=23)
    parser.add_argument("--frame-ms", type=int, default=150)
    parser.add_argument("--min-confidence", type=int, default=40)
    parser.add_argument("--vision-timeout-ms", type=int, default=150)
    parser.add_argument("--acquire-samples", type=int, default=3)
    parser.add_argument("--acquire-timeout", type=float, default=12.0)
    parser.add_argument("--acquire-stable-ms", type=int, default=500)
    parser.add_argument("--acquire-dist-spread-mm", type=float, default=60.0)
    parser.add_argument("--acquire-x-spread-mm", type=float, default=35.0)
    parser.add_argument("--predictive-stop-s", type=float, default=0.65)
    parser.add_argument("--settle-ms", type=int, default=600)
    parser.add_argument("--reset-back-s", type=float, default=1.5)
    parser.add_argument("--min-closure-mm", type=float, default=25.0)
    args = parser.parse_args()

    out_dir = Path("logs/step1_calibration")
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / (
        "step1_calibration_"
        + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + ".jsonl"
    )

    summaries = []
    write_jsonl(log_path, "run_start", args=vars(args))
    stream_handle = None
    try:
        if not args.per_attempt_stream:
            stream_handle = ensure_stream(
                args.vision_url,
                ready_timeout_s=45.0,
                on_event=lambda ev, **f: write_jsonl(log_path, ev, **f),
            )
            if not stream_handle.ready:
                write_jsonl(
                    log_path,
                    "shared_stream_not_ready",
                    vision_url=args.vision_url,
                )
            elif stream_handle.started and args.stream_warmup_s > 0:
                write_jsonl(
                    log_path,
                    "shared_stream_warmup",
                    duration_s=args.stream_warmup_s,
                )
                time.sleep(args.stream_warmup_s)

        for attempt in range(1, max(1, args.attempts) + 1):
            summary = run_child_attempt(attempt, args, log_path)
            summaries.append(summary)
            if summary.get("success"):
                reset_after_win(args, log_path, attempt)
    finally:
        if stream_handle is not None:
            stream_handle.stop()

        wins = sum(1 for s in summaries if s.get("success"))
        classes = Counter(s.get("classification") for s in summaries)
        final = {
            "attempts": len(summaries),
            "wins": wins,
            "success_pct": round(100.0 * wins / max(1, len(summaries)), 1),
            "classifications": dict(classes),
            "summaries": summaries,
            "log_path": str(log_path),
        }
        write_jsonl(log_path, "run_complete", **final)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
