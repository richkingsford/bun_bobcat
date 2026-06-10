#!/usr/bin/env python3
"""Run Bun's live PD alignment loop for a bounded validation window."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from host_controller import (
    CrawlCommandPolicy,
    INVERT_LEFT_MOTOR,
    INVERT_RIGHT_MOTOR,
    LiveBrickVision,
    PdController,
    RpcLink,
    SWAP_LEFT_RIGHT_MOTORS,
)
from vision_autostart import ensure_stream


def ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_jsonl(path: Path, event: str, **fields: object) -> None:
    row = {"ts": ts(), "event": event, **fields}
    print(json.dumps(row, sort_keys=True), flush=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True) + "\n")


def apply_wiring(l_pct: int, r_pct: int) -> tuple[int, int]:
    if SWAP_LEFT_RIGHT_MOTORS:
        l_pct, r_pct = r_pct, l_pct
    if INVERT_LEFT_MOTOR:
        l_pct = -l_pct
    if INVERT_RIGHT_MOTOR:
        r_pct = -r_pct
    return int(l_pct), int(r_pct)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--target-dist", type=float, default=170.0)
    parser.add_argument("--vision-url", default="http://127.0.0.1:8080/status")
    parser.add_argument("--min-confidence", type=int, default=55)
    parser.add_argument("--crawl-pwm", type=int, default=13)
    parser.add_argument("--turn-pwm", type=int, default=23)
    parser.add_argument("--frame-ms", type=int, default=150)
    parser.add_argument("--ctrl-hz", type=float, default=20.0)
    args = parser.parse_args()

    out_dir = Path("logs/step1_master")
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / (
        "align_bounded_"
        + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + ".jsonl"
    )

    vision = LiveBrickVision(args.vision_url, min_confidence=args.min_confidence)
    # This is a bounded run (often ~1.5 s), so the camera MUST be serving before
    # the loop starts or we'd just log "lost" for the whole window. Bring the
    # stream up ourselves (no-op if already running) and wait for it to be ready.
    stream_handle = ensure_stream(
        args.vision_url,
        ready_timeout_s=30.0,
        on_event=lambda ev, **f: write_jsonl(log_path, ev, **f),
    )
    if not stream_handle.ready:
        write_jsonl(log_path, "vision_stream_not_ready", vision_url=args.vision_url)

    pd = PdController(args.target_dist)
    policy = CrawlCommandPolicy(
        straight_pwm=args.crawl_pwm,
        turn_pwm=args.turn_pwm,
        frame_s=max(1, args.frame_ms) / 1000.0,
    )
    link = RpcLink()
    if not link.connect():
        write_jsonl(log_path, "link_failed")
        stream_handle.stop()
        return 1

    best = None
    tick = 0

    try:
        link.send(0, 0)
        before = vision.read()
        write_jsonl(
            log_path,
            "run_start",
            target_dist=args.target_dist,
            duration_s=args.duration,
            crawl_pwm=args.crawl_pwm,
            turn_pwm=args.turn_pwm,
            frame_ms=args.frame_ms,
            min_confidence=args.min_confidence,
            before=before,
        )

        start = time.monotonic()
        next_t = start
        last_t = start
        period = 1.0 / args.ctrl_hz

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= args.duration:
                break

            dt = max(now - last_t, 1e-3)
            last_t = now
            reading = vision.read()

            if reading is None:
                pd.reset()
                policy.reset()
                link.send(0, 0)
                write_jsonl(
                    log_path,
                    "tick",
                    tick=tick,
                    elapsed_s=round(elapsed, 3),
                    state="lost",
                    model_command=[0, 0],
                    raw_command=[0, 0],
                )
            else:
                x_mm, y_fwd_mm, dist_mm = reading
                model_pd = pd.step(x_mm, y_fwd_mm, dist_mm, dt)
                model_command = policy.step(model_pd[0], model_pd[1], now)
                raw_command = apply_wiring(model_command[0], model_command[1])
                link.send(raw_command[0], raw_command[1])

                dist_err = dist_mm - args.target_dist
                score = abs(dist_err) + abs(x_mm)
                rec = {
                    "x_mm": round(x_mm, 2),
                    "dist_mm": round(dist_mm, 2),
                    "dist_err": round(dist_err, 2),
                    "raw_command": list(raw_command),
                }
                if best is None or score < best["score"]:
                    best = {"score": round(score, 2), **rec}

                write_jsonl(
                    log_path,
                    "tick",
                    tick=tick,
                    elapsed_s=round(elapsed, 3),
                    state="tracking",
                    x_mm=round(x_mm, 2),
                    y_fwd_mm=round(y_fwd_mm, 2),
                    dist_mm=round(dist_mm, 2),
                    dist_err=round(dist_err, 2),
                    model_pd=list(model_pd),
                    model_command=list(model_command),
                    raw_command=list(raw_command),
                )

            tick += 1
            next_t += period
            sleep_s = min(
                next_t - time.monotonic(),
                args.duration - (time.monotonic() - start),
            )
            if sleep_s > 0:
                time.sleep(sleep_s)

        link.send(0, 0)
        after = vision.read()
        write_jsonl(
            log_path,
            "run_complete",
            elapsed_s=round(time.monotonic() - start, 3),
            after=after,
            best=best,
            log_path=str(log_path),
        )
    finally:
        try:
            link.send(0, 0)
        finally:
            link.close()
            # Stops only a stream this run started; leaves a shared one alone.
            stream_handle.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
