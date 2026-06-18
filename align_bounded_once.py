#!/usr/bin/env python3
"""Run Bun's live PD alignment loop for a bounded validation window."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from statistics import median
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
    VISION_MIN_CONFIDENCE,
)
from vision_autostart import ensure_stream, vision_is_running


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


def in_success_gate(reading, target_dist: float, x_tol: float,
                    dist_tol: float) -> bool:
    if reading is None:
        return False
    x_mm, _y_fwd_mm, dist_mm = reading
    return abs(x_mm) <= x_tol and abs(dist_mm - target_dist) <= dist_tol


def predictive_stop_reason(reading, target_dist: float, x_tol: float,
                           dist_tol: float, dist_velocity_mmps,
                           lookahead_s: float, min_speed_mmps: float):
    if reading is None or dist_velocity_mmps is None or lookahead_s <= 0:
        return None, None

    x_mm, _y_fwd_mm, dist_mm = reading
    if abs(x_mm) > x_tol:
        return None, None

    low = target_dist - dist_tol
    high = target_dist + dist_tol
    projected = dist_mm + dist_velocity_mmps * lookahead_s

    if low <= dist_mm <= high:
        return "current_gate", projected
    if dist_mm > high and dist_velocity_mmps < -min_speed_mmps:
        if projected <= high:
            return "predictive_from_far", projected
    if dist_mm < low and dist_velocity_mmps > min_speed_mmps:
        if projected >= low:
            return "predictive_from_close", projected
    return None, projected


def score_reading(reading, target_dist: float, x_tol: float) -> float | None:
    if reading is None:
        return None
    x_mm, _y_fwd_mm, dist_mm = reading
    return max(0.0, abs(x_mm) - x_tol) + abs(dist_mm - target_dist)


def staged_x_tol(dist_mm: float, final_x_tol: float, mid_x_tol: float,
                 far_x_tol: float, mid_dist_mm: float,
                 final_dist_mm: float) -> float:
    if dist_mm > mid_dist_mm:
        return far_x_tol
    if dist_mm > final_dist_mm:
        return mid_x_tol
    return final_x_tol


def acquire_stable_vision(vision: LiveBrickVision, samples: int,
                          timeout_s: float, interval_s: float,
                          stable_s: float, dist_spread_mm: float,
                          x_spread_mm: float,
                          log_path: Path):
    """Wait for consecutive valid brick readings before allowing motion."""
    need = max(1, int(samples))
    deadline = time.monotonic() + max(0.0, timeout_s)
    window = deque()
    last_reading = None
    polls = 0

    while time.monotonic() < deadline:
        polls += 1
        now = time.monotonic()
        reading = vision.read()
        if reading is None:
            window.clear()
        else:
            last_reading = reading
            window.append((now, reading))
            while len(window) > 1 and now - window[1][0] >= stable_s:
                window.popleft()

            if len(window) >= need and now - window[0][0] >= stable_s:
                readings = [item[1] for item in window]
                xs = [float(item[0]) for item in readings]
                ds = [float(item[2]) for item in readings]
                x_spread = max(xs) - min(xs)
                dist_spread = max(ds) - min(ds)
                if x_spread <= x_spread_mm and dist_spread <= dist_spread_mm:
                    stable_reading = (
                        float(median(xs)),
                        float(median(float(item[1]) for item in readings)),
                        float(median(ds)),
                    )
                    write_jsonl(
                        log_path,
                        "vision_acquired",
                        polls=polls,
                        samples=need,
                        window_samples=len(window),
                        window_s=round(now - window[0][0], 3),
                        x_spread_mm=round(x_spread, 2),
                        dist_spread_mm=round(dist_spread, 2),
                        reading=stable_reading,
                    )
                    return stable_reading
                write_jsonl(
                    log_path,
                    "vision_window_rejected",
                    polls=polls,
                    samples=need,
                    window_samples=len(window),
                    x_spread_mm=round(x_spread, 2),
                    dist_spread_mm=round(dist_spread, 2),
                )
                window.popleft()
        time.sleep(interval_s)

    write_jsonl(
        log_path,
        "vision_acquire_failed",
        polls=polls,
        samples=need,
        last_reading=last_reading,
    )
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--target-dist", type=float, default=170.0)
    parser.add_argument("--x-tol", type=float, default=8.0)
    parser.add_argument("--dist-tol", type=float, default=20.0)
    parser.add_argument("--mid-x-tol", type=float, default=12.0)
    parser.add_argument("--far-x-tol", type=float, default=25.0)
    parser.add_argument("--mid-dist-mm", type=float, default=250.0)
    parser.add_argument("--final-dist-mm", type=float, default=190.0)
    parser.add_argument("--vision-url", default="http://127.0.0.1:8080/status")
    parser.add_argument(
        "--reuse-stream",
        action="store_true",
        help="Assume the caller owns the camera stream and leave it running.",
    )
    parser.add_argument("--min-confidence", type=int, default=VISION_MIN_CONFIDENCE)
    parser.add_argument("--vision-timeout-ms", type=int, default=150)
    parser.add_argument("--acquire-samples", type=int, default=3)
    parser.add_argument("--acquire-timeout", type=float, default=10.0)
    parser.add_argument("--acquire-stable-ms", type=int, default=500)
    parser.add_argument("--acquire-dist-spread-mm", type=float, default=60.0)
    parser.add_argument("--acquire-x-spread-mm", type=float, default=35.0)
    parser.add_argument("--predictive-stop-s", type=float, default=0.65)
    parser.add_argument("--min-predictive-speed", type=float, default=20.0)
    parser.add_argument("--max-velocity-mmps", type=float, default=600.0)
    parser.add_argument("--settle-ms", type=int, default=600)
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

    vision = LiveBrickVision(
        args.vision_url,
        min_confidence=args.min_confidence,
        timeout_s=max(1, args.vision_timeout_ms) / 1000.0,
    )
    # This is a bounded run (often ~1.5 s), so the camera MUST be serving before
    # the loop starts or we'd just log "lost" for the whole window. A repeated
    # trial harness can own one shared stream and pass --reuse-stream; standalone
    # runs still bring the stream up themselves.
    stream_handle = None
    if args.reuse_stream:
        if vision_is_running(args.vision_url):
            write_jsonl(log_path, "stream_reused", vision_url=args.vision_url)
        else:
            write_jsonl(log_path, "vision_stream_not_ready", vision_url=args.vision_url)
    else:
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
    success = False
    stop_reason = None
    last_valid_t = None
    last_valid_dist = None
    dist_velocity_mmps = None

    try:
        link.send(0, 0)
        before = acquire_stable_vision(
            vision,
            samples=args.acquire_samples,
            timeout_s=args.acquire_timeout,
            interval_s=max(0.05, 1.0 / args.ctrl_hz),
            stable_s=max(0.0, args.acquire_stable_ms) / 1000.0,
            dist_spread_mm=args.acquire_dist_spread_mm,
            x_spread_mm=args.acquire_x_spread_mm,
            log_path=log_path,
        )
        if before is None:
            link.send(0, 0)
            write_jsonl(log_path, "run_aborted_no_vision")
            return 2

        write_jsonl(
            log_path,
            "run_start",
            target_dist=args.target_dist,
            x_tol=args.x_tol,
            dist_tol=args.dist_tol,
            mid_x_tol=args.mid_x_tol,
            far_x_tol=args.far_x_tol,
            mid_dist_mm=args.mid_dist_mm,
            final_dist_mm=args.final_dist_mm,
            duration_s=args.duration,
            crawl_pwm=args.crawl_pwm,
            turn_pwm=args.turn_pwm,
            frame_ms=args.frame_ms,
            min_confidence=args.min_confidence,
            vision_timeout_ms=args.vision_timeout_ms,
            predictive_stop_s=args.predictive_stop_s,
            min_predictive_speed=args.min_predictive_speed,
            settle_ms=args.settle_ms,
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
                if last_valid_t is not None:
                    valid_dt = max(now - last_valid_t, 1e-3)
                    raw_velocity = (dist_mm - last_valid_dist) / valid_dt
                    if abs(raw_velocity) > args.max_velocity_mmps:
                        pd.reset()
                        policy.reset()
                        link.send(0, 0)
                        write_jsonl(
                            log_path,
                            "tick",
                            tick=tick,
                            elapsed_s=round(elapsed, 3),
                            state="rejected_jump",
                            x_mm=round(x_mm, 2),
                            dist_mm=round(dist_mm, 2),
                            previous_dist_mm=round(last_valid_dist, 2),
                            raw_velocity_mmps=round(raw_velocity, 2),
                            max_velocity_mmps=args.max_velocity_mmps,
                            model_command=[0, 0],
                            raw_command=[0, 0],
                        )
                        tick += 1
                        next_t += period
                        sleep_s = min(
                            next_t - time.monotonic(),
                            args.duration - (time.monotonic() - start),
                        )
                        if sleep_s > 0:
                            time.sleep(sleep_s)
                        continue
                    if abs(raw_velocity) <= args.max_velocity_mmps:
                        if dist_velocity_mmps is None:
                            dist_velocity_mmps = raw_velocity
                        else:
                            dist_velocity_mmps = (
                                0.65 * dist_velocity_mmps
                                + 0.35 * raw_velocity
                            )
                last_valid_t = now
                last_valid_dist = dist_mm

                # Inside the QA margin, lateral error is good enough; do not
                # spend turn frames chasing measurement noise around zero.
                active_x_tol = staged_x_tol(
                    dist_mm,
                    args.x_tol,
                    args.mid_x_tol,
                    args.far_x_tol,
                    args.mid_dist_mm,
                    args.final_dist_mm,
                )
                if abs(x_mm) <= active_x_tol:
                    x_control_mm = 0.0
                else:
                    x_control_mm = math.copysign(abs(x_mm) - active_x_tol, x_mm)
                dist_err = dist_mm - args.target_dist
                dist_in_gate = abs(dist_err) <= args.dist_tol
                final_x_trim = dist_in_gate and abs(x_mm) > args.x_tol
                control_dist_mm = args.target_dist if final_x_trim else dist_mm
                score = score_reading(reading, args.target_dist, args.x_tol)
                gate = in_success_gate(
                    reading, args.target_dist, args.x_tol, args.dist_tol
                )
                reason, projected_dist = predictive_stop_reason(
                    reading,
                    args.target_dist,
                    args.x_tol,
                    args.dist_tol,
                    dist_velocity_mmps,
                    args.predictive_stop_s,
                    args.min_predictive_speed,
                )
                rec = {
                    "x_mm": round(x_mm, 2),
                    "x_control_mm": round(x_control_mm, 2),
                    "active_x_tol": round(active_x_tol, 2),
                    "dist_mm": round(dist_mm, 2),
                    "dist_err": round(dist_err, 2),
                    "control_dist_mm": round(control_dist_mm, 2),
                    "final_x_trim": final_x_trim,
                    "in_gate": gate,
                    "dist_velocity_mmps": (
                        None if dist_velocity_mmps is None
                        else round(dist_velocity_mmps, 2)
                    ),
                    "projected_dist_mm": (
                        None if projected_dist is None
                        else round(projected_dist, 2)
                    ),
                }
                if best is None or score < best["score"]:
                    best = {"score": round(score, 2), **rec}

                if reason is not None:
                    stop_reason = reason
                    success = reason == "current_gate"
                    link.send(0, 0)
                    write_jsonl(
                        log_path,
                        "stop_gate",
                        reason=reason,
                        tick=tick,
                        elapsed_s=round(elapsed, 3),
                        x_mm=round(x_mm, 2),
                        dist_mm=round(dist_mm, 2),
                        dist_err=round(dist_err, 2),
                        dist_velocity_mmps=(
                            None if dist_velocity_mmps is None
                            else round(dist_velocity_mmps, 2)
                        ),
                        projected_dist_mm=(
                            None if projected_dist is None
                            else round(projected_dist, 2)
                        ),
                    )
                    break

                model_pd = pd.step(x_control_mm, y_fwd_mm, control_dist_mm, dt)
                model_command = policy.step(model_pd[0], model_pd[1], now)
                raw_command = apply_wiring(model_command[0], model_command[1])
                link.send(raw_command[0], raw_command[1])
                rec["raw_command"] = list(raw_command)
                if best is not None and best.get("dist_mm") == rec["dist_mm"]:
                    best["raw_command"] = list(raw_command)

                write_jsonl(
                    log_path,
                    "tick",
                    tick=tick,
                    elapsed_s=round(elapsed, 3),
                    state="tracking",
                    x_mm=round(x_mm, 2),
                    x_control_mm=round(x_control_mm, 2),
                    active_x_tol=round(active_x_tol, 2),
                    y_fwd_mm=round(y_fwd_mm, 2),
                    dist_mm=round(dist_mm, 2),
                    dist_err=round(dist_err, 2),
                    control_dist_mm=round(control_dist_mm, 2),
                    final_x_trim=final_x_trim,
                    in_gate=gate,
                    dist_velocity_mmps=(
                        None if dist_velocity_mmps is None
                        else round(dist_velocity_mmps, 2)
                    ),
                    projected_dist_mm=(
                        None if projected_dist is None
                        else round(projected_dist, 2)
                    ),
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
        settle_s = max(0, args.settle_ms) / 1000.0
        if settle_s:
            time.sleep(settle_s)
        after = vision.read()
        after_gate = in_success_gate(
            after, args.target_dist, args.x_tol, args.dist_tol
        )
        success = success or after_gate
        if after_gate:
            write_jsonl(log_path, "success_after_settle", after=after)
        write_jsonl(
            log_path,
            "run_complete",
            elapsed_s=round(time.monotonic() - start, 3),
            after=after,
            after_in_gate=after_gate,
            best=best,
            stop_reason=stop_reason,
            success=success,
            log_path=str(log_path),
        )
    finally:
        try:
            link.send(0, 0)
        finally:
            link.close()
            # Stops only a stream this run started; shared streams are owned by
            # the parent harness.
            if stream_handle is not None:
                stream_handle.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
