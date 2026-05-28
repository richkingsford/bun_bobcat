#!/usr/bin/env python3
"""Bun step-1 brick alignment game.

This script tries to place Bun at the frozen brick win pose as many times as
possible. Every movement is gated by live brick vision, every movement parks
after a short pulse, and every attempt is logged as JSONL for later analysis.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
import json
from pathlib import Path
import random
import statistics
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


STATUS_URL = "http://127.0.0.1:8080/status"
TARGET_FILE = REPO_ROOT / "python" / "brick_vision" / "step1_target.json"
LOG_DIR = REPO_ROOT / "logs" / "step1"

DEFAULT_TARGET = {"x": 0, "y": -60, "dist": 261}
WIN_GATE_MM = 10
ATTEMPTS = 50
ATTEMPT_SECONDS = 7.0
MIN_CONFIDENCE = 10

COMMAND_PERIOD_S = 0.04
SAMPLE_COUNT = 5
SAMPLE_INTERVAL_S = 0.04
READ_TIMEOUT_S = 2.0

TREAD_POWER = 100
DRIVE_POWER = 100
MAST_POWER = 100
SPIN_MIN_S = 0.10
SPIN_MAX_S = 0.30
APPROACH_MIN_S = 0.12
APPROACH_MAX_S = 0.26
BACKUP_MIN_S = 0.12
BACKUP_MAX_S = 0.26
MAST_MIN_S = 0.12
MAST_MAX_S = 0.28
SETTLE_S = 1.0
APPROACH_X_LOW_MM = -20
APPROACH_X_HIGH_OFFSET_MM = -8
FAR_DIST_MM = 35


@dataclass
class Reading:
    x: int
    y: int
    dist: int
    confidence: int
    source: str


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def load_target() -> dict[str, int]:
    if TARGET_FILE.exists():
        with TARGET_FILE.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        target = payload.get("target_mm") or {}
        return {
            "x": int(target.get("x", DEFAULT_TARGET["x"])),
            "y": int(target.get("y", DEFAULT_TARGET["y"])),
            "dist": int(target.get("dist", DEFAULT_TARGET["dist"])),
        }
    return dict(DEFAULT_TARGET)


class JsonlLog:
    def __init__(self) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
        self.path = LOG_DIR / f"step1_{stamp}.jsonl"

    def write(self, event: str, **fields: object) -> None:
        row = {"ts": now_iso(), "event": event, **fields}
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


class Vision:
    def __init__(self, status_url: str, min_confidence: int) -> None:
        self.status_url = status_url.rstrip("/")
        if not self.status_url.endswith("/status"):
            self.status_url += "/status"
        self.min_confidence = min_confidence

    def read_one(self) -> Reading | None:
        try:
            with urlopen(self.status_url, timeout=0.45) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (OSError, URLError, ValueError, json.JSONDecodeError):
            return None

        detection = payload.get("detection") or {}
        spatial = detection.get("spatial") or {}
        confidence = int(detection.get("confidence") or 0)
        if (
            not detection.get("found")
            or confidence < self.min_confidence
            or not spatial.get("valid")
        ):
            return None

        try:
            return Reading(
                x=int(spatial["x_mm"]),
                y=int(spatial["y_mm"]),
                dist=int(spatial["dist_mm"]),
                confidence=confidence,
                source=str(spatial.get("dist_source") or "?"),
            )
        except (KeyError, TypeError, ValueError):
            return None

    def read_average(self) -> Reading | None:
        readings: list[Reading] = []
        deadline = time.monotonic() + READ_TIMEOUT_S
        while len(readings) < SAMPLE_COUNT and time.monotonic() < deadline:
            reading = self.read_one()
            if reading is not None:
                readings.append(reading)
            if len(readings) < SAMPLE_COUNT:
                time.sleep(SAMPLE_INTERVAL_S)

        if not readings:
            return None

        source_counts: dict[str, int] = {}
        for reading in readings:
            source_counts[reading.source] = source_counts.get(reading.source, 0) + 1
        dominant_source = max(source_counts, key=source_counts.get)
        source_readings = [r for r in readings if r.source == dominant_source]

        median_dist = statistics.median(r.dist for r in source_readings)
        kept = [r for r in source_readings if abs(r.dist - median_dist) <= 45]
        if not kept:
            kept = source_readings
        if len(kept) < 3:
            return None

        x_values = [r.x for r in kept]
        y_values = [r.y for r in kept]
        dist_values = [r.dist for r in kept]
        if (
            max(x_values) - min(x_values) > 140
            or max(y_values) - min(y_values) > 140
            or max(dist_values) - min(dist_values) > 140
        ):
            return None

        median_x = statistics.median(x_values)
        median_dist = statistics.median(dist_values)
        if abs(median_x) > 220 or median_dist > 700:
            return None

        return Reading(
            x=int(round(statistics.fmean(x_values))),
            y=int(round(statistics.fmean(y_values))),
            dist=int(round(statistics.fmean(dist_values))),
            confidence=int(round(statistics.fmean(r.confidence for r in kept))),
            source="avg:" + dominant_source,
        )


class Motion:
    def __init__(self, dry_run: bool) -> None:
        self.dry_run = dry_run
        self.bridge = None

    def connect(self) -> bool:
        if self.dry_run:
            print("[main] dry run: no motor commands will be sent")
            return True
        try:
            from arduino.app_utils import Bridge
        except ImportError as exc:
            print(f"[main] arduino bridge unavailable: {exc}")
            return False
        try:
            Bridge.call("drive_triple", 0, 0, 0, timeout=2)
        except Exception as exc:
            print(f"[main] drive_triple unavailable: {exc}")
            return False
        self.bridge = Bridge
        self.park()
        return True

    def park(self) -> None:
        if self.dry_run or self.bridge is None:
            return
        try:
            self.bridge.notify("drive_triple", 0, 0, 0)
            self.bridge.notify("mast", 0)
        except Exception:
            pass

    def pulse(self, left: int, right: int, mast: int, seconds: float) -> None:
        if self.dry_run:
            time.sleep(min(seconds, 0.05))
            return

        assert self.bridge is not None
        end_t = time.monotonic() + seconds
        try:
            while time.monotonic() < end_t:
                self.bridge.notify("drive_triple", int(left), int(right), int(mast))
                remaining = end_t - time.monotonic()
                time.sleep(min(COMMAND_PERIOD_S, max(0.0, remaining)))
        finally:
            self.park()


def errors(reading: Reading, target: dict[str, int]) -> dict[str, int]:
    return {
        "x": reading.x - target["x"],
        "y": reading.y - target["y"],
        "dist": reading.dist - target["dist"],
    }


def won(reading: Reading, target: dict[str, int]) -> bool:
    err = errors(reading, target)
    return all(abs(err[key]) <= WIN_GATE_MM for key in ("x", "y", "dist"))


def choose_action(reading: Reading, target: dict[str, int]) -> tuple[str, int, int, int, float]:
    err = errors(reading, target)

    if abs(err["y"]) > 18:
        seconds = clamp(abs(err["y"]) * 0.006, 0.18, 0.45)
        if err["y"] < 0:
            return "mast_up", 0, 0, -MAST_POWER, seconds
        return "mast_down", 0, 0, MAST_POWER, seconds

    if err["dist"] > FAR_DIST_MM:
        approach_x_low = target["x"] - 40
        approach_x_high = target["x"] + 40
        if reading.x > approach_x_high:
            seconds = clamp((reading.x - approach_x_high) * 0.004, 0.15, 0.60)
            return "lane_x_down", TREAD_POWER, 0, 0, seconds
        if reading.x < approach_x_low:
            seconds = clamp((approach_x_low - reading.x) * 0.0012, 0.08, 0.14)
            return "lane_x_up", -TREAD_POWER, 0, 0, seconds

        seconds = clamp(abs(err["dist"]) * 0.0008, 0.14, 0.28)
        return "approach_forward", DRIVE_POWER, DRIVE_POWER, 0, seconds

    if abs(err["x"]) > WIN_GATE_MM:
        seconds = clamp(abs(err["x"]) * 0.003, 0.08, 0.16)
        if err["x"] > 0:
            return "final_x_down", TREAD_POWER, 0, 0, seconds
        return "final_x_up", -TREAD_POWER, 0, 0, seconds

    if abs(err["y"]) > WIN_GATE_MM:
        seconds = clamp(abs(err["y"]) * 0.006, MAST_MIN_S, MAST_MAX_S)
        if err["y"] < 0:
            return "mast_up", 0, 0, -MAST_POWER, seconds
        return "mast_down", 0, 0, MAST_POWER, seconds

    if abs(err["dist"]) > WIN_GATE_MM:
        if err["dist"] > 0:
            seconds = clamp(abs(err["dist"]) * 0.0008, 0.10, 0.16)
            return "approach_forward", DRIVE_POWER, DRIVE_POWER, 0, seconds
        seconds = clamp(abs(err["dist"]) * 0.0015, BACKUP_MIN_S, BACKUP_MAX_S)
        return "back_up", -DRIVE_POWER, -DRIVE_POWER, 0, seconds

    return "park", 0, 0, 0, SETTLE_S


def reset_after_win(
    attempt: int,
    wins: int,
    motion: Motion,
    vision: Vision,
    log: JsonlLog,
) -> bool:
    backup_s = random.uniform(0.3, 1.0)
    turn_left = random.choice([True, False])
    turn_s = random.uniform(0.06, 0.14)
    turn = (TREAD_POWER, -TREAD_POWER) if turn_left else (-TREAD_POWER, TREAD_POWER)

    print(
        f"[main] reset after win {wins}: back {backup_s:.2f}s, "
        f"turn {'left' if turn_left else 'right'} {turn_s:.2f}s"
    )
    log.write(
        "reset_start",
        attempt=attempt,
        wins=wins,
        backup_s=round(backup_s, 3),
        turn_s=round(turn_s, 3),
        turn="left" if turn_left else "right",
    )

    motion.pulse(-DRIVE_POWER, -DRIVE_POWER, 0, backup_s)
    time.sleep(SETTLE_S)
    if vision.read_average() is None:
        log.write("reset_lost_vision", attempt=attempt, phase="backup")
        return False

    motion.pulse(turn[0], turn[1], 0, turn_s)
    time.sleep(SETTLE_S)
    reading = vision.read_average()
    if reading is None:
        log.write("reset_lost_vision", attempt=attempt, phase="turn")
        return False

    log.write("reset_done", attempt=attempt, wins=wins, reading=asdict(reading))
    return True


def run_game(args: argparse.Namespace) -> int:
    target = load_target()
    vision = Vision(args.vision_url, args.min_confidence)
    motion = Motion(args.dry_run)
    log = JsonlLog()

    print(f"[main] log: {log.path}")
    print(
        "[main] target "
        f"x={target['x']:+d} y={target['y']:+d} dist={target['dist']} "
        f"gate=+/-{WIN_GATE_MM}mm"
    )
    log.write(
        "game_start",
        target=target,
        gate_mm=WIN_GATE_MM,
        attempts=args.attempts,
        attempt_seconds=args.attempt_seconds,
    )

    if vision.read_average() is None:
        print("[main] no confident brick vision; refusing to move")
        log.write("abort_no_vision")
        return 4
    if not motion.connect():
        log.write("abort_no_motion_transport")
        return 1

    wins = 0
    try:
        for attempt in range(1, args.attempts + 1):
            attempt_start = time.monotonic()
            steps = 0
            missing_since = None
            print(f"[main] attempt {attempt}/{args.attempts}")
            log.write("attempt_start", attempt=attempt, wins=wins)

            while time.monotonic() - attempt_start < args.attempt_seconds:
                reading = vision.read_average()
                if reading is None:
                    motion.park()
                    now = time.monotonic()
                    if missing_since is None:
                        missing_since = now
                    if now - missing_since > 2.5:
                        print("[main] lost brick vision; parking and aborting")
                        log.write("lost_vision", attempt=attempt, step=steps)
                        return 4
                    print("[main] waiting for stable brick read")
                    log.write("unstable_vision", attempt=attempt, step=steps)
                    time.sleep(0.3)
                    continue

                missing_since = None

                err = errors(reading, target)
                log.write(
                    "sample",
                    attempt=attempt,
                    step=steps,
                    reading=asdict(reading),
                    error=err,
                )

                if won(reading, target):
                    wins += 1
                    motion.park()
                    print(
                        f"[main] WIN {wins}: x={reading.x:+d} y={reading.y:+d} "
                        f"dist={reading.dist} err={err}"
                    )
                    log.write(
                        "win",
                        attempt=attempt,
                        wins=wins,
                        step=steps,
                        reading=asdict(reading),
                        error=err,
                    )
                    if not reset_after_win(attempt, wins, motion, vision, log):
                        motion.park()
                        print("[main] reset lost brick vision; parking and aborting")
                        return 4
                    break

                action, left, right, mast, seconds = choose_action(reading, target)
                print(
                    f"[main] step {steps:02d}: {action} "
                    f"x={reading.x:+d} y={reading.y:+d} dist={reading.dist} "
                    f"err={err} L={left:+d} R={right:+d} M={mast:+d} "
                    f"{seconds:.2f}s"
                )
                log.write(
                    "action",
                    attempt=attempt,
                    step=steps,
                    action=action,
                    command={"left": left, "right": right, "mast": mast, "seconds": seconds},
                    reading=asdict(reading),
                    error=err,
                )
                motion.pulse(left, right, mast, seconds)
                steps += 1
                time.sleep(SETTLE_S)
            else:
                reading = vision.read_average()
                if reading is None:
                    motion.park()
                    print(
                        f"[main] attempt {attempt} timed out with unstable vision; "
                        "trying again"
                    )
                    log.write("attempt_timeout_unstable", attempt=attempt, wins=wins)
                    continue
                print(
                    f"[main] attempt {attempt} timed out with brick visible; "
                    "trying again"
                )
                log.write(
                    "attempt_timeout",
                    attempt=attempt,
                    wins=wins,
                    reading=asdict(reading),
                    error=errors(reading, target),
                )

        print(f"[main] finished {args.attempts} attempts with {wins} wins")
        log.write("game_done", wins=wins)
        return 0
    except KeyboardInterrupt:
        print("\n[main] interrupted; parking")
        log.write("interrupted", wins=wins)
        return 130
    finally:
        motion.park()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Bun's step-1 alignment game.")
    parser.add_argument("--vision-url", default=STATUS_URL)
    parser.add_argument("--attempts", type=int, default=ATTEMPTS)
    parser.add_argument("--attempt-seconds", type=float, default=ATTEMPT_SECONDS)
    parser.add_argument("--min-confidence", type=int, default=MIN_CONFIDENCE)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.attempts <= 0:
        parser.error("--attempts must be positive")
    if args.attempt_seconds <= 0:
        parser.error("--attempt-seconds must be positive")
    if args.min_confidence < 0:
        parser.error("--min-confidence must be non-negative")

    return run_game(args)


if __name__ == "__main__":
    raise SystemExit(main())
