#!/usr/bin/env python3
"""Calibrate Bun motion against the live brick vision pose."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import statistics
import sys
import time


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from align_to_brick import (  # noqa: E402
    BrickReading,
    BrickVisionClient,
    DEFAULT_FORWARD_COMMAND_SIGN,
    DEFAULT_MAST_UP_COMMAND_SIGN,
    DEFAULT_STATUS_URL,
    make_link,
)


DEFAULT_CALIBRATION_PATH = SCRIPT_DIR / "motion_calibration.json"
DEFAULT_DURATION_S = 1.0
DEFAULT_POWER = 70
DEFAULT_MAST_POWER = 100
DEFAULT_COMMAND_PERIOD_S = 0.05
DEFAULT_SETTLE_S = 0.45
DEFAULT_SAMPLES = 5
DEFAULT_SAMPLE_INTERVAL_S = 0.12
DEFAULT_READ_TIMEOUT_S = 2.5
DEFAULT_MIN_DIST_MM = 130
DEFAULT_MIN_CONFIDENCE = 70

SEED_DELTAS_PER_S = {
    "dist_toward": {"x_mm": 0.0, "y_mm": 0.0, "dist_mm": -45.0},
    "dist_away": {"x_mm": 0.0, "y_mm": 0.0, "dist_mm": 45.0},
    "x_spin_a": {"x_mm": 35.0, "y_mm": 0.0, "dist_mm": 0.0},
    "x_spin_b": {"x_mm": -35.0, "y_mm": 0.0, "dist_mm": 0.0},
    "y_mast_up": {"x_mm": 0.0, "y_mm": 35.0, "dist_mm": 0.0},
    "y_mast_down": {"x_mm": 0.0, "y_mm": -35.0, "dist_mm": 0.0},
}


@dataclass
class Act:
    name: str
    axis: str
    left: int
    right: int
    mast: int


def reading_to_dict(reading):
    return {
        "x_mm": float(reading.x_mm),
        "y_mm": float(reading.y_mm),
        "dist_mm": float(reading.dist_mm),
    }


def dict_to_reading(values, confidence=0, dist_source="calculated"):
    return BrickReading(
        x_mm=int(round(values["x_mm"])),
        y_mm=int(round(values["y_mm"])),
        dist_mm=int(round(values["dist_mm"])),
        confidence=int(confidence),
        dist_source=dist_source,
    )


def add_delta(values, delta):
    return {
        "x_mm": values["x_mm"] + delta["x_mm"],
        "y_mm": values["y_mm"] + delta["y_mm"],
        "dist_mm": values["dist_mm"] + delta["dist_mm"],
    }


def subtract_readings(after, before):
    return {
        "x_mm": float(after.x_mm - before.x_mm),
        "y_mm": float(after.y_mm - before.y_mm),
        "dist_mm": float(after.dist_mm - before.dist_mm),
    }


def scale_delta(delta, factor):
    return {key: float(value) * factor for key, value in delta.items()}


def fmt_reading(reading):
    return (
        f"x={reading.x_mm:+d}mm y={reading.y_mm:+d}mm "
        f"dist={reading.dist_mm:d}mm conf={reading.confidence}%"
    )


def fmt_delta(delta):
    return (
        f"dx={delta['x_mm']:+.1f}mm "
        f"dy={delta['y_mm']:+.1f}mm "
        f"ddist={delta['dist_mm']:+.1f}mm"
    )


def load_model(path):
    if path.exists():
        with path.open("r", encoding="utf-8") as handle:
            model = json.load(handle)
    else:
        model = {"version": 1, "acts": {}}

    acts = model.setdefault("acts", {})
    for name, seed in SEED_DELTAS_PER_S.items():
        acts.setdefault(name, {"delta_per_s": seed, "count": 0})
        acts[name].setdefault("delta_per_s", seed)
        acts[name].setdefault("count", 0)
    return model


def save_model(path, model):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(model, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def predicted_delta(model, act_name, duration_s):
    per_s = model["acts"][act_name]["delta_per_s"]
    return scale_delta(per_s, duration_s)


def update_model(model, act_name, observed_delta, duration_s):
    entry = model["acts"][act_name]
    observed_per_s = scale_delta(observed_delta, 1.0 / max(duration_s, 1e-6))
    count = int(entry.get("count", 0))
    old = entry["delta_per_s"]
    if count <= 0:
        new_delta = observed_per_s
    else:
        new_delta = {
            key: (float(old[key]) * count + observed_per_s[key]) / (count + 1)
            for key in ("x_mm", "y_mm", "dist_mm")
        }
    entry["delta_per_s"] = new_delta
    entry["count"] = count + 1
    entry["last_observed_delta"] = observed_delta
    entry["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    return new_delta


def read_stable(vision, samples, interval_s, timeout_s):
    readings = []
    deadline = time.monotonic() + timeout_s
    while len(readings) < samples and time.monotonic() < deadline:
        reading = vision.read()
        if reading is not None:
            readings.append(reading)
        time.sleep(interval_s)

    if not readings:
        raise RuntimeError("no valid brick readings")

    values = {
        "x_mm": statistics.median(item.x_mm for item in readings),
        "y_mm": statistics.median(item.y_mm for item in readings),
        "dist_mm": statistics.median(item.dist_mm for item in readings),
    }
    confidence = statistics.median(item.confidence for item in readings)
    return dict_to_reading(values, confidence=confidence, dist_source=readings[-1].dist_source)


def send_triple(link, left, right, mast):
    if getattr(link, "bridge", None) is not None:
        link.bridge.notify("drive_triple", int(left), int(right), int(mast))
    elif mast:
        raise RuntimeError("mast calibration requires rpc transport with drive_triple")
    else:
        link.send(left, right)


def hold_act(link, act, duration_s, command_period_s, dry_run=False):
    print(
        f"[cal] command {act.name}: "
        f"L={act.left:+d} R={act.right:+d} M={act.mast:+d} for {duration_s:.2f}s"
    )
    if dry_run:
        time.sleep(min(duration_s, 0.1))
        return

    end_t = time.monotonic() + duration_s
    try:
        while True:
            send_triple(link, act.left, act.right, act.mast)
            remaining = end_t - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(command_period_s, remaining))
    finally:
        link.coast()


def build_acts(args):
    forward = args.forward_command_sign * args.power
    mast_up = args.mast_up_command_sign * args.mast_power
    return [
        Act("dist_toward", "dist", forward, forward, 0),
        Act("dist_away", "dist", -forward, -forward, 0),
        Act("x_spin_a", "x", args.power, -args.power, 0),
        Act("x_spin_b", "x", -args.power, args.power, 0),
        Act("y_mast_up", "y", 0, 0, mast_up),
        Act("y_mast_down", "y", 0, 0, -mast_up),
    ]


def ensure_triple_rpc(link):
    if getattr(link, "bridge", None) is None:
        return
    link.bridge.call("drive_triple", 0, 0, 0, timeout=2)


def calibrate(args):
    model = load_model(args.calibration_file)
    vision = BrickVisionClient(
        args.vision_url,
        timeout_s=args.vision_timeout,
        min_confidence=args.min_confidence,
    )
    transport, link = make_link(args)

    if not link.connect():
        print(f"[cal] {transport} transport is not ready")
        return 1

    try:
        ensure_triple_rpc(link)
    except Exception as exc:
        print(f"[cal] drive_triple is unavailable: {exc}")
        link.close()
        return 1

    try:
        if args.pre_mast_up_seconds > 0:
            mast_up = args.mast_up_command_sign * args.mast_power
            print(
                f"[cal] pre-raising mast for {args.pre_mast_up_seconds:.2f}s "
                f"with M={mast_up:+d}"
            )
            hold_act(
                link,
                Act("pre_mast_up", "y", 0, 0, mast_up),
                args.pre_mast_up_seconds,
                args.command_period,
                dry_run=args.dry_run,
            )
            time.sleep(args.settle)

        print(f"[cal] reading brick pose from {args.vision_url}")
        before_all = read_stable(
            vision, args.samples, args.sample_interval, args.read_timeout
        )
        print(f"[cal] starting pose: {fmt_reading(before_all)}")

        for act in build_acts(args):
            before = read_stable(
                vision, args.samples, args.sample_interval, args.read_timeout
            )

            if act.name == "dist_toward" and before.dist_mm <= args.min_dist_mm:
                print(
                    f"[cal] skip {act.name}: dist={before.dist_mm}mm "
                    f"is at/below safety floor {args.min_dist_mm}mm"
                )
                continue

            pred_delta = predicted_delta(model, act.name, args.duration)
            pred_after = dict_to_reading(add_delta(reading_to_dict(before), pred_delta))
            print(
                f"[cal] {act.axis} {act.name} before: {fmt_reading(before)}"
            )
            print(
                f"[cal] predicted delta: {fmt_delta(pred_delta)} "
                f"=> {fmt_reading(pred_after)}"
            )

            hold_act(
                link,
                act,
                args.duration,
                args.command_period,
                dry_run=args.dry_run,
            )
            time.sleep(args.settle)

            after = read_stable(
                vision, args.samples, args.sample_interval, args.read_timeout
            )
            observed = subtract_readings(after, before)
            print(f"[cal] after: {fmt_reading(after)}")
            print(f"[cal] observed delta: {fmt_delta(observed)}")
            if args.dry_run:
                print(f"[cal] dry run: did not update {act.name}")
            else:
                updated = update_model(model, act.name, observed, args.duration)
                save_model(args.calibration_file, model)
                print(f"[cal] updated {act.name} per second: {fmt_delta(updated)}")

        final = read_stable(vision, args.samples, args.sample_interval, args.read_timeout)
        print(f"[cal] final pose: {fmt_reading(final)}")
        if not args.dry_run:
            print(f"[cal] saved calibration to {args.calibration_file}")
        return 0
    except KeyboardInterrupt:
        print("\n[cal] interrupted; coasting")
        return 130
    except RuntimeError as exc:
        print(f"[cal] stopped: {exc}")
        return 2
    finally:
        link.coast()
        link.close()


def main():
    parser = argparse.ArgumentParser(
        description="Measure 1s Bun movement deltas from live brick vision."
    )
    parser.add_argument("--vision-url", default=DEFAULT_STATUS_URL)
    parser.add_argument("--vision-timeout", type=float, default=0.35)
    parser.add_argument("--transport", choices=("rpc", "serial", "dry-run"), default="rpc")
    parser.add_argument("--router-socket", default="/var/run/arduino-router.sock")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--calibration-file", type=Path, default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--power", type=int, default=DEFAULT_POWER)
    parser.add_argument("--mast-power", type=int, default=DEFAULT_MAST_POWER)
    parser.add_argument("--forward-command-sign", type=int, choices=(-1, 1),
                        default=DEFAULT_FORWARD_COMMAND_SIGN)
    parser.add_argument("--mast-up-command-sign", type=int, choices=(-1, 1),
                        default=DEFAULT_MAST_UP_COMMAND_SIGN)
    parser.add_argument("--pre-mast-up-seconds", type=float, default=0.0)
    parser.add_argument("--command-period", type=float, default=DEFAULT_COMMAND_PERIOD_S)
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--sample-interval", type=float, default=DEFAULT_SAMPLE_INTERVAL_S)
    parser.add_argument("--read-timeout", type=float, default=DEFAULT_READ_TIMEOUT_S)
    parser.add_argument("--min-confidence", type=int, default=DEFAULT_MIN_CONFIDENCE)
    parser.add_argument("--min-dist-mm", type=int, default=DEFAULT_MIN_DIST_MM)
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")
    if not 1 <= args.power <= 100:
        parser.error("--power must be in 1..100")
    if not 1 <= args.mast_power <= 100:
        parser.error("--mast-power must be in 1..100")
    if args.pre_mast_up_seconds < 0 or args.command_period <= 0 or args.settle < 0:
        parser.error("timing values must be non-negative, and --command-period positive")
    if args.samples <= 0 or args.sample_interval < 0 or args.read_timeout <= 0:
        parser.error("sample settings must be positive")

    return calibrate(args)


if __name__ == "__main__":
    raise SystemExit(main())
