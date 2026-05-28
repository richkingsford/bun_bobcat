#!/usr/bin/env python3
"""Quick closed-loop alignment against the live brick vision stream."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from host_controller import (  # noqa: E402
    DEFAULT_PORT,
    DEFAULT_ROUTER_SOCKET,
    DryRunLink,
    RpcLink,
    SERIAL_BAUD,
    SerialLink,
)


DEFAULT_STATUS_URL = "http://127.0.0.1:8080/status"
DEFAULT_TARGET_X_MM = 0
DEFAULT_TARGET_Y_MM = 0
DEFAULT_TARGET_DIST_MM = 30
DEFAULT_X_DEADBAND_MM = 15
DEFAULT_Y_DEADBAND_MM = 15
DEFAULT_DIST_DEADBAND_MM = 15
DEFAULT_MIN_CONFIDENCE = 55
DEFAULT_POWER = 70
DEFAULT_MAST_POWER = 70
DEFAULT_FORWARD_COMMAND_SIGN = -1
DEFAULT_MAST_UP_COMMAND_SIGN = -1
DEFAULT_MIN_PULSE_MS = 35
DEFAULT_MAX_PULSE_MS = 120
DEFAULT_PULSE_MS_PER_MM = 1.1
DEFAULT_MAST_MS_PER_MM = 1.4
DEFAULT_DIST_MS_PER_MM = 1.2
DEFAULT_SAMPLE_PAUSE_S = 0.08
DEFAULT_MAX_SECONDS = 5.0
DEFAULT_PREFLIGHT_SECONDS = 2.0
DEFAULT_SETTLED_SAMPLES = 3
DEFAULT_NO_PROGRESS_SAMPLES = 6
DEFAULT_PROGRESS_EPSILON_MM = 8


@dataclass
class BrickReading:
    x_mm: int
    y_mm: int
    dist_mm: int
    confidence: int
    dist_source: str


class BrickVisionClient:
    def __init__(self, status_url, timeout_s=0.35, min_confidence=DEFAULT_MIN_CONFIDENCE):
        self.status_url = normalize_status_url(status_url)
        self.timeout_s = timeout_s
        self.min_confidence = min_confidence

    def read(self):
        try:
            with urlopen(self.status_url, timeout=self.timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (OSError, URLError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"vision status unavailable: {exc}") from exc

        detection = payload.get("detection") or {}
        if not detection.get("found"):
            return None

        confidence = int(detection.get("confidence") or 0)
        if confidence < self.min_confidence:
            return None

        spatial = detection.get("spatial") or {}
        if not spatial.get("valid"):
            return None

        return BrickReading(
            x_mm=int(spatial["x_mm"]),
            y_mm=int(spatial["y_mm"]),
            dist_mm=int(spatial["dist_mm"]),
            confidence=confidence,
            dist_source=str(spatial.get("dist_source") or "?"),
        )


def normalize_status_url(url):
    cleaned = url.strip()
    if cleaned.endswith("/status"):
        return cleaned
    return cleaned.rstrip("/") + "/status"


def clamp_int(value, low, high):
    return max(low, min(high, int(round(value))))


def make_link(args):
    if args.dry_run or args.transport == "dry-run":
        return "dry-run", DryRunLink()
    if args.transport == "serial":
        return "serial", SerialLink(args.port or DEFAULT_PORT, args.baud)
    return "rpc", RpcLink(args.router_socket)


def send_for(link, left, right, mast, seconds):
    if mast and getattr(link, "bridge", None) is not None:
        link.bridge.notify("drive_triple", int(left), int(right), int(mast))
    else:
        link.send(left, right)
    time.sleep(seconds)
    link.coast()


def wait_for_confident_brick(vision, seconds, pause_s):
    deadline = time.monotonic() + seconds
    last_error = None
    misses = 0

    while time.monotonic() < deadline:
        try:
            reading = vision.read()
        except RuntimeError as exc:
            last_error = str(exc)
            time.sleep(pause_s)
            continue

        if reading is not None:
            print(
                f"[align] vision gate passed: x={reading.x_mm:+d}mm "
                f"y={reading.y_mm:+d}mm dist={reading.dist_mm}mm "
                f"conf={reading.confidence}% src={reading.dist_source}"
            )
            return reading

        misses += 1
        time.sleep(pause_s)

    reason = last_error or f"no confident brick reading after {misses} samples"
    print(f"[align] aborting before motion: {reason}")
    return None


def align(args):
    vision = BrickVisionClient(
        args.vision_url,
        timeout_s=args.vision_timeout,
        min_confidence=args.min_confidence,
    )

    if args.require_vision and wait_for_confident_brick(
        vision, args.preflight_seconds, args.sample_pause
    ) is None:
        return 4

    transport, link = make_link(args)

    if not link.connect():
        print(f"[align] {transport} transport is not ready")
        return 1

    print(
        "[align] aligning brick; "
        f"target x={args.target_x_mm:+d}mm y={args.target_y_mm:+d}mm "
        f"dist={args.target_dist_mm}mm timeout={args.max_seconds:.1f}s"
    )

    start_t = time.monotonic()
    settled = 0
    misses = 0
    last_action = None
    last_error_mm = None
    no_progress_samples = 0

    try:
        while time.monotonic() - start_t < args.max_seconds:
            try:
                reading = vision.read()
            except RuntimeError as exc:
                print(f"[align] {exc}")
                link.coast()
                time.sleep(args.sample_pause)
                continue

            if reading is None:
                misses += 1
                print(f"[align] searching... miss={misses}")
                link.coast()
                time.sleep(args.sample_pause)
                continue

            misses = 0
            x_error = reading.x_mm - args.target_x_mm
            y_error = args.target_y_mm - reading.y_mm
            dist_error = reading.dist_mm - args.target_dist_mm
            x_ok = abs(x_error) <= args.x_deadband_mm
            y_ok = abs(y_error) <= args.y_deadband_mm
            dist_ok = abs(dist_error) <= args.dist_deadband_mm

            if x_ok and y_ok and dist_ok:
                settled += 1
                print(
                    f"[align] centered sample {settled}/{args.settled_samples}: "
                    f"x={reading.x_mm:+d}mm y={reading.y_mm:+d}mm dist={reading.dist_mm}mm "
                    f"conf={reading.confidence}%"
                )
                link.coast()
                if settled >= args.settled_samples:
                    return 0
                time.sleep(args.sample_pause)
                continue

            settled = 0

            left = 0
            right = 0
            mast = 0
            action = "coast"
            pulse_gain = args.pulse_ms_per_mm
            error_mm = 0

            if not x_ok:
                turn_right = x_error > 0
                if args.reverse_turn:
                    turn_right = not turn_right
                left = args.power if turn_right else -args.power
                right = -args.power if turn_right else args.power
                action = "turn right" if turn_right else "turn left"
                error_mm = abs(x_error)
                pulse_gain = args.pulse_ms_per_mm
            elif not dist_ok:
                needs_forward = dist_error > 0
                command_sign = args.forward_command_sign if needs_forward else -args.forward_command_sign
                if args.reverse_drive:
                    command_sign *= -1
                left = command_sign * args.power
                right = command_sign * args.power
                action = "forward" if needs_forward else "backward"
                error_mm = abs(dist_error)
                pulse_gain = args.dist_ms_per_mm
            else:
                needs_mast_up = y_error > 0
                command_sign = args.mast_up_command_sign if needs_mast_up else -args.mast_up_command_sign
                if args.reverse_mast:
                    command_sign *= -1
                mast = command_sign * args.mast_power
                action = "mast up" if needs_mast_up else "mast down"
                error_mm = abs(y_error)
                pulse_gain = args.mast_ms_per_mm

            pulse_ms = clamp_int(
                error_mm * pulse_gain,
                args.min_pulse_ms,
                args.max_pulse_ms,
            )
            print(
                f"[align] x={reading.x_mm:+d}mm y={reading.y_mm:+d}mm dist={reading.dist_mm}mm "
                f"err x={x_error:+d} y={y_error:+d} d={dist_error:+d} "
                f"conf={reading.confidence}% -> {action} "
                f"L={left:+d} R={right:+d} M={mast:+d} for {pulse_ms}ms"
            )

            if action == last_action and last_error_mm is not None:
                progress = last_error_mm - error_mm
                if progress < args.progress_epsilon_mm:
                    no_progress_samples += 1
                else:
                    no_progress_samples = 0
            else:
                no_progress_samples = 0

            last_action = action
            last_error_mm = error_mm
            if no_progress_samples >= args.no_progress_samples:
                print(
                    f"[align] stopping: no useful progress after "
                    f"{no_progress_samples} {action} samples"
                )
                return 3

            send_for(link, left, right, mast, pulse_ms / 1000.0)
            time.sleep(args.sample_pause)

        print("[align] timeout before target lock")
        return 2
    finally:
        link.coast()
        link.close()


def main():
    parser = argparse.ArgumentParser(
        description="Use the live brick livestream data to quickly align Bun with the brick."
    )
    parser.add_argument("--vision-url", default=DEFAULT_STATUS_URL,
                        help=f"livestream base URL or /status URL (default {DEFAULT_STATUS_URL})")
    parser.add_argument("--vision-timeout", type=float, default=0.35,
                        help="seconds to wait for each vision status read")
    parser.add_argument("--transport", choices=("rpc", "serial", "dry-run"), default="rpc",
                        help="movement transport (default rpc)")
    parser.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET,
                        help=f"App Lab router socket (default {DEFAULT_ROUTER_SOCKET})")
    parser.add_argument("--port", default=None,
                        help=f"serial port for --transport serial (default {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--dry-run", action="store_true",
                        help="read vision and print commands without moving hardware")
    parser.add_argument("--target-x-mm", type=int, default=DEFAULT_TARGET_X_MM,
                        help="desired brick x offset; + is camera-right")
    parser.add_argument("--target-y-mm", type=int, default=DEFAULT_TARGET_Y_MM,
                        help="desired brick y offset; + is lower in the camera frame")
    parser.add_argument("--target-dist-mm", type=int, default=DEFAULT_TARGET_DIST_MM,
                        help="desired forward distance from camera to brick")
    parser.add_argument("--x-deadband-mm", type=int, default=DEFAULT_X_DEADBAND_MM,
                        help="acceptable x error around target")
    parser.add_argument("--y-deadband-mm", type=int, default=DEFAULT_Y_DEADBAND_MM,
                        help="acceptable y error around target")
    parser.add_argument("--dist-deadband-mm", type=int, default=DEFAULT_DIST_DEADBAND_MM,
                        help="acceptable distance error around target")
    parser.add_argument("--settled-samples", type=int, default=DEFAULT_SETTLED_SAMPLES,
                        help="consecutive centered samples required")
    parser.add_argument("--no-progress-samples", type=int, default=DEFAULT_NO_PROGRESS_SAMPLES,
                        help="stop after this many repeated commands without error improvement")
    parser.add_argument("--progress-epsilon-mm", type=int, default=DEFAULT_PROGRESS_EPSILON_MM,
                        help="minimum improvement that counts as progress")
    parser.add_argument("--min-confidence", type=int, default=DEFAULT_MIN_CONFIDENCE,
                        help="minimum brick confidence before moving")
    parser.add_argument("--power", type=int, default=DEFAULT_POWER,
                        help="signed tread command magnitude 1-100")
    parser.add_argument("--mast-power", type=int, default=DEFAULT_MAST_POWER,
                        help="signed mast command magnitude 1-100")
    parser.add_argument("--forward-command-sign", type=int, choices=(-1, 1),
                        default=DEFAULT_FORWARD_COMMAND_SIGN,
                        help="signed tread direction that physically moves Bun forward")
    parser.add_argument("--mast-up-command-sign", type=int, choices=(-1, 1),
                        default=DEFAULT_MAST_UP_COMMAND_SIGN,
                        help="signed mast direction that physically raises the camera")
    parser.add_argument("--min-pulse-ms", type=int, default=DEFAULT_MIN_PULSE_MS)
    parser.add_argument("--max-pulse-ms", type=int, default=DEFAULT_MAX_PULSE_MS)
    parser.add_argument("--pulse-ms-per-mm", type=float, default=DEFAULT_PULSE_MS_PER_MM,
                        help="turn pulse duration gain from x error")
    parser.add_argument("--mast-ms-per-mm", type=float, default=DEFAULT_MAST_MS_PER_MM,
                        help="mast pulse duration gain from y error")
    parser.add_argument("--dist-ms-per-mm", type=float, default=DEFAULT_DIST_MS_PER_MM,
                        help="drive pulse duration gain from distance error")
    parser.add_argument("--sample-pause", type=float, default=DEFAULT_SAMPLE_PAUSE_S,
                        help="pause after each pulse before reading vision again")
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS,
                        help="alignment test timeout")
    parser.add_argument("--preflight-seconds", type=float, default=DEFAULT_PREFLIGHT_SECONDS,
                        help="seconds to wait for a confident brick reading before motors are opened")
    parser.add_argument("--allow-blind-start", action="store_true",
                        help="open the motor transport even if vision is not ready")
    parser.add_argument("--reverse-turn", action="store_true",
                        help="flip left/right spin direction if hardware is reversed")
    parser.add_argument("--reverse-drive", action="store_true",
                        help="temporary override: flip the configured forward command sign")
    parser.add_argument("--reverse-mast", action="store_true",
                        help="temporary override: flip the configured mast command sign")
    args = parser.parse_args()

    if not 1 <= args.power <= 100:
        parser.error("--power must be in 1..100")
    if not 1 <= args.mast_power <= 100:
        parser.error("--mast-power must be in 1..100")
    if args.x_deadband_mm < 0 or args.y_deadband_mm < 0 or args.dist_deadband_mm < 0:
        parser.error("deadbands must be non-negative")
    if args.settled_samples <= 0:
        parser.error("--settled-samples must be positive")
    if args.no_progress_samples <= 0 or args.progress_epsilon_mm < 0:
        parser.error("--no-progress-samples must be positive and --progress-epsilon-mm non-negative")
    if args.min_pulse_ms <= 0 or args.max_pulse_ms < args.min_pulse_ms:
        parser.error("pulse bounds must be positive and max >= min")
    if args.sample_pause < 0 or args.max_seconds <= 0:
        parser.error("--sample-pause must be non-negative and --max-seconds positive")
    if args.preflight_seconds < 0:
        parser.error("--preflight-seconds must be non-negative")
    args.require_vision = not args.allow_blind_start

    return align(args)


if __name__ == "__main__":
    raise SystemExit(main())
