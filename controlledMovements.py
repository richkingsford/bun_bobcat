#!/usr/bin/env python3
"""
Run Bun's controlled movement QA pass and host a before/after evidence site.

The script captures the live camera's annotated /detect.jpg before and after
each act, records /status telemetry, writes a static report under
.cache/controlled_movements/latest, and can serve that report from the bot.
All configured movement durations are capped at 1.0 second.

Usage:
    python3 controlledMovements.py
    python3 controlledMovements.py --port 8090
    python3 controlledMovements.py --serve-only
    python3 controlledMovements.py --dry-run --no-serve
"""

from __future__ import annotations

import argparse
import html
import json
import shutil
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.request import urlopen


VISION_URL = "http://127.0.0.1:8080"
OUT_ROOT = Path(".cache/controlled_movements")
LATEST_DIR = OUT_ROOT / "latest"
MAX_MOVE_SECONDS = 0.5
COMMAND_REFRESH_SECONDS = 0.10
DEFAULT_MOVE_SECONDS = 0.35
MIN_DRIVE_POWER = 35
RECOVERY_TURN_POWER = 60

try:
    import host_controller as motor_calibration
except Exception:
    motor_calibration = None

INVERT_LEFT_MOTOR = bool(getattr(motor_calibration, "INVERT_LEFT_MOTOR", False))
INVERT_RIGHT_MOTOR = bool(getattr(motor_calibration, "INVERT_RIGHT_MOTOR", False))
SWAP_LEFT_RIGHT_MOTORS = bool(getattr(motor_calibration, "SWAP_LEFT_RIGHT_MOTORS", False))


@dataclass(frozen=True)
class Movement:
    key: str
    title: str
    description: str
    command: tuple[int, int, int]
    duration_s: float
    category: str = "drive"
    expected_axis: str | None = None
    expected_sign: int = 0
    min_expected_delta_mm: float = 0.0


MOVEMENTS: list[Movement] = [
    Movement(
        "equal_forward_probe",
        "Equal Forward Probe",
        "Both tracks receive the same forward intent. This is the Stage 1 glide primitive under calibration.",
        (55, 55, 0),
        DEFAULT_MOVE_SECONDS,
        expected_axis="ddist_mm",
        expected_sign=-1,
        min_expected_delta_mm=2.0,
    ),
    Movement(
        "forward_approach",
        "Forward Approach",
        "Both tracks move forward with a mild correction bias; both sides stay above stall-prone power.",
        (55, 45, 0),
        DEFAULT_MOVE_SECONDS,
        expected_axis="ddist_mm",
        expected_sign=-1,
        min_expected_delta_mm=3.0,
    ),
    Movement(
        "camera_x_decrease",
        "Camera X Decrease",
        "Counter-rotating tracks reduce positive camera x offset.",
        (55, -55, 0),
        0.18,
        expected_axis="dx_mm",
        expected_sign=-1,
        min_expected_delta_mm=3.0,
    ),
    Movement(
        "camera_x_increase",
        "Camera X Increase",
        "Counter-rotating tracks increase camera x offset.",
        (-55, 55, 0),
        0.18,
        expected_axis="dx_mm",
        expected_sign=1,
        min_expected_delta_mm=3.0,
    ),
    Movement(
        "forward_gentle_left_turn",
        "Forward Gentle Left Turn",
        "Both tracks move forward with a small left-turn bias; neither side is allowed near stall.",
        (50, 40, 0),
        DEFAULT_MOVE_SECONDS,
    ),
    Movement(
        "forward_gentle_right_turn",
        "Forward Gentle Right Turn",
        "Both tracks move forward with a small right-turn bias; neither side is allowed near stall.",
        (40, 50, 0),
        DEFAULT_MOVE_SECONDS,
    ),
    Movement(
        "forward_sharp_left_turn",
        "Forward Sharp Left Turn",
        "Both tracks move forward with a stronger left-turn bias while keeping both sides moving.",
        (60, 40, 0),
        DEFAULT_MOVE_SECONDS,
    ),
    Movement(
        "forward_sharp_right_turn",
        "Forward Sharp Right Turn",
        "Both tracks move forward with a stronger right-turn bias while keeping both sides moving.",
        (40, 60, 0),
        DEFAULT_MOVE_SECONDS,
    ),
    Movement(
        "mast_down",
        "Down",
        "Mast axis moves down for a visible vertical-control sample.",
        (0, 0, -100),
        DEFAULT_MOVE_SECONDS,
        "mast",
    ),
    Movement(
        "mast_up",
        "Up",
        "Mast axis moves up for a visible vertical-control sample.",
        (0, 0, 100),
        DEFAULT_MOVE_SECONDS,
        "mast",
    ),
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def clamp_duration(duration_s: float) -> float:
    return max(0.0, min(MAX_MOVE_SECONDS, float(duration_s)))


def calibrated_command(command: tuple[int, int, int]) -> tuple[int, int, int]:
    """Convert semantic left/right/mast intent into Bun's raw drive_triple."""
    left, right, mast = command
    if SWAP_LEFT_RIGHT_MOTORS:
        left, right = right, left
    if INVERT_LEFT_MOTOR:
        left = -left
    if INVERT_RIGHT_MOTOR:
        right = -right
    return int(left), int(right), int(mast)


def status_url(base_url: str) -> str:
    return base_url.rstrip("/") + "/status"


def detect_url(base_url: str) -> str:
    return base_url.rstrip("/") + "/detect.jpg"


def read_status(base_url: str) -> dict[str, Any]:
    try:
        with urlopen(status_url(base_url), timeout=0.25) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except Exception as exc:
        return {"ok": False, "error": repr(exc), "ts": now_iso()}

    detection = payload.get("detection") or {}
    spatial = detection.get("spatial") or {}
    return {
        "ok": bool(detection.get("found") and spatial.get("valid")),
        "found": detection.get("found"),
        "confidence": detection.get("confidence"),
        "x_mm": spatial.get("x_mm"),
        "y_mm": spatial.get("y_mm"),
        "dist_mm": spatial.get("dist_mm"),
        "dist_source": spatial.get("dist_source"),
        "valid": spatial.get("valid"),
        "ts": now_iso(),
    }


def status_line(status: dict[str, Any]) -> str:
    if not status.get("ok"):
        return "not visible"
    return (
        f"x={status.get('x_mm')}mm, y={status.get('y_mm')}mm, "
        f"dist={status.get('dist_mm')}mm, conf={status.get('confidence')}"
    )


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_status(
    status: dict[str, Any],
    *,
    min_confidence: int,
    max_abs_x_mm: float,
    min_dist_mm: float,
    max_dist_mm: float,
) -> bool:
    if not status.get("ok"):
        return False
    confidence = as_float(status.get("confidence"))
    x_mm = abs(as_float(status.get("x_mm"), 9999.0))
    dist_mm = as_float(status.get("dist_mm"), -1.0)
    return (
        confidence >= min_confidence
        and x_mm <= max_abs_x_mm
        and min_dist_mm <= dist_mm <= max_dist_mm
    )


def within_box(
    status: dict[str, Any],
    baseline: dict[str, Any] | None,
    box_mm: float,
) -> bool:
    if not baseline or box_mm <= 0:
        return True
    if not (status.get("ok") and baseline.get("ok")):
        return False
    half = box_mm / 2.0
    dx = abs(as_float(status.get("x_mm")) - as_float(baseline.get("x_mm")))
    ddist = abs(as_float(status.get("dist_mm")) - as_float(baseline.get("dist_mm")))
    return dx <= half and ddist <= half


def box_delta(
    status: dict[str, Any],
    baseline: dict[str, Any] | None,
) -> dict[str, float] | None:
    if not baseline or not (status.get("ok") and baseline.get("ok")):
        return None
    return {
        "dx_from_origin_mm": round(as_float(status.get("x_mm")) - as_float(baseline.get("x_mm")), 2),
        "ddist_from_origin_mm": round(as_float(status.get("dist_mm")) - as_float(baseline.get("dist_mm")), 2),
    }


def centered_enough(
    status: dict[str, Any],
    min_confidence: int,
    recover_abs_x_mm: float,
) -> bool:
    if not status.get("ok"):
        return False
    return (
        as_float(status.get("confidence")) >= min_confidence
        and abs(as_float(status.get("x_mm"), 9999.0)) <= recover_abs_x_mm
    )


class MotorLink:
    def __init__(self, dry_run: bool):
        self.dry_run = dry_run
        self.bridge = None

    def connect(self) -> None:
        if self.dry_run:
            print("[dry-run] motor commands will be logged but not sent")
            return
        try:
            from arduino.app_utils import Bridge
        except ImportError as exc:
            raise RuntimeError("arduino.app_utils is unavailable on this host") from exc
        self.bridge = Bridge
        self.park()

    def send(self, left: int, right: int, mast: int) -> None:
        print(f"[cmd] drive_triple({left}, {right}, {mast})")
        if self.dry_run:
            return
        if self.bridge is None:
            raise RuntimeError("motor link is not connected")
        self.bridge.notify("drive_triple", int(left), int(right), int(mast))

    def park(self) -> None:
        for _ in range(7):
            self.send(0, 0, 0)
            time.sleep(0.025)


def prepare_output_dir() -> tuple[Path, Path]:
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = OUT_ROOT / run_id
    assets_dir = run_dir / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, assets_dir


def update_latest(run_dir: Path) -> None:
    if LATEST_DIR.exists() or LATEST_DIR.is_symlink():
        if LATEST_DIR.is_symlink() or LATEST_DIR.is_file():
            LATEST_DIR.unlink()
        else:
            shutil.rmtree(LATEST_DIR)
    shutil.copytree(run_dir, LATEST_DIR)


def capture_image(base_url: str, out_path: Path) -> str | None:
    try:
        with urlopen(detect_url(base_url), timeout=1.5) as resp:
            out_path.write_bytes(resp.read())
        return None
    except Exception as exc:
        return repr(exc)


def record_snapshot(
    *,
    base_url: str,
    assets_dir: Path,
    stem: str,
    label: str,
) -> dict[str, Any]:
    image_name = f"{stem}_{label}.jpg"
    image_path = assets_dir / image_name
    image_error = capture_image(base_url, image_path)
    return {
        "image": "assets/" + image_name if image_error is None else None,
        "image_error": image_error,
        "status": read_status(base_url),
        "captured_at": now_iso(),
    }


def delta(before: dict[str, Any], after: dict[str, Any]) -> dict[str, float] | None:
    before_status = before.get("status") or {}
    after_status = after.get("status") or {}
    if not (before_status.get("ok") and after_status.get("ok")):
        return None
    return {
        "dx_mm": round(as_float(after_status.get("x_mm")) - as_float(before_status.get("x_mm")), 2),
        "dy_mm": round(as_float(after_status.get("y_mm")) - as_float(before_status.get("y_mm")), 2),
        "ddist_mm": round(as_float(after_status.get("dist_mm")) - as_float(before_status.get("dist_mm")), 2),
    }


def direction_check(
    movement: Movement,
    delta_obj: dict[str, float] | None,
) -> dict[str, Any] | None:
    if not movement.expected_axis or movement.expected_sign == 0:
        return None
    if not delta_obj or movement.expected_axis not in delta_obj:
        return {
            "axis": movement.expected_axis,
            "expected_sign": movement.expected_sign,
            "min_delta_mm": movement.min_expected_delta_mm,
            "observed_mm": None,
            "passed": False,
            "reason": "delta unavailable",
        }
    observed = float(delta_obj[movement.expected_axis])
    signed_observed = observed * movement.expected_sign
    return {
        "axis": movement.expected_axis,
        "expected_sign": movement.expected_sign,
        "min_delta_mm": movement.min_expected_delta_mm,
        "observed_mm": observed,
        "passed": signed_observed >= movement.min_expected_delta_mm,
        "reason": (
            "ok"
            if signed_observed >= movement.min_expected_delta_mm
            else "expected larger movement in the opposite/declared direction"
        ),
    }


def recover(
    *,
    base_url: str,
    link: MotorLink,
    min_confidence: int,
    max_abs_x_mm: float,
    recover_abs_x_mm: float,
    min_dist_mm: float,
    max_dist_mm: float,
    assets_dir: Path,
    reason: str,
    last_good_x: float | None,
) -> tuple[bool, float | None, list[dict[str, Any]]]:
    events: list[dict[str, Any]] = []
    print(f"[recover] {reason}")
    link.park()
    time.sleep(0.25)
    current = read_status(base_url)
    events.append({"kind": "start", "reason": reason, "status": current, "ts": now_iso()})
    if centered_enough(current, min_confidence, recover_abs_x_mm):
        return True, as_float(current.get("x_mm")), events

    side = 1
    if current.get("ok"):
        side = 1 if as_float(current.get("x_mm")) >= 0 else -1
        last_good_x = as_float(current.get("x_mm"))
    elif last_good_x is not None:
        side = 1 if last_good_x >= 0 else -1

    for idx in range(14):
        probe = read_status(base_url)
        if not probe.get("ok") and idx and idx % 4 == 0:
            side *= -1
        cmd = (RECOVERY_TURN_POWER, -RECOVERY_TURN_POWER, 0) if side >= 0 else (
            -RECOVERY_TURN_POWER,
            RECOVERY_TURN_POWER,
            0,
        )
        before = record_snapshot(
            base_url=base_url,
            assets_dir=assets_dir,
            stem=f"recovery_{len(events):02d}",
            label="before",
        )
        link.send(*cmd)
        time.sleep(0.045)
        link.park()
        time.sleep(0.22)
        after = record_snapshot(
            base_url=base_url,
            assets_dir=assets_dir,
            stem=f"recovery_{len(events):02d}",
            label="after",
        )
        event = {
            "kind": "pulse",
            "command": cmd,
            "duration_s": 0.045,
            "before": before,
            "after": after,
            "delta": delta(before, after),
        }
        events.append(event)
        after_status = after["status"]
        if after_status.get("ok"):
            last_good_x = as_float(after_status.get("x_mm"))
            side = 1 if last_good_x >= 0 else -1
        if centered_enough(after_status, min_confidence, recover_abs_x_mm):
            events.append({"kind": "centered", "status": after_status, "ts": now_iso()})
            return True, last_good_x, events

    final_status = read_status(base_url)
    if safe_status(
        final_status,
        min_confidence=min_confidence,
        max_abs_x_mm=max_abs_x_mm,
        min_dist_mm=min_dist_mm,
        max_dist_mm=max_dist_mm,
    ):
        events.append({"kind": "safe_visible", "status": final_status, "ts": now_iso()})
        return True, as_float(final_status.get("x_mm"), last_good_x or 0.0), events

    events.append({"kind": "failed", "status": final_status, "ts": now_iso()})
    return False, last_good_x, events


def run_movement(
    *,
    movement: Movement,
    index: int,
    base_url: str,
    link: MotorLink,
    assets_dir: Path,
    min_confidence: int,
    max_abs_x_mm: float,
    recover_abs_x_mm: float,
    min_dist_mm: float,
    max_dist_mm: float,
    baseline_status: dict[str, Any] | None,
    box_mm: float,
    last_good_x: float | None,
) -> tuple[dict[str, Any], float | None, bool]:
    if movement.duration_s > MAX_MOVE_SECONDS:
        raise ValueError(f"{movement.key} exceeds {MAX_MOVE_SECONDS}s")
    if movement.category == "drive" and (movement.command[0] == 0 or movement.command[1] == 0):
        raise ValueError(f"{movement.key} must move both wheels")
    if movement.category == "drive" and (
        abs(movement.command[0]) < MIN_DRIVE_POWER
        or abs(movement.command[1]) < MIN_DRIVE_POWER
    ):
        raise ValueError(
            f"{movement.key} wheel commands must both be at least {MIN_DRIVE_POWER}%"
        )

    stem = f"{index:02d}_{movement.key}"
    raw_command = calibrated_command(movement.command)
    if movement.category == "drive" and (
        abs(raw_command[0]) < MIN_DRIVE_POWER
        or abs(raw_command[1]) < MIN_DRIVE_POWER
    ):
        raise ValueError(
            f"{movement.key} calibrated raw wheel commands must both be at least {MIN_DRIVE_POWER}%"
        )
    recovery_events: list[dict[str, Any]] = []
    print(
        f"\n[move] {movement.title} semantic={movement.command} "
        f"raw={raw_command} {movement.duration_s:.3f}s"
    )

    before_status = read_status(base_url)
    needs_recovery = not safe_status(
        before_status,
        min_confidence=min_confidence,
        max_abs_x_mm=max_abs_x_mm,
        min_dist_mm=min_dist_mm,
        max_dist_mm=max_dist_mm,
    ) or not within_box(before_status, baseline_status, box_mm)
    if (
        before_status.get("ok")
        and as_float(before_status.get("confidence")) >= min_confidence
        and abs(as_float(before_status.get("x_mm"), 9999.0)) > recover_abs_x_mm
    ):
        needs_recovery = True

    if needs_recovery:
        recovered, last_good_x, events = recover(
            base_url=base_url,
            link=link,
            min_confidence=min_confidence,
            max_abs_x_mm=max_abs_x_mm,
            recover_abs_x_mm=recover_abs_x_mm,
            min_dist_mm=min_dist_mm,
            max_dist_mm=max_dist_mm,
            assets_dir=assets_dir,
            reason=f"before {movement.title}",
            last_good_x=last_good_x,
        )
        recovery_events.extend(events)
        if not recovered:
            result = {
                "movement": asdict(movement),
                "skipped": True,
                "skip_reason": "Recovery failed before movement.",
                "recovery": recovery_events,
            }
            return result, last_good_x, False

    before = record_snapshot(
        base_url=base_url,
        assets_dir=assets_dir,
        stem=stem,
        label="before",
    )
    if before["status"].get("ok"):
        last_good_x = as_float(before["status"].get("x_mm"))

    duration_s = clamp_duration(movement.duration_s)
    link.send(*raw_command)
    move_started = time.monotonic()
    deadline = move_started + duration_s
    next_refresh = move_started + COMMAND_REFRESH_SECONDS
    samples = []
    guard_trip = None
    while time.monotonic() < deadline:
        time.sleep(0.025)
        sample = read_status(base_url)
        samples.append(sample)
        if sample.get("ok"):
            last_good_x = as_float(sample.get("x_mm"))
        if not safe_status(
            sample,
            min_confidence=min_confidence,
            max_abs_x_mm=max_abs_x_mm,
            min_dist_mm=min_dist_mm,
            max_dist_mm=max_dist_mm,
        ) or not within_box(sample, baseline_status, box_mm):
            guard_trip = sample
            print(f"[guard] {movement.title}: {status_line(sample)}")
            break
        now = time.monotonic()
        if now >= next_refresh and now < deadline:
            link.send(*raw_command)
            next_refresh = now + COMMAND_REFRESH_SECONDS
    stopped_at = time.monotonic()
    link.park()
    time.sleep(0.28)

    after = record_snapshot(
        base_url=base_url,
        assets_dir=assets_dir,
        stem=stem,
        label="after",
    )
    after_status = after["status"]
    if after_status.get("ok"):
        last_good_x = as_float(after_status.get("x_mm"))

    recovered_after = True
    if guard_trip is not None or not safe_status(
        after_status,
        min_confidence=min_confidence,
        max_abs_x_mm=max_abs_x_mm,
        min_dist_mm=min_dist_mm,
        max_dist_mm=max_dist_mm,
    ) or not within_box(after_status, baseline_status, box_mm) or (
        after_status.get("ok")
        and as_float(after_status.get("confidence")) >= min_confidence
        and abs(as_float(after_status.get("x_mm"), 9999.0)) > recover_abs_x_mm
    ):
        recovered_after, last_good_x, events = recover(
            base_url=base_url,
            link=link,
            min_confidence=min_confidence,
            max_abs_x_mm=max_abs_x_mm,
            recover_abs_x_mm=recover_abs_x_mm,
            min_dist_mm=min_dist_mm,
            max_dist_mm=max_dist_mm,
            assets_dir=assets_dir,
            reason=f"after {movement.title}",
            last_good_x=last_good_x,
        )
        recovery_events.extend(events)

    move_delta = delta(before, after)
    result = {
        "movement": asdict(movement),
        "raw_command": raw_command,
        "before": before,
        "after": after,
        "delta": move_delta,
        "direction_check": direction_check(movement, move_delta),
        "box_delta": box_delta(after_status, baseline_status),
        "requested_duration_s": duration_s,
        "actual_duration_s": round(min(stopped_at, deadline) - move_started, 3),
        "samples": samples,
        "guard_trip": guard_trip,
        "recovery": recovery_events,
        "ok": guard_trip is None and recovered_after and after_status.get("ok"),
    }
    return result, last_good_x, recovered_after


def h(text: Any) -> str:
    return html.escape(str(text), quote=True)


def image_html(snapshot: dict[str, Any], title: str) -> str:
    image = snapshot.get("image")
    status = snapshot.get("status") or {}
    if image:
        img = f'<img src="{h(image)}" alt="{h(title)} camera evidence">'
    else:
        img = f'<div class="missing">Image unavailable: {h(snapshot.get("image_error"))}</div>'
    return (
        f"<figure>{img}<figcaption><strong>{h(title)}</strong>"
        f"<span>{h(status_line(status))}</span></figcaption></figure>"
    )


def delta_html(delta_obj: dict[str, float] | None) -> str:
    if not delta_obj:
        return '<p class="muted">Delta unavailable; one side had no valid spatial fix.</p>'
    return (
        '<dl class="delta">'
        f"<div><dt>Delta x</dt><dd>{h(delta_obj['dx_mm'])} mm</dd></div>"
        f"<div><dt>Delta y</dt><dd>{h(delta_obj['dy_mm'])} mm</dd></div>"
        f"<div><dt>Delta dist</dt><dd>{h(delta_obj['ddist_mm'])} mm</dd></div>"
        "</dl>"
    )


def direction_html(check: dict[str, Any] | None) -> str:
    if not check:
        return ""
    expected = "increase" if check["expected_sign"] > 0 else "decrease"
    observed = check.get("observed_mm")
    observed_text = "n/a" if observed is None else f"{observed} mm"
    klass = "direction-ok" if check.get("passed") else "direction-warn"
    return (
        f'<p class="{klass}">Direction check: expected {h(check["axis"])} '
        f'to {h(expected)} by at least {h(check["min_delta_mm"])} mm; '
        f'observed {h(observed_text)}.</p>'
    )


def box_html(delta_obj: dict[str, float] | None) -> str:
    if not delta_obj:
        return ""
    return (
        '<p class="boxdelta">'
        f"From run origin: x {h(delta_obj['dx_from_origin_mm'])} mm, "
        f"dist {h(delta_obj['ddist_from_origin_mm'])} mm."
        "</p>"
    )


def target_box_passed(status: dict[str, Any], x_tolerance_mm: float = 5.0,
                      target_dist_mm: float = 100.0, dist_tolerance_mm: float = 5.0) -> bool:
    if not status.get("ok"):
        return False
    return (
        abs(as_float(status.get("x_mm"), 9999.0)) <= x_tolerance_mm
        and abs(as_float(status.get("dist_mm"), 9999.0) - target_dist_mm) <= dist_tolerance_mm
    )


def verdict_card(title: str, passed: bool, detail: str) -> str:
    klass = "pass" if passed else "fail"
    label = "Win" if passed else "Failure"
    return (
        f'<article class="qa-card {klass}">'
        f'<span>{h(label)}</span>'
        f'<h2>{h(title)}</h2>'
        f'<p>{h(detail)}</p>'
        "</article>"
    )


def qa_summary_html(summary: dict[str, Any]) -> str:
    calibration = summary.get("motor_calibration") or {}
    direction_total = int(summary.get("direction_checks_total") or 0)
    direction_passed = int(summary.get("direction_checks_passed") or 0)
    guard_trips = int(summary.get("guard_trips") or 0)
    recovery_events = int(summary.get("recovery_events") or 0)
    skipped = int(summary.get("skipped") or 0)
    final_status = summary.get("final_status") or {}

    cards = [
        verdict_card(
            "Stage 1 Exact Run",
            bool(summary.get("stage1_exact")),
            str(summary.get("stage1_note") or "Not recorded."),
        ),
        verdict_card(
            "Motor Polarity Settings",
            bool(
                calibration.get("invert_left")
                and calibration.get("invert_right")
                and calibration.get("swap_left_right")
            ),
            (
                f"invert_left={calibration.get('invert_left')} "
                f"invert_right={calibration.get('invert_right')} "
                f"swap_left_right={calibration.get('swap_left_right')}"
            ),
        ),
        verdict_card(
            "Both-Motor Drive Rule",
            bool(summary.get("both_motor_rule")),
            (
                f"Every drive act must command both wheels at >= "
                f"{summary.get('min_drive_power', MIN_DRIVE_POWER)}% after calibration."
            ),
        ),
        verdict_card(
            "Direction Checks",
            direction_total > 0 and direction_passed == direction_total,
            f"{direction_passed}/{direction_total} expected movement directions passed.",
        ),
        verdict_card(
            "Guard Trips",
            guard_trips == 0 and skipped == 0,
            f"{guard_trips} guard trips, {skipped} skipped acts.",
        ),
        verdict_card(
            "Recovery Events",
            recovery_events == 0,
            f"{recovery_events} recovery events were needed during this run.",
        ),
        verdict_card(
            "100mm Alignment Box",
            target_box_passed(final_status),
            (
                "Final target requires |x| <= 5mm and dist 95..105mm. "
                f"Observed: {status_line(final_status)}."
            ),
        ),
    ]
    return '<section class="qa-summary">' + "\n".join(cards) + "</section>"


def movement_html(result: dict[str, Any]) -> str:
    movement = result["movement"]
    title = movement["title"]
    command = tuple(movement["command"])
    raw_command = tuple(result.get("raw_command") or command)
    duration_s = movement["duration_s"]
    requested_duration_s = result.get("requested_duration_s", duration_s)
    actual_duration_s = result.get("actual_duration_s")
    direction = result.get("direction_check")
    recovery_count = len(result.get("recovery") or [])
    status_class = "ok" if result.get("ok") else "warn"
    status_text = "Pass" if result.get("ok") else "Review"
    if result.get("ok") and recovery_count:
        status_class = "warn"
        status_text = "Recovered"
    if result.get("guard_trip"):
        status_class = "fail"
        status_text = "Guard Trip"
    if direction and not direction.get("passed"):
        status_class = "fail"
        status_text = "Direction Fail"
    if result.get("skipped"):
        status_class = "fail"
        status_text = "Skipped"
    before = result.get("before")
    after = result.get("after")

    if before and after:
        left = image_html(before, "Before")
        right = image_html(after, "After")
    else:
        left = '<figure><div class="missing">No before image</div></figure>'
        right = '<figure><div class="missing">No after image</div></figure>'

    guard = ""
    if result.get("guard_trip"):
        guard = f'<p class="guard">Guard tripped: {h(status_line(result["guard_trip"]))}</p>'
    recovery = ""
    if recovery_count:
        recovery = f'<p class="recovery">Recovery events: {recovery_count}</p>'

    return f"""
    <section class="movement">
      {left}
      <div class="act">
        <span class="pill {status_class}">{h(status_text)}</span>
        <h2>{h(title)}</h2>
        <p>{h(movement["description"])}</p>
        <code>semantic {h(command)} -> drive_triple{h(raw_command)}; requested {h(requested_duration_s)}s; actual {h(actual_duration_s if actual_duration_s is not None else "n/a")}s</code>
        {delta_html(result.get("delta"))}
        {direction_html(direction)}
        {box_html(result.get("box_delta"))}
        {guard}
        {recovery}
      </div>
      {right}
    </section>
    """


def write_report(run_dir: Path, results: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    summary = dict(summary)
    direction_checks = [
        result["direction_check"]
        for result in results
        if result.get("direction_check") is not None
    ]
    summary["direction_checks_total"] = len(direction_checks)
    summary["direction_checks_passed"] = sum(
        1 for check in direction_checks if check.get("passed")
    )
    summary["guard_trips"] = sum(1 for result in results if result.get("guard_trip"))
    summary["recovery_events"] = sum(len(result.get("recovery") or []) for result in results)
    summary["skipped"] = sum(1 for result in results if result.get("skipped"))
    summary["target_box_passed"] = target_box_passed(summary.get("final_status") or {})
    report_json = {"summary": summary, "results": results}
    (run_dir / "report.json").write_text(json.dumps(report_json, indent=2), encoding="utf-8")
    movements = "\n".join(movement_html(result) for result in results)
    qa_summary = qa_summary_html(summary)
    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Bun Controlled Movement Evidence</title>
  <style>
    :root {{
      color-scheme: light;
      --ink: #172026;
      --muted: #5f6b73;
      --line: #d9e0e5;
      --paper: #f7f8f5;
      --panel: #ffffff;
      --accent: #0f7b6c;
      --warn: #9b5c00;
      --fail: #a33128;
      --ok-bg: #dff3ea;
      --warn-bg: #fff2d8;
      --fail-bg: #ffe1dc;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font: 15px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
      background: var(--paper);
    }}
    header {{
      padding: 28px clamp(18px, 4vw, 48px) 18px;
      border-bottom: 1px solid var(--line);
      background: #ffffff;
    }}
    h1, h2, p {{ margin-top: 0; }}
    h1 {{ margin-bottom: 8px; font-size: clamp(26px, 4vw, 42px); letter-spacing: 0; }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      color: var(--muted);
    }}
    .meta span {{
      border: 1px solid var(--line);
      background: #f9fbfb;
      padding: 5px 8px;
      border-radius: 6px;
    }}
    main {{ padding: 18px clamp(12px, 3vw, 36px) 40px; }}
    .qa-summary {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 18px;
    }}
    .qa-card {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-left: 6px solid var(--muted);
      border-radius: 8px;
      padding: 12px;
      min-width: 0;
    }}
    .qa-card.pass {{ border-left-color: var(--accent); }}
    .qa-card.fail {{ border-left-color: var(--fail); }}
    .qa-card span {{
      display: inline-flex;
      margin-bottom: 6px;
      font-size: 12px;
      font-weight: 700;
      color: var(--muted);
    }}
    .qa-card.pass span {{ color: var(--accent); }}
    .qa-card.fail span {{ color: var(--fail); }}
    .qa-card h2 {{
      margin: 0 0 6px;
      font-size: 17px;
      letter-spacing: 0;
    }}
    .qa-card p {{ margin: 0; color: var(--muted); }}
    .movement {{
      display: grid;
      grid-template-columns: minmax(220px, 1fr) minmax(230px, 0.62fr) minmax(220px, 1fr);
      gap: 14px;
      align-items: stretch;
      padding: 16px 0;
      border-bottom: 1px solid var(--line);
    }}
    figure {{
      margin: 0;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      min-width: 0;
    }}
    img {{
      display: block;
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #111;
    }}
    figcaption {{
      display: flex;
      justify-content: space-between;
      gap: 8px;
      padding: 9px 10px;
      color: var(--muted);
      font-size: 13px;
    }}
    figcaption span {{ text-align: right; }}
    .act {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
      min-width: 0;
    }}
    .act h2 {{ margin: 8px 0 8px; font-size: 20px; letter-spacing: 0; }}
    .act p {{ color: var(--muted); margin-bottom: 10px; }}
    code {{
      display: block;
      white-space: normal;
      overflow-wrap: anywhere;
      padding: 9px;
      border-radius: 6px;
      border: 1px solid var(--line);
      background: #f6f8f8;
      font-size: 13px;
    }}
    .pill {{
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      padding: 3px 8px;
      border-radius: 999px;
      font-weight: 650;
      font-size: 12px;
    }}
    .pill.ok {{ color: #075f51; background: var(--ok-bg); }}
    .pill.warn {{ color: var(--warn); background: var(--warn-bg); }}
    .pill.fail {{ color: var(--fail); background: var(--fail-bg); }}
    .delta {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 6px;
      margin: 12px 0 0;
    }}
    .delta div {{
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
      background: #fbfcfc;
    }}
    dt {{ color: var(--muted); font-size: 12px; }}
    dd {{ margin: 2px 0 0; font-weight: 700; }}
    .guard, .recovery, .direction-warn {{ margin-top: 10px; color: var(--warn); }}
    .direction-ok {{ margin-top: 10px; color: var(--accent); }}
    .boxdelta {{ margin-top: 10px; color: var(--muted); }}
    .missing {{
      min-height: 220px;
      display: grid;
      place-items: center;
      padding: 18px;
      color: var(--muted);
      background: #eef1f2;
    }}
    @media (max-width: 900px) {{
      .qa-summary {{ grid-template-columns: 1fr; }}
      .movement {{ grid-template-columns: 1fr; }}
      figcaption {{ display: block; }}
      figcaption span {{ display: block; text-align: left; margin-top: 3px; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>Bun Controlled Movement Evidence</h1>
    <div class="meta">
      <span>Run: {h(summary["run_started_at"])}</span>
      <span>Completed: {h(summary["completed"])} / {h(summary["total"])}</span>
      <span>Direction checks: {h(summary["direction_checks_passed"])} / {h(summary["direction_checks_total"])}</span>
      <span>Max move: {h(MAX_MOVE_SECONDS)}s</span>
      <span>Min wheel: {h(summary.get("min_drive_power", MIN_DRIVE_POWER))}%</span>
      <span>Box: {h(summary.get("box_mm", 0))}mm</span>
      <span>Final: {h(status_line(summary["final_status"]))}</span>
    </div>
  </header>
  <main>
    {qa_summary}
    {movements}
  </main>
</body>
</html>
"""
    (run_dir / "index.html").write_text(html_text, encoding="utf-8")


def serve(directory: Path, host: str, port: int) -> None:
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args: Any, **kwargs: Any):
            super().__init__(*args, directory=str(directory), **kwargs)

    httpd = ThreadingHTTPServer((host, port), Handler)
    print(f"[site] serving {directory} at http://{host}:{port}/")
    print("[site] from your machine, try http://q1.local:%d/" % port)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[site] stopping")
    finally:
        httpd.server_close()


def run_sequence(args: argparse.Namespace) -> Path:
    run_dir, assets_dir = prepare_output_dir()
    link = MotorLink(dry_run=args.dry_run)
    link.connect()
    link.park()
    time.sleep(0.25)

    results: list[dict[str, Any]] = []
    last_good_x: float | None = None
    started_at = now_iso()
    baseline_status = read_status(args.vision_url)
    print(f"[run] origin: {status_line(baseline_status)} | box={args.box_mm}mm")

    for idx, movement in enumerate(MOVEMENTS, start=1):
        result, last_good_x, should_continue = run_movement(
            movement=movement,
            index=idx,
            base_url=args.vision_url,
            link=link,
            assets_dir=assets_dir,
            min_confidence=args.min_confidence,
            max_abs_x_mm=args.max_abs_x_mm,
            recover_abs_x_mm=args.recover_abs_x_mm,
            min_dist_mm=args.min_dist_mm,
            max_dist_mm=args.max_dist_mm,
            baseline_status=baseline_status,
            box_mm=args.box_mm,
            last_good_x=last_good_x,
        )
        results.append(result)
        write_report(
            run_dir,
            results,
            {
                "run_started_at": started_at,
                "completed": len(results),
                "total": len(MOVEMENTS),
                "final_status": read_status(args.vision_url),
                "origin_status": baseline_status,
                "box_mm": args.box_mm,
                "motor_calibration": {
                    "swap_left_right": SWAP_LEFT_RIGHT_MOTORS,
                    "invert_left": INVERT_LEFT_MOTOR,
                    "invert_right": INVERT_RIGHT_MOTOR,
                },
                "both_motor_rule": True,
                "min_drive_power": MIN_DRIVE_POWER,
                "stage1_exact": False,
                "stage1_note": (
                    "No. The branch and polarity toggles were applied, then SWAP_LEFT_RIGHT_MOTORS "
                    "was enabled after the bot steered away. The one-shot glide to the 100mm setpoint "
                    "did not pass cleanly."
                ),
            },
        )
        if not should_continue:
            print("[run] stopping early because recovery failed")
            break
        time.sleep(args.pause_s)

    link.park()
    time.sleep(0.25)
    summary = {
        "run_started_at": started_at,
        "run_finished_at": now_iso(),
        "completed": len(results),
        "total": len(MOVEMENTS),
        "final_status": read_status(args.vision_url),
        "origin_status": baseline_status,
        "box_mm": args.box_mm,
        "dry_run": args.dry_run,
        "vision_url": args.vision_url,
        "min_confidence": args.min_confidence,
        "max_abs_x_mm": args.max_abs_x_mm,
        "recover_abs_x_mm": args.recover_abs_x_mm,
        "motor_calibration": {
            "swap_left_right": SWAP_LEFT_RIGHT_MOTORS,
            "invert_left": INVERT_LEFT_MOTOR,
            "invert_right": INVERT_RIGHT_MOTOR,
        },
        "both_motor_rule": True,
        "min_drive_power": MIN_DRIVE_POWER,
        "stage1_exact": False,
        "stage1_note": (
            "No. The branch and polarity toggles were applied, then SWAP_LEFT_RIGHT_MOTORS "
            "was enabled after the bot steered away. The one-shot glide to the 100mm setpoint "
            "did not pass cleanly."
        ),
    }
    write_report(run_dir, results, summary)
    update_latest(run_dir)
    print(f"[run] wrote {run_dir / 'index.html'}")
    print(f"[run] latest report copied to {LATEST_DIR / 'index.html'}")
    return LATEST_DIR


def parse_args(argv: list[str]) -> argparse.Namespace:
    global OUT_ROOT, LATEST_DIR
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vision-url", default=VISION_URL)
    parser.add_argument("--output-root", default=str(OUT_ROOT))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--min-confidence", type=int, default=30)
    parser.add_argument("--max-abs-x-mm", type=float, default=145.0)
    parser.add_argument("--recover-abs-x-mm", type=float, default=105.0)
    parser.add_argument("--box-mm", type=float, default=0.0,
                        help="optional run-local x/dist box size; 100 means +/-50mm from origin")
    parser.add_argument("--min-dist-mm", type=float, default=70.0)
    parser.add_argument("--max-dist-mm", type=float, default=750.0)
    parser.add_argument("--pause-s", type=float, default=0.22)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--serve-only", action="store_true")
    parser.add_argument("--no-serve", action="store_true")
    args = parser.parse_args(argv)
    OUT_ROOT = Path(args.output_root)
    LATEST_DIR = OUT_ROOT / "latest"
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.serve_only:
        if not LATEST_DIR.exists():
            print(f"No report exists yet at {LATEST_DIR}; run without --serve-only first.")
            return 2
        serve(LATEST_DIR, args.host, args.port)
        return 0

    report_dir = run_sequence(args)
    if not args.no_serve:
        serve(report_dir, args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
