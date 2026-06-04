#!/usr/bin/env python3
"""Small-pulse brick alignment trials with website evidence for Bun.

This supersedes the broad controlled-movement gallery for alignment work. It
uses a receding-horizon loop: read a stable brick pose, choose one short
calibrated two-track pulse, capture before/after evidence, score the result,
then adapt the next pulse.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import shutil
import statistics
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.request import urlopen

import host_controller as motor_calibration


VISION_URL = "http://127.0.0.1:8080"
OUT_ROOT = Path(".cache/controlled_movements")
LATEST_DIR = OUT_ROOT / "latest"
TARGET_DIST_MM = 100.0
TARGET_X_MM = 0.0
CAPTURE_DIST_MM = 5.0
CAPTURE_X_MM = 5.0
X_FUNNEL_MM = 35.0
MIN_CONFIDENCE = 55
MAX_ABS_X_MM = 180.0
MIN_DIST_MM = 65.0
MAX_DIST_MM = 700.0
MAX_ITERATIONS = 10
MIN_PULSE_S = 0.25
MAX_PULSE_S = 0.32
MAX_X_TRIM_S = 0.32
MIN_EFFECTIVE_PWM = 35
MAX_PWM = 65
EMA_ALPHA = 0.35
USE_EMA_FOR_PULSE_EVIDENCE = False
MEDIAN_WINDOW = 3
SETTLE_S = 0.45
COMMAND_REFRESH_S = 0.05
RECOVERY_RIGHT_BACK_PWM = 60
RECOVERY_RIGHT_BACK_PULSES = 3
RECOVERY_RIGHT_BACK_DURATION_S = 0.25
RECOVERY_RIGHT_BACK_PAUSE_S = 0.12
RECOVERY_X_TRIM_PWM = 55
RECOVERY_X_TRIM_DURATION_S = 0.10


@dataclass
class Pose:
    ok: bool
    found: bool = False
    confidence: int = 0
    x_mm: float | None = None
    y_mm: float | None = None
    dist_mm: float | None = None
    dist_source: str | None = None
    valid: bool = False
    reason: str = ""
    ts: str = ""
    bbox: dict[str, Any] | None = None
    quality: dict[str, Any] | None = None


@dataclass
class Command:
    action: str
    semantic_left: int
    semantic_right: int
    semantic_mast: int
    raw_left: int
    raw_right: int
    raw_mast: int
    duration_s: float
    rationale: str


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def h(value: Any) -> str:
    return html.escape(str(value), quote=True)


def status_url(base_url: str) -> str:
    return base_url.rstrip("/") + "/status"


def image_url(base_url: str) -> str:
    return base_url.rstrip("/") + "/detect.jpg"


def raw_status(base_url: str, timeout_s: float = 0.35) -> dict[str, Any]:
    with urlopen(status_url(base_url), timeout=timeout_s) as response:
        return json.loads(response.read().decode("utf-8"))


def pose_from_payload(payload: dict[str, Any], *, min_confidence: int) -> Pose:
    detection = payload.get("detection") or {}
    spatial = detection.get("spatial") or {}
    pose = Pose(
        ok=False,
        found=bool(detection.get("found")),
        confidence=int(detection.get("confidence") or 0),
        x_mm=spatial.get("x_mm"),
        y_mm=spatial.get("y_mm"),
        dist_mm=spatial.get("dist_mm"),
        dist_source=spatial.get("dist_source"),
        valid=bool(spatial.get("valid")),
        bbox=detection.get("bbox"),
        quality=detection.get("quality"),
        ts=now_iso(),
    )
    if not pose.found:
        pose.reason = detection.get("reason") or "brick not found"
        return pose
    if not pose.valid:
        pose.reason = "invalid spatial fix"
        return pose
    if pose.confidence < min_confidence:
        quality = pose.quality or {}
        detail = quality.get("bbox_note") or quality.get("bbox_source") or ""
        suffix = f" ({detail})" if detail else ""
        pose.reason = f"confidence {pose.confidence} below {min_confidence}{suffix}"
        return pose
    try:
        pose.x_mm = float(pose.x_mm)
        pose.y_mm = float(pose.y_mm)
        pose.dist_mm = float(pose.dist_mm)
    except (TypeError, ValueError):
        pose.reason = "non-numeric pose"
        return pose
    if abs(pose.x_mm) > MAX_ABS_X_MM:
        pose.reason = f"x outside safety gate: {pose.x_mm:.1f}mm"
        return pose
    if not MIN_DIST_MM <= pose.dist_mm <= MAX_DIST_MM:
        pose.reason = f"dist outside safety gate: {pose.dist_mm:.1f}mm"
        return pose
    pose.ok = True
    pose.reason = "ok"
    return pose


def read_pose(base_url: str, *, min_confidence: int = MIN_CONFIDENCE) -> Pose:
    try:
        return pose_from_payload(raw_status(base_url), min_confidence=min_confidence)
    except Exception as exc:
        return Pose(ok=False, reason=repr(exc), ts=now_iso())


def stable_pose(
    base_url: str,
    *,
    min_confidence: int,
    samples: int = MEDIAN_WINDOW,
    interval_s: float = 0.06,
    previous_ema: Pose | None = None,
) -> tuple[Pose, Pose | None, list[Pose]]:
    readings: list[Pose] = []
    for _ in range(samples):
        pose = read_pose(base_url, min_confidence=min_confidence)
        readings.append(pose)
        if len(readings) < samples:
            time.sleep(interval_s)

    valid = [pose for pose in readings if pose.ok]
    if not valid:
        return readings[-1], previous_ema, readings
    representative_pose = max(valid, key=lambda p: p.confidence)

    median_pose = Pose(
        ok=True,
        found=True,
        confidence=int(round(statistics.median(p.confidence for p in valid))),
        x_mm=float(statistics.median(p.x_mm for p in valid if p.x_mm is not None)),
        y_mm=float(statistics.median(p.y_mm for p in valid if p.y_mm is not None)),
        dist_mm=float(statistics.median(p.dist_mm for p in valid if p.dist_mm is not None)),
        dist_source=max({p.dist_source for p in valid}, key=[p.dist_source for p in valid].count),
        valid=True,
        reason="median",
        ts=now_iso(),
        bbox=representative_pose.bbox,
        quality=representative_pose.quality,
    )
    if USE_EMA_FOR_PULSE_EVIDENCE and previous_ema and previous_ema.ok:
        median_pose.x_mm = EMA_ALPHA * median_pose.x_mm + (1.0 - EMA_ALPHA) * float(previous_ema.x_mm)
        median_pose.y_mm = EMA_ALPHA * median_pose.y_mm + (1.0 - EMA_ALPHA) * float(previous_ema.y_mm)
        median_pose.dist_mm = EMA_ALPHA * median_pose.dist_mm + (1.0 - EMA_ALPHA) * float(previous_ema.dist_mm)
        median_pose.reason = "median+ema"
    return median_pose, median_pose, readings


def pose_line(pose: Pose | None) -> str:
    if pose is None:
        return "none"
    lock_detail = ""
    if pose.bbox:
        bbox = pose.bbox
        quality = pose.quality or {}
        source = quality.get("bbox_source") or "bbox"
        lock_detail = f", lock={source} {bbox.get('w')}x{bbox.get('h')}px"
    if not pose.ok:
        return f"not usable ({pose.reason}{lock_detail})"
    return (
        f"x={pose.x_mm:+.1f}mm, y={pose.y_mm:+.1f}mm, "
        f"dist={pose.dist_mm:.1f}mm, conf={pose.confidence}, src={pose.dist_source}{lock_detail}"
    )


def error_score(pose: Pose | None) -> float | None:
    if pose is None or not pose.ok or pose.x_mm is None or pose.dist_mm is None:
        return None
    return abs(pose.dist_mm - TARGET_DIST_MM) + 1.15 * abs(pose.x_mm - TARGET_X_MM)


def in_capture_box(pose: Pose | None) -> bool:
    if pose is None or not pose.ok or pose.x_mm is None or pose.dist_mm is None:
        return False
    return (
        abs(pose.x_mm - TARGET_X_MM) <= CAPTURE_X_MM
        and abs(pose.dist_mm - TARGET_DIST_MM) <= CAPTURE_DIST_MM
    )


def calibrated_command(left: int, right: int, mast: int = 0) -> tuple[int, int, int]:
    if motor_calibration.SWAP_LEFT_RIGHT_MOTORS:
        left, right = right, left
    if motor_calibration.INVERT_LEFT_MOTOR:
        left = -left
    if motor_calibration.INVERT_RIGHT_MOTOR:
        right = -right
    return int(left), int(right), int(mast)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def shape_drive(left: float, right: float) -> tuple[int, int]:
    peak = max(abs(left), abs(right), 1.0)
    if peak > MAX_PWM:
        scale = MAX_PWM / peak
        left *= scale
        right *= scale
    for name, value in (("left", left), ("right", right)):
        if abs(value) < MIN_EFFECTIVE_PWM:
            value = math.copysign(MIN_EFFECTIVE_PWM, value if value else 1.0)
        if name == "left":
            left = value
        else:
            right = value
    return int(round(left)), int(round(right))


def choose_command(
    pose: Pose,
    *,
    drive_sign: int,
    steer_sign: int,
    duration_scale: float,
    iteration: int,
) -> Command:
    assert pose.x_mm is not None and pose.dist_mm is not None
    dist_error = pose.dist_mm - TARGET_DIST_MM
    x_error = pose.x_mm - TARGET_X_MM

    if abs(x_error) > X_FUNNEL_MM:
        left = 55 if x_error > 0 else -55
        right = -55 if x_error > 0 else 55
        duration = clamp(0.25 + min(180.0, abs(x_error)) * 0.0004, 0.25, MAX_X_TRIM_S)
        left_i, right_i = shape_drive(left, right)
        raw_left, raw_right, raw_mast = calibrated_command(left_i, right_i, 0)
        return Command(
            action=f"x_trim_{iteration:02d}",
            semantic_left=left_i,
            semantic_right=right_i,
            semantic_mast=0,
            raw_left=raw_left,
            raw_right=raw_right,
            raw_mast=raw_mast,
            duration_s=round(duration * duration_scale, 3),
            rationale=(
                "x-first trim: lateral error is outside the approach funnel, "
                "so Bun closes x before trying to close distance."
            ),
        )

    if dist_error < -CAPTURE_DIST_MM:
        base = -drive_sign * clamp(36 + abs(dist_error) * 0.6, 40, 58)
        steer = steer_sign * clamp(abs(x_error) * 0.55, 0, 16)
        duration = clamp(0.08 + abs(dist_error) * 0.003, 0.08, MAX_PULSE_S)
        action = "back_off"
        rationale = "too close; reverse arc away from stack while reducing lateral error"
    elif abs(x_error) > 35 and dist_error < 35:
        base = 0
        steer = steer_sign * math.copysign(52, x_error)
        duration = 0.08
        action = "pivot_trim"
        rationale = "near target distance with large x; short counter-rotation trim"
    else:
        base = drive_sign * clamp(38 + dist_error * 0.16, 42, 62)
        steer = 0.0 if abs(x_error) <= CAPTURE_X_MM else steer_sign * clamp(abs(x_error) * 0.55, 0, 18)
        duration = clamp(0.09 + min(80.0, max(0.0, dist_error)) * 0.0013, 0.09, MAX_PULSE_S)
        action = "approach_arc" if abs(x_error) > CAPTURE_X_MM else "straight_approach"
        rationale = "bounded approach pulse; both tracks stay above effective PWM"

    if base == 0:
        turn = steer_sign * math.copysign(abs(steer), x_error)
        left = turn
        right = -turn
    else:
        correction = steer_sign * math.copysign(abs(steer), x_error)
        left = base + correction
        right = base - correction

    left_i, right_i = shape_drive(left, right)
    duration = clamp(duration * duration_scale, MIN_PULSE_S, max(MIN_PULSE_S, MAX_PULSE_S))
    raw_left, raw_right, raw_mast = calibrated_command(left_i, right_i, 0)
    return Command(
        action=f"{action}_{iteration:02d}",
        semantic_left=left_i,
        semantic_right=right_i,
        semantic_mast=0,
        raw_left=raw_left,
        raw_right=raw_right,
        raw_mast=raw_mast,
        duration_s=round(duration, 3),
        rationale=rationale,
    )


class MotorLink:
    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self.bridge = None

    def connect(self) -> None:
        if self.dry_run:
            return
        from arduino.app_utils import Bridge

        self.bridge = Bridge
        self.park()

    def send(self, command: Command) -> None:
        if self.dry_run:
            print(
                f"[dry] drive_triple({command.raw_left}, {command.raw_right}, "
                f"{command.raw_mast})"
            )
            return
        assert self.bridge is not None
        self.bridge.notify("drive_triple", command.raw_left, command.raw_right, command.raw_mast)

    def park(self) -> None:
        if self.dry_run:
            return
        if self.bridge is None:
            from arduino.app_utils import Bridge
            self.bridge = Bridge
        for _ in range(4):
            self.bridge.notify("drive_triple", 0, 0, 0)
            self.bridge.notify("mast", 0)
            time.sleep(0.018)


def capture(base_url: str, assets_dir: Path, stem: str) -> dict[str, Any]:
    image_name = f"{stem}.jpg"
    image_path = assets_dir / image_name
    error = None
    try:
        with urlopen(image_url(base_url), timeout=1.2) as response:
            image_path.write_bytes(response.read())
        image_ref = f"assets/{image_name}"
    except Exception as exc:
        error = repr(exc)
        image_ref = None
    pose = read_pose(base_url, min_confidence=0)
    return {
        "image": image_ref,
        "image_error": error,
        "pose": asdict(pose),
        "captured_at": now_iso(),
    }


def run_pulse(link: MotorLink, command: Command) -> None:
    deadline = time.monotonic() + command.duration_s
    next_refresh = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_refresh:
            link.send(command)
            next_refresh = now + COMMAND_REFRESH_S
        time.sleep(0.015)
    link.park()


def wheel_back_recovery_command(iteration: int, wheel: str = "right") -> Command:
    semantic_left = -RECOVERY_RIGHT_BACK_PWM if wheel == "left" else 0
    semantic_right = -RECOVERY_RIGHT_BACK_PWM if wheel == "right" else 0
    raw_left, raw_right, raw_mast = calibrated_command(semantic_left, semantic_right, 0)
    return Command(
        action=f"{wheel}_wheel_back_recovery_{iteration:02d}",
        semantic_left=semantic_left,
        semantic_right=semantic_right,
        semantic_mast=0,
        raw_left=raw_left,
        raw_right=raw_right,
        raw_mast=raw_mast,
        duration_s=RECOVERY_RIGHT_BACK_DURATION_S,
        rationale=(
            "Recovery exception: right wheel backward pulse to undo left-turn "
            "runaway and bring the brick back into view."
        ),
    )


def x_trim_recovery_command(iteration: int, x_mm: float) -> Command:
    semantic_left = RECOVERY_X_TRIM_PWM if x_mm > 0 else -RECOVERY_X_TRIM_PWM
    semantic_right = -RECOVERY_X_TRIM_PWM if x_mm > 0 else RECOVERY_X_TRIM_PWM
    raw_left, raw_right, raw_mast = calibrated_command(semantic_left, semantic_right, 0)
    return Command(
        action=f"x_trim_recovery_{iteration:02d}",
        semantic_left=semantic_left,
        semantic_right=semantic_right,
        semantic_mast=0,
        raw_left=raw_left,
        raw_right=raw_right,
        raw_mast=raw_mast,
        duration_s=RECOVERY_X_TRIM_DURATION_S,
        rationale=(
            "Recovery: brick is visible but outside the x/confidence gate, "
            "so Bun uses a mirrored two-track trim toward center."
        ),
    )


def run_recovery(
    *,
    link: MotorLink,
    base_url: str,
    assets_dir: Path,
    iteration: int,
    min_confidence: int,
    hint_pose: Pose | None = None,
) -> tuple[list[dict[str, Any]], Pose]:
    events = []
    for pulse in range(1, RECOVERY_RIGHT_BACK_PULSES + 1):
        live_hint = read_pose(base_url, min_confidence=0)
        pose_hint = live_hint if live_hint.found and live_hint.x_mm is not None else hint_pose
        if pose_hint and pose_hint.found and pose_hint.x_mm is not None:
            command = x_trim_recovery_command(iteration, float(pose_hint.x_mm))
        else:
            command = wheel_back_recovery_command(iteration, "right")
        before = capture(base_url, assets_dir, f"{iteration:02d}_recovery_{pulse}_before")
        run_pulse(link, command)
        time.sleep(RECOVERY_RIGHT_BACK_PAUSE_S)
        after = capture(base_url, assets_dir, f"{iteration:02d}_recovery_{pulse}_after")
        pose = read_pose(base_url, min_confidence=0)
        events.append({
            "pulse": pulse,
            "command": asdict(command),
            "before": before,
            "after": after,
            "pose": asdict(pose),
        })
    final_pose, _, _ = stable_pose(base_url, min_confidence=min_confidence)
    return events, final_pose


def prepare_run_dir() -> tuple[Path, Path]:
    run_dir = OUT_ROOT / datetime.now(timezone.utc).strftime("align_%Y%m%dT%H%M%SZ")
    assets_dir = run_dir / "assets"
    assets_dir.mkdir(parents=True, exist_ok=False)
    return run_dir, assets_dir


def copy_latest(run_dir: Path) -> None:
    if LATEST_DIR.exists() or LATEST_DIR.is_symlink():
        shutil.rmtree(LATEST_DIR)
    shutil.copytree(run_dir, LATEST_DIR)


def adapt_after_trial(
    *,
    before: Pose,
    after: Pose,
    command: Command,
    drive_sign: int,
    steer_sign: int,
    duration_scale: float,
) -> tuple[int, int, float, str]:
    before_score = error_score(before)
    after_score = error_score(after)
    note = "kept gains"
    if before_score is None or after_score is None:
        return drive_sign, steer_sign, max(0.72, duration_scale * 0.8), "vision failed; shortened next pulse"

    dist_delta = float(after.dist_mm) - float(before.dist_mm)
    x_before = abs(float(before.x_mm))
    x_after = abs(float(after.x_mm))

    if command.action.startswith(("approach", "straight")) and dist_delta > 1.5:
        drive_sign *= -1
        duration_scale = max(0.70, duration_scale * 0.82)
        note = "approach increased distance; flipped drive sign and shortened pulse"
    elif x_after > x_before + 4.0 and abs(float(before.x_mm)) > CAPTURE_X_MM:
        steer_sign *= -1
        duration_scale = max(0.72, duration_scale * 0.88)
        note = "x error grew; flipped steering sign and shortened pulse"
    elif after_score > before_score + 3.0:
        duration_scale = max(0.72, duration_scale * 0.82)
        note = "aggregate error worsened; shortened next pulse"
    elif after_score < before_score - 8.0:
        duration_scale = min(1.0, duration_scale * 1.04)
        note = "strong improvement; kept/very slightly relaxed pulse scale"
    return drive_sign, steer_sign, duration_scale, note


def trial_status(before: Pose, after: Pose) -> tuple[str, str]:
    if in_capture_box(after):
        return "capture", "Inside 10mm x 10mm box around x=0 and dist=100."
    before_score = error_score(before)
    after_score = error_score(after)
    if before_score is None or after_score is None:
        return "fault", "Vision was not usable after the pulse."
    if after_score < before_score - 2.0:
        return "improved", f"Aggregate error improved by {before_score - after_score:.1f}mm."
    if after_score <= before_score + 2.0:
        return "neutral", "No meaningful progress; next pulse adapts conservatively."
    return "worse", f"Aggregate error worsened by {after_score - before_score:.1f}mm."


def image_html(snapshot: dict[str, Any], label: str) -> str:
    pose = Pose(**(snapshot.get("pose") or {"ok": False, "reason": "missing"}))
    image = snapshot.get("image")
    if image:
        media = f'<img src="{h(image)}" alt="{h(label)} camera evidence">'
    else:
        media = f'<div class="missing">{h(snapshot.get("image_error") or "No image")}</div>'
    return (
        f"<figure>{media}<figcaption><strong>{h(label)}</strong>"
        f"<span>{h(pose_line(pose))}</span></figcaption></figure>"
    )


def trial_html(trial: dict[str, Any]) -> str:
    status = trial["status"]
    before_pose = Pose(**trial["before_pose"])
    after_pose = Pose(**trial["after_pose"])
    before_score = error_score(before_pose)
    after_score = error_score(after_pose)
    command = trial["command"]
    recovery = trial.get("recovery") or []
    recovery_html = ""
    if recovery:
        recovery_html = (
            f'<p class="recovery">Recovery: {len(recovery)} scripted '
            f'pulses. '
            f'Final recovery pose: {h(pose_line(Pose(**trial["after_pose"])))}.</p>'
        )
    score_text = (
        "n/a"
        if before_score is None or after_score is None
        else f"{before_score:.1f} -> {after_score:.1f}"
    )
    return f"""
    <section class="trial {h(status)}">
      {image_html(trial["before"], "Before")}
      <div class="act">
        <span class="pill {h(status)}">{h(status)}</span>
        <h2>Iteration {h(trial["iteration"])}: {h(command["action"])}</h2>
        <p>{h(command["rationale"])}</p>
        <code>semantic L={h(command["semantic_left"])} R={h(command["semantic_right"])} M={h(command["semantic_mast"])} -> raw drive_triple({h(command["raw_left"])}, {h(command["raw_right"])}, {h(command["raw_mast"])}), {h(command["duration_s"])}s</code>
        <dl class="metrics">
          <div><dt>Before</dt><dd>{h(pose_line(before_pose))}</dd></div>
          <div><dt>After</dt><dd>{h(pose_line(after_pose))}</dd></div>
          <div><dt>Error score</dt><dd>{h(score_text)}</dd></div>
        </dl>
        <p class="verdict">{h(trial["verdict"])}</p>
        <p class="learning">{h(trial["learning_note"])}</p>
        {recovery_html}
      </div>
      {image_html(trial["after"], "After")}
    </section>
    """


def write_report(run_dir: Path, trials: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    (run_dir / "report.json").write_text(
        json.dumps({"summary": summary, "trials": trials}, indent=2),
        encoding="utf-8",
    )
    rows = "\n".join(trial_html(trial) for trial in trials)
    chart_points = []
    for trial in trials:
        after_pose = Pose(**trial["after_pose"])
        score = error_score(after_pose)
        if score is not None:
            chart_points.append(f"{trial['iteration']}:{score:.1f}")
    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Bun Brick Alignment Evidence</title>
  <style>
    :root {{
      color-scheme: light;
      --ink:#172026; --muted:#5f6b73; --line:#d8e0e4; --paper:#f6f8f5;
      --panel:#ffffff; --good:#0f7b6c; --warn:#9b5c00; --bad:#a33128;
      --good-bg:#ddf3ea; --warn-bg:#fff2d8; --bad-bg:#ffe1dc;
    }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; color:var(--ink); background:var(--paper); font:15px/1.45 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }}
    header {{ padding:24px clamp(16px,4vw,44px); background:#fff; border-bottom:1px solid var(--line); }}
    h1 {{ margin:0 0 8px; font-size:clamp(26px,4vw,40px); letter-spacing:0; }}
    h2,p {{ margin-top:0; }}
    .meta {{ display:flex; flex-wrap:wrap; gap:8px; color:var(--muted); }}
    .meta span {{ border:1px solid var(--line); background:#f9fbfb; padding:5px 8px; border-radius:6px; }}
    main {{ padding:18px clamp(12px,3vw,36px) 42px; }}
    .summary {{ display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:12px; margin-bottom:18px; }}
    .card {{ background:#fff; border:1px solid var(--line); border-radius:8px; padding:12px; }}
    .card strong {{ display:block; font-size:24px; }}
    .card span {{ color:var(--muted); }}
    .trial {{ display:grid; grid-template-columns:minmax(220px,1fr) minmax(260px,.75fr) minmax(220px,1fr); gap:14px; align-items:stretch; padding:16px 0; border-bottom:1px solid var(--line); }}
    figure {{ margin:0; background:#fff; border:1px solid var(--line); border-radius:8px; overflow:hidden; }}
    img {{ display:block; width:100%; aspect-ratio:4/3; object-fit:contain; background:#111; }}
    figcaption {{ display:flex; justify-content:space-between; gap:8px; padding:9px 10px; color:var(--muted); font-size:13px; }}
    figcaption span {{ text-align:right; }}
    .act {{ background:#fff; border:1px solid var(--line); border-radius:8px; padding:14px; min-width:0; }}
    .act h2 {{ margin:8px 0; font-size:20px; letter-spacing:0; }}
    code {{ display:block; white-space:normal; overflow-wrap:anywhere; padding:9px; border:1px solid var(--line); border-radius:6px; background:#f6f8f8; font-size:13px; }}
    .pill {{ display:inline-flex; min-height:24px; align-items:center; padding:3px 8px; border-radius:999px; font-size:12px; font-weight:700; }}
    .pill.capture,.pill.improved,.pill.recovered {{ color:var(--good); background:var(--good-bg); }}
    .pill.neutral {{ color:var(--warn); background:var(--warn-bg); }}
    .pill.worse,.pill.fault {{ color:var(--bad); background:var(--bad-bg); }}
    .metrics {{ display:grid; gap:6px; margin:12px 0 0; }}
    .metrics div {{ border:1px solid var(--line); border-radius:6px; padding:8px; background:#fbfcfc; }}
    dt {{ color:var(--muted); font-size:12px; }}
    dd {{ margin:2px 0 0; font-weight:700; overflow-wrap:anywhere; }}
    .verdict {{ margin:10px 0 0; color:var(--ink); }}
    .recovery {{ margin:10px 0 0; color:var(--good); }}
    .learning {{ margin:6px 0 0; color:var(--muted); }}
    .missing {{ min-height:220px; display:grid; place-items:center; padding:18px; color:var(--muted); background:#eef1f2; }}
    @media (max-width:960px) {{ .summary,.trial {{ grid-template-columns:1fr; }} figcaption {{ display:block; }} figcaption span {{ display:block; text-align:left; margin-top:3px; }} }}
  </style>
</head>
<body>
  <header>
    <h1>Bun Brick Alignment Evidence</h1>
    <div class="meta">
      <span>Run: {h(summary["run_started_at"])}</span>
      <span>Goal: x=0mm, dist={h(TARGET_DIST_MM)}mm</span>
      <span>X funnel: +/-{h(X_FUNNEL_MM)}mm</span>
      <span>Iterations: {h(summary["completed"])} / {h(MAX_ITERATIONS)}</span>
      <span>Max pulse: {h(MAX_PULSE_S)}s</span>
      <span>Max x trim: {h(MAX_X_TRIM_S)}s</span>
      <span>Min wheel: {h(MIN_EFFECTIVE_PWM)}%</span>
      <span>Recovery: x-trim when visible, right-wheel-back when lost</span>
      <span>Score path: {h(", ".join(chart_points) or "n/a")}</span>
    </div>
  </header>
  <main>
    <section class="summary">
      <div class="card"><strong>{h(summary["captures"])}</strong><span>capture-box samples</span></div>
      <div class="card"><strong>{h(summary["improved"])}</strong><span>improving pulses</span></div>
      <div class="card"><strong>{h(summary["worse"])}</strong><span>worse pulses</span></div>
      <div class="card"><strong>{h(summary["final_pose"])}</strong><span>final pose</span></div>
    </section>
    {rows}
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
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()


def run_alignment(args: argparse.Namespace) -> Path:
    run_dir, assets_dir = prepare_run_dir()
    link = MotorLink(dry_run=args.dry_run)
    link.connect()
    link.park()
    time.sleep(0.2)

    trials: list[dict[str, Any]] = []
    started_at = now_iso()
    ema_pose: Pose | None = None
    drive_sign = 1
    # Seeded from Bun's previous real run: positive x grew when steer_sign=+1,
    # so start with the opposite arc and still let the online learner flip if
    # the evidence says otherwise.
    steer_sign = -1
    duration_scale = 1.0
    last_learning = "initial Leia-inspired pulse, seeded with Bun's observed steering sign"

    for iteration in range(1, args.iterations + 1):
        before_pose, ema_pose, samples = stable_pose(
            args.vision_url,
            min_confidence=args.min_confidence,
            previous_ema=ema_pose,
        )
        before_snapshot = capture(args.vision_url, assets_dir, f"{iteration:02d}_before")
        if not before_pose.ok:
            command = Command(
                action=f"fault_{iteration:02d}",
                semantic_left=0,
                semantic_right=0,
                semantic_mast=0,
                raw_left=0,
                raw_right=0,
                raw_mast=0,
                duration_s=0,
                rationale=f"no movement: {before_pose.reason}",
            )
            recovery_events, recovered_pose = run_recovery(
                link=link,
                base_url=args.vision_url,
                assets_dir=assets_dir,
                iteration=iteration,
                min_confidence=args.min_confidence,
                hint_pose=before_pose,
            )
            after_snapshot = capture(args.vision_url, assets_dir, f"{iteration:02d}_after")
            after_pose = recovered_pose
            status = "recovered" if after_pose.ok else "fault"
            verdict = (
                "Recovered brick lock with the scripted right-wheel-back sequence."
                if after_pose.ok
                else "Recovery sequence ran, but vision is still not usable."
            )
            trial = {
                "iteration": iteration,
                "command": asdict(command),
                "before": before_snapshot,
                "after": after_snapshot,
                "before_pose": asdict(before_pose),
                "after_pose": asdict(after_pose),
                "samples": [asdict(p) for p in samples],
                "status": status,
                "verdict": verdict,
                "learning_note": "Used fixed recovery instead of blind search.",
                "recovery": recovery_events,
            }
            trials.append(trial)
            if not after_pose.ok:
                break
            continue

        command = choose_command(
            before_pose,
            drive_sign=drive_sign,
            steer_sign=steer_sign,
            duration_scale=duration_scale,
            iteration=iteration,
        )
        print(
            f"[align] {iteration}/{args.iterations} {pose_line(before_pose)} -> "
            f"{command.action} semantic=({command.semantic_left},{command.semantic_right}) "
            f"raw=({command.raw_left},{command.raw_right}) {command.duration_s}s"
        )
        run_pulse(link, command)
        time.sleep(SETTLE_S)
        after_pose, ema_pose, after_samples = stable_pose(
            args.vision_url,
            min_confidence=args.min_confidence,
            previous_ema=ema_pose,
        )
        after_snapshot = capture(args.vision_url, assets_dir, f"{iteration:02d}_after")
        recovery_events = []
        status, verdict = trial_status(before_pose, after_pose)
        if status == "fault":
            recovery_events, recovered_pose = run_recovery(
                link=link,
                base_url=args.vision_url,
                assets_dir=assets_dir,
                iteration=iteration,
                min_confidence=args.min_confidence,
                hint_pose=after_pose,
            )
            if recovered_pose.ok:
                after_pose = recovered_pose
                after_snapshot = capture(args.vision_url, assets_dir, f"{iteration:02d}_after_recovered")
                status = "recovered"
                verdict = "Vision fault recovered with three right-wheel-back pulses."
        drive_sign, steer_sign, duration_scale, last_learning = adapt_after_trial(
            before=before_pose,
            after=after_pose,
            command=command,
            drive_sign=drive_sign,
            steer_sign=steer_sign,
            duration_scale=duration_scale,
        )
        trial = {
            "iteration": iteration,
            "command": asdict(command),
            "before": before_snapshot,
            "after": after_snapshot,
            "before_pose": asdict(before_pose),
            "after_pose": asdict(after_pose),
            "samples": [asdict(p) for p in samples],
            "after_samples": [asdict(p) for p in after_samples],
            "status": status,
            "verdict": verdict,
            "learning_note": last_learning,
            "recovery": recovery_events,
            "drive_sign_next": drive_sign,
            "steer_sign_next": steer_sign,
            "duration_scale_next": round(duration_scale, 3),
        }
        trials.append(trial)
        if args.stop_on_capture and in_capture_box(after_pose):
            break

    link.park()
    final_pose = None
    if trials:
        final_pose = Pose(**trials[-1]["after_pose"])
    summary = {
        "run_started_at": started_at,
        "run_finished_at": now_iso(),
        "completed": len(trials),
        "target_dist_mm": TARGET_DIST_MM,
        "target_x_mm": TARGET_X_MM,
        "x_funnel_mm": X_FUNNEL_MM,
        "captures": sum(1 for trial in trials if trial["status"] == "capture"),
        "improved": sum(1 for trial in trials if trial["status"] in ("capture", "improved")),
        "worse": sum(1 for trial in trials if trial["status"] == "worse"),
        "faults": sum(1 for trial in trials if trial["status"] == "fault"),
        "recoveries": sum(1 for trial in trials if trial["status"] == "recovered"),
        "final_pose": pose_line(final_pose),
        "min_confidence": args.min_confidence,
        "motor_calibration": {
            "swap_left_right": bool(motor_calibration.SWAP_LEFT_RIGHT_MOTORS),
            "invert_left": bool(motor_calibration.INVERT_LEFT_MOTOR),
            "invert_right": bool(motor_calibration.INVERT_RIGHT_MOTOR),
        },
        "source_files": [
            "leia_gap_closing_trials.csv",
            "brick_vision_model_lockdown.md",
            "leia_gap_closing_spec.json",
            "leia_gap_closing_spec_demo.html",
        ],
        "notes": [
            "Ported bounded receding-horizon visual servo behavior from the Leia spec.",
            "Rejected weak one-side/near-stall commands for Bun; every drive pulse uses both tracks above the effective PWM floor.",
            "No blind movement: a fault card is written instead of moving when vision is not confident.",
            "Known left-turn runaway recovery starts from the right-wheel-back primitive; visible off-side faults use mirrored x-trim recovery.",
            "After the 2026-06-04T02:06Z run, x-first trimming was added so Bun does not approach while lateral error is outside the funnel.",
            "Pulse evidence now uses median samples without EMA lag so each iteration learns from its own movement.",
            "Large x-trim duration is scaled up after a 0.119s trim produced only a tiny x change.",
        ],
    }
    write_report(run_dir, trials, summary)
    copy_latest(run_dir)
    return LATEST_DIR


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vision-url", default=VISION_URL)
    parser.add_argument("--iterations", type=int, default=MAX_ITERATIONS)
    parser.add_argument("--min-confidence", type=int, default=MIN_CONFIDENCE)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stop-on-capture", action="store_true")
    parser.add_argument("--no-serve", action="store_true")
    parser.add_argument("--serve-only", action="store_true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.serve_only:
        if not LATEST_DIR.exists():
            print(f"No report found at {LATEST_DIR}")
            return 2
        serve(LATEST_DIR, args.host, args.port)
        return 0
    report_dir = run_alignment(args)
    if not args.no_serve:
        serve(report_dir, args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
