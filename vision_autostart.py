#!/usr/bin/env python3
"""Shared brick-vision auto-start helper for Bun's standalone scripts.

main.py already brings the OAK-D stream up on its own via launch_stream(); the
standalone scripts (align_bounded_once.py, full_calibration.py, ...) historically
did not — if nobody had run python/brick_vision/stream.py by hand they would just
poll a dead endpoint and coast in silence, which looks identical to "the brick is
lost". This helper closes that gap: call ensure_stream() at the top of a run so the
script launches the stream itself, and — when the camera is missing — surfaces the
real reason (the stream's own stderr, e.g. "no DepthAI/OAK device found on USB")
instead of pretending the brick simply isn't in frame.

Behaviour mirrors main.py.launch_stream():
  * no-op if /status already answers (someone else owns the stream);
  * otherwise spawn stream.py and wait up to ready_timeout_s for it to serve;
  * a StreamHandle is returned so the caller can stop() the stream on exit, but
    ONLY if this process is the one that started it.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

REPO_ROOT = Path(__file__).resolve().parent
STREAM_SCRIPT = REPO_ROOT / "python" / "brick_vision" / "stream.py"
DEFAULT_VISION_URL = "http://127.0.0.1:8080/status"
DEFAULT_READY_TIMEOUT_S = 8.0
DEFAULT_STREAM_LOG = REPO_ROOT / "logs" / "step1_master" / "stream_latest.log"
OAK_USB_VID = "03e7"
DEFAULT_USB_SETTLE_S = 20.0
DEFAULT_START_ATTEMPTS = 2


def vision_is_running(vision_url: str = DEFAULT_VISION_URL,
                      timeout_s: float = 0.5) -> bool:
    """True if the /status endpoint answers with valid JSON."""
    try:
        with urlopen(vision_url, timeout=timeout_s) as response:
            json.loads(response.read().decode("utf-8"))
        return True
    except (OSError, URLError, json.JSONDecodeError, ValueError):
        return False


def oak_usb_devices() -> list[str]:
    """Return lsusb rows for OAK/Movidius devices currently on the bus."""
    try:
        result = subprocess.run(
            ["lsusb"], capture_output=True, text=True, timeout=5
        )
    except (OSError, subprocess.SubprocessError):
        return []
    if result.returncode != 0:
        return []

    needle = f"id {OAK_USB_VID}:"
    return [
        line.strip()
        for line in result.stdout.splitlines()
        if needle in line.lower()
    ]


def wait_for_oak_usb(timeout_s: float = DEFAULT_USB_SETTLE_S,
                     poll_s: float = 0.75) -> list[str]:
    """Wait briefly for the OAK to finish boot-time USB enumeration."""
    deadline = time.monotonic() + max(0.0, timeout_s)
    while True:
        devices = oak_usb_devices()
        if devices or time.monotonic() >= deadline:
            return devices
        time.sleep(poll_s)


class StreamHandle:
    """Bookkeeping for a stream this process may or may not have started."""

    def __init__(self, proc: "subprocess.Popen | None", started: bool,
                 ready: bool, log_path: "Path | None"):
        self.proc = proc          # the Popen we spawned, or None
        self.started = started    # True only if WE launched it
        self.ready = ready        # True if /status answered before we returned
        self.log_path = log_path

    def stop(self, timeout_s: float = 2.0) -> None:
        """Tear the stream down — but only if this process started it.

        If the stream was already up when ensure_stream() ran, another owner
        (e.g. main.py or a manual launch) is responsible for it; leave it alone.
        """
        if not self.started or self.proc is None or self.proc.poll() is not None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=timeout_s)


def ensure_stream(vision_url: str = DEFAULT_VISION_URL,
                  ready_timeout_s: float = DEFAULT_READY_TIMEOUT_S,
                  log_path: "Path | str | None" = None,
                  usb_settle_s: float = DEFAULT_USB_SETTLE_S,
                  start_attempts: int = DEFAULT_START_ATTEMPTS,
                  on_event=None) -> StreamHandle:
    """Make sure the brick-vision stream is serving, launching it if needed.

    on_event, if given, is called as on_event(event_name, **fields) for each
    lifecycle step so the caller can fold it into its own JSONL log.
    """

    def emit(event: str, **fields: object) -> None:
        if on_event is not None:
            on_event(event, **fields)

    if vision_is_running(vision_url):
        emit("stream_already_running", vision_url=vision_url)
        return StreamHandle(None, started=False, ready=True, log_path=None)

    log_path = Path(log_path) if log_path is not None else DEFAULT_STREAM_LOG
    log_path.parent.mkdir(parents=True, exist_ok=True)
    attempts = max(1, int(start_attempts))
    last_proc = None
    for attempt in range(1, attempts + 1):
        devices = oak_usb_devices()
        if devices:
            emit("oak_usb_seen", attempt=attempt, devices=devices)
        else:
            emit("oak_usb_wait", attempt=attempt, timeout_s=usb_settle_s)
            devices = wait_for_oak_usb(usb_settle_s)
            if devices:
                emit("oak_usb_seen", attempt=attempt, devices=devices)
            else:
                emit("oak_usb_missing", attempt=attempt)

        handle_file = log_path.open("a", encoding="utf-8")
        proc = subprocess.Popen(
            [sys.executable, "-u", str(STREAM_SCRIPT)],
            stdout=handle_file,
            stderr=subprocess.STDOUT,
            text=True,
        )
        handle_file.close()
        last_proc = proc
        emit("stream_launch", attempt=attempt, pid=proc.pid, log_path=str(log_path))

        deadline = time.monotonic() + ready_timeout_s
        while time.monotonic() < deadline:
            if vision_is_running(vision_url):
                emit("stream_ready", attempt=attempt, vision_url=vision_url)
                return StreamHandle(proc, started=True, ready=True,
                                    log_path=log_path)
            if proc.poll() is not None:
                # Stream died before serving. On cold boot the OAK can appear
                # seconds after the first failed scan, so retry through the
                # same path instead of handing the caller a permanent failure.
                tail = _log_tail(log_path)
                emit("stream_exited", attempt=attempt,
                     returncode=proc.returncode, log_path=str(log_path),
                     log_tail=tail)
                break
            time.sleep(0.25)
        else:
            emit("stream_not_ready", attempt=attempt, pid=proc.pid,
                 log_path=str(log_path))
            return StreamHandle(proc, started=True, ready=False,
                                log_path=log_path)

    return StreamHandle(last_proc, started=last_proc is not None, ready=False,
                        log_path=log_path)


def _log_tail(log_path: Path, lines: int = 5) -> str:
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as f:
            return "".join(f.readlines()[-lines:]).strip()
    except OSError:
        return ""
