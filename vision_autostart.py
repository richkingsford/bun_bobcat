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


def vision_is_running(vision_url: str = DEFAULT_VISION_URL,
                      timeout_s: float = 0.5) -> bool:
    """True if the /status endpoint answers with valid JSON."""
    try:
        with urlopen(vision_url, timeout=timeout_s) as response:
            json.loads(response.read().decode("utf-8"))
        return True
    except (OSError, URLError, json.JSONDecodeError, ValueError):
        return False


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
    handle_file = log_path.open("a", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, "-u", str(STREAM_SCRIPT)],
        stdout=handle_file,
        stderr=subprocess.STDOUT,
        text=True,
    )
    emit("stream_launch", pid=proc.pid, log_path=str(log_path))

    deadline = time.monotonic() + ready_timeout_s
    while time.monotonic() < deadline:
        if vision_is_running(vision_url):
            emit("stream_ready", vision_url=vision_url)
            return StreamHandle(proc, started=True, ready=True, log_path=log_path)
        if proc.poll() is not None:
            # Stream died before serving — almost always "no OAK on USB". Pull
            # the tail of its log so the caller can show the real cause.
            tail = _log_tail(log_path)
            emit("stream_exited", returncode=proc.returncode,
                 log_path=str(log_path), log_tail=tail)
            return StreamHandle(proc, started=True, ready=False, log_path=log_path)
        time.sleep(0.25)

    emit("stream_not_ready", pid=proc.pid, log_path=str(log_path))
    return StreamHandle(proc, started=True, ready=False, log_path=log_path)


def _log_tail(log_path: Path, lines: int = 5) -> str:
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as f:
            return "".join(f.readlines()[-lines:]).strip()
    except OSError:
        return ""
