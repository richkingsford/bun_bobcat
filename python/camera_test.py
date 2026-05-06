"""
Camera test: capture a single photo from a phone via WebSocket.

Run this app, then connect from the Arduino Lab mobile app using the
secret code printed in the logs. The first frame received is saved to
python/test_photo.jpg in the repo (mounted at /app).
"""

import secrets
import string
import os
import threading

import cv2
import numpy as np

from arduino.app_utils import App
from arduino.app_peripherals.camera import WebSocketCamera

OUTPUT_PATH = "/app/python/test_photo.jpg"
_saved = threading.Event()


def generate_secret(length=6) -> str:
    return "".join(secrets.choice(string.digits) for _ in range(length))


secret = generate_secret()
camera = WebSocketCamera(secret=secret, encrypt=True)


def on_status(evt_type, data):
    print(f"Camera status: {evt_type} — {data}")


camera.on_status_changed(on_status)


def loop():
    print("=== Camera Test ===")
    print(f"Connect from the Arduino Lab app using secret: {secret}")
    print(f"WebSocket: ws://{camera.ip}:{camera.port}")
    camera.start()
    print("Waiting for a frame...")

    for frame in camera.stream():
        if _saved.is_set():
            break
        if frame is None:
            continue
        _saved.set()
        cv2.imwrite(OUTPUT_PATH, frame)
        size = os.path.getsize(OUTPUT_PATH)
        print(f"Photo saved: {OUTPUT_PATH} ({size} bytes)")
        print("Done — you can stop the app.")
        break

    import time
    while True:
        time.sleep(1)


App.run(user_loop=loop)
