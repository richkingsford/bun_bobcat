"""
Dual-camera MJPEG livestream server.

  OAK-D Lite  — depthai CAM_A color     (see ../camera_oak_d_lite.txt)
  HBV HD CAM  — /dev/video2 V4L2/opencv  (see ../camera_hbv.txt)

URLs:
  http://192.168.1.44:8080/          — combined view (both cameras)
  http://192.168.1.44:8080/stream/oak
  http://192.168.1.44:8080/stream/hbv
"""

import time
import threading

import cv2
import depthai as dai
from flask import Flask, Response

HBV_DEVICE = 2  # /dev/video2 — HBV HD CAMERA
PORT = 8080

app = Flask(__name__)

_oak_frame: bytes = b""
_hbv_frame: bytes = b""
_lock_oak = threading.Lock()
_lock_hbv = threading.Lock()


# ── camera threads ────────────────────────────────────────────────────────────

def oak_capture_thread():
    global _oak_frame
    pipeline = dai.Pipeline()
    cap = dai.ImgFrameCapability()
    cap.size.fixed((1280, 720))
    cap.fps.fixed(15)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    q = cam.requestOutput(cap, True).createOutputQueue(maxSize=2, blocking=False)
    pipeline.start()
    print("OAK-D Lite stream started")
    while True:
        pkt = q.tryGet()
        if pkt is not None:
            _, buf = cv2.imencode(".jpg", pkt.getCvFrame(), [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _lock_oak:
                _oak_frame = buf.tobytes()
        else:
            time.sleep(0.01)


def hbv_capture_thread():
    global _hbv_frame
    cap = cv2.VideoCapture(HBV_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 15)
    print("HBV stream started")
    while True:
        ret, frame = cap.read()
        if ret:
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _lock_hbv:
                _hbv_frame = buf.tobytes()
        else:
            time.sleep(0.05)


# ── MJPEG helpers ─────────────────────────────────────────────────────────────

def mjpeg_stream(get_frame):
    while True:
        frame = get_frame()
        if frame:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        time.sleep(0.033)


def get_oak():
    with _lock_oak:
        return _oak_frame


def get_hbv():
    with _lock_hbv:
        return _hbv_frame


# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route("/stream/oak")
def stream_oak():
    return Response(mjpeg_stream(get_oak),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stream/hbv")
def stream_hbv():
    return Response(mjpeg_stream(get_hbv),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return """<!doctype html>
<html>
<head>
  <title>Bot Cameras</title>
  <style>
    body { background:#111; color:#eee; font-family:sans-serif; margin:0; padding:16px; }
    h1   { margin:0 0 12px; font-size:1.1rem; letter-spacing:.05em; }
    .grid { display:flex; gap:12px; flex-wrap:wrap; }
    .cam  { flex:1; min-width:300px; }
    .cam h2 { font-size:.85rem; margin:0 0 6px; color:#aaa; }
    .cam img { width:100%; border-radius:6px; display:block; }
  </style>
</head>
<body>
  <h1>Bot Cameras — live</h1>
  <div class="grid">
    <div class="cam">
      <h2>OAK-D Lite — color (1280&times;720)</h2>
      <img src="/stream/oak">
    </div>
    <div class="cam">
      <h2>HBV HD Camera — (1280&times;720)</h2>
      <img src="/stream/hbv">
    </div>
  </div>
</body>
</html>"""


# ── entry point ───────────────────────────────────────────────────────────────

def loop():
    threading.Thread(target=oak_capture_thread, daemon=True).start()
    threading.Thread(target=hbv_capture_thread, daemon=True).start()
    time.sleep(1)
    print(f"\n  Live feed: http://192.168.1.44:{PORT}/\n")
    app.run(host="0.0.0.0", port=PORT, threaded=True)


try:
    from arduino.app_utils import App
    App.run(user_loop=loop)
except ModuleNotFoundError:
    loop()
