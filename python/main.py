"""
Dual-camera livestream with cyan brick tracking on OAK-D Lite.

  OAK-D Lite  — color feed + stereo depth; detects cyan, draws green outline,
                overlays distance / x-pos / y-pos in real time
  HBV HD CAM  — plain video feed

URLs:
  http://192.168.1.44:8080/          — combined view
  http://192.168.1.44:8080/stream/oak
  http://192.168.1.44:8080/stream/hbv
"""

import time
import threading

import cv2
import numpy as np
import depthai as dai
from flask import Flask, Response

try:
    from arduino.app_utils import App
except ModuleNotFoundError:
    App = None

HBV_DEVICE = 2   # /dev/video2 — HBV HD CAMERA (see ../camera_hbv.txt)
PORT = 8080

# Cyan detection range in HSV (hue 80-105 covers cyan/teal)
CYAN_LO = np.array([80,  100,  80])
CYAN_HI = np.array([105, 255, 255])
MIN_AREA = 800   # px² — ignore tiny blobs

app = Flask(__name__)

_oak_frame: bytes = b""
_hbv_frame: bytes = b""
_lock_oak = threading.Lock()
_lock_hbv = threading.Lock()


# ── OAK-D Lite: color + stereo depth + cyan tracking ─────────────────────────

def oak_capture_thread():
    global _oak_frame

    pipeline = dai.Pipeline()

    # Color camera (CAM_A)
    cam_color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cap_color = dai.ImgFrameCapability()
    cap_color.size.fixed((1280, 720))
    cap_color.fps.fixed(15)
    q_color = cam_color.requestOutput(cap_color, True).createOutputQueue(maxSize=2, blocking=False)

    # Stereo depth (CAM_B + CAM_C → StereoDepth aligned to CAM_A)
    cam_left  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
    stereo.setLeftRightCheck(True)
    # No depth alignment to CAM_A (causes width-multiple-of-16 error).
    # We scale coordinates from color space → depth space in software.
    cam_left.requestFullResolutionOutput().link(stereo.left)
    cam_right.requestFullResolutionOutput().link(stereo.right)
    q_depth = stereo.depth.createOutputQueue(maxSize=2, blocking=False)

    pipeline.start()
    print("OAK-D Lite stream started (color + stereo depth)")

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    while True:
        color_pkt = q_color.tryGet()
        if color_pkt is None:
            time.sleep(0.01)
            continue

        frame      = color_pkt.getCvFrame()
        depth_pkt  = q_depth.tryGet()
        depth_map  = depth_pkt.getFrame() if depth_pkt is not None else None

        fh, fw = frame.shape[:2]

        # ── cyan detection ────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, CYAN_LO, CYAN_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dist_cm = x_pos = y_pos = None

        if contours:
            best = max(contours, key=cv2.contourArea)
            if cv2.contourArea(best) >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(best)
                cx, cy = x + w // 2, y + h // 2

                # Green outline (contour + bounding box)
                cv2.drawContours(frame, [best], -1, (0, 255, 0), 3)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Crosshair at center
                cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                               cv2.MARKER_CROSS, 20, 2)

                # Position relative to frame center (right/up = positive)
                x_pos = cx - fw // 2
                y_pos = -(cy - fh // 2)

                # Distance from aligned depth map
                if depth_map is not None:
                    dh, dw = depth_map.shape[:2]
                    # Depth is aligned to color so coords map 1:1 after scaling
                    sx = dw / fw
                    sy = dh / fh
                    rx1 = max(0,  int(x       * sx))
                    ry1 = max(0,  int(y       * sy))
                    rx2 = min(dw, int((x + w) * sx))
                    ry2 = min(dh, int((y + h) * sy))
                    roi = depth_map[ry1:ry2, rx1:rx2]
                    valid = roi[(roi > 100) & (roi < 10000)]
                    if valid.size > 0:
                        dist_cm = float(np.median(valid)) / 10.0

        # ── overlay text ──────────────────────────────────────────────────────
        def put(text, row):
            pos = (14, 34 + row * 38)
            cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
            cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        put(f"Distance: {dist_cm:6.1f} cm" if dist_cm  is not None else "Distance:    --", 0)
        put(f"X pos:   {x_pos:+6d} px"     if x_pos    is not None else "X pos:       --", 1)
        put(f"Y pos:   {y_pos:+6d} px"     if y_pos    is not None else "Y pos:       --", 2)

        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with _lock_oak:
            _oak_frame = buf.tobytes()


# ── HBV: plain video ──────────────────────────────────────────────────────────

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

def mjpeg(get_frame):
    while True:
        data = get_frame()
        if data:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
        time.sleep(0.033)

def get_oak():
    with _lock_oak: return _oak_frame

def get_hbv():
    with _lock_hbv: return _hbv_frame


# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route("/stream/oak")
def stream_oak():
    return Response(mjpeg(get_oak), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/stream/hbv")
def stream_hbv():
    return Response(mjpeg(get_hbv), mimetype="multipart/x-mixed-replace; boundary=frame")

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
      <h2>OAK-D Lite — cyan tracking + depth</h2>
      <img src="/stream/oak">
    </div>
    <div class="cam">
      <h2>HBV HD Camera</h2>
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


if App:
    App.run(user_loop=loop)
else:
    loop()
