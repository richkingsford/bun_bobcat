"""
Capture one frame from each camera on the bot:
  OAK-D Lite  — CAM_A color via depthai     (see ../camera_oak_d_lite.txt)
  HBV HD CAM  — /dev/video2 via V4L2/opencv  (see ../camera_hbv.txt)
"""

import os
import time
from datetime import datetime

import cv2
import depthai as dai

from arduino.app_utils import App

REPO_PATH = "/app/python"
HBV_DEVICE = "/dev/video2"


def timestamp_prefix() -> str:
    now = datetime.now()
    hour = str(int(now.strftime("%I")))
    minute = now.strftime("%M")
    ampm = now.strftime("%p").lower()
    month = now.strftime("%B")
    day = str(int(now.strftime("%d")))
    return f"photo - [{hour}:{minute}{ampm} on {month} {day}]"


def capture_oak() -> None:
    pipeline = dai.Pipeline()
    cap = dai.ImgFrameCapability()
    cap.size.fixed((1920, 1080))
    cap.fps.fixed(10)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    q = cam.requestOutput(cap, True).createOutputQueue(maxSize=4, blocking=False)

    pipeline.start()
    frame = None
    deadline = time.time() + 10
    while frame is None and time.time() < deadline:
        pkt = q.tryGet()
        if pkt is not None:
            frame = pkt.getCvFrame()
        else:
            time.sleep(0.05)
    pipeline.stop()

    if frame is not None:
        path = os.path.join(REPO_PATH, f"{timestamp_prefix()} - oak-color.jpg")
        cv2.imwrite(path, frame)
        print(f"Saved OAK:  {path} ({os.path.getsize(path):,} bytes)")
    else:
        print("ERROR: no frame from OAK-D Lite")


def capture_hbv() -> None:
    cap = cv2.VideoCapture(HBV_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    ret, frame = cap.read()
    cap.release()

    if ret:
        path = os.path.join(REPO_PATH, f"{timestamp_prefix()} - hbv.jpg")
        cv2.imwrite(path, frame)
        print(f"Saved HBV:  {path} ({os.path.getsize(path):,} bytes)")
    else:
        print("ERROR: no frame from HBV camera")


def loop():
    print("=== Dual Camera Capture ===")
    capture_oak()
    capture_hbv()
    print("Done.")
    while True:
        time.sleep(1)


App.run(user_loop=loop)
