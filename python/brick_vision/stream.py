#!/usr/bin/env python3
"""Bun brick-vision livestream for the USB-connected OAK-D Lite."""

import argparse
import socket
import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np
from flask import Flask, Response, jsonify


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 20.0
JPEG_QUALITY = 82
STREAM_SLEEP_S = 0.02
GREEN_LOWER_HSV = np.array([38, 65, 45])
GREEN_UPPER_HSV = np.array([95, 255, 255])
MIN_BRICK_AREA = 4000
HEX_SAMPLE_COUNT = 12
HEX_SAMPLE_PERIOD_S = 2.0
MAX_HEX_PIXELS = 6000
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 8000
DEPTH_HIST_BIN_MM = 50
AUX_FRAME_PERIOD_S = 0.5
CLOSE_SIZE_MODEL_A = 59076.899536
CLOSE_SIZE_MODEL_B = -10.031662
CLOSE_SIZE_SWITCH_MM = 450


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Bun Brick Vision</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #111417;
      color: #f2f5f7;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      background: #111417;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 12px 16px;
      border-bottom: 1px solid #29313a;
      background: #171b20;
    }
    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
    }
    .status {
      display: flex;
      align-items: center;
      gap: 8px;
      min-width: 0;
      font-size: 13px;
      color: #b8c1ca;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      background: #87909a;
      flex: 0 0 auto;
    }
    .dot.live { background: #45d483; box-shadow: 0 0 14px rgba(69, 212, 131, 0.55); }
    .dot.error { background: #ff6b6b; box-shadow: 0 0 14px rgba(255, 107, 107, 0.45); }
    main {
      flex: 1;
      display: grid;
      place-items: center;
      padding: 16px;
      overflow: hidden;
    }
    .stage {
      position: relative;
      width: min(100%, calc((100vh - 100px) * 16 / 9));
      max-height: calc(100vh - 100px);
      aspect-ratio: 16 / 9;
    }
    .frame {
      width: 100%;
      height: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #050607;
      border: 1px solid #29313a;
    }
    .hud {
      position: absolute;
      left: 12px;
      top: 12px;
      display: grid;
      grid-template-columns: repeat(3, minmax(86px, 1fr));
      gap: 8px;
      width: min(430px, calc(100% - 24px));
    }
    .metric {
      background: rgba(10, 12, 14, 0.78);
      border: 1px solid rgba(229, 237, 245, 0.25);
      padding: 8px 10px;
      min-height: 58px;
    }
    .metric span {
      display: block;
      color: #aeb8c2;
      font-size: 12px;
      line-height: 1;
      margin-bottom: 7px;
    }
    .metric strong {
      color: #f8fbff;
      font-size: 22px;
      line-height: 1;
      font-variant-numeric: tabular-nums;
      letter-spacing: 0;
    }
    .metric em {
      color: #aeb8c2;
      font-style: normal;
      font-size: 12px;
      margin-left: 4px;
    }
    footer {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      padding: 10px 16px 12px;
      color: #87909a;
      font-size: 12px;
      border-top: 1px solid #29313a;
      background: #171b20;
    }
    .swatches {
      display: flex;
      align-items: center;
      gap: 5px;
      min-height: 18px;
      flex-wrap: wrap;
    }
    .swatch {
      width: 18px;
      height: 18px;
      border: 1px solid rgba(255,255,255,0.4);
    }
    .links {
      display: flex;
      gap: 12px;
      white-space: nowrap;
    }
    .coords {
      color: #e5edf5;
      white-space: nowrap;
    }
    a { color: #cfe6ff; }
    @media (max-width: 720px) {
      header, footer { align-items: flex-start; flex-direction: column; }
      main { padding: 8px; }
      .stage {
        width: 100%;
        max-height: calc(100vh - 150px);
      }
      .hud {
        left: 8px;
        top: 8px;
        width: calc(100% - 16px);
        grid-template-columns: repeat(3, minmax(0, 1fr));
      }
      .metric { padding: 7px 8px; min-height: 52px; }
      .metric strong { font-size: 18px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Bun Brick Vision</h1>
    <div class="status"><span id="dot" class="dot"></span><span id="state">starting</span></div>
  </header>
  <main>
    <div class="stage">
      <img class="frame" src="/stream.mjpg" alt="OAK-D Lite livestream">
      <div class="hud" aria-live="polite">
        <div class="metric"><span>dist</span><strong id="distValue">--</strong><em>mm</em></div>
        <div class="metric"><span>x</span><strong id="xValue">--</strong><em>mm</em></div>
        <div class="metric"><span>y</span><strong id="yValue">--</strong><em>mm</em></div>
      </div>
    </div>
  </main>
  <footer>
    <span id="device">OAK-D Lite</span>
    <span id="coords" class="coords"></span>
    <div id="swatches" class="swatches"></div>
    <span class="links"><a href="/detect.json">detect.json</a><a href="/depth.jpg">depth</a><a href="/raw.jpg">raw</a><a href="/snapshot.jpg">snapshot</a></span>
  </footer>
  <script>
    const dot = document.getElementById("dot");
    const state = document.getElementById("state");
    const device = document.getElementById("device");
    const coords = document.getElementById("coords");
    const swatches = document.getElementById("swatches");
    const distValue = document.getElementById("distValue");
    const xValue = document.getElementById("xValue");
    const yValue = document.getElementById("yValue");

    function renderSwatches(colors) {
      swatches.innerHTML = "";
      for (const item of colors || []) {
        const el = document.createElement("span");
        el.className = "swatch";
        el.title = item.hex;
        el.style.background = item.hex;
        swatches.appendChild(el);
      }
    }

    async function refreshStatus() {
      try {
        const res = await fetch("/status", { cache: "no-store" });
        const data = await res.json();
        dot.className = "dot " + (data.running && data.frames > 0 ? "live" : data.error ? "error" : "");
        const confidence = data.detection && data.detection.found ? ` brick ${data.detection.confidence}%` : "";
        state.textContent = data.error || (data.running ? `${data.fps.toFixed(1)} fps${confidence}` : "starting");
        device.textContent = data.device || "OAK-D Lite";
        const spatial = data.detection && data.detection.spatial;
        if (spatial && spatial.valid) {
          distValue.textContent = spatial.dist_mm;
          xValue.textContent = spatial.x_mm > 0 ? `+${spatial.x_mm}` : spatial.x_mm;
          yValue.textContent = spatial.y_mm > 0 ? `+${spatial.y_mm}` : spatial.y_mm;
          coords.textContent = `dist ${spatial.dist_mm} mm  x ${spatial.x_mm} mm  y ${spatial.y_mm} mm`;
        } else {
          distValue.textContent = "--";
          xValue.textContent = "--";
          yValue.textContent = "--";
          coords.textContent = "";
        }
        renderSwatches(data.detection ? data.detection.hex_colors : []);
      } catch (err) {
        dot.className = "dot error";
        state.textContent = "offline";
      }
    }

    refreshStatus();
    setInterval(refreshStatus, 250);
  </script>
</body>
</html>
"""


def local_ip():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
        except OSError:
            return "127.0.0.1"


def socket_label(socket_value):
    return str(socket_value).split(".")[-1].replace("_", "").lower()


def rgb_to_hex(rgb):
    return f"#{int(rgb[0]):02X}{int(rgb[1]):02X}{int(rgb[2]):02X}"


def draw_label(frame, text, x, y, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    top = max(0, y - th - baseline - 6)
    left = max(0, x)
    cv2.rectangle(frame, (left, top), (left + tw + 8, top + th + baseline + 6), (10, 12, 14), -1)
    cv2.putText(frame, text, (left + 4, top + th + 2), font, scale, color, thickness, cv2.LINE_AA)


def draw_crosshair(frame, x, y, color):
    x = int(round(x))
    y = int(round(y))
    cv2.line(frame, (x - 12, y), (x + 12, y), color, 1, cv2.LINE_AA)
    cv2.line(frame, (x, y - 12), (x, y + 12), color, 1, cv2.LINE_AA)
    cv2.circle(frame, (x, y), 4, color, 1, cv2.LINE_AA)


class BrickDetector:
    def __init__(self):
        self.hex_colors = []
        self.last_hex_sample_t = 0.0

    def _sample_hex_colors(self, frame, mask, contour):
        contour_mask = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
        pixels = frame[(mask > 0) & (contour_mask > 0)]
        if len(pixels) == 0:
            return []

        if len(pixels) > MAX_HEX_PIXELS:
            step = max(1, len(pixels) // MAX_HEX_PIXELS)
            pixels = pixels[::step]

        rgb = pixels[:, ::-1].astype(np.float32)
        count = min(HEX_SAMPLE_COUNT, len(rgb))
        if count == 0:
            return []

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 1.0)
        _, labels, centers = cv2.kmeans(rgb, count, None, criteria, 2, cv2.KMEANS_PP_CENTERS)
        counts = np.bincount(labels.ravel(), minlength=count)

        colors = []
        for index in np.argsort(-counts):
            center = np.clip(np.rint(centers[index]), 0, 255).astype(int)
            colors.append({"hex": rgb_to_hex(center), "count": int(counts[index])})
        return colors

    def _find_notches(self, mask, contour):
        x, y, w, h = cv2.boundingRect(contour)
        roi_green = mask[y:y + h, x:x + w]
        silhouette = np.zeros_like(roi_green)
        cv2.drawContours(silhouette, [contour - np.array([x, y])], -1, 255, cv2.FILLED)
        silhouette = cv2.morphologyEx(silhouette, cv2.MORPH_CLOSE, np.ones((35, 35), np.uint8), iterations=1)
        holes = cv2.bitwise_and(cv2.bitwise_not(roi_green), silhouette)
        holes = cv2.morphologyEx(holes, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        contours, _ = cv2.findContours(holes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        notches = []
        for hole in contours:
            area = cv2.contourArea(hole)
            if area < 120:
                continue

            hx, hy, hw, hh = cv2.boundingRect(hole)
            if hw < 5 or hh < 5:
                continue

            perimeter = cv2.arcLength(hole, True)
            approx_triangle = cv2.approxPolyDP(hole, 0.09 * perimeter, True)
            approx_quad = cv2.approxPolyDP(hole, 0.07 * perimeter, True)
            aspect = hw / max(1, hh)
            extent = area / max(1, hw * hh)
            rel_x = (hx + hw / 2) / max(1, w)
            rel_y = (hy + hh / 2) / max(1, h)

            notch_type = "negative-space"
            central_aperture = (
                0.32 <= rel_x <= 0.76
                and 0.24 <= rel_y <= 0.88
                and 0.55 <= aspect <= 1.75
                and 0.40 <= extent <= 0.82
            )
            is_triangle = central_aperture and (
                len(approx_triangle) == 3
                or 3 <= len(approx_quad) <= 5
            )
            is_square = (
                4 <= len(approx_quad) <= 6
                and (rel_x <= 0.25 or rel_x >= 0.75)
                and 0.35 <= aspect <= 1.6
                and extent >= 0.30
            )

            if is_triangle:
                notch_type = "triangle-notch"
            elif is_square:
                notch_type = "square-notch"

            notches.append({
                "type": notch_type,
                "bbox": {"x": int(x + hx), "y": int(y + hy), "w": int(hw), "h": int(hh)},
                "area": round(float(area), 1),
                "vertices_triangle": int(len(approx_triangle)),
                "vertices_quad": int(len(approx_quad)),
                "aspect": round(float(aspect), 2),
                "extent": round(float(extent), 2),
                "rel_x": round(float(rel_x), 2),
                "rel_y": round(float(rel_y), 2),
            })

        notches.sort(key=lambda item: item["area"], reverse=True)
        return notches[:8]

    def _choose_brick_contour(self, contours):
        best = None
        best_score = -1.0
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if w < 50 or h < 50:
                continue

            aspect = w / max(1, h)
            extent = area / max(1, w * h)
            area_score = min(1.0, area / 22000.0)
            size_score = min(1.0, min(w, h) / 160.0)
            brick_shape = 0.55 <= aspect <= 1.55 and 0.55 <= extent <= 0.9
            score = area_score + size_score
            if brick_shape:
                score += 2.0
            if aspect > 1.65 or extent < 0.5:
                score -= 1.0

            if score > best_score:
                best_score = score
                best = contour
        return best

    def _estimate_spatial(self, mask, contour, depth, intrinsics):
        if intrinsics is None:
            return {
                "valid": False,
                "reason": "camera intrinsics unavailable",
                "axis_convention": "x right, y down, dist forward from RGB optical center",
            }

        x, y, w, h = cv2.boundingRect(contour)
        center_u = x + w / 2.0
        center_v = y + h / 2.0
        fx = float(intrinsics[0][0])
        fy = float(intrinsics[1][1])
        cx = float(intrinsics[0][2])
        cy = float(intrinsics[1][2])
        apparent_size_px = (w + h) / 2.0
        size_dist_mm = CLOSE_SIZE_MODEL_A / max(1.0, apparent_size_px) + CLOSE_SIZE_MODEL_B

        def size_only(reason, valid_px=0):
            x_mm = (center_u - cx) * size_dist_mm / fx
            y_mm = (center_v - cy) * size_dist_mm / fy
            radial_mm = float((x_mm * x_mm + y_mm * y_mm + size_dist_mm * size_dist_mm) ** 0.5)
            return {
                "valid": True,
                "reason": reason,
                "dist_mm": int(round(size_dist_mm)),
                "dist_source": "size",
                "stereo_dist_mm": None,
                "size_dist_mm": int(round(size_dist_mm)),
                "apparent_size_px": round(float(apparent_size_px), 1),
                "x_mm": int(round(x_mm)),
                "y_mm": int(round(y_mm)),
                "radial_mm": int(round(radial_mm)),
                "center_px": {"u": round(float(center_u), 1), "v": round(float(center_v), 1)},
                "camera_center_px": {"u": round(cx, 1), "v": round(cy, 1)},
                "valid_depth_px": int(valid_px),
                "used_depth_px": 0,
                "depth_iqr_mm": None,
                "depth_method": "size-only",
                "size_model": {
                    "fit": "dist_mm = a / apparent_size_px + b",
                    "a": CLOSE_SIZE_MODEL_A,
                    "b": CLOSE_SIZE_MODEL_B,
                    "switch_mm": CLOSE_SIZE_SWITCH_MM,
                },
                "axis_convention": "x right, y down, dist forward from RGB optical center",
            }

        if depth is None:
            return size_only("depth unavailable")

        contour_mask = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
        spatial_mask = cv2.bitwise_and(mask, contour_mask)
        spatial_mask = cv2.erode(spatial_mask, np.ones((5, 5), np.uint8), iterations=1)
        if cv2.countNonZero(spatial_mask) < 50:
            spatial_mask = cv2.bitwise_and(mask, contour_mask)

        depth_values = depth[spatial_mask > 0]
        depth_values = depth_values[
            (depth_values >= DEPTH_MIN_MM)
            & (depth_values <= DEPTH_MAX_MM)
        ]
        valid_px = int(depth_values.size)
        if valid_px < 30:
            return size_only("too few valid stereo depth pixels", valid_px)

        p05, p95 = np.percentile(depth_values, [5, 95])
        trimmed = depth_values[(depth_values >= p05) & (depth_values <= p95)]
        if trimmed.size >= 30:
            depth_values = trimmed

        depth_method = "trimmed-median"
        if depth_values.size >= 60:
            low = int(np.floor(float(depth_values.min()) / DEPTH_HIST_BIN_MM) * DEPTH_HIST_BIN_MM)
            high = int(np.ceil(float(depth_values.max()) / DEPTH_HIST_BIN_MM) * DEPTH_HIST_BIN_MM)
            if high > low:
                bins = np.arange(low, high + DEPTH_HIST_BIN_MM, DEPTH_HIST_BIN_MM)
                hist, edges = np.histogram(depth_values, bins=bins)
                if hist.size:
                    best_bin = int(np.argmax(hist))
                    bin_low = edges[best_bin]
                    bin_high = edges[best_bin + 1]
                    cluster = depth_values[(depth_values >= bin_low) & (depth_values < bin_high)]
                    if cluster.size >= 30:
                        depth_values = cluster
                        depth_method = "dominant-plane-median"

        stereo_dist_mm = float(np.median(depth_values))
        q25, q75 = np.percentile(depth_values, [25, 75])
        use_size_dist = size_dist_mm <= CLOSE_SIZE_SWITCH_MM
        dist_mm = size_dist_mm if use_size_dist else stereo_dist_mm
        x_mm = (center_u - cx) * dist_mm / fx
        y_mm = (center_v - cy) * dist_mm / fy
        radial_mm = float((x_mm * x_mm + y_mm * y_mm + dist_mm * dist_mm) ** 0.5)

        return {
            "valid": True,
            "dist_mm": int(round(dist_mm)),
            "dist_source": "size" if use_size_dist else "stereo",
            "stereo_dist_mm": int(round(stereo_dist_mm)),
            "size_dist_mm": int(round(size_dist_mm)),
            "apparent_size_px": round(float(apparent_size_px), 1),
            "x_mm": int(round(x_mm)),
            "y_mm": int(round(y_mm)),
            "radial_mm": int(round(radial_mm)),
            "center_px": {"u": round(float(center_u), 1), "v": round(float(center_v), 1)},
            "camera_center_px": {"u": round(cx, 1), "v": round(cy, 1)},
            "valid_depth_px": valid_px,
            "used_depth_px": int(depth_values.size),
            "depth_iqr_mm": int(round(float(q75 - q25))),
            "depth_method": depth_method,
            "size_model": {
                "fit": "dist_mm = a / apparent_size_px + b",
                "a": CLOSE_SIZE_MODEL_A,
                "b": CLOSE_SIZE_MODEL_B,
                "switch_mm": CLOSE_SIZE_SWITCH_MM,
            },
            "axis_convention": "x right, y down, dist forward from RGB optical center",
        }

    def detect(self, frame, depth=None, intrinsics=None):
        overlay = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_LOWER_HSV, GREEN_UPPER_HSV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if cv2.contourArea(contour) >= MIN_BRICK_AREA]
        if not contours:
            draw_label(overlay, "brick: searching", 16, 32, (180, 190, 200))
            return overlay, {
                "found": False,
                "confidence": 0,
                "hex_colors": self.hex_colors,
                "signals": {
                    "green_area": 0,
                    "brick_shape": False,
                    "triangle_notch": False,
                    "square_notch": False,
                },
                "notches": [],
            }

        contour = self._choose_brick_contour(contours)
        if contour is None:
            draw_label(overlay, "brick: searching", 16, 32, (180, 190, 200))
            return overlay, {
                "found": False,
                "confidence": 0,
                "hex_colors": self.hex_colors,
                "signals": {
                    "green_area": 0,
                    "brick_shape": False,
                    "triangle_notch": False,
                    "square_notch": False,
                },
                "notches": [],
            }

        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        extent = area / max(1, w * h)
        aspect = w / max(1, h)
        notches = self._find_notches(mask, contour)
        triangle_found = any(notch["type"] == "triangle-notch" for notch in notches)
        square_found = any(notch["type"] == "square-notch" for notch in notches)
        brick_shape = 0.45 <= extent <= 0.9 and 0.55 <= aspect <= 1.55 and w >= 80 and h >= 80
        spatial = self._estimate_spatial(mask, contour, depth, intrinsics)

        now = time.monotonic()
        if now - self.last_hex_sample_t >= HEX_SAMPLE_PERIOD_S or not self.hex_colors:
            self.hex_colors = self._sample_hex_colors(frame, mask, contour)
            self.last_hex_sample_t = now

        color_score = min(1.0, area / 22000.0)
        shape_score = 1.0 if brick_shape else 0.35
        triangle_score = 1.0 if triangle_found else 0.0
        square_score = 1.0 if square_found else 0.0
        confidence = int(round(100.0 * (
            0.34 * color_score
            + 0.22 * shape_score
            + 0.24 * triangle_score
            + 0.20 * square_score
        )))

        cv2.drawContours(overlay, [contour], -1, (45, 230, 120), 2)
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (45, 230, 120), 2)
        draw_label(overlay, f"brick {confidence}%", x, max(24, y - 8), (45, 230, 120))
        if intrinsics is not None:
            draw_crosshair(overlay, intrinsics[0][2], intrinsics[1][2], (255, 255, 255))
        if spatial["valid"]:
            draw_crosshair(overlay, spatial["center_px"]["u"], spatial["center_px"]["v"], (0, 255, 255))
            coord_label = (
                f"dist {spatial['dist_mm']}mm ({spatial['dist_source']})  "
                f"x {spatial['x_mm']:+d}mm  "
                f"y {spatial['y_mm']:+d}mm"
            )
            draw_label(overlay, coord_label, x, y + h + 24, (0, 255, 255))
        else:
            draw_label(overlay, "depth: waiting", x, y + h + 24, (0, 255, 255))

        for notch in notches:
            bbox = notch["bbox"]
            if notch["type"] == "triangle-notch":
                color = (0, 220, 255)
                label = "triangle"
            elif notch["type"] == "square-notch":
                color = (255, 210, 70)
                label = "square"
            else:
                color = (175, 175, 175)
                label = "space"
            cv2.rectangle(
                overlay,
                (bbox["x"], bbox["y"]),
                (bbox["x"] + bbox["w"], bbox["y"] + bbox["h"]),
                color,
                2,
            )
            if notch["type"] != "negative-space":
                draw_label(overlay, label, bbox["x"], bbox["y"], color)

        return overlay, {
            "found": True,
            "confidence": confidence,
            "bbox": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
            "area": round(float(area), 1),
            "aspect": round(float(aspect), 2),
            "extent": round(float(extent), 2),
            "hex_colors": self.hex_colors,
            "signals": {
                "green_area": round(float(area), 1),
                "brick_shape": bool(brick_shape),
                "triangle_notch": bool(triangle_found),
                "square_notch": bool(square_found),
            },
            "spatial": spatial,
            "notches": notches,
        }


class OakStreamer:
    def __init__(self, width, height, fps):
        self.width = width
        self.height = height
        self.fps = fps
        self.lock = threading.Condition()
        self.latest_jpeg = None
        self.latest_raw_jpeg = None
        self.latest_depth_jpeg = None
        self.latest_detection = {"found": False, "confidence": 0, "hex_colors": []}
        self.error = None
        self.running = False
        self.frames = 0
        self.measured_fps = 0.0
        self.device_label = None
        self.depth_label = None
        self.detector = BrickDetector()
        self.thread = threading.Thread(target=self._run, name="oak-streamer", daemon=True)

    def start(self):
        self.thread.start()

    def _set_error(self, error):
        with self.lock:
            self.error = str(error)
            self.running = False
            self.lock.notify_all()

    def _choose_color_socket(self, device):
        features = device.getConnectedCameraFeatures()
        for feature in features:
            if dai.CameraSensorType.COLOR in feature.supportedTypes:
                return feature.socket, feature
        raise RuntimeError("no color camera found on the connected DepthAI device")

    def _choose_stereo_sockets(self, device):
        try:
            calibration = device.readCalibration2()
            return (
                calibration,
                calibration.getStereoLeftCameraId(),
                calibration.getStereoRightCameraId(),
            )
        except Exception:
            mono_sockets = [
                feature.socket
                for feature in device.getConnectedCameraFeatures()
                if dai.CameraSensorType.MONO in feature.supportedTypes
            ]
            if len(mono_sockets) < 2:
                raise RuntimeError("no calibrated stereo pair found")
            return device.readCalibration2(), mono_sockets[0], mono_sockets[1]

    def _render_depth(self, depth):
        valid = depth[(depth >= DEPTH_MIN_MM) & (depth <= DEPTH_MAX_MM)]
        if valid.size == 0:
            return np.zeros((depth.shape[0], depth.shape[1], 3), np.uint8)
        low, high = np.percentile(valid, [5, 95])
        if high <= low:
            high = low + 1
        normalized = np.clip((depth.astype(np.float32) - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
        normalized[depth == 0] = 0
        return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

    def _run(self):
        try:
            with dai.Pipeline(dai.Device()) as pipeline:
                device = pipeline.getDefaultDevice()
                socket_value, feature = self._choose_color_socket(device)
                calibration, left_socket, right_socket = self._choose_stereo_sockets(device)
                intrinsics = calibration.getCameraIntrinsics(socket_value, self.width, self.height)
                self.device_label = (
                    f"{device.getProductName()} {device.getDeviceId()} "
                    f"{socket_value.name} {feature.sensorName} {feature.width}x{feature.height}"
                )
                self.depth_label = f"depth aligned to {socket_value.name} from {left_socket.name}/{right_socket.name}"

                camera = pipeline.create(dai.node.Camera).build(socket_value)
                output = camera.requestOutput((self.width, self.height), fps=self.fps)
                queue = output.createOutputQueue(maxSize=4, blocking=False)

                left = pipeline.create(dai.node.Camera).build(left_socket)
                right = pipeline.create(dai.node.Camera).build(right_socket)
                stereo = pipeline.create(dai.node.StereoDepth)
                stereo.setLeftRightCheck(True)
                stereo.setExtendedDisparity(True)
                stereo.setDepthAlign(socket_value)
                stereo.setOutputSize(self.width, self.height)
                left.requestFullResolutionOutput().link(stereo.left)
                right.requestFullResolutionOutput().link(stereo.right)
                depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

                pipeline.start()
                frame_timestamps = []
                last_aux_frame_t = 0.0

                with self.lock:
                    self.running = True
                    self.error = None
                    self.lock.notify_all()

                while True:
                    packet = queue.get()
                    depth_packet = depth_queue.get()
                    frame = packet.getCvFrame()
                    depth = depth_packet.getFrame()
                    overlay, detection = self.detector.detect(frame, depth, intrinsics)
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        overlay,
                        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
                    )
                    if not ok:
                        continue

                    now = time.monotonic()
                    raw_bytes = None
                    depth_bytes = None
                    if now - last_aux_frame_t >= AUX_FRAME_PERIOD_S:
                        raw_ok, raw_encoded = cv2.imencode(
                            ".jpg",
                            frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
                        )
                        depth_vis = self._render_depth(depth)
                        depth_ok, depth_encoded = cv2.imencode(
                            ".jpg",
                            depth_vis,
                            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
                        )
                        if raw_ok and depth_ok:
                            raw_bytes = raw_encoded.tobytes()
                            depth_bytes = depth_encoded.tobytes()
                            last_aux_frame_t = now

                    frame_timestamps.append(now)
                    while frame_timestamps and now - frame_timestamps[0] > 2.0:
                        frame_timestamps.pop(0)
                    if len(frame_timestamps) > 1:
                        elapsed = frame_timestamps[-1] - frame_timestamps[0]
                        if elapsed > 0:
                            self.measured_fps = (len(frame_timestamps) - 1) / elapsed

                    with self.lock:
                        self.latest_jpeg = encoded.tobytes()
                        if raw_bytes is not None:
                            self.latest_raw_jpeg = raw_bytes
                        if depth_bytes is not None:
                            self.latest_depth_jpeg = depth_bytes
                        self.latest_detection = detection
                        self.frames += 1
                        self.running = True
                        self.error = None
                        self.lock.notify_all()
        except Exception as exc:
            self._set_error(exc)

    def wait_for_frame(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        with self.lock:
            while self.latest_jpeg is None and self.error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.lock.wait(remaining)
            return self.latest_jpeg is not None

    def get_frame(self):
        with self.lock:
            return self.latest_jpeg

    def status(self):
        with self.lock:
            return {
                "running": self.running,
                "frames": self.frames,
                "fps": self.measured_fps,
                "device": self.device_label,
                "depth": self.depth_label,
                "error": self.error,
                "detection": self.latest_detection,
            }

    def get_raw_frame(self):
        with self.lock:
            return self.latest_raw_jpeg

    def get_depth_frame(self):
        with self.lock:
            return self.latest_depth_jpeg

    def get_detection(self):
        with self.lock:
            return self.latest_detection

    def stream(self):
        last_frame = None
        while True:
            with self.lock:
                self.lock.wait_for(
                    lambda: self.latest_jpeg is not None and self.latest_jpeg is not last_frame,
                    timeout=1.0,
                )
                frame = self.latest_jpeg
            if frame is None:
                time.sleep(STREAM_SLEEP_S)
                continue
            last_frame = frame
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )


def create_app(streamer):
    app = Flask(__name__)

    @app.get("/")
    def index():
        return Response(HTML, mimetype="text/html")

    @app.get("/stream.mjpg")
    def stream():
        return Response(
            streamer.stream(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/snapshot.jpg")
    def snapshot():
        frame = streamer.get_frame()
        if frame is None:
            return Response("no frame available yet\n", status=503, mimetype="text/plain")
        return Response(frame, mimetype="image/jpeg")

    @app.get("/detect.jpg")
    def detect_image():
        frame = streamer.get_frame()
        if frame is None:
            return Response("no frame available yet\n", status=503, mimetype="text/plain")
        return Response(frame, mimetype="image/jpeg")

    @app.get("/raw.jpg")
    def raw_image():
        frame = streamer.get_raw_frame()
        if frame is None:
            return Response("no raw frame available yet\n", status=503, mimetype="text/plain")
        return Response(frame, mimetype="image/jpeg")

    @app.get("/depth.jpg")
    def depth_image():
        frame = streamer.get_depth_frame()
        if frame is None:
            return Response("no depth frame available yet\n", status=503, mimetype="text/plain")
        return Response(frame, mimetype="image/jpeg")

    @app.get("/detect.json")
    def detect_json():
        return jsonify(streamer.get_detection())

    @app.get("/status")
    def status():
        return jsonify(streamer.status())

    return app


def main():
    parser = argparse.ArgumentParser(description="Serve Bun's OAK-D Lite camera as a browser livestream.")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"bind host (default {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"bind port (default {DEFAULT_PORT})")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help=f"stream width (default {DEFAULT_WIDTH})")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help=f"stream height (default {DEFAULT_HEIGHT})")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help=f"camera FPS (default {DEFAULT_FPS:g})")
    args = parser.parse_args()

    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("[vision] no DepthAI/OAK device found on USB", file=sys.stderr)
        return 1

    print("[vision] available DepthAI devices:")
    for device in devices:
        print(f"[vision] - name={device.name} state={device.state} protocol={device.protocol}")

    streamer = OakStreamer(args.width, args.height, args.fps)
    streamer.start()
    if streamer.wait_for_frame(timeout=15.0):
        print(f"[vision] first frame captured from {streamer.status()['device']}")
    else:
        status = streamer.status()
        if status["error"]:
            print(f"[vision] camera startup failed: {status['error']}", file=sys.stderr)
            return 2
        print("[vision] serving while waiting for first frame")

    ip = local_ip()
    print(f"[vision] local URL:   http://127.0.0.1:{args.port}/")
    print(f"[vision] network URL: http://{ip}:{args.port}/")

    app = create_app(streamer)
    app.run(host=args.host, port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
