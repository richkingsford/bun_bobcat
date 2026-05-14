"""
Speed ramp test: forward slow-to-ceiling over 2 seconds, pause, then backward
slow-to-ceiling over 2 seconds.

Motor API:
  Bridge.call("drive", motor, direction, power, duration_ms)
    motor:     0=left  1=right  2=both
    direction: 1=forward  -1=backward  0=stop
    power:     0-100 (%)
    duration_ms: how long to hold that power (ms)
"""

import sys
import json
import time
from pathlib import Path

from arduino.app_utils import App, Bridge

SPEED_REFERENCE = Path(__file__).with_name("approved_drive_speeds.json")
SPEEDS = json.loads(SPEED_REFERENCE.read_text())
STEP_MS = 200
COMMAND_MS = STEP_MS + 75
PAUSE_MS = 500
POWER_LEVELS = SPEEDS["power_pct"]
PHYSICAL_FORWARD = -1
PHYSICAL_BACKWARD = 1


def run_step(row, elapsed_ms, label, physical_direction, power):
    Bridge.call("drive", 2, physical_direction, power, COMMAND_MS)
    print(f"{row:>2}  {elapsed_ms:>5}ms  {label:<8}  {power:>3}%")
    time.sleep(STEP_MS / 1000)


def loop():
    print("=== Speed Ramp Test ===")
    print("step   time     direction  power")
    print("--------------------------------")

    row = 1

    try:
        for power in POWER_LEVELS:
            run_step(row, (row - 1) * STEP_MS, "forward", PHYSICAL_FORWARD, power)
            row += 1

        Bridge.call("drive", 2, 0, 0, 0)
        print(f"    pause {PAUSE_MS}ms")
        time.sleep(PAUSE_MS / 1000)

        for power in POWER_LEVELS:
            run_step(row, (row - 1) * STEP_MS, "backward", PHYSICAL_BACKWARD, power)
            row += 1
    finally:
        Bridge.call("drive", 2, 0, 0, 0)
        print("--------------------------------")
        print("Motors stopped.")
        sys.exit(0)


App.run(user_loop=loop)
