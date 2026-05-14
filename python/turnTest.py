"""
Approved-speed turn test.

The only wheel power values sent are 0 and the values in
approved_drive_speeds.json.
"""

import json
import signal
import sys
import time
from pathlib import Path

from arduino.app_utils import App, Bridge

SPEED_REFERENCE = Path(__file__).with_name("approved_drive_speeds.json")
SPEED_DATA = json.loads(SPEED_REFERENCE.read_text())
DRIVE_CONTRACT = SPEED_DATA["drive_contract"]
TIMING_MS = SPEED_DATA["timing_ms"]
TURN_PROFILE = SPEED_DATA["gradual_drive_turn"]["current_tested_profile"]

APPROVED_SPEEDS = SPEED_DATA["power_pct"]
APPROVED_SPEED_SET = set(APPROVED_SPEEDS)
AUTHORIZED_FLOOR_SPEED = DRIVE_CONTRACT["nonzero_power_floor_pct"]
TURN_CEILING_SPEED = DRIVE_CONTRACT["nonzero_power_ceiling_pct"]

STEP_MS = TURN_PROFILE["step_ms"]
ONE_WAY_MS = TURN_PROFILE["duration_ms"]
STEPS = ONE_WAY_MS // STEP_MS
COMMAND_MS = TIMING_MS["command"]
PAUSE_MS = TIMING_MS["pause_between_actions"]
PINNED_TREAD_STEPS = TURN_PROFILE["pin_slow_tread_last_steps"]
SLOW_TREAD_HOLD_POWER = TURN_PROFILE["slow_tread_hold_pct"]
SLOW_TREAD_STOP_POWER = TURN_PROFILE["slow_tread_stop_pct"]
FAST_TREAD_RAMP = TURN_PROFILE["fast_tread_ramp_pct"]

MOTOR_CODES = DRIVE_CONTRACT["motor_codes"]
DIRECTION_CODES = DRIVE_CONTRACT["direction_codes"]
LEFT_MOTOR = MOTOR_CODES["left"]
RIGHT_MOTOR = MOTOR_CODES["right"]
BOTH_MOTORS = MOTOR_CODES["both"]

signal.signal(signal.SIGINT, lambda *_: sys.exit(0))


def validate_profile():
    if STEP_MS <= 0 or ONE_WAY_MS % STEP_MS != 0:
        raise ValueError("Turn duration must divide evenly into step_ms")

    if len(FAST_TREAD_RAMP) != STEPS:
        raise ValueError("fast_tread_ramp_pct length must match duration_ms / step_ms")

    if AUTHORIZED_FLOOR_SPEED != APPROVED_SPEEDS[0]:
        raise ValueError("First approved speed must match nonzero_power_floor_pct")

    if TURN_CEILING_SPEED != APPROVED_SPEEDS[-1]:
        raise ValueError("Last approved speed must match nonzero_power_ceiling_pct")

    for power in [SLOW_TREAD_HOLD_POWER, *FAST_TREAD_RAMP]:
        validate_power(power)


def validate_power(power):
    if power != 0 and (
        power not in APPROVED_SPEED_SET or power < AUTHORIZED_FLOOR_SPEED
    ):
        raise ValueError(f"Unauthorized wheel power: {power}%")


def drive(motor, direction, power):
    validate_power(power)
    if power == 0:
        direction = 0
    Bridge.call("drive", motor, direction, power, COMMAND_MS)


def stop_all():
    Bridge.call("drive", BOTH_MOTORS, 0, 0, 0)


def drive_pair(direction, left_power, right_power):
    if left_power == right_power:
        drive(BOTH_MOTORS, direction, left_power)
        return

    drive(LEFT_MOTOR, direction, left_power)
    drive(RIGHT_MOTOR, direction, right_power)


def direction_code(direction_name):
    return DIRECTION_CODES[direction_name]


def run_step(row, label, action, fast_power, pin_slow_tread):
    direction = direction_code(action["direction"])
    slow_tread = action["slow_tread"]
    fast_tread = action["fast_tread"]
    slow_power = SLOW_TREAD_STOP_POWER if pin_slow_tread else SLOW_TREAD_HOLD_POWER

    tread_power = {
        slow_tread: slow_power,
        fast_tread: fast_power,
    }
    left_power = tread_power["left"]
    right_power = tread_power["right"]

    drive_pair(direction, left_power, right_power)
    print(
        f"{row:>2}  {label:<14} dir={direction:>2} "
        f"left={left_power:>2}% right={right_power:>2}%"
    )
    time.sleep(STEP_MS / 1000)


def loop():
    validate_profile()

    print("=== Turn Test ===")
    print("Approved speeds:", APPROVED_SPEEDS)
    print("Floor speed:    ", AUTHORIZED_FLOOR_SPEED)
    print("Ceiling speed:  ", TURN_CEILING_SPEED)
    print("Turn schedule:   ", FAST_TREAD_RAMP)
    print(f"Final pin:       slow tread at 0% for {PINNED_TREAD_STEPS} steps")
    print("--------------------------------")

    row = 1

    try:
        actions = list(TURN_PROFILE["actions"].items())
        for action_index, (label, action) in enumerate(actions):
            for index, power in enumerate(FAST_TREAD_RAMP):
                pin_slow_tread = index >= len(FAST_TREAD_RAMP) - PINNED_TREAD_STEPS
                run_step(row, label.replace("_", "-"), action, power, pin_slow_tread)
                row += 1

            if action_index < len(actions) - 1:
                stop_all()
                print(f"    pause {PAUSE_MS}ms")
                time.sleep(PAUSE_MS / 1000)
    finally:
        stop_all()
        print("--------------------------------")
        print("Motors stopped.")
        sys.exit(0)


if __name__ == "__main__":
    App.run(user_loop=loop)
