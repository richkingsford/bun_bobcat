"""
host_controller.py — Bun host-side PD controller (live brick-vision integration).

Closes a 20 Hz PD loop around live OAK-D Lite millimeter telemetry served by
python/brick_vision/stream.py at http://127.0.0.1:8080/status, and sends the
resulting wheel commands to the Uno Q through Arduino RouterBridge RPC by
default.
L and R are signed integers in [-100, 100]; sign maps to direction on the
sketch, magnitude maps to PWM duty. The mast servo command is kept at 0 by
this controller.

If the brick is lost — no detection, confidence below threshold, invalid
spatial fix, or the HTTP poll fails — the controller transmits a coast
command and clears the PD's derivative memory so the next valid
reading doesn't fire a phantom d/dt spike on reacquisition. The shipped
sketch latches the last command (it has no watchdog), so this loop is
the only line of defence and must never go quiet with tracks moving.

PD math is bit-identical to pd_simulator.pd_step; gains are analytically
tuned for ζ ≈ 1.5 (slightly overdamped) on both distance and heading loops
(see pd_simulator.py header for derivation).

Usage:
    python3 host_controller.py                        # RouterBridge RPC
    python3 host_controller.py --transport serial --port /dev/ttyACM1
    python3 host_controller.py --vision-url http://10.0.0.1:8080/status
    python3 host_controller.py --transport rpc        # explicit RouterBridge
    python3 host_controller.py --dry-run              # no hardware, no motion
    python3 host_controller.py --no-vision            # smoke test, will coast
"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
from urllib.error import URLError
from urllib.request import urlopen

from vision_autostart import ensure_stream


# ---------------------------------------------------------------------------
# Tunables — kept in lock-step with pd_simulator.py
# ---------------------------------------------------------------------------
WHEEL_BASE_MM        = 90.0
MAX_WHEEL_SPEED_MMPS = 250.0

# Step 1 currently stops 170 mm from the brick stack with x_off near zero.
STOP_OFFSET_MM = 170.0

KP_D = 5.0
KD_D = 0.9
KP_H = 1200.0
KD_H = 35.0
HEADING_PRIORITY_X_MM = 25.0
HEADING_FULL_TURN_X_MM = 90.0
HEADING_MAX_PRIORITY = 0.45

CTRL_HZ = 20

# Transports
DEFAULT_ROUTER_SOCKET = "/var/run/arduino-router.sock"
RPC_TIMEOUT_S         = 2.0
DEFAULT_PORT          = "/dev/ttyHS1"   # Uno Q hardware UART (serial path only)
SERIAL_BAUD           = 115200
SERIAL_TIMEOUT_S      = 0.05
RECONNECT_WAIT_S      = 1.0
ARDUINO_RESET_WAIT_S  = 2.0   # USB-CDC reset window after open

# Live vision
DEFAULT_VISION_URL    = "http://127.0.0.1:8080/status"
VISION_TIMEOUT_S      = 0.040   # < one 50 ms control period; a slow vision
                                # response must not stall the PD tick.
VISION_MIN_CONFIDENCE = 40      # close-range centered locks often score low-40s

# Telemetry print rate
PRINT_EVERY_N_TICKS = 5         # 20 Hz / 5 = 4 Hz log

# Crawl output policy. Bun's validated low-speed straight command is 13% PWM.
# One-wheel turn frames use 23% PWM because single-tread breakaway is higher.
CRAWL_PWM = 13
CRAWL_TURN_PWM = 23
CRAWL_FRAME_S = 0.150

# ---------------------------------------------------------------------------
# Step policy (default) -- burst-and-verify alignment.
#
# Why this exists: the June 10 live run ping-ponged on x (+67 mm -> -149 mm
# -> +108 mm) until the brick left the frame. Two plant facts make blended
# continuous steering unworkable at crawl speed:
#   1. Single-tread motion is stiction-dominated. The firmware boost fires
#      only on a start or direction change, so a *sustained* one-wheel
#      command stalls and snaps (the log shows ~1.2 s of commanded pivot
#      with x pinned at +67 mm, then a 66 mm jump in one frame).
#   2. The camera reports where the brick WAS, not where it is. Re-deciding
#      every 150 ms frame against a ~20 fps feed issues several turn frames
#      per piece of evidence, so every zero-crossing overshoots.
# The step policy therefore moves in short bursts that always start from
# rest (every burst gets the firmware kick -> repeatable bite), then stops
# and refuses to act again until at least one camera frame captured AFTER
# the burst has arrived. Decisions use a median over the last 3 fresh
# frames, so single-frame contour glitches (d jumped 177 -> 371 mm in the
# log) can never steer the robot. Every emitted command stays inside the
# validated vocabulary: 0, +/-STEP_SLOW_PWM straight, +/-STEP_FAST_PWM
# straight (below the 25% both-tread case motor_diagnostic Test 1 ran
# cleanly on June 8), and the existing one-wheel CRAWL_TURN_PWM frames.
# ---------------------------------------------------------------------------
STEP_SLOW_PWM         = CRAWL_PWM   # validated 13% two-tread crawl
STEP_FAST_PWM         = 20          # far-field straight only; < tested 25%
STEP_FAST_DIST_MM     = 250.0       # use FAST straight legs only beyond this
STEP_X_HOLD_MM        = 5.0         # matches main.py X_TOL_MM success gate
STEP_X_TRIM_EXIT_MM   = 7.0         # near aim exit, above the median-3
                                    #   noise floor (sigma ~2.3 mm)
STEP_X_REAIM_NEAR_MM  = 15.0        # near-field re-aim threshold
STEP_X_FAR_ENTER_FRAC = 0.18        # far aim threshold ~= 10 deg bearing
STEP_X_FAR_ENTER_MIN  = 30.0
STEP_X_FAR_EXIT_FRAC  = 0.07        # far aim exit ~= 4 deg bearing
STEP_X_FAR_EXIT_MIN   = 12.0
STEP_RING_IN_MM       = 12.0        # hold band: stop-12 .. stop+18 mm, inside
STEP_RING_OUT_MM      = 18.0        #   main.py's +/-20; deeper -> back out
STEP_AIM_BURST_MAX_S  = 0.15        # one turn burst per decision, never more
STEP_AIM_BURST_MIN_S  = 0.05        # one 20 Hz control tick
STEP_BURST_S_PER_RAD  = 0.90        # burst length per radian of needed yaw
                                    #   (~1/omega of a one-wheel kick burst)
STEP_SETTLE_S         = 0.35        # post-burst: tracks stop, image sharpens
STEP_LEG_FAR_S        = 1.20        # straight legs between re-aims
STEP_LEG_NEAR_S       = 0.45
STEP_LEG_CLOSE_S      = 0.30        # last ~60 mm before the ring
STEP_LEG_SETTLE_S     = 0.25
STEP_EST_FAST_MMPS    = 95.0        # conservative-high speed estimates used
STEP_EST_SLOW_MMPS    = 60.0        #   only to cap leg length near the ring
STEP_STALE_STOP_S     = 0.60        # moving with no fresh frame -> stop
STEP_LOST_GRACE_S     = 0.40        # mid-leg confidence flicker tolerated
STEP_SEEK_DELAY_S     = 2.0         # lost this long -> bounded reacquire scan
STEP_SEEK_BURSTS      = 10          # max one-frame pivots toward last-seen x
STEP_SEEK_LOOK_S      = 0.70
STEP_MEDIAN_N         = 3
STEP_JUMP_X_MM        = 60.0        # a parked robot rejects one frame whose
STEP_JUMP_D_MM        = 90.0        #   x/d jumps this far (contour glitch)
STEP_RING_WIN         = 6           # stationary samples judging the +/-5 mm
STEP_RING_MIN_N       = 4           #   question (sigma ~3 mm per sample)
STEP_X_REARM_MM       = 8.0         # parked: re-trim only past this margin
STEP_HOLD_RECHECK_S   = 1.0         # parked: re-judge at most this often

# Hard ceiling on the final wheel command, in percent of full duty, enforced
# at the wire for EVERY command policy. Why: the raw PD path saturates both
# wheels at 100% for any distance error beyond ~50 mm (KP_D = 5.0 against
# the 250 mm/s per-wheel cap), which is the full-speed launch that crashed
# Bun into the brick stack. The crawl policy tops out at CRAWL_TURN_PWM, so
# this cap is a no-op there; keep MAX_SPEED_LIMIT above CRAWL_TURN_PWM.
MAX_SPEED_LIMIT = 35


# ---------------------------------------------------------------------------
# Motor wiring calibration (host-side; no firmware change required)
#
# Live testing on Bun, corroborated by the May 28 step-1 alignment trials
# (motion_calibration.json: the "dist_toward" act made dist_mm INCREASE and
# "dist_away" made it DECREASE), shows the drive is inverted: a forward
# command drives the robot backward, and heading correction runs away into
# a ~90 deg spin. Reversed polarity on BOTH channels flips the sense of
# omega too, which is what turns the heading loop into positive feedback.
#
# These toggles rewrite the final l_pct / r_pct just before they are handed
# to the transport (RouterBridge drive_triple or serial), so the correction
# lives entirely on the host - no reflash. Set them per physical unit:
#   INVERT_LEFT_MOTOR / INVERT_RIGHT_MOTOR  negate that channel (reversed leads)
#   SWAP_LEFT_RIGHT_MOTORS                  exchange channels (sides swapped)
# For Bun's observed symptom set BOTH inversion flags to True (leave swap
# False). Defaults are False so a fresh checkout assumes correct wiring.
# ---------------------------------------------------------------------------
INVERT_LEFT_MOTOR      = True
INVERT_RIGHT_MOTOR     = True
SWAP_LEFT_RIGHT_MOTORS = True


# ---------------------------------------------------------------------------
# PD controller — math identical to pd_simulator.pd_step
# ---------------------------------------------------------------------------
class PdController:
    def __init__(self, stop_offset_mm=STOP_OFFSET_MM):
        self.stop_offset_mm = float(stop_offset_mm)
        self.last_dist_err = 0.0
        self.last_head_err = 0.0
        self.initialized   = False

    def reset(self):
        """Clear derivative memory. Call after a vision dropout so the next
        valid reading isn't seen as an enormous d/dt step (which would punch
        a momentum spike into the wheels)."""
        self.initialized = False

    @staticmethod
    def _wrap_pi(a):
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def step(self, x_off, y_off, distance, dt):
        """Run one PD tick. Returns (L_pct, R_pct) ints in [-100, 100]."""
        head_err = 0.0 if (x_off == 0.0 and y_off == 0.0) \
            else math.atan2(-x_off, y_off)
        dist_err = distance - self.stop_offset_mm

        if not self.initialized:
            self.last_dist_err = dist_err
            self.last_head_err = head_err
            self.initialized   = True

        d_dist = (dist_err - self.last_dist_err) / dt
        d_head = self._wrap_pi(head_err - self.last_head_err) / dt
        self.last_dist_err = dist_err
        self.last_head_err = head_err

        # Astolfi: forward velocity is throttled by cos(head_err). When the
        # robot is broadside to the brick this collapses to zero, so heading
        # torque dominates and the bot pivots into alignment before re-
        # accelerating — no fishtailing.
        v     = (KP_D * dist_err + KD_D * d_dist) * math.cos(head_err)
        omega = KP_H * head_err + KD_H * d_head

        # At Bun's validated crawl speed, large lateral errors need heading
        # authority first. This throttle fades out straight closure as x grows
        # so the crawl policy emits more one-wheel zero frames instead of
        # marching forward while the brick remains off-center.
        if abs(x_off) > HEADING_PRIORITY_X_MM:
            span = max(1.0, HEADING_FULL_TURN_X_MM - HEADING_PRIORITY_X_MM)
            priority = min(
                HEADING_MAX_PRIORITY,
                (abs(x_off) - HEADING_PRIORITY_X_MM) / span,
            )
            v *= (1.0 - priority)

        # Mix to wheel speeds (W/2 is already absorbed into KP_H/KD_H).
        left  = v - omega
        right = v + omega

        # Proportional saturation: if either wheel would clip the per-wheel
        # cap, scale BOTH wheels by the same factor. Clipping independently
        # warps the v/omega ratio and bins the steering curve during the
        # high-speed approach.
        peak = max(abs(left), abs(right))
        if peak > MAX_WHEEL_SPEED_MMPS:
            scale = MAX_WHEEL_SPEED_MMPS / peak
            left  *= scale
            right *= scale

        # mm/s -> percent for the dumb sketch.
        l_pct = int(round(100.0 * left  / MAX_WHEEL_SPEED_MMPS))
        r_pct = int(round(100.0 * right / MAX_WHEEL_SPEED_MMPS))
        l_pct = max(-100, min(100, l_pct))
        r_pct = max(-100, min(100, r_pct))
        return l_pct, r_pct


# ---------------------------------------------------------------------------
# Live OAK-D Lite brick vision client.
#
# Polls the JSON status endpoint exposed by python/brick_vision/stream.py.
# Returns (x_off_mm, y_off_mm, distance_mm) expressed in the planar PD frame:
#
#     x_off > 0   target is to the robot's right
#     y_off > 0   target is ahead of the robot (horizontal forward projection)
#     distance    straight-line distance to the target, in mm
#
# Returns None when the brick is not detected, confidence is below threshold,
# the spatial fix is invalid, or the HTTP poll fails / times out. The caller
# must treat None as a safety-coast signal.
# ---------------------------------------------------------------------------
class LiveBrickVision:
    def __init__(self, status_url=DEFAULT_VISION_URL,
                 min_confidence=VISION_MIN_CONFIDENCE,
                 timeout_s=VISION_TIMEOUT_S):
        self.status_url     = self._normalize(status_url)
        self.min_confidence = int(min_confidence)
        self.timeout_s      = float(timeout_s)
        self.last_source    = "?"
        self.last_conf      = 0
        self.last_frame_id  = 0

    @staticmethod
    def _normalize(url):
        cleaned = url.strip()
        if cleaned.endswith("/status"):
            return cleaned
        return cleaned.rstrip("/") + "/status"

    def read(self):
        try:
            with urlopen(self.status_url, timeout=self.timeout_s) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except (OSError, URLError, json.JSONDecodeError, ValueError):
            return None

        # /status "frames" counts captured camera frames. The step policy
        # compares successive values to tell a genuinely new frame from a
        # re-poll of the same one (the 20 Hz loop polls a ~20 fps stream),
        # so it never issues two corrections on one piece of evidence.
        try:
            self.last_frame_id = int(payload.get("frames") or 0)
        except (TypeError, ValueError):
            pass

        det = payload.get("detection") or {}
        if not det.get("found"):
            return None

        confidence = int(det.get("confidence") or 0)
        if confidence < self.min_confidence:
            return None

        spatial = det.get("spatial") or {}
        if not spatial.get("valid"):
            return None

        try:
            x_mm    = float(spatial["x_mm"])
            dist_mm = float(spatial["dist_mm"])
        except (KeyError, TypeError, ValueError):
            return None

        # OAK-D Lite reports x as lateral (camera-right positive) and dist
        # as full 3-D range. The PD's head_err = atan2(-x_off, y_off) needs
        # y_off to be the *horizontal* forward component along the robot's
        # heading. Reconstruct it from the right-triangle relationship; the
        # vertical y_mm (mast-relative height) folds out of the planar loop.
        # max(1.0, ...) guards against noisy depth where x² > dist² for
        # an instant.
        y_off = math.sqrt(max(1.0, dist_mm * dist_mm - x_mm * x_mm))

        self.last_source = str(spatial.get("dist_source") or "?")
        self.last_conf   = confidence
        return x_mm, y_off, dist_mm


# ---------------------------------------------------------------------------
# Motor transports
# ---------------------------------------------------------------------------
class SerialLink:
    """USB serial transport. Streams <L,R,M>\n frames at 115200 baud to the
    Uno Q. This is the low-latency default."""
    def __init__(self, port, baud=SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.ser  = None

    def connect(self) -> bool:
        try:
            import serial
        except ImportError as e:
            print(f"[serial] pyserial is required: {e}")
            return False

        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=SERIAL_TIMEOUT_S,
            )
            time.sleep(ARDUINO_RESET_WAIT_S)   # wait out USB-CDC reset
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            print(f"[serial] connected to {self.port} @ {self.baud}")
            return True
        except (serial.SerialException, OSError) as e:
            print(f"[serial] open failed: {e}")
            self.ser = None
            return False

    def send(self, l_pct, r_pct, m_pct=0):
        import serial
        if self.ser is None:
            raise serial.SerialException("not connected")
        self.ser.write(f"<{int(l_pct)},{int(r_pct)},{int(m_pct)}>\n".encode("ascii"))

    def coast(self):
        try:
            if self.ser is not None:
                self.ser.write(b"<0,0,0>\n")
                self.ser.flush()
        except Exception:
            pass

    def close(self):
        try:
            if self.ser is not None:
                self.ser.close()
        finally:
            self.ser = None


class RpcLink:
    """Arduino RouterBridge transport. This is the default on Uno Q because
    arduino-router owns the internal serial device."""
    def __init__(self, socket_path=DEFAULT_ROUTER_SOCKET):
        self.socket_path = socket_path
        self.bridge = None

    def connect(self) -> bool:
        if self.bridge is not None:
            return True
        if not Path(self.socket_path).exists():
            print(f"[rpc] router socket not found at {self.socket_path}")
            return False
        try:
            from arduino.app_utils import Bridge
        except ImportError as e:
            print(f"[rpc] arduino.app_utils unavailable: {e}")
            return False
        try:
            Bridge.call("drive_triple", 0, 0, 0, timeout=RPC_TIMEOUT_S)
        except ValueError as e:
            print(f"[rpc] Uno Q sketch lacks drive_triple: {e}")
            return False
        except (TimeoutError, RuntimeError, OSError) as e:
            print(f"[rpc] connection check failed: {e}")
            return False
        self.bridge = Bridge
        print(f"[rpc] connected through {self.socket_path}")
        return True

    def send(self, l_pct, r_pct):
        if self.bridge is None:
            raise RuntimeError("not connected")
        self.bridge.notify("drive_triple", int(l_pct), int(r_pct), 0)

    def coast(self):
        try:
            if self.bridge is not None:
                self.bridge.notify("drive_triple", 0, 0, 0)
        except Exception:
            pass

    def close(self):
        self.bridge = None


class DryRunLink:
    """Sends no commands. Used to validate the loop on a desk."""
    def connect(self) -> bool:
        print("[dry-run] no hardware attached; commands are discarded")
        return True

    def send(self, l_pct, r_pct):
        return None

    def coast(self):
        return None

    def close(self):
        return None


class CrawlCommandPolicy:
    """Quantize PD wheel intent into Bun's validated crawl vocabulary.

    Output commands are held for frame_s and each wheel is either stopped or
    driven at the calibrated crawl PWM. One-wheel turn frames use a separately
    calibrated turn PWM; two-wheel straight/arc frames stay at crawl PWM.
    """
    def __init__(self, straight_pwm=CRAWL_PWM, turn_pwm=CRAWL_TURN_PWM,
                 frame_s=CRAWL_FRAME_S, pwm=None):
        if pwm is not None:
            straight_pwm = pwm
        self.straight_pwm = int(straight_pwm)
        self.turn_pwm = int(turn_pwm)
        self.frame_s = float(frame_s)
        self.next_frame_t = 0.0
        self.current = (0, 0)
        self.acc_l = 0.0
        self.acc_r = 0.0

    @staticmethod
    def _sign(value):
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def reset(self):
        self.next_frame_t = 0.0
        self.current = (0, 0)
        self.acc_l = 0.0
        self.acc_r = 0.0

    def step(self, l_pct, r_pct, now):
        if now < self.next_frame_t:
            return self.current

        self.next_frame_t = now + self.frame_s
        l_pct = int(l_pct)
        r_pct = int(r_pct)
        max_abs = max(abs(l_pct), abs(r_pct))

        if max_abs <= 0:
            self.current = (0, 0)
            self.acc_l = 0.0
            self.acc_r = 0.0
            return self.current

        sign_l = self._sign(l_pct)
        sign_r = self._sign(r_pct)
        duty_l = abs(l_pct) / max_abs
        duty_r = abs(r_pct) / max_abs

        # Pure spin requests are converted into one-wheel arc turns. Bun has
        # not validated counter-rotating tracks as a safe alignment primitive.
        if sign_l and sign_r and sign_l != sign_r:
            if abs(l_pct) > abs(r_pct):
                duty_r = 0.0
            elif abs(r_pct) > abs(l_pct):
                duty_l = 0.0
            elif r_pct > 0:
                duty_l = 0.0
            else:
                duty_r = 0.0

        out_l = self._dda("l", duty_l, sign_l)
        out_r = self._dda("r", duty_r, sign_r)
        if out_l and not out_r:
            out_l = self._sign(out_l) * self.turn_pwm
        elif out_r and not out_l:
            out_r = self._sign(out_r) * self.turn_pwm
        self.current = (out_l, out_r)
        return self.current

    def _dda(self, side, duty, sign):
        if sign == 0 or duty <= 0.0:
            return 0
        if duty >= 0.98:
            return sign * self.straight_pwm

        if side == "l":
            self.acc_l += duty
            if self.acc_l >= 1.0:
                self.acc_l -= 1.0
                return sign * self.straight_pwm
            return 0

        self.acc_r += duty
        if self.acc_r >= 1.0:
            self.acc_r -= 1.0
            return sign * self.straight_pwm
        return 0


Crawl12CommandPolicy = CrawlCommandPolicy


class StepAligner:
    """Burst-and-verify Step 1 alignment supervisor (command policy "step").

    Emits only commands from Bun's validated vocabulary, always from rest so
    the firmware kick makes each bite repeatable, and never issues a new
    correction before seeing a camera frame captured after the previous one
    finished. See the STEP_* tunables block for the failure analysis behind
    the design.

    step() is called every control tick with the latest vision sample (or
    None) and returns (l_pct, r_pct, phase) in the PD sign convention; the
    caller applies the wiring swap/invert and the MAX_SPEED_LIMIT cap
    exactly as for every other policy.
    """

    def __init__(self, stop_offset_mm, turn_pwm=CRAWL_TURN_PWM,
                 slow_pwm=STEP_SLOW_PWM, fast_pwm=STEP_FAST_PWM,
                 settle_s=STEP_SETTLE_S, seek_enabled=True):
        self.stop = float(stop_offset_mm)
        self.turn_pwm = int(turn_pwm)
        self.slow_pwm = int(slow_pwm)
        self.fast_pwm = int(fast_pwm)
        self.settle_s = float(settle_s)
        self.seek_enabled = bool(seek_enabled)

        self.phase = "acquire"
        self.cmd = (0, 0)
        self.aiming = False
        self.plan_until = 0.0        # wall-clock end of the committed burst/leg
        self.settle_until = 0.0
        self.need_fresh_after = 0.0  # decisions wait for a frame after this

        self.xs = deque(maxlen=STEP_MEDIAN_N)
        self.ds = deque(maxlen=STEP_MEDIAN_N)
        self.xs_ring = deque(maxlen=STEP_RING_WIN)  # stationary-only window
        self.hold_recheck_t = 0.0
        self.evidence_stale = False  # set on loss; clears the window on return
        self.last_frame_id = object()  # sentinel: first real id always differs
        self.last_fresh_t = 0.0
        self.lost_since = None
        self.last_x_sign = 0
        self.seek_used = 0
        self.outlier_strikes = 0
        self.hold_announced = False

    # -- helpers ------------------------------------------------------------
    @staticmethod
    def _median(values):
        ordered = sorted(values)
        return ordered[len(ordered) // 2]

    def _stop(self, now, settle_s, phase):
        self.cmd = (0, 0)
        self.phase = phase
        self.settle_until = now + settle_s
        self.need_fresh_after = now

    @staticmethod
    def _plan(now, seconds):
        # Commit to a whole number of control ticks. Ending a burst exactly
        # ON a tick boundary lets loop jitter round every burst up by one
        # tick (double rotation for a one-tick trim), which is the same
        # overshoot disease the step policy exists to cure.
        ticks = max(1, int(round(seconds * CTRL_HZ)))
        return now + (ticks - 0.5) / CTRL_HZ

    def _turn_cmd(self, x_sign):
        # x > 0 means the brick sits right of center. The PD path maps that
        # to a left-wheel one-wheel frame (verified on the wire in the
        # June 10 log), so the supervisor keeps the identical mapping.
        if x_sign > 0:
            return (self.turn_pwm, 0)
        return (0, self.turn_pwm)

    def _aim_enter_mm(self, d):
        if d <= self.stop + STEP_RING_OUT_MM:
            return STEP_X_HOLD_MM
        if d > STEP_FAST_DIST_MM:
            return max(STEP_X_FAR_ENTER_MIN, STEP_X_FAR_ENTER_FRAC * d)
        return STEP_X_REAIM_NEAR_MM

    def _aim_exit_mm(self, d):
        if d > STEP_FAST_DIST_MM:
            return max(STEP_X_FAR_EXIT_MIN, STEP_X_FAR_EXIT_FRAC * d)
        # Never demand a finer landing than one control tick of rotation
        # can deliver, or the exit re-arms on its own minimum bite.
        bite_mm = d / (STEP_BURST_S_PER_RAD * CTRL_HZ)
        return max(STEP_X_TRIM_EXIT_MM, 0.8 * bite_mm)

    def _trim(self, now, x_evidence, d):
        # Size the burst to the yaw actually needed (small-angle x/d), so a
        # close-range trim is one short bite, not a fixed frame that rotates
        # past center and ping-pongs.
        want_rad = min(0.35, abs(x_evidence) / max(d, 60.0))
        burst = min(STEP_AIM_BURST_MAX_S,
                    max(STEP_AIM_BURST_MIN_S,
                        STEP_BURST_S_PER_RAD * want_rad))
        self.cmd = self._turn_cmd(1 if x_evidence > 0 else -1)
        self.phase = "aim"
        self.xs_ring.clear()
        self.plan_until = self._plan(now, burst)

    def _decide_in_ring(self, now, d):
        # Inside the ring the only question left is the +/-5 mm lateral
        # gate, and a parked robot re-judging sigma ~3 mm samples on every
        # 20 fps frame lets the noise tail fire within a second (the dither
        # the first simulator runs showed). So: judge only on a window of
        # stationary samples, grant hold at the gate width, and once parked
        # re-trim only past a wider margin, at most once per cooldown.
        self.aiming = False
        if len(self.xs_ring) < STEP_RING_MIN_N:
            self.cmd = (0, 0)
            if self.phase != "hold":
                self.phase = "settle"   # parked; gathering evidence
            return
        xe = self._median(self.xs_ring)
        if self.phase == "hold":
            if abs(xe) > STEP_X_REARM_MM and now >= self.hold_recheck_t:
                self.hold_recheck_t = now + STEP_HOLD_RECHECK_S
                self._trim(now, xe, d)
            return
        if abs(xe) <= STEP_X_HOLD_MM:
            self.cmd = (0, 0)
            self.phase = "hold"
            self.hold_recheck_t = now + STEP_HOLD_RECHECK_S
            if not self.hold_announced:
                print(f"[host] step gate satisfied: d={d:.0f}mm "
                      f"x={xe:+.0f}mm -- holding position.")
                self.hold_announced = True
            return
        self._trim(now, xe, d)

    # -- decision point: only ever reached standing still on fresh evidence --
    def _decide(self, now):
        if len(self.ds) < 2:
            self._stop(now, 0.10, "acquire")
            return
        d = self._median(self.ds)
        x = self._median(self.xs)
        if x:
            self.last_x_sign = 1 if x > 0 else -1

        # Too deep: back straight out. Pivoting inside the ring is how the
        # June 10 run lost the brick at d=135 mm.
        if d < self.stop - STEP_RING_IN_MM:
            self.aiming = False
            self.cmd = (-self.slow_pwm, -self.slow_pwm)
            self.phase = "backup"
            self.xs_ring.clear()
            self.plan_until = self._plan(now, STEP_LEG_CLOSE_S)
            return

        if d <= self.stop + STEP_RING_OUT_MM:
            self._decide_in_ring(now, d)
            return

        threshold = self._aim_exit_mm(d) if self.aiming else self._aim_enter_mm(d)
        if abs(x) > threshold:
            self.aiming = True
            self._trim(now, x, d)
            return
        self.aiming = False

        # Straight leg toward the ring, length-capped so a leg can never
        # carry the robot through the ring on stale evidence.
        dist_to_go = d - self.stop
        if dist_to_go <= 60.0:
            leg_s, pwm, est = STEP_LEG_CLOSE_S, self.slow_pwm, STEP_EST_SLOW_MMPS
        elif d <= STEP_FAST_DIST_MM:
            leg_s, pwm, est = STEP_LEG_NEAR_S, self.slow_pwm, STEP_EST_SLOW_MMPS
        else:
            leg_s, pwm, est = STEP_LEG_FAR_S, self.fast_pwm, STEP_EST_FAST_MMPS
        budget_mm = d - (self.stop + STEP_RING_OUT_MM) - 10.0
        if budget_mm > 0:
            leg_s = min(leg_s, max(0.15, budget_mm / est))
        self.cmd = (pwm, pwm)
        self.phase = "drive"
        self.xs_ring.clear()
        self.plan_until = self._plan(now, leg_s)

    # -- lost handling: coast, then a bounded reacquire scan -----------------
    def _lost(self, now, lost_for):
        self.evidence_stale = True
        if self.phase == "hold":
            # Gate already satisfied. A confidence flicker at rest must not
            # un-park the robot; staying put is the only move that cannot
            # make things worse.
            return 0, 0, "hold"
        if self.cmd != (0, 0) and self.phase != "seek":
            self._stop(now, self.settle_s, "seek_wait")
            return 0, 0, self.phase
        if self.phase == "seek":
            if now < self.plan_until:
                return self.cmd[0], self.cmd[1], "seek"
            self.cmd = (0, 0)
            self.phase = "seek_wait"
            self.settle_until = now + STEP_SEEK_LOOK_S
            return 0, 0, self.phase
        if (not self.seek_enabled) or self.last_x_sign == 0:
            self.phase = "lost_coast"
            return 0, 0, self.phase
        if lost_for < STEP_SEEK_DELAY_S or now < self.settle_until:
            self.phase = "seek_wait"
            return 0, 0, self.phase
        if self.seek_used >= STEP_SEEK_BURSTS:
            self.phase = "seek_done"
            return 0, 0, self.phase
        # One pivot frame toward where the brick was last seen, then look.
        self.seek_used += 1
        self.cmd = self._turn_cmd(self.last_x_sign)
        self.phase = "seek"
        self.xs_ring.clear()
        self.plan_until = self._plan(now, STEP_AIM_BURST_MAX_S)
        return self.cmd[0], self.cmd[1], self.phase

    # -- per-tick entry point -------------------------------------------------
    def step(self, sample, now):
        if sample is None:
            if self.lost_since is None:
                self.lost_since = now
            lost_for = now - self.lost_since
            # Let a committed burst/leg finish through a brief confidence
            # flicker; plan_until bounds it and the wire cap bounds the power.
            if (lost_for <= STEP_LOST_GRACE_S and self.cmd != (0, 0)
                    and now < self.plan_until):
                return self.cmd[0], self.cmd[1], self.phase
            return self._lost(now, lost_for)

        self.lost_since = None
        frame_id = sample.get("frame_id")
        if frame_id != self.last_frame_id:
            self.last_frame_id = frame_id
            if self.evidence_stale:
                # The world may have changed during the blackout; an old
                # median must never steer the first post-loss correction.
                self.xs.clear()
                self.ds.clear()
                self.xs_ring.clear()
                self.evidence_stale = False
            x_new = float(sample["x"])
            d_new = float(sample["dist"])
            if (self.cmd == (0, 0) and self.outlier_strikes == 0 and self.ds
                    and (abs(x_new - self._median(self.xs)) > STEP_JUMP_X_MM
                         or abs(d_new - self._median(self.ds)) > STEP_JUMP_D_MM)):
                # A parked robot cannot teleport. One frame that jumps this
                # far is the contour glitch from the June 10 log (d spiked
                # 177 -> 371 mm); two in a row means the world really
                # changed, so only the first is dropped.
                self.outlier_strikes = 1
            else:
                self.outlier_strikes = 0
                self.xs.append(x_new)
                self.ds.append(d_new)
                if self.cmd == (0, 0) and now >= self.settle_until:
                    # Settle exceeds camera latency, so frames landing here
                    # were captured with the robot genuinely at rest.
                    self.xs_ring.append(x_new)
            self.last_fresh_t = now
            if self.phase in ("seek", "seek_wait", "seek_done", "lost_coast"):
                self.seek_used = 0
                self._stop(now, 0.15, "acquire")
                return 0, 0, self.phase

        # Never keep moving on stale evidence.
        if self.cmd != (0, 0) and (now - self.last_fresh_t) > STEP_STALE_STOP_S:
            self._stop(now, self.settle_s, "settle")
            return 0, 0, self.phase

        if self.phase == "aim":
            if now < self.plan_until:
                return self.cmd[0], self.cmd[1], self.phase
            self._stop(now, self.settle_s, "settle")
            return 0, 0, self.phase

        if self.phase in ("drive", "backup"):
            d = self._median(self.ds)
            x = self._median(self.xs)
            ended = now >= self.plan_until
            if self.phase == "drive":
                if d <= self.stop + STEP_RING_OUT_MM + 10.0:
                    ended = True
                if abs(x) > self._aim_enter_mm(d):
                    ended = True
            elif d >= self.stop - STEP_RING_IN_MM + 2.0:
                ended = True
            if not ended:
                return self.cmd[0], self.cmd[1], self.phase
            self._stop(now, STEP_LEG_SETTLE_S, "settle")
            return 0, 0, self.phase

        # settle / acquire / hold: decide only once stopped, settled, and a
        # frame captured after the last motion has arrived.
        if (now >= self.settle_until
                and self.last_fresh_t >= self.need_fresh_after
                and len(self.ds) >= 2):
            self._decide(now)
        return self.cmd[0], self.cmd[1], self.phase


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--transport", choices=("serial", "rpc", "dry-run"),
                    default=None,
                    help="motor transport; default = rpc")
    ap.add_argument("--port", default=None,
                    help=f"USB serial port (default {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD)
    ap.add_argument("--router-socket", default=DEFAULT_ROUTER_SOCKET,
                    help=f"App Lab router socket (default {DEFAULT_ROUTER_SOCKET})")
    ap.add_argument("--vision-url", default=DEFAULT_VISION_URL,
                    help=f"brick-vision /status endpoint (default {DEFAULT_VISION_URL})")
    ap.add_argument("--min-confidence", type=int, default=VISION_MIN_CONFIDENCE,
                    help="ignore detections below this confidence [0-100]")
    ap.add_argument("--stop-offset", type=float, default=STOP_OFFSET_MM,
                    help=f"target straight-line distance in mm (default {STOP_OFFSET_MM:.0f})")
    ap.add_argument("--command-policy",
                    choices=("raw", "crawl", "crawl12", "step"),
                    default="step",
                    help="step = burst-and-verify aligner (default); "
                         "crawl = continuous calibrated frames; "
                         "raw = unshaped PD percentages")
    ap.add_argument("--crawl-pwm", type=int, default=CRAWL_PWM,
                    help=f"two-wheel PWM magnitude for crawl policy (default {CRAWL_PWM})")
    ap.add_argument("--turn-pwm", type=int, default=CRAWL_TURN_PWM,
                    help=f"one-wheel PWM magnitude for crawl policy (default {CRAWL_TURN_PWM})")
    ap.add_argument("--crawl-frame-ms", type=int,
                    default=int(CRAWL_FRAME_S * 1000),
                    help=f"crawl command frame length in ms (default {int(CRAWL_FRAME_S * 1000)})")
    ap.add_argument("--step-fast-pwm", type=int, default=STEP_FAST_PWM,
                    help="step policy: straight PWM for far-field legs "
                         f"(default {STEP_FAST_PWM}; wire cap still applies)")
    ap.add_argument("--step-settle-ms", type=int,
                    default=int(STEP_SETTLE_S * 1000),
                    help="step policy: post-burst settle before the next "
                         f"decision (default {int(STEP_SETTLE_S * 1000)} ms)")
    ap.add_argument("--no-seek", action="store_true",
                    help="step policy: disable the bounded lost-brick scan")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware; commands discarded but PD still runs")
    ap.add_argument("--no-vision", action="store_true",
                    help="skip the live camera and coast forever (smoke test)")
    args = ap.parse_args()

    pd = PdController(args.stop_offset)
    vision = None
    stream_handle = None
    if not args.no_vision:
        vision = LiveBrickVision(args.vision_url, args.min_confidence)
        # Aligning means we need the camera. Bring the stream up ourselves so a
        # bare `python3 host_controller.py` just works; this no-ops when the
        # stream is already serving (e.g. main.py launched it). Wait past
        # stream.py's own device-wait so we don't enter the loop blind.
        stream_handle = ensure_stream(
            args.vision_url,
            ready_timeout_s=30.0,
            on_event=lambda ev, **f: print(f"[vision] {ev}"
                                           + (f" {f}" if f else "")),
        )
        if not stream_handle.ready:
            print("[vision] stream not serving yet — alignment will coast as "
                  "'lost' until the OAK comes up (check the cable/replug).")
    command_policy = None
    step_policy = None
    if args.command_policy == "step":
        step_policy = StepAligner(
            args.stop_offset,
            turn_pwm=args.turn_pwm,
            fast_pwm=args.step_fast_pwm,
            settle_s=max(1, args.step_settle_ms) / 1000.0,
            seek_enabled=not args.no_seek,
        )
    elif args.command_policy in ("crawl", "crawl12"):
        command_policy = CrawlCommandPolicy(
            straight_pwm=args.crawl_pwm,
            turn_pwm=args.turn_pwm,
            frame_s=max(1, args.crawl_frame_ms) / 1000.0,
        )

    # Default to the known-good RouterBridge sketch; --dry-run wins if both
    # are unspecified together.
    transport = args.transport or ("dry-run" if args.dry_run else "rpc")
    if transport == "serial":
        link = SerialLink(args.port or DEFAULT_PORT, args.baud)
    elif transport == "rpc":
        link = RpcLink(args.router_socket)
    else:
        link = DryRunLink()

    # Don't enter the loop blind. Sending phantom commands while the
    # transport is down only hides bugs, and the shipped sketch has no
    # watchdog to coast for us.
    while not link.connect():
        print(f"[{transport}] retrying in {RECONNECT_WAIT_S:.1f}s "
              "(Ctrl-C to abort)...")
        try:
            time.sleep(RECONNECT_WAIT_S)
        except KeyboardInterrupt:
            print()
            if stream_handle is not None:
                stream_handle.stop()
            return

    period = 1.0 / CTRL_HZ
    next_t = time.monotonic()
    last_t = next_t
    tick   = 0
    last_state = "init"   # "tracking" | "lost" | "init"

    print(f"[host] PD loop @ {CTRL_HZ} Hz | STOP_OFFSET={args.stop_offset:.0f} mm | "
          f"vision={'off' if vision is None else args.vision_url} | "
          f"transport={transport} | policy={args.command_policy} "
          f"crawl_pwm={args.crawl_pwm} turn_pwm={args.turn_pwm} "
          f"frame_ms={args.crawl_frame_ms}. Ctrl-C to stop.")

    try:
        while True:
            now = time.monotonic()
            dt  = max(now - last_t, 1e-3)
            last_t = now

            reading = None if vision is None else vision.read()

            step_phase = ""
            if step_policy is not None:
                # Burst-and-verify supervisor. It owns motion timing, so it
                # consumes the raw sample (plus the camera frame counter for
                # freshness) instead of a per-tick PD output. state stays
                # "tracking"/"lost" so main.py's telemetry regex still binds.
                if reading is None:
                    sample = None
                    state = "lost"
                else:
                    x_off, y_off, dist = reading
                    sample = {"x": x_off, "dist": dist,
                              "frame_id": vision.last_frame_id}
                    state = "tracking"
                l_pct, r_pct, step_phase = step_policy.step(sample, now)
            elif reading is None:
                # Vision lost (or disabled) → coast and clear PD memory so
                # the next valid reading isn't seen as a giant derivative
                # step. The shipped sketch latches the last command (no
                # firmware watchdog), so this loop must never go quiet
                # with the tracks moving.
                l_pct, r_pct = 0, 0
                pd.reset()
                if command_policy is not None:
                    command_policy.reset()
                state = "lost"
            else:
                x_off, y_off, dist = reading
                l_pct, r_pct = pd.step(x_off, y_off, dist, dt)
                if command_policy is not None:
                    l_pct, r_pct = command_policy.step(l_pct, r_pct, now)
                state = "tracking"

            # Hardware-wiring correction on the final outputs (see toggles
            # up top). Swap-then-invert so INVERT_* always name the physical
            # channel after any swap. No-op on the (0,0) coast, and the
            # printed L/R below therefore reflect what goes on the wire.
            if SWAP_LEFT_RIGHT_MOTORS:
                l_pct, r_pct = r_pct, l_pct
            if INVERT_LEFT_MOTOR:
                l_pct = -l_pct
            if INVERT_RIGHT_MOTOR:
                r_pct = -r_pct

            # Last line of defence at the wire. Swap/invert only permute or
            # negate, so this is magnitude-only, and it caps every command
            # source above (raw PD, crawl frames, anything added later) so
            # no policy or typo'd flag can launch the tracks past a crawl.
            l_pct = max(-MAX_SPEED_LIMIT, min(MAX_SPEED_LIMIT, l_pct))
            r_pct = max(-MAX_SPEED_LIMIT, min(MAX_SPEED_LIMIT, r_pct))

            if transport != "dry-run":
                try:
                    link.send(l_pct, r_pct)
                except Exception as e:
                    print(f"[{transport}] write failed: {e} — reconnecting...")
                    link.close()
                    while not link.connect():
                        time.sleep(RECONNECT_WAIT_S)
                    pd.reset()
                    if command_policy is not None:
                        command_policy.reset()
                    continue

            tick += 1
            if tick % PRINT_EVERY_N_TICKS == 0 or state != last_state:
                # Appended AFTER every field main.py's TELEMETRY_RE binds to,
                # so the 20-trial scoreboard parses these lines unchanged.
                phase_sfx = f" phase={step_phase}" if step_phase else ""
                if reading is None:
                    print(f"  [{state}]  L={l_pct:+4d}  R={r_pct:+4d}  "
                          f"(no valid brick){phase_sfx}")
                else:
                    x_off, y_off, dist = reading
                    print(f"  [{state}]  d={dist:7.2f}mm  "
                          f"x={x_off:+7.2f}  y_fwd={y_off:+7.2f}  "
                          f"L={l_pct:+4d}  R={r_pct:+4d}  "
                          f"src={vision.last_source} conf={vision.last_conf}"
                          f"{phase_sfx}")
            last_state = state

            # Pace at CTRL_HZ; re-baseline if we ever fall behind so we
            # don't fire a catch-up burst.
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\n[host] Ctrl-C — coasting motors and exiting.")
    finally:
        link.coast()
        link.close()
        # Only tears down a stream WE started; leaves an externally-owned one
        # (main.py, manual launch) running.
        if stream_handle is not None:
            stream_handle.stop()


if __name__ == "__main__":
    main()
