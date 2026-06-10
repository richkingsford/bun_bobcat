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
reading doesn't fire a phantom d/dt spike on reacquisition. The sketch's
500 ms watchdog is a second line of defence; this is the first.

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
import json
import math
from pathlib import Path
import time
from urllib.error import URLError
from urllib.request import urlopen


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
HEADING_MAX_PRIORITY = 0.65

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
VISION_MIN_CONFIDENCE = 55      # matches align_to_brick.py default

# Telemetry print rate
PRINT_EVERY_N_TICKS = 5         # 20 Hz / 5 = 4 Hz log

# Crawl output policy. Bun's validated low-speed straight command is 13% PWM.
# One-wheel turn frames use 23% PWM because single-tread breakaway is higher.
CRAWL_PWM = 13
CRAWL_TURN_PWM = 23
CRAWL_FRAME_S = 0.150

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
    ap.add_argument("--command-policy", choices=("raw", "crawl", "crawl12"),
                    default="crawl",
                    help="raw PD percentages or Bun's calibrated crawl frames")
    ap.add_argument("--crawl-pwm", type=int, default=CRAWL_PWM,
                    help=f"two-wheel PWM magnitude for crawl policy (default {CRAWL_PWM})")
    ap.add_argument("--turn-pwm", type=int, default=CRAWL_TURN_PWM,
                    help=f"one-wheel PWM magnitude for crawl policy (default {CRAWL_TURN_PWM})")
    ap.add_argument("--crawl-frame-ms", type=int,
                    default=int(CRAWL_FRAME_S * 1000),
                    help=f"crawl command frame length in ms (default {int(CRAWL_FRAME_S * 1000)})")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware; commands discarded but PD still runs")
    ap.add_argument("--no-vision", action="store_true",
                    help="skip the live camera and coast forever (smoke test)")
    args = ap.parse_args()

    pd = PdController(args.stop_offset)
    vision = None if args.no_vision else LiveBrickVision(
        args.vision_url, args.min_confidence)
    command_policy = None
    if args.command_policy in ("crawl", "crawl12"):
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
    # transport is down only hides bugs; the sketch's watchdog will coast.
    while not link.connect():
        print(f"[{transport}] retrying in {RECONNECT_WAIT_S:.1f}s "
              "(Ctrl-C to abort)...")
        try:
            time.sleep(RECONNECT_WAIT_S)
        except KeyboardInterrupt:
            print(); return

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

            if reading is None:
                # Vision lost (or disabled) → coast and clear PD memory so
                # the next valid reading isn't seen as a giant derivative
                # step. Sketch's 500 ms watchdog is the secondary fence.
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
                if reading is None:
                    print(f"  [{state}]  L={l_pct:+4d}  R={r_pct:+4d}  "
                          f"(no valid brick)")
                else:
                    x_off, y_off, dist = reading
                    print(f"  [{state}]  d={dist:7.2f}mm  "
                          f"x={x_off:+7.2f}  y_fwd={y_off:+7.2f}  "
                          f"L={l_pct:+4d}  R={r_pct:+4d}  "
                          f"src={vision.last_source} conf={vision.last_conf}")
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


if __name__ == "__main__":
    main()
