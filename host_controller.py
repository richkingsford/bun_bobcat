"""
host_controller.py — Bun host-side PD controller.

The Arduino Uno Q is treated as a completely dumb motor driver. This script
owns the PD loop, the vision processing, and the trajectory. It pushes
framed speed commands over USB serial; the Arduino just decodes them.

Pipeline each 50 ms tick:

    [vision: x_off, y_off, distance]  ->  PD  ->  L%, R%  ->  "<L,R>\\n"

Serial protocol (must match sketch.ino):
    <L,R>\\n         L,R are signed decimal integers in [-100, 100]:
                       > 0  forward at |x|% (sign-mapped on Arduino)
                       < 0  reverse at |x|%
                       = 0  coast

Vision is supplied by VisionStub by default so the script is runnable
without a camera. To go live, swap VisionStub for an object exposing
.read() -> (x_off_mm, y_off_mm, distance_mm) in the robot frame.

Usage:
    python3 host_controller.py --port /dev/ttyACM0
    python3 host_controller.py --port /dev/ttyACM0 --target 400,250
    python3 host_controller.py --no-serial          # dry-run, no Arduino
"""

import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required:  pip install pyserial", file=sys.stderr)
    raise


# ---------------------------------------------------------------------------
# Tunables — kept in lock-step with pd_simulator.py
# ---------------------------------------------------------------------------
WHEEL_BASE_MM        = 90.0
MAX_WHEEL_SPEED_MMPS = 250.0

STOP_OFFSET_MM = 7.0

KP_D = 5.0
KD_D = 0.9
KP_H = 200.0
KD_H = 35.0

CTRL_HZ = 20

# Serial transport
DEFAULT_PORT         = "/dev/ttyACM0"
SERIAL_BAUD          = 115200
SERIAL_TIMEOUT_S     = 0.05
RECONNECT_WAIT_S     = 1.0
ARDUINO_RESET_WAIT_S = 2.0   # most Arduino boards reset on USB open

# Telemetry print rate
PRINT_EVERY_N_TICKS = 5      # 20 Hz / 5 = 4 Hz log


# ---------------------------------------------------------------------------
# PD controller — math identical to pd_simulator.pd_step
# ---------------------------------------------------------------------------
class PdController:
    def __init__(self):
        self.last_dist_err = 0.0
        self.last_head_err = 0.0
        self.initialized   = False

    @staticmethod
    def _wrap_pi(a):
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def step(self, x_off, y_off, distance, dt):
        """Run one PD tick. Returns (L_pct, R_pct) ints in [-100, 100]."""
        head_err = 0.0 if (x_off == 0.0 and y_off == 0.0) \
            else math.atan2(-x_off, y_off)
        dist_err = distance - STOP_OFFSET_MM

        if not self.initialized:
            self.last_dist_err = dist_err
            self.last_head_err = head_err
            self.initialized   = True

        d_dist = (dist_err - self.last_dist_err) / dt
        d_head = self._wrap_pi(head_err - self.last_head_err) / dt
        self.last_dist_err = dist_err
        self.last_head_err = head_err

        v     = (KP_D * dist_err + KD_D * d_dist) * math.cos(head_err)
        omega = KP_H * head_err + KD_H * d_head

        left  = v - omega
        right = v + omega

        # Proportional saturation — preserve v/omega ratio.
        peak = max(abs(left), abs(right))
        if peak > MAX_WHEEL_SPEED_MMPS:
            scale = MAX_WHEEL_SPEED_MMPS / peak
            left  *= scale
            right *= scale

        # mm/s -> percent for the dumb driver.
        l_pct = int(round(100.0 * left  / MAX_WHEEL_SPEED_MMPS))
        r_pct = int(round(100.0 * right / MAX_WHEEL_SPEED_MMPS))
        l_pct = max(-100, min(100, l_pct))
        r_pct = max(-100, min(100, r_pct))
        return l_pct, r_pct


# ---------------------------------------------------------------------------
# Vision stub — simulates the camera/perception so the loop is testable
# without hardware. Replace with the real pipeline when wiring up.
# ---------------------------------------------------------------------------
class VisionStub:
    def __init__(self, target_xy=(400.0, 250.0)):
        self.tx, self.ty = target_xy
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.left_v = 0.0; self.right_v = 0.0
        self.tau = 0.08

    def integrate(self, l_pct, r_pct, dt):
        l_target = (l_pct / 100.0) * MAX_WHEEL_SPEED_MMPS
        r_target = (r_pct / 100.0) * MAX_WHEEL_SPEED_MMPS
        alpha = dt / (self.tau + dt)
        self.left_v  += (l_target - self.left_v)  * alpha
        self.right_v += (r_target - self.right_v) * alpha
        v     = 0.5 * (self.left_v + self.right_v)
        omega = (self.right_v - self.left_v) / WHEEL_BASE_MM
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def read(self):
        dx, dy = self.tx - self.x, self.ty - self.y
        y_off =  dx * math.cos(self.theta) + dy * math.sin(self.theta)
        x_off =  dx * math.sin(self.theta) - dy * math.cos(self.theta)
        return x_off, y_off, math.hypot(dx, dy)


# ---------------------------------------------------------------------------
# Serial transport with auto-reconnect
# ---------------------------------------------------------------------------
class SerialLink:
    def __init__(self, port, baud=SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.ser  = None

    def connect(self) -> bool:
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

    def send(self, l_pct, r_pct):
        if self.ser is None:
            raise serial.SerialException("not connected")
        self.ser.write(f"<{l_pct},{r_pct}>\n".encode("ascii"))

    def coast(self):
        try:
            if self.ser is not None:
                self.ser.write(b"<0,0>\n")
                self.ser.flush()
        except Exception:
            pass

    def close(self):
        try:
            if self.ser is not None:
                self.ser.close()
        finally:
            self.ser = None


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"serial port (default {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD)
    ap.add_argument("--target", default="400,250",
                    help="virtual vision target in mm, format: x,y")
    ap.add_argument("--no-serial", action="store_true",
                    help="run the PD loop with no Arduino attached")
    args = ap.parse_args()

    try:
        tx, ty = (float(v) for v in args.target.split(","))
    except ValueError:
        ap.error("--target must be 'x,y' (mm)")

    pd     = PdController()
    vision = VisionStub((tx, ty))
    link   = SerialLink(args.port, args.baud)

    if not args.no_serial:
        # Don't enter the loop blind. Sending phantom commands while
        # disconnected would only hide bugs; the Arduino watchdog will
        # coast the motors after CMD_TIMEOUT_MS while we're down.
        while not link.connect():
            print(f"[serial] retrying in {RECONNECT_WAIT_S:.1f}s "
                  "(Ctrl-C to abort)...")
            try:
                time.sleep(RECONNECT_WAIT_S)
            except KeyboardInterrupt:
                print(); return

    period = 1.0 / CTRL_HZ
    next_t = time.monotonic()
    last_t = next_t
    tick   = 0

    print(f"[host] PD loop running at {CTRL_HZ} Hz. Ctrl-C to stop.")

    try:
        while True:
            now = time.monotonic()
            dt  = max(now - last_t, 1e-3)
            last_t = now

            x_off, y_off, dist = vision.read()
            l_pct, r_pct = pd.step(x_off, y_off, dist, dt)

            if not args.no_serial:
                try:
                    link.send(l_pct, r_pct)
                except (serial.SerialException, OSError) as e:
                    print(f"[serial] write failed: {e} — reconnecting...")
                    link.close()
                    while not link.connect():
                        time.sleep(RECONNECT_WAIT_S)
                    continue   # skip this tick's vision integration

            vision.integrate(l_pct, r_pct, period)

            tick += 1
            if tick % PRINT_EVERY_N_TICKS == 0:
                print(f"  d={dist:7.2f}mm  x_off={x_off:+7.2f}  "
                      f"y_off={y_off:+7.2f}  L={l_pct:+4d}  R={r_pct:+4d}")

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
