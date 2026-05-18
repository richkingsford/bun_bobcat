"""
pd_simulator.py — Bun PD-controlled approach simulator.

Standalone 2D top-down simulation: differential-drive robot uses a PD
controller with Astolfi-style velocity scaling to glide onto a target,
stopping at exactly STOP_OFFSET_MM with zero overshoot.

Plant model is two-stage: first-order motor lag (tau ≈ 80 ms) feeding
rigid-body diff-drive kinematics. Inner physics at 200 Hz; outer control
at 20 Hz to match the real host_controller cadence.

Gain derivation (both loops at zeta ≈ 1.5):

  Distance loop, with motor lag tau:
      tau * e_dd + (1 + Kd_d) * e_d + Kp_d * e = 0
      omega_n = sqrt(Kp_d / tau)
      zeta    = (1 + Kd_d) / (2 * sqrt(Kp_d * tau))
  tau=0.08, Kp_d=5, Kd_d=0.9 -> omega_n=7.91, zeta=1.50.

  Heading loop (W/2 absorbed into the gains so mixing is just v +/- omega):
      omega_n = sqrt((2/W) * Kp_h / tau)
      zeta    = (1 + (2/W) * Kd_h) / (2 * sqrt(tau * (2/W) * Kp_h))
  W=90, tau=0.08, Kp_h=200, Kd_h=35 -> omega_n=7.45, zeta=1.49.

Run:
    python3 pd_simulator.py
"""

import math
from dataclasses import dataclass, field
from typing import List

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.transforms as transforms
from matplotlib.patches import Circle, Rectangle


# ---------------------------------------------------------------------------
# Mechanical / plant constants
# ---------------------------------------------------------------------------
WHEEL_BASE_MM        = 90.0       # tread-to-tread separation
MAX_WHEEL_SPEED_MMPS = 250.0      # per-wheel saturation
MOTOR_TAU_S          = 0.08       # 1st-order motor lag

# ---------------------------------------------------------------------------
# PD gains (see header derivation, zeta ≈ 1.5 both loops)
# ---------------------------------------------------------------------------
KP_D = 5.0
KD_D = 0.9
KP_H = 200.0
KD_H = 35.0

# ---------------------------------------------------------------------------
# Setpoints, scenario, sim params
# ---------------------------------------------------------------------------
STOP_OFFSET_MM = 7.0
TARGET_XY      = (400.0, 250.0)
START_POSE     = (0.0, 0.0, 0.0)   # x_mm, y_mm, theta_rad

DT_S           = 1.0 / 200.0        # 200 Hz inner physics
CTRL_HZ        = 20                 # 20 Hz outer control (matches real host)
ANIM_FPS       = 30
SIM_TIMEOUT_S  = 6.0
SETTLED_TAIL_S = 0.3                # keep showing settled state briefly


# ---------------------------------------------------------------------------
# State containers
# ---------------------------------------------------------------------------
@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians, CCW positive, 0 = +x


@dataclass
class WheelState:
    v: float = 0.0      # actual mm/s after motor lag
    cmd: float = 0.0    # commanded mm/s


@dataclass
class PdMemory:
    last_dist_err: float = 0.0
    last_head_err: float = 0.0
    initialized: bool = False


@dataclass
class History:
    x: List[float]      = field(default_factory=list)
    y: List[float]      = field(default_factory=list)
    th: List[float]     = field(default_factory=list)
    t: List[float]      = field(default_factory=list)
    dist: List[float]   = field(default_factory=list)
    L: List[float]      = field(default_factory=list)
    R: List[float]      = field(default_factory=list)
    t_ctrl: List[float] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def wrap_pi(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def compute_offsets(pose: Pose, target_xy):
    """Return (x_off, y_off, dist) of target expressed in the robot frame.

    Convention:  y_off > 0 = ahead of robot,  x_off > 0 = to robot's right.
    """
    tx, ty = target_xy
    dx, dy = tx - pose.x, ty - pose.y
    y_off =  dx * math.cos(pose.theta) + dy * math.sin(pose.theta)
    x_off =  dx * math.sin(pose.theta) - dy * math.cos(pose.theta)
    return x_off, y_off, math.hypot(dx, dy)


# ---------------------------------------------------------------------------
# PD controller — one tick
# ---------------------------------------------------------------------------
def pd_step(x_off, y_off, distance, dt, mem: PdMemory):
    """One control tick. Returns (left_cmd, right_cmd) mm/s, saturated."""
    # Bearing to target from robot's forward axis, CCW positive.
    head_err = 0.0 if (x_off == 0.0 and y_off == 0.0) \
        else math.atan2(-x_off, y_off)
    dist_err = distance - STOP_OFFSET_MM

    if not mem.initialized:
        mem.last_dist_err = dist_err
        mem.last_head_err = head_err
        mem.initialized = True

    d_dist = (dist_err - mem.last_dist_err) / dt
    d_head = wrap_pi(head_err - mem.last_head_err) / dt
    mem.last_dist_err = dist_err
    mem.last_head_err = head_err

    # Astolfi: scale forward velocity by cos(bearing). When perpendicular to
    # the target the robot pivots in place; when facing it, full speed.
    v     = (KP_D * dist_err + KD_D * d_dist) * math.cos(head_err)
    omega = KP_H * head_err + KD_H * d_head

    # Mix to wheel speeds (W/2 already absorbed into KP_H/KD_H).
    left  = v - omega
    right = v + omega

    # Proportional saturation: if either wheel clips, scale BOTH by the same
    # factor. Clipping independently would warp v/omega and bin the steering
    # curve during the high-speed approach.
    peak = max(abs(left), abs(right))
    if peak > MAX_WHEEL_SPEED_MMPS:
        scale = MAX_WHEEL_SPEED_MMPS / peak
        left  *= scale
        right *= scale

    return left, right


# ---------------------------------------------------------------------------
# Plant: motor lag + diff-drive kinematics, one inner-physics step
# ---------------------------------------------------------------------------
def step_plant(pose: Pose, left: WheelState, right: WheelState, dt: float):
    alpha = dt / (MOTOR_TAU_S + dt)
    left.v  += (left.cmd  - left.v)  * alpha
    right.v += (right.cmd - right.v) * alpha

    v     = 0.5 * (right.v + left.v)
    omega = (right.v - left.v) / WHEEL_BASE_MM   # physical rad/s

    pose.theta = wrap_pi(pose.theta + omega * dt)
    pose.x += v * math.cos(pose.theta) * dt
    pose.y += v * math.sin(pose.theta) * dt


# ---------------------------------------------------------------------------
# Sim loop
# ---------------------------------------------------------------------------
def run_simulation() -> History:
    pose  = Pose(*START_POSE)
    left  = WheelState()
    right = WheelState()
    mem   = PdMemory()
    hist  = History()

    t = 0.0
    ctrl_period   = 1.0 / CTRL_HZ
    next_ctrl     = 0.0
    settled_since = None

    while t < SIM_TIMEOUT_S:
        if t >= next_ctrl:
            x_off, y_off, dist = compute_offsets(pose, TARGET_XY)
            L_cmd, R_cmd = pd_step(x_off, y_off, dist, ctrl_period, mem)
            left.cmd, right.cmd = L_cmd, R_cmd
            hist.L.append(L_cmd); hist.R.append(R_cmd)
            hist.dist.append(dist); hist.t_ctrl.append(t)
            next_ctrl += ctrl_period

        step_plant(pose, left, right, DT_S)
        hist.x.append(pose.x); hist.y.append(pose.y)
        hist.th.append(pose.theta); hist.t.append(t)
        t += DT_S

        # Stop once we're sitting on the stop-ring with ~zero velocity.
        dist_now = math.hypot(TARGET_XY[0] - pose.x, TARGET_XY[1] - pose.y)
        v_now    = 0.5 * (left.v + right.v)
        if abs(dist_now - STOP_OFFSET_MM) < 0.1 and abs(v_now) < 1.0:
            if settled_since is None:
                settled_since = t
            if t - settled_since >= SETTLED_TAIL_S:
                break
        else:
            settled_since = None

    return hist


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
def animate(hist: History):
    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [2, 1]}
    )
    fig.suptitle("Bun PD Approach Simulator", fontsize=13)

    # Top-down pane
    margin = 60
    xs, ys = hist.x, hist.y
    xmin = min(xs + [TARGET_XY[0], START_POSE[0]]) - margin
    xmax = max(xs + [TARGET_XY[0], START_POSE[0]]) + margin
    ymin = min(ys + [TARGET_XY[1], START_POSE[1]]) - margin
    ymax = max(ys + [TARGET_XY[1], START_POSE[1]]) + margin
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.set_title("Top-down trajectory (mm)")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.grid(alpha=0.3)

    ax.add_patch(Circle(TARGET_XY, STOP_OFFSET_MM, fill=False,
                        ec="crimson", ls="--", lw=1.4,
                        label=f"{STOP_OFFSET_MM:.0f} mm stop ring"))
    ax.plot(*TARGET_XY, marker="x", color="crimson",
            markersize=11, mew=2.0, label="target")
    ax.plot(START_POSE[0], START_POSE[1], marker="o",
            color="seagreen", markersize=8, label="start")

    trail, = ax.plot([], [], "-", color="steelblue", lw=1.4, alpha=0.85)
    body_w, body_l = WHEEL_BASE_MM, 100.0
    robot_body = Rectangle((-body_l / 2, -body_w / 2), body_l, body_w,
                           fc="#9ec5e8", ec="black", lw=1.2)
    ax.add_patch(robot_body)
    nose, = ax.plot([], [], "-", color="black", lw=2)
    ax.legend(loc="lower right", fontsize=8)

    # Distance pane
    ax2.set_xlim(0, max(hist.t_ctrl) if hist.t_ctrl else 1)
    ax2.set_ylim(0, max(hist.dist) * 1.05 if hist.dist else 1)
    ax2.axhline(STOP_OFFSET_MM, color="crimson", ls="--", lw=1.2,
                label=f"{STOP_OFFSET_MM:.0f} mm setpoint")
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("range to target (mm)")
    ax2.set_title("Closing range"); ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8)
    dist_line, = ax2.plot([], [], color="steelblue", lw=1.5)

    sim_hz   = int(round(1.0 / DT_S))
    stride   = max(1, sim_hz // ANIM_FPS)
    n_frames = max(1, len(xs) // stride)

    def init():
        trail.set_data([], []); nose.set_data([], [])
        dist_line.set_data([], [])
        return trail, nose, robot_body, dist_line

    def update(i):
        idx = min(i * stride, len(xs) - 1)
        trail.set_data(xs[: idx + 1], ys[: idx + 1])
        th = hist.th[idx]
        tr = (transforms.Affine2D().rotate(th)
              .translate(xs[idx], ys[idx])) + ax.transData
        robot_body.set_transform(tr)
        nose.set_data(
            [xs[idx], xs[idx] + 55 * math.cos(th)],
            [ys[idx], ys[idx] + 55 * math.sin(th)],
        )
        t_now = hist.t[idx]
        k = 0
        for j, tc in enumerate(hist.t_ctrl):
            if tc <= t_now: k = j
            else: break
        dist_line.set_data(hist.t_ctrl[: k + 1], hist.dist[: k + 1])
        return trail, nose, robot_body, dist_line

    ani = animation.FuncAnimation(
        fig, update, frames=n_frames, init_func=init,
        interval=1000 / ANIM_FPS, blit=False, repeat=False,
    )
    plt.tight_layout(rect=(0, 0, 1, 0.96))
    return fig, ani


if __name__ == "__main__":
    hist = run_simulation()
    final = math.hypot(hist.x[-1] - TARGET_XY[0], hist.y[-1] - TARGET_XY[1])
    print(f"Final range:    {final:7.3f} mm")
    print(f"Setpoint:       {STOP_OFFSET_MM:7.3f} mm")
    print(f"Steady-state e: {final - STOP_OFFSET_MM:+7.3f} mm")
    print(f"Sim duration:   {hist.t[-1]:7.2f} s")
    fig, ani = animate(hist)
    plt.show()
