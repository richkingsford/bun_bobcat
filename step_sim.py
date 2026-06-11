#!/usr/bin/env python3
"""Kinematic acceptance simulator for the "step" (burst-and-verify) policy.

pd_simulator.py proves the PD math; this file proves the StepAligner state
machine against a plant that reproduces the failure modes observed in the
June 10 live run on Bun:

  * tracks with a dead zone, a start/direction-change kick, and a stiction
    stall on sustained single-tread commands (the +67 mm pin-then-66 mm-snap
    in the log);
  * a ~20 fps camera with ~120 ms latency, gaussian noise, occasional
    outlier frames (the d 177 -> 371 mm glitch), dropped frames, and a hard
    field-of-view limit (how the brick was actually lost);
  * an optional scripted vision blackout to exercise the bounded seek.

Distances in mm, times in seconds, angles in radians. The sim runs in
policy space: +PWM = forward, x > 0 = brick right of camera center. The
host's wiring swap/invert sits outside the loop under test on real Bun.

Run:
    python3 step_sim.py                  # full acceptance matrix, exit 0/1
    python3 step_sim.py --policy crawl   # informational A/B of the old policy
    python3 step_sim.py --seed 3 --x0 67 --d0 320 --verbose
"""

import argparse
import math
import random
import sys

from host_controller import (
    CRAWL_FRAME_S,
    CRAWL_PWM,
    CRAWL_TURN_PWM,
    CrawlCommandPolicy,
    PdController,
    StepAligner,
)

# --------------------------------------------------------------------------
# Plant model, calibrated to the June 8 motor diagnostics and June 10 log:
# straight 13 percent covered ~55-64 mm/s; a sustained one-wheel 23 percent
# frame stalled, then snapped. Speeds are steady-state track surface speeds.
# --------------------------------------------------------------------------
WHEELBASE_MM = 90.0
PHYS_HZ = 200.0
CTRL_HZ = 20.0

KICK_GAIN = 1.5          # firmware boost on start / direction change
KICK_S = 0.12
STICTION_AFTER_S = 0.8   # sustained single-tread command begins to pin
SNAP_DEFICIT_MM = 20.0   # stored track travel before breakaway
SNAP_RELEASE_S = 0.12    # the stored travel releases in roughly one frame

CAM_FPS = 20.0
CAM_LATENCY_S = 0.12
CAM_SIGMA_X = 3.0
CAM_SIGMA_D = 4.0
CAM_OUTLIER_P = 0.015    # contour glitch: d inflates, x jumps
CAM_DROP_P = 0.04
CAM_FOV_DEG = 30.0

STOP_MM = 170.0
GATE_D_TOL = 20.0        # mirrors main.py DIST_TOL_MM
GATE_X_TOL = 5.0         # mirrors main.py X_TOL_MM
GATE_SAMPLES = 2         # mirrors main.py SUCCESS_SAMPLES


def two_track_speed(pwm: float) -> float:
    p = abs(pwm)
    if p < 11.0:
        return 0.0
    if p < 13.0:
        v = 25.0 + (p - 11.0) * 17.5
    else:
        v = 60.0 + (p - 13.0) * 5.0          # 13 -> 60, 20 -> 95, 25 -> 120
    return math.copysign(v, pwm)


def one_track_speed(pwm: float) -> float:
    p = abs(pwm)
    if p < 11.0:
        return 0.0
    return math.copysign(70.0 * (p - 10.0) / 13.0, pwm)   # 23 -> 70


class Track:
    """One tread: dead zone, start kick, and the pin-then-snap stiction the
    June 10 log shows on sustained single-tread commands (x pinned for
    ~1.2 s at +67 mm, then a 66 mm jump in one frame)."""

    def __init__(self):
        self.pwm = 0.0
        self.since_change = 1e9   # seconds since start or direction change
        self.solo_for = 0.0       # continuous seconds as the only mover
        self.deficit = 0.0        # commanded-but-undelivered track travel
        self.release_left = 0.0
        self.release_speed = 0.0

    def command(self, pwm: float, dt: float, solo: bool) -> float:
        started = (self.pwm == 0.0) != (pwm == 0.0)
        flipped = self.pwm * pwm < 0.0
        if started or flipped:
            self.since_change = 0.0
            self.solo_for = 0.0
            self.deficit = 0.0
            self.release_left = 0.0
        else:
            self.since_change += dt
        self.pwm = pwm
        if solo and pwm != 0.0:
            self.solo_for += dt
        else:
            self.solo_for = 0.0
            self.deficit = 0.0
            self.release_left = 0.0

        v = one_track_speed(pwm) if solo else two_track_speed(pwm)
        if self.since_change < KICK_S:
            return v * KICK_GAIN
        if solo and pwm != 0.0 and self.solo_for > STICTION_AFTER_S:
            if self.release_left > 0.0:
                self.release_left -= dt
                return math.copysign(self.release_speed, pwm)
            self.deficit += abs(v) * dt
            if self.deficit >= SNAP_DEFICIT_MM:
                self.release_speed = self.deficit / SNAP_RELEASE_S
                self.release_left = SNAP_RELEASE_S
                self.deficit = 0.0
            return 0.0
        return v


class World:
    def __init__(self, d0: float, x0: float, seed: int):
        self.rng = random.Random(seed)
        # Brick at origin; robot placed so the camera-frame x equals x0.
        self.px, self.py = -x0, -d0
        self.theta = 0.0
        self.left = Track()
        self.right = Track()
        self.t = 0.0

    def true_view(self):
        bx, by = -self.px, -self.py
        h = (math.sin(self.theta), math.cos(self.theta))
        r = (math.cos(self.theta), -math.sin(self.theta))
        fwd = bx * h[0] + by * h[1]
        lat = bx * r[0] + by * r[1]
        dist = math.hypot(bx, by)
        bearing = math.degrees(math.atan2(lat, max(fwd, 1e-6)))
        visible = fwd > 0 and abs(bearing) <= CAM_FOV_DEG
        return lat, dist, visible

    def advance(self, l_pwm: float, r_pwm: float, dt: float):
        solo_l = l_pwm != 0.0 and r_pwm == 0.0
        solo_r = r_pwm != 0.0 and l_pwm == 0.0
        vl = self.left.command(l_pwm, dt, solo_l)
        vr = self.right.command(r_pwm, dt, solo_r)
        v = (vl + vr) / 2.0
        omega_cw = (vl - vr) / WHEELBASE_MM
        self.theta += omega_cw * dt
        self.px += math.sin(self.theta) * v * dt
        self.py += math.cos(self.theta) * v * dt
        self.t += dt


class Camera:
    """Latent, noisy 20 fps feed with a monotonically increasing frame id."""

    def __init__(self, world: World, blackout=None):
        self.world = world
        self.rng = world.rng
        self.blackout = blackout            # (t_start, t_end) or None
        self.next_capture = 0.0
        self.fid = 0
        self.delivered = []                 # (deliver_t, fid, sample|None)

    def maybe_capture(self, t: float):
        while t >= self.next_capture:
            self.fid += 1
            lat, dist, visible = self.world.true_view()
            sample = None
            dark = (self.blackout
                    and self.blackout[0] <= self.next_capture < self.blackout[1])
            if visible and not dark and self.rng.random() >= CAM_DROP_P:
                x = lat + self.rng.gauss(0.0, CAM_SIGMA_X)
                d = dist + self.rng.gauss(0.0, CAM_SIGMA_D)
                if self.rng.random() < CAM_OUTLIER_P:
                    d += 170.0
                    x += math.copysign(140.0, self.rng.random() - 0.5)
                sample = {"x": x, "dist": max(d, 1.0), "frame_id": self.fid}
            self.delivered.append((self.next_capture + CAM_LATENCY_S,
                                   self.fid, sample))
            self.next_capture += 1.0 / CAM_FPS

    def latest(self, t: float):
        """Newest capture whose latency has elapsed, like polling /status."""
        newest = None
        while self.delivered and self.delivered[0][0] <= t:
            newest = self.delivered.pop(0)
            self._last = newest
        if newest is None:
            newest = getattr(self, "_last", None)
        return None if newest is None else newest[2]


def run_once(policy_name: str, d0: float, x0: float, seed: int,
             t_max: float = 90.0, blackout=None, verbose: bool = False):
    world = World(d0, x0, seed)
    cam = Camera(world, blackout=blackout)

    aligner = StepAligner(STOP_MM)
    pd = PdController(STOP_MM)
    crawl = CrawlCommandPolicy(straight_pwm=CRAWL_PWM, turn_pwm=CRAWL_TURN_PWM,
                               frame_s=CRAWL_FRAME_S)

    dt_phys = 1.0 / PHYS_HZ
    ctrl_period = 1.0 / CTRL_HZ
    next_ctrl = 0.0
    l_cmd = r_cmd = 0.0

    gate_run = 0
    gate_t = None
    aim_dirs = []
    max_abs_x = 0.0

    while world.t < t_max:
        cam.maybe_capture(world.t)
        if world.t >= next_ctrl:
            sample = cam.latest(world.t)
            if policy_name == "step":
                l_cmd, r_cmd, phase = aligner.step(sample, world.t)
                if phase == "aim" and (l_cmd, r_cmd) != (0, 0):
                    direction = 1 if l_cmd > 0 else -1
                    if not aim_dirs or aim_dirs[-1] != direction:
                        aim_dirs.append(direction)
            else:
                if sample is None:
                    l_cmd, r_cmd = 0, 0
                    pd.reset()
                    crawl.reset()
                else:
                    x = sample["x"]
                    d = sample["dist"]
                    y_off = math.sqrt(max(1.0, d * d - x * x))
                    l_cmd, r_cmd = pd.step(x, y_off, d, ctrl_period)
                    l_cmd, r_cmd = crawl.step(l_cmd, r_cmd, world.t)
            # Wire cap, as in the host (swap/invert are sign games only).
            l_cmd = max(-35, min(35, l_cmd))
            r_cmd = max(-35, min(35, r_cmd))

            if sample is None:
                gate_run = 0
            else:
                ok = (abs(sample["dist"] - STOP_MM) <= GATE_D_TOL
                      and abs(sample["x"]) <= GATE_X_TOL)
                gate_run = gate_run + 1 if ok else 0
                if gate_run >= GATE_SAMPLES and gate_t is None:
                    gate_t = world.t
            if verbose and abs(next_ctrl * 2 - round(next_ctrl * 2)) < 1e-9:
                lat, dist, vis = world.true_view()
                print(f"t={world.t:6.2f}  true d={dist:6.1f} x={lat:+7.1f} "
                      f"vis={int(vis)}  cmd=({l_cmd:+3.0f},{r_cmd:+3.0f})  "
                      f"{'phase=' + phase if policy_name == 'step' else ''}")
            next_ctrl += ctrl_period
        world.advance(l_cmd, r_cmd, dt_phys)
        lat, _, _ = world.true_view()
        if world.t > 1.0:
            max_abs_x = max(max_abs_x, abs(lat))
        if gate_t is not None and world.t > gate_t + 2.0:
            break   # held the gate; nothing more to learn from this run

    flips = sum(1 for a, b in zip(aim_dirs, aim_dirs[1:]) if a != b)
    return {
        "gate_t": gate_t,
        "flips": flips,
        "max_abs_x": max_abs_x,
        "blowup": max_abs_x > abs(x0) + 150.0,
    }


def acceptance() -> int:
    starts = [(320.0, 67.0), (400.0, -120.0), (250.0, 20.0)]
    seeds = range(1, 9)
    failures = []
    times = []
    print(f"{'d0':>5} {'x0':>6} {'seed':>4}   {'gate':>7}  {'flips':>5}  "
          f"{'max|x|':>7}  verdict")
    for d0, x0 in starts:
        for seed in seeds:
            r = run_once("step", d0, x0, seed, t_max=60.0)
            ok = (r["gate_t"] is not None and r["flips"] < 6
                  and not r["blowup"])
            verdict = "PASS" if ok else "FAIL"
            if not ok:
                failures.append((d0, x0, seed, r))
            else:
                times.append(r["gate_t"])
            gate = f"{r['gate_t']:6.1f}s" if r["gate_t"] else "  none "
            print(f"{d0:5.0f} {x0:+6.0f} {seed:4d}   {gate}  "
                  f"{r['flips']:5d}  {r['max_abs_x']:7.1f}  {verdict}")

    blk = run_once("step", 320.0, 67.0, 3, t_max=75.0, blackout=(8.0, 11.0))
    blk_ok = blk["gate_t"] is not None
    gate_str = f"{blk['gate_t']:.1f}s" if blk_ok else "none"
    print(f"blackout 8-11s reacquire: {'PASS' if blk_ok else 'FAIL'} "
          f"(gate {gate_str})")
    if not blk_ok:
        failures.append(("blackout", 0, 3, blk))

    if times:
        times.sort()
        print(f"\nstep gate times: min {times[0]:.1f}s  "
              f"median {times[len(times) // 2]:.1f}s  max {times[-1]:.1f}s "
              f"over {len(times)} passing runs")
    print("\ninformational A/B (old crawl policy, same plant):")
    for d0, x0, seed in [(320.0, 67.0, 1), (320.0, 67.0, 2), (400.0, -120.0, 1)]:
        r = run_once("crawl", d0, x0, seed, t_max=60.0)
        gate = f"{r['gate_t']:5.1f}s" if r["gate_t"] else "no gate in 60s"
        print(f"  crawl d0={d0:.0f} x0={x0:+.0f} seed={seed}: {gate}, "
              f"max|x| {r['max_abs_x']:.0f} mm")

    if failures:
        print(f"\n{len(failures)} FAILURES")
        return 1
    print("\nALL STEP ACCEPTANCE RUNS PASSED")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", choices=("step", "crawl"), default=None,
                    help="run a single policy once instead of the matrix")
    ap.add_argument("--d0", type=float, default=320.0)
    ap.add_argument("--x0", type=float, default=67.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--blackout", action="store_true",
                    help="black out vision from t=8s to t=11s")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.policy is None:
        return acceptance()
    r = run_once(args.policy, args.d0, args.x0, args.seed,
                 blackout=(8.0, 11.0) if args.blackout else None,
                 verbose=args.verbose)
    print(r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
