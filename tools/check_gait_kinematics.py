"""Offline, deterministic check of trot_gait.cpp's velocity decomposition.

WHY THIS EXISTS
---------------
trot_gait.cpp is not exercised by the current simulation: with
use_planar_move:=true the base is driven by a Gazebo plugin and no
gait_controller node runs at all. So a bug in the gait is invisible to every
end-to-end test in this repo -- it will only surface when someone flips that
flag. This tool checks the kinematics on paper, where it is cheap to check.

THE RIGID-BODY RELATION
-----------------------
Let the body frame move with velocity (vx, vy) and yaw rate w, both expressed
in body axes. A foot planted at body coordinates (x_i, y_i) has world velocity

    V_i = (vx - w*y_i, vy + w*x_i)

and for it to stay planted that must be zero, so the foot's velocity *in the
body frame* is the negative of that. generateTrajectory moves the foot by
(-step_x, -step_y) over a stance lasting Ts = period * duty_factor, hence

    step_x_i / Ts = vx - w*y_i
    step_y_i / Ts = vy + w*x_i

Two consequences the current code violates:

  1. step_x must depend on y_i (LEFT/RIGHT), because rotation's fore-aft
     component comes from -w*y_i. The code sets step_x = vx*period, with no
     leg dependence at all, so rotation has no longitudinal component.
  2. step_y must depend on x_i (FRONT/REAR) for the rotational part, but must
     be UNIFORM across legs for the translational part vy. The code flips the
     sign of BOTH terms by left/right, so a vy command produces yaw and no
     lateral motion at all.

CHECK PERFORMED
---------------
For each instant, take the legs currently in stance, and solve the 4+ equations
above for the 3 unknowns (vx, vy, w) by least squares. The residual is the
answer: if it is non-zero the legs in stance disagree about how the body is
moving, i.e. they are fighting each other. Then average the solution over a
full cycle and compare with what was commanded.

Usage:
    python3 tools/check_gait_kinematics.py [--mode current|rigid]
"""

import math
import sys

# Body geometry, from puppy.urdf.xacro:
#   x = +/-(body_length/2 - 0.02)   = +/-0.130 m  (front / rear)
#   y = +/-(body_width/2 + hip_length/2) = +/-0.1125 m  (left / right)
HALF_LENGTH = 0.130
HALF_WIDTH = 0.1125

# (x_i, y_i) per leg, body frame. REP-103: +x forward, +y left.
LEGS = {
    "FR": (+HALF_LENGTH, -HALF_WIDTH),
    "FL": (+HALF_LENGTH, +HALF_WIDTH),
    "RR": (-HALF_LENGTH, -HALF_WIDTH),
    "RL": (-HALF_LENGTH, +HALF_WIDTH),
}

# Trot: diagonals in phase (FR+RL, FL+RR), second pair offset by half a cycle.
PHASE_OFFSET = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}

PERIOD = 0.5
DUTY = 0.5


def steps_legacy(leg, vx, vy, wz, period=PERIOD, duty=DUTY):
    """Mirror of trot_gait.cpp::getFootPosition BEFORE the rigid-body fix.

    Kept so the regression can be demonstrated rather than merely claimed:
    --mode legacy reproduces 5/7 inconsistent commands, --mode rigid gives 0.
    """
    step_x = vx * period
    is_right = leg in ("FR", "RR")
    rot_step = wz * period * 0.1
    if is_right:
        step_y = -vy * period - rot_step
    else:
        step_y = vy * period + rot_step
    return step_x, step_y


def steps_rigid(leg, vx, vy, wz, period=PERIOD, duty=DUTY):
    """Correct rigid-body decomposition."""
    x_i, y_i = LEGS[leg]
    ts = period * duty
    step_x = (vx - wz * y_i) * ts
    step_y = (vy + wz * x_i) * ts
    return step_x, step_y


def solve_ls(rows, rhs):
    """Least squares for a 3-column system, via normal equations."""
    n = 3
    ata = [[0.0] * n for _ in range(n)]
    atb = [0.0] * n
    for r, b in zip(rows, rhs):
        for i in range(n):
            atb[i] += r[i] * b
            for j in range(n):
                ata[i][j] += r[i] * r[j]
    # Gaussian elimination with partial pivoting.
    for c in range(n):
        piv = max(range(c, n), key=lambda k: abs(ata[k][c]))
        if abs(ata[piv][c]) < 1e-12:
            return None
        ata[c], ata[piv] = ata[piv], ata[c]
        atb[c], atb[piv] = atb[piv], atb[c]
        for k in range(c + 1, n):
            f = ata[k][c] / ata[c][c]
            for j in range(c, n):
                ata[k][j] -= f * ata[c][j]
            atb[k] -= f * atb[c]
    u = [0.0] * n
    for i in reversed(range(n)):
        s = atb[i] - sum(ata[i][j] * u[j] for j in range(i + 1, n))
        u[i] = s / ata[i][i]
    return u


def residuals(u, rows, rhs):
    return [sum(r[k] * u[k] for k in range(3)) - b for r, b in zip(rows, rhs)]


def analyse(vx, vy, wz, mode, period=PERIOD, duty=DUTY, samples=200):
    step_fn = steps_legacy if mode == "legacy" else steps_rigid
    ts = period * duty

    steps = {}
    for leg in LEGS:
        sx, sy = step_fn(leg, vx, vy, wz, period, duty)
        steps[leg] = (sx / ts, sy / ts)  # a_i, b_i

    worst_res = 0.0
    worst_phase = None
    acc = [0.0, 0.0, 0.0]
    counted = 0

    for s in range(samples):
        phase = s / samples
        rows, rhs = [], []
        for leg, (a, b) in steps.items():
            leg_phase = (phase + PHASE_OFFSET[leg]) % 1.0
            if leg_phase >= duty:
                continue  # swing phase: foot in the air, drives nothing
            x_i, y_i = LEGS[leg]
            rows.append([1.0, 0.0, -y_i])
            rhs.append(a)
            rows.append([0.0, 1.0, +x_i])
            rhs.append(b)
        if len(rows) < 3:
            continue
        u = solve_ls(rows, rhs)
        if u is None:
            continue
        res = max(abs(v) for v in residuals(u, rows, rhs))
        if res > worst_res:
            worst_res = res
            worst_phase = phase
        for k in range(3):
            acc[k] += u[k]
        counted += 1

    if counted == 0:
        return None
    mean = [a / counted for a in acc]
    return {"mode": mode, "worst_residual": worst_res,
            "worst_phase": worst_phase, "mean": mean,
            "cmd": (vx, vy, wz)}


CASES = [
    ("forward   vx=+0.1", 0.1, 0.0, 0.0),
    ("backward  vx=-0.1", -0.1, 0.0, 0.0),
    ("strafe +y vy=+0.1", 0.0, 0.1, 0.0),
    ("strafe -y vy=-0.1", 0.0, -0.1, 0.0),
    ("turn CCW  wz=+0.3", 0.0, 0.0, 0.3),
    ("turn CW   wz=-0.3", 0.0, 0.0, -0.3),
    ("arc  vx=0.1 wz=0.3", 0.1, 0.0, 0.3),
]


def main():
    mode = "rigid"
    if "--mode" in sys.argv:
        mode = sys.argv[sys.argv.index("--mode") + 1]
    if mode not in ("legacy", "rigid"):
        print("mode must be 'legacy' (pre-fix behaviour) or 'rigid' (current code)")
        return 2

    print("trot_gait decomposition: mode=%s" % mode)
    print("period=%.2fs duty=%.2f  half_length=%.4fm half_width=%.4fm"
          % (PERIOD, DUTY, HALF_LENGTH, HALF_WIDTH))
    print()
    print("%-20s %11s %11s %11s %11s" %
          ("command", "residual", "mean_vx", "mean_vy", "mean_wz"))
    print("-" * 68)

    failures = 0
    for label, vx, vy, wz in CASES:
        r = analyse(vx, vy, wz, mode)
        if r is None:
            print("%-20s  (no stance legs -- check duty_factor)" % label)
            continue
        mvx, mvy, mwz = r["mean"]
        bad = r["worst_residual"] > 1e-6
        if bad:
            failures += 1
        print("%-20s %11.2e %11.4f %11.4f %11.4f%s" %
              (label, r["worst_residual"], mvx, mvy, mwz,
               "   <-- LEGS FIGHT" if bad else ""))

    print()
    print("residual = disagreement between legs that are in stance together.")
    print("           Non-zero means no rigid-body motion satisfies them; the")
    print("           pair scissors against itself and the command is lost.")
    print()
    print("%-20s %11s %11s %11s" % ("command", "vx ratio", "vy ratio", "wz ratio"))
    print("-" * 60)
    for label, vx, vy, wz in CASES:
        r = analyse(vx, vy, wz, mode)
        if r is None:
            continue
        mvx, mvy, mwz = r["mean"]

        def ratio(a, b):
            return ("%10.2f" % (a / b)) if abs(b) > 1e-9 else "         -"

        print("%-20s %11s %11s %11s"
              % (label, ratio(mvx, vx), ratio(mvy, vy), ratio(mwz, wz)))

    print()
    if failures:
        print("VERDICT: %d/%d commands are NOT rigid-body consistent."
              % (failures, len(CASES)))
        return 1
    print("VERDICT: every command decomposes into a consistent rigid motion.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
