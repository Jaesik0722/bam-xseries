# Copyright 2026 Jaesik Jeong
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Added to Better Actuator Models (Copyright 2025 Marc Duclusaud & Gregoire Passault)
# to support the DYNAMIXEL X series.

"""Compare a fitted X series model with the ROBOTIS catalogue (PROTOCOL.md section 8.1).

The catalogue quotes output-shaft figures, so the model's friction is included:

  breakaway torque  the largest load torque the motor can start to lift from rest at full
                    duty: kt*V/R minus the static friction budget at that load. This is not
                    what ROBOTIS calls stall torque (the maximum momentary torque), so the
                    comparison is coarse.
  no-load speed     the speed the unloaded output reaches from rest at full duty, simulated
                    with the fitted friction (bam.simulate's static-friction rule). With a
                    Stribeck term the torque balance can have several roots, and the one a
                    root search finds need not be the one the motor reaches; every root is
                    listed so that the case is visible.

kt*V/R and V/kt, the frictionless motor values, are printed alongside for reference. The
+-20 % comparison is the M1 criterion of the SIMPAT plan; a miss calls for a look at the fit,
not for discarding it on that ground alone.

    python -m bam.catalog_check --params params/xh540_A_m4.json --vin 14.8
"""

import argparse
import json

import numpy as np

from .model import load_model_from_dict

RPM = 2 * np.pi / 60

# ROBOTIS e-Manual, Specifications: (voltage V, stall torque N m, no-load speed rev/min).
# The e-Manual describes stall torque as the maximum momentary torque; its N-T curves,
# measured under a gradually increasing load, sit below it.
CATALOGUE = {
    "xh540": [(11.1, 9.2, 36), (12.0, 9.9, 39), (14.8, 11.7, 46)],
    "xm540": [(11.1, 10.0, 28), (12.0, 10.6, 30), (14.8, 12.9, 37)],
    "xm430": [(11.1, 3.8, 43), (12.0, 4.1, 46), (14.8, 4.8, 57)],
}


def catalogue_at(actuator: str, vin: float) -> tuple[float, float]:
    """Stall torque [N m] and no-load speed [rad/s], interpolated between listed voltages."""
    rows = CATALOGUE[actuator]
    volts = [r[0] for r in rows]
    if not volts[0] <= vin <= volts[-1]:
        raise SystemExit(f"--vin {vin} V is outside the catalogue range "
                         f"{volts[0]}-{volts[-1]} V (the input range ends at 14.8 V)")
    stall = np.interp(vin, volts, [r[1] for r in rows])
    speed = np.interp(vin, volts, [r[2] for r in rows]) * RPM
    return float(stall), float(speed)


def _bisect(f, lo, hi, iterations=100):
    """Root of a function that is positive at lo and negative at hi."""
    for _ in range(iterations):
        mid = (lo + hi) / 2
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def predicted_stall(model, kt: float, R: float, vin: float) -> float:
    """Largest opposing load torque that full duty still breaks away from rest."""
    tau_m = kt * vin / R

    def margin(load):
        # BAM's external torque has the sign of the joint, so a load opposing the motor is
        # negative; the friction budget at rest is frictionloss alone.
        frictionloss, _ = model.compute_frictions(tau_m, -load, 0.0)
        return tau_m - load - frictionloss

    if margin(0.0) <= 0:
        return 0.0
    return _bisect(margin, 0.0, tau_m)


def noload_margin(model, kt, R, vin, speed):
    """Motor torque left after friction at full duty, no load and this speed."""
    tau_m = kt * (vin - kt * speed) / R
    frictionloss, damping = model.compute_frictions(tau_m, 0.0, speed)
    return tau_m - frictionloss - damping * speed


def noload_equilibria(model, kt: float, R: float, vin: float, samples=4000) -> list:
    """Every speed in (0, V/kt) where the torque balance crosses zero."""
    speeds = np.linspace(0.0, vin / kt, samples)
    margins = [noload_margin(model, kt, R, vin, w) for w in speeds]
    roots = []
    for k in range(1, samples):
        if (margins[k - 1] > 0) != (margins[k] > 0):
            roots.append(_bisect(lambda w: noload_margin(model, kt, R, vin, w),
                                 speeds[k - 1], speeds[k])
                         if margins[k - 1] > 0 else
                         _bisect(lambda w: -noload_margin(model, kt, R, vin, w),
                                 speeds[k - 1], speeds[k]))
    return roots


def noload_timestep(model, kt: float, R: float, vin: float, armature: float,
                    dt_max=1e-4, fraction=0.1, samples=200) -> float:
    """Timestep for the no-load simulation, a fraction of the mechanical time constant.

    The unloaded output has only the armature as inertia, so with a small armature the
    time constant armature / (kt^2/R + viscous) can be shorter than a fixed timestep. The
    explicit update is unstable beyond twice the time constant: the speed then flips
    between two values from one step to the next instead of settling.
    """
    damping = 0.0
    for speed in np.linspace(0.0, vin / kt, samples):
        tau_m = kt * (vin - kt * speed) / R
        damping = max(damping, model.compute_frictions(tau_m, 0.0, speed)[1])
    return min(dt_max, fraction * armature / (kt**2 / R + damping))


def predicted_noload_speed(model, kt: float, R: float, vin: float, armature: float,
                           dt=None, window=0.5, max_time=60.0,
                           rtol=1e-4, torque_tol=1e-3) -> tuple[float, bool]:
    """Speed reached from rest at full duty with no load (bam.simulate's friction rule).

    Integrates until, over a whole `window` seconds, the speed stays within `rtol`
    (relative) of its mean and the torque balance at that speed is within `torque_tol`
    of kt*V/R; up to `max_time`. Checking every sample of the window, not only its ends,
    keeps an oscillation from passing as a steady state. The default timestep follows
    `noload_timestep`. Returns (speed, converged).
    """
    if dt is None:
        dt = noload_timestep(model, kt, R, vin, armature)
    tolerance = torque_tol * kt * vin / R
    speed = 0.0
    low, high = np.inf, -np.inf
    steps_per_window = max(1, int(round(window / dt)))
    for step in range(1, int(max_time / dt) + 1):
        tau_m = kt * (vin - kt * speed) / R
        frictionloss, damping = model.compute_frictions(tau_m, 0.0, speed)
        tau_stop = (armature / dt) * speed + tau_m
        friction = -np.sign(tau_stop) * min(abs(tau_stop),
                                            frictionloss + damping * abs(speed))
        speed += (tau_m + friction) / armature * dt
        low, high = min(low, speed), max(high, speed)
        if step % steps_per_window == 0:
            steady = high - low <= rtol * max(abs(speed), 1e-9)
            residual = noload_margin(model, kt, R, vin, speed)
            # At rest, static friction may hold the output: a deficit is then balanced.
            balanced = abs(residual) <= tolerance or (speed == 0.0 and residual <= 0.0)
            if steady and balanced:
                return float(speed), True
            low, high = np.inf, -np.inf
    return float(speed), False


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--params", required=True, help="fitted JSON from bam.fit")
    parser.add_argument("--vin", type=float, required=True,
                        help="supply voltage to compare at (the one identified with)")
    parser.add_argument("--tolerance", type=float, default=0.20,
                        help="relative; 20 %% is the criterion in the SIMPAT plan (M1)")
    args = parser.parse_args()

    with open(args.params) as f:
        data = json.load(f)
    actuator = data["actuator"]
    if actuator not in CATALOGUE:
        raise SystemExit(f"no catalogue figures for actuator {actuator!r}")
    model = load_model_from_dict(data)
    kt, R = data["kt"], data["R"]

    stall_cat, speed_cat = catalogue_at(actuator, args.vin)
    stall = predicted_stall(model, kt, R, args.vin)
    armature = model.actuator.get_extra_inertia()
    speed, converged = predicted_noload_speed(model, kt, R, args.vin, armature)
    roots = noload_equilibria(model, kt, R, args.vin)

    print(f"{args.params}: {actuator} {data['model']}, kt {kt:.3f}, R {R:.3f}, "
          f"at {args.vin} V\n")
    print(f"{'':16}{'model':>10}{'catalogue':>11}{'diff':>9}{'frictionless':>15}")
    failed = False
    for name, value, reference, ideal, unit in [
        ("breakaway/stall", stall, stall_cat, kt * args.vin / R, "N m"),
        ("no-load speed", speed, speed_cat, args.vin / kt, "rad/s"),
    ]:
        diff = value / reference - 1
        mark = "" if abs(diff) <= args.tolerance else "  ! outside tolerance"
        failed |= bool(mark)
        print(f"{name:16}{value:10.3f}{reference:11.3f}{100 * diff:+8.1f}%"
              f"{ideal:15.3f}  {unit}{mark}")
    if not converged:
        print("\nnote: the no-load simulation had not settled after 60 s (speed still "
              "changing within the last 0.5 s, or torque balance not met); the speed above "
              "is not a steady state")
        failed = True
    if len(roots) > 1:
        print("\nnote: the no-load torque balance has several roots, "
              + ", ".join(f"{w:.3f}" for w in roots)
              + " rad/s; the speed above is the one reached from rest")
    print(f"\nM1 criterion +-{100 * args.tolerance:.0f} % against the catalogue: "
          f"{'outside' if failed else 'inside'} (see PROTOCOL.md 8.1 before acting on it)")


if __name__ == "__main__":
    main()
