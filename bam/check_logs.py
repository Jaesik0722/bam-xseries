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

"""Check identification logs before fitting (PROTOCOL.md section 6).

Works on raw logs from bam.dynamixel.record and on logs resampled by bam.process. It only
reads files and needs no hardware.

    python -m bam.check_logs --logdir logs_raw/xh540 --csv checks_xh540.csv

Two kinds of output. A **flag** is a fault in the recording itself (aborted, supply, heat,
no motion, a zero beyond what the fit can absorb, a register the model does not describe,
sampling gaps): such a log is moved aside before fitting. A **note** is diagnostic and does
not by itself exclude a log; the free-swing and PWM comparisons are notes, because an
approximate formula disagreeing with a log does not show that the log is wrong.

Per log it looks at:
  * supply:      --vin against the median measured input voltage, and the sag under load
  * temperature: maximum temperature against the 70 degC limit
  * motion:      the arm moved (all trajectories except `nothing`), or stayed still (`nothing`)
  * zero:        where the arm rests with torque off (`nothing`), against the +-0.1 rad that
                 the fitted q_offset can absorb
  * free swing:  the period after the `lift_and_drop` release, corrected for amplitude, and
                 the armature it implies once the geometry is subtracted
  * PWM gain:    the duty the firmware applied against error_gain * kp * position error, for
                 logs that carry the `pwm` channel
  * registers:   the register snapshot, when the log has one, against what the model assumes
  * sampling:    mean and worst sample interval
and across logs it lists missing and duplicated (configuration, kp, trajectory) cells, and
compares the free-swing armature between load configurations.

The free swing is not a check on mass and length alone: with torque off the rotor and gears
still turn with the arm, so the reflected inertia (armature) is part of the period, and the
load-dependent friction of m4 changes it too. Two comparisons are made:

  * before a fit, the closed-form pendulum period with the median armature the unit's
    configurations imply, which finds configurations that disagree with the others;
  * after a fit, with --params on logs resampled by bam.process, the period of the same
    swing simulated with the whole fitted model (bam.simulate, friction included). This is
    the M1 criterion of the SIMPAT plan: within 3 %.
"""

import argparse
import csv
import glob
import json
import os
from collections import Counter

import numpy as np

from .dynamixel.xseries_actuator import X_ERROR_GAIN

X_SERIES = ("xh540", "xm540", "xm430")
G = 9.80665  # same as bam.testbench
TRAJECTORIES = ["sin_time_square", "sin_sin", "lift_and_drop", "up_and_down",
                "half_sine", "steps", "nothing"]


def pendulum_terms(mass: float, arm_mass: float, length: float) -> tuple[float, float]:
    """Inertia about the pivot and gravity stiffness of BAM's pendulum (bam.testbench)."""
    inertia = mass * length**2 + arm_mass * length**2 / 3
    stiffness = (mass * length + arm_mass * length / 2) * G
    return inertia, stiffness


def pendulum_period(mass: float, arm_mass: float, length: float,
                    armature: float = 0.0, viscous: float = 0.0) -> float:
    """Small-swing period with the reflected actuator inertia included (PROTOCOL.md 6.4).

    With `viscous` [N m s/rad] it is the damped period, longer by 1/sqrt(1 - zeta^2).
    """
    inertia, stiffness = pendulum_terms(mass, arm_mass, length)
    inertia += armature
    omega2 = stiffness / inertia - (viscous / (2 * inertia))**2
    return 2 * np.pi / np.sqrt(omega2) if omega2 > 0 else np.inf


def implied_armature(period: float, mass: float, arm_mass: float, length: float) -> float:
    """The armature that makes the pendulum swing with this small-swing period."""
    inertia, stiffness = pendulum_terms(mass, arm_mass, length)
    return period**2 * stiffness / (4 * np.pi**2) - inertia


def amplitude_factor(amplitude):
    """Exact pendulum period over the small-swing period: (2/pi) K(sin(theta/2)).

    K is the complete elliptic integral of the first kind, computed with the
    arithmetic-geometric mean. The factor is 1.180 at 90 degrees, the value PROTOCOL.md
    quotes for the release.
    """
    k = np.sin(np.asarray(amplitude, dtype=float) / 2)
    a, b = np.ones_like(k), np.sqrt(1 - k**2)
    for _ in range(20):
        a, b = (a + b) / 2, np.sqrt(a * b)
    return 1 / a


def free_swing(t, q, torque, max_amplitude, hysteresis=0.01, min_amplitude=0.03):
    """Time each free-swing cycle after the last torque release.

    Cycles run between upward crossings of the swing centre. Returns (periods, amplitudes)
    for the cycles whose amplitude lies in [min_amplitude, max_amplitude], or None if
    there is none.
    """
    on = np.flatnonzero(torque)
    start = on[-1] + 1 if len(on) else 0
    t, q = t[start:], q[start:]
    if len(t) < 10:
        return None
    # The arm hangs at zero up to the small q_offset; take the centre of the last second
    # of motion so that offset does not skew the crossing times.
    tail = q[t >= t[-1] - 1.0]
    x = q - (np.max(tail) + np.min(tail)) / 2

    # Upward crossings with hysteresis, so encoder noise near rest does not count.
    crossings, armed = [], False
    for k in range(1, len(x)):
        if x[k] < -hysteresis:
            armed = True
        elif armed and x[k - 1] < 0 <= x[k]:
            frac = -x[k - 1] / (x[k] - x[k - 1])
            crossings.append(t[k - 1] + frac * (t[k] - t[k - 1]))
            armed = False

    periods, amplitudes = [], []
    for a, b in zip(crossings[:-1], crossings[1:]):
        window = x[(t >= a) & (t <= b)]
        amplitude = (np.max(window) - np.min(window)) / 2
        if min_amplitude <= amplitude <= max_amplitude:
            periods.append(b - a)
            amplitudes.append(amplitude)
    if not periods:
        return None
    return np.array(periods), np.array(amplitudes)


def check_log(path: str, args, model=None) -> dict:
    with open(path) as f:
        log = json.load(f)
    entries = log["entries"]
    if len(entries) < 2:
        reason = log.get("aborted", "no samples")
        return {"file": os.path.relpath(path, args.logdir), "motor": log.get("motor"),
                "unit": log.get("unit", ""), "mass": log.get("mass"),
                "length": log.get("length"), "kp": log.get("kp"),
                "trajectory": log.get("trajectory"), "samples": len(entries),
                "flags": f"recording aborted before any data, do not fit ({reason})",
                "notes": ""}
    if "timestamp" in entries[0]:
        t = np.array([e["timestamp"] for e in entries])
    else:
        t = np.arange(len(entries)) * log["dt"]
    q = np.array([e["position"] for e in entries])
    torque = np.array([bool(e["torque_enable"]) for e in entries])
    volts = np.array([e.get("input_volts", np.nan) for e in entries], dtype=float)
    temps = np.array([e.get("temp", np.nan) for e in entries], dtype=float)

    arm_mass = log.get("arm-mass", log.get("arm_mass"))
    row = {
        "file": os.path.relpath(path, args.logdir),
        "motor": log.get("motor"),
        "unit": log.get("unit", ""),
        "mass": log.get("mass"),
        "arm_mass": arm_mass,
        "length": log.get("length"),
        "kp": log.get("kp"),
        "trajectory": log.get("trajectory"),
        "vin": log.get("vin"),
        "samples": len(entries),
    }
    flags, notes = [], []
    if "aborted" in log:
        flags.append(f"recording aborted, partial log, do not fit ({log['aborted']})")
    if "shutdown_errors" in log:
        notes.append("shutdown failed after recording (data complete): "
                     + ", ".join(log["shutdown_errors"]))

    dt = np.diff(t)
    row["dt_mean_ms"] = round(1e3 * float(np.mean(dt)), 2)
    row["dt_max_ms"] = round(1e3 * float(np.max(dt)), 2)
    if row["dt_max_ms"] > args.max_dt_ms:
        flags.append(f"sample gap {row['dt_max_ms']} ms")

    if np.isfinite(volts).any():
        v_med = float(np.nanmedian(volts))
        row["volts_median"] = round(v_med, 2)
        if row["vin"] is not None and abs(v_med - row["vin"]) > args.vin_tolerance:
            flags.append(f"--vin {row['vin']} V vs measured {v_med:.2f} V")
        loaded = volts[torque] if torque.any() else volts
        sag = float(np.nanmedian(volts) - np.nanmin(loaded))
        row["volts_sag"] = round(sag, 2)
        if sag > args.sag_tolerance:
            flags.append(f"supply sag {sag:.2f} V")

    if np.isfinite(temps).any():
        row["temp_start"] = float(temps[0])
        row["temp_max"] = float(np.nanmax(temps))
        if row["temp_max"] > args.max_temp:
            flags.append(f"temperature {row['temp_max']:.0f} degC")

    motion = float(np.ptp(q))
    row["motion_rad"] = round(motion, 3)
    if row["trajectory"] == "nothing":
        if motion > args.still_tolerance:
            flags.append(f"moved {motion:.3f} rad with torque off (something loose?)")
    elif motion < args.min_motion:
        flags.append(f"barely moved ({motion:.3f} rad)")

    if row["trajectory"] == "nothing":
        rest = float(np.median(q))
        row["rest_rad"] = round(rest, 4)
        if abs(rest) > args.zero_tolerance:
            flags.append(f"arm rests at {rest:+.3f} rad with torque off, and the fitted "
                         "q_offset only spans +-0.1 rad (re-seat the horn, PROTOCOL.md 5.4)")

    if row["trajectory"] == "lift_and_drop" and None not in (row["mass"], arm_mass,
                                                              row["length"]):
        swing = free_swing(t, q, torque, max_amplitude=np.radians(args.max_swing_deg))
        if swing is None:
            notes.append("free swing: no usable cycle (damped out or did not drop)")
        else:
            periods, amplitudes = swing
            # Each cycle reduced to its small-swing equivalent. Armature only adds inertia,
            # so the same amplitude correction holds with it.
            small = float(np.median(periods / amplitude_factor(amplitudes)))
            row["period_measured_s"] = round(float(np.median(periods)), 4)
            row["period_small_s"] = round(small, 4)
            row["period_cycles"] = len(periods)
            row["period_no_armature_s"] = round(
                pendulum_period(row["mass"], arm_mass, row["length"]), 4)
            row["armature_implied"] = round(
                implied_armature(small, row["mass"], arm_mass, row["length"]), 5)
            if row["armature_implied"] < 0:
                notes.append("free swing faster than the closed-form pendulum allows even "
                             "with zero armature (check --mass, --arm-mass, --length)")
            if model is not None:
                model_swing_check(log, path, row, notes, model, args)

    pwm = np.array([e.get("pwm", np.nan) for e in entries], dtype=float)
    goal = np.array([e.get("goal_position", np.nan) for e in entries], dtype=float)
    if np.isfinite(pwm).any() and row["kp"] and row["motor"] in X_SERIES:
        predicted = X_ERROR_GAIN * row["kp"] * (goal - q)
        # Unsaturated samples with torque on and an error well above encoder resolution.
        use = (torque & np.isfinite(pwm) & np.isfinite(predicted)
               & (np.abs(predicted) > 0.03) & (np.abs(predicted) < 0.9)
               & (np.abs(pwm) < 0.95))
        row["pwm_samples"] = int(use.sum())
        if use.sum() >= args.min_pwm_samples:
            ratio = float(np.sum(pwm[use] * predicted[use]) / np.sum(predicted[use]**2))
            row["pwm_gain_ratio"] = round(ratio, 3)
            if not 1 / args.pwm_ratio_tolerance <= ratio <= args.pwm_ratio_tolerance:
                notes.append(f"applied PWM is {ratio:.2f} x error_gain * kp * error "
                             "(PROTOCOL.md 9)")

    if "registers" in log:
        problems = log.get("register_problems")
        if problems is None:
            try:
                from .dynamixel.xseries import check_registers
                problems = check_registers(log["registers"])
            except ImportError:  # dynamixel_sdk is not needed just to read logs
                problems = ["snapshot present but not checked (dynamixel_sdk missing)"]
        flags.extend(f"register {p}" for p in problems)
    elif row["motor"] in X_SERIES:
        row["unit"] = row["unit"] or "?"
        notes.append("no register snapshot (recorded before it was added)")

    row["flags"] = "; ".join(flags)
    row["notes"] = "; ".join(notes)
    return row


def model_swing_check(log, path, row, notes, model, args):
    """Free-swing period of the fitted model's simulation of this log against the log."""
    if "dt" not in log:
        notes.append("model check needs logs resampled by bam.process")
        return
    from .simulate import Simulator

    sim_log = dict(log)
    sim_log["arm_mass"] = log.get("arm_mass", log.get("arm-mass"))
    positions, _, _ = Simulator(model).rollout_log(sim_log, simulate_control=True)
    entries = log["entries"]
    t = np.arange(len(entries)) * log["dt"]
    torque = np.array([bool(e["torque_enable"]) for e in entries])
    q_sim = np.asarray(positions, dtype=float)
    swing = free_swing(t, q_sim, torque, max_amplitude=np.radians(args.max_swing_deg))
    if swing is None:
        notes.append("model check: the simulated swing has no usable cycle")
        return
    periods, amplitudes = swing
    simulated = float(np.median(periods / amplitude_factor(amplitudes)))
    error = row["period_small_s"] / simulated - 1
    row["period_model_s"] = round(simulated, 4)
    row["period_model_error"] = round(error, 4)
    if abs(error) > args.period_tolerance:
        notes.append(f"free-swing period {100 * error:+.1f} % from the fitted model's "
                     f"simulation (M1 criterion {100 * args.period_tolerance:.0f} %)")


def grid_report(rows):
    """Missing and duplicated (motor, unit, mass, length, kp, trajectory) cells."""
    lines = []
    cell = lambda r: (r["motor"], r["unit"], r["mass"], r["length"], r["kp"], r["trajectory"])
    counts = Counter(cell(r) for r in rows)
    for key, n in sorted(counts.items(), key=str):
        if n > 1:
            lines.append(f"duplicate x{n}: {key}")

    groups = {}
    for r in rows:
        groups.setdefault((r["motor"], r["unit"]), []).append(r)
    for (motor, unit), group in groups.items():
        configs = sorted({(r["mass"], r["length"]) for r in group})
        kps = sorted({r["kp"] for r in group})
        trajs = [t for t in TRAJECTORIES if any(r["trajectory"] == t for r in group)]
        trajs += sorted({r["trajectory"] for r in group} - set(TRAJECTORIES))
        lines.append(f"{motor} {unit or ''}: {len(configs)} load configurations, "
                     f"kp {kps}, {len(trajs)} trajectories, {len(group)} logs")
        missing_traj = [t for t in TRAJECTORIES if t not in trajs]
        if missing_traj:
            lines.append(f"  trajectories never recorded: {missing_traj}")
        for mass, length in configs:
            for kp in kps:
                for traj in trajs:
                    if counts[(motor, unit, mass, length, kp, traj)] == 0:
                        lines.append(f"  missing: mass {mass} length {length} kp {kp} {traj}")
    return lines


def swing_report(rows, armature, tolerance, min_cycles=2, viscous=0.0):
    """Compare the free-swing periods of each unit against one armature value.

    The reference is --armature when given (after a fit) and otherwise the median of the
    implied armatures over the unit's load configurations. Notes are added to the rows.
    """
    lines = []
    groups = {}
    for r in rows:
        if "armature_implied" not in r:
            continue
        if r["period_cycles"] < min_cycles:
            # Near the end of the swing static friction distorts the last cycle, and one
            # cycle alone cannot show that.
            note = f"free swing: only {r['period_cycles']} usable cycle, not judged"
            r["notes"] = "; ".join(filter(None, [r.get("notes"), note]))
            continue
        groups.setdefault((r["motor"], r["unit"]), []).append(r)
    for (motor, unit), group in groups.items():
        implied = np.array([r["armature_implied"] for r in group])
        configs = {(r["mass"], r["length"]) for r in group}
        reference = armature if armature is not None else float(np.median(implied))
        source = "--armature" if armature is not None else "median over configurations"
        lines.append(f"{motor} {unit or ''}: free-swing armature over {len(group)} logs, "
                     f"{len(configs)} configurations: median {np.median(implied):.4f}, "
                     f"range {implied.min():.4f} to {implied.max():.4f} kg m^2")
        errors = []
        for r in group:
            expected = pendulum_period(r["mass"], r["arm_mass"], r["length"], reference,
                                       viscous)
            error = r["period_small_s"] / expected - 1
            r["period_reference_s"] = round(expected, 4)
            r["period_error"] = round(error, 4)
            errors.append(error)
            if abs(error) > tolerance:
                note = (f"free-swing period {100 * error:+.1f} % from the closed-form one "
                        f"with armature {reference:.4f} ({source})")
                r["notes"] = "; ".join(filter(None, [r.get("notes"), note]))
        errors = np.array(errors)
        worst = errors[np.argmax(np.abs(errors))]
        lines.append(f"  closed-form period with armature {reference:.4f} ({source}): "
                     f"median {100 * np.median(errors):+.1f} %, worst {100 * worst:+.1f} %"
                     " (reference only; friction not included)")
        modelled = np.array([r["period_model_error"] for r in group
                             if "period_model_error" in r])
        if len(modelled):
            inside = int(np.sum(np.abs(modelled) <= tolerance))
            worst = modelled[np.argmax(np.abs(modelled))]
            lines.append(f"  fitted model's simulated swing: median "
                         f"{100 * np.median(modelled):+.1f} %, worst {100 * worst:+.1f} %, "
                         f"{inside}/{len(modelled)} within {100 * tolerance:.0f} % (M1)")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--logdir", required=True, help="searched recursively for *.json")
    parser.add_argument("--csv", default=None, help="write one row per log")
    parser.add_argument("--vin-tolerance", type=float, default=0.3, help="V")
    parser.add_argument("--sag-tolerance", type=float, default=0.3, help="V")
    parser.add_argument("--max-temp", type=float, default=70.0, help="degC")
    parser.add_argument("--min-motion", type=float, default=0.05, help="rad")
    parser.add_argument("--still-tolerance", type=float, default=0.02, help="rad")
    parser.add_argument("--period-tolerance", type=float, default=0.03,
                        help="relative; 3 %% is the criterion in the SIMPAT plan (M1)")
    parser.add_argument("--params", default=None,
                        help="fitted JSON from bam.fit: simulate each lift_and_drop log "
                             "with the whole model (needs bam.process output)")
    parser.add_argument("--armature", type=float, default=None,
                        help="armature [kg m^2] for the closed-form period; default: "
                             "the fitted value with --params, else the median the logs imply")
    parser.add_argument("--viscous", type=float, default=0.0,
                        help="friction_viscous [N m s/rad] for the closed-form period")
    parser.add_argument("--min-swing-cycles", type=int, default=2)
    parser.add_argument("--zero-tolerance", type=float, default=0.05,
                        help="rad; the fitted q_offset is bounded to +-0.1")
    parser.add_argument("--pwm-ratio-tolerance", type=float, default=1.25,
                        help="flag a PWM gain ratio outside [1/x, x]; a divisor of 256 "
                             "instead of 128 would show as 0.5")
    parser.add_argument("--min-pwm-samples", type=int, default=100)
    parser.add_argument("--max-swing-deg", type=float, default=80.0,
                        help="only cycles below this amplitude are timed (each is "
                             "corrected for amplitude exactly)")
    parser.add_argument("--max-dt-ms", type=float, default=20.0)
    args = parser.parse_args()

    model = None
    if args.params:
        from .model import load_model

        model = load_model(args.params)
        if args.armature is None:
            args.armature = model.actuator.get_extra_inertia()
            args.viscous = model.friction_viscous.value

    paths = sorted(glob.glob(os.path.join(args.logdir, "**", "*.json"), recursive=True))
    rows = []
    for path in paths:
        try:
            rows.append(check_log(path, args, model))
        except (KeyError, IndexError, ValueError, json.JSONDecodeError) as e:
            rows.append({"file": os.path.relpath(path, args.logdir),
                         "flags": f"unreadable: {type(e).__name__}: {e}"})
    if not rows:
        raise SystemExit(f"no *.json under {args.logdir}")

    # Logs with no data are listed with their flag but take no part in the grid.
    readable = [r for r in rows if "motor" in r and r.get("samples", 2) >= 2]
    swing_lines = swing_report(readable, args.armature, args.period_tolerance,
                               args.min_swing_cycles, args.viscous)

    flagged = [r for r in rows if r.get("flags")]
    noted = [r for r in rows if r.get("notes")]
    print(f"{len(rows)} logs, {len(flagged)} flagged (move aside before fitting), "
          f"{len(noted)} with notes (diagnostic)\n")
    for r in rows:
        if not (r.get("flags") or r.get("notes")):
            continue
        print(f"{r['file']}  [{r.get('kp')} {r.get('trajectory')}]")
        for flag in filter(None, r.get("flags", "").split("; ")):
            print(f"    ! {flag}")
        for note in filter(None, r.get("notes", "").split("; ")):
            print(f"    - {note}")

    if swing_lines:
        print()
        for line in swing_lines:
            print(line)

    ratios = {}
    for r in readable:
        if "pwm_gain_ratio" in r:
            ratios.setdefault((r["motor"], r["unit"], r["kp"]), []).append(r["pwm_gain_ratio"])
    if ratios:
        print("\napplied PWM / (error_gain * kp * error), median per P gain "
              "(1.0 = model's error_gain, 0.5 = divisor 256):")
        for (motor, unit, kp), values in sorted(ratios.items(), key=str):
            print(f"  {motor} {unit or ''} kp {kp}: {np.median(values):.3f} "
                  f"({len(values)} logs)")

    print()
    for line in grid_report(readable):
        print(line)

    if args.csv:
        keys = []
        for r in rows:
            keys += [k for k in r if k not in keys]
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nwrote {args.csv}")


if __name__ == "__main__":
    main()
