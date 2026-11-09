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

Per log it checks:
  * supply:      --vin against the median measured input voltage, and the sag under load
  * temperature: maximum temperature against the 70 degC limit
  * motion:      the arm moved (all trajectories except `nothing`), or stayed still (`nothing`)
  * free swing:  the period after the `lift_and_drop` release against the pendulum geometry,
                 with each cycle corrected for its amplitude
  * registers:   the register snapshot, when the log has one, against what the model assumes
  * sampling:    mean and worst sample interval
and across logs it lists missing and duplicated (configuration, kp, trajectory) cells.
"""

import argparse
import csv
import glob
import json
import os
from collections import Counter

import numpy as np

G = 9.81
TRAJECTORIES = ["sin_time_square", "sin_sin", "lift_and_drop", "up_and_down",
                "half_sine", "steps", "nothing"]


def pendulum_period(mass: float, arm_mass: float, length: float) -> float:
    """Small-swing period of BAM's pendulum: point mass plus uniform rod (PROTOCOL.md 6.4)."""
    inertia = mass * length**2 + arm_mass * length**2 / 3
    gravity = (mass * length + arm_mass * length / 2) * G
    return 2 * np.pi * np.sqrt(inertia / gravity)


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


def check_log(path: str, args) -> dict:
    with open(path) as f:
        log = json.load(f)
    entries = log["entries"]
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
    flags = []

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

    if row["trajectory"] == "lift_and_drop" and None not in (row["mass"], arm_mass,
                                                              row["length"]):
        t0 = pendulum_period(row["mass"], arm_mass, row["length"])
        row["period_model_s"] = round(t0, 4)
        swing = free_swing(t, q, torque, max_amplitude=np.radians(args.max_swing_deg))
        if swing is None:
            row["period_error"] = ""
            flags.append("free swing: no usable cycle (damped out or did not drop)")
        else:
            periods, amplitudes = swing
            expected = t0 * amplitude_factor(amplitudes)
            error = float(np.median(periods / expected - 1))
            row["period_measured_s"] = round(float(np.median(periods)), 4)
            row["period_cycles"] = len(periods)
            row["period_error"] = round(error, 4)
            if abs(error) > args.period_tolerance:
                flags.append(f"free-swing period off by {100 * error:+.1f} % "
                             "(check --mass, --arm-mass, --length)")

    if "registers" in log:
        problems = log.get("register_problems")
        if problems is None:
            try:
                from .dynamixel.xseries import check_registers
                problems = check_registers(log["registers"])
            except ImportError:  # dynamixel_sdk is not needed just to read logs
                problems = ["snapshot present but not checked (dynamixel_sdk missing)"]
        flags.extend(f"register {p}" for p in problems)
    elif row["motor"] in ("xh540", "xm540", "xm430"):
        row["unit"] = row["unit"] or "?"
        flags.append("no register snapshot (recorded before it was added)")

    row["flags"] = "; ".join(flags)
    return row


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
    parser.add_argument("--max-swing-deg", type=float, default=60.0,
                        help="only cycles below this amplitude are timed (each is "
                             "corrected for amplitude exactly)")
    parser.add_argument("--max-dt-ms", type=float, default=20.0)
    args = parser.parse_args()

    paths = sorted(glob.glob(os.path.join(args.logdir, "**", "*.json"), recursive=True))
    rows = []
    for path in paths:
        try:
            rows.append(check_log(path, args))
        except (KeyError, IndexError, ValueError, json.JSONDecodeError) as e:
            rows.append({"file": os.path.relpath(path, args.logdir),
                         "flags": f"unreadable: {type(e).__name__}: {e}"})
    if not rows:
        raise SystemExit(f"no *.json under {args.logdir}")

    flagged = [r for r in rows if r.get("flags")]
    print(f"{len(rows)} logs, {len(flagged)} flagged\n")
    for r in flagged:
        print(f"{r['file']}  [{r.get('kp')} {r.get('trajectory')}]")
        for flag in r["flags"].split("; "):
            print(f"    ! {flag}")

    swings = [r for r in rows if isinstance(r.get("period_error"), float)]
    if swings:
        errors = np.array([r["period_error"] for r in swings])
        print(f"\nfree-swing period error over {len(swings)} lift_and_drop logs: "
              f"median {100 * np.median(errors):+.1f} %, worst {100 * errors[np.argmax(np.abs(errors))]:+.1f} %")

    readable = [r for r in rows if "motor" in r]
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
