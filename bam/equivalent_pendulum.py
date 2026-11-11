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

"""Recorder arguments that reproduce a real arm exactly in BAM's pendulum (MEASUREMENTS.md).

BAM's pendulum is a point mass `mass` at `length` on a uniform rod of mass `arm_mass`:

    gravity torque  (mass + arm_mass/2) * length * g * sin(q)
    inertia         (mass + arm_mass/3) * length**2

Two equations, so `mass` and `arm_mass` can be chosen to match both the real gravity torque
and the real inertia for any `length`. Matching only the torque, as the earlier sheet did,
leaves an inertia error that the fit puts into `armature`, and since the error differs between
load configurations it cannot be removed afterwards by a constant.

    python -m bam.equivalent_pendulum --arm-mass 0.210 --arm-com 0.118 \\
        --arm-inertia 0.00395 --block 0.512:0.135 --block 0.512:0.165

Each --block is mass:radius or mass:radius:inertia_about_its_own_centre (kg, m, kg m^2).
`arm_mass` may come out negative; bam.fit and bam.check_logs accept that (only gravity torque
and inertia enter), but the MuJoCo testbench needs a positive total mass.
"""

import argparse

G = 9.80665


def equivalent(arm_mass, arm_com, arm_inertia, blocks, length=None):
    """Return (mass, arm_mass, length) matching the real gravity torque and inertia.

    :param blocks: list of (mass, radius, inertia_about_own_centre)
    :param length: the length to use; default is the mass-weighted radius of the blocks
    """
    m_load = sum(m for m, _, _ in blocks)
    moment = sum(m * r for m, r, _ in blocks) + arm_mass * arm_com       # kg m
    inertia = sum(m * r**2 + i for m, r, i in blocks) + arm_inertia      # kg m^2
    if length is None:
        length = sum(m * r for m, r, _ in blocks) / m_load
    a = moment / length          # = mass + arm_mass/2
    b = inertia / length**2      # = mass + arm_mass/3
    return 3 * b - 2 * a, 6 * (a - b), length


def torque_only(arm_mass, arm_com, blocks):
    """The earlier sheet's values: block mass at the mean radius, arm mass matched on torque."""
    m_load = sum(m for m, _, _ in blocks)
    length = sum(m * r for m, r, _ in blocks) / m_load
    return m_load, 2 * arm_mass * arm_com / length, length


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--arm-mass", type=float, required=True, help="arm alone [kg]")
    parser.add_argument("--arm-com", type=float, required=True,
                        help="rotation axis to arm centre of mass [m]")
    parser.add_argument("--arm-inertia", type=float, required=True,
                        help="arm alone, about the rotation axis [kg m^2]")
    parser.add_argument("--block", action="append", required=True,
                        help="mass:radius[:inertia_about_own_centre], once per block fitted")
    parser.add_argument("--length", type=float, default=None,
                        help="default: mass-weighted radius of the blocks")
    args = parser.parse_args()

    blocks = []
    for spec in args.block:
        parts = [float(x) for x in spec.split(":")]
        if len(parts) == 2:
            parts.append(0.0)
        if len(parts) != 3:
            parser.error(f"--block {spec}: expected mass:radius[:inertia]")
        blocks.append(tuple(parts))

    mass, arm_mass, length = equivalent(args.arm_mass, args.arm_com, args.arm_inertia,
                                        blocks, args.length)
    true_inertia = (sum(m * r**2 + i for m, r, i in blocks) + args.arm_inertia)
    true_torque = (sum(m * r for m, r, _ in blocks) + args.arm_mass * args.arm_com) * G

    print(f"--mass {mass:.5f} --arm-mass {arm_mass:.5f} --length {length:.5f}\n")
    print(f"real gravity torque, arm horizontal: {true_torque:.4f} N m")
    print(f"real inertia about the axis:         {true_inertia:.6f} kg m^2")
    if arm_mass < 0:
        print("note: arm_mass is negative; fine for bam.fit, not for the MuJoCo testbench")

    m0, a0, l0 = torque_only(args.arm_mass, args.arm_com, blocks)
    i0 = (m0 + a0 / 3) * l0**2
    print(f"\ntorque-only values (earlier sheet): --mass {m0:.5f} --arm-mass {a0:.5f} "
          f"--length {l0:.5f}")
    print(f"  their inertia is {i0:.6f} kg m^2, {true_inertia - i0:+.6f} from the real one; "
          "a fit with them puts that difference into armature")


if __name__ == "__main__":
    main()
