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

"""Read the control-table registers of one X series servo and save them as JSON.

Read-only: nothing is written to the servo, and torque is left as it is.

    python -m bam.dynamixel.dump_registers --port /dev/ttyUSB0 --unit xh540_A \
        --out registers/xh540_A.json
"""

import argparse
import datetime
import json
import os

from .xseries import DynamixelXSeries, check_registers

arg_parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--id", type=int, default=1)
arg_parser.add_argument("--baudrate", type=int, default=1_000_000)
arg_parser.add_argument("--unit", type=str, required=True,
                        help="label written on the servo, e.g. xh540_A")
arg_parser.add_argument("--out", type=str, required=True)
args = arg_parser.parse_args()

dxl = DynamixelXSeries(args.port, id=args.id, baudrate=args.baudrate)
registers = dxl.read_registers()
problems = check_registers(registers)

width = max(len(name) for name in registers)
for name, value in registers.items():
    print(f"  {name:<{width}}  {value}")

if problems:
    print("\nDiffers from what the identification assumes:")
    for problem in problems:
        print(f"  ! {problem}")
else:
    print("\nAll checked registers match what the identification assumes.")

os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
with open(args.out, "w") as f:
    json.dump({
        "unit": args.unit,
        "port": args.port,
        "id": args.id,
        "read_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "registers": registers,
        "problems": problems,
    }, f, indent=2)
print(f"\nSaved to {args.out}")
