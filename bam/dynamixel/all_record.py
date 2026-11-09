# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import argparse
import os
import time


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--arm-mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--vin", type=float, required=False, default=None)
arg_parser.add_argument("--unit", type=str, default=None)
arg_parser.add_argument("--kp", type=int, nargs="+", default=None,
                        help="override the P gain list, e.g. to add the robot's own gain")
arg_parser.add_argument("--speak", action="store_true")
args = arg_parser.parse_args()

KPS = {
    "mx64": [4, 8, 16, 32],
    "mx106": [4, 8, 16, 32],
    "xl320": [4, 8, 16, 32],
    "xl330": [50, 100, 200, 300, 400],
    "xl330i": [50, 100, 200, 300, 400],
    "xh540": [100, 200, 400, 800],
    "xm540": [100, 200, 400, 800],
    "xm430": [100, 200, 400, 800],
}
XSERIES = {"xh540", "xm540", "xm430"}

if args.motor not in KPS:
    raise ValueError(f"Unknown motor '{args.motor}'. Known motors: {list(KPS.keys())}")

kps = args.kp if args.kp is not None else KPS[args.motor]
if args.motor in XSERIES:
    if args.vin is None or args.unit is None:
        arg_parser.error("--vin and --unit are required for the X series")
    # PROTOCOL.md section 5.1: all seven trajectories
    trajectories = ["sin_time_square", "sin_sin", "lift_and_drop", "up_and_down",
                    "half_sine", "steps", "nothing"]
else:
    if args.vin is None:
        args.vin = 15.0
    trajectories = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]

command_base = f"uv run -m bam.dynamixel.record --mass {args.mass} --arm-mass {args.arm_mass} --length {args.length}"
command_base += (
    f" --port {args.port} --logdir {args.logdir} --motor {args.motor} --vin {args.vin}"
)
if args.unit is not None:
    command_base += f" --unit {args.unit}"


for kp in kps:
    for trajectory in trajectories:
        sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
        print(sentence)

        if args.speak:
            from gtts import gTTS

            myobj = gTTS(text=sentence, lang="en", slow=False)
            myobj.save("/tmp/message.mp3")
            os.system("mpg321 /tmp/message.mp3")

        command = f"{command_base} --kp {kp} --trajectory {trajectory}"
        os.system(command)

        if trajectory == "sin_time_square":
            time.sleep(3)
