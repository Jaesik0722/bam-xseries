# Actuator Identification Protocol — DYNAMIXEL XH540-W270, XM540-W270, XM430-W350

A single sheet to take someone with no prior exposure from an empty bench to a set of
identified actuator parameters. Work through it in order. Every step that can go wrong has a
check immediately after it.

**What you will produce:** for each of the three actuator models, a JSON file containing the
torque constant `kt`, the winding resistance `R`, the reflected rotor inertia `armature`, and
the friction terms. These go into the simulator so that simulated joints behave like the real
ones.

**Time:** per unit, about 45 minutes of recorder time (252 logs of about 10 s each including
initialisation and the return to zero), plus weight changes and cooling: 2 to 3.5 hours in
all. Fitting runs unattended; time the first fit before planning the rest. Building the
bench is the long pole.

---

## 1. What you need

### Hardware

| Item | Specification | Notes |
|---|---|---|
| Actuators | XH540-W270, XM540-W270, XM430-W350 | one of each; they are identified separately |
| Power supply | **14.8 V**, 6 A or more | the top of the 10.0–14.8 V input range of all three models; XH540 is rated 5.9 A at stall at this voltage |
| Interface | U2D2 + U2D2 Power Hub Board | USB to TTL/RS-485 |
| Aluminium profile | 20×20 or 30×30 mm, ~30 cm | the mast the actuator hangs from |
| Bench vice or C-clamps | — | clamps the profile to the table |
| Mounting bracket | 3D printed or machined, actuator-specific | see §2 |
| Pendulum arms | 3 lengths: 100, 150, 200 mm | see §2 |
| Weights | 0.2, 0.5, 1.2, 1.5, 3.0 kg | drilled and tapped, bolted to the arm end |
| Bolt | M8 × 40 or similar | holds the weight to the arm |
| Kitchen or lab scale | 1 g resolution | you must weigh everything |
| Ruler or calipers | 1 mm resolution | |

A reference design for the arms is published by the BAM authors as a parametric Onshape
model with configurable arm length:
<https://cad.onshape.com/documents/c132b33797dc72aa58be8a7c/v/9bb6aabfc542c5f6e9dabb39/e/8e6b266057c61625732fd9b0>

### Software

Python 3.12 is required (`requires-python` in `pyproject.toml`).

```bash
git clone <this repository>
cd bam-xseries
git checkout xseries
pip install -e ".[identification]"   # dynamixel_sdk, optuna, cmaes, matplotlib, ...
sudo apt install setserial            # required; the driver sets low-latency mode
```

`pip install -e .` alone installs only numpy and colorama; recording would work but
`bam.fit` would not.

Give yourself permission to the USB port, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

---

## 2. Building the bench

### 2.1 Mount the actuator

The actuator body must not move at all while the arm swings. Any compliance in the mount is
indistinguishable, in the data, from compliance in the actuator, and it will corrupt the
`armature` and friction estimates.

1. Clamp the aluminium profile to the table so that **one end overhangs the edge**. The arm
   must be able to swing to **±105° from vertical** without hitting anything: `sin_sin`
   commands up to ±100.8°, and the arm overshoots its target under load.
2. Bolt the actuator bracket to the overhanging end, using **at least two bolts**.
3. Bolt the actuator to the bracket through all available mounting holes.

> **Check.** Grip the arm and try to twist it. If you can see or feel the actuator *body*
> move relative to the profile, tighten or redesign the bracket before continuing. This is
> the single most common cause of a bad identification.

### 2.2 Build the arms

Use a **metal horn**, not a plastic one — plastic horns develop backlash under the loads
in §4 and the backlash appears in the fit as extra compliance.

- 3D printed arms are acceptable up to roughly 1.5 kg if printed in **PETG or ABS at 50 %
  infill or more**. PLA will creep under sustained load.
- For the 3.0 kg configurations use a **metal arm**, or shorten the arm (see §4.4).

> **Check.** Load the arm horizontally with the heaviest weight you intend to use on it and
> sight along it. Visible bending means the rigid-pendulum model is wrong and the fit will be
> biased. Use a stiffer arm.

### 2.3 Wire it up

```
14.8 V PSU ──► U2D2 Power Hub ──► actuator
                   │
                 U2D2 ──► USB to PC
```

Set the power supply to **exactly 14.8 V** and record what it reads.

14.8 V is the maximum rated input for all three units, and the `Max Voltage Limit` register
(address 32) defaults to 16.0 V, so nothing has to be reconfigured and no protection has to
be defeated. It is chosen over 12 V because it matches what the robot runs on and because the
higher no-load speed sweeps a wider part of the torque-speed plane, which separates `kt` from
`R` more cleanly.

Use a bench supply, not a battery. What matters is not the voltage level but that it is
**known and constant**: a pack discharging through a session puts that drift into the fitted
`kt` and `R`. (A fully charged 4S pack is 16.8 V, which is above the 14.8 V input range of
all three models.) If the supply sags under load the data is compromised (see §6.3).

---

## 3. Measuring the geometry

Three numbers per configuration. All of them are mandatory and all must be *measured*, not
assumed.

| Argument | What it is | How to get it |
|---|---|---|
| `--arm-mass` | mass of the arm alone, in kg | weigh the arm before fitting the weight |
| `--mass` | mass of the weight, in kg | weigh the weight and its bolt together |
| `--length` | distance from the **rotation axis** to the **centre of mass of the weight**, in metres | measure from the output shaft centre to the middle of the weight |

BAM's pendulum is a point mass on a uniform rod. If the arm is not a uniform rod -- a light
shaft with a heavy fixture at the end, for instance -- or the weight is split over blocks at
different radii, the values above are not the raw ones. See [MEASUREMENTS.md](MEASUREMENTS.md)
for what to read off the CAD model, and use `python -m bam.equivalent_pendulum` to convert it.
Values that match only the gravity torque leave an inertia error, and the fit puts it into
`armature`.

`--length` is *not* the length of the arm. If the weight is a 40 mm tall cylinder whose near
face sits 130 mm from the axis, then `--length` is 0.150 m, not 0.130 m.

---

## 4. Choosing the loads

The point of varying the load is **identifiability**. With one configuration, `armature` and
`friction_viscous` both act to slow the joint down in a similar way, and the fit cannot
separate them. Varying mass and length changes the inertia and the gravity torque
independently, which breaks the degeneracy, and varying the P gain sweeps the controller
through different parts of its range.

The tables below cover roughly 5 % to 50 % of each actuator's stall torque **at 14.8 V**.
The upper end matters. Measured on this robot's own policies, the knee runs at a median of
46 % and a 95th percentile of 54 % of the 9.9 N·m figure in the URDF, which is 4.6 to
5.4 N·m in absolute terms. The heaviest configuration below reaches 5.96 N·m, so the whole
operating range of the real joint sits inside the identified set.

Stall torque rises with supply voltage, so the same weights correspond to lower percentages
here than they would at 12 V. ROBOTIS e-Manual figures:

| | stall at 12 V | stall at 14.8 V |
|---|---|---|
| XH540-W270 | 9.9 N·m | **11.7 N·m** |
| XM540-W270 | 10.6 N·m | **12.9 N·m** |
| XM430-W350 | 4.1 N·m | **4.8 N·m** |

### 4.1 XH540-W270 and XM540-W270

Use the same set for both.

| Weight | 0.10 m | 0.15 m | 0.20 m |
|---|---|---|---|
| 0.5 kg | 0.53 N·m (5 %) | 0.79 (7 %) | 1.06 (9 %) |
| 1.5 kg | 1.51 (13 %) | 2.27 (19 %) | 3.02 (26 %) |
| 3.0 kg | 2.98 (25 %) | 4.47 (38 %) | 5.96 (51 %) |

Percentages are against the XH540 stall torque; for the XM540 they are one or two points
lower, which does not change the set.

### 4.2 XM430-W350

| Weight | 0.10 m | 0.15 m | 0.20 m |
|---|---|---|---|
| 0.2 kg | 0.24 N·m (5 %) | 0.35 (7 %) | 0.47 (10 %) |
| 0.5 kg | 0.53 (11 %) | 0.79 (16 %) | 1.06 (22 %) |
| 1.2 kg | 1.22 (25 %) | 1.82 (38 %) | 2.43 (51 %) |

Percentages are the gravity torque with the arm horizontal, assuming an 80 g arm.

### 4.3 P gains

The X series `Position P Gain` register runs 0–16383 and defaults to 800. Record each
configuration at:

```
100, 200, 400, 800
```

**The value your robot actually uses must be in this set.** Read it off the robot before you
start (register address 84) and add it if it is not already listed. Without it you cannot
convert the identified model into the stiffness your simulator should use.

### 4.4 If the arm bends

Same torque, less bending: halve the length and double the weight. 6 kg at 0.10 m gives the
same 5.9 N·m as 3 kg at 0.20 m. Note that this also halves the load inertia (2m·(L/2)² =
m·L²/2), which makes
`armature` *easier* to identify, so it is worth having both short-heavy and long-light
configurations in the set rather than only one kind.

---

## 5. Recording

### 5.1 The seven trajectories

Record every configuration with all seven. They are not interchangeable; each one reaches a
part of the operating space the others do not. All seven ship with the tool and run for 6 s.

| Trajectory | Motion | What it isolates |
|---|---|---|
| `sin_time_square` | sin(t²) | the primary trajectory: a chirp that sweeps a wide velocity range in one run |
| `sin_sin` | sin(t)·π/2 + sin(5t)·0.5·sin(2t) | several frequencies at once, covering velocity and acceleration together |
| `lift_and_drop` | move to −π/2, then torque off | with the H-bridge released there is no back-EMF, which is what separates viscous friction from the motor term |
| `up_and_down` | slow cubic 0 → π/2 → 0.8·π/2 | static friction and load-dependent effects, and the back-driving regime |
| `half_sine` | slow half-sine 0 → π/2 | the quasi-static end of the range |
| `steps` | step changes in target | the initial acceleration after a step is τ/J, so this bears directly on `armature` |
| `nothing` | torque disabled throughout | where the arm rests with no torque, which checks the zero (§5.4), and whether anything is loose |

The original study used the first four. `steps` and `nothing` are included here because
`armature` is the parameter this exercise exists to obtain and `steps` speaks to it directly,
and because `nothing` shows the zero offset the fit will have to absorb.

A plain fixed-frequency sine is **not** worth adding: `sin_time_square` is a swept sine and
already contains it. What improves an identification is reaching a regime the other
trajectories miss, not re-covering one they already visit.

### 5.2 Command

**Label every servo** with a short name written on its case (`xh540_A`, `xh540_B`, …) and
use that name for `--unit` and for the log directory. Logs of different units of the same
model must never share a directory: the fit reads every file in it as the same actuator.

**Before the first log of a unit, save its registers.** The recorder writes only the P gain
and Torque Enable; everything else (Operating Mode, PWM Limit, Profile Velocity and
Acceleration, I and D gains, feedforward) is whatever is stored in the servo, and the model
assumes particular values for them (§9). This reads the control table without writing
anything and prints any register that differs from what the model assumes:

```bash
python -m bam.dynamixel.dump_registers --port /dev/ttyUSB0 \
  --unit xh540_A --out registers/xh540_A.json
```

Fix any register it reports **before** recording, and record the fix in your notes.

Then, per log:

```bash
python -m bam.dynamixel.record \
  --motor xh540 --unit xh540_A \
  --port /dev/ttyUSB0 \
  --mass 1.5 --arm-mass 0.082 --length 0.15 \
  --kp 800 --vin 14.8 \
  --trajectory sin_time_square \
  --logdir logs_raw/xh540_A
```

`--motor` is one of `xh540`, `xm540`, `xm430`. `--vin` and `--unit` are required for these
models. `--vin` is the voltage **the fit will use**; the voltage the servo measures is logged
but not used by the fit, so set `--vin` to the supply voltage you actually read, and the
recorder warns if the two differ by more than 0.3 V. Each log also carries a snapshot of the
registers it was recorded under.

To record all P gains and all seven trajectories of one load configuration in one go:

```bash
python -m bam.dynamixel.all_record --motor xh540 --unit xh540_A \
  --mass 1.5 --arm-mass 0.082 --length 0.15 --vin 14.8 \
  --logdir logs_raw/xh540_A
# add the robot's own gain if it is not 100, 200, 400 or 800:  --kp 100 200 400 800 <robot kp>
```

The recorder checks that `--logdir` can be written before the arm moves. If a recording fails
part-way (a bus error, Ctrl-C), it still lowers the arm, disables torque, and saves what it
has as `<date>_aborted.json`, then exits with an error; `all_record` stops at the first such
failure instead of running the rest of the grid. Move aborted logs out of the directory
(`bam.check_logs` flags them) and record the missing cells again.

### 5.3 Order of work

Per actuator: 9 load configurations × 4 P gains × 7 trajectories = **252 logs of 6 s each**,
about 25 minutes of recording plus the time to swap weights.

```
for each load configuration (weight, length):
    fit the weight, re-check the zero (§5.4)
    for kp in 100, 200, 400, 800:
        for trajectory in sin_time_square, sin_sin, lift_and_drop, up_and_down,
                          half_sine, steps, nothing:
            record
```

Two rules about ordering:

- **Randomise the order of the configurations.** Recording them in sequence makes temperature
  rise monotonically through the session, which makes temperature indistinguishable from
  whatever else you changed. Shuffling lets you check for thermal drift afterwards.
- **Leave the heaviest configurations until last** and let the actuator cool between them.
  Seven trajectories at 3.0 kg × 0.20 m is nearly three minutes of near-continuous load; check
  the temperature between trajectories, not only between configurations.

### 5.4 Zero position

The model's zero is the arm hanging **straight down**, but nothing is set in software: the
driver reads encoder count 2048 as 0 rad, always. The zero is therefore fixed by how the horn
and arm are bolted on, and the fit's `q_offset` can absorb only ±0.1 rad (±5.7°).

1. With torque disabled, let the arm hang and settle.
2. Read the position:

   ```bash
   python -c "from bam.dynamixel.xseries import DynamixelXSeries as D; print(D('/dev/ttyUSB0').read_data()['position'])"
   ```

3. If it is more than about 0.03 rad (2°) from zero, re-seat the arm on the horn at the
   spline position closest to zero and read again. Do not change `Homing Offset` (register
   20): the register check would not catch the change in what 0 rad means.
4. Write the reading in your notes, and repeat it each time the weight is changed.

`bam.check_logs` flags a `nothing` log whose arm rests more than 0.05 rad from zero.

### 5.5 What gets logged

Every sample records position, velocity, current, the applied PWM duty (`pwm`, used in §9),
input voltage and **temperature**. Keep
the temperature: the winding resistance rises about 0.39 %/°C and the magnets weaken about
0.11 %/°C, so stall torque falls roughly 20 % between 25 °C and 70 °C. You want to be able
to check whether that affected your session.

---

## 6. Checks before you fit

Do not skip this. Fitting bad data produces plausible-looking numbers.

Every check below except the visual ones in §6.1 and §6.2 is automated. Run it on the raw
logs after each session, while the bench is still set up:

```bash
python -m bam.check_logs --logdir logs_raw/xh540_A --csv checks_xh540_A.csv
```

It reports two kinds of finding per log.

- **Flags (`!`)** are faults in the recording: an aborted recording, `--vin` against the
  measured voltage, supply sag (§6.3), temperature over 70 °C (§6.5), an arm that did not
  move or a `nothing` log that did, a `nothing` log resting beyond what `q_offset` can absorb
  (§5.4), registers that differ from what the model assumes, and gaps in the sampling.
  **Flagged logs are excluded from the fit.**
- **Notes (`-`)** are diagnostics: the free-swing comparisons (§6.4), the applied PWM
  against `error_gain` (§9), a missing register snapshot. A note does not by itself exclude a
  log; it tells you what to look at.

It then lists every (load, kp, trajectory) cell that is missing or recorded twice. **Never
delete a log.** Move excluded ones to a separate directory (`logs_raw/<unit>_excluded/`) and
keep the CSV, so that what was excluded and why is on record. A `--vin` that was simply
entered wrong can be corrected in a copy of the log instead; note that you did.

### 6.1 Did the arm move?

Open a few logs and plot `position`. A configuration where the controller saturated and the
arm barely moved carries no information. Move those logs aside (they are flagged).

### 6.2 Does `lift_and_drop` decay?

The free oscillation after the drop should decay visibly. If it does not, the arm is
probably fouling something.

### 6.3 Did the supply hold up?

Plot `input_volts` on the heaviest configuration. If it sags more than a few tenths of a volt,
your supply is too small and every log taken at that load is biased. Fix the supply and redo
them.

### 6.4 Is the free swing consistent across configurations?

`lift_and_drop` raises the arm to −π/2 and then disables torque. With torque off there is no
motor torque, but the rotor and gears still turn with the arm, so the swing is a pendulum
whose inertia includes the reflected rotor inertia `armature`:

```
T0 = 2π · sqrt( (m·L² + m_arm·L²/3 + armature) / ((m·L + m_arm·L/2)·g) )
```

For the light configurations `armature` is as large as the load inertia (XH540: about
0.027 kg·m² against 0.034 kg·m² for 1.5 kg at 0.15 m), so leaving it out is not a small
error.

The release is from 90°, where the period is about 1.18·T0. `bam.check_logs` times every
cycle below 80° and corrects each for its amplitude exactly (armature adds inertia only, so
the same correction holds), then solves the formula above for the `armature` each log
implies. Coulomb friction shortens the swing but not its period. Viscous friction lengthens
it by 1/√(1−ζ²): under 1 % for `friction_viscous` around 0.05 N·m·s/rad, but several percent
if it is a few tenths, so after the fit pass it too. Logs with fewer than two usable cycles
are reported but not judged.

The closed-form period is only a reference. It leaves out the load-dependent friction of
m4, which also changes the period: on logs generated by an m4 model itself, the formula with
that model's exact `armature` and `friction_viscous` was off by up to 16 % on the
light, short configurations. **Do not exclude a log because of it.**

What it is used for:

- **Before fitting** (note only): the median implied `armature` over the unit's
  configurations, and the configurations whose period is more than 3 % away from the one
  computed with it. Look at those: their mass, length and arm values, and whether the arm
  fouls something. A negative implied `armature` means the entered geometry is too heavy or
  too long for the swing it produced.
- **After fitting** (the M1 criterion of the SIMPAT plan): run the check on the resampled
  logs with the fitted model,

  ```bash
  python -m bam.check_logs --logdir logs/xh540_A --params params/xh540_A_m4.json
  ```

  Each `lift_and_drop` log is then simulated with the whole fitted model, friction included,
  and the period of the simulated swing is compared with the measured one. The summary
  reports how many logs are within 3 %. This is a result to report in M1, not a reason to
  drop logs and refit.

This is not an independent check on mass and length. An error in either is partly absorbed
by the implied `armature`: a 0.15 m entry for a weight really at 0.17 m (0.5 kg) moves the
period only about 1.5 %. Measure the geometry carefully; the swing catches only gross errors
and configurations that disagree with each other.

`nothing` is a weaker version of the same check: it disables torque with the target at zero,
so unless the arm happens to be displaced when it starts, it simply sits still. Its value is
in confirming that a stationary arm really is stationary — if it drifts, something is loose.

### 6.5 Temperature

Note the start and end temperature of each log. If anything exceeded 70 °C, set those logs
aside and look at them separately — the actuator was not the same device it was when cold.

> **Safety.** The `Temperature Limit` register defaults to 80 °C, at which point the firmware
> raises a hardware error and **disables torque**. A loaded arm will then fall. Keep hands and
> equipment clear, and cool the actuator rather than working near the limit.

---

## 7. Fitting

### 7.1 Resample

The recorder samples at whatever rate the bus allows; the fit needs a fixed time step. Resample
the logs that passed §6 into a separate directory first (the fit fails on raw logs):

```bash
mkdir -p logs/xh540_A
python -m bam.process --raw logs_raw/xh540_A --logdir logs/xh540_A --dt 0.005
```

### 7.2 Fit m4 and m1, holding out kp 400

```bash
for model in m4 m1; do
  python -m bam.fit \
    --logdir logs/xh540_A \
    --actuator xh540 \
    --method cmaes \
    --model $model \
    --trials 4000 \
    --validation_kp 400 \
    --output params/xh540_A_$model.json | tee fit_xh540_A_$model.txt
done
```

**`m4` is the model used in the experiments** (fixed in the SIMPAT plan): Coulomb and
viscous friction, plus Stribeck and load-dependent terms. `m1` (Coulomb and viscous only) is
fitted alongside it as the baseline: the improvement of m4 over m1 on the held-out kp is
reported, so both validation scores are needed.

`m5` and `m6` add direction-dependent terms. The original BAM study fitted all of m1–m6 on
the same kind of pendulum bench, so they can be fitted here too. Fit `m6` only in addition to
m4, never instead of it, and treat it as a candidate: it would be worth considering only if
it lowers the held-out error clearly and gives similar parameters when the fit is repeated.
Report it as an extra result; it does not replace m4 in the experiments.

The fit prints `Validation score (kp = 400)` while it runs: the mean absolute position error
on the held-out gain, in radians. Keep the last printed value (the `tee` above saves it).

### 7.3 Fit each P gain on its own

This is the drift check of §9. It needs no validation split:

```bash
for kp in 100 200 400 800; do
  python -m bam.fit --logdir logs/xh540_A --actuator xh540 --method cmaes \
    --model m4 --trials 4000 --only_kp $kp \
    --output params/xh540_A_m4_kp$kp.json
done
```

Time the first fit before planning the rest; with about 190 logs per fit and 4000 trials,
expect tens of minutes per run. The runs are independent and can be started in parallel.

### 7.4 What to hand in

For each unit: the raw logs (`logs_raw/<unit>/`, including the excluded ones in their own
directory), the register dump (`registers/<unit>.json`), the `bam.check_logs` CSV, the
geometry sheet (MEASUREMENTS.md values, and mass, arm mass and length per configuration),
the supply voltage you read, the hanging position read per configuration (§5.4), the fitted
JSON files of §7.2 and §7.3 with their printed output, the `bam.catalog_check` output for
the m4 fit, and the commit hash of this repository you fitted with (`git rev-parse HEAD`).

---

## 8. Reading the result

```json
{
  "kt": 2.94, "R": 3.56, "armature": 0.027,
  "friction_base": 0.01, "friction_viscous": 0.05,
  "friction_stribeck": 0.1, "load_friction_base": 0.009,
  "model": "m4", "actuator": "xh540"
}
```

### 8.1 Comparison with the catalogue

The catalogue quotes output-shaft figures, so compare what the **whole model, friction
included**, predicts, not the motor terms alone:

```bash
python -m bam.catalog_check --params params/xh540_A_m4.json --vin 14.8
```

It prints:

- **breakaway/stall**: the largest load the motor can start to lift from rest at full duty,
  that is `kt·V/R` minus the static friction budget at that load. This is not the same
  quantity as ROBOTIS's stall torque (below);
- **no-load speed**: the speed the unloaded output reaches when started from rest at full
  duty, simulated with the fitted friction. With a Stribeck term the torque balance can have
  more than one root, and solving for "the" root can pick one the motor never reaches from
  rest; if there are several, they are all printed;
- the frictionless `kt·V/R` and `V/kt` alongside, for reference only. With a load-dependent
  friction model these are well above the catalogue even for a correct fit.

The catalogue values (ROBOTIS e-Manual) at the listed voltages are below. The script
interpolates between them and refuses voltages outside 11.1–14.8 V.

| | stall 12 V | stall **14.8 V** | no-load 12 V | no-load **14.8 V** |
|---|---|---|---|---|
| XH540-W270 | 9.9 N·m | **11.7 N·m** | 39 rpm (4.08 rad/s) | **46 rpm (4.82 rad/s)** |
| XM540-W270 | 10.6 | **12.9** | 30 (3.14) | **37 (3.87)** |
| XM430-W350 | 4.1 | **4.8** | 46 (4.82) | **57 (5.97)** |

The e-Manual defines stall torque as the maximum momentary torque and notes that its N–T
curves, measured under a gradually increasing load, sit below it. The comparison is
therefore coarse. The SIMPAT plan's criterion is **±20 %**, and it is reported as such, but
a miss is a reason to investigate, not to reject the fit on that ground alone. The usual
causes: the mount moved, the geometry was mis-entered, the friction terms traded off against
`kt` and `R`, or `error_gain` is wrong (§9). The held-out validation error (§7.2) is the
primary measure of the fit.

### 8.2 Expected magnitudes

| Parameter | Plausible range for these units |
|---|---|
| `kt` | 2–4 V/(rad/s) |
| `R` | 3–8 Ω |
| `armature` | 0.01–0.05 kg·m² |
| `friction_viscous` | 0.01–0.1 N·m·s/rad |

`kt` and `R` are properties of the magnets and windings. They do not depend on the supply
voltage, so the ranges above apply whichever voltage you identified at, and a model fitted at
14.8 V predicts behaviour elsewhere in the 10.0–14.8 V input range by substituting `V`.

A parameter that sits on the edge of its search range (the fit prints `min` and `max` for
each) needs a closer look before it is used. Either the true value lies outside the range,
or the data do not constrain it, or it trades off against another parameter. Repeat the fit
(CMA-ES is stochastic) and see whether it stays there and whether the validation error
changes. Report what you find. Do not widen the range just to move it off the edge.

For context, the URDF currently models the neck link as 9 × 10⁻⁶ kg·m² of inertia driven by a
10 N·m actuator. The `armature` term is roughly a thousand times larger and is the dominant
inertia at that joint. This is the number the whole exercise exists to obtain.

---

## 9. Verifying `error_gain`

`error_gain` converts a position error in radians into a PWM duty cycle. It is **not fitted**;
it is a property of the firmware, derived here as

```
error_gain = (counts_per_rev / 2π) / (KP_DIVISOR × PWM_LIMIT)
           = (4096 / 2π) / (128 × 885)
           = 5.755e-03
```

The e-Manual gives `KP_DIVISOR = 128`. The BAM authors found that the XL330 actually behaves
as if it were 256. **This has not been verified on the X540/X430 series.**

`PWM_LIMIT` is the register value the model assumes (885, the factory default). If the
register dump of §5.2 shows a different `pwm_limit`, a non-zero profile, I or D gain, or an
Operating Mode other than 3, the model does not describe the controller that was recorded,
and `kt` and `R` will absorb the difference.

**Direct check.** Each sample logs the duty the firmware actually applied (`pwm`, from
Present PWM, register 124) next to the goal and present position. With a pure P loop the
firmware's duty should be `error_gain · kp · (q_target − q)` until it saturates.
`bam.check_logs` fits that ratio on the unsaturated samples of every log and prints its median
per P gain:

- about **1.0** at every P gain: `error_gain` is right;
- about **0.5** at every P gain: the firmware behaves as if the divisor were 256, as BAM found
  for the XL330;
- different values at different P gains, or far from both: a register (§5.2) or
  something else in the firmware path is not what the model assumes. Changing the divisor
  would not fix it.

Only change `X_KP_DIVISOR` in `bam/dynamixel/xseries_actuator.py` on the second result, and
record the change with the fit. Logs recorded before the `pwm` channel was added cannot be
checked this way.

**Indirect check.** Fit each P gain separately (§7.3) and compare `kt` and `R` across them.
They are properties of the motor and should agree within 10 %. A systematic drift with the
P gain has several possible causes besides `error_gain` — a non-default register, PWM
saturation at high gains, friction terms trading off against `R` differently at each gain —
so use the direct check above to tell them apart rather than changing the divisor to make
the drift disappear.

---

## 10. Using the result in simulation

The identified numbers describe the actuator, not the controller. Nothing changes on the
robot: it still runs its own firmware position loop. What changes is the simulator.

To reproduce the identified model, the simulator has to run the same computation as the fit,
every physics step. Do not re-derive it: call or port the code that the fit itself used,
`bam/simulate.py` (`Simulator.step`) with `Model.compute_frictions` in `bam/model.py`; for
MuJoCo and mjlab, `bam/mujoco.py` and `bam/mjlab.py` already do this. In outline:

```python
# at asset creation
armature = params["armature"]                      # added to the joint inertia

# each physics step
duty  = clip(error_gain * kp * (q_target - q), -1, 1)     # firmware P loop
tau_m = kt * (duty * V_supply - kt * dq) / R              # applied voltage is duty * V_supply
tau_g = bias torque at the joint (gravity, contacts)      # "external_torque" in BAM
frictionloss, damping = model.compute_frictions(tau_m, tau_g, dq)
# Stribeck and load-dependent terms make frictionloss depend on tau_m, tau_g and dq, so it
# must be recomputed every step; a constant joint friction does not reproduce m4.
# Static friction: clip the torque needed to stop the joint this step,
#   tau_stop = (inertia / dt) * dq + tau_m + tau_g,
# to +-(frictionloss + damping * |dq|), and apply it against the motion (simulate.py).
```

In this fork, `bam/mujoco.py` and `bam/mjlab.py` compute the friction budget from the motor
torque of the current step (zero when torque is off), as the fit does, and the m6 quadratic
term follows `bam/model.py` exactly. The external torque they use is still the previous
solve's, since MuJoCo only provides it after a step; on the bench pendulum this keeps the
MuJoCo rollout within about 0.02 rad (mean) of the fitting simulator. When porting to another
engine, keep the same split: this step's motor torque, and the latest available external
torque.

`kt` already refers to the output shaft (it includes the gear ratio), so `tau_m` must not be
multiplied by the gear ratio again. The voltage that enters the torque is the PWM-applied
voltage `duty · V_supply`, so the supply voltage the robot actually sees has to be known.

Linearising the motor term away from duty saturation gives the effective PD gains for the
P gain your robot runs and the supply voltage it sees:

```
effective stiffness = error_gain × kp × V_supply × kt / R
effective damping   = kt² / R                    (this is the back-EMF term; the identified
                                                  friction_viscous adds to it)
```

These are for comparing with a plain PD model (they are what a calibrated-PD baseline uses).
They leave out the duty saturation and the friction model, so a simulation built from them
alone is not the identified model. The stiffness is proportional to the supply voltage:
14.8 V against 12.0 V is a 23 % difference.

So the bench does not hand you a stiffness directly — it hands you the function that turns
your robot's firmware P gain into the stiffness your simulator should use.

---

## 11. Checklist

```
[ ] Bench supply set to 14.8 V, 6 A or more, verified not to sag under the heaviest load
[ ] Actuator body immovable when the arm is twisted by hand
[ ] Metal horn fitted
[ ] Arm does not visibly bend at its heaviest intended load
[ ] Python 3.12, installed with pip install -e ".[identification]"
[ ] Arm clears everything through ±105° from vertical
[ ] Arm mass, weight mass and axis-to-weight-centre distance all weighed and measured
[ ] Non-uniform arm or split weights converted with bam.equivalent_pendulum (MEASUREMENTS.md)
[ ] Robot's own Position P Gain read off and included in the gain list
[ ] Servo labelled; --unit and log directory use the label
[ ] Registers dumped before the first log; nothing reported, or the fix noted
[ ] --vin set to the supply voltage actually read
[ ] Hanging position read with torque off, within 0.03 rad of zero, noted per configuration
[ ] Configuration order randomised
[ ] Temperature logged, nothing over 70 °C
[ ] Logs inspected before fitting (motion, decay, supply voltage)
[ ] bam.check_logs run, CSV kept, flagged logs moved aside (not deleted)
[ ] free-swing notes looked at (§6.4); after the fit, check_logs --params on the resampled logs
[ ] applied PWM / predicted PWM about 1.0 at every P gain (§9)
[ ] Logs resampled with bam.process before fitting (§7.1)
[ ] m4 and m1 fitted with kp 400 held out; both validation scores kept (§7.2)
[ ] bam.catalog_check run, friction included, result reported against ±20 % (§8.1)
[ ] No fitted parameter on the edge of its range, or the ones that are reported
[ ] kt and R checked for drift across P gains (§7.3, §9)
[ ] Everything in §7.4 handed in, with the commit hash
```
