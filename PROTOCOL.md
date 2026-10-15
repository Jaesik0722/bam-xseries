# Actuator Identification Protocol — DYNAMIXEL XH540-W270, XM540-W270, XM430-W350

A single sheet to take someone with no prior exposure from an empty bench to a set of
identified actuator parameters. Work through it in order. Every step that can go wrong has a
check immediately after it.

**What you will produce:** for each of the three actuator models, a JSON file containing the
torque constant `kt`, the winding resistance `R`, the reflected rotor inertia `armature`, and
the friction terms. These go into the simulator so that simulated joints behave like the real
ones.

**Time:** about half a day of recording once the bench exists, plus a few minutes of fitting
per actuator. Building the bench is the long pole.

---

## 1. What you need

### Hardware

| Item | Specification | Notes |
|---|---|---|
| Actuators | XH540-W270, XM540-W270, XM430-W350 | one of each; they are identified separately |
| Power supply | **14.8 V**, 6 A or more | matches 4S nominal, which is what the robot runs; XH540 draws about 6 A at stall at this voltage |
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

```bash
git clone <this repository>
cd bam-xseries
pip install -e .
pip install dynamixel_sdk
sudo apt install setserial          # required; the driver sets low-latency mode
```

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
   must be able to swing to ±90° from vertical without hitting anything.
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
**known and constant**: a pack drifting from 16.8 V down to 14 V through a session puts that
drift into the fitted `kt` and `R`. If the supply sags under load the data is compromised
(see §6.3).

---

## 3. Measuring the geometry

Three numbers per configuration. All of them are mandatory and all must be *measured*, not
assumed.

| Argument | What it is | How to get it |
|---|---|---|
| `--arm-mass` | mass of the arm alone, in kg | weigh the arm before fitting the weight |
| `--mass` | mass of the weight, in kg | weigh the weight and its bolt together |
| `--length` | distance from the **rotation axis** to the **centre of mass of the weight**, in metres | measure from the output shaft centre to the middle of the weight |

If the arm is not a uniform rod -- a light shaft with a heavy fixture at the end, for
instance -- the values above are not the raw ones. See [MEASUREMENTS.md](MEASUREMENTS.md) for
what to read off the CAD model and how to convert it.

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
here than they would at 12 V:

| | stall at 12 V | stall at 14.8 V |
|---|---|---|
| XH540-W270 | 9.9 N·m | **12.21 N·m** |
| XM540-W270 | 10.6 N·m | **13.07 N·m** |
| XM430-W350 | 4.1 N·m | **5.06 N·m** |

### 4.1 XH540-W270 and XM540-W270

Use the same set for both.

| Weight | 0.10 m | 0.15 m | 0.20 m |
|---|---|---|---|
| 0.5 kg | 0.53 N·m (4 %) | 0.79 (7 %) | 1.06 (9 %) |
| 1.5 kg | 1.51 (12 %) | 2.27 (19 %) | 3.02 (25 %) |
| 3.0 kg | 2.98 (24 %) | 4.47 (37 %) | 5.96 (49 %) |

Percentages are against the XH540 stall torque; for the XM540 they are one or two points
lower, which does not change the set.

### 4.2 XM430-W350

| Weight | 0.10 m | 0.15 m | 0.20 m |
|---|---|---|---|
| 0.2 kg | 0.24 N·m (5 %) | 0.35 (7 %) | 0.47 (9 %) |
| 0.5 kg | 0.53 (10 %) | 0.79 (16 %) | 1.06 (21 %) |
| 1.2 kg | 1.22 (24 %) | 1.82 (36 %) | 2.43 (48 %) |

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
same 5.9 N·m as 3 kg at 0.20 m. Note that this also quarters the load inertia, which makes
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
| `nothing` | torque disabled throughout | pure gravity response, which is the only way to check the pendulum model itself (see §6.5) |

The original study used the first four. `steps` and `nothing` are included here because
`armature` is the parameter this exercise exists to obtain and `steps` speaks to it directly,
and because `nothing` is the one trajectory that can tell you your mass and length entries
are wrong.

A plain fixed-frequency sine is **not** worth adding: `sin_time_square` is a swept sine and
already contains it. What improves an identification is reaching a regime the other
trajectories miss, not re-covering one they already visit.

### 5.2 Command

```bash
python -m bam.dynamixel.record \
  --motor xh540 \
  --port /dev/ttyUSB0 \
  --mass 1.5 --arm-mass 0.082 --length 0.15 \
  --kp 800 --vin 14.8 \
  --trajectory sin_time_square \
  --logdir logs/xh540
```

`--motor` is one of `xh540`, `xm540`, `xm430`.

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

Zero radians is the arm hanging **straight down**. Disable torque, let the arm settle under
gravity, and take that as zero. The fit has a `q_offset` parameter that absorbs a small
error, but starting close is better.

### 5.5 What gets logged

Every sample records position, velocity, current, input voltage and **temperature**. Keep
the temperature: the winding resistance rises about 0.39 %/°C and the magnets weaken about
0.11 %/°C, so stall torque falls roughly 20 % between 25 °C and 70 °C. You want to be able
to check whether that affected your session.

---

## 6. Checks before you fit

Do not skip this. Fitting bad data produces plausible-looking numbers.

### 6.1 Did the arm move?

Open a few logs and plot `position`. A configuration where the controller saturated and the
arm barely moved carries no information. Delete those logs.

### 6.2 Does `lift_and_drop` decay?

The free oscillation after the drop should decay visibly. If it does not, the arm is
probably fouling something.

### 6.3 Did the supply hold up?

Plot `input_volts` on the heaviest configuration. If it sags more than a few tenths of a volt,
your supply is too small and every log taken at that load is biased. Fix the supply and redo
them.

### 6.4 Does the free swing match the geometry you entered?

`lift_and_drop` raises the arm to −π/2 and then disables torque, so the rest of that log is a
plain pendulum. Its period depends only on the mass, the length and gravity — no actuator
parameter enters — which makes it an independent check on the two numbers you measured by
hand.

For small swings,

```
T0 = 2π · sqrt( (m·L² + m_arm·L²/3) / ((m·L + m_arm·L/2)·g) )
```

The release is from 90°, where the true period is about **1.18·T0**; by the time the swing
has decayed to roughly 30° it is within 2 % of `T0`. So measure the period of the *later*
swings in the log and compare against `T0`.

A disagreement beyond a few per cent means `--mass` or `--length` is wrong, and every other
log from that configuration is wrong with it.

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

```bash
python -m bam.fit \
  --logdir logs/xh540 \
  --actuator xh540 \
  --method cmaes \
  --model m4 \
  --trials 4000 \
  --output params/xh540_m4.json
```

`m4` is the model that performed best for Dynamixel units in the original study: Coulomb and
viscous friction, plus Stribeck and load-dependent terms. Start there. If you want to see
whether the complexity is earning its place, fit `m1` as well and compare the validation
error.

Hold out one P gain for validation rather than reporting the fit error:

```bash
  --validation_kp 400
```

Fitting takes a few minutes per actuator.

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

### 8.1 Sanity check against the catalogue

```
stall torque   = kt * V / R
no-load speed  = V / kt
```

Evaluate these at the voltage you actually used. The catalogue figures are quoted at 12 V, so
at 14.8 V the same motor should come out higher:

| | stall, 12 V | stall, **14.8 V** | no-load, 12 V | no-load, **14.8 V** |
|---|---|---|---|---|
| XH540-W270 | 9.9 N·m | **12.21** | 4.08 rad/s | **5.04** |
| XM540-W270 | 10.6 | **13.07** | 3.14 | **3.88** |
| XM430-W350 | 4.1 | **5.06** | 4.82 | **5.94** |

The starting values shipped in this repository reproduce the 12 V catalogue exactly, so a
fitted result far from the right column is telling you something is wrong — most likely the
mount moved, a mass or length was mis-entered, or `error_gain` is wrong (§9).

### 8.2 Expected magnitudes

| Parameter | Plausible range for these units |
|---|---|
| `kt` | 2–4 V/(rad/s) |
| `R` | 3–8 Ω |

`kt` and `R` are properties of the magnets and windings. They do not depend on the supply
voltage, so the ranges above apply whichever voltage you identified at, and a model fitted at
14.8 V predicts behaviour at 12 V or 16.8 V by substituting `V` in the equations above.
| `armature` | 0.01–0.05 kg·m² |
| `friction_viscous` | 0.01–0.1 N·m·s/rad |

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

How to tell: fit each P gain separately and compare `kt` and `R` across them. If `error_gain`
is correct, those values are the same whatever the gain, because they are properties of the
motor. If they drift systematically with the P gain, `error_gain` is off by a constant
factor — change `X_KP_DIVISOR` in `bam/dynamixel/xseries_actuator.py` to 256 and refit. If
neither value works, the gain must be measured directly with an oscilloscope on the motor
terminals.

---

## 10. Using the result in simulation

The identified numbers describe the actuator, not the controller. Nothing changes on the
robot: it still runs its own firmware position loop. What changes is the simulator.

```python
# at asset creation
props["armature"] = 0.027        # identified
props["friction"] = 0.05         # identified

# each control step
tau_cmd   = kt * V / R - kt**2 * dq / R
tau_avail = tau_stall * (1 - clamp(dq * sign(tau_cmd) / omega_noload, 0, 1))
tau       = clamp(tau_cmd, -tau_avail, tau_avail)
```

The effective PD gains follow from the identified parameters and the P gain your robot runs:

```
effective stiffness = error_gain × kp × kt / R
effective damping   = kt² / R                    (this is the back-EMF term)
```

So the bench does not hand you a stiffness directly — it hands you the function that turns
your robot's firmware P gain into the stiffness your simulator should use.

---

## 11. Checklist

```
[ ] Bench supply set to 14.8 V, 6 A or more, verified not to sag under the heaviest load
[ ] Actuator body immovable when the arm is twisted by hand
[ ] Metal horn fitted
[ ] Arm does not visibly bend at its heaviest intended load
[ ] Arm clears everything through ±90° from vertical
[ ] Arm mass, weight mass and axis-to-weight-centre distance all weighed and measured
[ ] Robot's own Position P Gain read off and included in the gain list
[ ] Zero set with torque disabled and the arm hanging free
[ ] Configuration order randomised
[ ] Temperature logged, nothing over 70 °C
[ ] Logs inspected before fitting (motion, decay, supply voltage)
[ ] free-swing period in `lift_and_drop` checked against the measured mass and length (§6.4)
[ ] Fitted stall torque and no-load speed compared against the catalogue
[ ] kt and R checked for drift across P gains (§9)
```
