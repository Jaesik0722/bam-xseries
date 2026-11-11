# What to Measure Before Recording

The pendulum model BAM fits assumes a **uniform rod** carrying a **point mass at its tip**.
The arm in this design is not uniform — its mass sits mostly in the cage at the far end — and
the weight can be split over slots at different radii, so the numbers passed to the recorder
have to be equivalent values, not raw ones. Sections A–C are measured once per arm; D is
computed per configuration.

---

## A. In Fusion — the arm alone

Suppress or exclude the weight blocks, then run **INSPECT → Physical Properties** on the arm
assembly (everything that rotates: bracket side excluded, cage included).

Set the reference to the **servo output shaft axis**.

| Symbol | What to read | Unit |
|---|---|---|
| `m_arm` | Mass | kg |
| `d_arm` | Distance from the rotation axis to the centre of mass | m |
| `I_arm` | Moment of inertia about the rotation axis | kg·m² |

> Make sure the centre of mass is reported **relative to the rotation axis**, not to the
> origin of the design. If Fusion gives it in design coordinates, subtract the axis position.

---

## B. In Fusion — the slot positions

For each of the four slots, read the distance from the rotation axis to the **centre of the
slot**:

| Slot | Symbol | Value |
|---|---|---|
| upper left | `r1` | |
| upper right | `r2` | |
| lower left | `r3` | |
| lower right | `r4` | |

The two upper slots should share a radius, and the two lower slots another. If so, only two
numbers are needed: `r_upper` and `r_lower`.

---

## C. On the bench — the weights

| Symbol | What to do | Unit |
|---|---|---|
| `m_block` | Weigh one block **with its fastener**, on a 1 g scale | kg |
| `I_block` | Its moment of inertia about its **own** centre, from CAD (optional; small for compact blocks) | kg·m² |

Weigh them individually if they are not identical, and keep them labelled.

---

## D. What to compute, per configuration

A configuration is a choice of which slots are filled. BAM's pendulum has two physical
properties, the gravity torque and the inertia about the axis, and three numbers to set them
(`--mass`, `--arm-mass`, `--length`). Choose them so that **both** match the real arm:

```
moment  = sum(m_i * r_i) + m_arm * d_arm                  # kg·m, sets the gravity torque
inertia = sum(m_i * r_i^2 + I_block_i) + I_arm            # kg·m², about the axis
L       = sum(m_i * r_i) / sum(m_i)                       # mass-weighted block radius
a       = moment / L
b       = inertia / L^2
mass     = 3*b - 2*a
arm_mass = 6*(a - b)
```

The tool does this and prints the recorder arguments:

```bash
python -m bam.equivalent_pendulum --arm-mass <m_arm> --arm-com <d_arm> \
  --arm-inertia <I_arm> --block <m_1>:<r_1> --block <m_2>:<r_2>
```

`arm_mass` may come out negative. That is expected, because it is an equivalent value, not a
physical one, and `bam.fit` and `bam.check_logs` accept it.

**Why not the mean radius.** Blocks at different radii carry `sum(m_i·r_i²)`, which is
larger than `m_load·L²` at their mean radius. An earlier version of this sheet matched only
the gravity torque, so the model's inertia came out low. The fit then puts the shortfall
into `armature`, and since the shortfall differs between configurations, the result is a
compromise rather than a constant offset. The tool prints the shortfall of those torque-only
values too.

**Logs recorded with the earlier values** can still be used. The gravity torque was right,
so only `mass` and `arm-mass` need replacing. Write corrected copies into a new directory,
keep the originals untouched, note the change, and then run `bam.process` on the copies.

---

## E. One check to report

For the lightest and the heaviest configuration, report the real inertia
(`inertia` above) next to the starting `armature` of the motor (0.027 kg·m² for XH540 and
XM540, 0.012 kg·m² for XM430). Where the load inertia is much smaller than `armature`, the
swing and the steps are dominated by the motor, which is fine for identifying `armature`
but leaves the arm values poorly checked by the free swing (PROTOCOL.md §6.4).

---

## Summary — numbers to send

```
m_arm   = ___ kg          arm alone, from Fusion
d_arm   = ___ m           axis to arm centre of mass
I_arm   = ___ kg·m²       about the rotation axis
r_upper = ___ m           axis to upper slot centres
r_lower = ___ m           axis to lower slot centres
m_block = ___ kg          one block, weighed
I_block = ___ kg·m²       one block about its own centre (optional)
```

Everything else is computed from them with `bam.equivalent_pendulum`.
