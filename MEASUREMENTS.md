# What to Measure Before Recording

The pendulum model BAM fits assumes a **uniform rod** carrying a **point mass at its tip**.
The arm in this design is not uniform — its mass sits mostly in the cage at the far end — so
the numbers passed to the recorder have to be equivalent values, not raw ones. Everything
below is measured once per arm, not per configuration.

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

Weigh them individually if they are not identical, and keep them labelled.

---

## D. What to compute, per configuration

A configuration is a choice of which slots are filled.

**Total load**

```
m_load = sum of the blocks fitted
```

**Load radius** — the mass-weighted mean of the filled slot centres. With identical blocks
this is the plain average:

```
L = (r_i + r_j + ...) / (number of blocks fitted)
```

**Equivalent arm mass** — makes BAM's gravity torque match the real one:

```
m_arm_eq = 2 * m_arm * d_arm / L
```

These three are what go to the recorder:

```bash
--mass <m_load>  --arm-mass <m_arm_eq>  --length <L>
```

---

## E. One check to report

The equivalent arm mass fixes the gravity torque exactly, but the inertia will not match
perfectly. Compute both and compare:

```
I_bam  = m_load * L^2 + (m_arm_eq / 3) * L^2
I_true = m_load * L^2 + I_arm
```

Report `I_true / I_bam` for the lightest and the heaviest configuration. Below about 1.10 the
residual is small enough to ignore. Above that, the arm is too heavy relative to the load and
either the cage should be lightened or heavier blocks used.

---

## Summary — numbers to send

```
m_arm   = ___ kg          arm alone, from Fusion
d_arm   = ___ m           axis to arm centre of mass
I_arm   = ___ kg·m²       about the rotation axis
r_upper = ___ m           axis to upper slot centres
r_lower = ___ m           axis to lower slot centres
m_block = ___ kg          one block, weighed
```

Six numbers. Everything else is computed from them.
