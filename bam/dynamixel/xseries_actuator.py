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

# Actuator definitions for the DYNAMIXEL X series used on the Robinion2S humanoid:
# XH540-W270 (legs), XM540-W270 (shoulders, elbow yaw, hip yaw) and XM430-W350 (elbow pitch).
#
# These are voltage-controlled, position-controlled units, so they follow the same pattern as
# BAM's MXActuator: the firmware P gain is a known input, not a fitted parameter, and what is
# identified is the motor (kt, R), the reflected rotor inertia (armature) and the friction.

import numpy as np

from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench

# Encoder resolution, shared by the whole X series.
X_ENCODER_COUNTS_PER_REV = 4096

# The firmware divides the Position P Gain register before it reaches the PWM stage. The
# e-Manual gives 128; BAM found 256 for the XL330, so this is the first thing to check if the
# fit is poor in a way that scales with the P gain. Section "Verifying error_gain" of the
# protocol sheet says how to tell.
X_KP_DIVISOR = 128

# Default PWM Limit register value: 885 counts at 0.113 % per count is 100 % duty.
X_PWM_LIMIT = 885

# error [rad] -> counts -> PWM counts (via kp/divisor) -> duty fraction
X_ERROR_GAIN = (X_ENCODER_COUNTS_PER_REV / (2 * np.pi)) / (X_KP_DIVISOR * X_PWM_LIMIT)


class _XSeriesActuator(VoltageControlledActuator):
    """Common structure; subclasses only supply the catalogue-derived starting point.

    ``kt`` and ``R`` are initialised from the 12 V catalogue figures through
    ``kt = V / omega_noload`` and ``R = kt * V / tau_stall``, with bounds wide enough that the
    fit is not pinned to the catalogue. Both are properties of the magnets and windings, so
    they do not depend on the supply: the same starting values are right whether the bench
    runs at 12 V or at the 14.8 V default below. A fitted value far outside these bounds means
    the bench moved, the load was mis-measured, or ``error_gain`` is wrong.
    """

    #: (kt_guess, R_guess, armature_guess) and the bounds applied around them
    KT = R = ARMATURE = None

    def __init__(self, testbench_class: Testbench, kp: int = 800, vin: float = 14.8):
        super().__init__(
            testbench_class,
            vin=vin,
            kp=kp,
            error_gain=X_ERROR_GAIN,
            max_pwm=1.0,
        )

    def initialize(self):
        kt, R, arm = self.KT, self.R, self.ARMATURE
        self.model.kt = Parameter(kt, 0.4 * kt, 2.5 * kt)
        self.model.R = Parameter(R, 0.4 * R, 2.5 * R)
        # The URDF carries link inertia only; this is the term it omits. The lower bound is
        # kept well below any plausible value so the fit can report a small one rather than
        # being held up by the prior.
        self.model.armature = Parameter(arm, 0.0005, 0.20)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class XH540W270Actuator(_XSeriesActuator):
    """XH540-W270. 12 V catalogue: 9.9 N.m stall, 39 rev/min no load, 272.5:1."""

    KT, R, ARMATURE = 2.94, 3.56, 0.027


class XM540W270Actuator(_XSeriesActuator):
    """XM540-W270. 12 V catalogue: 10.6 N.m stall, 30 rev/min no load, 272.5:1."""

    KT, R, ARMATURE = 3.82, 4.32, 0.027


class XM430W350Actuator(_XSeriesActuator):
    """XM430-W350. 12 V catalogue: 4.1 N.m stall, 46 rev/min no load, 353.5:1."""

    KT, R, ARMATURE = 2.49, 7.29, 0.012
