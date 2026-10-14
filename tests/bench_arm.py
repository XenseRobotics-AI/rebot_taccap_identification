"""A test-only arm: ideal position tracking, measured torque = gravity of an arm 10 % heavier
than its URDF + offset + noise.

Registered as ``bench_test_arm`` so the hardware workflow can be exercised end to end
without hardware. Not part of the package.
"""

import time
from dataclasses import dataclass

import numpy as np

from rebot_taccap_identification.gravity import gravity_torque
from rebot_taccap_identification.robots import Robot, RobotConfig, State


@RobotConfig.register_subclass("bench_test_arm")
@dataclass(kw_only=True)
class BenchArmConfig(RobotConfig):
    offset: float = 0.05
    noise: float = 0.005


class BenchArm(Robot):
    def __init__(self, config):
        super().__init__(config)
        self.truth = self.model.nominal_parameters[:, :10] * 1.1      # the real arm is 10 % heavier than the CAD
        self.rng = np.random.default_rng(0)
        self.q, self.dq, self.tau = self.rest(), np.zeros(self.dof), np.zeros(self.dof)

    def connect(self): pass
    def disconnect(self): pass
    def enable(self): self.enabled = True
    def disable(self): self.enabled = False

    def read(self):
        return State(self.q.copy(), self.dq.copy(), self.tau.copy(), np.full(self.dof, 40.0), time.perf_counter())

    def command(self, q, dq, kp, kd, tau):
        q, dq = np.asarray(q, float), np.asarray(dq, float)
        if not np.any(kp):                    # free mode: the arm stays where it is
            q, dq = self.q, np.zeros(self.dof)
        self.q, self.dq = q.copy(), dq.copy()
        self.tau = (gravity_torque(self.model, q[None], self.truth)[0] + self.config.offset
                    + self.rng.normal(scale=self.config.noise, size=self.dof))
