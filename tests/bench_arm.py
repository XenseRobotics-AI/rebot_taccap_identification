"""Test-only arms: ``bench_test_arm`` tracks ideally and measures the gravity of an arm 10 %
heavier than its URDF + offset + noise; ``dynamic_test_arm`` has to be held against a load
its model lacks.

Registered so the hardware workflow can be exercised end to end without hardware. Not part
of the package.
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


@RobotConfig.register_subclass("dynamic_test_arm")
@dataclass(kw_only=True)
class DynamicArmConfig(RobotConfig):
    load_mass: float = 0.0                    # load on the tip link that no model knows about [kg]
    load_com: tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia: float = 0.05                     # per joint, reflected rotor inertia dominates [kg m^2]
    friction: float = 0.1                     # Coulomb = static friction [N·m]


class DynamicArm(Robot):
    """A test-only arm that has to be held: the MIT law runs at 1 kHz on joints with diagonal
    inertia and Coulomb friction, against the gravity of the URDF arm plus a hidden load.
    The measured torque is the motor torque."""

    def __init__(self, config):
        from rebot_taccap_identification.payload import payload_parameters

        super().__init__(config)
        self.truth = self.model.nominal_parameters[:, :10].copy()
        self.truth[-1] += payload_parameters(config.load_mass, config.load_com, np.zeros((3, 3)))
        self.q, self.dq, self.tau = self.rest(), np.zeros(self.dof), np.zeros(self.dof)

    def connect(self): pass
    def disconnect(self): pass
    def enable(self): self.enabled = True
    def disable(self): self.enabled = False

    def read(self):
        return State(self.q.copy(), self.dq.copy(), self.tau.copy(), np.full(self.dof, 40.0), time.perf_counter())

    def command(self, q, dq, kp, kd, tau):
        """Advance one command period."""
        steps, f = 10, self.config.friction
        h, limit = 1.0 / (self.config.rate * steps), self.torque_limits()
        g = gravity_torque(self.model, self.q[None], self.truth)[0]
        for _ in range(steps):
            motor = np.clip(kp * (q - self.q) + kd * (dq - self.dq) + tau, -limit, limit)
            net = motor - g
            moving = np.abs(self.dq) > 1e-4
            dq_new = self.dq + h * (net - np.where(moving, f * np.sign(self.dq), np.clip(net, -f, f))) / self.config.inertia
            self.dq = np.where(moving & (np.sign(dq_new) != np.sign(self.dq)), 0.0, dq_new)   # sticks at reversal
            self.q = self.q + h * self.dq
        self.tau = motor
