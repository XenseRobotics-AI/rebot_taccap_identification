"""Bounded integral holding control for static data collection with an unknown load.

Payload data is collected before a payload model exists, so the gravity feedforward (the
arm model, plus a CAD guess of the payload or nothing at all) can be wrong by more than the
holding stiffness tolerates within the tracking limit. A bounded, rate-limited integral of
the position error learns the missing torque at each pose instead; it does not depend on
the mass or CoM of whatever is mounted.

The correction is a temporary control state, never a fitted model or an extra term in the
measured training torques: static measurements still use motor feedback, and the
correction is frozen while they are averaged. This mode is for collision-free
identification motions, not contact or teleoperation.
"""

from __future__ import annotations

import numpy as np

from .common import Controller, SafetyStop


class LoadBalancedController(Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not np.isfinite(self.limits.max_tracking) or self.limits.max_tracking <= 0:
            raise ValueError("负载静态采集需要有限且大于零的跟踪误差保护")
        # Stiffness that holds half the torque range within the tracking limit before the
        # integral catches up, raised only as far as kd can follow (kd ∝ sqrt(kp) keeps the
        # damping ratio); joints whose kd is already at its hardware range keep their gains.
        kd_max = self.arm.per_joint(self.arm.config.kd_max, "kd_max", np.inf)
        wanted = .5 * self.tmax / self.limits.max_tracking / np.maximum(self.kp, 1e-9)
        ratio = np.clip(wanted, 1.0, (kd_max / np.maximum(self.kd, 1e-9)) ** 2)
        self.kp, self.kd = self.arm.clip_gains(self.kp * ratio, self.kd * np.sqrt(ratio))
        self.correction = np.zeros(self.dof)        # command torque units, not feedback units
        self.correction_limit = .5 * self.tmax
        self.correction_slew = .5 * self.tmax       # N·m/s
        self.integral_gain = 2.0 * self.kp          # N·m/rad/s: integral time constant 0.5 s
        self.frozen = False                         # True while static samples are averaged

    def send(self, q, dq, tau, kp=None, kd=None) -> None:
        q, dq, tau = (np.asarray(v, dtype=float) for v in (q, dq, tau))
        state = self.arm.read()
        # A missing sample skips one update; the monitor handles stale feedback.
        if state is not None and not self.frozen:
            kp, kd = self.arm.clip_gains(self.kp if kp is None else kp, self.kd if kd is None else kd)
            error = q - state.q
            step = np.clip(self.integral_gain * error, -self.correction_slew, self.correction_slew) / self.rate
            total = tau / self.torque_gain + self.correction + kp * error + kd * (dq - state.dq)
            # Conditional integration: never wind further into actuator saturation.
            bound = self.limits.torque_ratio * self.tmax
            step[((total >= bound) & (step > 0)) | ((total <= -bound) & (step < 0))] = 0.0
            self.correction = np.clip(self.correction + step, -self.correction_limit, self.correction_limit)
        # The base sender divides by torque_gain; scale the command-space correction back
        # so the division is performed exactly once.
        super().send(q, dq, tau + self.correction * self.torque_gain, kp=kp, kd=kd)

    def engage(self, ramp: float = 0.5) -> np.ndarray:
        print("[负载平衡] 积分补偿模型未包含的负载力矩，不依赖夹爪质量/质心", flush=True)
        print(f"[负载平衡] kp={np.round(self.kp, 1).tolist()} kd={np.round(self.kd, 2).tolist()}；"
              f"补偿上限 {np.round(self.correction_limit, 2).tolist()} N·m", flush=True)
        super().engage(ramp)
        # Balance where the stiffness ramp left the arm: an unknown load may have pulled it
        # off the latched pose before the integral caught up.
        q_hold = self.wait_state().q.copy()
        self.settle(q_hold)
        return q_hold

    def settle(self, q, timeout: float = 6.0) -> None:
        """Hold ``q`` until the arm is still and close to it, before static data is accepted."""
        q = np.asarray(q, dtype=float)
        tau, zero = self.dyn.gravity(q)[0], np.zeros(self.dof)
        close = min(np.radians(1.0), self.limits.max_tracking / 3)
        stable, previous = 0.0, None
        for _ in range(max(1, int(timeout * self.rate))):
            state = self.state()
            self.monitor.check(state, q)
            if state is None:
                stable, previous = 0.0, None
            else:
                # Position travel, not the noisy firmware velocity, tells whether the arm moves.
                quiet = previous is not None and np.max(np.abs(state.q - previous)) * self.rate < np.radians(3)
                stable = stable + 1 / self.rate if quiet and np.max(np.abs(q - state.q)) < close else 0.0
                previous = state.q.copy()
            self.send(q, zero, tau)
            self.ticker.wait()
            if stable >= .4:
                return
        raise SafetyStop("负载平衡未收敛，停止采集；检查负载是否超出力矩能力、机械干涉或反馈异常")
