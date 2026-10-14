"""reBot Arm B601 RobStride: six MIT-mode motors on one CAN bus, through the motorbridge SDK.

Each entry of ``config.motors`` is one joint in URDF order::

    {"id": 0x01,
     "feedback_id": 0xFD,
     "model": "rs-06",
     "direction": 1,          # -1 if the motor turns opposite to the URDF axis
     "offset_deg": 0.0}       # motor angle at the URDF zero pose

Motor coordinates are mapped to URDF ones as q = direction * (motor - offset).
"""

from __future__ import annotations

import threading
import time

import numpy as np

from ..robot import Robot, State
from .config_rebot_b601_rs import MODEL_LIMITS, RebotB601RSConfig


class RebotB601RS(Robot):

    def __init__(self, config: RebotB601RSConfig):
        super().__init__(config)
        if len(config.motors) != self.dof:
            raise ValueError(f"配置了 {len(config.motors)} 个电机，URDF 有 {self.dof} 个关节")
        defaults = dict(feedback_id=0xFD, direction=1, offset_deg=0.0)
        self.specs = [{**defaults, **m} for m in config.motors]
        self.direction = np.array([float(m["direction"]) for m in self.specs])
        self.offset = np.radians([float(m["offset_deg"]) for m in self.specs])
        known = [MODEL_LIMITS.get(m["model"]) for m in self.specs]
        if config.torque_limits is None and all(known):
            config.torque_limits = [k[0] for k in known]
        if config.kp_max is None and all(known):
            config.kp_max = [k[1] for k in known]
        if config.kd_max is None and all(known):
            config.kd_max = [k[2] for k in known]
        self.ctrl = None
        self.motors = []
        self._stop = threading.Event()
        self._poller = None

    # ----------------------------------------------------------------- bus
    def connect(self) -> None:
        if self.ctrl is not None:
            return
        try:
            from motorbridge import Controller, Mode
        except ImportError as exc:
            raise RuntimeError("需要 motorbridge：pip install motorbridge") from exc
        self._mode = Mode
        self.ctrl = Controller(self.config.channel)
        self.motors = [self.ctrl.add_robstride_motor(int(m["id"]), int(m["feedback_id"]), m["model"])
                       for m in self.specs]
        # Status frames and ACKs are only processed while someone drains the bus.
        self._stop.clear()
        self._poller = threading.Thread(target=self._poll, name="motorbridge-poll", daemon=True)
        self._poller.start()

    def _poll(self) -> None:
        while not self._stop.is_set():
            try:
                self.ctrl.poll_feedback_once()
            except Exception:  # noqa: BLE001 - transient bus errors must not kill the drain
                pass
            time.sleep(0.0005)

    def disconnect(self) -> None:
        if self.ctrl is None:
            return
        self._stop.set()
        if self._poller is not None:
            self._poller.join(timeout=0.5)
        try:
            self.ctrl.shutdown()
        except Exception:  # noqa: BLE001
            pass
        self.ctrl.close()
        self.ctrl, self.motors = None, []

    # ----------------------------------------------------------- interface
    def read(self) -> State | None:
        states = [m.get_state() for m in self.motors]
        if not states or any(s is None for s in states):
            return None
        pos = np.array([s.pos for s in states])
        return State(self.direction * (pos - self.offset), self.direction * np.array([s.vel for s in states]),
                     self.direction * np.array([s.torq for s in states]), np.array([s.t_mos for s in states]),
                     time.perf_counter())

    def enable(self) -> None:
        self.connect()
        try:
            self.ctrl.disable_all()
        except Exception as exc:  # noqa: BLE001 - no motor acknowledged anything
            from ...runtime.common import SafetyStop
            raise SafetyStop(
                f"电机无应答（{exc}）。请检查：电机电源/急停、CAN 线；然后重启 CAN 口清掉卡住的发送队列："
                f"sudo ip link set {self.config.channel} down && "
                f"sudo ip link set {self.config.channel} up type can bitrate 1000000") from exc
        for m in self.motors:
            for attempt in range(10):
                try:
                    m.ensure_mode(self._mode.MIT)
                    break
                except Exception:
                    if attempt == 9:
                        raise
                    time.sleep(0.05)
        self.ctrl.enable_all()
        self.enabled = True

    def command(self, q, dq, kp, kd, tau) -> None:
        motor_q = np.asarray(q, dtype=float) * self.direction + self.offset
        motor_dq = np.asarray(dq, dtype=float) * self.direction
        motor_tau = np.asarray(tau, dtype=float) * self.direction
        for i, m in enumerate(self.motors):
            m.send_mit(float(motor_q[i]), float(motor_dq[i]), float(kp[i]), float(kd[i]), float(motor_tau[i]))

    def disable(self) -> None:
        try:
            if self.ctrl is not None:
                self.ctrl.disable_all()
        finally:
            self.enabled = False

    def ping(self) -> list[tuple[str, bool, str]]:
        self.connect()
        time.sleep(0.1)
        out = []
        try:
            for name, spec, motor in zip(self.joint_names, self.specs, self.motors):
                try:
                    motor.robstride_ping()
                    state = motor.get_state()
                    detail = f"id 0x{int(spec['id']):02X} {spec['model']}"
                    if state is not None:
                        detail += f"  位置 {state.pos:+.3f} rad  MOS {state.t_mos:.0f}°C"
                    out.append((name, True, detail))
                except Exception as exc:  # noqa: BLE001
                    out.append((name, False, f"id 0x{int(spec['id']):02X} 无应答（{exc}）"))
        finally:
            self.disconnect()
        return out
