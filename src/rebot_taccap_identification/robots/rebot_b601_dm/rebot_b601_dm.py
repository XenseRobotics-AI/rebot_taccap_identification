"""DM MIT interface using motorbridge, with a read-only connection check.

motorbridge 0.5.6 exposes cached status without a receive timestamp. A separate
register-read watchdog requires a *new reply* from each joint, including stationary
joints. It detects communication loss, not the age of an individual status frame.
No zero-setting, parameter storage, or automatic fault clearing is performed here.
"""

import logging
import threading
import time

import numpy as np

from ..robot import Robot, State
from .config_rebot_b601_dm import MODEL_LIMITS, RebotB601DMConfig

logger = logging.getLogger(__name__)


class _RegisterTimeout(TimeoutError):
    """One read-only heartbeat request timed out; transport/protocol errors are separate."""


class RebotB601DM(Robot):
    def __init__(self, config: RebotB601DMConfig):
        super().__init__(config)
        if len(config.motors) != self.dof:
            raise ValueError(f"motors 需要 {self.dof} 项，按 URDF 关节顺序排列")
        self.specs = [{"direction": 1, "offset_deg": 0, "feedback_id": 0x10 + int(m["id"]), **m}
                      for m in config.motors]
        for spec in self.specs:
            if spec["model"] not in MODEL_LIMITS:
                raise ValueError("B601 DM 支持的电机型号：4340P / 4310；必须与实机匹配")
            if spec["direction"] not in (-1, 1) or not np.isfinite(spec["offset_deg"]):
                raise ValueError("direction 必须为 ±1，offset_deg 必须有限")
            if not 1 <= spec["id"] <= 15 or not 1 <= spec["feedback_id"] <= 0x7FF:
                raise ValueError("DM motor id 范围 1..15；feedback_id 范围 1..0x7FF")
        for field in ("id", "feedback_id"):
            if len({s[field] for s in self.specs}) != self.dof:
                raise ValueError(f"DM {field} 不能重复")
        self.direction = np.array([s["direction"] for s in self.specs])
        self.offset = np.radians([s["offset_deg"] for s in self.specs])
        self.ranges = np.array([MODEL_LIMITS[s["model"]] for s in self.specs])
        # URDF effort and encoding range are ceilings, not a continuous-duty rating.
        limits = super().torque_limits()
        if np.any(limits <= 0) or np.any(limits > self.ranges[:, 0]):
            raise ValueError("torque_limits 必须为正且不超过对应 DM 电机的 MIT 量程")
        for field, column in (("kp_max", 2), ("kd_max", 3)):
            value = self.per_joint(getattr(config, field), field, self.ranges[:, column])
            if not np.all(np.isfinite(value)) or np.any(value < 0) or np.any(value > self.ranges[:, column]):
                raise ValueError(f"{field} 超出 DM MIT 量程")
            setattr(config, field, value.tolist())
        self.ctrl = None
        self.motors = []
        self._stop = threading.Event()
        self._health_stop = threading.Event()
        self._poller = self._watchdog = None
        self._error = None
        self._last_reply = np.zeros(self.dof)
        self._probe_errors = [""] * self.dof
        self.channel = config.channel

    def connect(self):
        if self.ctrl is not None:
            return
        from motorbridge import Controller, Mode

        self._mode = Mode
        self.channel = self.config.channel
        print(f"[DM] {self.config.transport}: {self.channel}", flush=True)
        self._error = None
        self._last_reply[:] = 0
        self._probe_errors = [""] * self.dof
        self._stop.clear()
        self._health_stop.clear()
        try:
            self.ctrl = (Controller.from_dm_serial(self.channel, self.config.dm_serial_baud)
                         if self.config.transport == "serial" else Controller(self.channel))
            for s in self.specs:
                self.motors.append(self.ctrl.add_damiao_motor(int(s["id"]), int(s["feedback_id"]), s["model"]))
            self._poller = threading.Thread(target=self._poll, name="dm-feedback", daemon=True)
            self._poller.start()
        except BaseException:
            self.disconnect()
            raise

    def _poll(self):
        while not self._stop.is_set():
            try:
                self.ctrl.poll_feedback_once()
            except Exception as exc:
                self._error = f"DM 接收失败：{exc}"
                return
            self._stop.wait(0.0005)

    def _probe(self, i):
        motor = self.motors[i]
        # ESC_ID is read-only here. motorbridge discards cached register replies.
        try:
            actual = motor.get_register_u32(8, timeout_ms=80)
        except Exception as exc:
            # motorbridge 0.5.6 flattens native Timeout into CallError. Match only
            # this register-read timeout, not send failures or malformed replies.
            if isinstance(exc, TimeoutError) or str(exc).startswith(
                    "get_register_u32 failed: register 8 not received within "):
                raise _RegisterTimeout(str(exc)) from exc
            raise
        if actual != self.specs[i]["id"]:
            raise RuntimeError(f"ESC_ID={actual}，期望 {self.specs[i]['id']}")
        # A reply arriving after the per-joint deadline must not revive the arm.
        self._check_health()
        motor.request_feedback()
        self._last_reply[i] = time.perf_counter()

    def _heartbeat_once(self):
        for i, name in enumerate(self.joint_names):
            if self._health_stop.is_set():
                return False
            try:
                self._check_health()
                try:
                    self._probe(i)
                except _RegisterTimeout as exc:
                    first_miss = not self._probe_errors[i]
                    self._probe_errors[i] = str(exc)
                    self._check_health()
                    if first_miss:
                        logger.warning("%s DM 寄存器应答超时，继续重试；自上次应答 %.3fs，通信上限 %.3fs：%s",
                                       name, time.perf_counter() - self._last_reply[i],
                                       self.config.feedback_timeout_s, exc)
                    # Do not renew last_reply on failure. Other joints still get
                    # probed; a single missing joint retains its own deadline.
                    continue
                if self._probe_errors[i]:
                    logger.info("%s DM 寄存器应答恢复", name)
                self._probe_errors[i] = ""
            except Exception as exc:
                self._error = self._error or f"{name} DM 通信中断：{exc}"
                return False
        return True

    def _heartbeat(self):
        while not self._health_stop.is_set():
            if not self._heartbeat_once():
                return
            self._health_stop.wait(0.1)

    def _verify_motor(self, i):
        self._probe(i)
        motor, spec = self.motors[i], self.specs[i]
        feedback_id = motor.get_register_u32(7, timeout_ms=200)
        actual = [motor.get_register_f32(rid, timeout_ms=200) for rid in (21, 22, 23)]
        expected = [12.5, self.ranges[i, 1], self.ranges[i, 0]]
        if feedback_id != spec["feedback_id"] or not np.allclose(actual, expected, rtol=1e-4, atol=1e-4):
            raise ValueError(f"{self.joint_names[i]} DM 配置不匹配：feedback_id={feedback_id}, "
                             f"PMAX/VMAX/TMAX={actual}；期望 {spec['feedback_id']}, {expected}")

    def _wait_status(self, expected):
        from ...runtime.common import SafetyStop

        deadline = time.perf_counter() + self.config.feedback_timeout_s
        while time.perf_counter() < deadline:
            self.request()
            self.read()  # bus errors, fault codes, NaN
            states = [m.get_state() for m in self.motors]
            if all(s is not None and s.status_code == expected for s in states):
                return
            time.sleep(0.01)
        raise SafetyStop(f"DM 状态确认超时：期望 status_code={expected}")

    def _check_health(self):
        from ...runtime.common import SafetyStop

        if self._error:
            raise SafetyStop(self._error)
        if self.enabled:
            age = time.perf_counter() - self._last_reply
            stale = age >= self.config.feedback_timeout_s
            if stale.any():
                i = int(np.argmax(age))
                detail = f"；最近错误：{self._probe_errors[i]}" if self._probe_errors[i] else ""
                self._error = (f"{self.joint_names[i]} DM 通信中断：{age[i]:.3f}s 无新寄存器应答，"
                               f"上限 {self.config.feedback_timeout_s:.3f}s{detail}")
                raise SafetyStop(self._error)

    def request(self):
        for motor in self.motors:
            motor.request_feedback()

    def read(self):
        from ...runtime.common import SafetyStop

        self._check_health()
        states = [m.get_state() for m in self.motors]
        if not states or any(s is None for s in states):
            return None
        for name, s in zip(self.joint_names, states):
            if s.status_code not in (0, 1):
                raise SafetyStop(f"{name} DM 故障码 0x{s.status_code:X}；不会自动清故障/重新使能")
            if self.enabled and s.status_code != 1:
                raise SafetyStop(f"{name} DM 意外失能")
        values = np.array([[s.pos, s.vel, s.torq, s.t_mos, s.t_rotor] for s in states])
        if not np.all(np.isfinite(values)):
            raise SafetyStop("DM 反馈包含非有限数值")
        return State(self.direction * (values[:, 0] - self.offset), self.direction * values[:, 1],
                     self.direction * values[:, 2], values[:, 3:].max(axis=1), time.perf_counter())

    def enable(self):
        self.connect()
        try:
            for i in range(self.dof):
                self._verify_motor(i)
            # Catch existing faults before changing modes. Do not clear errors.
            self.read()
            for motor in self.motors:
                motor.disable()
            self._wait_status(0)
            for motor in self.motors:
                motor.ensure_mode(self._mode.MIT)
            for motor in self.motors:
                motor.enable()
            self._wait_status(1)
            for i in range(self.dof):
                self._probe(i)
            self.enabled = True
            self.read()
            self._health_stop.clear()
            self._watchdog = threading.Thread(target=self._heartbeat, name="dm-watchdog", daemon=True)
            self._watchdog.start()
        except BaseException:
            self.disable()  # includes partially enabled motors
            raise

    def command(self, q, dq, kp, kd, tau):
        from ...runtime.common import SafetyStop

        if not self.enabled:
            raise SafetyStop("DM 未使能，拒绝发送 MIT 指令")
        self._check_health()
        arrays = [self.per_joint(v, key) for v, key in zip((q, dq, kp, kd, tau), ("q", "dq", "kp", "kd", "tau"))]
        q, dq, kp, kd, tau = arrays
        if not np.all(np.isfinite(arrays)):
            raise SafetyStop("MIT 指令包含非有限数值")
        motor_q, motor_dq, motor_tau = self.direction * q + self.offset, self.direction * dq, self.direction * tau
        if (np.any(np.abs(motor_q) > 12.5) or np.any(np.abs(motor_dq) > self.ranges[:, 1])
                or np.any(np.abs(motor_tau) > self.torque_limits()) or np.any(kp < 0) or np.any(kd < 0)
                or np.any(kp > self.config.kp_max) or np.any(kd > self.config.kd_max)):
            raise SafetyStop("MIT 指令超出 DM 配置/协议范围")
        for i, motor in enumerate(self.motors):
            motor.send_mit(float(motor_q[i]), float(motor_dq[i]), float(kp[i]), float(kd[i]), float(motor_tau[i]))

    def disable(self):
        self._health_stop.set()
        if self._watchdog is not None:
            self._watchdog.join()
            self._watchdog = None
        try:
            for motor in self.motors:
                try:
                    motor.disable()
                except Exception as exc:
                    logger.error("DM 失能指令未确认：%s", exc)
        finally:
            self.enabled = False

    def disconnect(self):
        if self.enabled:
            self.disable()
        self._health_stop.set()
        if self._watchdog is not None:
            self._watchdog.join()
            self._watchdog = None
        self._stop.set()
        if self._poller is not None:
            self._poller.join()
            self._poller = None
        # Do not call shutdown(): read-only ping must not send disable commands.
        for motor in self.motors:
            motor.close()
        self.motors = []
        if self.ctrl is not None:
            self.ctrl.close()
            self.ctrl = None

    def ping(self):
        self.connect()
        out = []
        try:
            for i, (name, spec) in enumerate(zip(self.joint_names, self.specs)):
                try:
                    self._verify_motor(i)
                    detail = f"{self.channel} id=0x{spec['id']:02X} feedback=0x{spec['feedback_id']:02X} {spec['model']}"
                    out.append((name, True, detail))
                except Exception as exc:
                    out.append((name, False, str(exc)))
        finally:
            self.disconnect()
        return out
