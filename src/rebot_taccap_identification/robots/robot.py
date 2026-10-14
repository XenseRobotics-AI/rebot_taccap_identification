"""Robot base class .

A subclass implements the six hardware methods; kinematics, joint count, limits and
safety come from the URDF in its config. Joint order and units of ``read``/``command``
follow the URDF: rad, rad/s, N·m, positive along each joint axis, zero at the URDF zero.
"""

from __future__ import annotations

import abc
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import RobotConfig, project_root

CALIBRATION_ROOT = Path.home() / ".cache" / "rebot_taccap_identification" / "calibration" / "robots"


@dataclass
class State:
    """One joint-space sample; arrays follow the URDF joint order."""

    q: np.ndarray                 # rad
    dq: np.ndarray                # rad/s
    tau: np.ndarray               # N·m, measured (motor feedback)
    temperature: np.ndarray       # °C, NaN if unknown
    stamp: float                  # time.perf_counter() of the reading


class Robot(abc.ABC):
    """Base class of all arms. Subclasses implement the hardware methods only."""

    def __init__(self, config: RobotConfig):
        from ..robot import RobotModel

        self.config = config
        self.id = config.id
        self.calibration_dir = Path(config.calibration_dir) if config.calibration_dir else CALIBRATION_ROOT / config.type
        self.calibration_fpath = self.calibration_dir / f"{self.id or 'default'}.json"
        self.calibration: dict = {}
        if self.calibration_fpath.is_file():
            self._load_calibration()
        self.model = RobotModel.from_urdf(config.urdf_path, gravity=tuple(config.gravity), tip_link=config.tip_link)
        self.dof = self.model.dof
        self.joint_names = list(self.model.joint_names)
        self.enabled = False

    def __str__(self) -> str:
        return f"{self.config.type}{'/' + self.id if self.id else ''} ({self.dof} 轴, {self.config.urdf_path.name})"

    # ------------------------------------------------------- hardware (implement)
    @abc.abstractmethod
    def connect(self) -> None:
        """Open the bus. Must not enable torque."""

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Release the bus. Called after ``disable``; must be safe to call twice."""

    @abc.abstractmethod
    def enable(self) -> None:
        """Switch all joints to impedance (MIT) mode and enable torque. Set ``self.enabled``."""

    @abc.abstractmethod
    def disable(self) -> None:
        """Disable torque on all joints. Set ``self.enabled = False``."""

    @abc.abstractmethod
    def read(self) -> State | None:
        """Latest feedback of all joints, or None if any joint has not reported yet."""

    @abc.abstractmethod
    def command(self, q, dq, kp, kd, tau) -> None:
        """Impedance command per joint: tau_motor = kp (q - q_meas) + kd (dq - dq_meas) + tau."""

    # ------------------------------------------------------------ optional
    def request(self) -> None:
        """Ask the motors for feedback (only needed by buses that do not reply to commands)."""

    def ping(self) -> list[tuple[str, bool, str]]:
        """Read-only check that every joint answers: (joint, ok, detail). No torque."""
        self.connect()
        try:
            deadline = time.perf_counter() + 1.0
            state = None
            while time.perf_counter() < deadline and state is None:
                self.request()
                time.sleep(0.02)
                state = self.read()
            if state is None:
                return [(name, False, "无反馈") for name in self.joint_names]
            return [(name, True, f"位置 {q:+.3f} rad") for name, q in zip(self.joint_names, state.q)]
        finally:
            self.disconnect()

    # ------------------------------------------------- derived from config + URDF
    def per_joint(self, value, what: str, default=None) -> np.ndarray | None:
        value = default if value is None else value
        if value is None:
            return None
        array = np.asarray(value, dtype=float).reshape(-1)
        if array.size not in (1, self.dof):
            raise ValueError(f"{what} 需要 1 或 {self.dof} 个值，得到 {array.size} 个")
        return np.broadcast_to(array, (self.dof,)).copy()

    def torque_limits(self) -> np.ndarray:
        limits = self.per_joint(self.config.torque_limits, "torque_limits")
        if limits is None:
            limits = np.array([j.effort if j.effort else np.nan for j in self.model.config.links], dtype=float)
        if np.any(~np.isfinite(limits)):
            missing = [n for n, v in zip(self.joint_names, limits) if not np.isfinite(v)]
            raise ValueError(f"缺少力矩上限：{', '.join(missing)}（在配置中设 torque_limits 或在 URDF 中设 effort）")
        return limits

    def clip_gains(self, kp, kd) -> tuple[np.ndarray, np.ndarray]:
        kp, kd = self.per_joint(kp, "kp"), self.per_joint(kd, "kd")
        kp_max = self.per_joint(self.config.kp_max, "kp_max", np.inf)
        kd_max = self.per_joint(self.config.kd_max, "kd_max", np.inf)
        return np.clip(kp, 0.0, kp_max), np.clip(kd, 0.0, kd_max)

    def safety_window(self, margin_deg: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """URDF limits intersected with ``soft_limits_deg`` [rad], shrunk by ``margin_deg``."""
        lower = np.array([j.lower if j.lower is not None else -np.pi for j in self.model.config.links])
        upper = np.array([j.upper if j.upper is not None else np.pi for j in self.model.config.links])
        if self.config.soft_limits_deg is not None:
            soft = np.radians(np.asarray(self.config.soft_limits_deg, dtype=float))
            if soft.shape != (self.dof, 2):
                raise ValueError(f"soft_limits_deg 需要 {self.dof} 组 [下限, 上限]")
            lower, upper = np.maximum(lower, soft[:, 0]), np.minimum(upper, soft[:, 1])
        return lower + np.radians(margin_deg), upper - np.radians(margin_deg)

    def rest(self) -> np.ndarray:
        return np.radians(self.per_joint(self.config.rest_deg, "rest_deg"))

    def results_dir(self) -> Path:
        if self.config.results_dir:
            return self.config.resolve(self.config.results_dir)
        return project_root() / "results" / self.robot_id()

    def robot_id(self) -> str:
        """``<type>`` or ``<type>_<id>``: names this arm's results."""
        return self.config.type + (f"_{self.id}" if self.id else "")

    def model_prefix(self) -> str:
        return self.config.name or self.config.urdf_path.stem

    # ---------------------------------------------------------- calibration
    def _load_calibration(self) -> None:
        self.calibration = json.loads(self.calibration_fpath.read_text())

    def _save_calibration(self) -> None:
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath.write_text(json.dumps(self.calibration, indent=2))

    @property
    def is_calibrated(self) -> bool:
        return "torque_gain" in self.calibration

    def torque_gain(self) -> np.ndarray:
        """measured / actual joint torque: config value, else the saved calibration, else 1."""
        value = self.config.torque_gain
        if value is None:
            value = self.calibration.get("torque_gain", 1.0)
        return self.per_joint(value, "torque_gain")

    def save_torque_gain(self, gain) -> Path:
        self.calibration["torque_gain"] = [float(v) for v in self.per_joint(gain, "torque_gain")]
        self._save_calibration()
        return self.calibration_fpath
