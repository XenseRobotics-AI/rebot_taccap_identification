"""B601 DM arm, bare joint-6 output + C1 + external TacCap.

Coordinates are the DM motor coordinates: J2/J3 have negative working angles.
The default gains follow libpyrebot's DM configuration, not the RS configuration.
"""

from dataclasses import dataclass, field
from math import pi
from pathlib import Path

from ..config import RobotConfig, project_root

# MIT encoding ranges, NOT continuous torque ratings: (torque Nm, velocity rad/s, kp, kd).
MODEL_LIMITS = {"4340P": (28.0, 10.0, 500.0, 5.0), "4310": (10.0, 30.0, 500.0, 5.0)}
URDF = Path(__file__).parent / "urdf/rebot_b601_dm/urdf/rebot_b601_dm.urdf"


def _motors():
    return [{"id": i, "feedback_id": 0x10 + i, "model": "4340P" if i <= 3 else "4310"}
            for i in range(1, 7)]


@RobotConfig.register_subclass("rebot_b601_dm")
@dataclass(kw_only=True)
class RebotB601DMConfig(RobotConfig):
    urdf: str = "package://rebot_taccap_identification/robots/rebot_b601_dm/urdf/rebot_b601_dm/urdf/rebot_b601_dm.urdf"
    tip_link: str | None = "link6"
    name: str | None = "rebot_b601_dm"
    transport: str = "serial"                      # serial / socketcan
    channel: str = "/dev/ttyACM0"                  # serial: /dev/...; socketcan: can0
    dm_serial_baud: int = 921600
    feedback_timeout_s: float = 0.5                # per-motor communication watchdog
    motors: list = field(default_factory=_motors)  # URDF order; optional direction, offset_deg
    kp: object = field(default_factory=lambda: [45, 45, 45, 8, 9, 8])
    kd: object = field(default_factory=lambda: [5, 5, 5, 1, 1, 1])
    soft_limits_deg: list | None = field(
        default_factory=lambda: [[-145, 145], [-170, 0], [-170, 0], [-100, 80], [-85, 85], [-130, 130]])
    gripper_repo: str = ""
    gripper_urdf: str | None = None
    gripper_parent_link: str = "adapter_link"
    gripper_xyz: tuple[float, float, float] = (0.0, 0.014, 0.0)
    gripper_rpy: tuple[float, float, float] = (0.0, pi / 2, pi / 2)

    def __post_init__(self):
        if self.transport not in ("serial", "socketcan"):
            raise ValueError("transport: serial / socketcan")
        if self.transport == "socketcan" and (self.channel.startswith("auto:") or "/" in self.channel):
            raise ValueError("SocketCAN 需要显式 channel（如 can0）")
        if self.transport == "serial" and not self.channel.startswith("/"):
            raise ValueError("串口 channel 需要设备路径（如 /dev/ttyACM0 或 /dev/serial/by-path/…）")
        if self.dm_serial_baud <= 0 or not 0.3 <= self.feedback_timeout_s <= 2.0:
            raise ValueError("dm_serial_baud 必须为正；feedback_timeout_s 范围 [0.3, 2.0]")

    @property
    def gripper_urdf_path(self) -> Path:
        from ...assets import gripper_urdf

        if self.gripper_urdf:
            return self.resolve(self.gripper_urdf)
        repository = self.resolve(self.gripper_repo) if self.gripper_repo else project_root() / "third_party/XTac-UMI-Description"
        return gripper_urdf(repository)

    @property
    def urdf_path(self) -> Path:
        from ...assets import compose_urdf

        arm = super().urdf_path
        if arm.resolve() != URDF.resolve():
            return arm
        return compose_urdf(arm, self.gripper_urdf_path, parent_link=self.gripper_parent_link,
                            xyz=self.gripper_xyz, rpy=self.gripper_rpy)
