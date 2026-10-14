"""reBot Arm B601 with RobStride (灵足) motors, direct motor output, C1 adapter and TacCap G1 gripper.

The arm and C1 adapter ship with this package; the gripper comes from the external
XTac-UMI-Description repository. ``link6`` is the bare joint-6 output frame (no mass);
the C1 adapter and the externally mounted gripper hang below it and are the payload
(removed for the arm identification). Motor coordinates equal the URDF ones (direction 1,
offset 0). Gains are holding gains for use with gravity feedforward.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import pi
from pathlib import Path

from ..config import RobotConfig, project_root


# MIT ranges of RobStride models: (torque [N·m], kp max, kd max).
MODEL_LIMITS = {
    "rs-00": (14.0, 500.0, 5.0), "rs-01": (17.0, 500.0, 5.0), "rs-02": (17.0, 500.0, 5.0),
    "rs-03": (60.0, 5000.0, 100.0), "rs-04": (120.0, 5000.0, 100.0), "rs-05": (5.5, 500.0, 5.0),
    "rs-06": (36.0, 5000.0, 100.0),
}

URDF = Path(__file__).parent / "urdf" / "rebot_b601_rs" / "urdf" / "rebot_b601_rs.urdf"


def _motors() -> list[dict]:
    return [{"id": i, "model": "rs-06" if i <= 3 else "rs-00"} for i in range(1, 7)]


@RobotConfig.register_subclass("rebot_b601_rs")
@dataclass(kw_only=True)
class RebotB601RSConfig(RobotConfig):
    urdf: str = str(URDF)
    tip_link: str | None = "link6"
    name: str | None = "rebot_b601_rs"             # exported: rebot_b601_rs_identified_<version>.urdf
    channel: str = "can0"                          # CAN interface (PCAN-USB)
    # One entry per joint in URDF order: id, model, optional feedback_id (0xFD),
    # direction (-1 if the motor turns against the URDF axis), offset_deg (motor angle at URDF zero).
    motors: list = field(default_factory=_motors)
    rate: float = 100.0                            # command rate [Hz]
    kp: object = field(default_factory=lambda: [50.0, 50.0, 50.0, 30.0, 50.0, 50.0])
    kd: object = field(default_factory=lambda: [3.0, 5.0, 5.0, 3.0, 4.0, 4.0])
    soft_limits_deg: list | None = field(
        default_factory=lambda: [[-145, 145], [0, 170], [0, 200], [-80, 90], [-90, 90], [-130, 130]])
    rest_deg: object = 0.0
    free_kd: float = 1.0
    gripper_repo: str = ""                         # default: <project>/third_party/XTac-UMI-Description
    gripper_urdf: str | None = None                 # explicit external URDF overrides the repository
    gripper_parent_link: str = "adapter_link"
    gripper_xyz: tuple[float, float, float] = (0.0, 0.014, 0.0)
    gripper_rpy: tuple[float, float, float] = (0.0, pi / 2, pi / 2)

    @property
    def gripper_urdf_path(self) -> Path:
        from ...assets import gripper_urdf

        if self.gripper_urdf:
            return self.resolve(self.gripper_urdf)
        repository = self.resolve(self.gripper_repo) if self.gripper_repo else project_root() / "third_party" / "XTac-UMI-Description"
        return gripper_urdf(repository)

    @property
    def urdf_path(self) -> Path:
        from ...assets import compose_urdf

        arm = super().urdf_path
        # An explicitly supplied robot URDF is already a complete custom model.
        if arm.resolve() != URDF.resolve():
            return arm
        return compose_urdf(arm, self.gripper_urdf_path, parent_link=self.gripper_parent_link,
                            xyz=self.gripper_xyz, rpy=self.gripper_rpy)
