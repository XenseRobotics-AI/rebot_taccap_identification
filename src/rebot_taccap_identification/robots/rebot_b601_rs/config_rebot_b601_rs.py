"""reBot Arm B601 with RobStride (灵足) motors, direct motor output, C1 adapter and TacCap G1 gripper.

The URDF ships with this package (urdf/rebot_b601_rs_taccap). ``link6`` is the bare joint-6
output frame (no mass); the C1 adapter and the gripper hang below it and are the payload
(removed for the arm identification). Motor coordinates equal the URDF ones (direction 1,
offset 0). Gains are holding gains for use with gravity feedforward.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from ..config import RobotConfig


# MIT ranges of RobStride models: (torque [N·m], kp max, kd max).
MODEL_LIMITS = {
    "rs-00": (14.0, 500.0, 5.0), "rs-01": (17.0, 500.0, 5.0), "rs-02": (17.0, 500.0, 5.0),
    "rs-03": (60.0, 5000.0, 100.0), "rs-04": (120.0, 5000.0, 100.0), "rs-05": (5.5, 500.0, 5.0),
    "rs-06": (36.0, 5000.0, 100.0),
}

URDF = Path(__file__).parent / "urdf" / "rebot_b601_rs_taccap" / "urdf" / "rebot_b601_rs_taccap.urdf"


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
