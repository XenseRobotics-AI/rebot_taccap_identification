"""Robot configuration.

Each robot type is a dataclass registered with ``@RobotConfig.register_subclass("<type>")``;
its defaults describe that model of arm (URDF, motors, gains, limits). Values for one
physical arm are given on the command line (``--robot.channel=can1``) and its measured
calibration is stored per ``id`` (see ``Robot``).
"""

from __future__ import annotations

import abc
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar


def project_root() -> Path:
    """Base of relative paths in configs: the repository in a source checkout, else the cwd."""
    repo = Path(__file__).resolve().parents[3]
    return repo if (repo / "pyproject.toml").is_file() else Path.cwd()


@dataclass(kw_only=True)
class RobotConfig(abc.ABC):
    """What the generic workflow needs to know about an arm.

    Per-joint values accept one number (all joints) or one per joint. Relative paths are
    relative to ``project_root()``.
    """

    _registry: ClassVar[dict[str, type["RobotConfig"]]] = {}

    # Distinguishes physical arms of the same type (calibration file, results folder).
    id: str | None = None
    calibration_dir: Path | None = None       # default ~/.cache/rebot_taccap_identification/calibration/robots/<type>

    urdf: str = ""
    tip_link: str | None = None               # drop everything below (e.g. a removed gripper)
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.80665)
    name: str | None = None                   # prefix of exported models; default: URDF stem
    results_dir: str | None = None            # default: results/<type>[_<id>]
    rate: float = 100.0                       # command rate [Hz]
    kp: Any = 30.0                            # tracking / holding gains
    kd: Any = 2.0
    kp_max: Any = None                        # hardware limits of the gains (None: no clipping)
    kd_max: Any = None
    torque_limits: Any = None                 # [N·m]; default: URDF <limit effort>
    soft_limits_deg: list | None = None       # [[lo, hi], ...] intersected with URDF limits
    rest_deg: Any = 0.0                       # pose the arm returns to before disabling
    torque_gain: Any = None                   # feedback / commanded torque; default: calibration file, else 1
    table_z: float = 0.0                      # table height in the base frame [m]
    move_speed: float = 0.3                   # point-to-point speed [rad/s]
    free_kd: float = 1.0                      # damping of free-floating gravity compensation

    @classmethod
    def register_subclass(cls, name: str):
        """Class decorator: make a config dataclass available as ``--robot.type=<name>``."""
        def decorate(subclass):
            cls._registry[name] = subclass
            subclass._type_name = name
            return subclass
        return decorate

    @classmethod
    def get_choice_class(cls, name: str) -> type["RobotConfig"]:
        if name not in cls._registry:
            raise ValueError(f"未知的机器人类型 {name!r}，可用：{', '.join(sorted(cls._registry))}")
        return cls._registry[name]

    @classmethod
    def get_known_choices(cls) -> dict[str, type["RobotConfig"]]:
        return dict(cls._registry)

    @property
    def type(self) -> str:
        return getattr(type(self), "_type_name", type(self).__name__)

    def resolve(self, path: str | Path) -> Path:
        prefix = "package://rebot_taccap_identification/"
        if str(path).startswith(prefix):
            package = Path(__file__).resolve().parents[1]
            resolved = (package / str(path)[len(prefix):]).resolve()
            if not resolved.is_relative_to(package):
                raise ValueError("package URI 不能引用包外的文件")
            return resolved
        path = Path(path).expanduser()
        return path if path.is_absolute() else (project_root() / path).resolve()

    @property
    def urdf_path(self) -> Path:
        return self.resolve(self.urdf)
