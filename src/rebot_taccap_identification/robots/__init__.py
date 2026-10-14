"""Robot drivers for hardware identification.

One sub-package per robot type (``<name>/config_<name>.py`` with a config registered by
``@RobotConfig.register_subclass("<name>")``, ``<name>/<name>.py`` with the ``Robot``
subclass). Third-party robots: installed packages named ``rebot_taccap_identification_robot_*``.
"""

from .config import RobotConfig
from .robot import Robot, State
from .utils import (
    make_robot, make_robot_config, make_robot_from_config, pop_robot_args, register_robots, robot_args,
)

__all__ = [
    "Robot", "RobotConfig", "State", "make_robot", "make_robot_config", "make_robot_from_config",
    "pop_robot_args", "register_robots", "robot_args",
]
