"""Kinematics of a serial arm (link poses, joint axes); geometry comes from ``urdf.URDFRobotModel``."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .urdf import URDFConfig, URDFRobotModel


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


class RobotModel:
    """Serial robot model; inertial parameters per link are ``[m, l(3), L(6)]``.

    ``l = m c`` is the first moment of mass, ``L`` the inertia tensor about the link-frame
    origin, stored as ``[Lxx, Lxy, Lxz, Lyy, Lyz, Lzz]``. Subclasses provide the joint
    geometry (``_joint_geometry_batch``); build one with ``RobotModel.from_urdf``.
    """

    def __init__(self, config: URDFConfig):
        self.config = config
        self.dof = config.dof

    @classmethod
    def from_urdf(
        cls, path: str | Path, *,
        gravity: tuple[float, float, float] = (0.0, 0.0, -9.80665),
        tip_link: str | None = None,
    ) -> "URDFRobotModel":
        """Load a fixed-base serial URDF. Nothing is converted to DH; joint frames are used as given.

        ``tip_link`` drops every link below it (e.g. a removed gripper).
        """
        from .urdf import URDFRobotModel

        return URDFRobotModel(path, gravity=gravity, tip_link=tip_link)

    def _joint_geometry_batch(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Per sample and joint: link rotation, origin and joint axis in the base frame,
        for ``(n_samples, dof)`` positions."""
        raise NotImplementedError

    def angular_velocity(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Angular velocity of the last link in the base frame, for one pose ``q`` and velocity ``dq``."""
        _, _, axes = self._joint_geometry_batch(np.asarray(q, dtype=float)[None])
        revolute = np.array([link.joint == "revolute" for link in self.config.links])
        return (axes[0][revolute] * np.asarray(dq, dtype=float)[revolute, None]).sum(axis=0)
