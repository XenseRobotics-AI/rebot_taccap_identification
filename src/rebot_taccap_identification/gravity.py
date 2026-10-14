"""Gravity torque of a serial arm, linear in each link's mass and first moment.

A link ``k`` with mass ``m_k`` and first moment ``h_k = m_k c_k`` (CoM ``c_k`` in the link
frame) hangs from joint ``j <= k``. With ``g`` the gravity vector, ``o`` joint origins,
``a`` joint axes and ``R_k`` the link's orientation (all in the base frame), the torque
joint ``j`` must apply to hold it is

    tau_j = -a_j · ((o_k - o_j) × g) · m_k  -  (R_k^T (g × a_j)) · h_k        (revolute)
    tau_j = -a_j · g · m_k                                                     (prismatic)

so ``tau = Y(q) theta`` with ``theta = [m_1, h_1, …, m_n, h_n]`` (4 per link). Rotational
inertia never appears: static data cannot see it.
"""

from __future__ import annotations

import numpy as np

from .robot import RobotModel

PER_LINK = 4                          # m, hx, hy, hz


def gravity_regressor(robot: RobotModel, q: np.ndarray) -> np.ndarray:
    """``(n, dof, 4 * dof)`` for ``(n, dof)`` joint positions."""
    q = np.atleast_2d(np.asarray(q, dtype=float))
    n, dof = q.shape
    rotations, origins, axes = robot._joint_geometry_batch(q)
    g = np.asarray(robot.config.gravity, dtype=float)
    Y = np.zeros((n, dof, PER_LINK * dof))
    for j, link in enumerate(robot.config.links):
        a = axes[:, j]
        for k in range(j, dof):
            column = PER_LINK * k
            if link.joint == "prismatic":
                Y[:, j, column] = -(a @ g)
                continue
            Y[:, j, column] = -np.einsum("ni,ni->n", a, np.cross(origins[:, k] - origins[:, j], g))
            Y[:, j, column + 1:column + 4] = -np.einsum("nji,nj->ni", rotations[:, k], np.cross(g, a))
    return Y


def mass_parameters(inertial: np.ndarray) -> np.ndarray:
    """``(dof, 10)`` inertial parameters -> ``(dof * 4,)`` ``[m, h]`` per link."""
    return np.asarray(inertial, dtype=float).reshape(-1, 10)[:, :4].reshape(-1)


def gravity_torque(robot: RobotModel, q: np.ndarray, inertial: np.ndarray | None = None) -> np.ndarray:
    """``(n, dof)`` holding torque of ``inertial`` (default: the URDF's)."""
    inertial = robot.nominal_parameters[:, :10] if inertial is None else inertial
    return gravity_regressor(robot, q) @ mass_parameters(inertial)
