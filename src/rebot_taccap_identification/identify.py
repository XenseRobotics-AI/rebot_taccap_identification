"""Mass and centre of mass of every link from static holding torques.

At rest the measured torque is gravity plus a constant offset per joint (torque zero
error, cable preload):

    tau_meas / gain = Y(q) theta + b

``theta`` holds mass and first moment of each link (``gravity.py``), ``b`` the offsets.
Static data only determine some combinations of ``theta`` (e.g. the first link turns
about the vertical and never feels gravity), so the solution is a weighted least squares
with a pull toward the URDF values, each parameter relative to its own size (mass ~ m,
first moment ~ m·L). The pull fixes the directions the data cannot see; it acts on all
directions, so the others are also biased slightly toward the URDF values. Each link's
rotational inertia about its CoM stays at the URDF value. A link the URDF gives no mass (a
virtual frame, e.g. a bare motor output whose rotor belongs to the previous link) stays
massless: the data are explained without it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .gravity import PER_LINK, gravity_regressor, mass_parameters
from .robot import RobotModel


@dataclass
class MassResult:
    inertial: np.ndarray            # (dof, 10) [m, h, I about the link origin], CAD inertia about the CoM
    offset: np.ndarray              # (dof,) N·m
    rmse: np.ndarray                # (dof,) N·m, on the fitted data
    rank: int                       # combinations of mass/CoM the data determine
    n_samples: int
    notes: list[str] = field(default_factory=list)

    @property
    def mass(self) -> np.ndarray:
        return self.inertial[:, 0]

    @property
    def com(self) -> np.ndarray:
        """Per link; 0 for a massless link."""
        mass = self.inertial[:, :1]
        return np.divide(self.inertial[:, 1:4], mass, out=np.zeros((len(mass), 3)), where=mass > 0)


def _scale(nominal: np.ndarray, length: float = 0.05, min_mass: float = 0.02) -> np.ndarray:
    """Typical size of m and h per link, so the prior pulls them equally in relative terms."""
    scale = []
    for row in nominal:
        mass = max(abs(row[0]), min_mass)
        trace = abs(row[4] + row[7] + row[9])
        radius = max(length, np.sqrt(trace / (2 * mass)) if trace > 0 else 0.0)
        scale += [mass, mass * radius, mass * radius, mass * radius]
    return np.asarray(scale)


def with_cad_inertia(nominal: np.ndarray, masses: np.ndarray, first_moments: np.ndarray) -> np.ndarray:
    """``(dof, 10)`` from identified mass and first moment, inertia about the CoM taken from ``nominal``.
    A link with zero mass gets a zero row."""
    out = np.zeros((len(masses), 10))
    for k, (row, m, h) in enumerate(zip(nominal, masses, first_moments)):
        if m == 0:
            continue
        m0, c0 = row[0], row[1:4] / row[0] if row[0] > 0 else np.zeros(3)
        origin = np.array([[row[4], row[5], row[6]], [row[5], row[7], row[8]], [row[6], row[8], row[9]]])
        about_com = origin - m0 * (c0 @ c0 * np.eye(3) - np.outer(c0, c0))
        c = h / m
        inertia = about_com + m * (c @ c * np.eye(3) - np.outer(c, c))
        out[k] = [m, *h, inertia[0, 0], inertia[0, 1], inertia[0, 2], inertia[1, 1], inertia[1, 2], inertia[2, 2]]
    return out


def fit_masses(robot: RobotModel, q: np.ndarray, tau: np.ndarray, weights: np.ndarray | None = None,
               prior_weight: float = 0.1) -> MassResult:
    """``q`` (n, dof) static poses [rad], ``tau`` (n, dof) holding torque in true units, ``weights`` (n,)."""
    q, tau = np.atleast_2d(q), np.atleast_2d(tau)
    n, dof = q.shape
    weights = np.ones(n) if weights is None else np.asarray(weights, dtype=float)
    Y = gravity_regressor(robot, q)                                    # (n, dof, 4 dof)
    A = np.concatenate([Y, np.broadcast_to(np.eye(dof), (n, dof, dof))], axis=2)   # + offsets
    # Each joint's equations scaled by its torque range, so joints with small and large gravity torque count equally.
    joint_scale = np.maximum(np.ptp(tau, axis=0), 0.2)
    root = np.sqrt(weights)[:, None, None] / joint_scale[None, :, None]
    A_w = (A * root).reshape(n * dof, -1)
    b_w = (tau[:, :, None] * root).reshape(-1)
    nominal = robot.nominal_parameters[:, :10]
    massive = nominal[:, 0] > 0                                        # massless links stay at zero
    free = np.concatenate([np.repeat(massive, PER_LINK), np.ones(dof, dtype=bool)])
    A_w = A_w[:, free]
    prior = np.concatenate([mass_parameters(nominal), np.zeros(dof)])[free]
    scale = np.concatenate([_scale(nominal), np.full(dof, np.inf)])[free]    # offsets: no prior
    inertial_columns = np.isfinite(scale)
    lam = prior_weight * float(np.sum((A_w[:, inertial_columns] * scale[inertial_columns]) ** 2)) / inertial_columns.sum()
    penalty = np.where(inertial_columns, 1.0 / np.where(inertial_columns, scale, 1.0) ** 2, 0.0)
    theta = np.zeros(free.size)
    theta[free] = np.linalg.solve(A_w.T @ A_w + lam * np.diag(penalty), A_w.T @ b_w + lam * penalty * prior)

    params = theta[:PER_LINK * dof].reshape(dof, PER_LINK)
    masses, first_moments, offset = params[:, 0], params[:, 1:4], theta[PER_LINK * dof:]
    notes = []
    if np.any(massive & (masses <= 0)):
        bad = [robot.link_names[k] for k in np.flatnonzero(massive & (masses <= 0))]
        raise ValueError(f"{', '.join(bad)} 的质量辨识为非正数：静态位姿覆盖的姿态不足（可增加不同姿态的静态数据，或加大 --prior-weight）")
    ratio = masses / np.maximum(nominal[:, 0], 1e-9)
    for k in np.flatnonzero(massive & ((ratio < 0.5) | (ratio > 2.0))):
        notes.append(f"{robot.link_names[k]} 质量 {masses[k]:.3f} kg，是原 URDF 的 {ratio[k]:.1f} 倍：单个连杆的值只供参考，"
                     "重力补偿用的是它们的组合")
    # Mass/CoM combinations the data determine: what the weighted regressor adds beyond the offsets.
    used = np.repeat(weights > 0, dof)
    tol = 1e-8 * max(np.abs(A_w).max(), 1e-12)
    rank = int(np.linalg.matrix_rank(A_w[used], tol=tol) - np.linalg.matrix_rank(A_w[used][:, -dof:], tol=tol))
    inertial = with_cad_inertia(nominal, masses, first_moments)
    residual = tau - (Y @ mass_parameters(inertial) + offset)
    return MassResult(inertial, offset, np.sqrt(np.mean(residual**2, axis=0)), rank, n, notes)
