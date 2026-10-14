"""End-effector payload (e.g. a gripper) on an already identified arm, from static data.

The arm model stays fixed; only the mass and centre of mass of the payload rigidly
attached to the last link are identified. Its first moment adds to the last link's, so
the last link's gravity columns are the payload's regressor and the problem is a small
least squares on the holding torque the arm model does not explain:

    tau_meas / gain - g_arm(q) - offset = Y_last(q) · [m_p, m_p c_p] + db_last

The torque offset of the last joint is fitted again (``db_last``). Without the payload
that joint carries no gravity, so the arm data never turn it and its offset there is
whatever held at one angle (e.g. a cable); with the payload (its cable, its weight on
the wrist bearing) it is different, and keeping the old value drives the joint round
in gravity compensation.

The payload's inertia about its CoM comes from the CAD model of the payload when the
robot's URDF has it (``cad``, scaled to the identified mass), else from a solid sphere
(``radius``). Static data cannot see it, and gravity compensation does not need it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .gravity import PER_LINK, gravity_regressor, gravity_torque
from .robot import RobotModel


@dataclass
class PayloadResult:
    parameters: np.ndarray        # (10,) [m, mc, I about the link origin] in the last link's frame
    mass: float
    com: np.ndarray               # (3,) m, in the last link's frame
    rmse_arm: np.ndarray          # per joint, arm model only
    rmse_payload: np.ndarray      # per joint, arm + payload
    offset: np.ndarray | None = None   # (dof,) joint torque offsets to use with the payload
    inertia_source: str = ""      # where the inertia about the CoM comes from
    notes: list[str] = field(default_factory=list)


MIN_MASS = 0.01                   # kg; below this the payload is noise and its CoM meaningless


def sphere_inertia(mass: float, radius: float) -> np.ndarray:
    return np.eye(3) * 0.4 * mass * radius**2


def cad_payload(urdf: str | Path, tip_link: str) -> tuple[float, np.ndarray, np.ndarray] | None:
    """Mass, CoM and inertia about the CoM (``tip_link`` frame) of everything the URDF has below
    ``tip_link`` (e.g. a gripper), all joints at 0; None if there is nothing with mass."""
    import xml.etree.ElementTree as ET

    from .urdf import _inertial, _origin

    root = ET.parse(urdf).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    children: dict[str, list] = {}
    for joint in root.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    bodies = []

    def walk(name: str, rotation: np.ndarray, position: np.ndarray) -> None:
        for joint in children.get(name, []):
            joint_rotation, joint_position = _origin(joint)
            child = joint.find("child").get("link")
            R, t = rotation @ joint_rotation, position + rotation @ joint_position
            inertial = _inertial(links[child])
            if inertial is not None and inertial[0] > 0:
                mass, com, inertia = inertial
                bodies.append((mass, t + R @ com, R @ inertia @ R.T))
            walk(child, R, t)

    walk(tip_link, np.eye(3), np.zeros(3))
    if not bodies:
        return None
    mass = sum(b[0] for b in bodies)
    com = sum(b[0] * b[1] for b in bodies) / mass
    inertia = sum(I + m * ((c - com) @ (c - com) * np.eye(3) - np.outer(c - com, c - com)) for m, c, I in bodies)
    return float(mass), com, inertia


def payload_parameters(mass: float, com: np.ndarray, inertia_com: np.ndarray) -> np.ndarray:
    """[m, m c, I about the origin] from mass, CoM and inertia about the CoM."""
    com = np.asarray(com, dtype=float)
    inertia = np.asarray(inertia_com, dtype=float) + mass * (com @ com * np.eye(3) - np.outer(com, com))
    return np.array([mass, *(mass * com), inertia[0, 0], inertia[0, 1], inertia[0, 2],
                     inertia[1, 1], inertia[1, 2], inertia[2, 2]])


def arm_torque(robot: RobotModel, q, offset=None) -> np.ndarray:
    """Holding torque of the arm model plus its joint offsets."""
    tau = gravity_torque(robot, q)
    return tau if offset is None else tau + np.asarray(offset, dtype=float)


def fit_payload(robot: RobotModel, q, tau, *, offset=None, sample_weights=None, radius: float = 0.03,
                cad=None) -> PayloadResult:
    """Identify mass and CoM of the payload on ``robot``'s last link; ``tau`` in true torque units."""
    q, tau = np.atleast_2d(np.asarray(q, dtype=float)), np.atleast_2d(np.asarray(tau, dtype=float))
    dof = robot.dof
    offset = np.zeros(dof) if offset is None else np.asarray(offset, dtype=float)
    residual = tau - arm_torque(robot, q, offset)
    Y = gravity_regressor(robot, q)[:, :, PER_LINK * (dof - 1):]          # (n, dof, 4)
    last = np.zeros((len(q), dof, 1))
    last[:, -1, 0] = 1.0                                                  # last joint's offset change
    root = np.sqrt(np.ones(len(q)) if sample_weights is None else np.asarray(sample_weights, dtype=float))
    A = (np.concatenate([Y, last], axis=2) * root[:, None, None]).reshape(-1, PER_LINK + 1)
    b = (residual * root[:, None]).reshape(-1)
    # Directions the data cannot see (e.g. a CoM offset along an axis that never tilts
    # against gravity) get the least-squares minimum-norm value; name them.
    norms = np.linalg.norm(A, axis=0)
    seen = norms > 1e-9 * norms.max()
    An = np.where(seen, A / np.where(seen, norms, 1.0), 0.0)
    _, s, vt = np.linalg.svd(An, full_matrices=False)
    null = vt[s < 1e-6 * s[0]]
    names = ["m", "mc_x", "mc_y", "mc_z"]
    unobservable = [names[i] for i in range(PER_LINK) if not seen[i] or (null.size and np.linalg.norm(null[:, i]) > 0.5)]
    solution, *_ = np.linalg.lstsq(A, b, rcond=1e-6)
    theta, shift = solution[:PER_LINK], float(solution[PER_LINK])
    offset = offset.copy()
    offset[-1] += shift
    mass = float(theta[0])
    if mass < MIN_MASS:
        raise ValueError(f"辨识出的负载质量为 {1000 * mass:.0f} g（小于 {1000 * MIN_MASS:.0f} g）：数据里看不到负载，"
                         "检查是否装了负载、数据是否覆盖足够多的姿态")
    com = theta[1:4] / mass
    notes = []
    if mass < 0.02:
        notes.append(f"负载质量只有 {1000 * mass:.0f} g，几乎为 0：质心没有意义。确认采数据时装着负载")
    if cad is not None:
        cad_mass, _, cad_inertia = cad
        inertia = cad_inertia * (mass / cad_mass)          # same shape, identified mass
        source = f"CAD（URDF 中 {cad_mass:.3f} kg 的负载，按辨识质量缩放）"
    else:
        inertia = sphere_inertia(mass, radius)
        source = f"实心球估计（半径 {1000 * radius:.0f} mm）"
    parameters = payload_parameters(mass, com, inertia)
    prediction = Y @ theta + last[:, :, 0] * shift
    rmse_arm = np.sqrt(np.mean(residual**2, axis=0))
    rmse_payload = np.sqrt(np.mean((residual - prediction) ** 2, axis=0))
    if unobservable:
        notes.append(f"数据确定不了 {', '.join(unobservable)}（取最小范数解）：这些方向在静态数据的姿态里从未影响力矩，"
                     "增加转动末端关节的不同姿态可使其可辨识")
    return PayloadResult(parameters, mass, com, rmse_arm, rmse_payload, offset, source, notes)


def with_payload(robot: RobotModel, payload: np.ndarray) -> np.ndarray:
    """``(dof, 10)`` inertial parameters of the arm with the payload merged into the last link."""
    inertial = robot.nominal_parameters[:, :10].copy()
    inertial[-1] += np.asarray(payload, dtype=float)
    return inertial
