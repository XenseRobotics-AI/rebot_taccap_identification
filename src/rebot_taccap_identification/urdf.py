"""URDF geometry and nominal inertial parameters for fixed-base serial arms.

Fixed descendants (including payloads) are merged into their nearest movable
ancestor. Links below ``tip_link`` are dropped unparsed, so they may contain what a
serial model cannot (e.g. gripper fingers with mimic joints). A link without an
inertial (a virtual frame) has zero parameters. Meshes are irrelevant to dynamics
and are never opened.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .robot import RobotModel, _skew



def _vector(value: str, size: int, context: str) -> np.ndarray:
    try:
        result = np.array([float(item) for item in value.split()])
    except ValueError as exc:
        raise ValueError(f"{context} must contain numeric values; expand xacro first") from exc
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{context} must contain {size} finite values")
    return result


def _origin(element: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = element.find("origin")
    if origin is None:
        return np.eye(3), np.zeros(3)
    translation = _vector(origin.get("xyz", "0 0 0"), 3, "origin xyz")
    roll, pitch, yaw = _vector(origin.get("rpy", "0 0 0"), 3, "origin rpy")
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    # URDF uses fixed-axis roll, pitch, yaw: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    return rotation, translation


@dataclass(frozen=True)
class URDFJoint:
    name: str
    child: str
    joint: str
    rotation: np.ndarray
    translation: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None
    effort: float | None


@dataclass(frozen=True)
class URDFConfig:
    name: str
    links: tuple[URDFJoint, ...]
    root_link: str
    gravity: tuple[float, float, float]

    @property
    def dof(self) -> int:
        return len(self.links)


def _inertial(link: ET.Element) -> tuple[float, np.ndarray, np.ndarray] | None:
    element = link.find("inertial")
    if element is None:
        return None
    mass_element, tensor = element.find("mass"), element.find("inertia")
    if mass_element is None or tensor is None:
        raise ValueError(f"link {link.get('name')}: inertial needs mass and inertia")
    mass = _vector(mass_element.get("value", ""), 1, "mass")[0]
    if mass < 0:
        raise ValueError("link mass must be non-negative")
    xx, xy, xz, yy, yz, zz = _vector(
        " ".join(tensor.get(key, "") for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")),
        6, "inertia tensor",
    )
    rotation, com = _origin(element)
    inertia_com = rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T
    return float(mass), com, inertia_com


class URDFRobotModel(RobotModel):
    """Serial URDF model; joint position columns follow ``joint_names``.

    ``nominal_parameters`` (dof, 10) holds the URDF inertial parameters expressed about
    each movable joint's child-link origin; links without an inertial contribute zero.
    """

    def __init__(
        self, path: str | Path, *,
        gravity: tuple[float, float, float] = (0.0, 0.0, -9.80665),
        tip_link: str | None = None,
    ):
        gravity_array = np.asarray(gravity, dtype=float)
        if gravity_array.shape != (3,) or not np.all(np.isfinite(gravity_array)):
            raise ValueError("gravity must contain three finite values")
        self.source = Path(path)
        try:
            document = ET.parse(self.source).getroot()
        except ET.ParseError as exc:
            raise ValueError(f"invalid URDF XML: {exc}") from exc
        if document.tag != "robot":
            raise ValueError("URDF root must be <robot>")
        if any("xacro" in el.tag or any("${" in v or "$(" in v for v in el.attrib.values())
               for el in document.iter()):
            raise ValueError("expand xacro to a plain URDF before loading")

        links: dict[str, ET.Element] = {}
        for link in document.findall("link"):
            name = link.get("name", "")
            if not name or name in links:
                raise ValueError("URDF link names must be nonempty and unique")
            links[name] = link
        children: dict[str, list[tuple[ET.Element, str]]] = {name: [] for name in links}
        parented: set[str] = set()
        joint_names: set[str] = set()
        for joint in document.findall("joint"):
            name = joint.get("name", "")
            if not name or name in joint_names:
                raise ValueError("URDF joint names must be nonempty and unique")
            joint_names.add(name)
            kind = joint.get("type")
            if kind not in {"fixed", "revolute", "continuous", "prismatic"}:
                raise ValueError(f"joint {name}: unsupported joint type {kind!r}")
            parent_el, child_el = joint.find("parent"), joint.find("child")
            parent = parent_el.get("link") if parent_el is not None else None
            child = child_el.get("link") if child_el is not None else None
            if parent not in links or child not in links:
                raise ValueError(f"joint {name}: parent and child must reference existing links")
            if child in parented:
                raise ValueError(f"link {child} has multiple parent joints")
            parented.add(child)
            children[parent].append((joint, child))
        if tip_link is not None and tip_link not in links:
            raise ValueError(f"tip link {tip_link!r} is not in the URDF")
        roots = set(links) - parented
        if len(roots) != 1:
            raise ValueError("URDF must have exactly one root link and form a connected tree")
        root_link = roots.pop()
        active: list[URDFJoint] = []
        parents: list[int] = []
        parameters: list[np.ndarray] = []
        visited: set[str] = set()
        dropped: set[str] = set()

        def drop(name: str) -> None:
            for _, child in children[name]:
                if child in dropped:
                    raise ValueError("URDF contains a cycle")
                dropped.add(child)
                drop(child)

        def walk(name: str, owner: int, rotation: np.ndarray, position: np.ndarray) -> None:
            if name in visited:
                raise ValueError("URDF contains a cycle")
            visited.add(name)
            body = _inertial(links[name])
            if owner >= 0:
                if body is not None:
                    mass, com, inertia = body
                    com = position + rotation @ com
                    inertia = rotation @ inertia @ rotation.T
                    inertia += mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
                    parameters[owner] += np.array([
                        mass, *(mass * com), inertia[0, 0], inertia[0, 1], inertia[0, 2],
                        inertia[1, 1], inertia[1, 2], inertia[2, 2],
                    ])
            if name == tip_link:
                drop(name)
                return
            for element, child in children[name]:
                local_rotation, local_position = _origin(element)
                next_position = position + rotation @ local_position
                next_rotation = rotation @ local_rotation
                kind = element.get("type")
                if kind == "fixed":
                    walk(child, owner, next_rotation, next_position)
                    continue
                if element.find("mimic") is not None:
                    raise ValueError(f"joint {element.get('name')}: mimic joints are not supported; "
                                     "set tip_link above them or supply an arm-only URDF")
                axis_element = element.find("axis")
                axis = _vector(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0", 3, "joint axis")
                norm = np.linalg.norm(axis)
                if not np.isfinite(norm) or norm < 1e-12:
                    raise ValueError(f"joint {element.get('name')}: axis must be nonzero")
                axis /= norm
                limit = element.find("limit")

                def bound(key: str) -> float | None:
                    if limit is None or key not in limit.attrib:
                        return None
                    return float(_vector(limit.attrib[key], 1, f"joint limit {key}")[0])

                lower, upper = (None, None) if kind == "continuous" else (bound("lower"), bound("upper"))
                if lower is not None and upper is not None and lower > upper:
                    raise ValueError(f"joint {element.get('name')}: lower limit exceeds upper limit")
                index = len(active)
                parents.append(owner)
                active.append(URDFJoint(
                    element.attrib["name"], child, "prismatic" if kind == "prismatic" else "revolute",
                    next_rotation, next_position, axis, lower, upper, bound("effort"),
                ))
                parameters.append(np.zeros(10))
                walk(child, index, np.eye(3), np.zeros(3))

        walk(root_link, -1, np.eye(3), np.zeros(3))
        if visited | dropped != set(links):
            raise ValueError("URDF must be connected and have no cycles")
        if not active:
            raise ValueError("URDF must contain at least one movable joint")
        # DFS produces serial order iff every movable joint follows the previous one.
        if parents != list(range(-1, len(active) - 1)):
            raise ValueError("branched movable joints are not supported; supply an arm-only serial URDF")
        config = URDFConfig(document.get("name", self.source.stem), tuple(active), root_link, tuple(gravity_array))
        super().__init__(config)
        self.joint_names = tuple(joint.name for joint in active)
        self.link_names = tuple(joint.child for joint in active)
        self.nominal_parameters = np.vstack(parameters)

    def _joint_geometry_batch(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        n = q.shape[0]
        rotations, origins, axes = [], [], []
        world_rotation = np.broadcast_to(np.eye(3), (n, 3, 3))
        world_origin = np.zeros((n, 3))
        for i, joint in enumerate(self.config.links):
            world_origin = world_origin + world_rotation @ joint.translation
            world_rotation = world_rotation @ joint.rotation
            axis = world_rotation @ joint.axis
            axes.append(axis)
            if joint.joint == "revolute":
                skew = _skew(joint.axis)
                sin, cos = np.sin(q[:, i])[:, None, None], np.cos(q[:, i])[:, None, None]
                world_rotation = world_rotation @ (np.eye(3) + sin * skew + (1.0 - cos) * (skew @ skew))
            else:
                world_origin = world_origin + axis * q[:, i, None]
            origins.append(world_origin)
            rotations.append(world_rotation)
        return np.stack(rotations, axis=1), np.stack(origins, axis=1), np.stack(axes, axis=1)
