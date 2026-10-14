"""Write identification results back as a URDF and an offset YAML.

The URDF keeps the source kinematics and visuals. Each movable link gets the
given inertial parameters (fixed children that were folded into it lose theirs), links
below ``tip_link`` are removed, and meshes are copied beside the model and referenced
by relative paths. Every link must have a positive
mass and a positive definite inertia about its centre of mass, or all-zero parameters (a
massless frame, written without an inertial).

The YAML is a small fixed subset (one flow mapping per joint) so it can be read
without PyYAML; ``load_offsets`` parses exactly what ``write_offsets`` emits.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from .urdf import URDFRobotModel
from .resources import copy_meshes


def inertial_to_urdf(parameters: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """``[m, mc, I_origin]`` -> mass, CoM and inertia about the CoM (link frame axes)."""
    p = np.asarray(parameters, dtype=float)
    mass = float(p[0])
    if mass <= 0:
        raise ValueError(f"mass must be positive, got {mass:g}")
    com = p[1:4] / mass
    inertia = np.array([[p[4], p[5], p[6]], [p[5], p[7], p[8]], [p[6], p[8], p[9]]])
    inertia_com = inertia - mass * (com @ com * np.eye(3) - np.outer(com, com))
    if np.linalg.eigvalsh(inertia_com)[0] <= 0:
        raise ValueError("inertia about the centre of mass is not positive definite")
    return mass, com, inertia_com


def _number(value: float) -> str:
    return f"{value:.10g}"


def _set_inertial(link: ET.Element, mass: float, com: np.ndarray, inertia: np.ndarray) -> None:
    old = link.find("inertial")
    position = list(link).index(old) if old is not None else 0
    if old is not None:
        link.remove(old)
    inertial = ET.Element("inertial")
    ET.SubElement(inertial, "origin", xyz=" ".join(_number(x) for x in com), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=_number(mass))
    ET.SubElement(inertial, "inertia", **{
        key: _number(inertia[i, j]) for key, (i, j) in
        zip(("ixx", "ixy", "ixz", "iyy", "iyz", "izz"), ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)))
    })
    link.insert(position, inertial)


def set_link_inertial(link: ET.Element, parameters: np.ndarray) -> None:
    """Write ``[m, mc, I_origin]`` into ``link``; all zeros (a massless frame) removes its inertial."""
    if np.any(parameters):
        _set_inertial(link, *inertial_to_urdf(parameters))
    elif (old := link.find("inertial")) is not None:
        link.remove(old)


def export_urdf(robot: URDFRobotModel, inertial: np.ndarray, out: str | Path, *, tip_link: str | None = None,
                note: str = "") -> list[str]:
    """Write ``robot.source`` with identified ``(dof, 10)`` inertia; returns removed link names."""
    inertial = np.asarray(inertial, dtype=float).reshape(robot.dof, 10)
    for row in inertial[inertial.any(axis=1)]:
        inertial_to_urdf(row)            # raises unless mass > 0 and inertia about the CoM > 0
    tree = ET.parse(robot.source)
    root = tree.getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    child_joints: dict[str, list[ET.Element]] = {name: [] for name in links}
    for joint in root.findall("joint"):
        child_joints[joint.find("parent").get("link")].append(joint)

    removed: list[str] = []
    if tip_link is not None:
        stack = list(child_joints[tip_link])
        while stack:
            joint = stack.pop()
            child = joint.find("child").get("link")
            stack.extend(child_joints[child])
            root.remove(joint)
            root.remove(links.pop(child))
            removed.append(child)

    owners = set(robot.link_names)
    for index, name in enumerate(robot.link_names):
        set_link_inertial(links[name], inertial[index])
        # Fixed descendants were folded into this link by the identification model.
        stack = [j for j in child_joints[name] if j.get("type") == "fixed"]
        while stack:
            joint = stack.pop()
            child = joint.find("child").get("link")
            if child not in links or child in owners:
                continue
            if (old := links[child].find("inertial")) is not None:
                links[child].remove(old)
            stack.extend(j for j in child_joints[child] if j.get("type") == "fixed")

    out = Path(out)
    copy_meshes(root, robot.source, out)

    header = (f" Identified by rebot_taccap_identification on {datetime.now():%Y-%m-%d %H:%M}. Source: {robot.source.name}."
              f" Inertia of {', '.join(robot.link_names)} replaced"
              + (f"; removed below {tip_link}: {', '.join(removed)}" if removed else "") + (f". {note}" if note else "") + " ")
    root.insert(0, ET.Comment(header))
    ET.indent(tree, space="  ")
    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return removed


def write_offsets(path: str | Path, joint_names, offset, source: str = "") -> None:
    """Constant torque offset per joint (identified with gravity; part of the holding torque)."""
    lines = [f"# rebot_taccap_identification joint offsets, {datetime.now():%Y-%m-%d %H:%M}" + (f", from {source}" if source else ""),
             "# holding torque = URDF gravity + offset  [N·m]", "joints:"]
    for name, value in zip(joint_names, offset):
        lines.append(f"  {name}: {{offset: {_number(float(value))}}}")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_offsets(path: str | Path, joint_names) -> np.ndarray:
    """Read a file written by ``write_offsets``."""
    joints = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].rstrip()
        if match := re.fullmatch(r"\s+([\w.-]+):\s*\{(.*)\}", line):
            entries = (item.split(":") for item in match.group(2).split(",") if item.strip())
            joints[match.group(1)] = {k.strip(): float(v) for k, v in entries}
    missing = [name for name in joint_names if name not in joints]
    if missing:
        raise ValueError(f"{path}: 缺少关节 {', '.join(missing)}")
    return np.array([joints[name].get("offset", 0.0) for name in joint_names])


def offsets_path(urdf: str | Path) -> Path:
    """The offset file exported next to a model URDF."""
    urdf = Path(urdf)
    return urdf.with_name(urdf.stem + "_offset.yaml")
