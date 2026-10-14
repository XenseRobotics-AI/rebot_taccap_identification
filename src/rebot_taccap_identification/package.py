"""A complete, self-contained URDF package of an identified model (arm, or arm + payload).

The robot's original URDF is the template: same links, joints, visuals and collisions,
same layout (``<package>/urdf/<package>.urdf`` and ``<package>/meshes/``). Only the
inertials change:

* link1 … the last arm link: the identified values;
* without a payload, everything below the last arm link (e.g. the gripper) is removed;
* with a payload, the template's payload bodies stay (meshes and joints included) and
  are made to carry exactly the identified payload: every body's mass is scaled by the
  same factor and all their centres of mass are shifted by the same vector, so the
  composite mass and CoM are the identified ones; the remaining difference to the
  identified inertia about the CoM is added to the heaviest body when that keeps its
  inertia positive definite.

Every mesh the URDF references is copied into ``meshes/``, keeping its sub-folder below the
template's own ``meshes/`` folder, and referenced relative to the URDF file
(``../meshes/<sub-folder>/<file>``), so the folder can be moved anywhere. The joint
torque offsets (which a URDF cannot hold) are copied next to the URDF.
"""

from __future__ import annotations

import json
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .export import _set_inertial, inertial_to_urdf, set_link_inertial
from .payload import cad_payload
from .resources import copy_asset_notices as _copy_asset_notices, mesh_name as _mesh_name
from .robot import RobotModel
from .urdf import _inertial, _origin
from .visual import _resolve_mesh


@dataclass
class PackageReport:
    urdf: Path
    meshes: list[str]
    removed: list[str] = field(default_factory=list)
    payload: dict | None = None            # name, mass, com of the payload written
    notes: list[str] = field(default_factory=list)


def has_payload(urdf: str | Path, tip_link: str) -> bool:
    """Whether a URDF has bodies with mass below ``tip_link`` (a package exported with a payload)."""
    return cad_payload(urdf, tip_link) is not None


def find_payload(model_urdf: str | Path) -> dict | None:
    """``payload.json`` written by ``rebot_taccap_identification payload`` next to a payload model, else None."""
    path = Path(model_urdf).with_name("payload.json")
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def _children(root: ET.Element) -> dict[str, list[ET.Element]]:
    out: dict[str, list[ET.Element]] = {}
    for joint in root.findall("joint"):
        out.setdefault(joint.find("parent").get("link"), []).append(joint)
    return out


def _below(root: ET.Element, link: str) -> list[tuple[ET.Element, ET.Element, np.ndarray, np.ndarray]]:
    """(joint, child link, R, t) of every link below ``link``, poses in ``link``'s frame at q = 0."""
    links = {element.get("name"): element for element in root.findall("link")}
    children, out = _children(root), []

    def walk(name: str, rotation: np.ndarray, position: np.ndarray) -> None:
        for joint in children.get(name, []):
            joint_rotation, joint_position = _origin(joint)
            child = joint.find("child").get("link")
            R, t = rotation @ joint_rotation, position + rotation @ joint_position
            out.append((joint, links[child], R, t))
            walk(child, R, t)

    walk(link, np.eye(3), np.zeros(3))
    return out


def _place_payload(root: ET.Element, link: str, target: np.ndarray, notes: list[str]) -> None:
    """Make the bodies below ``link`` add up to ``target`` = ``[m, m c, I about the link origin]``."""
    bodies = [(element, R, t, *inertial) for _, element, R, t in _below(root, link)
              if (inertial := _inertial(element)) is not None and inertial[0] > 0]
    if not bodies:
        raise ValueError(f"模板 URDF 在 {link} 下面没有带质量的负载连杆，无法写入负载")
    cad_mass = sum(b[3] for b in bodies)
    cad_com = sum(b[3] * (b[2] + b[1] @ b[4]) for b in bodies) / cad_mass
    mass, com, inertia = inertial_to_urdf(target)
    ratio, shift = mass / cad_mass, com - cad_com
    scaled = []
    for element, R, t, m, c, I in bodies:
        scaled.append((element, R, t, m * ratio, c + R.T @ shift, I * ratio))
    # Inertia about the composite CoM: identified minus what the scaled CAD bodies give.
    have = sum(R @ I @ R.T + m * (np.dot(p, p) * np.eye(3) - np.outer(p, p))
               for _, R, t, m, c, I in scaled for p in [t + R @ c - com])
    delta = inertia - have
    if np.abs(delta).max() > 1e-9:
        k = max(range(len(scaled)), key=lambda i: scaled[i][3])
        element, R, t, m, c, I = scaled[k]
        candidate = I + R.T @ delta @ R
        if np.linalg.eigvalsh(candidate)[0] > 0:
            scaled[k] = (element, R, t, m, c, candidate)
        else:
            notes.append("辨识出的负载惯量放不进模板的负载连杆（会让某个连杆惯量非正定），保留按质量缩放的 CAD 惯量")
    for element, R, t, m, c, I in scaled:
        _set_inertial(element, m, c, (I + I.T) / 2)


def export_package(template: str | Path, arm_urdf: str | Path, out_dir: str | Path, *, tip_link: str,
                   payload: dict | None = None, offset_yaml: str | Path | None = None,
                   name: str | None = None) -> PackageReport:
    """Write ``<out_dir>/urdf/<name>.urdf`` and ``<out_dir>/meshes/`` (see the module doc)."""
    template, arm_urdf, out_dir = Path(template), Path(arm_urdf), Path(out_dir)
    name = name or out_dir.name
    arm = RobotModel.from_urdf(arm_urdf, tip_link=tip_link)
    if arm.link_names[-1] != tip_link:
        raise ValueError(f"机械臂模型的最后一个连杆是 {arm.link_names[-1]}，不是 {tip_link}")
    tree = ET.parse(template)
    root = tree.getroot()
    links = {element.get("name"): element for element in root.findall("link")}
    joints = {element.get("name"): element for element in root.findall("joint")}
    missing = [n for n in [*arm.link_names, *arm.joint_names] if n not in links and n not in joints]
    if missing:
        raise ValueError(f"模板 URDF 里没有 {', '.join(missing)}：模板和辨识模型不是同一台机械臂")
    report = PackageReport(out_dir / "urdf" / f"{name}.urdf", [])

    for index, link in enumerate(arm.link_names):
        set_link_inertial(links[link], arm.nominal_parameters[index, :10])

    if payload is None:
        for joint, element, _, _ in _below(root, tip_link):
            root.remove(joint)
            root.remove(element)
            report.removed.append(element.get("name"))
    else:
        if payload.get("link", tip_link) != tip_link:
            raise ValueError(f"负载装在 {payload['link']} 上，但机械臂末端是 {tip_link}")
        _place_payload(root, tip_link, np.asarray(payload["inertial_10"], dtype=float), report.notes)

    mesh_dir = out_dir / "meshes"
    if mesh_dir.exists():
        shutil.rmtree(mesh_dir)
    mesh_dir.mkdir(parents=True)
    copied: dict[str, Path] = {}
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        source = _resolve_mesh(filename, template)
        if not source.is_file():
            raise FileNotFoundError(f"模板引用的网格 {filename} 不存在（{source}）")
        target = _mesh_name(source)
        if target in copied and copied[target] != source.resolve():
            raise ValueError(f"两个不同的网格同名：{copied[target]} 和 {source}")
        copied[target] = source.resolve()
        mesh.set("filename", f"../meshes/{target}")
    for target, source in copied.items():
        (mesh_dir / target).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, mesh_dir / target)
    report.meshes = sorted(copied)
    _copy_asset_notices(copied.values(), out_dir)

    text = (f" Exported by rebot_taccap_identification on {datetime.now():%Y-%m-%d %H:%M}. Template: {template.name}."
            f" Identified arm: {arm_urdf.name} ({', '.join(arm.link_names)})"
            + (f"; payload {payload.get('name', '')} {payload['mass_kg']:.3f} kg on {tip_link}" if payload else
               f"; removed below {tip_link}: {', '.join(report.removed) or 'nothing'}")
            + (". Joint torque offsets are not in URDF: see the _offset.yaml file. "
               if offset_yaml is not None and Path(offset_yaml).exists() else ". "))
    root.insert(0, ET.Comment(text))
    ET.indent(tree, space="  ")
    report.urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(report.urdf, encoding="utf-8", xml_declaration=True)
    if offset_yaml is not None and Path(offset_yaml).exists():
        shutil.copy2(offset_yaml, report.urdf.with_name(f"{name}_offset.yaml"))

    check(report, arm, tip_link, payload)
    return report


def check(report: PackageReport, arm, tip_link: str, payload: dict | None) -> None:
    """Re-read the written package: meshes all there, arm inertia, payload mass and CoM as identified."""
    root = ET.parse(report.urdf).getroot()
    for mesh in root.iter("mesh"):
        if not (report.urdf.parent / mesh.get("filename")).is_file():
            raise RuntimeError(f"导出后网格缺失：{mesh.get('filename')}")
    written = RobotModel.from_urdf(report.urdf, tip_link=tip_link)
    error = np.abs(written.nominal_parameters[:, :10] - arm.nominal_parameters[:, :10]).max()
    if error > 1e-6:
        raise RuntimeError(f"导出的机械臂惯性参数和辨识模型差 {error:.2g}")
    if payload is not None:
        body = cad_payload(report.urdf, tip_link)
        if body is None or abs(body[0] - payload["mass_kg"]) > 1e-6 or \
                np.abs(body[1] - np.asarray(payload["com_m"])).max() > 1e-6:
            raise RuntimeError("导出的负载质量/质心和辨识结果对不上")
        report.payload = {"name": payload.get("name", ""), "mass_kg": body[0], "com_m": body[1].tolist()}
