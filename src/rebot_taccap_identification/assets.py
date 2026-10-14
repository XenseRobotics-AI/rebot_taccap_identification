"""Compose an arm/adapter URDF with the gripper from its external asset repository."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from .visual import _resolve_mesh


def gripper_urdf(repository: str | Path) -> Path:
    """Read the slave gripper entry point from XTac-UMI-Description's asset manifest."""
    package = Path(repository).expanduser().resolve() / "ros" / "taccap_description"
    manifest = package / "asset.json"
    if not manifest.is_file():
        raise FileNotFoundError(
            f"夹爪仓库没有资产清单：{manifest}。先在项目目录运行 "
            "git submodule update --init --remote -- third_party/XTac-UMI-Description；"
            "也可以用 --robot.gripper_repo=<仓库目录> 或 --robot.gripper_urdf=<URDF> 指定外部模型。"
        )
    entry = json.loads(manifest.read_text())["entrypoints"]["default"]
    path = (package / entry).resolve()
    if not path.is_relative_to(package) or not path.is_file():
        raise FileNotFoundError(f"夹爪清单引用的 URDF 不存在或不在资产包中：{path}")
    return path


def _names(root: ET.Element, kind: str) -> set[str]:
    names = [e.get("name", "") for e in root.findall(kind)]
    if any(not n for n in names) or len(set(names)) != len(names):
        raise ValueError(f"URDF 的 {kind} 名称为空或重复")
    return set(names)


def _root_link(root: ET.Element) -> str:
    links, joints = _names(root, "link"), _names(root, "joint")
    children: dict[str, list[str]] = {name: [] for name in links}
    parents: set[str] = set()
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None or parent.get("link") not in links or child.get("link") not in links:
            raise ValueError(f"夹爪关节 {joint.get('name')} 引用了不存在的连杆")
        child_name = child.get("link")
        if child_name in parents:
            raise ValueError(f"夹爪连杆 {child_name} 有多个父关节")
        parents.add(child_name)
        children[parent.get("link")].append(child_name)
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") not in joints:
            raise ValueError(f"夹爪关节 {joint.get('name')} 的 mimic 目标不存在")
    roots = links - parents
    if len(roots) != 1:
        raise ValueError("夹爪 URDF 必须有且只有一个根连杆")
    root_name = next(iter(roots))
    pending, visited = [root_name], set()
    while pending:
        name = pending.pop()
        if name in visited:
            raise ValueError("夹爪 URDF 的关节树有环")
        visited.add(name)
        pending.extend(children[name])
    if visited != links:
        raise ValueError("夹爪 URDF 有未连接到根的连杆")
    return root_name


def _vector_text(value, what: str) -> str:
    values = tuple(float(v) for v in value)
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"{what} 必须是三个有限数值")
    return " ".join(f"{v:.17g}" for v in values)


def compose_urdf(arm: str | Path, gripper: str | Path, *, parent_link: str,
                 xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), cache_dir: str | Path | None = None) -> Path:
    """Build a cached template, preserving upstream gripper joints, inertials and geometry.

    Meshes remain in their source repositories; only their paths become absolute. The
    cache changes when either URDF, the mounting transform or mesh metadata changes.
    No Git or network operation runs while loading a robot.
    """
    arm, gripper = Path(arm).resolve(), Path(gripper).resolve()
    root, tool = ET.parse(arm).getroot(), ET.parse(gripper).getroot()
    if parent_link not in _names(root, "link"):
        raise ValueError(f"机械臂 URDF 中没有夹爪安装连杆 {parent_link}")
    tool_root = _root_link(tool)
    for kind in ("link", "joint"):
        overlap = _names(root, kind) & _names(tool, kind)
        if overlap:
            raise ValueError(f"机械臂和夹爪的 {kind} 名称冲突：{', '.join(sorted(overlap))}")
    mount_name = "external_gripper_mount"
    if mount_name in (_names(root, "joint") | _names(tool, "joint")):
        raise ValueError(f"URDF 已包含安装关节 {mount_name}")

    mesh_states = []
    for element, source in ((root, arm), (tool, gripper)):
        for mesh in element.iter("mesh"):
            path = _resolve_mesh(mesh.get("filename", ""), source).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"URDF {source} 引用的网格不存在：{path}")
            mesh.set("filename", str(path))
            stat = path.stat()
            mesh_states.append((str(path), stat.st_mtime_ns, stat.st_size))

    materials = {m.get("name"): m for m in root.findall("material")}
    for element in tool:
        if element.tag == "material" and element.get("name") in materials:
            if ET.tostring(element) != ET.tostring(materials[element.get("name")]):
                raise ValueError(f"机械臂和夹爪的材料名称冲突：{element.get('name')}")
            continue
        root.append(element)
    mount = ET.SubElement(root, "joint", name=mount_name, type="fixed")
    ET.SubElement(mount, "parent", link=parent_link)
    ET.SubElement(mount, "child", link=tool_root)
    ET.SubElement(mount, "origin", xyz=_vector_text(xyz, "gripper_xyz"), rpy=_vector_text(rpy, "gripper_rpy"))
    root.set("name", arm.stem + "_taccap")
    root.insert(0, ET.Comment(f" Composed from arm {arm} and external gripper {gripper}. "))
    ET.indent(root, space="  ")
    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    digest = hashlib.sha256(data + json.dumps(mesh_states).encode()).hexdigest()
    cache = Path(cache_dir) if cache_dir is not None else Path(tempfile.gettempdir()) / "rebot_taccap_identification" / "urdf"
    output = cache / digest / f"{arm.stem}_taccap.urdf"
    if not output.is_file():
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=output.parent, delete=False) as file:
            temporary = Path(file.name)
            file.write(data)
        try:
            os.replace(temporary, output)
        finally:
            temporary.unlink(missing_ok=True)
    return output
