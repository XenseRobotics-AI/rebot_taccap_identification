"""Portable URDF resources and relative references in saved result files."""

from __future__ import annotations

import os
import hashlib
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from .visual import _resolve_mesh


def relative_reference(path: str | Path, directory: str | Path) -> str:
    """Serialize a file reference relative to its metadata file's directory."""
    return Path(os.path.relpath(Path(path).resolve(), Path(directory).resolve())).as_posix()


def resolve_reference(reference: str | Path, directory: str | Path) -> Path:
    """Read relative references and remain compatible with existing absolute ones."""
    path = Path(reference)
    return path if path.is_absolute() else Path(directory) / path


def resource_digest(urdf: str | Path) -> str:
    """Cache identity for a model and its current mesh files, without storing host paths."""
    urdf = Path(urdf)
    digest = hashlib.sha256(urdf.read_bytes())
    meshes = []
    for mesh in ET.parse(urdf).iter("mesh"):
        path = _resolve_mesh(mesh.get("filename", ""), urdf)
        meshes.append(path)
        stat = path.stat()
        digest.update(f"{mesh.get('filename')}:{stat.st_size}:{stat.st_mtime_ns}".encode())
    for target, source in sorted(asset_notices(meshes).items()):
        digest.update(target.encode() + b"\0" + source.read_bytes())
    return digest.hexdigest()


def mesh_name(source: Path) -> str:
    """Namespace flat mesh folders and preserve existing mesh subdirectories."""
    parts = source.resolve().parts
    if "meshes" in parts[:-1]:
        relative = "/".join(parts[len(parts) - parts[::-1].index("meshes"):])
        return relative if "/" in relative else f"{source.parent.parent.name}/{relative}"
    return source.name


def copy_file(source: Path, destination: Path) -> None:
    """Atomically copy a resource; source and destination may already be the same file."""
    if source.resolve() == destination.resolve():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as file:
        temporary = Path(file.name)
    try:
        shutil.copy2(source, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def _asset_package(path: Path) -> bool:
    return any((path / name).is_file() for name in ("asset.json", "sources.json", "package.xml"))


def asset_notices(meshes) -> dict[str, Path]:
    """Map output notice paths to sources, preserving ROS packages and saved bundles."""
    packages = set()
    for mesh in meshes:
        package = next((p for p in Path(mesh).parents
                        if _asset_package(p)
                        or ((p / "LICENSES").is_dir() and (p / "meshes").is_dir())), None)
        if package is not None:
            packages.add(package)
    notices = {}
    for package in sorted(packages):
        original = _asset_package(package)
        target = Path("LICENSES") / package.name if original else Path()
        files = [p for p in (package / "LICENSES").rglob("*") if p.is_file()]
        if original:
            files += [p for p in (package / "NOTICE.md", package / "LICENSE") if p.is_file()]
        for source in files:
            notices[(target / source.relative_to(package)).as_posix()] = source
        if ((package / "asset.json").is_file() or (package / "package.xml").is_file()) and not (package / "LICENSE").is_file():
            license_file = next((p / "LICENSE" for p in package.parents if (p / "LICENSE").is_file()), None)
            if license_file is not None:
                notices[(target / "LICENSE").as_posix()] = license_file
    return notices


def copy_asset_notices(meshes, out_dir: Path) -> None:
    """Copy notices from source asset packages or an already assembled URDF bundle."""
    for relative, source in asset_notices(meshes).items():
        copy_file(source, out_dir / relative)


def copy_meshes(root: ET.Element, source_urdf: Path, output_urdf: Path) -> dict[str, Path]:
    """Copy mesh files next to an output URDF and write only relative filenames."""
    copied = {}
    for mesh in root.iter("mesh"):
        source = _resolve_mesh(mesh.get("filename", ""), source_urdf).resolve()
        if not source.is_file():
            raise FileNotFoundError(f"URDF 引用的网格不存在：{source}")
        target = mesh_name(source)
        if target in copied and copied[target] != source:
            raise ValueError(f"两个不同的网格同名：{target}")
        copied[target] = source
        destination = output_urdf.parent / "meshes" / target
        mesh.set("filename", "meshes/" + target)
        copy_file(source, destination)
    copy_asset_notices(copied.values(), output_urdf.parent)
    return copied


def bundle_urdf(source: str | Path, output: str | Path) -> Path:
    """Save a relocatable snapshot of an existing URDF, its meshes and notices."""
    source, output = Path(source), Path(output)
    tree = ET.parse(source)
    copy_meshes(tree.getroot(), source, output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output
