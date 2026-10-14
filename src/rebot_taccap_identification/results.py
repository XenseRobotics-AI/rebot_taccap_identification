"""Layout of a robot's results directory.

One folder per arm (``results/<type>[_<id>]``), one sub-folder per step::

    static/<name>.csv              static check (holding torques)
    static/<name>.poses.json       the poses of that check
    gravity/<time>_<model>.csv     gravity-compensation log (drift analysis), with .meta.json
    <payload>/                     one package per end-effector payload (e.g. gripper/): its own
                                   static/, gravity/ and (from the workbench) models/<arm version>/
    models/<version>/              <model>_identified_<version>.urdf, ..._offset.yaml, result.json,
                                   inputs.json (data and settings used)
    export/<package>/              complete URDF packages (urdf/ + meshes/)
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

from .resources import relative_reference


def _names(folder: Path, pattern: str, suffix: str) -> list[str]:
    return sorted(p.name[:-len(suffix)] for p in folder.glob(pattern)) if folder.is_dir() else []


def _version_key(version: str) -> tuple:
    match = re.fullmatch(r"v(\d+)", version)
    return (0, int(match.group(1)), "") if match else (1, 0, version)


# Step folders of a results directory; any other sub-folder is a payload package.
RESERVED = ("static", "models", "gravity", "export")


@dataclass(frozen=True)
class Results:
    root: Path

    @classmethod
    def for_robot(cls, robot) -> "Results":
        return cls(robot.results_dir())

    # ------------------------------------------------------------- folders
    @property
    def static_dir(self) -> Path:
        return self.root / "static"

    def payload(self, name: str) -> "Results":
        """The package of an end-effector payload (``<root>/<name>/``): same layout, data taken with it mounted."""
        if name in RESERVED:
            raise ValueError(f"负载名称不能是 {name}（结果目录里已有同名的步骤目录）")
        return Results(self.root / name)

    def payload_names(self) -> list[str]:
        """Payload packages: sub-folders that are not step folders and contain step folders."""
        if not self.root.is_dir():
            return []
        return sorted(p.name for p in self.root.iterdir()
                      if p.is_dir() and p.name not in RESERVED and any((p / d).is_dir() for d in RESERVED))

    def payload_of(self, path) -> str | None:
        """Name of the payload package ``path`` (e.g. a model URDF) lies in, None if it is not in one."""
        path = Path(path).resolve()
        for name in self.payload_names():
            if self.payload(name).root.resolve() in path.parents:
                return name
        return None

    def package_of(self, path) -> "Results":
        """The package (payload or this arm) ``path`` belongs to."""
        name = self.payload_of(path)
        return self if name is None else self.payload(name)

    def payload_models(self) -> list[tuple[str, Path]]:
        """(``<arm version>+<payload>``, URDF) of all payload packages, newest arm version first."""
        return [(f"{version}+{name}", path) for name in self.payload_names()
                for version, path in self.payload(name).model_urdfs()]

    @property
    def gravity_dir(self) -> Path:
        return self.root / "gravity"

    @property
    def export_dir(self) -> Path:
        """Complete URDF packages (step 5): ``export/<package>/urdf/`` + ``meshes/``."""
        return self.root / "export"

    def export_urdfs(self) -> list[tuple[str, Path]]:
        """(package name, URDF) of the step 5 packages."""
        if not self.export_dir.is_dir():
            return []
        return [(p.name, p / "urdf" / f"{p.name}.urdf") for p in sorted(self.export_dir.iterdir())
                if (p / "urdf" / f"{p.name}.urdf").is_file()]

    def is_export(self, path) -> bool:
        return self.export_dir.resolve() in Path(path).resolve().parents

    @property
    def model_root(self) -> Path:
        return self.root / "models"

    # --------------------------------------------------------------- files
    def static(self, name: str) -> Path:
        return self.static_dir / f"{name}.csv"

    def poses(self, name: str) -> Path:
        return self.static_dir / f"{name}.poses.json"

    def model_dir(self, version: str) -> Path:
        return self.model_root / version

    def result(self, version: str) -> Path:
        return self.model_dir(version) / "result.json"

    def urdf(self, version: str, prefix: str) -> Path:
        return self.model_dir(version) / f"{prefix}_identified_{version}.urdf"

    def inputs(self, version: str) -> Path:
        return self.model_dir(version) / "inputs.json"

    # ------------------------------------------------------------- listing
    def static_names(self) -> list[str]:
        return _names(self.static_dir, "*.csv", ".csv")

    def versions(self) -> list[str]:
        """Model versions, oldest first (v1, v2, ..., then other names)."""
        found = [p.name for p in self.model_root.glob("*") if p.is_dir()] if self.model_root.is_dir() else []
        return sorted(found, key=_version_key)

    def next_version(self) -> str:
        numbers = [_version_key(v)[1] for v in self.versions() if _version_key(v)[0] == 0]
        return f"v{max(numbers, default=0) + 1}"

    def model_urdfs(self) -> list[tuple[str, Path]]:
        """(version, exported URDF), newest version first."""
        out = []
        for version in reversed(self.versions()):
            urdfs = [p for p in self.model_dir(version).glob("*.urdf")]
            if urdfs:
                out.append((version, urdfs[0]))
        return out

    def load_inputs(self, version: str) -> dict:
        try:
            return json.loads(self.inputs(version).read_text())
        except (OSError, ValueError):
            return {}

    def save_inputs(self, version: str, **inputs) -> None:
        self.model_dir(version).mkdir(parents=True, exist_ok=True)
        directory = self.model_dir(version)
        if "files" in inputs:
            references = []
            for value in inputs["files"]:
                filename, separator, weight = str(value).rpartition("@")
                try:
                    if not separator:
                        raise ValueError
                    float(weight)
                except ValueError:
                    filename, separator, weight = str(value), "", ""
                references.append(relative_reference(filename, directory) + (separator + weight))
            inputs["files"] = references
        if inputs.get("arm") and Path(inputs["arm"]).is_file():
            inputs["arm"] = relative_reference(inputs["arm"], directory)
        self.inputs(version).write_text(json.dumps(inputs, indent=2, ensure_ascii=False))
