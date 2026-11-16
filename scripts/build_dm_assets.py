"""Rebuild the bundled DM arm/C1 assets from local source repositories (no downloads).

python scripts/build_dm_assets.py --dm-repo <reBot-Isaacsim-directory>

Joint-6 is moved +4.316 mm along link5 x to the bare motor output, matching
libpyrebot DM_Taccap. The original output flange and gripper are removed.
The C1 adapter is the same CAD input already shipped for B601-RS.
Arm geometry and inertia priors come from the MIT-licensed simulation model.
These lighter CAD priors require identification and validation on the actual arm.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dm-repo", type=Path, required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    robots = project / "src/rebot_taccap_identification/robots"
    rs = robots / "rebot_b601_rs/urdf/rebot_b601_rs"
    dst = robots / "rebot_b601_dm/urdf/rebot_b601_dm"
    src = args.dm_repo.resolve() / "urdf/reBot_B601_DM"
    arm = src / "urdf/reBot_B601_DM.urdf"
    rs_arm = rs / "urdf/rebot_b601_rs.urdf"
    source = ET.parse(arm).getroot()
    adapter = ET.parse(rs_arm).getroot()
    root = ET.Element("robot", name="rebot_b601_dm")
    root.append(ET.Comment(" DM CAD arm; direct motor output; C1 adapter; external TacCap composed at runtime. "))
    ET.SubElement(root, "link", name="world")
    joint = ET.SubElement(root, "joint", name="world_to_base", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="base_link")
    ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    for name in ["base_link", *[f"link{i}" for i in range(1, 6)]]:
        link = copy.deepcopy(source.find(f"link[@name='{name}']"))
        root.append(link)
    ET.SubElement(root, "link", name="link6")
    for i in range(1, 7):
        joint = copy.deepcopy(source.find(f"joint[@name='joint{i}']"))
        if i == 6:
            joint.find("origin").set("xyz", "0.028008 0 0.04")
        root.append(joint)
    for mesh in root.iter("mesh"):
        name = Path(mesh.get("filename")).name
        target = dst / "meshes/rebot_b601_dm" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src / "meshes" / name, target)
        mesh.set("filename", f"../meshes/rebot_b601_dm/{name}")
    root.append(copy.deepcopy(adapter.find("link[@name='adapter_link']")))
    root.append(copy.deepcopy(next(j for j in adapter.findall("joint")
                                   if j.find("child").get("link") == "adapter_link")))
    shutil.copytree(rs / "meshes/c1_adapter", dst / "meshes/c1_adapter", dirs_exist_ok=True)
    licenses = dst / "LICENSES"
    licenses.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.dm_repo / "LICENSE", licenses / "REBOT_ISAACSIM_MIT")
    shutil.copyfile(rs / "LICENSES/C1_ADAPTER_NOTICE.md", licenses / "C1_ADAPTER_NOTICE.md")
    shutil.copyfile(rs / "LICENSES/TACCAP_CAD_PERMISSION.md", licenses / "TACCAP_CAD_PERMISSION.md")
    (licenses / "DM_ARM_INERTIALS_NOTICE.md").write_text(
        "# DM arm nominal inertial parameters\n\n"
        "Mass, center of mass and inertia for base_link and link1..5 are taken from\n"
        "hopcan/reBot-Isaacsim, urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf.\n"
        "Repository revision and source SHA-256 are recorded in sources.json.\n"
        "Geometry and inertia priors are distributed under the source repository's\n"
        "MIT license, reproduced in REBOT_ISAACSIM_MIT (Copyright 2026 Seeed Studio).\n"
        "These are simulation CAD priors, not measured identification results.\n"
        "They replace the previous control-repository priors whose redistribution\n"
        "license was not established. Masses differ: re-identify and validate on\n"
        "the actual DM arm before using the new model for gravity compensation.\n"
        "Joint geometry and the custom massless link6 motor-output frame are unchanged.\n")
    urdf = dst / "urdf/rebot_b601_dm.urdf"
    urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(urdf, encoding="utf-8", xml_declaration=True)
    def repository_info(path):
        git = lambda *cmd: subprocess.check_output(["git", "-C", str(path), *cmd], text=True).strip()
        return {"url": git("remote", "get-url", "origin"), "commit": git("rev-parse", "HEAD"),
                "clean": not bool(git("status", "--porcelain"))}
    sources = {
        "robot": "rebot_b601_dm", "layout": "arm_and_c1_adapter_only",
        "repositories": {"reBot-Isaacsim": repository_info(args.dm_repo)},
        "inputs": {"arm": {"path": str(arm.relative_to(args.dm_repo.resolve())), "sha256": sha(arm)},
                   "arm_inertials": {"repository": "reBot-Isaacsim", "license": "MIT",
                                     "path": str(arm.relative_to(args.dm_repo.resolve())), "sha256": sha(arm)},
                   "adapter": {"path": str(rs_arm.relative_to(project)), "sha256": sha(rs_arm)},
                   "adapter_provenance": json.loads((rs / "sources.json").read_text())["inputs"]["adapter"]},
        "changes": ["Keep simulation mesh geometry and joint1..5 geometry unchanged",
                    "Keep base_link and link1..5 inertia priors from the MIT-licensed simulation model",
                    "Remove stock flange/gripper; link6 is massless bare motor output",
                    "joint6 origin x=0.028008 (stock 0.023692 + 0.004316 m), as libpyrebot DM_Taccap",
                    "Attach existing C1 CAD at link6, xyz=0, rpy=[pi/2,0,pi/2]; no identified RS masses"],
        "meshes": {str(p.relative_to(dst)): sha(p) for p in sorted((dst / "meshes").rglob("*")) if p.is_file()},
        "licenses": {str(p.relative_to(dst)): sha(p) for p in sorted(licenses.iterdir())},
        "urdf_sha256": sha(urdf),
        "external_gripper": {"repository": "https://github.com/XenseRobotics-AI/XTac-UMI-Description.git",
                             "branch": "main",
                             "entrypoint": "ros/taccap_description/urdf/taccap_g1_slave.urdf",
                             "mount_configuration": "RebotB601DMConfig.gripper_parent_link/gripper_xyz/gripper_rpy"},
    }
    (dst / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(urdf)


if __name__ == "__main__":
    main()
