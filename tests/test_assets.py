"""External gripper composition, updates, failure modes and self-contained export."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rebot_taccap_identification.assets import compose_urdf, gripper_urdf
from rebot_taccap_identification.export import export_urdf
from rebot_taccap_identification.package import export_package
from rebot_taccap_identification.payload import cad_payload, payload_parameters
from rebot_taccap_identification.robot import RobotModel
from rebot_taccap_identification.resources import bundle_urdf
from rebot_taccap_identification.robots.rebot_b601_rs.config_rebot_b601_rs import RebotB601RSConfig, URDF
from rebot_taccap_identification.visual import load_visuals


def external_gripper(tmp_path, mass=0.3):
    package = tmp_path / "external/ros/taccap_description"
    (package / "urdf").mkdir(parents=True, exist_ok=True)
    (package / "meshes").mkdir(exist_ok=True)
    mesh = package / "meshes/base_link.STL"
    mesh.write_text("solid test\nfacet normal 0 0 1\nouter loop\n"
                    "vertex 0 0 0\nvertex 0.01 0 0\nvertex 0 0.01 0\n"
                    "endloop\nendfacet\nendsolid test\n")
    path = package / "urdf/custom.urdf"
    path.write_text(f'''<robot name="external_test">
      <link name="external_base"><inertial><origin xyz="0 0 0.1"/>
        <mass value="{mass}"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
        </inertial><visual><geometry><mesh filename="../meshes/base_link.STL"/></geometry></visual></link>
      <link name="external_finger"/>
      <joint name="external_finger_joint" type="revolute"><parent link="external_base"/>
        <child link="external_finger"/><origin xyz="0 0 0.12"/><axis xyz="1 0 0"/>
        <limit effort="1" lower="0" upper="1" velocity="1"/></joint>
    </robot>''')
    (package / "asset.json").write_text('{"entrypoints": {"default": "urdf/custom.urdf"}}')
    return tmp_path / "external", path, mesh


def test_manifest_and_source_updates_change_payload_without_changing_arm(tmp_path):
    repository, path, mesh = external_gripper(tmp_path)
    assert gripper_urdf(repository) == path
    config = RebotB601RSConfig(gripper_repo=str(repository))
    before = config.urdf_path
    model_before = RobotModel.from_urdf(before, tip_link="link6")
    body_before = cad_payload(before, "link6")
    assert body_before[0] == pytest.approx(0.32)
    tool_visuals = load_visuals(model_before, "link6", tool=True)
    assert any(v.path is not None and v.path.name == mesh.name and v.path.read_bytes() == mesh.read_bytes()
               for v in tool_visuals)
    assert ET.parse(before).find("joint[@name='external_finger_joint']").get("type") == "revolute"

    external_gripper(tmp_path, mass=0.7)
    after = config.urdf_path
    assert before != after
    assert cad_payload(after, "link6")[0] == pytest.approx(0.72)
    assert cad_payload(before, "link6")[0] == pytest.approx(0.32)
    model_after = RobotModel.from_urdf(after, tip_link="link6")
    np.testing.assert_allclose(model_before.nominal_parameters, model_after.nominal_parameters)
    mesh.write_text(mesh.read_text() + "\n")
    assert config.urdf_path != after


def test_mount_transform_is_applied_in_the_tip_frame(tmp_path):
    _, path, _ = external_gripper(tmp_path)
    mounted = compose_urdf(URDF, path, parent_link="link6", xyz=(0.1, 0.2, 0.3),
                           rpy=(0, np.pi / 2, 0), cache_dir=tmp_path / "cache")
    arm_body = cad_payload(URDF, "link6")
    body = cad_payload(mounted, "link6")
    expected_com = (arm_body[0] * arm_body[1] + 0.3 * np.array([0.2, 0.2, 0.3])) / 0.32
    assert body[0] == pytest.approx(0.32)
    np.testing.assert_allclose(body[1], expected_com, atol=1e-12)
    assert np.linalg.eigvalsh(body[2]).min() > 0


def test_missing_assets_conflicting_links_and_custom_complete_urdf(tmp_path):
    with pytest.raises(FileNotFoundError, match="git submodule update"):
        RebotB601RSConfig(gripper_repo=str(tmp_path / "missing")).urdf_path
    _, path, mesh = external_gripper(tmp_path)
    path.write_text(path.read_text().replace('name="external_base"', 'name="link6"').replace('link="external_base"', 'link="link6"'))
    with pytest.raises(ValueError, match="名称冲突"):
        compose_urdf(URDF, path, parent_link="adapter_link")
    _, path, mesh = external_gripper(tmp_path)
    mesh.unlink()
    with pytest.raises(FileNotFoundError, match="网格不存在"):
        compose_urdf(URDF, path, parent_link="adapter_link")
    config = RebotB601RSConfig(urdf=str(path), gripper_repo=str(tmp_path / "missing"))
    assert config.urdf_path == path


def test_export_copies_external_geometry_and_keeps_identified_payload(tmp_path):
    _, path, mesh = external_gripper(tmp_path)
    config = RebotB601RSConfig(gripper_urdf=str(path))
    template = config.urdf_path
    arm = RobotModel.from_urdf(template, tip_link="link6")
    identified = tmp_path / "identified/arm.urdf"
    export_urdf(arm, arm.nominal_parameters[:, :10], identified, tip_link="link6")
    mass, com, inertia = cad_payload(template, "link6")
    payload = {"name": "external", "link": "link6", "mass_kg": mass, "com_m": com.tolist(),
               "inertial_10": payload_parameters(mass, com, inertia).tolist()}
    report = export_package(template, identified, tmp_path / "export", tip_link="link6", payload=payload)
    # Identical mesh basenames in the arm and tool must remain distinct.
    assert {"rebot_b601_rs/base_link.STL", "taccap_description/base_link.STL"} <= set(report.meshes)
    path.unlink()
    mesh.unlink()
    for element in ET.parse(report.urdf).iter("mesh"):
        assert not Path(element.get("filename")).is_absolute()
        assert (report.urdf.parent / element.get("filename")).is_file()
    written = cad_payload(report.urdf, "link6")
    assert written[0] == pytest.approx(mass)
    np.testing.assert_allclose(written[1], com, atol=1e-8)


def test_current_ros_layout_preserves_licenses_across_cache_and_exports(tmp_path):
    repository, legacy, _ = external_gripper(tmp_path)
    package = legacy.parent.parent
    (package / "asset.json").unlink()
    current = legacy.rename(legacy.with_name("taccap_g1_slave.urdf"))
    (package / "package.xml").write_text('<package><name>taccap_description</name></package>')
    (package / "NOTICE.md").write_text("Upstream attribution and exceptions\n")
    (repository / "LICENSE").write_text("Repository MIT license\n")
    (package / "LICENSES").mkdir()
    (package / "LICENSES/third_party.txt").write_text("Third-party license\n")
    assert gripper_urdf(repository) == current
    config = RebotB601RSConfig(gripper_repo=str(repository))
    template = config.urdf_path
    # Notice-only updates must not reuse a cache containing stale license statements.
    (package / "NOTICE.md").write_text("Updated upstream attribution and exceptions\n")
    updated = config.urdf_path
    assert updated != template
    snapshot = bundle_urdf(updated, tmp_path / "snapshot/model.urdf")
    arm = RobotModel.from_urdf(snapshot, tip_link="link6")
    identified = tmp_path / "identified/arm.urdf"
    export_urdf(arm, arm.nominal_parameters[:, :10], identified, tip_link="link6")
    report = export_package(snapshot, identified, tmp_path / "export", tip_link="link6")
    for directory in (updated.parent, snapshot.parent, report.urdf.parent.parent):
        notices = directory / "LICENSES/taccap_description"
        assert (notices / "LICENSE").read_bytes() == (repository / "LICENSE").read_bytes()
        assert (notices / "NOTICE.md").read_bytes() == (package / "NOTICE.md").read_bytes()
        assert (notices / "LICENSES/third_party.txt").is_file()


def test_manifest_cannot_escape_asset_package(tmp_path):
    repository, path, _ = external_gripper(tmp_path)
    (path.parent.parent / "asset.json").write_text('{"entrypoints":{"default":"../../../outside.urdf"}}')
    with pytest.raises(ValueError, match="资产包之外"):
        gripper_urdf(repository)
