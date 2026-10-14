"""Saved models and metadata remain usable after moving their whole directory."""

import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

from rebot_taccap_identification.cli import main
from rebot_taccap_identification.export import export_urdf
from rebot_taccap_identification.gravity import gravity_torque
from rebot_taccap_identification.payload import cad_payload, payload_parameters, with_payload
from rebot_taccap_identification.resources import resolve_reference
from rebot_taccap_identification.results import Results
from rebot_taccap_identification.robot import RobotModel
from rebot_taccap_identification.robots import make_robot
from rebot_taccap_identification.robots.rebot_b601_rs.config_rebot_b601_rs import RebotB601RSConfig
from rebot_taccap_identification.runtime.common import Dynamics
from rebot_taccap_identification.runtime.gravity_comp import keep_log
from test_assets import external_gripper
from test_robots import arm_fields


def check_relative_meshes(urdf):
    for mesh in ET.parse(urdf).iter("mesh"):
        reference = mesh.get("filename")
        assert not Path(reference).is_absolute()
        assert (Path(urdf).parent / reference).is_file()


def test_composed_model_survives_relocation_and_missing_external_source(tmp_path):
    _, gripper, _ = external_gripper(tmp_path)
    config = RebotB601RSConfig(gripper_urdf=str(gripper))
    source = config.urdf_path
    assert not config.urdf.startswith("/")
    assert config.urdf.startswith("package://")
    expected = cad_payload(source, "link6")
    moved = tmp_path / "relocated"
    shutil.copytree(source.parent, moved)
    shutil.rmtree(tmp_path / "external")
    model = moved / source.name
    assert str(tmp_path) not in model.read_text()
    check_relative_meshes(model)
    assert RobotModel.from_urdf(model, tip_link="link6").dof == 6
    actual = cad_payload(model, "link6")
    for a, b in zip(actual, expected):
        np.testing.assert_allclose(a, b)


def test_payload_results_export_after_original_inputs_are_removed(tmp_path, capsys):
    _, gripper, _ = external_gripper(tmp_path)
    fields = {"gripper_urdf": str(gripper), "calibration_dir": str(tmp_path / "calibration")}
    r = make_robot("rebot_b601_rs", **fields)
    inputs = tmp_path / "inputs"
    base = inputs / "arm.urdf"
    export_urdf(r.model, r.model.nominal_parameters[:, :10], base, tip_link="link6")
    cad = cad_payload(r.config.urdf_path, "link6")
    parameters = payload_parameters(0.6, np.array([0.01, -0.005, 0.09]), cad[2] * 0.6 / cad[0])
    q = np.random.default_rng(8).uniform(-2, 2, (50, 6))
    tau = gravity_torque(r.model, q, with_payload(r.model, parameters))
    csv = inputs / "static.csv"
    np.savetxt(csv, np.hstack([np.degrees(q), tau]), delimiter=",", comments="",
               header=",".join([f"q_deg{i+1}" for i in range(6)] + [f"tau_meas{i+1}" for i in range(6)]))
    directory = tmp_path / "payload_result"
    out = directory / "identified.urdf"
    assert main(["payload", "--urdf", str(base), "--static", str(csv), "--out-urdf", str(out),
                 "--torque-gain", "1", "--robot.gripper_urdf=" + str(gripper),
                 "--robot.calibration_dir=" + fields["calibration_dir"]]) == 0
    info = json.loads((directory / "payload.json").read_text())
    assert str(tmp_path) not in json.dumps(info)
    for key in ("arm_urdf", "cad_urdf"):
        assert not Path(info[key]).is_absolute()
        check_relative_meshes(resolve_reference(info[key], directory))
    snapshot = resolve_reference(info["cad_urdf"], directory)
    assert info["cad_urdf_sha256"] == hashlib.sha256(snapshot.read_bytes()).hexdigest()
    check_relative_meshes(out)
    shutil.rmtree(inputs)
    shutil.rmtree(tmp_path / "external")
    moved = tmp_path / "moved"
    shutil.move(str(directory), moved)
    arm = resolve_reference(info["arm_urdf"], moved)
    exported = tmp_path / "exported"
    assert main(["package", "--model", str(moved / out.name), "--out", str(exported),
                 "--robot.urdf=" + str(arm), "--robot.calibration_dir=" + fields["calibration_dir"]]) == 0
    written = next((exported / "urdf").glob("*.urdf"))
    check_relative_meshes(written)
    actual = cad_payload(written, "link6")
    np.testing.assert_allclose(actual[0], info["mass_kg"], atol=1e-8)
    np.testing.assert_allclose(actual[1], info["com_m"], atol=1e-8)


def test_weighted_input_references_and_gravity_metadata_are_relative(tmp_path):
    fields = arm_fields(tmp_path)
    r = make_robot("bench_test_arm", **fields)
    results = Results.for_robot(r)
    results.static_dir.mkdir(parents=True)
    csv = results.static("train")
    csv.write_text("test\n")
    results.save_inputs("v1", files=[str(csv) + "@5"], arm=str(r.model.source))
    inputs = results.load_inputs("v1")
    assert inputs["files"] == ["../../static/train.csv@5"]
    assert not Path(inputs["arm"]).is_absolute()
    args = SimpleNamespace(mode="free", rate=1, full_urdf=False, table_z=0.0, ff_scale_used=np.ones(3))
    log = [(float(i), *(np.zeros(3) for _ in range(4)), np.ones(3)) for i in range(3)]
    saved = keep_log(r, Dynamics.for_robot(r), log, args)
    path = saved[0]
    meta = json.loads(path.with_suffix(".meta.json").read_text())
    assert not Path(meta["urdf"]).is_absolute()
    moved = tmp_path / "moved_results"
    shutil.move(str(results.root), moved)
    r.model.source.unlink()
    model = resolve_reference(meta["urdf"], moved / "gravity")
    assert RobotModel.from_urdf(model, tip_link="l3").dof == 3
