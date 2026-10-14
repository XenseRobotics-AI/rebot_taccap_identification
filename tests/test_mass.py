"""Gravity regressor, mass/CoM identification, payload, export."""

from pathlib import Path

import numpy as np
import pytest

from rebot_taccap_identification import RobotModel, fit_masses, fit_payload, gravity_torque
from rebot_taccap_identification.export import export_urdf, load_offsets, offsets_path, write_offsets
from rebot_taccap_identification.payload import payload_parameters, with_payload
from rebot_taccap_identification.robots import make_robot_config

from helpers import random_poses
from test_visual import boxed_arm


def rebot_urdf():
    config = make_robot_config("rebot_b601_rs")
    if not Path(config.urdf_path).is_file():
        pytest.skip("reBot URDF not installed")
    return config


def potential_energy(robot, q):
    """Gravitational potential energy of all links (independent of the regressor)."""
    rotations, origins, _ = robot._joint_geometry_batch(np.atleast_2d(q))
    g = np.asarray(robot.config.gravity)
    params = robot.nominal_parameters
    centres = origins[0] * params[:, :1] + np.einsum("kij,kj->ki", rotations[0], params[:, 1:4])  # m * CoM
    return -float(np.sum(centres @ g))


@pytest.mark.parametrize("which", ["boxed", "rebot"])
def test_gravity_regressor_is_gradient_of_potential_energy(tmp_path, which):
    if which == "rebot":
        config = rebot_urdf()
        robot = RobotModel.from_urdf(config.urdf_path, tip_link=config.tip_link)
    else:
        robot = RobotModel.from_urdf(boxed_arm(tmp_path), tip_link="l3")
    q = np.random.default_rng(0).uniform(-2, 2, size=(8, robot.dof))
    h = 1e-6
    expected = np.array([[(potential_energy(robot, x + h * e) - potential_energy(robot, x - h * e)) / (2 * h)
                          for e in np.eye(robot.dof)] for x in q])
    np.testing.assert_allclose(gravity_torque(robot, q), expected, atol=1e-6)


def test_fit_masses_reproduces_gravity_keeps_cad_inertia(tmp_path):
    config = rebot_urdf()
    robot = RobotModel.from_urdf(config.urdf_path, tip_link=config.tip_link)
    truth = robot.nominal_parameters[:, :10].copy()
    truth[1:, 0] *= 1.15                                   # heavier links, shifted CoMs
    truth[1:, 1:4] *= 1.2
    offset = np.array([0.0, -0.2, 0.1, 0.05, 0.0, 0.0])
    q = random_poses(robot, 80)
    tau = gravity_torque(robot, q, truth) + offset
    result = fit_masses(robot, q, tau, prior_weight=1e-6)
    check = random_poses(robot, 40, seed=1)
    np.testing.assert_allclose(gravity_torque(robot, check, result.inertial) + result.offset,
                               gravity_torque(robot, check, truth) + offset, atol=1e-4)
    # link6 is the bare joint-6 output frame: massless in the URDF, kept massless.
    assert result.rank == 8 and np.all(result.mass[:5] > 0) and not result.inertial[5].any()
    np.testing.assert_array_equal(result.com[5], 0.0)
    # Inertia about each CoM is the CAD one.
    from rebot_taccap_identification.export import inertial_to_urdf
    for row, cad in zip(result.inertial[:5], robot.nominal_parameters[:5, :10]):
        np.testing.assert_allclose(inertial_to_urdf(row)[2], inertial_to_urdf(cad)[2], atol=1e-12)
    out = tmp_path / "arm.urdf"
    export_urdf(robot, result.inertial, out, tip_link=config.tip_link)
    np.testing.assert_allclose(RobotModel.from_urdf(out, tip_link=config.tip_link).nominal_parameters[:, :10],
                               result.inertial, atol=1e-8)


def test_mimic_fingers_only_below_tip_link():
    config = rebot_urdf()
    assert RobotModel.from_urdf(config.urdf_path, tip_link=config.tip_link).dof == 6
    with pytest.raises(ValueError, match="mimic"):
        RobotModel.from_urdf(config.urdf_path)


def test_export_and_offsets_round_trip(tmp_path):
    robot = RobotModel.from_urdf(boxed_arm(tmp_path), tip_link="l3")
    identified = robot.nominal_parameters[:, :10] * 1.2
    out = tmp_path / "out" / "arm_identified_v1.urdf"
    export_urdf(robot, identified, out, tip_link="l3")
    write_offsets(offsets_path(out), robot.joint_names, [0.1, -0.2, 0.3])
    again = RobotModel.from_urdf(out, tip_link="l3")
    np.testing.assert_allclose(again.nominal_parameters[:, :10], identified, atol=1e-8)
    np.testing.assert_allclose(load_offsets(offsets_path(out), robot.joint_names), [0.1, -0.2, 0.3])


def test_payload_recovers_mass_and_com(tmp_path):
    config = rebot_urdf()
    robot = RobotModel.from_urdf(config.urdf_path, tip_link=config.tip_link)
    payload = payload_parameters(0.6, np.array([0.01, -0.005, 0.06]), np.eye(3) * 1e-3)
    q = random_poses(robot, 40)
    offset = np.full(robot.dof, 0.05)
    tau = gravity_torque(robot, q, with_payload(robot, payload)) + offset
    result = fit_payload(robot, q, tau, offset=offset)
    assert result.mass == pytest.approx(0.6, abs=1e-6)
    np.testing.assert_allclose(result.com, [0.01, -0.005, 0.06], atol=1e-6)
    assert np.all(result.rmse_payload < 1e-6)


def test_package_export_with_and_without_payload(tmp_path):
    import xml.etree.ElementTree as ET

    from rebot_taccap_identification.package import export_package
    from rebot_taccap_identification.payload import cad_payload

    config = rebot_urdf()
    template = Path(config.urdf_path)
    arm = RobotModel.from_urdf(template, tip_link=config.tip_link)
    identified = arm.nominal_parameters[:, :10] * 1.1
    arm_urdf = tmp_path / "models" / "arm.urdf"
    export_urdf(arm, identified, arm_urdf, tip_link=config.tip_link)

    plain = export_package(template, arm_urdf, tmp_path / "plain", tip_link="link6")
    root = ET.parse(plain.urdf).getroot()
    assert {"adapter_link", "taccap_base_link", "taccap_left_finger_link"}.isdisjoint(
        link.get("name") for link in root.findall("link"))
    assert all((plain.urdf.parent / m.get("filename")).is_file() for m in root.iter("mesh"))
    assert not any(name.startswith(("c1_adapter/", "taccap_")) for name in plain.meshes)
    assert root.find("link[@name='link6']/inertial") is None

    cad = cad_payload(template, "link6")
    payload = {"name": "gripper", "link": "link6", "mass_kg": 0.6, "com_m": [0.01, 0.0, 0.05],
               "inertial_10": payload_parameters(0.6, np.array([0.01, 0.0, 0.05]), cad[2] * 0.6 / cad[0]).tolist()}
    full = export_package(template, arm_urdf, tmp_path / "full", tip_link="link6", payload=payload)
    # Arm and gripper both have a base_link.STL: sub-folders keep them apart.
    assert {"rebot_b601_rs/base_link.STL", "taccap_description/base_link.STL"} <= set(full.meshes)
    external_package = config.gripper_urdf_path.parents[1]
    assert (tmp_path / "full/LICENSES/taccap_description/NOTICE.md").read_bytes() == (external_package / "NOTICE.md").read_bytes()
    assert (tmp_path / "full/LICENSES/taccap_description/LICENSE").is_file()
    assert ET.parse(full.urdf).getroot().find("link[@name='link6']/inertial") is None
    np.testing.assert_allclose(full.payload["com_m"], payload["com_m"], atol=1e-9)
    np.testing.assert_allclose(RobotModel.from_urdf(full.urdf, tip_link="link6").nominal_parameters[:, :10],
                               identified, atol=1e-8)


def test_full_urdf_counts_links_below_tip_as_payload():
    from rebot_taccap_identification.payload import cad_payload
    from rebot_taccap_identification.runtime.common import Dynamics

    config = rebot_urdf()
    cut = Dynamics(config.urdf_path, config.tip_link)
    full = Dynamics(config.urdf_path, config.tip_link, full=True)
    body = cad_payload(config.urdf_path, config.tip_link)
    import xml.etree.ElementTree as ET

    gripper = ET.parse(config.gripper_urdf_path).getroot()
    gripper_mass = sum(float(m.get("value")) for m in gripper.findall("link/inertial/mass"))
    assert full.payload_mass == pytest.approx(body[0]) == pytest.approx(0.02 + gripper_mass)
    q = np.random.default_rng(0).uniform(-1, 1, (20, 6))
    merged = cut.robot.nominal_parameters[:, :10].copy()
    merged[-1] += payload_parameters(*body)
    np.testing.assert_allclose(full.gravity(q), gravity_torque(cut.robot, q, merged), atol=1e-12)
    assert np.abs(full.gravity(q) - cut.gravity(q)).max() > 0.5


def test_static_check_samples_j1_by_default():
    from rebot_taccap_identification.robots.utils import make_robot_from_config
    from rebot_taccap_identification.runtime.common import Dynamics
    from rebot_taccap_identification.runtime.static_check import default_sample, generate_poses, pose_space

    robot = make_robot_from_config(rebot_urdf())
    dynamics = Dynamics.for_robot(robot)
    _, weights = pose_space(robot, dynamics)
    assert weights[0] == 0 and weights[5] == 0          # vertical J1, massless link6: no gravity
    assert default_sample(weights) == [True, True, True, True, True, False]
    q = generate_poses(robot, dynamics, 12, 0)
    assert np.ptp(q[:, 0]) > np.radians(30) and np.ptp(q[:, 5]) == 0


def test_generated_poses_keep_approach_moves_off_the_table():
    from rebot_taccap_identification.robots.utils import make_robot_from_config
    from rebot_taccap_identification.runtime.common import Dynamics, check_clearance
    from rebot_taccap_identification.runtime.static_check import approach_offsets, generate_poses

    robot = make_robot_from_config(rebot_urdf())
    dynamics = Dynamics.for_robot(robot).with_tool(robot)
    q = generate_poses(robot, dynamics, 10, 3, table_z=0.0, delta=np.radians(5.0))
    offsets = np.array([approach_offsets(p, np.radians(5.0), *robot.safety_window()) for p in q])
    approaches = np.vstack([q - offsets, q + offsets])
    report = check_clearance(dynamics, approaches, np.arange(float(len(approaches))), 0.0, 0.01)
    assert report["table"]["min_z"] >= 0.01


def test_payload_refits_last_joint_offset():
    config = rebot_urdf()
    robot = RobotModel.from_urdf(config.urdf_path, tip_link=config.tip_link)
    payload = payload_parameters(0.65, np.array([-0.022, 0.005, 0.076]), np.eye(3) * 1e-3)
    q = random_poses(robot, 60)
    q[:, 5] = np.random.default_rng(2).uniform(-2, 2, len(q))     # the wrist turns with the payload
    arm_offset = np.array([0.0, -0.14, 0.02, 0.04, 0.03, 0.19])  # J6 held at one angle without the payload
    true_offset = arm_offset.copy()
    true_offset[5] = -0.03
    tau = gravity_torque(robot, q, with_payload(robot, payload)) + true_offset
    result = fit_payload(robot, q, tau, offset=arm_offset)
    assert result.mass == pytest.approx(0.65, abs=1e-6)
    np.testing.assert_allclose(result.com, [-0.022, 0.005, 0.076], atol=1e-6)
    np.testing.assert_allclose(result.offset, true_offset, atol=1e-6)


def test_payload_poses_sample_every_joint():
    from rebot_taccap_identification.robots.utils import make_robot_from_config
    from rebot_taccap_identification.runtime.common import Dynamics
    from rebot_taccap_identification.runtime.static_check import generate_poses, payload_sample

    robot = make_robot_from_config(rebot_urdf())
    dynamics = Dynamics.for_robot(robot).with_tool(robot)
    q = generate_poses(robot, dynamics, 12, 0, sample=payload_sample(robot.dof))
    assert np.all(np.ptp(q, axis=0) > np.radians(30))


def test_gravity_feedforward_scale():
    from types import SimpleNamespace

    from rebot_taccap_identification.robots.utils import make_robot_from_config
    from rebot_taccap_identification.runtime.common import Dynamics
    from rebot_taccap_identification.runtime.gravity_comp import Compensator

    robot = make_robot_from_config(rebot_urdf())
    dynamics = Dynamics.for_robot(robot)
    args = SimpleNamespace(wall_kp=0.0, offset_file=None, no_offset=True, ff_scale=[1, 1.04, 1, 1, 1, 1])
    q = np.radians([0, 120, 60, 0, 0, 0])
    _, tau = Compensator(robot, dynamics, args)(q)
    np.testing.assert_allclose(tau, dynamics.gravity(q)[0] * [1, 1.04, 1, 1, 1, 1])
