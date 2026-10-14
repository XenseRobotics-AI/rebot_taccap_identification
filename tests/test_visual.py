import numpy as np

from rebot_taccap_identification import RobotModel
from rebot_taccap_identification.visual import (
    clearance_report, load_visuals, read_stl, simplify_triangles, stl_bytes, surface_points,
)

from helpers import inertial, write_robot


def box(size, xyz):
    return f'<visual><origin xyz="{xyz}"/><geometry><box size="{size}"/></geometry></visual>'


def boxed_arm(tmp_path):
    """3-DOF arm with box visuals and a gripper below ``l3`` (j3 swings l3 in the vertical plane)."""
    return write_robot(tmp_path, f'''
      <link name="base">{box("0.2 0.2 0.02", "0 0 0.01")}</link>
      <link name="l1">{inertial(1.5, "0 0 0.1")}{box("0.06 0.06 0.2", "0 0 0.1")}</link>
      <link name="l2">{inertial(1.2, "0.2 0 0")}{box("0.4 0.05 0.05", "0.2 0 0")}</link>
      <link name="l3">{inertial(0.6, "0.15 0 0")}{box("0.3 0.04 0.04", "0.15 0 0")}</link>
      <link name="hand">{box("0.05 0.05 0.05", "0 0 0")}</link><link name="finger">{box("0.01 0.01 0.04", "0 0 0")}</link>
      <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.02"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="30" velocity="3"/></joint>
      <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0.2" rpy="{np.pi / 2} 0 0"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="30" velocity="3"/></joint>
      <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0.4 0 0"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="30" velocity="3"/></joint>
      <joint name="mount" type="fixed"><parent link="l3"/><child link="hand"/><origin xyz="0.3 0 0"/></joint>
      <joint name="grip" type="prismatic"><parent link="hand"/><child link="finger"/><axis xyz="0 1 0"/></joint>''')


def test_visuals_follow_owner_links_and_respect_tip_link(tmp_path):
    robot = RobotModel.from_urdf(boxed_arm(tmp_path), tip_link="l3")
    visuals = load_visuals(robot, tip_link="l3")
    assert [(v.owner, v.link) for v in visuals] == [(-1, "base"), (0, "l1"), (1, "l2"), (2, "l3")]
    np.testing.assert_allclose(visuals[2].transform[:3, 3], [0.2, 0, 0])
    points = surface_points(visuals, points_per_link=500)
    assert set(points) == {-1, 0, 1, 2}
    # Box surface samples stay on the box.
    assert np.all(np.abs(points[2] - [0.15, 0, 0]) <= np.array([0.15, 0.02, 0.02]) + 1e-12)


def test_stl_round_trip_and_simplification(tmp_path):
    rng = np.random.default_rng(0)
    triangles = rng.random((2000, 3, 3)) * 0.1
    path = tmp_path / "mesh.stl"
    path.write_bytes(stl_bytes(triangles))
    np.testing.assert_allclose(read_stl(path), triangles, atol=1e-7)
    simple = simplify_triangles(triangles, 0.02)
    assert 0 < len(simple) < len(triangles)
    assert np.all(simple.reshape(-1, 3).min(axis=0) >= -1e-9) and np.all(simple.reshape(-1, 3).max(axis=0) <= 0.1)


def test_clearance_finds_table_contact_only_where_it_happens(tmp_path):
    robot = RobotModel.from_urdf(boxed_arm(tmp_path), tip_link="l3")
    points = surface_points(load_visuals(robot, tip_link="l3"), points_per_link=800)
    time = np.linspace(0, 4, 81)
    q = np.zeros((time.size, 3))
    q[:, 2] = np.pi / 2 * np.sin(np.pi * time / 2)      # l3 swings up, back, then down
    report = clearance_report(robot, q, time, points, table_z=0.0, clearance=0.01, reference_q=np.zeros(3))
    assert report["table"]["violated"] and report["table"]["link"] == "l3"
    times = [event["time"] for event in report["events"]]
    assert times and min(times) > 0.5          # the start pose is clear
    assert all(event["links"] == ["l3"] for event in report["events"])
    level = clearance_report(robot, np.zeros((5, 3)), np.arange(5.0), points, reference_q=np.zeros(3))
    assert not level["events"] and not level["table"]["violated"]
