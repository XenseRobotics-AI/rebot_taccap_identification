"""Small URDFs and random poses for the tests."""

import numpy as np


def inertial(mass=2.0, xyz="0.4 0 0", rpy="0 0 0", diagonal=(0.02, 0.03, 0.04)):
    xx, yy, zz = diagonal
    return f'''<inertial><origin xyz="{xyz}" rpy="{rpy}"/>
      <mass value="{mass}"/><inertia ixx="{xx}" ixy="0" ixz="0"
      iyy="{yy}" iyz="0" izz="{zz}"/></inertial>'''


def write_robot(tmp_path, body):
    path = tmp_path / "robot.urdf"
    path.write_text(f'<robot name="test">{body}</robot>')
    return path


def random_poses(robot, n, seed=0):
    lower = np.array([j.lower for j in robot.config.links])
    upper = np.array([j.upper for j in robot.config.links])
    return np.random.default_rng(seed).uniform(lower, upper, size=(n, robot.dof))
