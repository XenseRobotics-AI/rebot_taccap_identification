"""Robot config/driver, results layout, drift analysis and the hardware workflow on a test-only arm."""

from pathlib import Path
import sys
import types

import numpy as np
import pytest

from rebot_taccap_identification.cli import main
from rebot_taccap_identification.robots import make_robot, make_robot_config, pop_robot_args, register_robots, robot_args
import bench_arm  # noqa: F401 - registers bench_test_arm
from test_visual import boxed_arm


def arm_fields(tmp_path, **extra) -> dict:
    """Fields for a 3-axis test arm; calibration goes to tmp_path, not ~/.cache."""
    return {"urdf": str(boxed_arm(tmp_path)), "tip_link": "l3", "results_dir": str(tmp_path / "results"),
            "calibration_dir": str(tmp_path / "calibration"), **extra}


def arm_args(tmp_path, robot_type: str = "bench_test_arm", **extra) -> list[str]:
    return robot_args(robot_type, arm_fields(tmp_path, **extra))


def test_config_fields_limits_and_errors(tmp_path):
    robot = make_robot("bench_test_arm", **arm_fields(tmp_path, kp=[10, 20, 30], torque_limits=5, rest_deg=[0, 10, 0],
                                                 soft_limits_deg=[[-90, 90], [-30, 120], [-180, 180]]))
    assert robot.dof == 3 and robot.results_dir() == tmp_path / "results"
    lower, upper = robot.safety_window()
    np.testing.assert_allclose(np.degrees(lower), [-90, -30, -np.degrees(2)])
    np.testing.assert_allclose(np.degrees(upper), [90, np.degrees(1), np.degrees(2)])
    with pytest.raises(ValueError, match="没有字段 chanel"):
        make_robot_config("bench_test_arm", chanel="can0")


def test_robot_arguments():
    rest, robot_type, fields = pop_robot_args(
        ["static", "--robot.type=rebot_b601_rs", "--robot.channel", "can1", "--robot.kp=[1, 2.5]", "--random", "5"])
    assert rest == ["static", "--random", "5"] and robot_type == "rebot_b601_rs"
    assert fields == {"channel": "can1", "kp": [1, 2.5]}
    assert pop_robot_args(robot_args(robot_type, fields)) == ([], robot_type, fields)


def test_calibration_is_per_id(tmp_path):
    robot = make_robot("bench_test_arm", **arm_fields(tmp_path, id="arm2"))
    assert robot.torque_gain().tolist() == [1, 1, 1]
    assert robot.save_torque_gain([1, 1.05, 1]) == tmp_path / "calibration" / "arm2.json"
    np.testing.assert_allclose(make_robot("bench_test_arm", **arm_fields(tmp_path, id="arm2")).torque_gain(), [1, 1.05, 1])


def test_rebot_registers_and_ships_urdf_and_meshes():
    import rebot_taccap_identification.robots
    from rebot_taccap_identification.robot import RobotModel
    from rebot_taccap_identification.visual import load_visuals

    assert not register_robots()
    robot = make_robot("rebot_b601_rs")
    assert len(robot.config.motors) == 6
    package = Path(rebot_taccap_identification.robots.__file__).parent / "rebot_b601_rs"
    model = RobotModel.from_urdf(robot.config.resolve(robot.config.urdf), tip_link=robot.config.tip_link)
    for visual in load_visuals(model, robot.config.tip_link):
        if visual.kind == "mesh":
            assert visual.path.is_file() and visual.path.resolve().is_relative_to(package), visual.path


def test_rebot_driver_maps_directions_offsets_and_model_limits(tmp_path, monkeypatch):
    sent = []

    class Motor:
        def __init__(self, mid):
            self.mid = mid

        def get_state(self):
            return types.SimpleNamespace(pos=1.0 + self.mid, vel=0.5, torq=2.0, t_mos=40.0)

        def send_mit(self, *values):
            sent.append((self.mid, values))

    class Controller:
        def __init__(self, channel): self.channel = channel
        def add_robstride_motor(self, mid, fid, model): return Motor(mid)
        def poll_feedback_once(self): pass
        def shutdown(self): pass
        def close(self): pass

    monkeypatch.setitem(sys.modules, "motorbridge", types.SimpleNamespace(Controller=Controller, Mode=None))
    motors = [{"id": i, "model": f"rs-0{m}", "direction": d, "offset_deg": o}
              for i, m, d, o in ((1, 6, 1, 0), (2, 6, -1, 0), (3, 0, 1, 10))]
    robot = make_robot("rebot_b601_rs", **arm_fields(tmp_path), soft_limits_deg=None, motors=motors)
    np.testing.assert_allclose(robot.torque_limits(), [36, 36, 14])
    robot.connect()
    state = robot.read()
    np.testing.assert_allclose(state.q, [2.0, -3.0, 4.0 - np.radians(10)])
    robot.command(np.zeros(3), np.ones(3), np.ones(3), np.ones(3), np.ones(3))
    assert dict(sent)[2][4] == -1.0                                          # torque flipped
    robot.disconnect()


def test_results_layout(tmp_path):
    from rebot_taccap_identification.results import Results

    real = Results.for_robot(make_robot("bench_test_arm", **arm_fields(tmp_path)))
    assert real.next_version() == "v1"
    for version in ("v1", "v10", "v2"):
        real.save_inputs(version, static=["train"])
    real.urdf("v2", "arm").write_text("<robot/>")
    assert real.versions() == ["v1", "v2", "v10"] and real.next_version() == "v11"
    assert real.model_urdfs() == [("v2", real.urdf("v2", "arm"))]
    gripper = real.payload("gripper")
    (gripper.root / "models" / "v2").mkdir(parents=True)
    assert real.payload_of(gripper.root / "models" / "v2" / "x.urdf") == "gripper"


def test_drift_detection_and_screening():
    from rebot_taccap_identification.runtime.drift import Drift, find_drifts, screen

    rate = 100
    t = np.arange(0, 12, 1 / rate)
    q = np.zeros((t.size, 3))
    hand = (t > 1) & (t < 1.5)                       # fast push of J1: 60° in 0.5 s
    q[hand, 0] = np.radians(120) * (t[hand] - 1)
    q[t >= 1.5, 0] = np.radians(60)
    slow = (t > 4) & (t < 8)                         # J2 creeps 20° in 4 s on its own
    q[slow, 1] = -np.radians(5) * (t[slow] - 4)
    q[t >= 8, 1] = -np.radians(20)
    drifts = find_drifts(t, q, np.gradient(q, t, axis=0), kd=0.5)
    assert [(d.joint, round(d.travel_deg)) for d in drifts] == [(1, -20)]

    pose = [0.0, 10.0, 5.0]
    make = lambda joint, travel, peak, at=pose: Drift(joint, 0.0, 1.0, travel, list(at), [list(at)], 0.1,
                                                      np.radians(peak))
    kept, dropped = screen([make(1, 15, 10), make(2, 10, 45), make(2, 12, 10),
                            make(2, -12, 10, [0.0, 12.0, 8.0]), make(2, -12, 10, [0.0, 60.0, 8.0])])
    assert [(d.joint, d.travel_deg) for d in kept] == [(1, 15), (2, -12)]
    assert dropped == {"hand": 1, "no_gravity": 0, "conflict": 2}


def test_hardware_workflow_end_to_end(tmp_path, capsys):
    """On the bench test arm: static check -> identify (with acceptance table) -> short gravity run ->
payload refused (nothing mounted) -> package."""
    cfg = arm_args(tmp_path, rate=100, kp=30, kd=2, torque_limits=40)
    results = tmp_path / "results"
    fast = ["--move-speed", "5", "--execute"]
    assert main(["ping", *cfg]) == 0
    assert "3/3" in capsys.readouterr().out
    for name, seed in (("train_rand12", 0), ("val_rand6", 100)):
        main(["static", *cfg, "--random", "12" if "train" in name else "6", "--seed", str(seed),
              "--settle", "0.05", "--hold", "0.1", "--out", str(results / "static" / f"{name}.csv"), *fast])
    assert "测得：torque_gain" in capsys.readouterr().out
    assert main(["identify", *cfg, "--static", str(results / "static" / "train_rand12.csv"),
                 "--val", str(results / "static" / "val_rand6.csv"), "--prior-weight", "1e-4"]) == 0
    out = capsys.readouterr().out
    urdf = next((results / "models" / "v1").glob("*_identified_v1.urdf"))
    assert "已导出" in out and urdf.with_name(urdf.stem + "_offset.yaml").is_file()
    # The bench arm is 10 % heavier than its CAD: the identified model's RMS is below 20 % of the CAD model's.
    table = [line for line in out.splitlines() if line.startswith(("原始", "v1"))]
    rms = [float(line.split()[1].split("/")[0]) for line in table]
    assert rms[1] < 0.2 * rms[0], table
    main(["gravity", *cfg, "--urdf", str(urdf), "--duration", "0.3", *fast])
    assert "电机已失能" in capsys.readouterr().out
    payload = results / "gripper" / "models" / "v1" / "arm_gripper_identified_v1.urdf"
    with pytest.raises(SystemExit, match="看不到负载"):           # nothing mounted on the simulated arm
        main(["payload", *cfg, "--urdf", str(urdf), "--static", str(results / "static" / "train_rand12.csv"),
              "--out-urdf", str(payload)])
    assert main(["package", *cfg, "--model", str(urdf), "--template", str(boxed_arm(tmp_path))]) == 0
    assert any((results / "export").rglob("*.urdf"))
