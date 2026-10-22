"""Offscreen smoke test of the studio GUI on the test-only bench arm (no 3-D view)."""

import os

import pytest

pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

import numpy as np  # noqa: E402

import bench_arm  # noqa: E402,F401
from test_robots import arm_fields  # noqa: E402


@pytest.fixture(scope="module")
def app():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_studio_pages_and_commands(app, tmp_path):
    from rebot_taccap_identification.studio import app as studio

    fields = arm_fields(tmp_path, torque_limits=30)
    studio.make_robot("bench_test_arm", **fields).save_torque_gain([1, 1.05, 1])
    window = studio.Studio("bench_test_arm", fields, with_3d=False)
    assert window.steps.count() == len(studio.STEPS) == 6
    for row in range(len(studio.STEPS)):
        window.steps.setCurrentRow(row)
    assert window.version.text() == "v1" and [s.value() for s in window.gain] == [1.0, 1.05, 1.0]
    started = []
    window.start = lambda *commands, **kwargs: started.extend(commands)
    window.ping()
    assert started[-1][3:5] == ["ping", "--robot.type=bench_test_arm"]
    static = window.results.static_dir
    static.mkdir(parents=True)
    for name in ("train_a", "drift_b", "val_c"):
        (static / f"{name}.csv").write_text("q_deg1,q_deg2,q_deg3,tau_meas1,tau_meas2,tau_meas3\n0,0,0,0,0,0\n")
    window.refresh_files()
    assert window.checked("static") == ["drift_b", "train_a"] and window.checked("holdout") == ["val_c"]
    for i in range(20):                        # many files: the list grows (up to a limit) and counts them
        (static / f"train_long_name_{i:02d}_drift_20261013_152111_gripper_identified_v2.csv").write_text(
            "q_deg1,q_deg2,q_deg3,tau_meas1,tau_meas2,tau_meas3\n0,0,0,0,0,0\n")
    small = window.lists["static"].height()
    window.refresh_files()
    assert window.lists["static"].height() > small and window.lists["static"].count() == 23
    caption = window.lists["static"].parentWidget().findChild(studio.QLabel)
    assert "已选 22/23" in caption.text()
    for i in range(20):
        (static / f"train_long_name_{i:02d}_drift_20261013_152111_gripper_identified_v2.csv").unlink()
    window.refresh_files()
    window.run_identify()
    command = started[-1]
    assert command[3] == "identify" and any(arg.endswith("drift_b.csv@5") for arg in command)
    assert command[command.index("--val") + 1].endswith("val_c.csv")
    window.ff_scale[1].setValue(1.04)
    window.confirm_hardware = lambda: True
    window.gravity_model.setCurrentIndex(0)
    window.run_gravity()
    command = started[-1]
    assert command[3] == "gravity" and command[command.index("--ff-scale") + 1:][:3] == ["1", "1.04", "1"]
    window.close()


def test_dm_studio_uses_serial_and_forwards_channel(app, tmp_path):
    from rebot_taccap_identification.studio import app as studio

    fields = {"channel": "/dev/ttyACM2", "results_dir": str(tmp_path / "results"),
              "calibration_dir": str(tmp_path / "cal")}
    window = studio.Studio("rebot_b601_dm", fields, with_3d=False)
    assert window.dof == 6 and window.steps.count() == 6
    assert not hasattr(window, "show_can_commands")
    assert any("DM 串口" in label.text() for label in window.findChildren(studio.QLabel))
    started = []
    window.start = lambda *commands, **kwargs: started.extend(commands)
    window.ping()
    assert {"--robot.type=rebot_b601_dm", "--robot.channel=/dev/ttyACM2"} <= set(started[-1])
    assert not any(arg.startswith("--robot.id=") for arg in started[-1])
    window.close()


def test_payload_drift_poses_load_into_payload_step(app, tmp_path):
    from rebot_taccap_identification.runtime.static_check import save_pose_file
    from rebot_taccap_identification.studio import app as studio

    window = studio.Studio("bench_test_arm", arm_fields(tmp_path, torque_limits=30), with_3d=False)
    poses = window.results.payload("gripper").poses("drift_x_gripper_v1")
    save_pose_file(poses, np.zeros((2, 3)))
    window.show_drift({"n_drifts": 1, "summary": ["J2"], "poses": str(poses), "n_poses": 2})
    assert "第 4 步" in window.drift_load.text()
    window.load_drift_poses()
    assert window.steps.currentRow() == 4 and window.payload_name.text() == "gripper"
    assert len(window.payload_poses) == 2
    window.close()


def test_results_are_parsed_after_the_log_widget_is_full(app, tmp_path):
    """The log keeps only the last 5000 lines; a run's result must still be found."""
    from rebot_taccap_identification.studio import app as studio

    window = studio.Studio("bench_test_arm", arm_fields(tmp_path, torque_limits=30), with_3d=False)
    for i in range(6000):
        window.log_line(f"old line {i}")
    callbacks = []
    window.runner.run = lambda *commands: None
    window.start = (lambda original: lambda *c, on_done=None, live=False: (original(*c, on_done=on_done, live=live),
                                                                          callbacks.append(on_done)))(window.start)
    window.ping()
    window.log_line("结果：3/3 个关节应答")
    callbacks[-1](True)
    assert "已连接：3/3" in window.connection.text()
    window.close()
