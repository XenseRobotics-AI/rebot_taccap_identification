"""DM hardware contract with a fake bus, and identification on the actual DM model."""

import json
import sys
import time
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rebot_taccap_identification import RobotModel, gravity_torque
from rebot_taccap_identification.cli import main
from rebot_taccap_identification.export import load_offsets, offsets_path
from rebot_taccap_identification.payload import payload_parameters
from rebot_taccap_identification.robots import make_robot, robot_args
from rebot_taccap_identification.runtime.common import SafetyStop
from test_robots import arm_fields


@pytest.fixture
def bus(monkeypatch):
    events = []

    class Motor:
        def __init__(self, mid, fid, model):
            self.mid, self.fid, self.model = mid, fid, model
            self.fail_probe = self.fail_enable = False
            self.tmax = 28.0 if model == "4340P" else 10.0
            self.state = types.SimpleNamespace(pos=0.1 * mid, vel=0.2, torq=1.2, t_mos=35, t_rotor=42,
                                               status_code=0)

        def get_register_u32(self, rid, timeout_ms):
            if self.fail_probe:
                raise TimeoutError("missing joint reply")
            return self.mid if rid == 8 else self.fid

        def get_register_f32(self, rid, timeout_ms):
            return {21: 12.5, 22: 10.0 if self.model == "4340P" else 30.0, 23: self.tmax}[rid]

        def get_state(self): return self.state
        def request_feedback(self): pass
        def close(self): events.append(("close_motor", self.mid))
        def ensure_mode(self, mode): events.append(("mode", self.mid, mode))
        def send_mit(self, *values): events.append(("mit", self.mid, values))

        def enable(self):
            if self.fail_enable:
                raise RuntimeError("enable failed")
            events.append(("enable", self.mid))
            self.state.status_code = 1

        def disable(self):
            events.append(("disable", self.mid))
            self.state.status_code = 0

    class Controller:
        def __init__(self, channel): events.append(("socketcan", channel))

        @classmethod
        def from_dm_serial(cls, channel, baud):
            events.append(("serial", channel, baud))
            return cls.__new__(cls)

        def add_damiao_motor(self, mid, fid, model):
            events.append(("add", mid, fid, model))
            return Motor(mid, fid, model)

        def poll_feedback_once(self): pass
        def close(self): events.append(("close_bus",))

    monkeypatch.setitem(sys.modules, "motorbridge", types.SimpleNamespace(Controller=Controller,
                                                                       Mode=types.SimpleNamespace(MIT=1)))
    return events


def driver(tmp_path, **kwargs):
    motors = [{"id": i, "model": "4340P", "direction": -1 if i == 2 else 1, "offset_deg": 10 if i == 3 else 0}
              for i in range(1, 4)]
    fields = dict(**arm_fields(tmp_path), motors=motors, channel="/dev/test-dm", torque_limits=20,
                  kp=10, kd=1, soft_limits_deg=None)
    return make_robot("rebot_b601_dm", **(fields | kwargs))


@pytest.mark.parametrize("transport,channel", [("serial", "/dev/test-dm"), ("socketcan", "can-test")])
def test_ping_is_read_only_and_verifies_transport_and_addresses(tmp_path, bus, transport, channel):
    r = driver(tmp_path, transport=transport, channel=channel)
    assert all(ok for _, ok, _ in r.ping())
    assert bus[0][:2] == (transport, channel)
    assert ("add", 2, 0x12, "4340P") in bus
    assert not any(e[0] in ("enable", "disable", "mode", "mit") for e in bus)
    assert r.ctrl is None and not r.motors
    r.disconnect()


def test_dm_mapping_and_command_validation(tmp_path, bus):
    r = driver(tmp_path)
    try:
        r.enable()
        state = r.read()
        np.testing.assert_allclose(state.q, [.1, -.2, .3 - np.radians(10)])
        np.testing.assert_allclose(state.tau, [1.2, -1.2, 1.2])
        np.testing.assert_allclose(state.temperature, 42)
        r.command([0, .1, .2], .3, 10, 1, 2)
        commands = {e[1]: e[2] for e in bus if e[0] == "mit"}
        np.testing.assert_allclose(commands[2], [-.1, -.3, 10, 1, -2])
        assert commands[3][0] == pytest.approx(.2 + np.radians(10))
        count = len([e for e in bus if e[0] == "mit"])
        for q, kd, tau in ((np.nan, 1, 0), (0, 6, 0), (0, 1, 21), (13, 1, 0)):
            with pytest.raises(SafetyStop):
                r.command(q, 0, 10, kd, tau)
        assert len([e for e in bus if e[0] == "mit"]) == count
    finally:
        r.disconnect()
    assert not r.enabled


def test_enable_failure_disables_all_including_partial_enable(tmp_path, bus):
    r = driver(tmp_path)
    r.connect()
    r.motors[1].fail_enable = True
    with pytest.raises(RuntimeError, match="enable failed"):
        r.enable()
    assert not r.enabled
    assert all(m.state.status_code == 0 for m in r.motors)
    assert {e[1] for e in bus if e[0] == "disable"} == {1, 2, 3}
    r.disconnect()


def test_wrong_hardware_range_refused_before_enable(tmp_path, bus):
    r = driver(tmp_path)
    r.connect()
    r.motors[1].tmax = 10
    with pytest.raises(ValueError, match="配置不匹配"):
        r.enable()
    assert not any(e[0] == "enable" for e in bus)
    r.disconnect()


@pytest.mark.parametrize("change,match", [("fault", "故障码"), ("disabled", "意外失能"), ("nan", "非有限")])
def test_faults_and_bad_feedback_stop_the_runtime(tmp_path, bus, change, match):
    r = driver(tmp_path)
    try:
        r.enable()
        if change == "nan":
            r.motors[1].state.torq = np.nan
        else:
            r.motors[1].state.status_code = 8 if change == "fault" else 0
        with pytest.raises(SafetyStop, match=match):
            r.read()
    finally:
        r.disconnect()


def test_one_missing_joint_is_not_masked_by_other_joints(tmp_path, bus):
    r = driver(tmp_path)
    try:
        r.enable()
        r.motors[1].fail_probe = True
        deadline = time.perf_counter() + 1
        while r._error is None and time.perf_counter() < deadline:
            r.motors[0].state.pos += .01
            time.sleep(.005)
        with pytest.raises(SafetyStop, match="j2 DM 通信中断"):
            r.read()
        with pytest.raises(SafetyStop):
            r.command(0, 0, 10, 1, 0)
    finally:
        r.disconnect()


@pytest.fixture
def heartbeat_driver(tmp_path, bus, monkeypatch):
    """Drive heartbeat cycles with a deterministic clock, without hardware or watchdog threads."""
    from rebot_taccap_identification.robots.rebot_b601_dm import rebot_b601_dm as dm

    r = driver(tmp_path)
    r.connect()
    clock = types.SimpleNamespace(now=10.0)
    monkeypatch.setattr(dm, "time", types.SimpleNamespace(perf_counter=lambda: clock.now))
    r.enabled = True
    for motor in r.motors:
        motor.state.status_code = 1
    r._last_reply[:] = clock.now
    try:
        yield r, clock
    finally:
        r.disconnect()


@pytest.mark.parametrize("error", [TimeoutError("missing joint reply"),
                                  RuntimeError("get_register_u32 failed: register 8 not received within 80ms")])
def test_heartbeat_retries_one_timeout_without_refreshing_deadline(heartbeat_driver, monkeypatch, caplog, error):
    r, clock = heartbeat_driver
    motor = r.motors[1]
    read_register = motor.get_register_u32

    def missed_reply(*args, **kwargs):
        raise error

    monkeypatch.setattr(motor, "get_register_u32", missed_reply)
    clock.now += .08
    assert r._heartbeat_once()
    assert r._last_reply[1] == 10.0 and r._error is None
    assert r.read() is not None
    r.command(0, 0, 10, 1, 0)
    assert "继续重试" in caplog.text
    monkeypatch.setattr(motor, "get_register_u32", read_register)
    clock.now += .1
    assert r._heartbeat_once()
    assert r._last_reply[1] == clock.now and not r._probe_errors[1]
    assert r._error is None


@pytest.mark.parametrize("missing", [[1], [0, 1, 2]])
def test_repeated_timeouts_expire_per_joint_and_remain_latched(heartbeat_driver, missing):
    r, clock = heartbeat_driver
    for i in missing:
        r.motors[i].fail_probe = True
    for elapsed in (.1, .2, .3, .4, .49):
        clock.now = 10 + elapsed
        assert r._heartbeat_once() and r.read() is not None
        np.testing.assert_equal(r._last_reply[missing], 10.0)
    clock.now = 10.501
    # The control loop must stop even if the next heartbeat hasn't run yet.
    with pytest.raises(SafetyStop, match="DM 通信中断.*上限 0.500s"):
        r.command(0, 0, 10, 1, 0)
    assert not r._heartbeat_once()
    for i in missing:
        r.motors[i].fail_probe = False
    r._last_reply[:] = clock.now
    with pytest.raises(SafetyStop, match="DM 通信中断"):
        r.read()


@pytest.mark.parametrize("failure", ["io", "wrong_id"])
def test_heartbeat_does_not_retry_transport_or_protocol_errors(heartbeat_driver, monkeypatch, failure):
    r, clock = heartbeat_driver

    def broken_read(*args, **kwargs):
        if failure == "io":
            raise RuntimeError("get_register_u32 failed: serial port disconnected")
        return 9

    monkeypatch.setattr(r.motors[1], "get_register_u32", broken_read)
    clock.now += .01
    assert not r._heartbeat_once()
    with pytest.raises(SafetyStop, match="serial port disconnected|ESC_ID=9"):
        r.read()


def test_late_successful_reply_does_not_renew_expired_deadline(heartbeat_driver, monkeypatch):
    r, clock = heartbeat_driver
    clock.now = 10.49
    r._last_reply[[0, 2]] = clock.now

    def late_reply(*args, **kwargs):
        clock.now = 10.51
        return 2

    monkeypatch.setattr(r.motors[1], "get_register_u32", late_reply)
    assert not r._heartbeat_once()
    assert r._last_reply[1] == 10.0
    with pytest.raises(SafetyStop, match="j2 DM 通信中断"):
        r.read()


def test_dm_assets_and_gravity_sign():
    from test_mass import potential_energy

    r = make_robot("rebot_b601_dm")
    model = r.model
    np.testing.assert_allclose(r.torque_limits(), [27, 27, 27, 7, 7, 7])
    assert model.dof == 6 and not model.nominal_parameters[-1].any()
    q = np.array([0, -.8, -.8, .5, .2, -.3])
    h = 1e-6
    numeric = [(potential_energy(model, q + h * e) - potential_energy(model, q - h * e)) / (2 * h)
               for e in np.eye(6)]
    np.testing.assert_allclose(gravity_torque(model, q)[0], numeric, atol=1e-7)
    assert gravity_torque(model, q)[0, 2] < 0  # holding torque at the DM working pose
    # MIT-licensed reBot-Isaacsim CAD priors; identify again before hardware use.
    np.testing.assert_allclose(model.nominal_parameters[:, 0],
                               [.0840736498625263, .710394385134637, .524941641463797,
                                .191139210958177, .150004102098704, 0])
    root = ET.parse(r.config.resolve(r.config.urdf)).getroot()
    assert root.find("joint[@name='joint6']/origin").get("xyz") == "0.028008 0 0.04"
    for mesh in root.iter("mesh"):
        assert (r.config.resolve(r.config.urdf).parent / mesh.get("filename")).is_file()
    assert r.config.channel == "/dev/ttyACM0" and r.id is None
    assert r.results_dir().name == "rebot_b601_dm" and r.calibration_fpath.name == "default.json"


def test_dm_preflight_never_opens_bus(tmp_path, monkeypatch, capsys):
    from rebot_taccap_identification.robots.rebot_b601_dm import RebotB601DM

    monkeypatch.setattr(RebotB601DM, "connect", lambda _: pytest.fail("preview opened hardware"))
    cfg = robot_args("rebot_b601_dm", {"results_dir": str(tmp_path), "calibration_dir": str(tmp_path / "cal")})
    assert main(["static", *cfg, "--random", "6"]) == 0
    assert "[预览]" in capsys.readouterr().out


def test_dm_identify_payload_and_portable_export(tmp_path, monkeypatch, capsys):
    """Train on synthetic measured torques; independently validate unseen DM poses.

    Per-link masses are not uniquely observable; gravity predictions and payload
    mass/CoM are the quantities checked here, not a spurious exact mass recovery.
    """
    from rebot_taccap_identification.robots.rebot_b601_dm import RebotB601DM
    from rebot_taccap_identification.runtime.common import Dynamics

    monkeypatch.setattr(RebotB601DM, "connect", lambda _: pytest.fail("offline fit opened hardware"))
    fields = dict(results_dir=str(tmp_path / "results"), calibration_dir=str(tmp_path / "cal"))
    cfg = robot_args("rebot_b601_dm", fields)
    robot = make_robot("rebot_b601_dm", **fields)
    model = robot.model
    rng = np.random.default_rng(11)
    lower, upper = robot.safety_window(5)
    train, val = rng.uniform(lower, upper, (100, 6)), rng.uniform(lower, upper, (35, 6))
    truth = model.nominal_parameters[:, :10].copy()
    truth[1:5, 0] *= 1.12
    truth[1:5, 1:4] *= 1.18
    bias = np.array([.01, -.04, .06, -.02, .01, .02])

    def csv(name, q, tau):
        path = tmp_path / f"{name}.csv"
        header = ",".join(f"{s}{i}" for s in ("q_deg", "tau_meas") for i in range(1, 7))
        np.savetxt(path, np.column_stack([np.degrees(q), tau]), header=header, comments="", delimiter=",")
        return str(path)

    training = csv("train", train, gravity_torque(model, train, truth) + bias)
    validation = csv("val", val, gravity_torque(model, val, truth) + bias)
    assert main(["identify", *cfg, "--static", training, "--val", validation, "--prior-weight", "1e-6"]) == 0
    arm = next((tmp_path / "results/models/v1").glob("*.urdf"))
    identified = Dynamics(arm, "link6")
    np.testing.assert_allclose(identified.gravity(val) + load_offsets(offsets_path(arm), model.joint_names),
                               gravity_torque(model, val, truth) + bias, atol=2e-4)
    assert not RobotModel.from_urdf(arm, tip_link="link6").nominal_parameters[-1].any()
    from rebot_taccap_identification.export import inertial_to_urdf
    for fitted, original in zip(identified.inertial[:5], model.nominal_parameters[:5, :10]):
        np.testing.assert_allclose(inertial_to_urdf(fitted)[2], inertial_to_urdf(original)[2], atol=1e-10)
    result = json.loads((arm.parent / "result.json").read_text())
    assert result["robot_type"] == "rebot_b601_dm" and result["robot_id"] is None
    assert result["identification_kind"] == "static_gravity" and result["rank"] < 4 * 5

    payload = payload_parameters(.85, np.array([-.025, .008, .105]), np.eye(3) * .002)
    loaded = truth.copy()
    loaded[-1] += payload
    train_load = csv("payload_train", train, gravity_torque(model, train, loaded) + bias)
    val_load = csv("payload_val", val, gravity_torque(model, val, loaded) + bias)
    out = tmp_path / "results/taccap/models/v1/dm_taccap.urdf"
    assert main(["payload", *cfg, "--urdf", str(arm), "--static", train_load, "--val", val_load,
                 "--name", "taccap", "--out-urdf", str(out)]) == 0
    info = json.loads(out.with_name("payload.json").read_text())
    assert info["mass_kg"] == pytest.approx(.85, abs=2e-4)
    np.testing.assert_allclose(info["com_m"], [-.025, .008, .105], atol=2e-4)
    np.testing.assert_allclose(Dynamics(out, "link6").gravity(val) + load_offsets(offsets_path(out), model.joint_names),
                               gravity_torque(model, val, loaded) + bias, atol=2e-4)
    assert main(["package", *cfg, "--model", str(out)]) == 0
    exported = next((tmp_path / "results/export").glob("*/urdf/*.urdf"))
    # The package keeps separate payload geometry/inertias below link6.
    np.testing.assert_allclose(Dynamics(exported, "link6", full=True).gravity(val)
                               + load_offsets(offsets_path(exported), model.joint_names),
                               gravity_torque(model, val, loaded) + bias, atol=2e-4)
    assert (exported.parents[1] / "LICENSES").is_dir()
    for mesh in ET.parse(exported).getroot().iter("mesh"):
        assert (exported.parent / mesh.get("filename")).is_file()
    np.testing.assert_allclose(load_offsets(offsets_path(exported), model.joint_names), bias, atol=2e-4)
    assert "数据确定了" in capsys.readouterr().out
