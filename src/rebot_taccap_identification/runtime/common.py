"""Robot-independent runtime for hardware identification: model, safety monitor, motions.

Works on any ``rebot_taccap_identification.robots.Robot``; the joint count, limits and gains come
from the robot's URDF and config.
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..gravity import gravity_torque
from ..robot import RobotModel
from ..robots import Robot, State
from ..robots.utils import ROBOT_HELP, cli_robot


class SafetyStop(RuntimeError):
    """Raised by the monitor; the caller brings the arm to rest."""


# ---------------------------------------------------------------- model
class Dynamics:
    """Gravity of a URDF (plus a mounted payload), used for feedforward and checks."""

    def __init__(self, urdf: str | Path, tip_link: str | None = None, gravity=(0.0, 0.0, -9.80665),
                 full: bool = False):
        self.robot = RobotModel.from_urdf(urdf, tip_link=tip_link, gravity=tuple(gravity))
        self.tip_link = tip_link
        self.inertial = self.robot.nominal_parameters[:, :10].copy()
        self.payload_mass = 0.0
        if full and tip_link is not None:
            # A complete URDF (step 5 package): what hangs below the tip link (a gripper, its
            # fingers at their zero position) is a payload rigidly attached to the last link.
            from ..payload import cad_payload

            self.add_payload(cad_payload(urdf, tip_link))
        # Collision geometry of a mounted tool (see visual.load_visuals); dynamics are unaffected.
        self.tool: tuple[float, float] | bool | None = None
        self.collision: "Dynamics | None" = None      # other URDF (same kinematics) for collision checks

    def add_payload(self, body: tuple[float, np.ndarray, np.ndarray] | None) -> "Dynamics":
        """Add a payload (mass, CoM, inertia about the CoM, in the last link's frame) to the last link."""
        if body is not None:
            from ..payload import payload_parameters

            self.inertial[-1] += payload_parameters(*body)
            self.payload_mass += body[0]
        return self

    def with_tool(self, robot: Robot, length: float = 0.0, radius: float = 0.03) -> "Dynamics":
        """Check collisions with the mounted tool: the parts the robot's URDF has below its tip link
        (e.g. the gripper), or a cylinder ``length`` x ``radius`` along the last link's z axis."""
        from ..visual import has_tool_links

        geometry = Dynamics(robot.config.urdf_path, robot.config.tip_link, robot.config.gravity)
        if length > 0:
            geometry.tool = (float(length), float(radius))
        elif has_tool_links(geometry.robot, robot.config.tip_link):
            geometry.tool = True
        else:
            raise SystemExit(f"{robot.config.urdf_path.name} 里 {robot.config.tip_link} 下面没有工具，"
                             "用 --tool-length/--tool-radius 给出工具尺寸")
        self.collision = geometry
        return self

    @classmethod
    def for_robot(cls, robot: Robot, urdf: str | Path | None = None, full: bool = False) -> "Dynamics":
        """The robot's model, or another URDF (e.g. an identified one) with the same tip/gravity.
        ``full``: the URDF is a complete package whose links below the tip are a mounted payload."""
        dynamics = cls(urdf or robot.config.urdf_path, robot.config.tip_link, robot.config.gravity, full)
        if dynamics.robot.dof != robot.dof:
            raise SystemExit(f"{urdf}: {dynamics.robot.dof} 个关节，机器人有 {robot.dof} 个")
        return dynamics

    @property
    def dof(self) -> int:
        return self.robot.dof

    def gravity(self, q) -> np.ndarray:
        return gravity_torque(self.robot, np.atleast_2d(np.asarray(q, dtype=float)), self.inertial)


_SURFACE_CACHE: dict = {}


def check_clearance(dynamics: Dynamics, q_path: np.ndarray, time_axis: np.ndarray, table_z: float,
                    clearance: float, reference_q=None) -> dict | None:
    """Mesh clearance along a joint path; None if the meshes are unavailable."""
    from ..visual import clearance_report, load_visuals, surface_points

    dynamics = dynamics.collision or dynamics
    key = (str(dynamics.robot.source), dynamics.tip_link, dynamics.tool)
    if key not in _SURFACE_CACHE:
        visuals = load_visuals(dynamics.robot, dynamics.tip_link, tool=dynamics.tool)
        has_meshes = any(v.path is not None and v.path.is_file() for v in visuals if v.kind == "mesh")
        _SURFACE_CACHE[key] = surface_points(visuals) if has_meshes else None
    points = _SURFACE_CACHE[key]
    if points is None:
        return None
    return clearance_report(dynamics.robot, q_path, time_axis, points, table_z=table_z, clearance=clearance,
                            reference_q=q_path[0] if reference_q is None else reference_q)


def describe_clearance(report: dict | None) -> tuple[bool, str]:
    if report is None:
        return True, "碰撞检查跳过（找不到网格文件）"
    table = report["table"]
    lines = [f"桌面最小间隙 {1000 * (table['min_z'] - table['table_z']):.0f} mm（{table['link']}，t={table['time']:.2f}s）"]
    for pair in report["pairs"][:3]:
        flag = " ⚠" if pair["violated"] else (" (参考位姿已接触，忽略)" if pair["touching_at_reference"] else "")
        lines.append(f"{pair['a']}↔{pair['b']} 最小 {1000 * pair['min_distance']:.0f} mm{flag}")
    ok = not report["events"]
    return ok, ("无碰撞；" if ok else f"⚠ {len(report['events'])} 个碰撞采样点；") + "；".join(lines)


# ----------------------------------------------------------------- control
class Ticker:
    """Fixed-rate loop timing with overrun accounting."""

    def __init__(self, rate: float):
        self.period = 1.0 / rate
        self.next = time.perf_counter()
        self.overruns = 0
        self.worst = 0.0

    def wait(self) -> None:
        self.next += self.period
        now = time.perf_counter()
        late = now - self.next
        if late > 0:
            self.worst = max(self.worst, late)
            if late > 0.5 * self.period:
                self.overruns += 1
            if late > 5 * self.period:                       # badly behind: resynchronise
                self.next = now
            return
        if late < -0.0015:
            time.sleep(-late - 0.001)
        while time.perf_counter() < self.next:
            pass


@dataclass
class Limits:
    lower: np.ndarray
    upper: np.ndarray
    torque: np.ndarray
    max_tracking: float = np.radians(8.0)
    torque_ratio: float = 0.85
    max_temperature: float = 100.0
    stale_cycles: int = 50
    check_window: bool = True


class Monitor:
    """Aborts on window, tracking, torque, temperature or frozen feedback."""

    def __init__(self, limits: Limits):
        self.limits = limits
        self._last = None
        self._same = 0

    def check(self, state: State | None, q_ref: np.ndarray) -> None:
        lim = self.limits
        if state is None:
            self._same += 1
            if self._same > lim.stale_cycles:
                raise SafetyStop("长时间收不到电机反馈")
            return
        signature = np.concatenate([state.q, state.dq, state.tau])
        self._same = self._same + 1 if self._last is not None and np.array_equal(signature, self._last) else 0
        self._last = signature
        if self._same > lim.stale_cycles:
            raise SafetyStop("电机反馈长时间不变（可能通信中断）")
        if lim.check_window:
            tol = np.radians(3.0)
            bad = np.flatnonzero((state.q < lim.lower - tol) | (state.q > lim.upper + tol))
            if bad.size:
                i = bad[0]
                raise SafetyStop(f"J{i + 1} 超出安全窗口：{np.degrees(state.q[i]):.1f}°")
        error = np.abs(state.q - q_ref)
        if np.any(error > lim.max_tracking):
            i = int(np.argmax(error))
            raise SafetyStop(f"J{i + 1} 跟踪误差 {np.degrees(error[i]):.1f}° 超过 {np.degrees(lim.max_tracking):.1f}°")
        ratio = np.abs(state.tau) / lim.torque
        if np.any(ratio > lim.torque_ratio):
            i = int(np.argmax(ratio))
            raise SafetyStop(f"J{i + 1} 力矩 {state.tau[i]:.2f} N·m 超过力矩上限的 {100 * lim.torque_ratio:.0f}%")
        temperature = np.nan_to_num(state.temperature, nan=-np.inf)
        if np.any(temperature > lim.max_temperature):
            i = int(np.argmax(temperature))
            raise SafetyStop(f"J{i + 1} 温度 {state.temperature[i]:.0f}°C 过高")


def quintic(q0: np.ndarray, q1: np.ndarray, duration: float, t: float) -> tuple[np.ndarray, np.ndarray]:
    """Rest-to-rest quintic interpolation (zero velocity and acceleration at both ends)."""
    s = np.clip(t / duration, 0.0, 1.0)
    p = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
    d = q1 - q0
    return q0 + p * d, v * d


class Controller:
    """Commands, monitoring and safe motions on top of a ``Robot``."""

    def __init__(self, robot: Robot, dynamics: Dynamics, rate: float, kp, kd, limits: Limits, torque_gain=1.0):
        self.arm, self.dyn, self.rate = robot, dynamics, rate
        self.dof = robot.dof
        # Motor feedback torque = torque_gain * actual torque. Models identified without the
        # calibration predict feedback units, so their feedforward is divided by it.
        self.torque_gain = robot.per_joint(torque_gain, "torque_gain")
        self.kp, self.kd = robot.clip_gains(kp, kd)
        self.limits = limits
        self.monitor = Monitor(limits)
        self.ticker = Ticker(rate)
        self.tmax = limits.torque
        self.q_cmd = None
        self.dq_cmd = np.zeros(self.dof)
        self.kp_cmd, self.kd_cmd = self.kp.copy(), self.kd.copy()
        self.tau_cmd = np.zeros(self.dof)
        self.stream = False          # --stream: machine-readable "@state {...}" lines for the GUI
        self._last_stream = 0.0
        self.phase = ""              # what the arm is doing, for the stop report
        self.last_state: State | None = None
        self.trace = deque(maxlen=int(8 * rate))      # recent feedback and commands (save_trace)

    def send(self, q, dq, tau, kp=None, kd=None) -> None:
        tau = np.clip(np.asarray(tau, dtype=float) / self.torque_gain, -self.tmax, self.tmax)
        kp, kd = self.arm.clip_gains(self.kp if kp is None else kp, self.kd if kd is None else kd)
        self.arm.command(q, dq, kp, kd, tau)
        self.q_cmd = np.asarray(q, dtype=float)
        self.dq_cmd = np.asarray(dq, dtype=float)
        self.kp_cmd, self.kd_cmd = kp.copy(), kd.copy()
        self.tau_cmd = tau

    def commanded_torque(self, state: State) -> np.ndarray:
        """Total MIT command at this feedback pose, including the last sent feedforward."""
        return (self.tau_cmd + self.kp_cmd * (self.q_cmd - state.q)
                + self.kd_cmd * (self.dq_cmd - state.dq))

    def state(self) -> State | None:
        state = self.arm.read()
        if state is not None:
            self.last_state = state
            if self.q_cmd is not None:
                self.trace.append((state.stamp, state.q, self.q_cmd, state.tau, self.tau_cmd))
        if self.stream and state is not None and state.stamp - self._last_stream >= 0.05:
            self._last_stream = state.stamp
            print("@state " + json.dumps({
                "q": np.round(state.q, 5).tolist(), "dq": np.round(state.dq, 4).tolist(),
                "tau": np.round(state.tau, 4).tolist(), "tau_cmd": np.round(self.tau_cmd, 4).tolist(),
                "q_cmd": None if self.q_cmd is None else np.round(self.q_cmd, 5).tolist(),
                "temp": np.round(np.nan_to_num(state.temperature), 1).tolist(),
            }), flush=True)
        return state

    def wait_state(self, timeout: float = 1.0) -> State:
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            state = self.arm.read()
            if state is not None:
                return state
            time.sleep(0.005)
        raise SafetyStop("读取电机状态超时")

    def engage(self, ramp: float = 0.5) -> np.ndarray:
        """Enable, latch the current pose, ramp stiffness up with gravity feedforward."""
        self.arm.enable()
        zero = np.zeros(self.dof)
        end = time.perf_counter() + 0.3
        state = None
        while time.perf_counter() < end and state is None:   # damping-only commands make the motors reply
            self.arm.command(zero, zero, zero, np.minimum(self.kd, 1.0), zero)
            time.sleep(0.01)
            state = self.arm.read()
        if state is None:
            raise SafetyStop("使能后收不到电机反馈")
        q_hold = state.q.copy()
        steps = max(1, int(ramp * self.rate))
        self.ticker = Ticker(self.rate)
        for k in range(1, steps + 1):
            alpha = k / steps
            self.send(q_hold, zero, self.dyn.gravity(q_hold)[0], kp=alpha * self.kp, kd=self.kd)
            self.ticker.wait()
        return q_hold

    def hold(self, q: np.ndarray, duration: float, check=True, record=None) -> None:
        tau = self.dyn.gravity(q)[0]
        zero = np.zeros(self.dof)
        for _ in range(max(1, int(duration * self.rate))):
            state = self.state()
            if check:
                self.monitor.check(state, q)
            if record is not None and state is not None:
                record(state)
            self.send(q, zero, tau)
            self.ticker.wait()

    def move_to(self, target: np.ndarray, speed: float, check=True, start: np.ndarray | None = None) -> None:
        """Quintic joint move with peak joint speed ``speed`` rad/s (moves shorter than 0.5 s are slower)."""
        q0 = self.wait_state().q if start is None else np.asarray(start, dtype=float)
        target = np.asarray(target, dtype=float)
        duration = max(1.875 * float(np.max(np.abs(target - q0))) / speed, 0.5)
        steps = int(np.ceil(duration * self.rate))
        for k in range(1, steps + 1):
            q, dq = quintic(q0, target, duration, k / self.rate)
            state = self.state()
            if check:
                self.monitor.check(state, self.q_cmd if self.q_cmd is not None else q)
            self.send(q, dq, self.dyn.gravity(q)[0])     # slow moves: gravity feedforward is enough
            self.ticker.wait()

    def stop_report(self) -> list[str]:
        """Where the arm was and what it was told when it stopped."""
        state = self.last_state
        if state is None or self.q_cmd is None:
            return []
        return [f"[stop] 阶段：{self.phase or '—'}",
                f"[stop] 实测 q {np.round(np.degrees(state.q), 1).tolist()}°  "
                f"指令 q {np.round(np.degrees(self.q_cmd), 1).tolist()}°",
                f"[stop] 实测力矩 {np.round(state.tau, 2).tolist()}  "
                f"模型重力 {np.round(self.dyn.gravity(state.q)[0], 2).tolist()}  "
                f"前馈 {np.round(self.tau_cmd, 2).tolist()} N·m"]

    def save_trace(self, path: str | Path) -> Path | None:
        """The last seconds of feedback and commands (time, q, q_cmd [deg], tau, tau_cmd [N·m]) as CSV."""
        if not self.trace:
            return None
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        rows = np.array([[t, *np.degrees(q), *np.degrees(q_cmd), *tau, *tau_cmd]
                         for t, q, q_cmd, tau, tau_cmd in self.trace])
        rows[:, 0] -= rows[-1, 0]
        names = ["t"] + [f"{k}{i}" for k in ("q_deg", "q_cmd_deg", "tau_meas", "tau_cmd") for i in range(1, self.dof + 1)]
        np.savetxt(path, rows, delimiter=",", header=",".join(names), comments="", fmt="%.5g")
        return path

    def safe_stop(self, rest: np.ndarray, speed: float = 0.3, reason: str = "") -> None:
        """Hold where we are, go to ``rest`` slowly and disable. Ctrl+C here disables at once."""
        if reason:
            print(f"\n[stop] {reason}")
            for line in self.stop_report():
                print(line)
        try:
            state = self.wait_state(0.3)
            relaxed = Limits(self.limits.lower, self.limits.upper, self.limits.torque, max_tracking=np.radians(20.0),
                             torque_ratio=0.95, max_temperature=self.limits.max_temperature + 15,
                             check_window=False)
            self.monitor = Monitor(relaxed)
            self.hold(state.q, 0.5)
            print(f"[stop] 以 {speed:.2f} rad/s 回到休息位姿 {np.round(np.degrees(rest), 1).tolist()}°")
            self.move_to(rest, speed, start=state.q)
            self.hold(rest, 0.5)
        except KeyboardInterrupt:
            print("[stop] 再次 Ctrl+C：立即失能")
        except Exception as exc:  # noqa: BLE001
            print(f"[stop] 回休息位失败（{exc}），直接失能")
        finally:
            self.arm.disable()
            print("[stop] 电机已失能")


# --------------------------------------------------------------- arguments
def add_common_args(parser) -> None:
    """Shared options; the robot itself is ``--robot.type=... --robot.<field>=...`` (see ``ROBOT_HELP``)."""
    parser.epilog = (parser.epilog + "\n" if parser.epilog else "") + ROBOT_HELP
    parser.add_argument("--urdf", help="前馈/检查用的模型（默认用配置中的 URDF；可换成辨识后的 URDF）")
    parser.add_argument("--full-urdf", action="store_true",
                        help="--urdf 是第 5 步导出的完整 URDF：末端连杆以下的连杆（夹爪等，手指在 0 位）当作负载算进重力")
    parser.add_argument("--with-tool", action="store_true",
                        help="碰撞检查包含装在末端的工具（夹爪等）：取 URDF 中末端连杆以下的部件，或 --tool-length")
    parser.add_argument("--payload-cad", action="store_true",
                        help="前馈加上机器人 URDF 中末端连杆以下部件（CAD 负载，手指在 0 位）的重力：装着负载、还没有负载模型时用")
    parser.add_argument("--tool-length", type=float, default=0.0, help="URDF 里没有工具时：工具长度 [m]（沿末端 z 轴的圆柱）")
    parser.add_argument("--tool-radius", type=float, default=0.03, help="工具半径 [m]")
    parser.add_argument("--rate", type=float, help="指令频率 [Hz]（默认取配置）")
    parser.add_argument("--kp", type=float, nargs="+", help="kp（1 或每轴一个；默认取配置）")
    parser.add_argument("--kd", type=float, nargs="+", help="kd（1 或每轴一个；默认取配置）")
    parser.add_argument("--rest", type=float, nargs="+", help="结束时回到的位姿 [deg]（默认取配置）")
    parser.add_argument("--move-speed", type=float, help="点到点移动的最大速度 [rad/s]（默认取配置）")
    parser.add_argument("--max-track-err", type=float, default=8.0, help="跟踪误差上限 [deg]")
    parser.add_argument("--torque-ratio", type=float, default=0.85, help="实测力矩超过力矩上限（电机 MIT 量程峰值）的这个比例时安全停止")
    parser.add_argument("--max-temp", type=float, default=100.0, help="温度上限 [°C]")
    parser.add_argument("--table-z", type=float, help="桌面高度 [m]（URDF 基坐标系 z；默认取配置）")
    parser.add_argument("--clearance", type=float, default=0.01, help="碰撞检查安全距离 [m]")
    parser.add_argument("--execute", action="store_true", help="真的运动机械臂；不加只做检查和预览")
    parser.add_argument("--torque-gain", type=float, nargs="+",
                        help="反馈力矩/实际力矩（1 或每轴一个）。控制用未做力矩标定的模型时才需要；默认 1")
    parser.add_argument("--stream", action="store_true", help="以 20 Hz 输出 '@state {json}' 行（供 GUI 实时显示）")


def open_robot(args) -> tuple[Robot, Dynamics]:
    """Robot from ``--robot.type``/``--robot.<field>`` and the feedforward model.

    Command-line values override the config; the resolved values are written back to ``args``.
    """
    robot = cli_robot()
    cfg = robot.config
    args.rate = args.rate or cfg.rate
    args.kp = args.kp if args.kp is not None else cfg.kp
    args.kd = args.kd if args.kd is not None else cfg.kd
    args.move_speed = args.move_speed or cfg.move_speed
    args.table_z = cfg.table_z if args.table_z is None else args.table_z
    args.rest = robot.rest() if args.rest is None else np.radians(robot.per_joint(args.rest, "--rest"))
    args.torque_gain = [1.0] if args.torque_gain is None else args.torque_gain
    robot.config.rate = args.rate
    dynamics = Dynamics.for_robot(robot, args.urdf, full=args.full_urdf)
    if dynamics.payload_mass:
        print(f"完整 URDF：{robot.config.tip_link} 以下的负载 {dynamics.payload_mass:.3f} kg 算进重力")
    if args.payload_cad:
        from ..payload import cad_payload

        body = cad_payload(robot.config.urdf_path, robot.config.tip_link)
        if body is None:
            raise SystemExit(f"{robot.config.urdf_path.name} 里 {robot.config.tip_link} 下面没有带质量的负载，不能用 --payload-cad")
        dynamics.add_payload(body)
        print(f"前馈加上 CAD 负载 {body[0]:.3f} kg（{robot.config.urdf_path.name} 中 {robot.config.tip_link} 以下）")
    if args.with_tool or args.tool_length > 0:
        dynamics.with_tool(robot, args.tool_length, args.tool_radius)
    return robot, dynamics


def make_controller(robot: Robot, dynamics: Dynamics, args, check_window: bool = True,
                    controller_type=Controller) -> Controller:
    lower, upper = robot.safety_window()
    limits = Limits(lower, upper, robot.torque_limits(), max_tracking=np.radians(args.max_track_err),
                    torque_ratio=args.torque_ratio, max_temperature=args.max_temp, check_window=check_window)
    ctrl = controller_type(robot, dynamics, args.rate, args.kp, args.kd,
                           limits, args.torque_gain)
    ctrl.stream = args.stream
    return ctrl
