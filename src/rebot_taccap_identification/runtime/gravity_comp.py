"""Gravity compensation on any arm (closed-loop test of an identified model).

  --mode free:  kp=0, kd=--comp-kd, tau = g(q). The arm floats and can be moved by hand.
  --mode lock:  holds q_target with kp/kd plus an integral term; q_target follows the
                arm while the link6 frame moves faster than --vel-threshold m/s or
                --w-threshold rad/s, so a push repositions it.

With --stream the feel can be tuned while running: lines ``@set kd=1.5`` (also
``lock_kp``, and ``scale2=1.04`` for joint 2's feedforward scale) on stdin take effect at once.

--ff-scale multiplies each joint's feedforward (gravity + offset), e.g. 1.04 on J2 when the
arm sinks where J2 carries a lot (a model a few percent light there, or a hot motor giving
less torque per command). It is a test knob: the logged run and its drift analysis use the
scaled feedforward, the model itself is unchanged.

In free mode the run is logged (<results>/gravity/) and analysed afterwards: joints that
moved on their own show where the model is wrong, and static poses there are suggested
(see ``rebot_taccap_identification.runtime.drift``).

The joint torque offsets identified together with the masses (``<model>_offset.yaml`` next to the URDF) are
part of the holding torque and are applied when the file exists (``--no-offset`` leaves
them out).

A virtual wall at the safety window, torque/temperature/feedback monitoring and a
smooth hand-over from position hold. On exit (--on-exit rest, default) the arm is
driven back to the rest pose if the straight path is collision-free, otherwise it is
held until Enter; --on-exit release removes the torque.

  rebot_taccap_identification gravity --urdf results/rebot_b601_rs/models/v1/rebot_b601_rs_identified_v1.urdf --execute
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

from ..export import load_offsets, offsets_path
from ..io import save_motion_csv
from ..results import Results
from ..resources import bundle_urdf, relative_reference, resource_digest
from ..robots import Robot
from .common import (
    Controller, Dynamics, Limits, Monitor, SafetyStop, Ticker, add_common_args, check_clearance,
    describe_clearance, make_controller, open_robot,
)


class Compensator:
    """tau = g(q) [+ offset] [+ virtual wall], in true torque units."""

    def __init__(self, robot: Robot, dynamics: Dynamics, args):
        self.dyn = dynamics
        self.lower, self.upper = robot.safety_window()
        self.wall = args.wall_kp
        path = Path(args.offset_file) if args.offset_file else offsets_path(dynamics.robot.source)
        self.offset_file = path if path.exists() and not args.no_offset else None
        self.offset = load_offsets(path, dynamics.robot.joint_names) if self.offset_file else np.zeros(robot.dof)
        self.scale = robot.per_joint(args.ff_scale, "--ff-scale")

    def __call__(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        tau_g = self.scale * (self.dyn.gravity(q)[0] + self.offset)
        tau = tau_g.copy()
        # Push back toward the window instead of letting a hand move the arm past it.
        tau += self.wall * (np.maximum(self.lower - q, 0.0) - np.maximum(q - self.upper, 0.0))
        return tau, tau_g


class Tuning:
    """Live settings from stdin (``@set name=value``); other lines answer prompts."""

    NAMES = ("kd", "lock_kp")           # and scale<joint> (1-based), e.g. scale2

    def __init__(self, enabled: bool):
        self.updates: queue.Queue = queue.Queue()
        self.lines: queue.Queue = queue.Queue()
        self.enabled = enabled
        if enabled:
            threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if line.startswith("@set "):
                for item in line[5:].split():
                    name, _, value = item.partition("=")
                    try:
                        if name in self.NAMES or (name.startswith("scale") and name[5:].isdigit()):
                            self.updates.put((name, float(value)))
                    except ValueError:
                        pass
            else:
                self.lines.put(line)

    def pending(self) -> list[tuple[str, float]]:
        out = []
        while not self.updates.empty():
            out.append(self.updates.get())
        return out

    def ask(self, prompt: str) -> None:
        """Like ``input(prompt)``, but compatible with the reader thread."""
        if not self.enabled:
            input(prompt)
            return
        print(prompt, end="", flush=True)
        self.lines.get()


def tip_velocity(dynamics: Dynamics, q: np.ndarray, dq: np.ndarray) -> tuple[float, float]:
    """Linear [m/s] and angular [rad/s] speed of the last link frame."""
    robot, h = dynamics.robot, 1e-4
    _, origins, _ = robot._joint_geometry_batch(np.vstack([q, q + h * dq]))
    omega = robot.angular_velocity(q, dq)
    return float(np.linalg.norm((origins[1, -1] - origins[0, -1]) / h)), float(np.linalg.norm(omega))


def hand_over(ctrl: Controller, comp: Compensator, kp_final: np.ndarray, kd: np.ndarray, duration: float) -> np.ndarray:
    """From position hold (with gravity) to the compensation gains."""
    state = ctrl.wait_state()
    q_hold = state.q.copy()
    steps = max(1, int(duration * ctrl.rate))
    for k in range(1, steps + 1):
        alpha = k / steps
        state = ctrl.state() or state
        tau, _ = comp(state.q)
        ctrl.send(q_hold, np.zeros(ctrl.dof), tau, kp=(1 - alpha) * ctrl.kp + alpha * kp_final,
                  kd=(1 - alpha) * ctrl.kd + alpha * kd)
        ctrl.ticker.wait()
    return q_hold


def leave(ctrl: Controller, dynamics: Dynamics, comp: Compensator, rest: np.ndarray, args, tuning: Tuning) -> None:
    """Stiffen at the current pose, then go to rest (collision-checked) or release."""
    print("\n[stop] 结束重力补偿")
    try:
        state = ctrl.wait_state(0.3)
        ctrl.monitor = Monitor(Limits(ctrl.limits.lower, ctrl.limits.upper, ctrl.limits.torque,
                                      max_tracking=np.radians(20.0), torque_ratio=0.95,
                                      max_temperature=args.max_temp + 15, check_window=False))
        zero = np.zeros(ctrl.dof)
        steps = int(0.5 * ctrl.rate)
        tau = comp.offset + dynamics.gravity(state.q)[0]
        for k in range(1, steps + 1):                       # stiffen without a jump
            ctrl.send(state.q, zero, tau, kp=(k / steps) * ctrl.kp)
            ctrl.ticker.wait()
        if args.on_exit == "release":
            print("[stop] --on-exit release：撤掉力矩（机械臂会落下，请扶住）")
            for _ in range(10):
                ctrl.arm.command(state.q, zero, zero, zero, zero)
                time.sleep(0.02)
            return
        # Both ends are known-safe poses (where the arm is now, and its usual rest pose).
        path = np.linspace(state.q, rest, 60)
        ok, text = describe_clearance(check_clearance(dynamics, path, np.arange(60.0), args.table_z,
                                                      args.clearance, reference_q=[state.q, rest]))
        if not ok:
            print(f"[stop] 直接回休息位可能碰撞（{text}）")
            tuning.ask("[stop] 机械臂保持在当前位置。请用手扶住，然后按回车松开…")
            return
        print(f"[stop] 以 {args.move_speed:.2f} rad/s 回到休息位姿")
        ctrl.ticker = Ticker(ctrl.rate)
        ctrl.move_to(rest, args.move_speed, start=state.q)
        ctrl.hold(rest, 0.5)
    except KeyboardInterrupt:
        print("[stop] 再次 Ctrl+C：立即失能")
    except Exception as exc:  # noqa: BLE001
        print(f"[stop] 退出流程出错（{exc}），直接失能")
    finally:
        ctrl.arm.disable()
        print("[stop] 电机已失能")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(prog="rebot_taccap_identification gravity", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(parser)
    parser.add_argument("--mode", choices=("free", "lock"), default="free")
    parser.add_argument("--comp-kd", type=float, help="阻尼 kd（free 和 lock 模式；默认取配置 free_kd）")
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数，0 表示直到 Ctrl+C")
    parser.add_argument("--offset-file", help="关节力矩偏置文件（默认取 URDF 旁边的 <模型>_offset.yaml）")
    parser.add_argument("--no-offset", action="store_true", help="不加辨识出的关节力矩偏置")
    parser.add_argument("--ff-scale", type=float, nargs="+", default=[1.0],
                        help="前馈倍率（1 个或每轴一个）：重力前馈乘以它，如 J2 伸直时下沉用 1 1.04 1 1 1 1")
    parser.add_argument("--wall-kp", type=float, default=20.0, help="安全窗口外的虚拟墙刚度 [N·m/rad]")
    parser.add_argument("--lock-kp", type=float, default=8.0, help="lock 模式刚度")
    parser.add_argument("--vel-threshold", type=float, default=0.04, help="lock：线速度阈值 [m/s]")
    parser.add_argument("--w-threshold", type=float, default=0.08, help="lock：角速度阈值 [rad/s]")
    parser.add_argument("--ki", type=float, default=1.0, help="lock：积分增益 [N·m/(rad·s)]")
    parser.add_argument("--integral-limit", type=float, default=0.5, help="lock：积分力矩上限 [N·m]")
    parser.add_argument("--no-analysis", action="store_true", help="free 模式结束后不做漂移分析")
    parser.add_argument("--on-exit", choices=("rest", "release"), default="rest",
                        help="rest：回休息位后失能；release：直接撤力矩（机械臂会落下）")
    args = parser.parse_args(argv)

    robot, dynamics = open_robot(args)
    dof = robot.dof
    comp = Compensator(robot, dynamics, args)
    args.ff_scale_used = comp.scale            # same array: live changes end up in the log's meta
    kd = robot.per_joint(args.comp_kd if args.comp_kd is not None else robot.config.free_kd, "--comp-kd")
    kp_final = robot.per_joint(args.lock_kp if args.mode == "lock" else 0.0, "--lock-kp")
    print(f"重力补偿（{robot}）：mode={args.mode}  模型={dynamics.robot.source}")
    print(f"  kp={kp_final.tolist()}  kd={kd.tolist()}  {args.rate:.0f} Hz")
    if args.torque_gain != [1.0]:
        print(f"  力矩增益（反馈/指令）：{robot.per_joint(args.torque_gain, '--torque-gain').tolist()}（仅用于未做力矩标定的模型）")
    if np.any(comp.scale != 1.0):
        print(f"  前馈倍率：{np.round(comp.scale, 3).tolist()}")
    print("  关节力矩偏置：" + (f"{comp.offset_file.name} {np.round(comp.offset, 3).tolist()} N·m"
                              if comp.offset_file else "无"))
    lower, upper = robot.safety_window()
    for label, q in (("休息位", args.rest), ("窗口中心", (lower + upper) / 2)):
        print(f"  {label} 重力力矩 [N·m]：{np.round(dynamics.gravity(q)[0], 2).tolist()}")
    if not args.execute:
        print("\n[预览] 加 --execute 才会使能。free 模式下机械臂靠模型托住，模型不准时会缓慢下沉或上漂，手要随时能扶住。")
        return

    rest = args.rest
    tuning = Tuning(args.stream)
    ctrl = make_controller(robot, dynamics, args, check_window=False)    # config kp/kd hold during engage/exit
    ctrl.monitor.limits.max_tracking = np.inf
    tmax = robot.torque_limits() * args.torque_ratio
    log: list = []
    saved = None
    try:
        robot.connect()
        q_now = ctrl.engage()
        print(f"[engage] 当前位姿 {np.round(np.degrees(q_now), 1).tolist()}°，1 s 内过渡到重力补偿")
        q_target = hand_over(ctrl, comp, kp_final, kd, 1.0)
        integral = np.zeros(dof)
        start = last_print = time.perf_counter()
        print("[run] 运行中，Ctrl+C 结束", flush=True)
        while args.duration <= 0 or time.perf_counter() - start < args.duration:
            state = ctrl.state()
            ctrl.monitor.check(state, state.q if state is not None else q_target)
            if state is None:
                ctrl.ticker.wait()
                continue
            for name, value in tuning.pending():
                if name == "kd":
                    kd = np.full(dof, max(value, 0.0))
                elif name == "lock_kp" and args.mode == "lock":
                    kp_final = np.full(dof, max(value, 0.0))
                elif name.startswith("scale") and 1 <= int(name[5:]) <= dof:
                    comp.scale[int(name[5:]) - 1] = float(np.clip(value, 0.5, 1.5))
                print(f"[set] {name} = {value:g}", flush=True)
            tau, tau_g = comp(state.q)
            log.append((state.stamp, state.q, state.dq, state.tau, tau, kd.copy()))
            status = ""
            if args.mode == "lock":
                v, w = tip_velocity(dynamics, state.q, state.dq)
                if v > args.vel_threshold or w > args.w_threshold:
                    q_target = state.q.copy()
                    integral *= 0.9
                    status = "UPDATE"
                else:
                    integral = np.clip(integral + args.ki * (q_target - state.q) / args.rate,
                                       -args.integral_limit, args.integral_limit)
                    status = "LOCKED"
                tau = tau + integral
                ctrl.send(q_target, np.zeros(dof), np.clip(tau, -tmax, tmax), kp=kp_final, kd=kd)
            else:
                ctrl.send(state.q, np.zeros(dof), np.clip(tau, -tmax, tmax), kp=kp_final, kd=kd)
            now = time.perf_counter()
            if now - last_print >= 1.0:
                last_print = now
                temperature = np.nanmax(state.temperature) if np.isfinite(state.temperature).any() else float("nan")
                print(f"[{now - start:6.1f}s] {status:6s} q={np.round(np.degrees(state.q), 1).tolist()}° "
                      f"tau_g={np.round(tau_g, 2).tolist()} 实测={np.round(state.tau, 2).tolist()} "
                      f"T={temperature:.0f}°C", flush=True)
            ctrl.ticker.wait()
        saved = keep_log(robot, dynamics, log, args)
        leave(ctrl, dynamics, comp, rest, args, tuning)
    except (SafetyStop, KeyboardInterrupt) as exc:
        # Save first (milliseconds): a further stop press during the way back must not lose the run.
        saved = saved or keep_log(robot, dynamics, log, args)
        if not robot.enabled:
            print(f"\n[error] {exc}")
            return
        if isinstance(exc, SafetyStop):
            print(f"\n[safety] {exc}")
        leave(ctrl, dynamics, comp, rest, args, tuning)
    finally:
        robot.disconnect()
        if ctrl.ticker.overruns:
            print(f"注意：{ctrl.ticker.overruns} 个周期超时（最差 {1000 * ctrl.ticker.worst:.1f} ms）")
    if saved is not None and not args.no_analysis:
        analyze_log(robot, dynamics, saved, args)


def keep_log(robot: Robot, dynamics: Dynamics, log: list, args):
    """Save a free-mode log (<results>/gravity/); returns what ``analyze_log`` needs, None if nothing to keep."""
    if args.mode != "free" or len(log) <= 2 * args.rate:
        return None
    # A payload model's run belongs to its package: drift poses are measured with the payload mounted.
    results = Results.for_robot(robot).package_of(dynamics.robot.source)
    name = time.strftime("%Y%m%d_%H%M%S") + "_" + dynamics.robot.source.stem.replace(robot.model_prefix() + "_", "")
    path = results.gravity_dir / f"{name}.csv"
    stamp, q, dq, tau, tau_cmd, kd = (np.array(c) for c in zip(*log))
    save_motion_csv(path, stamp - stamp[0], q=q, dq=dq, tau=tau, taucmd=tau_cmd, kd=kd)
    source = dynamics.robot.source
    if not source.resolve().is_relative_to(results.root.resolve()):
        snapshot = results.gravity_dir / "models" / resource_digest(source) / source.name
        if not snapshot.is_file():
            bundle_urdf(source, snapshot)
        source = snapshot
    meta = {"urdf": relative_reference(source, path.parent), "rate": args.rate, "full_urdf": bool(args.full_urdf),
            "table_z": args.table_z, "ff_scale": np.asarray(args.ff_scale_used, dtype=float).tolist()}
    path.with_suffix(".meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    print(f"\n已保存重力补偿记录 {path}", flush=True)
    return path, results.poses(f"drift_{name}"), stamp, q, dq, kd


def analyze_log(robot: Robot, dynamics: Dynamics, saved, args) -> None:
    """Look for drift in a saved log (where the model needs more static data)."""
    from .drift import analyze

    path, out, stamp, q, dq, kd = saved
    result = analyze(robot, dynamics, {"time": stamp - stamp[0], "q": q, "dq": dq, "kd": kd},
                     rest=args.rest, table_z=args.table_z, out=out)
    if args.stream:
        print("@drift " + json.dumps({"log": str(path), "poses": result["poses"], "n_poses": result["n_poses"],
                                      "summary": result["summary"], "n_drifts": len(result["drifts"])},
                                     ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
