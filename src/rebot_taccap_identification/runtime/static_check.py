"""Static torque check: measured holding torque vs model gravity at many poses.

Each pose is approached once from below and once from above; the mean of the
two holds approximately cancels static friction, half their difference is a lower
bound of it. Per joint, ``tau_meas = k * g_model + b`` is fitted (k ≈ 1: sign and
scale agree with the URDF; k ≈ -1: direction flipped; a large b: zero or torque
offset), and the torque gain (feedback / commanded) is estimated. The CSV doubles as static data for
identification (``rebot_taccap_identification identify --static``).

Poses come from ``--random N`` (spread over the safety window, collision-free,
ordered as a tour) or ``--poses file.json``. Without --execute nothing moves.

  rebot_taccap_identification static --robot.type=rebot_b601_rs --random 50
  rebot_taccap_identification static --robot.type=rebot_b601_rs --random 50 --execute
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from ..results import Results
from ..robots import Robot
from .common import (
    Controller, Dynamics, SafetyStop, add_common_args, check_clearance, describe_clearance, make_controller,
    open_robot,
)


def structural_reference(robot: Robot) -> np.ndarray:
    """Centre of the safety window. Link pairs already closer than the clearance there
    (e.g. adjacent wrist housings) are close by design and only flagged if they are also
    at the start of a path."""
    lower, upper = robot.safety_window(margin_deg=3.0)
    return ((lower + upper) / 2)[None]


def pose_space(robot: Robot, dynamics: Dynamics, samples: int = 300, seed: int = 0,
               relative_threshold: float = 1e-9) -> tuple[np.ndarray, np.ndarray]:
    """Gravity sensitivity of each joint and its coverage weight (0: joint held fixed), derived from the model.

    The sensitivity of joint j is the RMS norm of d g(q) / d q_j over random poses
    [N·m/rad]. Joints whose rotation never changes any gravity torque (e.g. a vertical
    base axis) are held fixed; all others, the light wrist joints included (they turn
    a payload against gravity), are sampled, weighted by sensitivity (clipped to 0.3..1).
    """
    lower, upper = robot.safety_window(margin_deg=3.0)
    rng = np.random.default_rng(seed)
    q = rng.uniform(lower, upper, size=(samples, robot.dof))
    g0, step = dynamics.gravity(q), 1e-3
    sensitivity = np.zeros(robot.dof)
    for j in range(robot.dof):
        shifted = q.copy()
        shifted[:, j] += step
        sensitivity[j] = np.sqrt(np.mean(np.sum(((dynamics.gravity(shifted) - g0) / step) ** 2, axis=1)))
    sampled = sensitivity > relative_threshold * sensitivity.max()
    weights = np.where(sampled, np.clip(sensitivity / sensitivity.max(), 0.3, 1.0), 0.0)
    return sensitivity, weights


def default_sample(weights) -> list[bool]:
    """Joints the static check samples by default: those that change gravity (``pose_space``),
    plus J1 (the base axis), which is sampled even when it changes no gravity torque."""
    return [bool(w > 0) or j == 0 for j, w in enumerate(weights)]


def payload_sample(dof: int) -> list[bool]:
    """Joints sampled for payload data: all of them. The payload turns with every wrist joint, and
    the base joint covers the directions the payload faces (and its cable pulls)."""
    return [True] * dof


def parse_focus(specs, dof: int) -> list[tuple[int, float, float, float]]:
    """``"J:lo:hi:fraction"`` (joint 1-based, degrees) -> (index, lo_rad, hi_rad, fraction)."""
    out = []
    for spec in specs or []:
        joint, lo, hi, fraction = spec.split(":")
        index = int(joint.lstrip("Jj")) - 1
        if not 0 <= index < dof or float(hi) <= float(lo) or not 0 < float(fraction) <= 1:
            raise SystemExit(f"--focus {spec}: 需要 关节:下限:上限:比例，如 2:3:32:0.3")
        out.append((index, np.radians(float(lo)), np.radians(float(hi)), float(fraction)))
    return out


def generate_poses(robot: Robot, dynamics: Dynamics, n: int, seed: int = 0, rest: np.ndarray | None = None,
                   table_z: float = 0.0, clearance: float = 0.01, focus=(), sample=None,
                   delta: float = np.radians(5.0)) -> np.ndarray:
    """``n`` collision-free static poses covering the safety window, ordered as a tour from ``rest``.

    Joints come from ``default_sample`` (``sample``, one bool per joint, overrides it),
    weights from ``pose_space`` (at least 0.3 for a sampled joint). Farthest-point sampling in the weighted joint space spreads the
    poses. Each ``focus`` entry ``(joint, lo, hi, fraction)`` places that fraction of
    the poses with the joint in [lo, hi] (e.g. where the arm usually rests, or where
    a model was inaccurate). A candidate is dropped when it, or one of its two approach
    poses (``delta`` below and above, see ``measure``), collides. The greedy tour keeps
    moves short and only links poses whose straight joint path is collision-free, with
    the same test as ``plan``, so no generated pose is skipped at run time.
    """
    rng = np.random.default_rng(seed)
    rest = robot.rest() if rest is None else np.asarray(rest, dtype=float)
    candidates = int(np.clip(25 * n, 600, 3000))
    lower, upper = robot.safety_window(margin_deg=3.0)
    sensitivity, weights = pose_space(robot, dynamics)
    sample = np.asarray(default_sample(weights) if sample is None else sample, dtype=bool)
    weights = np.where(sample, np.clip(sensitivity / max(sensitivity.max(), 1e-12), 0.3, 1.0), 0.0)
    sampled = weights > 0
    if not sampled.any():
        raise ValueError("至少要有一个关节采样")
    fixed = np.clip(rest, lower, upper)

    focus = [(j, max(lo, lower[j]), min(hi, upper[j]), f) for j, lo, hi, f in focus if sampled[j]]
    focus = [spec for spec in focus if spec[2] > spec[1]]
    counts = [int(round(f * n)) for *_, f in focus]
    if sum(counts) > n:                                  # fractions add up to more than 100 %
        counts = [int(c * n / sum(counts)) for c in counts]
    n_base = n - sum(counts)

    def draw(count: int, joint: int | None = None, lo: float = 0.0, hi: float = 0.0) -> np.ndarray:
        q = np.tile(fixed, (count, 1))
        q[:, sampled] = rng.uniform(lower[sampled], upper[sampled], size=(count, int(sampled.sum())))
        if joint is not None:
            q[:, joint] = rng.uniform(lo, hi, size=count)
        return q

    q = np.vstack([draw(candidates)] + [draw(max(candidates // 2, 20 * c), j, lo, hi)
                                        for (j, lo, hi, _), c in zip(focus, counts) if c])
    reference = structural_reference(robot)
    window = robot.safety_window()                       # what measure() uses for the approach
    offsets = np.array([approach_offsets(p, delta, *window) for p in q])
    stacked = np.vstack([q, q - offsets, q + offsets])
    report = check_clearance(dynamics, stacked, np.arange(float(len(stacked))), table_z, clearance, reference_q=reference)
    if report is not None:
        bad = {int(round(event["time"])) % len(q) for event in report["events"]}
        q = q[[i for i in range(len(q)) if i not in bad]]
    span = np.where(sampled, (upper - lower) / np.maximum(weights, 1e-9), np.inf)
    coords = q / span
    regions = [(q[:, j] >= lo) & (q[:, j] <= hi) for j, lo, hi, _ in focus]

    def farthest(count: int, allowed: np.ndarray, chosen: list[int], distance: np.ndarray) -> np.ndarray:
        for _ in range(count):
            masked = np.where(allowed, distance, -1.0)
            if masked.max() < 0:
                break
            nxt = int(np.argmax(masked))
            chosen.append(nxt)
            distance = np.minimum(distance, np.linalg.norm(coords - coords[nxt], axis=1))
        return distance

    def select(scale: int) -> list[int]:
        chosen = [int(np.argmin(np.abs(q - rest).max(axis=1)))]
        distance = np.linalg.norm(coords - coords[chosen[0]], axis=1)
        distance = farthest(scale * n_base - 1, np.ones(len(q), bool), chosen, distance)
        for region, count in zip(regions, counts):
            distance = farthest(scale * count, region, chosen, distance)
        return chosen

    main = select(1)
    # Spares (in the same proportions) replace poses the tour cannot reach.
    extra = [i for i in select(3) if i not in set(main)]
    return plan_tour(dynamics, [q[i] for i in main], [q[i] for i in extra], n, rest, table_z, clearance,
                     reference)


def plan_tour(dynamics: Dynamics, pool: list, spare: list, n: int, rest: np.ndarray, table_z: float,
              clearance: float, reference: np.ndarray) -> np.ndarray:
    """Greedy nearest-neighbour tour from ``rest`` through ``pool`` along collision-free straight
    joint paths; a pose that cannot be reached is replaced by the next ``spare``."""
    pool, spare = list(pool), list(spare)
    dof = len(rest)
    tour, current = [], rest
    while pool and len(tour) < n:
        pool.sort(key=lambda p: float(np.abs(p - current).max()))
        for k, candidate in enumerate(pool[:8]):
            path = np.linspace(current, candidate, 60)
            ok, _ = describe_clearance(check_clearance(dynamics, path, np.arange(60.0), table_z, clearance,
                                                       reference_q=[current, *reference]))
            if ok:
                tour.append(candidate)
                current = pool.pop(k)
                break
        else:
            pool.pop(0)                               # unreachable from here: replace it by a spare
            if spare:
                pool.append(spare.pop(0))
    return np.array(tour).reshape(-1, dof)


def load_pose_file(path: str | Path, dof: int) -> np.ndarray:
    raw = json.loads(Path(path).read_text())
    poses = np.asarray(raw["poses_deg"] if isinstance(raw, dict) else raw, dtype=float)
    if poses.ndim != 2 or poses.shape[1] != dof:
        raise SystemExit(f"{path}: 需要 N×{dof} 的位姿列表（度）")
    return np.radians(poses)


def save_pose_file(path: str | Path, poses: np.ndarray, **meta) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps({**meta, "poses_deg": np.round(np.degrees(poses), 2).tolist()}, indent=1))


def plan(robot: Robot, dynamics: Dynamics, poses: np.ndarray, rest: np.ndarray, args) -> tuple[np.ndarray, list[str]]:
    """Keep poses inside the window whose path (from rest, through the previous pose, and the two
    approach moves of ``measure``) is collision-free."""
    # Only 2 deg of the window are reserved for the approach; measure() shrinks the
    # approach offset of joints close to a limit (two-sided approach still cancels friction).
    lower, upper = robot.safety_window(margin_deg=2.0)
    kept, notes, previous = [], [], rest
    reference = structural_reference(robot)
    for pose in poses:
        label = np.round(np.degrees(pose), 0).astype(int).tolist()
        if np.any(pose < lower) or np.any(pose > upper):
            notes.append(f"跳过 {label}：超出安全窗口")
            continue
        offsets = approach_offsets(pose, np.radians(args.delta), *robot.safety_window())
        stops = [previous, pose - offsets, pose, pose + offsets, pose]
        path = np.vstack([np.linspace(a, b, 30) for a, b in zip(stops[:-1], stops[1:])])
        ok, text = describe_clearance(check_clearance(dynamics, path, np.arange(float(len(path))), args.table_z,
                                                      args.clearance, reference_q=[previous, *reference]))
        if not ok:
            notes.append(f"跳过 {label}：从上一位姿过去会碰撞（{text}）")
            continue
        kept.append(pose)
        previous = pose
    return np.array(kept).reshape(-1, robot.dof), notes


def approach_offsets(pose: np.ndarray, delta: float, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Per-joint approach offset: ``delta``, reduced near a limit, at least 1 deg."""
    room = np.minimum(pose - lower, upper - pose) - np.radians(1.0)
    return np.clip(room, np.radians(1.0), delta)


def measure(ctrl: Controller, pose: np.ndarray, delta: float, args) -> dict:
    samples = {}
    offsets = approach_offsets(pose, delta, *ctrl.arm.safety_window())
    for side, sign in (("below", -1.0), ("above", 1.0)):
        ctrl.move_to(pose + sign * offsets, args.move_speed)
        ctrl.move_to(pose, args.move_speed / 2)
        ctrl.hold(pose, args.settle)
        rows = []
        ctrl.hold(pose, args.hold, record=rows.append)
        # What the motor was actually told: feedforward (after the gain) + kp*error + kd*(0 - dq).
        command = [ctrl.dyn.gravity(pose)[0] / ctrl.torque_gain + ctrl.kp * (pose - r.q) - ctrl.kd * r.dq for r in rows]
        samples[side] = (np.mean([r.q for r in rows], axis=0), np.mean([r.tau for r in rows], axis=0),
                         np.mean(command, axis=0))
    q = 0.5 * (samples["below"][0] + samples["above"][0])
    return {
        "command": 0.5 * (samples["below"][2] + samples["above"][2]),
        "q": q,
        "tau": 0.5 * (samples["below"][1] + samples["above"][1]),
        "friction": 0.5 * np.abs(samples["above"][1] - samples["below"][1]),
        "gravity": ctrl.dyn.gravity(q)[0],
    }


def verdict(g: np.ndarray, tau: np.ndarray) -> tuple[str, float, float, float, float]:
    """Fit tau = k*g + b; judge k only when its 2-sigma interval is narrow enough."""
    span = float(np.ptp(g))
    if span < 0.3 or len(g) < 4:
        return f"重力变化仅 {span:.2f} N·m，无法判断", np.nan, np.nan, np.nan, np.nan
    A = np.column_stack([g, np.ones_like(g)])
    (k, b), *_ = np.linalg.lstsq(A, tau, rcond=None)
    residual = tau - A @ [k, b]
    rms = float(np.sqrt(np.mean(residual ** 2)))
    k2 = 2.0 * float(np.sqrt(residual @ residual / (len(g) - 2) / np.sum((g - g.mean()) ** 2)))
    if k2 > 0.25:
        return f"? k 的不确定度 ±{k2:.2f} 太大（重力变化 {span:.2f} N·m 相对测量噪声太小），无法判断", k, b, rms, k2
    if abs(k - 1.0) <= max(0.1, k2):
        text = "✓ 符号/比例一致" + ("" if abs(b) < 0.3 else f"，但有偏置 {b:+.2f} N·m（零位或力矩零偏）")
    elif abs(k + 1.0) <= max(0.2, k2):
        text = "✗ 符号相反：关节方向与 URDF axis 不一致"
    elif 0.6 <= k <= 1.5:
        text = f"△ 方向正确，模型重力偏{'小' if k > 1 else '大'} {abs(k - 1) * 100:.0f}%（URDF 质量/质心与实测不符）"
    else:
        text = f"✗ 比例 {k:.2f} 不对：检查力矩单位/减速比、零位或 URDF 质量"
    return text, k, b, rms, k2


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(prog="rebot_taccap_identification static", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(parser)
    parser.add_argument("--poses", help="位姿文件（JSON：{\"poses_deg\": [[J1..Jn], ...]}）")
    parser.add_argument("--random", type=int, default=0, metavar="N", help="随机生成 N 个覆盖安全窗口的无碰撞位姿")
    parser.add_argument("--seed", type=int, default=0, help="--random 的随机种子")
    parser.add_argument("--sample", help="--random 中采样的关节（如 2,3,4），默认 J1 加上由重力灵敏度自动决定的关节，其余固定")
    parser.add_argument("--focus", action="append", metavar="J:下限:上限:比例",
                        help="--random 的重点区（可重复）：该比例的位姿让关节 J 落在 [下限, 上限]°，如 --focus 2:3:32:0.3")
    parser.add_argument("--delta", type=float, default=5.0, help="上下接近的偏移 [deg]")
    parser.add_argument("--settle", type=float, default=1.0, help="到位后等待 [s]")
    parser.add_argument("--hold", type=float, default=1.5, help="取平均的保持时间 [s]")
    parser.add_argument("--out", help="输出 CSV（默认 <结果目录>/static/check.csv）")
    parser.add_argument("--save-calibration", action="store_true",
                        help="把测得的力矩增益写入该机器人的标定文件（~/.cache/rebot_taccap_identification/calibration/robots/<类型>/<id>.json）")
    args = parser.parse_args(argv)

    robot, dynamics = open_robot(args)
    dof = robot.dof
    rest, delta = args.rest, np.radians(args.delta)
    if args.poses:
        requested = load_pose_file(args.poses, dof)
    elif args.random > 0:
        if args.sample:
            sample = [j + 1 in {int(v) for v in args.sample.split(",")} for j in range(dof)]
        elif dynamics.collision is not None or dynamics.payload_mass > 0:
            sample = payload_sample(dof)              # as the workbench does for payload data
        else:
            sample = None
        requested = generate_poses(robot, dynamics, args.random, args.seed, rest, args.table_z, args.clearance,
                                   focus=parse_focus(args.focus, dof), sample=sample, delta=np.radians(args.delta))
    else:
        raise SystemExit("需要 --random N 或 --poses 文件")
    poses, notes = plan(robot, dynamics, requested, rest, args)
    print(f"静态力矩核对（{robot}）：{len(poses)} 个位姿，频率 {args.rate:.0f} Hz，每个位姿约 "
          f"{2 * (args.settle + args.hold) + 4:.0f} s")
    for note in notes:
        print("  " + note)
    for pose in poses:
        g = dynamics.gravity(pose)[0]
        print(f"  {np.round(np.degrees(pose)).astype(int).tolist()}  模型重力 {np.round(g, 2).tolist()} N·m")
    if len(poses) < 3:
        raise SystemExit("可用位姿太少")
    if not args.execute:
        print("\n[预览] 加 --execute 才会运动。运行前确认：通信正常、机械臂周围无人无障碍、急停可用。")
        return

    out = Path(args.out) if args.out else Results.for_robot(robot).static("check")
    ctrl = make_controller(robot, dynamics, args, check_window=False)
    results = []
    try:
        robot.connect()
        q_now = ctrl.engage()
        print(f"[engage] 当前位姿 {np.round(np.degrees(q_now), 1).tolist()}°")
        for index, pose in enumerate(poses, start=1):
            print(f"[{index}/{len(poses)}] {np.round(np.degrees(pose)).astype(int).tolist()}°", flush=True)
            results.append(measure(ctrl, pose, delta, args))
        ctrl.safe_stop(rest, args.move_speed)
    except (SafetyStop, KeyboardInterrupt) as exc:
        if robot.enabled:
            ctrl.safe_stop(rest, args.move_speed, reason=str(exc) or "Ctrl+C")
        else:
            print(f"\n[error] {exc}")
            return
    finally:
        robot.disconnect()
    if not results:
        return

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([f"{k}{i}" for k in ("q_deg", "tau_meas", "tau_model", "static_friction", "tau_cmd")
                         for i in range(1, dof + 1)])
        for r in results:
            writer.writerow([*np.degrees(r["q"]), *r["tau"], *r["gravity"], *r["friction"], *r["command"]])
    g = np.array([r["gravity"] for r in results])
    tau = np.array([r["tau"] for r in results])
    friction = np.array([r["friction"] for r in results])
    print(f"\n{'位姿':>4} " + " ".join(f"{'J' + str(i + 1) + ' 实测/模型':>16}" for i in range(dof)))
    for n, (tm, gm) in enumerate(zip(tau, g), start=1):
        print(f"{n:>4} " + " ".join(f"{a:7.2f}/{b:<8.2f}" for a, b in zip(tm, gm)))
    print("\n关节  斜率 k (±2σ)     偏置 b   残差RMS  |实测-模型|RMS  静摩擦≈   结论")
    for j in range(dof):
        text, k, b, rms, k2 = verdict(g[:, j], tau[:, j])
        error = float(np.sqrt(np.mean((tau[:, j] - g[:, j]) ** 2)))
        print(f"J{j + 1}  {k:6.2f} ±{k2:<6.2f}  {b:+7.2f}  {rms:7.3f}  {error:12.3f}  {np.mean(friction[:, j]):7.2f}   {text}")
    print("\n力矩增益（反馈 = s·指令 + c，指令含 kp/kd 项）：")
    gains = []
    command = np.array([r["command"] for r in results])
    for j in range(dof):
        span = float(np.ptp(command[:, j]))
        if span < 1.0:
            gains.append(1.0)
            print(f"J{j + 1}  指令变化仅 {span:.2f} N·m，无法估计，取 1")
            continue
        A = np.column_stack([command[:, j], np.ones(len(command))])
        (s, c), *_ = np.linalg.lstsq(A, tau[:, j], rcond=None)
        res = tau[:, j] - A @ [s, c]
        s2 = 2.0 * float(np.sqrt(res @ res / max(len(res) - 2, 1) / np.sum((command[:, j] - command[:, j].mean()) ** 2)))
        gains.append(round(float(s), 3))
        print(f"J{j + 1}  s={s:.3f} ±{s2:.3f}  c={c:+.3f}  残差RMS={np.sqrt(np.mean(res ** 2)):.3f}")
    print(f"测得：torque_gain = [{', '.join(f'{g:g}' for g in gains)}]（用 --save-calibration 存入标定后，辨识自动使用）")
    if args.save_calibration:
        print(f"已写入力矩标定 {robot.save_torque_gain(gains)}")
    elif robot.is_calibrated:
        print(f"当前标定 {np.round(robot.torque_gain(), 3).tolist()}（{robot.calibration_fpath}）；加 --save-calibration 覆盖")
    print(f"\n已保存 {out}")


if __name__ == "__main__":
    main()
