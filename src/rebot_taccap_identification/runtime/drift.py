"""Where does a gravity model let the arm drift? Analysis of a free-mode gravity-compensation log.

In free mode (kp = 0) a joint that moves on its own, slowly and in one direction, is
pushed by the model error: the commanded gravity differs from the true one by more
than static friction can hold. Each such segment gives a pose and the sign of the
error; its size is estimated as ``kd·speed`` (the drift settles at the speed where
damping balances the error; friction is not included).

Hand motion cannot be told apart from drift by the motor signals alone, so segments
are only taken where the motion is slow (drift speeds) and sustained, and then screened
(``screen``): a fast peak is a hand push, and so is any motion while another joint is
moved fast (the push reaches the joint through the arm's dynamic coupling); a joint the arm's gravity never loads (e.g. a
vertical base joint) cannot drift from a gravity error; and at one pose the error has
one sign, so a joint going both ways from (nearly) the same pose was moved by hand. The result is a set of static poses around the
drift locations, to be measured in a static check and added to the static data of the
next identification.

  rebot_taccap_identification drift --robot.type=rebot_b601_rs --log results/<robot>/gravity/<log>.csv
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from ..io import named_columns, read_named_csv
from ..resources import resolve_reference
from ..robots import Robot
from .common import Dynamics, check_clearance

MIN_SPEED = np.radians(0.6)       # rad/s: below this the joint is at rest
MAX_SPEED = np.radians(20.0)      # rad/s: mean speed above this is taken as hand motion
MIN_DURATION = 0.8                # s
MIN_TRAVEL = np.radians(3.0)      # rad
MAX_PEAK = np.radians(30.0)       # rad/s: a peak above this is a hand push
CONFLICT = np.radians(12.0)       # rad: opposite drifts of one joint closer than this contradict each other
MIN_GRAVITY = 0.1                 # N·m: joints whose gravity torque never exceeds this are not analysed


@dataclass
class Drift:
    joint: int                    # 0-based
    start: float                  # s
    duration: float               # s
    travel_deg: float             # signed
    pose_deg: list[float]         # where it started drifting
    path_deg: list[list[float]]   # up to 6 poses along the drift, at least ~8° apart (the error is there too)
    error: float                  # N·m, estimated model error (commanded - needed), sign = drift direction
    peak_speed: float = 0.0       # rad/s
    other_peak: float = 0.0       # rad/s, fastest other joint meanwhile

    def describe(self) -> str:
        pose = ", ".join(f"{v:.0f}" for v in self.pose_deg)
        return (f"J{self.joint + 1} 在 t={self.start:.1f}s 自己{'正' if self.travel_deg > 0 else '负'}向转了 "
                f"{abs(self.travel_deg):.0f}°（{self.duration:.1f} s），起点 [{pose}]°，模型误差约 {self.error:+.2f} N·m")


def _smooth(x: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return x
    kernel = np.ones(window) / window
    return np.vstack([np.convolve(x[:, j], kernel, mode="same") for j in range(x.shape[1])]).T


def find_drifts(t: np.ndarray, q: np.ndarray, dq: np.ndarray, kd: np.ndarray) -> list[Drift]:
    """Slow, sustained, one-directional motion of single joints (see the module doc)."""
    t, q, dq = (np.asarray(x, dtype=float) for x in (t, q, dq))
    kd = np.broadcast_to(np.asarray(kd, dtype=float), q.shape)
    dof = q.shape[1]
    rate = 1.0 / max(float(np.median(np.diff(t))), 1e-3)
    v = _smooth(dq, int(round(0.2 * rate)))
    drifts = []
    for j in range(dof):
        moving = np.where(np.abs(v[:, j]) > MIN_SPEED, np.sign(v[:, j]), 0.0)
        edges = np.flatnonzero(np.diff(np.concatenate([[0.0], moving, [0.0]])))
        for a, b in zip(edges[:-1], edges[1:]):
            if moving[a] == 0 or b - a < 2:
                continue
            duration = t[b - 1] - t[a]
            travel = q[b - 1, j] - q[a, j]
            speed = abs(travel) / max(duration, 1e-6)
            if duration < MIN_DURATION or abs(travel) < MIN_TRAVEL or speed > MAX_SPEED:
                continue
            # The drift direction is that of the excess torque; damping balances it.
            error = float(np.sign(travel) * np.mean(kd[a:b, j]) * speed)
            steps = int(np.clip(abs(travel) // np.radians(8.0), 0, 5)) + 1
            along = [a + int(round(k * (b - 1 - a) / max(steps - 1, 1))) for k in range(steps)]
            drifts.append(Drift(j, float(t[a]), float(duration), float(np.degrees(travel)),
                                np.round(np.degrees(q[a]), 1).tolist(),
                                [np.round(np.degrees(q[i]), 1).tolist() for i in along], error,
                                float(np.abs(v[a:b, j]).max()),
                                # skip the first 0.3 s: the hand that just let go of another joint
                                float(np.abs(np.delete(v[min(a + int(0.3 * rate), b - 1):b], j, axis=1))
                                      .max(initial=0.0))))
    return sorted(drifts, key=lambda d: d.start)


def screen(drifts: list[Drift], dynamics: Dynamics | None = None, lower=None, upper=None,
           seed: int = 0) -> tuple[list[Drift], dict[str, int]]:
    """Drop the drifts that cannot be gravity-model errors (see the module doc); returns the kept
    drifts and how many were dropped for each reason."""
    dropped = {"hand": 0, "no_gravity": 0, "conflict": 0}
    kept = []
    for d in drifts:
        if max(d.peak_speed, d.other_peak) > MAX_PEAK:
            dropped["hand"] += 1
        else:
            kept.append(d)
    if dynamics is not None and lower is not None and kept:
        q = np.random.default_rng(seed).uniform(lower, upper, size=(300, len(lower)))
        loaded = np.abs(dynamics.gravity(q)).max(axis=0) > MIN_GRAVITY
        dropped["no_gravity"] = sum(not loaded[d.joint] for d in kept)
        kept = [d for d in kept if loaded[d.joint]]
    conflicting = set()
    for i, a in enumerate(kept):
        for k in range(i + 1, len(kept)):
            b = kept[k]
            if a.joint == b.joint and np.sign(a.travel_deg) != np.sign(b.travel_deg) and \
                    np.abs(np.radians(np.subtract(a.pose_deg, b.pose_deg))).max() < CONFLICT:
                conflicting |= {i, k}
    dropped["conflict"] = len(conflicting)
    return [d for i, d in enumerate(kept) if i not in conflicting], dropped


def suggest_poses(robot: Robot, dynamics: Dynamics, drifts: list[Drift], rest: np.ndarray, table_z: float,
                  max_poses: int = 30, spread_deg: float = 8.0, clearance: float = 0.01, seed: int = 0) -> np.ndarray:
    """Static poses along the drift paths and around them, collision-free, ordered as a tour.

    Each location (poses along a drift, ``spread_deg`` apart) gets itself plus two
    neighbours that vary the joints which change gravity (``pose_space``), so the static
    check maps the error around it. At most ``max_poses`` (locations far apart first).
    """
    from .static_check import pose_space, plan_tour, structural_reference

    if not drifts:
        return np.zeros((0, robot.dof))
    rng = np.random.default_rng(seed)
    lower, upper = robot.safety_window(margin_deg=3.0)
    _, weights = pose_space(robot, dynamics)
    sampled = weights > 0
    spread = np.radians(spread_deg)
    locations = []
    for pose in np.radians([p for d in drifts for p in d.path_deg]):
        if all(np.abs((pose - u)[sampled]).max() > spread for u in locations):
            locations.append(np.clip(pose, lower, upper))
    candidates, group = [], []
    for k, centre in enumerate(locations):
        candidates.append(centre)
        group.append((0, k))
        for n in range(1, 7):
            p = centre.copy()
            p[sampled] += rng.uniform(-1, 1, int(sampled.sum())) * spread
            candidates.append(np.clip(p, lower, upper))
            group.append((n, k))
    candidates = np.array(candidates)
    reference = structural_reference(robot)
    report = check_clearance(dynamics, candidates, np.arange(float(len(candidates))), table_z, clearance,
                             reference_q=reference)
    bad = {int(round(e["time"])) for e in report["events"]} if report is not None else set()
    keep = [i for i in range(len(candidates)) if i not in bad]
    # Locations first, then one neighbour of each, then a second; spares after that.
    order = sorted(keep, key=lambda i: group[i])
    n = min(max_poses, 3 * len(locations), len(order))
    return plan_tour(dynamics, [candidates[i] for i in order[:n]], [candidates[i] for i in order[n:]], n, rest,
                     table_z, clearance, reference)


def summarize(drifts: list[Drift]) -> list[str]:
    lines = []
    for j in sorted({d.joint for d in drifts}):
        own = [d for d in drifts if d.joint == j]
        sign = np.sign(sum(d.error for d in own))
        lines.append(f"J{j + 1}：{len(own)} 次自己转动，多为{'正' if sign > 0 else '负'}向，"
                     f"估计模型误差 {min(d.error for d in own):+.2f} … {max(d.error for d in own):+.2f} N·m")
    return lines


def load_log(path: str | Path, dof: int) -> dict[str, np.ndarray]:
    header, data = read_named_csv(path)
    out = {"time": data[:, header.index("time")]}
    for prefix in ("q", "dq", "kd"):
        columns = named_columns(header, prefix, dof)
        if columns is None:
            raise SystemExit(f"{path}: 缺少 {prefix}1..{prefix}{dof} 列（需要 rebot_taccap_identification gravity 记录的日志）")
        out[prefix] = data[:, columns]
    return out


def analyze(robot: Robot, dynamics: Dynamics, log: dict, rest=None, table_z: float = 0.0,
            out: Path | None = None) -> dict:
    """Find drifts in a log, print a report and (with ``out``) save suggested static poses."""
    from .static_check import save_pose_file

    found = find_drifts(log["time"], log["q"], log["dq"], log["kd"])
    drifts, dropped = screen(found, dynamics, *robot.safety_window())
    result = {"drifts": [asdict(d) for d in drifts], "summary": summarize(drifts), "poses": None, "n_poses": 0,
              "dropped": dropped}
    print("\n== 漂移分析（free 模式下关节自己转动的地方 = 模型重力不准的地方）")
    reasons = [f"{n} 段{text}" for key, text in (("hand", "速度太快或同时有别的关节被快速推动（手推）"),
                                                ("no_gravity", "在不受重力的关节上"),
                                                ("conflict", "同一位姿正反两个方向都转过（手来回推）"))
               if (n := dropped[key])]
    if reasons:
        print(f"  忽略 {sum(dropped.values())} 段：" + "，".join(reasons))
    if not drifts:
        print("没有发现自己转动的关节（只针对这次走过的区域）。")
    for d in drifts:
        print("  " + d.describe())
    for line in result["summary"]:
        print("  → " + line)
    if drifts and out is not None:
        rest = robot.rest() if rest is None else rest
        poses = suggest_poses(robot, dynamics, drifts, rest, table_z)
        if len(poses):
            save_pose_file(out, poses, source="rebot_taccap_identification drift analysis", drifts=result["drifts"])
            result["poses"], result["n_poses"] = str(out), int(len(poses))
            print(f"已生成 {len(poses)} 个补数据的静态位姿 → {out}\n"
                  "  用法：第 1 步静态核对（负载模型：第 4 步负载辨识）载入并执行，再把结果和原来的静态数据一起作为静态训练数据重新辨识。")
    return result


def main(argv=None) -> None:
    from ..results import Results
    from ..robots.utils import ROBOT_HELP, cli_robot

    parser = argparse.ArgumentParser(prog="rebot_taccap_identification drift", description=__doc__, epilog=ROBOT_HELP,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", required=True, help="重力补偿日志（<结果目录>/gravity/*.csv）")
    parser.add_argument("--urdf", help="补偿时用的模型（默认取日志的 .meta.json）")
    parser.add_argument("--out", help="补数据位姿文件（默认 <模型所在的结果目录>/static/drift_<日志名>.poses.json）")
    parser.add_argument("--table-z", type=float, help="桌面高度 [m]（默认取日志的 .meta.json，再默认取配置）")
    args = parser.parse_args(argv)
    robot = cli_robot()
    meta_path = Path(args.log).with_suffix(".meta.json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    urdf = args.urdf or (resolve_reference(meta["urdf"], meta_path.parent) if meta.get("urdf") else None)
    dynamics = Dynamics.for_robot(robot, urdf, full=bool(meta.get("full_urdf")))
    table_z = args.table_z if args.table_z is not None else meta.get("table_z", robot.config.table_z)
    out = Path(args.out) if args.out else Results.for_robot(robot).package_of(dynamics.robot.source).poses(
        f"drift_{Path(args.log).stem}")
    analyze(robot, dynamics, load_log(args.log, robot.dof), table_z=table_z, out=out)


if __name__ == "__main__":
    main()
