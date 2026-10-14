"""Compare gravity models against static torque measurements (acceptance test).

For every static CSV and model: per-joint RMS error, mean bias (> 0: the model
pushes harder than gravity needs, the arm drifts in +q under gravity
compensation), and how many poses would drift (|error| > static friction
measured at that pose). Only joints whose gravity torque varies are shown.
Measured torques are divided by the robot's torque calibration (``robot.torque_gain()``).
As in gravity compensation, a model is its URDF plus the joint offsets exported next to
it (``<urdf stem>_offset.yaml``), if there are any.

  rebot_taccap_identification evaluate --robot.type=rebot_b601_rs --models v3=a.urdf v4=b.urdf --static results/rebot_b601_rs/static/check.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from ..export import load_offsets, offsets_path
from ..io import named_columns, read_named_csv
from ..robots.utils import ROBOT_HELP, cli_robot
from .common import Dynamics


def load_static(path: str, dof: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    header, data = read_named_csv(path)
    columns = {p: named_columns(header, p, dof) for p in ("q_deg", "tau_meas", "static_friction")}
    if columns["q_deg"] is None or columns["tau_meas"] is None:
        raise SystemExit(f"{path}: 不是静态核对的 CSV（需要 q_deg*, tau_meas* 列）")
    friction = data[:, columns["static_friction"]] if columns["static_friction"] else np.full((len(data), dof), np.nan)
    return np.radians(data[:, columns["q_deg"]]), data[:, columns["tau_meas"]], friction


class Model:
    """URDF gravity plus the joint offsets exported with it."""

    def __init__(self, dynamics: Dynamics, urdf: str, with_bias: bool = True):
        self.dynamics = dynamics
        path = offsets_path(urdf)
        self.has_offset = with_bias and path.exists()
        self.offset = load_offsets(path, dynamics.robot.joint_names) if self.has_offset else 0.0

    def gravity(self, q) -> np.ndarray:
        return self.dynamics.gravity(q) + self.offset


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(prog="rebot_taccap_identification evaluate", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=ROBOT_HELP)
    parser.add_argument("--models", nargs="+", required=True, help="名称=URDF（真实力矩单位的模型）")
    parser.add_argument("--static", nargs="+", required=True, help="静态核对 CSV")
    parser.add_argument("--torque-gain", type=float, nargs="+", help="力矩标定：反馈/指令力矩（1 个或每轴一个；默认取标定文件）")
    parser.add_argument("--full-urdf", action="store_true",
                        help="模型是第 5 步导出的完整 URDF（末端以下的负载算进重力）")
    parser.add_argument("--no-offset", action="store_true", help="只用 URDF，不加关节力矩偏置")
    parser.add_argument("--min-span", type=float, default=0.3, help="重力力矩变化小于此值 [N·m] 的关节不显示")
    args = parser.parse_args(argv)

    robot = cli_robot()
    gain = robot.per_joint(args.torque_gain, "torque_gain") if args.torque_gain is not None else robot.torque_gain()
    models = {}
    for spec in args.models:
        name, _, path = spec.partition("=")
        models[name] = Model(Dynamics.for_robot(robot, path or name, full=args.full_urdf), path or name,
                             not args.no_offset)
    with_bias = [name for name, model in models.items() if model.has_offset]
    if with_bias:
        print(f"加上关节力矩偏置（_offset.yaml）：{', '.join(with_bias)}")
    width = max(max(len(n) for n in models), 4)
    for path in args.static:
        q, tau, friction = load_static(path, robot.dof)
        tau = tau / gain
        reference = next(iter(models.values())).gravity(q)
        joints = [j for j in range(robot.dof) if np.ptp(reference[:, j]) >= args.min_span]
        print(f"\n== {Path(path).name}: {len(q)} 个位姿（显示 {', '.join(f'J{j + 1}' for j in joints)}）")
        header = "  ".join(f"J{j + 1} RMS/偏差/会漂" for j in joints)
        print(f"{'模型':{width}s}  {header}")
        for name, model in models.items():
            error = model.gravity(q) - tau
            cells = []
            for j in joints:
                known = np.isfinite(friction[:, j]).all()
                drift = f"{int(np.sum(np.abs(error[:, j]) > friction[:, j])):>2d}" if known else "--"
                cells.append(f"{np.sqrt(np.mean(error[:, j] ** 2)):.3f}/{error[:, j].mean():+.3f}/{drift}")
            print(f"{name:{width}s}  " + "  ".join(f"{c:>17s}" for c in cells))


if __name__ == "__main__":
    main()
