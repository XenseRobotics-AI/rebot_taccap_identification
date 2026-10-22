"""Command line: ``rebot_taccap_identification <command> [--robot.<field>=<value>] [options]``.

``static`` and ``gravity`` only check and preview unless ``--execute`` is given; ``ping``
is read-only. The robot defaults to ``rebot_b601_rs``.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import sys
from pathlib import Path

import numpy as np

from .resources import bundle_urdf, relative_reference, resolve_reference

DEFAULT_ROBOT = "rebot_b601_rs"

HARDWARE_COMMANDS = {
    "ping": "rebot_taccap_identification.runtime.ping",
    "static": "rebot_taccap_identification.runtime.static_check",
    "gravity": "rebot_taccap_identification.runtime.gravity_comp",
    "evaluate": "rebot_taccap_identification.runtime.evaluate",
    "drift": "rebot_taccap_identification.runtime.drift",
}

USAGE = """usage: rebot_taccap_identification <command> [--robot.<字段>=<值>] [options]

  studio     图形工作台
  ping       只读检测每个关节是否应答
  static     静态力矩核对：在一组静止位姿测保持力矩（辨识的数据）
  identify   用静态数据辨识各连杆的质量、质心，导出 URDF
  evaluate   在静态数据上比较模型（验收表）
  gravity    重力补偿（闭环检验模型）
  drift      分析重力补偿记录，找出会漂的地方并生成补数据的静态位姿
  payload    装上负载（夹爪）后，在本体模型上辨识负载的质量、质心
  package    以原始 URDF 为模板导出完整的 URDF 包（urdf/ + meshes/）
  list       列出可用的机器人类型

每个命令的参数用 -h 查看。"""


# ------------------------------------------------------------------ helpers
def _robot():
    from .robots.utils import cli_robot

    return cli_robot()


def _per_joint(values, dof: int, name: str) -> np.ndarray:
    values = list(np.ravel(values))
    if len(values) not in (1, dof):
        raise SystemExit(f"{name} 需要 1 个或 {dof} 个值")
    return np.broadcast_to(np.asarray(values, dtype=float), (dof,)).copy()


def static_spec(spec: str) -> tuple[str, float]:
    """``FILE`` or ``FILE@K`` -> (file, weight factor)."""
    path, at, factor = spec.rpartition("@")
    if not at:
        return spec, 1.0
    try:
        return path, float(factor)
    except ValueError:
        return spec, 1.0


def load_statics(specs: list[str], dof: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Poses [rad], holding torques and per-sample weights of static check CSVs (``FILE[@K]``)."""
    from .runtime.evaluate import load_static

    qs, taus, weights = [], [], []
    for spec in specs:
        path, factor = static_spec(spec)
        q, tau, _ = load_static(path, dof)
        qs.append(q), taus.append(tau), weights.append(np.full(len(q), factor))
    if not qs:
        raise SystemExit("需要 --static 数据")
    return np.vstack(qs), np.vstack(taus), np.concatenate(weights)


def _gain(robot, values) -> np.ndarray:
    return robot.torque_gain() if values is None else _per_joint(values, robot.dof, "--torque-gain")


def _print_rmse(label: str, values) -> None:
    print(f"{label}：{np.array2string(np.asarray(values), precision=3)} N·m")


# ------------------------------------------------------------------ identify
def cmd_identify(args: argparse.Namespace) -> int:
    from .export import export_urdf, offsets_path, write_offsets
    from .identify import fit_masses
    from .results import Results
    from .runtime.common import Dynamics

    robot = _robot()
    model = Dynamics.for_robot(robot).robot
    q, tau, weights = load_statics(args.static, robot.dof)
    gain = _gain(robot, args.torque_gain)
    print(f"静态数据：{len(q)} 个位姿（{len(args.static)} 个文件）；力矩标定 {np.round(gain, 3).tolist()}")
    result = fit_masses(model, q, tau / gain, weights, prior_weight=args.prior_weight)

    results = Results.for_robot(robot)
    version = args.version or results.next_version()
    out = results.urdf(version, robot.model_prefix())
    export_urdf(model, result.inertial, out, tip_link=robot.config.tip_link,
                note=f"Mass and centre of mass from static data ({len(q)} poses); inertia about each CoM from CAD")
    write_offsets(offsets_path(out), model.joint_names, result.offset, source=str(out.name))
    summary = {"version": version, "urdf": out.name, "mass_kg": result.mass.tolist(), "com_m": result.com.tolist(),
               "offset_nm": result.offset.tolist(), "rmse_nm": result.rmse.tolist(), "rank": result.rank,
               "samples": result.n_samples, "notes": result.notes,
               "robot_type": robot.config.type, "robot_id": robot.id, "joint_names": model.joint_names,
               "gravity_m_s2": list(robot.config.gravity), "identification_kind": "static_gravity",
               "inertia_source": "CAD (not identified from static measurements)"}
    results.result(version).write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    names = lambda specs: [Path(static_spec(spec)[0]).stem for spec in specs]
    results.save_inputs(version, static=names(args.static), holdout=names(args.val), files=args.static + args.val,
                        prior_weight=args.prior_weight, torque_gain=gain.tolist())

    print(f"数据确定了 {result.rank} 个质量/质心组合；其余方向由先验（原 URDF 的值）决定")
    for name, m0, m, c in zip(model.link_names, model.nominal_parameters[:, 0], result.mass, result.com):
        print(f"  {name}: 质量 {m0:.3f} → {m:.3f} kg，质心 [{', '.join(f'{1000 * v:.1f}' for v in c)}] mm")
    print(f"  关节力矩偏置：{np.round(result.offset, 3).tolist()} N·m")
    _print_rmse("拟合残差 RMS", result.rmse)
    for note in result.notes:
        print("note: " + note)
    print(f"已导出 {out}\n       {offsets_path(out).name}（关节力矩偏置，重力补偿和验收自动加上）")
    if args.val:
        from .runtime import evaluate

        evaluate.main(["--models", f"原始={robot.config.urdf_path}", f"{version}={out}", "--static", *args.val,
                       "--torque-gain", *[f"{g:g}" for g in gain]])
    return 0


# ------------------------------------------------------------------ payload
def cmd_payload(args: argparse.Namespace) -> int:
    from .export import export_urdf, load_offsets, offsets_path, write_offsets
    from .payload import cad_payload, fit_payload, with_payload
    from .robot import RobotModel

    robot = _robot()
    base = Path(args.urdf)
    model = RobotModel.from_urdf(base, tip_link=robot.config.tip_link, gravity=tuple(robot.config.gravity))
    offset = load_offsets(offsets_path(base), model.joint_names) if offsets_path(base).exists() else None
    q, tau, weights = load_statics(args.static, robot.dof)
    gain = _gain(robot, args.torque_gain)
    cad_template = robot.config.urdf_path
    cad = cad_payload(cad_template, model.link_names[-1])
    result = fit_payload(model, q, tau / gain, offset=offset, sample_weights=weights, radius=args.radius, cad=cad)
    link = model.link_names[-1]
    print(f"负载 {args.name}（固定在 {link} 上）：质量 {result.mass:.3f} kg，"
          f"质心 [{', '.join(f'{1000 * v:.1f}' for v in result.com)}] mm（{link} 坐标系）")
    print(f"惯量（关于质心）：{result.inertia_source}")
    _print_rmse("残差 RMS  只用本体模型", result.rmse_arm)
    _print_rmse("          加上负载    ", result.rmse_payload)
    before = 0.0 if offset is None else float(offset[-1])
    print(f"{model.joint_names[-1]} 力矩偏置（装上负载后重新估计）：{before:+.3f} → {result.offset[-1]:+.3f} N·m")
    for note in result.notes:
        print("note: " + note)
    out = Path(args.out_urdf)
    arm_snapshot = bundle_urdf(base, out.parent / "arm_model" / base.name)
    cad_snapshot = bundle_urdf(cad_template, out.parent / "cad_model" / cad_template.name)
    export_urdf(model, with_payload(model, result.parameters), out, tip_link=robot.config.tip_link,
                note=f"Payload {args.name} ({result.mass:.3f} kg) merged into {link}; arm model {base.name}")
    write_offsets(offsets_path(out), model.joint_names, result.offset,
                  source=f"{base.name} + payload {args.name} (last joint refitted)")
    info = {"name": args.name, "link": link, "mass_kg": result.mass, "com_m": result.com.tolist(),
            "cad_urdf": relative_reference(cad_snapshot, out.parent),
            "cad_urdf_sha256": hashlib.sha256(cad_snapshot.read_bytes()).hexdigest(),
            "inertial_10": result.parameters.tolist(), "inertia_source": result.inertia_source,
            "arm_urdf": relative_reference(arm_snapshot, out.parent), "rmse_arm": result.rmse_arm.tolist(),
            "rmse_with_payload": result.rmse_payload.tolist(), "offset_nm": result.offset.tolist(),
            "torque_gain": gain.tolist(), "notes": result.notes,
            "robot_type": robot.config.type, "robot_id": robot.id, "joint_names": model.joint_names,
            "gravity_m_s2": list(robot.config.gravity), "identification_kind": "static_payload"}
    out.with_name("payload.json").write_text(json.dumps(info, indent=2, ensure_ascii=False))
    print(f"已导出 {out}（本体 + 负载，负载并入 {link}）和 payload.json")
    if args.val:
        from .runtime import evaluate

        evaluate.main(["--models", f"本体={base}", f"本体+{args.name}={out}", "--static", *args.val,
                       "--torque-gain", *[f"{g:g}" for g in gain]])
    return 0


# ------------------------------------------------------------------ package
def cmd_package(args: argparse.Namespace) -> int:
    from .export import offsets_path
    from .package import export_package, find_payload
    from .results import Results

    robot = _robot()
    model = Path(args.model)
    payload = find_payload(model)
    arm = resolve_reference(payload["arm_urdf"], model.parent) if payload else model
    saved_template = resolve_reference(payload["cad_urdf"], model.parent) if payload and payload.get("cad_urdf") else None
    template = Path(args.template) if args.template else (
        saved_template if saved_template is not None and saved_template.is_file() else robot.config.urdf_path)
    tip_link = robot.config.tip_link
    tag = model.stem.split("_identified_")[-1] if "_identified_" in model.stem else model.stem
    name = args.name or f"{template.stem}_{tag}" + (f"_{payload.get('name', 'payload')}" if payload else "")
    out = Path(args.out) if args.out else Results.for_robot(robot).export_dir / name
    # A payload model has its own offsets (the last joint's is refitted with the payload mounted).
    report = export_package(template, arm, out, tip_link=tip_link, payload=payload, offset_yaml=offsets_path(model),
                            name=name)
    print(f"已导出 {report.urdf}")
    print(f"  网格 {len(report.meshes)} 个 → {report.urdf.parent.parent / 'meshes'}（全部存在，路径 ../meshes/…）")
    if report.payload:
        com = ", ".join(f"{1000 * v:.1f}" for v in report.payload["com_m"])
        print(f"  负载 {report.payload['name']}：{report.payload['mass_kg']:.3f} kg，质心 [{com}] mm（{tip_link} 坐标系），"
              "模板的负载连杆、网格和关节都保留")
    else:
        print(f"  不带负载：去掉了 {tip_link} 以下的 {', '.join(report.removed) or '（没有）'}")
    for note in report.notes:
        print("  note: " + note)
    return 0


def _list_robots() -> int:
    from .robots import RobotConfig, register_robots

    errors = register_robots()
    for name in sorted(RobotConfig.get_known_choices()):
        print(name)
    for module, error in errors.items():
        print(f"[导入失败] {module}: {error}")
    return 0


# ------------------------------------------------------------------ main
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="rebot_taccap_identification", description=USAGE,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    identify = sub.add_parser("identify", help="静态数据 → 质量、质心 → URDF")
    identify.add_argument("--static", nargs="+", required=True,
                          help="静态核对 CSV；文件@K 让这个文件的权重乘 K（如重力补偿漂移补点 drift_…csv@5）")
    identify.add_argument("--val", nargs="*", default=[], help="静态验证 CSV（不参与辨识，只出验收表）")
    identify.add_argument("--version", help="版本名（默认下一个 vN）")
    identify.add_argument("--prior-weight", type=float, default=0.1,
                          help="拉向原 URDF 质量/质心的先验强度（相对值，默认 0.1；作用于所有方向）")
    identify.add_argument("--torque-gain", type=float, nargs="+", help="力矩标定：反馈/指令力矩（1 个或每轴一个；默认取标定文件）")

    payload = sub.add_parser("payload", help="在已辨识的本体上辨识末端负载的质量、质心")
    payload.add_argument("--urdf", required=True, help="已辨识的本体 URDF（models/vN/…，不含负载）")
    payload.add_argument("--static", nargs="+", required=True, help="装着负载采的静态 CSV（可写 文件@K）")
    payload.add_argument("--val", nargs="*", default=[], help="装着负载采的静态验证 CSV")
    payload.add_argument("--name", default="gripper", help="负载名称")
    payload.add_argument("--out-urdf", required=True, help="本体 + 负载的 URDF（负载并入末端连杆）")
    payload.add_argument("--radius", type=float, default=0.03, help="URDF 里没有负载 CAD 时，按此半径的实心球估计惯量 [m]")
    payload.add_argument("--torque-gain", type=float, nargs="+", help="力矩标定：反馈/指令力矩（1 个或每轴一个；默认取标定文件）")

    package = sub.add_parser("package", help="完整 URDF 包（模板 + 辨识的质量质心 + 全部网格）")
    package.add_argument("--model", required=True, help="辨识出的 URDF：本体（models/vN/）或本体 + 负载（<负载>/models/vN/）")
    package.add_argument("--template", help="原始 URDF（默认用机器人自带的）")
    package.add_argument("--name", help="包名（默认 <模板>_<版本>[_<负载>]）")
    package.add_argument("--out", help="输出目录（默认 <结果目录>/export/<包名>）")
    return parser


def main(argv: list[str] | None = None) -> int:
    from .robots.utils import pop_robot_args, set_cli_robot

    argv = sys.argv[1:] if argv is None else list(argv)
    argv, robot_type, robot_fields = pop_robot_args(argv)
    set_cli_robot(robot_type or DEFAULT_ROBOT, robot_fields)
    command = argv[0] if argv else ""
    if command in ("", "-h", "--help"):
        print(USAGE)
        return 0 if command else 2
    if command == "list":
        return _list_robots()
    if command == "studio":
        try:
            from .studio.app import main as studio_main
        except ImportError as exc:
            raise SystemExit(f"工作台需要图形界面依赖（{exc.name}）：pip install -e .（或 pip install PySide6 pyqtgraph PyOpenGL）") from exc

        return studio_main(argv[1:]) or 0
    if command in HARDWARE_COMMANDS:
        importlib.import_module(HARDWARE_COMMANDS[command]).main(argv[1:])
        return 0
    args = build_parser().parse_args(argv)
    handlers = {"identify": cmd_identify, "payload": cmd_payload, "package": cmd_package}
    try:
        return handlers[args.command](args)
    except ValueError as exc:
        raise SystemExit(f"错误：{exc}") from exc


if __name__ == "__main__":
    raise SystemExit(main())
