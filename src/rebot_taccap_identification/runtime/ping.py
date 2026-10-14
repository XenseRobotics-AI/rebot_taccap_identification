"""Read-only check that every joint of an arm answers (no torque, no motion).

  rebot_taccap_identification ping --robot.type=rebot_b601_rs
"""

from __future__ import annotations

import argparse

from ..robots.utils import ROBOT_HELP, cli_robot


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(prog="rebot_taccap_identification ping", description=__doc__, epilog=ROBOT_HELP,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.parse_args(argv)
    robot = cli_robot()
    print(f"{robot}：关节 {', '.join(robot.joint_names)}")
    try:
        results = robot.ping()
    except Exception as exc:  # noqa: BLE001 - e.g. bus not available
        print(f"无法打开通信：{exc}")
        results = [(name, False, "") for name in robot.joint_names]
    for j, (name, ok, detail) in enumerate(results, start=1):
        print(f"J{j} {name}: {'OK' if ok else '无应答'}  {detail}", flush=True)
    good = sum(ok for _, ok, _ in results)
    print(f"结果：{good}/{robot.dof} 个关节应答" + ("" if good == robot.dof else "。检查电源/急停/通信线，必要时重启通信口。"))


if __name__ == "__main__":
    main()
