"""Robot factory, plugin discovery and ``--robot.<field>`` arguments .

- Built-in robots are the sub-packages of ``rebot_taccap_identification.robots`` (``<name>/config_<name>.py``
  and ``<name>/<name>.py``); they are imported so their configs register.
- Third-party robots are installed packages whose name starts with ``rebot_taccap_identification_robot_``;
  they are imported the same way.
- The robot class of a config is found by name: ``FooConfig`` -> ``Foo``, looked up in the
  config's module, its package, and the sibling module without the ``config_`` prefix.
"""

from __future__ import annotations

import dataclasses
import importlib
import importlib.metadata
import pkgutil
import tomllib
from typing import Any

from .config import RobotConfig
from .robot import Robot

PLUGIN_PREFIX = "rebot_taccap_identification_robot_"
IMPORT_ERRORS: dict[str, str] = {}
_imported = False


def register_robots() -> dict[str, str]:
    """Import built-in robot packages and third-party plugins once; returns import errors."""
    global _imported
    if _imported:
        return dict(IMPORT_ERRORS)
    from . import __path__ as package_path

    names = [f"{__package__}.{m.name}" for m in pkgutil.iter_modules(package_path) if m.ispkg]
    for dist in importlib.metadata.distributions():
        dist_name = (dist.metadata.get("Name") or "").replace("-", "_")
        if dist_name.startswith(PLUGIN_PREFIX):
            names.append(dist_name)
    for name in dict.fromkeys(names):
        try:
            importlib.import_module(name)
        except Exception as exc:  # noqa: BLE001 - one broken robot must not hide the others
            IMPORT_ERRORS[name] = f"{type(exc).__name__}: {exc}"
    _imported = True
    return dict(IMPORT_ERRORS)


def robot_class_for_config(config_cls: type[RobotConfig]) -> type[Robot]:
    name = config_cls.__name__
    if not name.endswith("Config"):
        raise ValueError(f"配置类名 {name!r} 必须以 Config 结尾")
    device = name[:-len("Config")]
    module = config_cls.__module__
    parts = module.split(".")
    candidates = [module, ".".join(parts[:-1])]
    if parts[-1].startswith("config_"):
        candidates.append(".".join(parts[:-1] + [parts[-1][len("config_"):]]))
    for candidate in dict.fromkeys(c for c in candidates if c):
        try:
            found = getattr(importlib.import_module(candidate), device, None)
        except ImportError:
            continue
        if isinstance(found, type) and issubclass(found, Robot):
            return found
    raise ImportError(f"找不到 {name} 对应的机器人类 {device}（在 {', '.join(candidates)} 中）")


def _parse_value(text: str) -> Any:
    """``--robot.kp=[1,2]`` -> [1, 2]; numbers, booleans and lists use TOML syntax, else a string."""
    try:
        return tomllib.loads(f"v = {text}")["v"]
    except tomllib.TOMLDecodeError:
        return text


def make_robot_config(robot_type: str, **fields: Any) -> RobotConfig:
    register_robots()
    config_cls = RobotConfig.get_choice_class(robot_type)
    known = {f.name for f in dataclasses.fields(config_cls)}
    unknown = sorted(set(fields) - known)
    if unknown:
        raise ValueError(f"{robot_type} 没有字段 {', '.join(unknown)}；可用：{', '.join(sorted(known))}")
    return config_cls(**fields)


def make_robot_from_config(config: RobotConfig) -> Robot:
    return robot_class_for_config(type(config))(config)


def make_robot(robot_type: str, **fields: Any) -> Robot:
    """Shortcut: ``make_robot("rebot_b601_rs", channel="can1")``."""
    return make_robot_from_config(make_robot_config(robot_type, **fields))


# ------------------------------------------------------------------ command line
def pop_robot_args(argv: list[str]) -> tuple[list[str], str | None, dict[str, Any]]:
    """Take ``--robot.type=X`` and ``--robot.<field>=<value>`` out of ``argv``.

    Returns the remaining arguments, the robot type and the field values. Both
    ``--robot.f=v`` and ``--robot.f v`` are accepted.
    """
    rest, fields, robot_type = [], {}, None
    i = 0
    while i < len(argv):
        token = argv[i]
        if token.startswith("--robot."):
            key, eq, value = token[len("--robot."):].partition("=")
            if not eq:
                i += 1
                if i >= len(argv):
                    raise SystemExit(f"{token} 缺少值")
                value = argv[i]
            if key == "type":
                robot_type = value
            else:
                fields[key.replace("-", "_")] = _parse_value(value)
        else:
            rest.append(token)
        i += 1
    return rest, robot_type, fields


def robot_args(robot_type: str, fields: dict[str, Any]) -> list[str]:
    """Inverse of ``pop_robot_args``, to start a child process with the same robot."""
    import json

    out = [f"--robot.type={robot_type}"]
    for key, value in fields.items():
        out.append(f"--robot.{key}={json.dumps(value) if not isinstance(value, str) else value}")
    return out


ROBOT_HELP = """机器人：
  --robot.type=TYPE        机器人类型，如 rebot_b601_rs（rebot_taccap_identification list 列出全部）
  --robot.id=ID            同型号多台实机时区分（标定文件、结果目录）
  --robot.<字段>=<值>      覆盖该类型配置的任意字段，如 --robot.channel=can1 --robot.kp=[50,50,50,30,50,50]
"""

_CLI_ROBOT: dict[str, Any] = {"type": None, "fields": {}}


def set_cli_robot(robot_type: str | None, fields: dict[str, Any]) -> None:
    """Called by the command line entry point after ``pop_robot_args``."""
    _CLI_ROBOT["type"], _CLI_ROBOT["fields"] = robot_type, dict(fields)


def cli_robot_type() -> str | None:
    return _CLI_ROBOT["type"]


def cli_robot_config() -> RobotConfig:
    if not _CLI_ROBOT["type"]:
        raise SystemExit("需要 --robot.type=<类型>（rebot_taccap_identification list 列出全部类型）")
    return make_robot_config(_CLI_ROBOT["type"], **_CLI_ROBOT["fields"])


def cli_robot() -> Robot:
    return make_robot_from_config(cli_robot_config())
