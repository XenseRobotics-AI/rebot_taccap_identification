"""Mass and centre of mass identification workbench for the reBot B601 RS (PySide6).

One window for the whole procedure, step by step:

  0 设置与连接     robot id, torque calibration, joint check
  1 静态核对       direction/zero check, torque-gain measurement, static data for identification
  2 辨识质量质心   mass and CoM of every link from the static data, export URDF, acceptance table
  3 重力补偿       closed-loop test of a model on the arm, drift analysis
  4 负载辨识       mass and CoM of a payload (gripper) on an identified arm
  5 导出 URDF      complete URDF package (urdf/ + meshes/)

Hardware runs, identification and export run ``python -m rebot_taccap_identification.cli <command>`` as
child processes (static and gravity with --stream), with the same safety checks and
safe stop as on the command line. Stop = Ctrl+C (safe stop), press again = disable.
Pose generation runs in the workbench process.

  rebot_taccap_identification studio
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QEvent, QThread, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QMessageBox,
    QPlainTextEdit, QPushButton, QScrollArea, QSlider, QSpinBox, QSplitter, QStackedWidget, QTableWidget,
    QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from ..robots import RobotConfig, make_robot, register_robots, robot_args
from ..robots.utils import IMPORT_ERRORS, cli_robot_type, pop_robot_args
from ..results import Results
from ..runtime.common import Dynamics
from .runner import LivePlots, PoseCoverage, Runner


def _last_robot_file() -> Path:
    return Path.home() / ".config" / "rebot_taccap_identification" / "last_robot.json"


def remember_last_robot(robot_type: str, fields: dict) -> None:
    try:
        _last_robot_file().parent.mkdir(parents=True, exist_ok=True)
        _last_robot_file().write_text(json.dumps({"type": robot_type, "fields": fields}))
    except OSError:
        pass


def is_static_val(name: str) -> bool:
    """Static data for validation only: names containing "val"."""
    return "val" in name


def available_robots() -> list[tuple[str, str | None]]:
    """Registered robot types and why each cannot be built with its defaults (None: usable)."""
    register_robots()
    out = []
    for name in sorted(RobotConfig.get_known_choices()):
        try:
            make_robot(name)
            out.append((name, None))
        except Exception as exc:  # noqa: BLE001
            out.append((name, f"{type(exc).__name__}: {exc}"))
    return out


def default_robot() -> tuple[str, dict] | None:
    """Last used robot if it still loads, else the first usable one."""
    try:
        last = json.loads(_last_robot_file().read_text())
        make_robot(last["type"], **last.get("fields", {}))
        return last["type"], dict(last.get("fields", {}))
    except Exception:  # noqa: BLE001 - missing, stale or unregistered
        pass
    usable = [name for name, error in available_robots() if error is None]
    return (usable[0], {}) if usable else None


# Static data purposes: training (used for identification) and validation (only scored).
PURPOSE_COLORS = {"train": "#4c9be8", "val": "#e8a23c"}

STEPS = ("0  设置与连接", "1  静态核对", "2  辨识质量质心", "3  重力补偿", "4  负载辨识", "5  导出 URDF")


def coverage_pairs(joints: list[int]) -> list[tuple[int, int]]:
    """Joint pairs for the coverage plots: consecutive sampled joints, every one shown at least once.

    An odd last joint is paired with its neighbour; a single joint is plotted against
    the pose index (second element -1).
    """
    joints = sorted(joints)
    if len(joints) == 1:
        return [(joints[0], -1)]
    pairs = [(joints[i], joints[i + 1]) for i in range(0, len(joints) - 1, 2)]
    if len(joints) % 2:
        pairs.append((joints[-2], joints[-1]))
    return pairs


class Job(QThread):
    """Runs a Python callable off the GUI thread."""

    done = Signal(object)
    failed = Signal(str)

    def __init__(self, fn, parent=None):
        super().__init__(parent)
        self.fn = fn

    def run(self) -> None:
        try:
            self.done.emit(self.fn())
        except Exception as exc:  # noqa: BLE001 - shown to the user
            self.failed.emit(f"{type(exc).__name__}: {exc}")


def label(text: str, muted: bool = True) -> QLabel:
    widget = QLabel(text)
    widget.setWordWrap(True)
    if muted:
        widget.setStyleSheet("color: #8f8e85;")
    return widget


def short_path(path) -> str:
    """Path relative to the project (or ~) for display."""
    from ..robots.config import project_root

    path = Path(path)
    for base, prefix in ((project_root(), ""), (Path.home(), "~/")):
        try:
            return prefix + str(path.relative_to(base))
        except ValueError:
            continue
    return str(path)


def button(text: str, slot, primary: bool = False) -> QPushButton:
    widget = QPushButton(text)
    widget.clicked.connect(slot)
    if primary:
        widget.setStyleSheet("QPushButton { background: #2a78d6; color: white; font-weight: 600; padding: 6px 14px; }"
                             "QPushButton:disabled { background: #555; }")
    return widget


def data_columns(layout: QVBoxLayout, columns) -> dict[str, QListWidget]:
    """Side-by-side checkable lists of data files, ``columns`` = ((key, title, color), ...).

    The divider between them can be dragged to widen one; each list is as tall as its
    rows (``fit_rows``), shows the full file name as a tooltip and scrolls sideways
    instead of cutting names; its caption counts the checked files.
    """
    splitter = QSplitter(Qt.Horizontal)
    splitter.setChildrenCollapsible(False)
    lists = {}
    for key, title, color in columns:
        box = QWidget()
        column = QVBoxLayout(box)
        column.setContentsMargins(0, 0, 4, 0)
        caption = QLabel(title)
        caption.setStyleSheet(f"color: {color}; font-weight: 700;")
        widget = QListWidget()
        widget.setTextElideMode(Qt.ElideNone)
        widget.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        widget.caption, widget.title = caption, title
        widget.itemChanged.connect(lambda _, w=widget: count_checked(w))
        column.addWidget(caption)
        column.addWidget(widget)
        splitter.addWidget(box)
        lists[key] = widget
    layout.addWidget(splitter)
    return lists


def count_checked(widget: QListWidget) -> None:
    """Caption of a ``data_columns`` list: its title and how many files are checked."""
    checked = sum(widget.item(i).checkState() == Qt.Checked for i in range(widget.count()))
    widget.caption.setText(f"{widget.title}（已选 {checked}/{widget.count()}）")


def fit_rows(widget: QListWidget, min_rows: int = 4, max_rows: int = 16) -> None:
    """Make a list as tall as its rows, between ``min_rows`` and ``max_rows`` (beyond that it scrolls),
    and update its caption."""
    count_checked(widget)
    row = widget.sizeHintForRow(0) if widget.count() else widget.fontMetrics().height() + 6
    rows = min(max(widget.count(), min_rows), max_rows)
    widget.setFixedHeight(rows * row + 2 * widget.frameWidth() + widget.horizontalScrollBar().sizeHint().height())


class Studio(QMainWindow):
    def __init__(self, robot_type: str, fields: dict | None = None, with_3d: bool = True):
        super().__init__()
        self.robot_type, self.robot_fields = robot_type, dict(fields or {})
        self.robot_args = robot_args(robot_type, self.robot_fields)   # passed on to every child process
        self.robot = make_robot(robot_type, **self.robot_fields)     # hardware is only opened by child processes
        self.dof = self.robot.dof
        self.setWindowTitle(f"质量质心辨识 · {self.robot.robot_id()} · {self.robot}")
        self.resize(1500, 950)
        self.runner = Runner(Path.cwd(), self)
        self.runner.line.connect(self.log_line)
        self.runner.state.connect(self.on_state)
        self.runner.prompt.connect(self.on_prompt)
        self.runner.finished.connect(self.on_finished)
        self.runner.started.connect(lambda c: self.set_busy(True))
        self.on_done = None                  # callback after the current run
        self.run_lines: list[str] = []       # output of the current run (parsed by its callback)
        self.jobs: list[Job] = []
        self.confirmed_hardware = False
        self.dynamics = Dynamics.for_robot(self.robot)

        self.steps = QListWidget()
        self.steps.setFixedWidth(170)
        self.steps.setFont(QFont(self.font().family(), 11))
        for step in STEPS:
            self.steps.addItem(QListWidgetItem(step))
        self.pages = QStackedWidget()
        self.coverage = PoseCoverage(self.dof)
        for build in (self.page_setup, self.page_static,
                      self.page_identify, self.page_gravity, self.page_payload, self.page_package):
            page = QWidget()
            layout = QVBoxLayout(page)
            build(layout)
            layout.addStretch(1)
            scroll = QScrollArea()                     # pages never get squeezed below their size
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.NoFrame)
            scroll.setWidget(page)
            self.pages.addWidget(scroll)
        self.steps.currentRowChanged.connect(self.on_step)

        if with_3d:
            from .viewer import RobotView

            self.view = RobotView(self.dynamics.robot, self.robot.config.tip_link)
        else:
            self.view = None
        self.live = LivePlots(self.dof)
        self.plots = QStackedWidget()
        self.plots.addWidget(self.live)
        self.coverage.picked.connect(lambda i: self.pose_list.setCurrentRow(i))
        self.plots.addWidget(self.coverage)
        right = QSplitter(Qt.Vertical)
        viewer_box = QWidget()
        viewer_layout = QVBoxLayout(viewer_box)
        viewer_layout.setContentsMargins(0, 0, 0, 0)
        if self.view is not None:
            viewer_layout.addWidget(self.view, 1)
        self.pose_label = QLabel("")
        self.pose_label.setStyleSheet("color: #c3c2b7; font-family: monospace;")
        bar = QHBoxLayout()
        bar.addWidget(self.pose_label, 1)
        if self.view is not None:
            bar.addWidget(button("归位视角", self.view.home))
        viewer_layout.addLayout(bar)
        right.addWidget(viewer_box)
        right.addWidget(self.plots)
        right.setSizes([520, 380])

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.log.setFont(QFont("monospace", 9))
        center = QSplitter(Qt.Vertical)
        center.addWidget(self.pages)
        center.addWidget(self.log)
        center.setSizes([620, 300])
        center.setChildrenCollapsible(False)
        self.log.setMinimumHeight(80)
        self.center = center
        main = QSplitter(Qt.Horizontal)
        main.addWidget(self.steps)
        main.addWidget(center)
        main.addWidget(right)
        main.setSizes([170, 560, 770])
        # Full-width stop bar on top: always visible, large, and reachable without aiming.
        self.stop_button = QPushButton()
        self.stop_button.setMinimumHeight(64)
        self.stop_button.setCursor(Qt.PointingHandCursor)
        self.stop_button.setFocusPolicy(Qt.NoFocus)           # never steals focus from the form
        self.stop_button.pressed.connect(self.runner.stop)    # on press, not on release: faster
        self.stop_button.setStyleSheet(
            "QPushButton { background: #c8312f; color: white; font-size: 22px; font-weight: 700;"
            " border: 3px solid #ff6b69; border-radius: 6px; }"
            "QPushButton:hover { background: #e03b38; }"
            "QPushButton:disabled { background: #2b3038; color: #6b6b6b; border: 1px solid #393936;"
            " font-size: 15px; font-weight: 400; }")
        self.runner.line.connect(lambda _: self.update_stop_button())
        central = QWidget()
        column = QVBoxLayout(central)
        column.setContentsMargins(6, 6, 6, 0)
        column.addWidget(self.stop_button)
        column.addWidget(main, 1)
        self.setCentralWidget(central)
        # Esc / Space stop from anywhere in the application (input fields, popups, dialogs
        # included); the event filter sees every key before the focused widget does.
        QApplication.instance().installEventFilter(self)
        self.update_stop_button()

        self.refresh_files()
        self.set_static_poses(np.zeros((0, self.dof)), "")
        self.update_pose_seeds()
        self.pose_summary.setText("在“静态训练”或“静态验证”里点“生成”。")
        self.steps.setCurrentRow(0)

    # ------------------------------------------------------------------ pages
    def page_setup(self, layout: QVBoxLayout) -> None:
        layout.addWidget(label("<h3>0 设置与连接</h3>", False))
        cfg = self.robot.config

        # --- which arm
        box = QGroupBox("机械臂")
        form = QFormLayout(box)
        self.robot_select = QComboBox()
        self.robot_select.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.robot_select.setMinimumContentsLength(16)
        self.robot_select.setToolTip("机械臂类型。切换后工作台重新启动。")
        self.robot_select.activated.connect(self.on_robot_selected)
        row = QHBoxLayout()
        row.addWidget(self.robot_select, 1)
        row.addWidget(button("刷新", self.fill_robot_list))
        form.addRow("类型", row)
        self.fill_robot_list()
        self.robot_id = QLineEdit(self.robot.id or "")
        self.robot_id.setPlaceholderText("default（同型号有多台实机时填写，如 arm2）")
        self.robot_id.editingFinished.connect(self.on_robot_id)
        form.addRow("ID", self.robot_id)
        model = label(f"{self.dof} 轴 · {cfg.urdf_path.name}" + (f" · 末端 {cfg.tip_link}" if cfg.tip_link else ""), False)
        model.setToolTip(f"{cfg.urdf_path}\n关节：{'、'.join(self.robot.joint_names)}")
        form.addRow("模型", model)
        results = label(short_path(self.robot.results_dir()), False)
        results.setToolTip(f"{self.robot.results_dir()}\n子目录 static / models / gravity / export，负载在 <负载名>/ 下")
        form.addRow("结果目录", results)
        overrides = {k: v for k, v in self.robot_fields.items() if k != "id"}
        if overrides:
            form.addRow("覆盖字段", label(" ".join(robot_args(self.robot_type, overrides)[1:]), False))
        layout.addWidget(box)

        # --- connection
        box = QGroupBox("连接")
        column = QVBoxLayout(box)
        row = QHBoxLayout()
        self.connect_button = button("连接", self.ping, primary=True)
        self.connect_button.setToolTip("检测每个关节是否应答：只读，不使能，不运动")
        row.addWidget(self.connect_button)
        self.connection = label("未连接（只读检测每个关节，不使能、不运动）")
        row.addWidget(self.connection, 1)
        column.addLayout(row)
        channel = getattr(cfg, "channel", None)
        if channel and getattr(cfg, "transport", None) == "serial":
            column.addWidget(label(
                f"DM 串口：{channel} · {cfg.dm_serial_baud} baud\n"
                "通过 --robot.channel 指定当前机械臂的串口，如 /dev/ttyACM0 或 /dev/serial/by-path/…\n"
                "此接口不需要配置 SocketCAN。"))
        elif channel:
            command = QPlainTextEdit(
                "sudo modprobe peak_usb          # PCAN-USB 驱动\n"
                "ip -br link                     # 应能看到 " + channel + "\n"
                f"sudo ip link set {channel} down 2>/dev/null\n"
                f"sudo ip link set {channel} type can bitrate 1000000\n"
                f"sudo ip link set {channel} up")
            command.setReadOnly(True)
            command.setFont(QFont("monospace", 9))
            command.setFixedHeight(command.fontMetrics().lineSpacing() * 5 + 14)
            command.setToolTip("选中复制到终端执行")
            command.setVisible(False)
            toggle = QPushButton(f"▸ {channel} 没起来时的启动命令")
            toggle.setFlat(True)
            toggle.setStyleSheet("text-align: left; color: #9aa4b2;")
            toggle.setCursor(Qt.PointingHandCursor)

            def show_commands(on: bool) -> None:
                command.setVisible(on)
                toggle.setText(("▾ " if on else "▸ ") + f"{channel} 没起来时在终端执行：")
                QTimer.singleShot(0, self.fit_page)
            toggle.clicked.connect(lambda: show_commands(not command.isVisible()))
            self.show_can_commands = show_commands
            column.addWidget(toggle)
            column.addWidget(command)
        layout.addWidget(box)

        # --- run settings
        box = QGroupBox("运行参数")
        row = QHBoxLayout(box)
        self.table_z = QDoubleSpinBox(decimals=3, minimum=-0.5, maximum=0.5, singleStep=0.01, suffix=" m")
        self.table_z.setValue(float(cfg.table_z))
        self.table_z.setToolTip("碰撞检查用的桌面高度（URDF 基坐标系 z）")
        self.rate = QSpinBox(minimum=10, maximum=2000, value=int(cfg.rate), suffix=" Hz")
        self.rate.setToolTip("发送指令的频率")
        for title, widget in (("桌面高度", self.table_z), ("指令频率", self.rate)):
            row.addWidget(QLabel(title))
            row.addWidget(widget, 1)
        layout.addWidget(box)

        # --- torque calibration
        box = QGroupBox("力矩标定（反馈 / 指令）")
        grid = QGridLayout(box)
        self.gain = []
        default_gain = self.robot.torque_gain()
        columns = 3
        for j in range(self.dof):
            spin = QDoubleSpinBox(decimals=3, minimum=0.5, maximum=1.5, singleStep=0.005, value=default_gain[j])
            grid.addWidget(QLabel(f"J{j + 1}"), j // columns, (j % columns) * 2)
            grid.addWidget(spin, j // columns, (j % columns) * 2 + 1)
            self.gain.append(spin)
        for c in range(columns):
            grid.setColumnStretch(c * 2 + 1, 1)
        self.gain_source = label("")
        grid.addWidget(self.gain_source, (self.dof + columns - 1) // columns, 0, 1, columns * 2)
        if cfg.torque_gain is not None:
            self.show_gain_source("配置字段 torque_gain")
        elif self.robot.is_calibrated:
            self.show_gain_source(self.robot.calibration_fpath)
        else:
            self.show_gain_source(None)
        layout.addWidget(box)

    def show_gain_source(self, source) -> None:
        if source is None:
            text = "尚未标定（全部取 1）。第 1 步静态核对后点“采用测得的力矩标定”。"
        elif isinstance(source, Path):
            text = f"来自标定文件 {source.name}"
            self.gain_source.setToolTip(str(source))
        else:
            text = f"来自{source}"
        self.gain_source.setText(text + "  辨识时测量力矩先除以它。")

    def page_static(self, layout: QVBoxLayout) -> None:
        layout.addWidget(label("<h3>1 静态核对</h3>", False))
        layout.addWidget(label("在静止位姿测保持力矩。先生成训练集，再生成验证集。点击列表在右侧显示。"))
        # Two rows, each in its own colour (also used for the name field below): training poses
        # are used for identification, validation poses are only scored.
        self.pose_count, self.pose_seed, self.generate_buttons = {}, {}, {}
        rows = QVBoxLayout()
        rows.setSpacing(4)
        for purpose, title, count, tip in (
                ("train", "静态训练", 50, "参与辨识"),
                ("val", "静态验证", 20, "另一批位姿，不参与辨识，只用于第 2 步的验收")):
            color = PURPOSE_COLORS[purpose]
            frame = QFrame()
            frame.setObjectName("purpose")
            r, g, b = (int(color[i:i + 2], 16) for i in (1, 3, 5))
            frame.setStyleSheet(f"QFrame#purpose {{ background: rgba({r}, {g}, {b}, 40); border-left: 4px solid {color};"
                                " border-radius: 4px; }")
            frame.setToolTip(tip)
            line = QHBoxLayout(frame)
            line.setContentsMargins(8, 4, 6, 4)
            name = QLabel(title)
            name.setStyleSheet(f"color: {color}; font-weight: 700; background: transparent;")
            name.setMinimumWidth(64)
            hint = QLabel(f"（{tip}）" if purpose == "train" else "（只用于评估）")
            hint.setStyleSheet("color: #8f8e85; background: transparent;")
            self.pose_count[purpose] = QSpinBox(minimum=3, maximum=200, value=count)
            self.pose_seed[purpose] = QSpinBox(minimum=0, maximum=9999)
            generate = QPushButton("生成")
            generate.clicked.connect(lambda _=False, p=purpose: self.generate_static_poses(p))
            generate.setStyleSheet(f"QPushButton {{ background: {color}; color: black; font-weight: 600;"
                                   " padding: 4px 16px; } QPushButton:disabled { background: #555; }")
            self.generate_buttons[purpose] = generate
            for widget in (name, hint):
                line.addWidget(widget)
            line.addStretch(1)
            for text, widget in (("位姿数", self.pose_count[purpose]), ("种子", self.pose_seed[purpose])):
                caption = QLabel(text)
                caption.setStyleSheet("background: transparent;")
                line.addWidget(caption)
                line.addWidget(widget)
            line.addWidget(generate)
            rows.addWidget(frame)
        row = rows
        layout.addLayout(row)
        self.static_poses = np.zeros((0, self.dof))
        self.generated_settings = None
        self.static_purpose = "train"
        self.focus_table = QTableWidget(self.dof, 5)
        self.focus_table.setHorizontalHeaderLabels(["重力灵敏度", "采样", "重点下限 [°]", "重点上限 [°]", "重点比例 [%]"])
        self.focus_table.setVerticalHeaderLabels([f"J{j + 1}" for j in range(self.dof)])
        self.focus_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.focus_table.setMinimumHeight(self.focus_table.horizontalHeader().height() + self.dof * 26 + 6)
        self.focus_table.verticalHeader().setDefaultSectionSize(26)
        self.focus_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.focus_table.setToolTip("灵敏度 = 转动该关节时各关节重力力矩的变化 [N·m/rad]，由模型自动算出。"
                                    "“采样”列默认按灵敏度判断（转动不改变重力的关节固定在休息位），可手动改。"
                                    "重点比例 > 0 时，该比例的位姿让此关节落在重点区间内（只对采样的关节有效）。")
        layout.addWidget(self.focus_table)
        self.stale_hint = QLabel("⚠ 采样/重点设置已改动，下面的位姿还是按旧设置生成的（新采样的关节都在休息位）。请点“重新生成”。")
        self.stale_hint.setWordWrap(True)
        self.stale_hint.setStyleSheet("color: #ffffff; background: #8a3a12; padding: 6px; border-radius: 4px;")
        self.stale_hint.setVisible(False)
        layout.addWidget(self.stale_hint)
        self.fill_focus_table()
        self.pose_list = QListWidget()
        self.pose_list.setFont(QFont("monospace", 9))
        self.pose_list.setMinimumHeight(80)
        self.pose_list.currentRowChanged.connect(self.show_static_pose)
        layout.addWidget(self.pose_list)
        row = QHBoxLayout()
        self.pose_summary = label("")
        row.addWidget(self.pose_summary, 1)
        row.addWidget(button("删除选中", self.delete_static_pose))
        layout.addLayout(row)
        self.static_model = QComboBox()
        self.static_model.setToolTip("对照模型：用它的重力做前馈，并与实测力矩比较")
        self.static_out = QLineEdit("")
        self.static_out.setPlaceholderText("点“生成”后自动填写（train_… / val_…）")
        self.static_out.setToolTip("保存为 static/<名称>.csv；名称含 val 的在辨识页默认勾在“静态验证”，其余默认勾在“静态训练”")
        self.static_out.textChanged.connect(lambda _: self.show_static_purpose())
        self.static_purpose_tag = QLabel("")
        row = QHBoxLayout()
        row.addWidget(QLabel("名称"))
        row.addWidget(self.static_out, 1)
        row.addWidget(self.static_purpose_tag)
        row.addWidget(QLabel("模型"))
        row.addWidget(self.static_model, 1)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(button("预检（不运动）", lambda: self.run_static(False)))
        row.addWidget(button("执行", lambda: self.run_static(True), primary=True))
        row.addStretch(1)
        self.adopt_gain = button("采用测得的力矩标定", self.apply_measured_gain)
        self.adopt_gain.setEnabled(False)
        row.addWidget(self.adopt_gain)
        layout.addLayout(row)
        self.gain_hint = label("")
        self.gain_hint.setVisible(False)
        layout.addWidget(self.gain_hint)
        self.measured_gain: list[float] | None = None
        self.focus_table.itemChanged.connect(lambda *_: self.check_stale())

    def page_identify(self, layout: QVBoxLayout) -> None:
        layout.addWidget(label("<h3>2 辨识质量、质心</h3>", False))
        layout.addWidget(label("勾选静态数据：训练集用于辨识，验证集只用于验收（同一份数据不要两边都勾）。"
                               "辨识前测量力矩会除以第 0 步的力矩标定，导出的 URDF 是真实力矩单位。"))
        self.lists = data_columns(layout, (("static", "静态训练", PURPOSE_COLORS["train"]),
                                           ("holdout", "静态验证", PURPOSE_COLORS["val"])))
        form = QFormLayout()
        self.version = QLineEdit(self.results.next_version())
        form.addRow("版本名", self.version)
        self.drift_factor = QDoubleSpinBox(minimum=1, maximum=20, value=5, decimals=0)
        self.drift_factor.setPrefix("× ")
        self.drift_factor.setToolTip("静态训练里 drift_ 开头的文件（重力补偿漂移分析生成的补点）的权重倍数。"
                                     "1 = 和随机位姿权重相同")
        form.addRow("漂移补点权重", self.drift_factor)
        layout.addLayout(form)
        row = QHBoxLayout()
        row.addWidget(button("开始辨识", self.run_identify, primary=True))
        row.addWidget(button("刷新文件", self.refresh_files))
        row.addStretch(1)
        layout.addLayout(row)
        self.identify_summary = label("")
        layout.addWidget(self.identify_summary)

    def page_gravity(self, layout: QVBoxLayout) -> None:
        layout.addWidget(label("<h3>3 重力补偿</h3>", False))
        layout.addWidget(label("free：只加重力前馈，手可拖动，松手应停住（用来检验模型）；"
                               "lock：推动时跟随，不推时以刚度 kp 保持位置。曲线中灰色为实测反馈，蓝色为发送的模型力矩。"))
        form = QFormLayout()
        self.gravity_model = QComboBox()
        form.addRow("模型", self.gravity_model)
        self.gravity_payload_hint = QLabel("")
        self.gravity_payload_hint.setWordWrap(True)
        self.gravity_payload_hint.setStyleSheet("color: #ffffff; background: #8a3a12; padding: 4px; border-radius: 4px;")
        self.gravity_payload_hint.setVisible(False)
        form.addRow("", self.gravity_payload_hint)
        self.gravity_mode = QComboBox()
        self.gravity_mode.addItems(["free", "lock"])
        self.gravity_mode.currentIndexChanged.connect(lambda _: self.update_gravity_controls())
        form.addRow("模式", self.gravity_mode)
        layout.addLayout(form)

        box = QGroupBox("手感（运行中拖动即时生效）")
        grid = QGridLayout(box)
        default_kd = float(np.mean(self.robot.per_joint(self.robot.config.free_kd, "free_kd")))
        self.feel = {}
        specs = (("kd", "阻尼 kd", 0.0, 3.0, 0.05, default_kd, "关节阻尼 [N·m·s/rad]"),
                 ("lock_kp", "锁定刚度 kp", 0.0, 30.0, 0.5, 8.0, "lock 模式下保持位置的刚度 [N·m/rad]"))
        for row, (key, title, lo, hi, step, value, tip) in enumerate(specs):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, int(round((hi - lo) / step)))
            spin = QDoubleSpinBox(minimum=lo, maximum=hi, singleStep=step, decimals=2, value=value)
            slider.setValue(int(round((value - lo) / step)))
            slider.valueChanged.connect(lambda v, spin=spin, lo=lo, step=step: spin.setValue(lo + v * step))
            spin.valueChanged.connect(lambda v, slider=slider, lo=lo, step=step: (
                slider.blockSignals(True), slider.setValue(int(round((v - lo) / step))), slider.blockSignals(False)))
            spin.valueChanged.connect(lambda v, key=key: self.send_feel(key, v))
            name = QLabel(title)
            for widget in (name, slider, spin):
                widget.setToolTip(tip)
            grid.addWidget(name, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            self.feel[key] = (name, slider, spin)
        grid.setColumnStretch(1, 1)
        # Feedforward scale per joint: e.g. J2 1.04 when the arm sinks where J2 carries a lot.
        name = QLabel("前馈倍率")
        tip = ("每个关节的重力前馈乘以它（运行中修改即时生效）。伸直时往下掉可把 J2 调到 1.02–1.06，"
               "往上漂就调小；只是检验用，不改模型。记录和漂移分析用的是调过的前馈")
        name.setToolTip(tip)
        grid.addWidget(name, len(specs), 0)
        row = QHBoxLayout()
        self.ff_scale = []
        for j in range(self.dof):
            spin = QDoubleSpinBox(minimum=0.8, maximum=1.2, singleStep=0.01, decimals=2, value=1.0)
            spin.setToolTip(f"J{j + 1}：{tip}")
            spin.valueChanged.connect(lambda v, j=j: self.send_feel(f"scale{j + 1}", v))
            row.addWidget(QLabel(f"J{j + 1}"))
            row.addWidget(spin, 1)
            self.ff_scale.append(spin)
        grid.addLayout(row, len(specs), 1, 1, 2)
        self.offset_file = label("")
        grid.addWidget(self.offset_file, len(specs) + 1, 0, 1, 3)
        layout.addWidget(box)
        self.gravity_model.currentIndexChanged.connect(lambda _: self.update_gravity_controls())
        self.drift_box = QGroupBox("漂移分析（free 模式结束后自动进行）")
        column = QVBoxLayout(self.drift_box)
        self.drift_text = label("free 模式下，把机械臂推到各处松手：自己转动的关节说明那里模型不准。"
                                "结束后自动找出这些地方，并生成补数据用的静态位姿。")
        column.addWidget(self.drift_text)
        self.drift_load = button("载入建议位姿 → 第 1 步静态核对", self.load_drift_poses, primary=True)
        self.drift_load.setVisible(False)
        column.addWidget(self.drift_load)
        self.drift_poses = self.drift_payload = None
        row = QHBoxLayout()
        row.addWidget(button("启动重力补偿", self.run_gravity, primary=True))
        row.addStretch(1)
        layout.addLayout(row)
        layout.addWidget(self.drift_box)

    def page_package(self, layout: QVBoxLayout) -> None:
        layout.addWidget(label("<h3>5 导出完整 URDF</h3>", False))
        layout.addWidget(label("以原始 URDF 为模板，导出一个完整、可单独拷走的包：<code>urdf/</code> + <code>meshes/</code>，"
                               "连杆、关节、外观和碰撞都和原始 URDF 一样，惯性参数换成辨识结果。"
                               "选本体模型导出不带负载的（去掉末端以下的连杆）；选带负载的模型（第 4 步）导出带负载的："
                               "保留模板里负载的连杆、网格和关节，质量、质心改成辨识值。"))
        form = QFormLayout()
        self.package_model = QComboBox()
        self.package_model.currentIndexChanged.connect(lambda _: self.update_package_info())
        form.addRow("模型", self.package_model)
        self.package_info = label("")
        form.addRow("", self.package_info)
        layout.addLayout(form)
        row = QHBoxLayout()
        row.addWidget(button("导出", self.run_package, primary=True))
        row.addStretch(1)
        layout.addLayout(row)
        self.package_summary = label("")
        layout.addWidget(self.package_summary)

    def package_target(self) -> tuple[str, Path, bool] | None:
        """(package name, folder, with payload) for the selected model."""
        from ..package import find_payload

        model = self.package_model.currentData()
        if not model:
            return None
        model, payload = Path(model), find_payload(model)
        template = Path(self.robot.config.urdf_path)
        tag = model.stem.split("_identified_")[-1] if "_identified_" in model.stem else model.stem
        name = f"{template.stem}_{tag}" + (f"_{payload.get('name', 'payload')}" if payload else "")
        return name, self.results.export_dir / name, payload is not None

    def update_package_info(self) -> None:
        target = self.package_target()
        if target is None:
            self.package_info.setText("还没有辨识出的模型（第 2 步，或第 4 步的负载模型）。")
            return
        name, folder, with_payload = target
        tip = self.robot.config.tip_link
        self.package_info.setText(("带负载：保留模板中负载的连杆和网格，" if with_payload else
                                   f"不带负载：去掉 {tip} 以下的连杆，") + f"导出到 {short_path(folder)}/")

    def run_package(self) -> None:
        target = self.package_target()
        if target is None:
            return
        name, folder, _ = target
        if folder.exists() and QMessageBox.question(self, "覆盖", f"{short_path(folder)} 已存在，覆盖？") != QMessageBox.Yes:
            return

        def done(ok: bool) -> None:
            text = "\n".join(self.run_lines)
            lines = [line.strip() for line in text.splitlines() if line.startswith(("已导出", "  "))]
            self.package_summary.setText(("✓ " + "<br>".join(lines)) if ok and lines else "⚠ 导出失败，见日志")
            self.package_summary.setTextFormat(Qt.RichText)
        self.start(self.cli("package", *self.robot_args, "--model", self.package_model.currentData(),
                            "--out", str(folder)), on_done=done)

    def page_payload(self, layout: QVBoxLayout) -> None:
        layout.addWidget(label("<h3>4 负载辨识（夹爪等）</h3>", False))
        layout.addWidget(label("装上负载后采静态数据，只辨识负载的质量和质心（本体参数不变）。碰撞检查包含负载外形（3D 中橙色）。"))
        row = QHBoxLayout()
        row.addWidget(QLabel("本体"))
        self.payload_base = QComboBox()
        self.payload_base.setToolTip("已辨识的机械臂本体（不含负载）")
        row.addWidget(self.payload_base, 1)
        row.addWidget(QLabel("名称"))
        self.payload_name = QLineEdit("gripper")
        self.payload_name.setMaximumWidth(90)
        self.payload_name.textChanged.connect(lambda _: self.on_payload_name())
        row.addWidget(self.payload_name)
        layout.addLayout(row)
        self.payload_package = label("")
        self.payload_package.setToolTip("装负载后采的数据和负载模型都存在这个包里，与机械臂本体的数据分开")

        # --- static data with the payload mounted (collision checks include it)
        from ..visual import has_tool_links

        self.tool_from_urdf = has_tool_links(self.dynamics.robot, self.robot.config.tip_link)
        row = QHBoxLayout()
        if self.tool_from_urdf:
            row.addWidget(label(f"负载外形：URDF 中 {self.robot.config.tip_link} 以下的部件"))
            row.addWidget(self.payload_package)
            self.tool_length = self.tool_radius = None
        else:
            row.addWidget(QLabel("负载外形：圆柱 长"))
            self.tool_length = QDoubleSpinBox(minimum=0.0, maximum=0.5, value=0.15, singleStep=0.01, suffix=" m")
            row.addWidget(self.tool_length)
            row.addWidget(QLabel("半径"))
            self.tool_radius = QDoubleSpinBox(minimum=0.005, maximum=0.2, value=0.04, singleStep=0.005, suffix=" m")
            row.addWidget(self.tool_radius)
            row.addWidget(self.payload_package, 1)
        layout.addLayout(row)
        self.payload_count, self.payload_seed = {}, {}
        for purpose, title, count in (("train", "负载训练", 40), ("val", "负载验证", 20)):
            color = PURPOSE_COLORS[purpose]
            frame = QFrame()
            frame.setObjectName("purpose")
            r, g, b = (int(color[i:i + 2], 16) for i in (1, 3, 5))
            frame.setStyleSheet(f"QFrame#purpose {{ background: rgba({r}, {g}, {b}, 40); border-left: 4px solid {color};"
                                " border-radius: 4px; }")
            line = QHBoxLayout(frame)
            line.setContentsMargins(8, 4, 6, 4)
            name = QLabel(title)
            name.setStyleSheet(f"color: {color}; font-weight: 700; background: transparent;")
            line.addWidget(name)
            line.addStretch(1)
            self.payload_count[purpose] = QSpinBox(minimum=4, maximum=200, value=count)
            self.payload_seed[purpose] = QSpinBox(minimum=0, maximum=9999)
            for text, widget in (("位姿数", self.payload_count[purpose]), ("种子", self.payload_seed[purpose])):
                caption = QLabel(text)
                caption.setStyleSheet("background: transparent;")
                line.addWidget(caption)
                line.addWidget(widget)
            generate = QPushButton("生成")
            generate.clicked.connect(lambda _=False, p=purpose: self.generate_payload_poses(p))
            generate.setStyleSheet(f"QPushButton {{ background: {color}; color: black; font-weight: 600;"
                                   " padding: 4px 16px; } QPushButton:disabled { background: #555; }")
            line.addWidget(generate)
            layout.addWidget(frame)
        self.payload_pose_list = QListWidget()
        self.payload_pose_list.setFont(QFont("monospace", 9))
        self.payload_pose_list.setMaximumHeight(90)
        self.payload_pose_list.currentRowChanged.connect(self.show_payload_pose)
        layout.addWidget(self.payload_pose_list)
        row = QHBoxLayout()
        self.payload_pose_name = label("", False)
        row.addWidget(self.payload_pose_name, 1)
        row.addWidget(button("预检（不运动）", lambda: self.run_payload_static(False)))
        row.addWidget(button("执行", lambda: self.run_payload_static(True), primary=True))
        layout.addLayout(row)

        self.payload_poses = np.zeros((0, self.dof))
        self.on_payload_name()

        self.payload_lists = data_columns(layout, (("train", "负载训练数据", PURPOSE_COLORS["train"]),
                                                   ("val", "负载验证数据（只用于评估）", PURPOSE_COLORS["val"])))
        row = QHBoxLayout()
        row.addWidget(button("辨识负载", self.run_payload, primary=True))
        row.addStretch(1)
        layout.addLayout(row)
        self.payload_summary = label("")
        layout.addWidget(self.payload_summary)

    def tool_dynamics(self) -> Dynamics:
        """Arm model whose collision checks include the mounted payload."""
        dynamics = Dynamics.for_robot(self.robot)
        if self.tool_from_urdf:
            return dynamics.with_tool(self.robot)
        return dynamics.with_tool(self.robot, self.tool_length.value(), self.tool_radius.value())

    def show_tool(self, on: bool) -> None:
        if self.view is None:
            return
        if not on:
            self.view.set_tool(None)
            return
        from ..visual import load_visuals

        geometry = self.tool_dynamics().collision
        arm = {v.link for v in load_visuals(geometry.robot, geometry.tip_link)}
        self.view.set_tool([v for v in load_visuals(geometry.robot, geometry.tip_link, tool=geometry.tool)
                            if v.link not in arm])

    def payload_label(self) -> str:
        from ..results import RESERVED

        name = re.sub(r"[^\w.-]", "_", self.payload_name.text().strip() or "payload")
        return name + "_payload" if name in RESERVED else name

    def payload_feedforward(self) -> list[str]:
        """Feedforward model while collecting payload data: the payload identified on the selected arm
        version if there is one, else that arm plus the CAD payload of the robot's URDF. Without the
        payload in the feedforward only kp holds it, and heavy poses exceed the tracking limit."""
        base = self.payload_base.currentData()
        if not base:
            return ["--payload-cad"] if self.tool_from_urdf else []
        identified = self.payload_results.model_dir(Path(base).parent.name)
        models = sorted(identified.glob("*.urdf")) if identified.is_dir() else []
        if models:
            self.log_line(f"[负载] 前馈用已辨识的负载模型 {models[0].name}")
            return ["--urdf", str(models[0])]
        self.log_line(f"[负载] 前馈用 {Path(base).name} + URDF 中的 CAD 负载（还没有辨识出的负载模型）")
        return ["--urdf", base] + (["--payload-cad"] if self.tool_from_urdf else [])

    @property
    def payload_results(self) -> Results:
        """Package of the payload named on step 4: results/<robot>/<name>/."""
        return self.results.payload(self.payload_label())

    def on_payload_name(self) -> None:
        self.update_payload_seeds()
        self.fill_payload_lists(guess=True)
        self.payload_package.setText("数据包：" + short_path(self.payload_results.root))

    def update_payload_seeds(self) -> None:
        names = set(self.payload_results.static_names())
        for purpose, start in (("train", 200), ("val", 300)):
            seed = start
            while any(name.endswith(f"_s{seed}") for name in names):
                seed += 1
            self.payload_seed[purpose].setValue(seed)

    def generate_payload_poses(self, purpose: str) -> None:
        from ..runtime.static_check import generate_poses, payload_sample

        n, seed = self.payload_count[purpose].value(), self.payload_seed[purpose].value()
        robot, table_z = self.robot, self.table_z.value()
        dynamics = self.tool_dynamics()
        sample = payload_sample(robot.dof)
        name = f"{purpose}_rand{n}_s{seed}"
        self.payload_pose_name.setText(f"生成中：{name}（碰撞检查含负载）…")

        self.run_job(lambda: generate_poses(robot, dynamics, n, seed, table_z=table_z, sample=sample),
                     lambda poses: self.show_payload_poses(poses, name, purpose))

    def show_payload_poses(self, poses, name: str, purpose: str) -> None:
        self.payload_poses = np.asarray(poses).reshape(-1, self.dof)
        self.payload_pose_list.clear()
        for i, pose in enumerate(np.degrees(self.payload_poses), start=1):
            self.payload_pose_list.addItem(f"{i:3d}  " + " ".join(f"{v:+6.1f}" for v in pose))
        color = PURPOSE_COLORS[purpose]
        self.payload_pose_name.setText(f"<span style='color:{color}; font-weight:700'>{name}</span>"
                                       f"：{len(self.payload_poses)} 个位姿，约 {len(self.payload_poses) * 9 / 60:.0f} 分钟")
        self.payload_pose_name.setProperty("name", name)
        if len(self.payload_poses):
            self.payload_pose_list.setCurrentRow(0)

    def show_payload_pose(self, row: int) -> None:
        if self.view is not None and 0 <= row < len(self.payload_poses) and not self.runner.busy:
            self.view.set_pose(self.payload_poses[row])

    def run_payload_static(self, execute: bool) -> None:
        from ..runtime.static_check import save_pose_file

        name = self.payload_pose_name.property("name")
        if not len(self.payload_poses) or not name:
            QMessageBox.information(self, "没有位姿", "先点“负载训练”或“负载验证”的“生成”。")
            return
        if execute and not self.confirm_hardware():
            return
        out, pose_file = self.payload_results.static(name), self.payload_results.poses(name)
        if execute and out.exists() and QMessageBox.question(self, "覆盖", f"{out.name} 已存在，覆盖？") != QMessageBox.Yes:
            return
        save_pose_file(pose_file, self.payload_poses, source="rebot_taccap_identification studio (payload)")
        args = ["--poses", str(pose_file), "--out", str(out), "--balance-load",
                *self.common_args(), *self.payload_feedforward()]
        args += ["--with-tool"] if self.tool_from_urdf else \
            ["--tool-length", f"{self.tool_length.value():g}", "--tool-radius", f"{self.tool_radius.value():g}"]

        def done(ok: bool) -> None:
            if execute:
                self.update_payload_seeds()
            self.fill_payload_lists(guess=True)
        self.start(self.robot_cmd("static", *args + (["--execute"] if execute else [])), on_done=done, live=execute)

    def fill_payload_lists(self, guess: bool = False) -> None:
        """Data of this payload's package only (taken with the payload mounted); training names are
        preselected in the training list, validation names (containing "val") in the validation list."""
        if not hasattr(self, "payload_lists"):
            return
        results = self.payload_results
        names = results.static_names()
        for key, widget in self.payload_lists.items():
            checked = {widget.item(i).text() for i in range(widget.count())
                       if widget.item(i).checkState() == Qt.Checked}
            widget.clear()
            for item_name in names:
                item = QListWidgetItem(item_name)
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setToolTip(item_name)
                on = (is_static_val(item_name) == (key == "val")) if guess or not checked else item_name in checked
                item.setCheckState(Qt.Checked if on else Qt.Unchecked)
                widget.addItem(item)
            fit_rows(widget)
        QTimer.singleShot(0, self.fit_page)

    def payload_checked(self, key: str) -> list[str]:
        widget = self.payload_lists[key]
        return [widget.item(i).text() for i in range(widget.count())
                if widget.item(i).checkState() == Qt.Checked]

    # ------------------------------------------------------------- helpers
    def log_line(self, text: str) -> None:
        self.log.appendPlainText(text)
        self.run_lines.append(text)          # the log widget drops old lines; results are parsed from this
        # Static check progress "[3/30] [0, 45, 80, ...]°": select that pose in the list.
        match = re.match(r"\[(\d+)/\d+\] \[([-\d., ]+)\]°", text)
        if match and len(self.static_poses):
            pose = np.array([float(v) for v in match.group(2).split(",")])
            row = int(np.argmin(np.abs(np.degrees(self.static_poses) - pose).max(axis=1)))
            self.pose_list.blockSignals(True)
            self.pose_list.setCurrentRow(row)
            self.pose_list.blockSignals(False)
            self.coverage.select(row)

    STOP_KEYS = (Qt.Key_Escape, Qt.Key_Space)

    def eventFilter(self, obj, event) -> bool:
        if (event.type() == QEvent.KeyPress and event.key() in self.STOP_KEYS and self.runner.busy
                and not event.isAutoRepeat()):
            self.runner.stop()
            return True                                  # swallow it while something is running
        return False

    def update_stop_button(self) -> None:
        busy, stops = self.runner.busy, self.runner.stops
        self.stop_button.setEnabled(busy)
        if not busy:
            self.stop_button.setText("没有运行中的任务　　（运行时：Esc / 空格 = 停止）")
        elif stops == 0:
            self.stop_button.setText("■  停止（安全停止）　　Esc / 空格")
        elif stops == 1:
            self.stop_button.setText("■  正在安全停止…　再按一次：立即失能")
        else:
            self.stop_button.setText("■  已失能　再按一次：强制结束进程")

    def set_busy(self, busy: bool) -> None:
        self.update_stop_button()
        self.statusBar().showMessage("运行中…（Esc / 空格 / 顶部红色按钮 = 安全停止）" if busy else "就绪")

    def common_args(self) -> list[str]:
        args = [*self.robot_args, "--rate", str(self.rate.value()),
                "--table-z", f"{self.table_z.value():g}", "--stream"]
        return args

    def gain_values(self) -> list[str]:
        return [f"{s.value():g}" for s in self.gain]

    def robot_cmd(self, command: str, *args: str) -> list[str]:
        return [sys.executable, "-m", "rebot_taccap_identification.cli", command, *args]

    def cli(self, *args: str) -> list[str]:
        return [sys.executable, "-m", "rebot_taccap_identification.cli", *args]

    @property
    def results(self) -> Results:
        return Results.for_robot(self.robot)

    def confirm_hardware(self) -> bool:
        if self.confirmed_hardware:
            return True
        answer = QMessageBox.question(
            self, "真机运行", "机械臂将会运动。\n\n请确认：周围无人无障碍、急停在手边、末端负载与所选步骤和模型一致。\n"
                           "运行中按 Esc 或红色按钮安全停止，再按一次立即失能。")
        self.confirmed_hardware = answer == QMessageBox.Yes
        return self.confirmed_hardware

    def start(self, *commands: list[str], on_done=None, live: bool = False) -> None:
        if self.runner.busy:
            QMessageBox.information(self, "忙", "有任务在运行，请先停止或等待完成。")
            return
        self.on_done = on_done
        self.run_lines = []
        if live:
            self.live.reset()
            self.plots.setCurrentWidget(self.live)
        self.runner.run(*commands)

    def on_finished(self, ok: bool) -> None:
        self.set_busy(False)
        callback, self.on_done = self.on_done, None
        if callback is not None:
            callback(ok)
        self.refresh_files()

    def on_state(self, state: dict) -> None:
        q = np.asarray(state["q"])
        if self.view is not None:
            self.view.set_pose(q)
        self.live.push(state, time.monotonic())
        self.pose_label.setText("q [°] " + "  ".join(f"{v:+6.1f}" for v in np.degrees(q))
                                + f"   温度 {max(state['temp']):.0f}°C")

    def on_prompt(self, text: str) -> None:
        QMessageBox.warning(self, "需要扶住机械臂", text.strip() + "\n\n扶住机械臂后点“确定”松开。")
        self.runner.answer("\n")

    # ---------------------------------------------------------------- files
    def models(self) -> list[tuple[str, str]]:
        """Original URDF and every identified model, newest first."""
        items = [("原始 URDF", str(self.robot.config.urdf_path))]
        for version, path in self.results.model_urdfs():
            items.append((f"辨识 {version}", str(path)))
        for version, path in self.results.payload_models():
            items.append((f"辨识 {version}", str(path)))
        return items

    def refresh_files(self) -> None:
        models = self.models()
        if hasattr(self, "package_model"):
            current = self.package_model.currentData()
            self.package_model.clear()
            for name, path in models[1:]:              # identified models only
                self.package_model.addItem(name, path)
            index = self.package_model.findData(current) if current else -1
            self.package_model.setCurrentIndex(max(index, 0))
            self.update_package_info()
        if hasattr(self, "payload_base"):
            current = self.payload_base.currentData()
            self.payload_base.clear()
            for version, path in self.results.model_urdfs():
                self.payload_base.addItem(f"辨识 {version}", str(path))
            index = self.payload_base.findData(current) if current else -1
            self.payload_base.setCurrentIndex(max(index, 0))
            self.fill_payload_lists()
        exported = [(f"导出 {name}", str(path)) for name, path in self.results.export_urdfs()]
        for combo, default in ((self.static_model, 1), (self.gravity_model, 1)):
            current = combo.currentData()
            combo.clear()
            for name, path in models + (exported if combo is self.gravity_model else []):
                combo.addItem(name, path)
            index = combo.findData(current) if current else -1
            combo.setCurrentIndex(index if index >= 0 else min(default, combo.count() - 1))
        results = self.results
        statics = results.static_names()
        # First fill: the selection of the latest model, else a guess from the names.
        versions = results.versions()
        last = results.load_inputs(versions[-1]) if versions else {}
        guess = {"static": lambda n: not is_static_val(n), "holdout": is_static_val}
        fresh = getattr(self, "_listed_root", None) != results.root
        self._listed_root = results.root
        for key in ("static", "holdout"):
            widget = self.lists[key]
            checked = {widget.item(i).text() for i in range(widget.count())
                       if widget.item(i).checkState() == Qt.Checked}
            listed = {widget.item(i).text() for i in range(widget.count())}
            widget.clear()
            for name in statics:
                item = QListWidgetItem(name)
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setToolTip(name)
                if not fresh and name not in listed:          # new since the last refresh
                    on = guess[key](name)
                elif not fresh:
                    on = name in checked
                elif last:
                    on = name in last.get(key, [])
                else:
                    on = guess[key](name)
                item.setCheckState(Qt.Checked if on else Qt.Unchecked)
                widget.addItem(item)
            fit_rows(widget)
        QTimer.singleShot(0, self.fit_page)
        if not self.runner.busy:
            self.version.setText(results.next_version())

    def is_payload_model(self, model: str | None) -> bool:
        """A model with a payload: from a payload package (results/<robot>/<payload>/models/...), or a
        step 5 package that keeps links below the tip link."""
        if not model:
            return False
        if self.results.is_export(model):
            from ..package import has_payload

            return has_payload(model, self.robot.config.tip_link)
        return self.results.payload_of(model) is not None

    def offset_yaml(self) -> Path | None:
        """Joint offsets exported with the selected model (identified models only)."""
        from ..export import offsets_path

        model = self.gravity_model.currentData()
        path = offsets_path(model) if model else None
        return path if path is not None and path.exists() else None

    def update_gravity_controls(self) -> None:
        if self.pages.currentIndex() == 3 and hasattr(self, "payload_poses"):
            self.show_tool(self.is_payload_model(self.gravity_model.currentData()))
        # An arm-only model with a payload mounted lets the arm fall: remind when payload models exist.
        payloads = [name for name, path in self.models() if self.is_payload_model(path)]
        arm_only = not self.is_payload_model(self.gravity_model.currentData())
        if payloads and arm_only:
            self.gravity_payload_hint.setText(f"⚠ 这个模型不含负载。如果末端装着负载（夹爪），请选 {'、'.join(payloads)}，"
                                              "否则负载的重量没有补偿，机械臂会下坠，碰撞检查也不包含负载。")
        self.gravity_payload_hint.setVisible(bool(payloads and arm_only))
        offset = self.offset_yaml()
        lock = self.gravity_mode.currentText() == "lock"
        for widget in self.feel["lock_kp"]:
            widget.setEnabled(lock)
        self.offset_file.setText(f"关节力矩偏置：{offset.name}（自动加上）" if offset is not None
                                 else "这个模型旁边没有关节力矩偏置文件（<模型>_offset.yaml）。")

    def show_drift(self, result: dict) -> None:
        if not result["n_drifts"]:
            self.drift_text.setText("✓ 没有发现自己转动的关节（只针对这次走过的区域）。")
            return
        lines = "<br>".join(result["summary"])
        poses = result.get("poses")
        self.drift_text.setText(f"⚠ 发现 {result['n_drifts']} 处自己转动（详情见日志）：<br>{lines}<br>"
                                + ((f"已生成 {result['n_poses']} 个补数据的静态位姿（装着负载采）。下一步：载入 → 第 4 步执行负载静态 → "
                                    "在负载训练数据里勾上它，重新辨识负载。" if poses and self.results.payload_of(poses) else
                                    f"已生成 {result['n_poses']} 个补数据的静态位姿。下一步：载入 → 第 1 步执行静态核对 → "
                                    "第 2 步把它和原来的静态数据一起勾为“静态训练”，辨识下一个版本。") if poses else
                                   "但这些地方生成不了无碰撞的静态位姿。"))
        self.drift_text.setTextFormat(Qt.RichText)
        self.drift_poses = poses
        self.drift_payload = self.results.payload_of(poses) if poses else None
        self.drift_load.setText(f"载入建议位姿 → 第 4 步负载静态（{self.drift_payload}）" if self.drift_payload
                                else "载入建议位姿 → 第 1 步静态核对")
        self.drift_load.setVisible(bool(poses))
        QTimer.singleShot(0, self.fit_page)

    def load_drift_poses(self) -> None:
        from ..runtime.static_check import load_pose_file

        if not self.drift_poses or not Path(self.drift_poses).exists():
            return
        name = Path(self.drift_poses).name.removesuffix(".poses.json")
        poses = load_pose_file(self.drift_poses, self.dof)
        if self.drift_payload:                      # taken with the payload mounted: step 4, its package
            self.payload_name.setText(self.drift_payload)
            self.on_payload_name()
            self.show_payload_poses(poses, name, "train")
            self.steps.setCurrentRow(4)
            self.log_line(f"[负载静态] 已载入 {len(poses)} 个补数据位姿（{name}，碰撞检查含负载），先预检再执行")
            return
        self.generated_settings = None              # not generated from the table: no stale warning
        self.set_static_poses(poses, name)
        self.steps.setCurrentRow(1)
        self.log_line(f"[静态核对] 已载入 {len(self.static_poses)} 个补数据位姿（{name}），先预检再执行")

    def send_feel(self, key: str, value: float) -> None:
        """Pass a feel setting to a running gravity compensation."""
        if self.runner.busy and getattr(self, "gravity_running", False):
            self.runner.answer(f"@set {key}={value:g}\n")

    def checked(self, key: str) -> list[str]:
        widget = self.lists[key]
        return [widget.item(i).text() for i in range(widget.count()) if widget.item(i).checkState() == Qt.Checked]

    # ---------------------------------------------------------------- steps
    def fit_page(self) -> None:
        """Give the page the height it needs (no scroll bar if possible); the log gets the rest."""
        scroll = self.pages.currentWidget()
        total = sum(self.center.sizes())
        if not total:
            return
        need = scroll.widget().sizeHint().height() + 4
        page = max(min(need, total - self.log.minimumHeight()), total // 3)
        self.center.setSizes([page, total - page])

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if hasattr(self, "center"):
            QTimer.singleShot(0, self.fit_page)

    def on_step(self, index: int) -> None:
        self.pages.setCurrentIndex(index)
        if hasattr(self, "payload_poses"):
            self.show_tool(index == 4 or (index == 3 and self.is_payload_model(self.gravity_model.currentData()))
                           or (index == 5 and self.is_payload_model(self.package_model.currentData())))
            if index == 4:
                self.show_payload_pose(self.payload_pose_list.currentRow())
        QTimer.singleShot(0, self.fit_page)
        self.plots.setCurrentWidget(self.coverage if index == 1 else self.live)
        if index == 1:
            self.show_static_pose(self.pose_list.currentRow())

    def ping(self) -> None:
        args = [*self.robot_args]
        self.connection.setText("连接中…")

        def done(ok: bool) -> None:
            text = "\n".join(self.run_lines)
            match = re.search(r"结果：(\d+)/(\d+)", text)
            if match and match.group(1) == match.group(2):
                self.connection.setText(f"✓ 已连接：{match.group(1)}/{match.group(2)} 个关节应答")
                self.connection.setStyleSheet("color: #4cbb6c; font-weight: 600;")
            else:
                got = f"{match.group(1)}/{match.group(2)} 个关节应答" if match else "通信失败"
                self.connection.setText(f"✗ {got}，见日志（检查电源、急停、通信线）")
                if hasattr(self, "show_can_commands"):
                    self.show_can_commands(True)
                self.connection.setStyleSheet("color: #ef6b69; font-weight: 600;")
        self.start(self.robot_cmd("ping", *args), on_done=done)

    # --- robot selection
    def fill_robot_list(self) -> None:
        self.robot_select.clear()
        classes = RobotConfig.get_known_choices()
        for name, error in available_robots():
            # Base configs that only make sense through a concrete arm are not listed.
            if error and any(c is not classes[name] and issubclass(c, classes[name]) for c in classes.values()):
                continue
            self.robot_select.addItem(name if not error else f"{name}　—　不可用", name)
            index = self.robot_select.count() - 1
            if error:
                self.robot_select.model().item(index).setEnabled(False)
                self.robot_select.setItemData(index, error, Qt.ToolTipRole)
            if name == self.robot_type:
                self.robot_select.setCurrentIndex(index)
        for module, error in IMPORT_ERRORS.items():
            self.robot_select.addItem(f"{module}　—　导入失败", None)
            self.robot_select.model().item(self.robot_select.count() - 1).setEnabled(False)
            self.robot_select.setItemData(self.robot_select.count() - 1, error, Qt.ToolTipRole)

    def on_robot_selected(self, index: int) -> None:
        name = self.robot_select.itemData(index)
        if name and name != self.robot_type:
            self.switch_robot(name, {"id": self.robot.id} if self.robot.id else {})

    def on_robot_id(self) -> None:
        robot_id = self.robot_id.text().strip() or None
        if robot_id != self.robot.id:
            fields = {k: v for k, v in self.robot_fields.items() if k != "id"}
            self.switch_robot(self.robot_type, {**fields, **({"id": robot_id} if robot_id else {})})

    def switch_robot(self, robot_type: str, fields: dict) -> None:
        """Restart the workbench for another arm (or another ``id``).

        Every page depends on the joint count and model, and pyqtgraph's OpenGL items
        cannot move to a new context, so the workbench restarts as a new process. Results are
        files, nothing is lost; switching is refused while a task runs.
        """
        if self.runner.busy:
            QMessageBox.information(self, "忙", "有任务在运行，请先停止。")
            self.fill_robot_list()
            self.robot_id.setText(self.robot.id or "")
            return
        try:
            make_robot(robot_type, **fields)          # validate before leaving the current arm
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "无法加载", f"{robot_type}\n\n{type(exc).__name__}: {exc}")
            self.fill_robot_list()
            return
        remember_last_robot(robot_type, fields)
        argv = [sys.executable, "-m", "rebot_taccap_identification.cli", "studio", *robot_args(robot_type, fields)]
        argv += ["--no-3d"] if self.view is None else []
        self.close()
        os.execv(sys.executable, argv)

    def set_static_poses(self, poses: np.ndarray, name: str) -> None:
        """Show poses (radians) in the list and the coverage plot."""
        self.static_poses = np.asarray(poses, dtype=float).reshape(-1, self.dof)
        gravity = self.dynamics.gravity(self.static_poses) if len(self.static_poses) else np.zeros((0, self.dof))
        self.pose_list.blockSignals(True)
        self.pose_list.clear()
        # Gravity torque of the (up to) three joints with the highest gravity sensitivity.
        shown = sorted(np.argsort(-self.sensitivity)[:3]) if hasattr(self, "sensitivity") else range(min(3, self.dof))
        for i, (pose, g) in enumerate(zip(np.degrees(self.static_poses), gravity), start=1):
            self.pose_list.addItem(f"{i:3d}  " + " ".join(f"{v:+6.1f}" for v in pose)
                                   + "   重力 " + " ".join(f"J{j + 1} {g[j]:+5.2f}" for j in shown))
        self.pose_list.blockSignals(False)
        self.coverage.set_poses(np.degrees(self.static_poses))
        n = len(self.static_poses)
        self.pose_summary.setText(f"{n} 个位姿，预计约 {n * 9 / 60:.0f} 分钟。"
                                  f"列：J1…J{self.dof} [°]，以及重力灵敏度最高的关节的模型重力力矩 [N·m]")
        self.static_out.setText(name)
        self.check_stale()
        if n:
            self.pose_list.setCurrentRow(0)

    def show_static_purpose(self) -> None:
        """Colour the name field and the run button like the purpose of the listed poses."""
        if not self.static_out.text().strip():
            self.static_out.setStyleSheet("")
            self.static_purpose_tag.setText("")
            self.static_purpose_tag.setStyleSheet("")
            return
        purpose = "val" if is_static_val(self.static_out.text()) else "train"
        color = PURPOSE_COLORS[purpose]
        self.static_out.setStyleSheet(f"QLineEdit {{ border: 2px solid {color}; border-radius: 3px; }}")
        self.static_purpose_tag.setText("静态验证" if purpose == "val" else "静态训练")
        self.static_purpose_tag.setStyleSheet(f"color: black; background: {color}; font-weight: 600;"
                                              " padding: 1px 8px; border-radius: 3px;")

    @staticmethod
    def static_name(purpose: str, n: int, seed: int) -> str:
        return f"{purpose}_rand{n}_s{seed}"

    def update_pose_seeds(self) -> None:
        """For each row (training / validation) a seed whose data does not exist yet (other poses than before);
        validation seeds start at 100, apart from training ones."""
        names = set(self.results.static_names())
        for purpose, start in (("train", 0), ("val", 100)):
            seed = start
            while any(name.endswith(f"_s{seed}") for name in names):
                seed += 1
            self.pose_seed[purpose].setValue(seed)

    def generate_static_poses(self, purpose: str = "train") -> None:
        from ..runtime.static_check import generate_poses

        n, seed, table_z = self.pose_count[purpose].value(), self.pose_seed[purpose].value(), self.table_z.value()
        self.static_out.setText(self.static_name(purpose, n, seed))      # shows the purpose at once
        focus = self.focus_specs()
        sample = self.sampled_joints()
        if not any(sample):
            QMessageBox.information(self, "没有采样关节", "至少把一个关节设为“采样”。")
            return
        dynamics, robot = self.dynamics, self.robot
        for widget in self.generate_buttons.values():
            widget.setEnabled(False)
        self.pose_summary.setText("生成中：碰撞筛选并排路线…")

        def finished(poses) -> None:
            for widget in self.generate_buttons.values():
                widget.setEnabled(True)
            self.static_purpose = purpose
            self.show_static_purpose()
            self.generated_settings = (tuple(sample), tuple(focus))
            self.set_static_poses(poses, self.static_name(purpose, n, seed))
            text = "，".join(f"J{j + 1} {np.degrees(lo):.0f}…{np.degrees(hi):.0f}° {f:.0%}" for j, lo, hi, f in focus)
            joints = ",".join(f"J{j + 1}" for j, on in enumerate(sample) if on)
            self.log_line(f"[位姿] 随机生成{'静态验证' if purpose == 'val' else '静态训练'}位姿 {len(poses)} 个（种子 {seed}，采样 {joints}，重点：{text or '无，均匀覆盖'}）")
        self.run_job(lambda: generate_poses(robot, dynamics, n, seed, table_z=table_z, focus=focus, sample=sample), finished,
                     lambda e: [widget.setEnabled(True) for widget in self.generate_buttons.values()])

    def fill_focus_table(self) -> None:
        from ..runtime.static_check import default_sample, pose_space

        sensitivity, weights = pose_space(self.robot, self.dynamics)
        sample = default_sample(weights)
        self.sensitivity = sensitivity
        lower, upper = self.robot.safety_window(margin_deg=3.0)
        self.sample_boxes = []
        for j in range(self.dof):
            for c, text in enumerate((f"{sensitivity[j]:.3f}", "",
                                      f"{np.degrees(lower[j]):.0f}", f"{np.degrees(upper[j]):.0f}", "0")):
                item = QTableWidgetItem(text)
                if c < 2:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.focus_table.setItem(j, c, item)
            box = QComboBox()
            box.addItems(["采样", "固定"])
            box.setCurrentIndex(0 if sample[j] else 1)
            box.setToolTip("默认：" + ("采样" if weights[j] > 0 else
                                      "采样（J1 默认采样；转动它不改变任何关节的重力力矩）" if sample[j] else
                                      "固定（转动它不改变任何关节的重力力矩）"))
            box.currentIndexChanged.connect(lambda *_: self.update_sampling())
            self.focus_table.setCellWidget(j, 1, box)
            self.sample_boxes.append(box)
        self.update_sampling()

    def check_stale(self) -> bool:
        """True (and a visible warning) when the listed poses no longer match the table."""
        if self.generated_settings is None or not len(self.static_poses):
            return False
        stale = self.generated_settings != (tuple(self.sampled_joints()), tuple(self.focus_specs()))
        self.stale_hint.setVisible(stale)
        for purpose, widget in self.generate_buttons.items():
            widget.setText("重新生成 ⟳" if stale and purpose == self.static_purpose else "生成")
        return stale

    def sampled_joints(self) -> list[bool]:
        return [box.currentIndex() == 0 for box in self.sample_boxes]

    def update_sampling(self) -> None:
        """Grey out fixed joints and show coverage plots of the sampled ones."""
        sampled = self.sampled_joints()
        for j, on in enumerate(sampled):
            for c in (0, 2, 3, 4):
                item = self.focus_table.item(j, c)
                if item is not None:
                    item.setForeground(QColor("#e6e6e6" if on else "#8f8e85"))
        lower, upper = self.robot.safety_window(margin_deg=3.0)
        self.coverage.set_axes(coverage_pairs([j for j, on in enumerate(sampled) if on]), np.degrees(lower), np.degrees(upper))
        if hasattr(self, "stale_hint"):
            self.check_stale()

    def focus_specs(self) -> list[tuple[int, float, float, float]]:
        specs = []
        for j in range(self.dof):
            try:
                lo, hi, pct = (float(self.focus_table.item(j, c).text()) for c in (2, 3, 4))
            except (ValueError, AttributeError):
                continue
            if pct > 0 and hi > lo and self.sampled_joints()[j]:
                specs.append((j, np.radians(lo), np.radians(hi), min(pct, 100.0) / 100))
        return specs

    def delete_static_pose(self) -> None:
        row = self.pose_list.currentRow()
        if row < 0:
            return
        name = self.static_out.text().strip().removesuffix(".csv")
        self.set_static_poses(np.delete(self.static_poses, row, axis=0), name)
        self.pose_list.setCurrentRow(min(row, self.pose_list.count() - 1))

    def show_static_pose(self, row: int) -> None:
        self.coverage.select(row)
        if self.view is not None and 0 <= row < len(self.static_poses) and not self.runner.busy:
            self.view.set_pose(self.static_poses[row])
            self.pose_label.setText(f"位姿 {row + 1}   q [°] " + "  ".join(f"{v:+6.1f}" for v in np.degrees(self.static_poses[row])))

    def run_static(self, execute: bool) -> None:
        from ..runtime.static_check import save_pose_file

        if not len(self.static_poses):
            QMessageBox.information(self, "没有位姿", "先点“静态训练”或“静态验证”的“生成”。")
            return
        if self.check_stale() and QMessageBox.question(
                self, "设置已改动", "位姿是按改动前的采样/重点设置生成的。仍用这些位姿运行？") != QMessageBox.Yes:
            return
        if execute and not self.confirm_hardware():
            return
        name = self.static_out.text().strip().removesuffix(".csv") or "check"
        out, pose_file = self.results.static(name), self.results.poses(name)
        if execute and out.exists() and QMessageBox.question(self, "覆盖", f"static/{out.name} 已存在，覆盖？") != QMessageBox.Yes:
            return
        save_pose_file(pose_file, self.static_poses, source="rebot_taccap_identification studio")
        args = ["--poses", str(pose_file), "--urdf", self.static_model.currentData(),
                "--out", str(out), *self.common_args()] + (["--execute"] if execute else [])
        self.gain_hint.setText("")
        self.gain_hint.setVisible(False)
        self.adopt_gain.setEnabled(False)

        def done(ok: bool) -> None:
            if execute:
                self.update_pose_seeds()             # the next generation gets other poses
            text = "\n".join(self.run_lines)
            match = re.search(r"测得：torque_gain = \[([\d., ]+)\]", text)
            if match:
                self.measured_gain = [float(v) for v in match.group(1).split(",")]
                self.gain_hint.setText("本次测得力矩标定：" + " ".join(f"{v:g}" for v in self.measured_gain)
                                       + "（某关节的指令力矩正负都覆盖时才可靠；只覆盖单侧时与偏置混叠）")
                self.gain_hint.setVisible(True)
                self.adopt_gain.setEnabled(True)
                QTimer.singleShot(0, self.fit_page)
        self.start(self.robot_cmd("static", *args), on_done=done, live=execute)
        if not execute:
            self.plots.setCurrentWidget(self.coverage)

    def apply_measured_gain(self) -> None:
        for spin, value in zip(self.gain, self.measured_gain or []):
            spin.setValue(value)
        self.log_line("[设置] 已采用力矩标定 " + " ".join(self.gain_values()))
        path = self.robot.save_torque_gain([s.value() for s in self.gain])
        self.show_gain_source(path)
        self.log_line(f"[设置] 已写入 {path}")

    def run_job(self, fn, on_done, on_fail=None) -> None:
        """Run ``fn`` in a thread."""
        job = Job(fn, self)
        self.jobs.append(job)
        job.done.connect(on_done)
        job.failed.connect(lambda msg: (self.log_line("[错误] " + msg), QMessageBox.warning(self, "出错", msg),
                                        on_fail(msg) if on_fail else None))
        job.finished.connect(lambda: self.jobs.remove(job))
        job.start()

    # --- identification (arm and payload)
    def run_payload(self) -> None:
        base = self.payload_base.currentData()
        if not base:
            QMessageBox.information(self, "没有本体模型", "先辨识机械臂本体（不装负载），再辨识负载。")
            return
        static, holdout = self.payload_checked("train"), self.payload_checked("val")
        if not static:
            QMessageBox.information(self, "缺少数据", "勾选装上负载后采的静态训练数据。")
            return
        if set(static) & set(holdout) and QMessageBox.question(
                self, "训练/验证重叠", "同一份数据同时用于训练和验证，评估将不公正。继续？") != QMessageBox.Yes:
            return
        name = self.payload_label()
        arm_version = Path(base).parent.name
        version = f"{arm_version}+{name}"
        results = self.payload_results
        out = results.urdf(arm_version, f"{self.robot.model_prefix()}_{name}")
        if out.exists() and QMessageBox.question(self, "覆盖", f"{name}/models/{arm_version} 已存在，覆盖？") != QMessageBox.Yes:
            return
        gain = self.gain_values()
        command = self.cli("payload", *self.robot_args, "--urdf", base, "--name", name, "--torque-gain", *gain,
                           "--out-urdf", str(out), "--static", *[str(results.static(n)) for n in static])
        if holdout:
            command += ["--val", *[str(results.static(n)) for n in holdout]]
        results.save_inputs(arm_version, static=static, holdout=holdout, payload=name, arm=base,
                            torque_gain=[float(g) for g in gain])
        self.payload_summary.setText("辨识负载中…")

        def done(ok: bool) -> None:
            text = "\n".join(self.run_lines)
            match = re.search(r"负载 .*?：质量 ([\d.]+) kg，质心 \[(.*?)\] mm", text)
            if not ok or not match:
                self.payload_summary.setText("⚠ 负载辨识失败，见日志")
                return
            note = "；".join(re.findall(r"^note: (.*)$", text, re.M))
            self.payload_summary.setText(f"✓ {version}：质量 {match.group(1)} kg，质心 [{match.group(2)}] mm。"
                                         "第 3 步选它做重力补偿检验。" + (f"<br>⚠ {note}" if note else ""))
        self.start(command, on_done=done)

    def run_identify(self) -> None:
        static, holdout = self.checked("static"), self.checked("holdout")
        if not static:
            QMessageBox.information(self, "缺少数据", "至少勾选一份静态训练数据。")
            return
        overlap = set(static) & set(holdout)
        if overlap and QMessageBox.question(self, "训练/验证重叠", f"{', '.join(sorted(overlap))} 同时用于训练和评估，"
                                            "评估将不公正。继续？") != QMessageBox.Yes:
            return
        name = self.version.text().strip()
        results = self.results
        if results.model_dir(name).exists() and \
                QMessageBox.question(self, "覆盖", f"models/{name} 已存在，覆盖？") != QMessageBox.Yes:
            return
        factor = self.drift_factor.value()
        weighted = [str(results.static(n)) + (f"@{factor:g}" if n.startswith("drift_") and factor != 1 else "")
                    for n in static]
        command = self.cli("identify", *self.robot_args, "--torque-gain", *self.gain_values(), "--version", name,
                           "--static", *weighted)
        if holdout:
            command += ["--val", *[str(results.static(n)) for n in holdout]]
        self.identify_summary.setText("辨识中…")

        def done(ok: bool) -> None:
            if not ok:
                self.identify_summary.setText("⚠ 辨识失败，见日志")
                return
            text = "\n".join(self.run_lines)
            match = re.search(r"^已导出 (.*)$", text, re.M)
            self.identify_summary.setText(f"✓ {Path(match.group(1)).name if match else name} 已导出。"
                                          + ("验收表见日志（看静态验证集上的结果）。" if holdout else "")
                                          + "下一步：第 3 步用它做重力补偿。")
            self.version.setText(self.results.next_version())
        self.start(command, on_done=done)

    # --- gravity compensation
    def run_gravity(self) -> None:
        if not self.confirm_hardware():
            return
        model = self.gravity_model.currentData()
        value = lambda key: f"{self.feel[key][2].value():g}"
        args = ["--urdf", model, "--mode", self.gravity_mode.currentText(), "--comp-kd", value("kd"),
                "--lock-kp", value("lock_kp"), "--ff-scale", *[f"{s.value():g}" for s in self.ff_scale],
                *self.common_args(), "--execute"]
        if self.results.is_export(model):          # complete URDF: the links below the tip are the payload
            args += ["--full-urdf"]
        if self.is_payload_model(model):          # collision checks (exit path to rest) include the payload
            args += ["--with-tool"] if self.tool_from_urdf else \
                ["--tool-length", f"{self.tool_length.value():g}", "--tool-radius", f"{self.tool_radius.value():g}"]
        self.gravity_running = True
        self.drift_load.setVisible(False)

        def done(ok: bool) -> None:
            self.gravity_running = False
            text = "\n".join(self.run_lines)
            match = re.search(r"^@drift (.*)$", text, re.M)
            if match:
                self.show_drift(json.loads(match.group(1)))
        self.start(self.robot_cmd("gravity", *args), on_done=done, live=True)

    def closeEvent(self, event) -> None:
        if self.runner.busy:
            if QMessageBox.question(self, "退出", "有任务在运行，安全停止并退出？") != QMessageBox.Yes:
                event.ignore()
                return
            self.runner.stop()
            if self.runner.process is not None:
                self.runner.process.waitForFinished(15000)
        # Background pose generation (collision screening) cannot be interrupted;
        # destroying a running QThread aborts the process, so let them finish unseen.
        for job in list(self.jobs):
            for signal in (job.done, job.failed):
                signal.disconnect()
            job.wait()
        event.accept()


def dark_palette() -> QPalette:
    """Dark Fusion palette matching the 3-D view."""
    palette = QPalette()
    colors = {
        QPalette.Window: "#1e2126", QPalette.WindowText: "#e6e6e6", QPalette.Base: "#16181c",
        QPalette.AlternateBase: "#22262c", QPalette.Text: "#e6e6e6", QPalette.Button: "#2b3038",
        QPalette.ButtonText: "#e6e6e6", QPalette.Highlight: "#2a78d6", QPalette.HighlightedText: "#ffffff",
        QPalette.ToolTipBase: "#2b3038", QPalette.ToolTipText: "#e6e6e6", QPalette.PlaceholderText: "#8f8e85",
    }
    for role, color in colors.items():
        palette.setColor(role, QColor(color))
    palette.setColor(QPalette.Disabled, QPalette.ButtonText, QColor("#6b6b6b"))
    palette.setColor(QPalette.Disabled, QPalette.Text, QColor("#6b6b6b"))
    return palette


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="rebot_taccap_identification studio", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-3d", action="store_true", help="不创建 3D 视图（无 OpenGL 时）")
    from ..robots.utils import ROBOT_HELP, _CLI_ROBOT

    parser.epilog = ROBOT_HELP
    argv, robot_type, fields = pop_robot_args(sys.argv[1:] if argv is None else list(argv))
    if robot_type is None and cli_robot_type():                 # already taken out by rebot_taccap_identification.cli
        robot_type, fields = _CLI_ROBOT["type"], dict(_CLI_ROBOT["fields"])
    args = parser.parse_args(argv)
    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    if robot_type is None:
        found = default_robot()
        if found is None:
            QMessageBox.critical(None, "没有机械臂", "没有可用的机器人类型（rebot_taccap_identification list）")
            return 1
        robot_type, fields = found
    window = Studio(robot_type, fields, with_3d=not args.no_3d)
    remember_last_robot(robot_type, fields)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
