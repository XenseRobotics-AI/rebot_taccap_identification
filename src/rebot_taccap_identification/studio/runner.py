"""Background processes and live plots for the identification studio.

Commands run as child processes (the same commands as on the command line). Their
stdout goes to the log; ``@state {json}`` lines (``--stream``) feed the 3-D view and
the plots. Stop sends SIGINT, which the hardware commands handle as their safe stop;
a second stop sends another SIGINT (immediate disable), a third kills the process.
"""

from __future__ import annotations

import json
import os
import signal
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, Qt, Signal



class Runner(QObject):
    """Runs a queue of commands one after another."""

    line = Signal(str)
    state = Signal(dict)
    prompt = Signal(str)           # a script waits for Enter
    started = Signal(str)
    finished = Signal(bool)        # True when every command exited with code 0

    def __init__(self, workdir: Path, parent=None):
        super().__init__(parent)
        self.workdir = Path(workdir)
        self.process: QProcess | None = None
        self.queue: list[list[str]] = []
        self.stops = 0
        self.ok = True
        self.buffer = ""

    @property
    def busy(self) -> bool:
        return self.process is not None

    def run(self, *commands: list[str]) -> bool:
        if self.busy:
            return False
        self.queue = [list(c) for c in commands]
        self.ok = True
        self._next()
        return True

    def _next(self) -> None:
        if not self.queue:
            self.process = None
            self.finished.emit(self.ok)
            return
        command = self.queue.pop(0)
        process = QProcess(self)
        env = QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONUNBUFFERED", "1")
        env.insert("PATH", str(Path(sys.executable).parent) + os.pathsep + env.value("PATH"))
        process.setProcessEnvironment(env)
        process.setWorkingDirectory(str(self.workdir))
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._read)
        process.finished.connect(self._done)
        self.process, self.stops, self.buffer = process, 0, ""
        self.started.emit(" ".join(command))
        self.line.emit("$ " + " ".join(command))
        process.start(command[0], command[1:])

    def _read(self) -> None:
        self.buffer += bytes(self.process.readAllStandardOutput()).decode("utf-8", "replace")
        *lines, self.buffer = self.buffer.split("\n")
        for text in lines:
            if text.startswith("@state "):
                try:
                    self.state.emit(json.loads(text[7:]))
                except json.JSONDecodeError:
                    pass
                continue
            self.line.emit(text)
        if "按回车" in self.buffer:          # input() prompt without a newline
            self.line.emit(self.buffer)
            self.prompt.emit(self.buffer)
            self.buffer = ""

    def _done(self, code: int, status) -> None:
        if self.buffer:
            self.line.emit(self.buffer)
        self.line.emit(f"[退出码 {code}]")
        if code != 0:
            self.ok = False
            self.queue.clear()
        self.process = None
        self._next()

    def answer(self, text: str = "\n") -> None:
        if self.process is not None:
            self.process.write(text.encode())

    def stop(self) -> None:
        if self.process is None:
            return
        self.queue.clear()
        self.stops += 1
        pid = self.process.processId()
        if self.stops <= 2 and pid:
            os.kill(pid, signal.SIGINT)
            self.line.emit("[停止] 已发送 Ctrl+C（安全停止）" if self.stops == 1 else "[停止] 再次 Ctrl+C：立即失能")
        else:
            self.process.kill()
            self.line.emit("[停止] 强制结束进程")


class LivePlots(pg.GraphicsLayoutWidget):
    """Rolling per-joint torque: measured feedback vs commanded/model."""

    def __init__(self, dof: int, seconds: float = 10.0, parent=None):
        super().__init__(parent)
        self.setBackground((20, 23, 28))
        self.t0 = None
        self.data = deque(maxlen=int(seconds * 20))         # @state lines come at 20 Hz
        self.curves = []
        for j in range(dof):
            plot = self.addPlot(row=j // 3, col=j % 3, title=f"J{j + 1} 力矩 [N·m]")
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setMenuEnabled(False)
            measured = plot.plot(pen=pg.mkPen((170, 170, 170), width=1.5), name="实测")
            command = plot.plot(pen=pg.mkPen((58, 135, 229), width=2), name="指令/模型")
            self.curves.append((measured, command))
        self.addLabel("灰：实测反馈  蓝：指令（含前馈）", row=(dof + 2) // 3, col=0, colspan=3, color=(150, 150, 150))

    def reset(self) -> None:
        self.data.clear()
        self.t0 = None
        for measured, command in self.curves:
            measured.setData([], [])
            command.setData([], [])

    def push(self, state: dict, now: float) -> None:
        if self.t0 is None:
            self.t0 = now
        self.data.append((now - self.t0, state["tau"], state["tau_cmd"]))
        t = np.array([d[0] for d in self.data])
        tau = np.array([d[1] for d in self.data])
        cmd = np.array([d[2] for d in self.data])
        for j, (measured, command) in enumerate(self.curves):
            measured.setData(t, tau[:, j])
            command.setData(t, cmd[:, j])


class PoseCoverage(pg.GraphicsLayoutWidget):
    """Where the static poses lie, for the sampled joints; click selects a pose."""

    picked = Signal(int)

    def __init__(self, dof: int, parent=None):
        super().__init__(parent)
        self.dof = dof
        self.setBackground((20, 23, 28))
        self.poses_deg = np.zeros((0, dof))
        self.panels = []          # (plot, window, scatter, mark, a, b)
        self.lower = self.upper = np.zeros(dof)

    def set_axes(self, pairs: list[tuple[int, int]], lower_deg: np.ndarray, upper_deg: np.ndarray) -> None:
        """One scatter per joint pair (0-based indices), with the safety window dashed."""
        self.clear()
        self.panels = []
        self.lower, self.upper = np.asarray(lower_deg), np.asarray(upper_deg)
        for col, (a, b) in enumerate(pairs):
            single = b < 0                      # one sampled joint: plot it against the pose index
            plot = self.addPlot(row=0, col=col, title=f"J{a + 1} 覆盖 [°]" if single else f"J{a + 1}–J{b + 1} 覆盖 [°]")
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setLabel("bottom", f"J{a + 1}")
            plot.setLabel("left", "位姿序号" if single else f"J{b + 1}")
            plot.setMenuEnabled(False)
            window = pg.PlotCurveItem(pen=pg.mkPen((110, 110, 110), style=Qt.DashLine))
            if not single:
                window.setData([self.lower[a], self.upper[a], self.upper[a], self.lower[a], self.lower[a]],
                               [self.lower[b], self.lower[b], self.upper[b], self.upper[b], self.lower[b]])
            scatter = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(58, 135, 229, 200), pen=None, hoverable=True)
            scatter.sigClicked.connect(lambda _, points, ev: points and self.picked.emit(int(points[0].data())))
            mark = pg.ScatterPlotItem(size=18, brush=None, pen=pg.mkPen((235, 104, 52), width=2.5))
            for item in (window, scatter, mark):
                plot.addItem(item)
            self.panels.append((plot, window, scatter, mark, a, b))
        self.addLabel("蓝点：位姿；橙圈：当前选中；虚线：安全窗口。点击点可选中。", row=1, col=0,
                      colspan=max(len(pairs), 1), color=(150, 150, 150))
        self.set_poses(self.poses_deg)

    def set_poses(self, poses_deg: np.ndarray) -> None:
        self.poses_deg = np.asarray(poses_deg, dtype=float).reshape(-1, self.dof)
        index = np.arange(1, len(self.poses_deg) + 1, dtype=float)
        for plot, _, scatter, _, a, b in self.panels:
            y = index if b < 0 else self.poses_deg[:, b]
            scatter.setData(x=self.poses_deg[:, a], y=y, data=list(range(len(self.poses_deg))))
            # Range = safety window (+5 %), so every pose and the dashed window are always in view.
            pad_x = 0.05 * (self.upper[a] - self.lower[a])
            plot.setXRange(self.lower[a] - pad_x, self.upper[a] + pad_x, padding=0)
            if b < 0:
                plot.setYRange(0, max(len(index), 1) + 1, padding=0)
            else:
                pad_y = 0.05 * (self.upper[b] - self.lower[b])
                plot.setYRange(self.lower[b] - pad_y, self.upper[b] + pad_y, padding=0)
        self.select(-1)

    def select(self, index: int) -> None:
        for _, _, _, mark, a, b in self.panels:
            if 0 <= index < len(self.poses_deg):
                mark.setData(x=[self.poses_deg[index, a]], y=[index + 1 if b < 0 else self.poses_deg[index, b]])
            else:
                mark.setData(x=[], y=[])
