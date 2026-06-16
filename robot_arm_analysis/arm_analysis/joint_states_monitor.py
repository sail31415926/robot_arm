#!/usr/bin/env python3
"""
Joint States Monitor — /joint_states 实时监视 GUI

功能:
  - 实时滚动曲线: 位置(rad) / 速度(rad/s) / 力矩(N·m)
  - 当前值数值表格（每帧刷新）
  - 可调时间窗口、暂停/恢复、清除缓冲区、保存 CSV

用法:
  ros2 run robot_arm_analysis joint_states_monitor
  ros2 run robot_arm_analysis joint_states_monitor --ros-args -p window_sec:=60.0
"""

import collections
import csv
import datetime
import sys
import time
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QGroupBox, QStatusBar,
    QTableWidget, QTableWidgetItem, QSizePolicy, QHeaderView, QSplitter,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QFont, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# 6轴配色，与关节顺序一一对应
_JOINT_COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c']


# ─────────────────────────────────────────────────────────────────────────────
# ROS 信号桥（跨线程传数据到 GUI 线程）
# ─────────────────────────────────────────────────────────────────────────────

class _Signals(QObject):
    joint_state_received = pyqtSignal(list, list, list, list)   # names, pos, vel, eff


# ─────────────────────────────────────────────────────────────────────────────
# ROS 节点
# ─────────────────────────────────────────────────────────────────────────────

class JointMonitorNode(Node):
    def __init__(self, signals: _Signals, window_sec: float = 30.0):
        super().__init__('joint_states_monitor')
        self.declare_parameter('window_sec', window_sec)
        window_sec = self.get_parameter('window_sec').value

        self._signals = signals
        self._lock = threading.Lock()
        self._window_sec = window_sec
        self._joint_names: list = []

        maxlen = self._maxlen(window_sec)
        self._time: collections.deque = collections.deque(maxlen=maxlen)
        self._buf: dict = {}   # joint_name → {'pos', 'vel', 'eff'}: deque

        # Rate estimation
        self._msg_count = 0
        self._rate_t0 = time.monotonic()
        self._rate_hz = 0.0

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 100)
        self.get_logger().info(f'Monitoring /joint_states  (window={window_sec}s)')

    # ── public API ────────────────────────────────────────────────────────────

    def set_window(self, sec: float):
        with self._lock:
            self._window_sec = sec
            maxlen = self._maxlen(sec)
            self._time = collections.deque(self._time, maxlen=maxlen)
            for d in self._buf.values():
                for key in ('pos', 'vel', 'eff'):
                    d[key] = collections.deque(d[key], maxlen=maxlen)

    def clear(self):
        with self._lock:
            self._time.clear()
            for d in self._buf.values():
                for key in ('pos', 'vel', 'eff'):
                    d[key].clear()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'names': list(self._joint_names),
                'time':  list(self._time),
                'buf':   {n: {k: list(v) for k, v in d.items()}
                          for n, d in self._buf.items()},
                'rate':  self._rate_hz,
            }

    def save_csv(self, path: str) -> int:
        snap = self.snapshot()
        if not snap['time']:
            return 0
        names = snap['names']
        header = (['timestamp'] +
                  [f'{n}_pos' for n in names] +
                  [f'{n}_vel' for n in names] +
                  [f'{n}_eff' for n in names])
        rows = []
        for i, t in enumerate(snap['time']):
            row = [t]
            for key in ('pos', 'vel', 'eff'):
                for n in names:
                    arr = snap['buf'].get(n, {}).get(key, [])
                    row.append(arr[i] if i < len(arr) else 0.0)
            rows.append(row)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
        return len(rows)

    # ── internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _maxlen(sec: float) -> int:
        return int(sec * 200)   # 200 Hz max headroom

    def _on_joint_state(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        names = list(msg.name)

        # Rate
        self._msg_count += 1
        now = time.monotonic()
        elapsed = now - self._rate_t0
        if elapsed >= 1.0:
            self._rate_hz = self._msg_count / elapsed
            self._msg_count = 0
            self._rate_t0 = now

        with self._lock:
            # Init new joints
            if names != self._joint_names:
                self._joint_names = names
                maxlen = self._maxlen(self._window_sec)
                for n in names:
                    if n not in self._buf:
                        self._buf[n] = {
                            'pos': collections.deque(maxlen=maxlen),
                            'vel': collections.deque(maxlen=maxlen),
                            'eff': collections.deque(maxlen=maxlen),
                        }

            self._time.append(t)
            for i, n in enumerate(names):
                self._buf[n]['pos'].append(msg.position[i] if i < len(msg.position) else 0.0)
                self._buf[n]['vel'].append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                self._buf[n]['eff'].append(msg.effort[i]   if i < len(msg.effort)    else 0.0)

        # Emit latest values (GUI thread safe via signal queue)
        pos = [msg.position[i] if i < len(msg.position) else 0.0 for i in range(len(names))]
        vel = [msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in range(len(names))]
        eff = [msg.effort[i]   if i < len(msg.effort)   else 0.0 for i in range(len(names))]
        self._signals.joint_state_received.emit(names, pos, vel, eff)


# ─────────────────────────────────────────────────────────────────────────────
# GUI 主窗口
# ─────────────────────────────────────────────────────────────────────────────

class JointMonitorWindow(QMainWindow):
    def __init__(self, node: JointMonitorNode, signals: _Signals):
        super().__init__()
        self._node = node
        self._paused = False
        self._lines_ready = False
        self._plot_lines: dict = {}    # key('pos'/'vel'/'eff') → {joint: Line2D}
        self._prev_npts = -1

        self.setWindowTitle('Joint States Monitor — /joint_states')
        self.setMinimumSize(960, 720)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(4)
        root.setContentsMargins(8, 8, 8, 4)

        # Splitter: plot上 / 数值表下
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self._build_plot_widget())
        splitter.addWidget(self._build_table_group())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter)

        root.addLayout(self._build_controls())

        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._statusbar.showMessage('等待 /joint_states …')

        signals.joint_state_received.connect(self._on_joint_state_ui)

        # 绘图刷新 20 Hz
        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(50)
        self._plot_timer.timeout.connect(self._refresh_plot)
        self._plot_timer.start()

        # 状态栏刷新 1 Hz
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start()

    # ── widgets ───────────────────────────────────────────────────────────────

    def _build_plot_widget(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)

        self._fig = Figure(tight_layout=True)
        self._ax_pos = self._fig.add_subplot(311)
        self._ax_vel = self._fig.add_subplot(312, sharex=self._ax_pos)
        self._ax_eff = self._fig.add_subplot(313, sharex=self._ax_pos)
        self._style_axes()

        self._canvas = FigureCanvas(self._fig)
        self._canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self._canvas)
        return w

    def _style_axes(self):
        specs = [
            (self._ax_pos, '位置 (rad)',    'pos (rad)'),
            (self._ax_vel, '速度 (rad/s)',  'vel (rad/s)'),
            (self._ax_eff, '力矩 (N·m)',    'effort (N·m)'),
        ]
        for ax, title, ylabel in specs:
            ax.set_title(title, fontsize=9, pad=2)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, linestyle='--', alpha=0.35)
        self._ax_eff.set_xlabel('time (s)', fontsize=8)

    def _build_table_group(self) -> QGroupBox:
        box = QGroupBox('当前关节值')
        box.setMaximumHeight(170)
        v = QVBoxLayout(box)
        v.setContentsMargins(4, 2, 4, 4)

        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(['关节', '位置 (rad)', '速度 (rad/s)', '力矩 (N·m)'])
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.NoSelection)
        self._table.setAlternatingRowColors(True)
        f = QFont('Monospace')
        f.setPointSize(8)
        self._table.setFont(f)
        v.addWidget(self._table)

        self._table_rows: dict = {}   # joint_name → row_index
        return box

    def _build_controls(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)

        row.addWidget(QLabel('时间窗口:'))
        self._spin = QDoubleSpinBox()
        self._spin.setRange(5.0, 300.0)
        self._spin.setValue(30.0)
        self._spin.setSuffix(' s')
        self._spin.setSingleStep(5.0)
        self._spin.setFixedWidth(90)
        self._spin.valueChanged.connect(lambda v: self._node.set_window(v))
        row.addWidget(self._spin)

        row.addSpacing(10)

        self._btn_pause = QPushButton('⏸ 暂停')
        self._btn_pause.setCheckable(True)
        self._btn_pause.setFixedWidth(80)
        self._btn_pause.toggled.connect(self._on_pause)
        row.addWidget(self._btn_pause)

        btn_clear = QPushButton('🗑 清除')
        btn_clear.setFixedWidth(70)
        btn_clear.clicked.connect(self._on_clear)
        row.addWidget(btn_clear)

        btn_save = QPushButton('💾 保存 CSV')
        btn_save.setFixedWidth(100)
        btn_save.clicked.connect(self._on_save)
        row.addWidget(btn_save)

        row.addStretch()

        self._lbl_rate = QLabel('Rate: -- Hz')
        self._lbl_rate.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self._lbl_rate)

        return row

    # ── slots ─────────────────────────────────────────────────────────────────

    def _on_joint_state_ui(self, names: list, pos: list, vel: list, eff: list):
        """更新当前值表格（GUI 线程，由信号触发）。"""
        for i, name in enumerate(names):
            if name not in self._table_rows:
                r = self._table.rowCount()
                self._table.insertRow(r)
                self._table_rows[name] = r

                name_item = QTableWidgetItem(name)
                c = QColor(_JOINT_COLORS[i % len(_JOINT_COLORS)])
                name_item.setBackground(c)
                name_item.setForeground(QColor('#ffffff'))
                name_item.setFont(QFont('', 8, QFont.Bold))
                self._table.setItem(r, 0, name_item)

            r = self._table_rows[name]
            self._table.setItem(r, 1, QTableWidgetItem(f'{pos[i]: .5f}'))
            self._table.setItem(r, 2, QTableWidgetItem(f'{vel[i]: .5f}'))
            self._table.setItem(r, 3, QTableWidgetItem(f'{eff[i]: .4f}'))

    def _on_pause(self, checked: bool):
        self._paused = checked
        self._btn_pause.setText('▶ 恢复' if checked else '⏸ 暂停')

    def _on_clear(self):
        self._node.clear()
        self._lines_ready = False
        self._plot_lines = {}
        self._prev_npts = -1
        for ax in (self._ax_pos, self._ax_vel, self._ax_eff):
            ax.cla()
        self._style_axes()
        self._canvas.draw_idle()

    def _on_save(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out = Path.home() / 'robot_arm_data'
        out.mkdir(parents=True, exist_ok=True)
        path = str(out / f'joint_states_{ts}.csv')
        n = self._node.save_csv(path)
        self._statusbar.showMessage(f'已保存 {n} 行 → {path}', 6000)

    # ── plot refresh ──────────────────────────────────────────────────────────

    def _refresh_plot(self):
        if self._paused:
            return

        snap = self._node.snapshot()
        if not snap['names'] or not snap['time']:
            return

        # 首次收到数据时建立 Line2D 对象（只建一次，后续只更新 data）
        if not self._lines_ready:
            self._plot_lines = {'pos': {}, 'vel': {}, 'eff': {}}
            ax_map = {'pos': self._ax_pos, 'vel': self._ax_vel, 'eff': self._ax_eff}
            for i, name in enumerate(snap['names']):
                c = _JOINT_COLORS[i % len(_JOINT_COLORS)]
                for key, ax in ax_map.items():
                    line, = ax.plot([], [], color=c, lw=1.3,
                                    label=(name if key == 'pos' else '_'))
                    self._plot_lines[key][name] = line
            self._ax_pos.legend(loc='upper left', fontsize=7,
                                 ncol=min(len(snap['names']), 6),
                                 framealpha=0.6)
            self._lines_ready = True

        npts = len(snap['time'])
        if npts == self._prev_npts:
            return   # 无新数据，跳过重绘
        self._prev_npts = npts

        t0 = snap['time'][0]
        t_rel = [t - t0 for t in snap['time']]

        for key, lines_dict in self._plot_lines.items():
            for name, line in lines_dict.items():
                arr = snap['buf'].get(name, {}).get(key, [])
                n = min(len(t_rel), len(arr))
                line.set_data(t_rel[:n], arr[:n])

        for ax in (self._ax_pos, self._ax_vel, self._ax_eff):
            ax.relim()
            ax.autoscale_view()

        self._canvas.draw_idle()

    def _refresh_status(self):
        snap = self._node.snapshot()
        self._lbl_rate.setText(f'Rate: {snap["rate"]:.1f} Hz')
        n_joints = len(snap['names'])
        n_pts = len(snap['time'])
        if n_joints:
            self._statusbar.showMessage(
                f'/joint_states  │  {n_joints} 关节  │  '
                f'{snap["rate"]:.0f} Hz  │  缓冲 {n_pts} 点  │  '
                f'窗口 {self._spin.value():.0f} s')


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    signals = _Signals()
    node = JointMonitorNode(signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    win = JointMonitorWindow(node, signals)
    win.show()

    ret = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)
