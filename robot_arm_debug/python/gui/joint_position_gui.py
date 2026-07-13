#!/usr/bin/env python3
"""
@file   joint_position_gui.py
@brief  eMeetArm_models 6轴机械臂关节滑块控制 GUI
@version 1.1
@date   2026-07-10

基于 PyQt5 + ROS2 实现以下功能：
         - 6个关节独立滑块控制，滑块与数值框双向同步
         - 发布轨迹指令至 /arm_controller/joint_trajectory
         - 订阅 /joint_states 实时显示各关节位置与速度
         - 通过 TF2 查询并显示末端 tool0 在 base_link 下的坐标
         - 支持可调运动时间与一键回零位功能

v1.1 两个实机安全/体验修复：
  1. 滑块初值播种：启动后用第一帧 /joint_states 初始化滑块/数值框为当前关节位置
     （此前初值恒为 0，臂不在零位时一发送就会全臂朝零位跑）。
  2. 点到点发送（默认）：拖动滑块只更新显示，**松手才发送一次**（运动时长取
     下方设置值）——与实物 PP 模式（驱动器自规划）匹配；此前拖动即以 0.05s
     时长高频流式发布，PP 下每条都触发重规划，表现为卡顿走停。
     「启动发送」开关保留为流式模式（200ms 周期），适合仿真/IP 后端。

用法：
  ros2 run robot_arm_node joint_position_gui
  ros2 launch robot_arm_gazebo gazebo.launch.py controller:=slider
  ros2 launch robot_arm_mujoco mujoco.launch.py controller:=slider
  ros2 launch robot_arm_bringup real.launch.py   controller:=slider

@copyright Copyright (c) 2026 eMeet
"""

import sys
import threading
import rclpy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QSlider, QDoubleSpinBox, QPushButton,
    QGroupBox, QStatusBar,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QFont

from joint_position_controller_node import JointPositionControllerNode, JOINT_NAMES

JOINT_LIMITS = [
    (-3.1,    3.1),
    (-0.8,    3.14),
    (-3.14,   0.0),
    (-3.1,    3.1),
    (-0.7854, 0.7854),
    (-1.5,    0.5),
]
SLIDER_SCALE = 1000


class RosSignals(QObject):
    joint_state_received = pyqtSignal(list, list)
    end_effector_received = pyqtSignal(float, float, float, float, float, float)


class MainWindow(QMainWindow):
    def __init__(self, node: JointPositionControllerNode, signals: RosSignals):
        super().__init__()
        self.node = node
        self._seeded = False   # 滑块是否已用第一帧 /joint_states 播种
        self.setWindowTitle('eMeet 6轴机械臂关节控制器')
        self.setMinimumWidth(720)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setSpacing(8)
        root_layout.setContentsMargins(12, 12, 12, 8)

        root_layout.addWidget(self._build_joint_group())
        root_layout.addWidget(self._build_end_effector_group())
        root_layout.addWidget(self._build_duration_group())
        root_layout.addWidget(self._build_button_row())

        self._send_timer = QTimer(self)
        self._send_timer.setInterval(200)
        self._send_timer.timeout.connect(self._send)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage(
            '发布: /arm_controller/joint_trajectory  |  订阅: /joint_states')

        signals.joint_state_received.connect(self._on_joint_state)
        signals.end_effector_received.connect(self._on_end_effector)

    def _build_joint_group(self) -> QGroupBox:
        box = QGroupBox('关节控制')
        grid = QGridLayout(box)
        grid.setSpacing(6)

        bold = QFont()
        bold.setBold(True)
        for col, text in enumerate(['关节', '滑块', '目标位置 (rad)', '当前位置 (rad)', '当前速度 (rad/s)']):
            lbl = QLabel(text)
            lbl.setFont(bold)
            lbl.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl, 0, col)

        self.sliders: list[QSlider] = []
        self.spinboxes: list[QDoubleSpinBox] = []
        self.cur_pos_labels: list[QLabel] = []
        self.cur_vel_labels: list[QLabel] = []

        for i, (name, (lo, hi)) in enumerate(zip(JOINT_NAMES, JOINT_LIMITS)):
            row = i + 1

            grid.addWidget(QLabel(name, alignment=Qt.AlignCenter), row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, SLIDER_SCALE)
            slider.setValue(self._rad_to_tick(0.0, lo, hi))
            slider.setTickInterval(SLIDER_SCALE // 10)
            slider.setTickPosition(QSlider.TicksBelow)
            self.sliders.append(slider)
            grid.addWidget(slider, row, 1)

            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            spin.setValue(0.0)
            spin.setFixedWidth(90)
            self.spinboxes.append(spin)
            grid.addWidget(spin, row, 2)

            cur_pos = QLabel('--', alignment=Qt.AlignCenter)
            cur_vel = QLabel('--', alignment=Qt.AlignCenter)
            cur_pos.setStyleSheet('background:#f0f0f0; border:1px solid #ccc; padding:2px;')
            cur_vel.setStyleSheet('background:#f0f0f0; border:1px solid #ccc; padding:2px;')
            self.cur_pos_labels.append(cur_pos)
            self.cur_vel_labels.append(cur_vel)
            grid.addWidget(cur_pos, row, 3)
            grid.addWidget(cur_vel, row, 4)

            lo_, hi_ = lo, hi
            # 拖动/改数只同步显示；流式模式（启动发送）下才随动发布
            slider.valueChanged.connect(
                lambda val, s=spin, l=lo_, h=hi_: (
                    s.blockSignals(True),
                    s.setValue(self._tick_to_rad(val, l, h)),
                    s.blockSignals(False),
                    self._send_realtime(),
                )
            )
            spin.valueChanged.connect(
                lambda val, sl=slider, l=lo_, h=hi_: (
                    sl.blockSignals(True),
                    sl.setValue(self._rad_to_tick(val, l, h)),
                    sl.blockSignals(False),
                    self._send_realtime(),
                )
            )
            # 点到点（默认）：滑块松手 / 数值框回车或失焦时发送一次（PP 友好）
            slider.sliderReleased.connect(self._send_on_release)
            spin.editingFinished.connect(self._send_on_release)

        grid.setColumnStretch(1, 1)
        return box

    def _build_end_effector_group(self) -> QGroupBox:
        box = QGroupBox('末端位姿 (base_link → tool0)')
        grid = QGridLayout(box)
        grid.setSpacing(6)
        style = 'background:#f0f0f0; border:1px solid #ccc; padding:2px;'
        self.ee_labels: dict[str, QLabel] = {}
        for col, (key, unit) in enumerate([('X', 'm'), ('Y', 'm'), ('Z', 'm')]):
            grid.addWidget(QLabel(f'{key} ({unit}):'), 0, col * 2, Qt.AlignRight)
            lbl = QLabel('--')
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(90)
            lbl.setStyleSheet(style)
            self.ee_labels[key] = lbl
            grid.addWidget(lbl, 0, col * 2 + 1)
        for col, (key, label, unit) in enumerate([('Roll', 'R', '°'), ('Pitch', 'P', '°'), ('Yaw', 'Y', '°')]):
            grid.addWidget(QLabel(f'{label} ({unit}):'), 1, col * 2, Qt.AlignRight)
            lbl = QLabel('--')
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(90)
            lbl.setStyleSheet(style)
            self.ee_labels[key] = lbl
            grid.addWidget(lbl, 1, col * 2 + 1)
        return box

    def _build_duration_group(self) -> QGroupBox:
        box = QGroupBox('运动时间（点到点发送的运动时长；流式模式下拖动固定 0.05 s）')
        layout = QHBoxLayout(box)
        layout.addWidget(QLabel('运动时间 (s):'))

        self.dur_slider = QSlider(Qt.Horizontal)
        self.dur_slider.setRange(1, 50)
        self.dur_slider.setValue(10)
        layout.addWidget(self.dur_slider, 1)

        self.dur_spin = QDoubleSpinBox()
        self.dur_spin.setRange(0.1, 5.0)
        self.dur_spin.setDecimals(1)
        self.dur_spin.setSingleStep(0.1)
        self.dur_spin.setValue(1.0)
        self.dur_spin.setFixedWidth(72)
        layout.addWidget(self.dur_spin)

        layout.addWidget(QLabel('← 快    慢 →'))

        self.dur_slider.valueChanged.connect(
            lambda v: (self.dur_spin.blockSignals(True),
                       self.dur_spin.setValue(v * 0.1),
                       self.dur_spin.blockSignals(False))
        )
        self.dur_spin.valueChanged.connect(
            lambda v: (self.dur_slider.blockSignals(True),
                       self.dur_slider.setValue(round(v / 0.1)),
                       self.dur_slider.blockSignals(False))
        )
        return box

    def _build_button_row(self) -> QWidget:
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)

        self.send_btn = QPushButton('启 动 发 送')
        self.send_btn.setCheckable(True)
        self.send_btn.setFixedHeight(40)
        self.send_btn.setStyleSheet('background:#27AE60; color:white; font-size:13px; font-weight:bold;')
        self.send_btn.clicked.connect(self._toggle_send)
        layout.addWidget(self.send_btn)

        reset_btn = QPushButton('回 零 位')
        reset_btn.setFixedHeight(40)
        reset_btn.setStyleSheet('background:#E74C3C; color:white; font-size:13px; font-weight:bold;')
        reset_btn.clicked.connect(self._reset)
        layout.addWidget(reset_btn)

        return row

    def _rad_to_tick(self, rad: float, lo: float, hi: float) -> int:
        return round((rad - lo) / (hi - lo) * SLIDER_SCALE)

    def _tick_to_rad(self, tick: int, lo: float, hi: float) -> float:
        return lo + tick / SLIDER_SCALE * (hi - lo)

    def _toggle_send(self, checked: bool):
        if checked:
            self._send_timer.start()
            self.send_btn.setText('关 闭 发 送')
            self.send_btn.setStyleSheet(
                'background:#E67E22; color:white; font-size:13px; font-weight:bold;')
            self.status_bar.showMessage('● 持续发送中  |  /arm_controller/joint_trajectory')
        else:
            self._send_timer.stop()
            self.send_btn.setText('启 动 发 送')
            self.send_btn.setStyleSheet(
                'background:#27AE60; color:white; font-size:13px; font-weight:bold;')
            self.status_bar.showMessage(
                '发布: /arm_controller/joint_trajectory  |  订阅: /joint_states')

    def _send(self, duration: float | None = None):
        positions = [spin.value() for spin in self.spinboxes]
        self.node.publish_trajectory(
            positions,
            duration if duration is not None else self.dur_spin.value(),
        )

    def _send_realtime(self):
        if not self._send_timer.isActive():
            return
        self._send(duration=0.05)

    def _send_on_release(self):
        """点到点发送（默认交互）：松手/确认输入时发一条完整时长的轨迹。

        流式模式（启动发送）激活时由 200ms 定时器负责，这里不重复发。
        实物 PP 模式下驱动器按 6081 自规划一条平滑梯形——单发即流畅；
        高频流式发布会让 PP 每条都重规划，表现为走停卡顿。
        """
        if self._send_timer.isActive():
            return
        self._send()

    def _reset(self):
        for spin in self.spinboxes:
            spin.blockSignals(True)
            spin.setValue(0.0)
            spin.blockSignals(False)
        for i, (lo, hi) in enumerate(JOINT_LIMITS):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(self._rad_to_tick(0.0, lo, hi))
            self.sliders[i].blockSignals(False)
        self._send()
        if self._send_timer.isActive():
            self._send_timer.stop()
            self.send_btn.setChecked(False)
            self.send_btn.setText('启 动 发 送')
            self.send_btn.setStyleSheet(
                'background:#27AE60; color:white; font-size:13px; font-weight:bold;')

    def _on_joint_state(self, positions: list, velocities: list):
        # 首帧播种：滑块/数值框初始化为当前关节位置（不触发发送），
        # 避免"初值 0 + 一发送 → 全臂朝零位跑"的安全隐患
        if not self._seeded:
            self._seeded = True
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                pos = min(max(positions[i], lo), hi)
                self.spinboxes[i].blockSignals(True)
                self.spinboxes[i].setValue(pos)
                self.spinboxes[i].blockSignals(False)
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(self._rad_to_tick(pos, lo, hi))
                self.sliders[i].blockSignals(False)
        for i in range(len(JOINT_NAMES)):
            self.cur_pos_labels[i].setText(f'{positions[i]:.4f}')
            self.cur_vel_labels[i].setText(f'{velocities[i]:.4f}')

    def _on_end_effector(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        self.ee_labels['X'].setText(f'{x:.4f}')
        self.ee_labels['Y'].setText(f'{y:.4f}')
        self.ee_labels['Z'].setText(f'{z:.4f}')
        self.ee_labels['Roll'].setText(f'{roll:.2f}')
        self.ee_labels['Pitch'].setText(f'{pitch:.2f}')
        self.ee_labels['Yaw'].setText(f'{yaw:.2f}')


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    signals = RosSignals()
    node = JointPositionControllerNode(
        on_joint_state=lambda pos, vel: signals.joint_state_received.emit(pos, vel),
        on_end_effector=lambda x, y, z, ro, pi, ya:
            signals.end_effector_received.emit(x, y, z, ro, pi, ya),
    )

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    win = MainWindow(node, signals)
    win.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
