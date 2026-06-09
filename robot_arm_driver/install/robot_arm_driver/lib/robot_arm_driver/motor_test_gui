#!/usr/bin/env python3
"""
@file   motor_test_gui.py
@brief  arm_motor_node 多关节调试 GUI（PyQt5 + rclpy）
@version 2.0
@date   2026-06-04

运行方式：
    ros2 run robot_arm_driver motor_test_gui

架构说明：
    三个关节各自对应一个 Tab，每个 Tab 拥有独立的 MotorRosNode 连接。
    使用 QTimer 每 10ms 在 Qt 主线程依次调用三个节点的 spin_once()，
    所有 ROS2 回调在主线程执行，无多线程竞争。

用法：
  ros2 run robot_arm_driver motor_test_gui
  ros2 launch robot_arm_bringup motor.launch.py

@copyright Copyright (c) 2026 EMEET
"""

import sys
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton,
    QSlider, QDoubleSpinBox, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont

# ── 关节配置（ns, label, pos_min_rad, pos_max_rad）────────────────────────────
JOINTS = [
    ("/joint1/arm_motor_node", "J1", -3.0,     3.0),
    ("/joint2/arm_motor_node", "J2", -0.8,     3.0),
    ("/joint3/arm_motor_node", "J3", -3.0,     0.05),
]

# ── 默认量程（速度/力矩，所有关节相同）────────────────────────────────────────
POS_MIN, POS_MAX = -math.pi, math.pi  # 仅作 J1 默认值
VEL_MIN, VEL_MAX = -2.0, 2.0
EFF_MIN, EFF_MAX = -5.0, 5.0
SLIDER_STEPS = 2000

_LEVEL_STYLE = {
    DiagnosticStatus.OK:    ("#27ae60", "就绪"),
    DiagnosticStatus.WARN:  ("#f39c12", "警告"),
    DiagnosticStatus.ERROR: ("#e74c3c", "错误"),
}


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 节点（每个关节独立一个实例）
# ─────────────────────────────────────────────────────────────────────────────
class MotorRosNode(Node):
    _counter = 0

    def __init__(self):
        MotorRosNode._counter += 1
        super().__init__(f"arm_motor_gui_{MotorRosNode._counter}")
        self._gui_pubs    = {}
        self._gui_subs    = []
        self._gui_clients = {}

        self.on_joint_state  = None
        self.on_status       = None
        self.on_mode         = None
        self.on_service_done = None

    def connect(self, ns: str):
        ns = ns.rstrip("/")

        for p in self._gui_pubs.values():
            self.destroy_publisher(p)
        self._gui_pubs.clear()

        for s in self._gui_subs:
            self.destroy_subscription(s)
        self._gui_subs.clear()

        for c in self._gui_clients.values():
            self.destroy_client(c)
        self._gui_clients.clear()

        def t(suffix):
            return f"{ns}/{suffix}"

        self._gui_pubs["pos"] = self.create_publisher(Float64, t("cmd_pos"), 10)
        self._gui_pubs["vel"] = self.create_publisher(Float64, t("cmd_vel"), 10)
        self._gui_pubs["eff"] = self.create_publisher(Float64, t("cmd_eff"), 10)

        self._gui_subs.append(self.create_subscription(
            JointState,       t("joint_states"), self._cb_joint_state, 10))
        self._gui_subs.append(self.create_subscription(
            DiagnosticStatus, t("status"),       self._cb_status,      10))
        self._gui_subs.append(self.create_subscription(
            String,           t("mode"),         self._cb_mode,        10))

        for name in ["enable", "disable", "recover",
                     "position_mode", "velocity_mode", "torque_mode", "ip_mode",
                     "homing", "set_home"]:
            self._gui_clients[name] = self.create_client(Trigger, t(name))

        self.get_logger().info(f"连接到: {ns}")
        return ns

    def _cb_joint_state(self, msg: JointState):
        if self.on_joint_state:
            pos = msg.position[0] if msg.position else 0.0
            vel = msg.velocity[0] if msg.velocity else 0.0
            eff = msg.effort[0]   if msg.effort   else 0.0
            self.on_joint_state(pos, vel, eff)

    def _cb_status(self, msg: DiagnosticStatus):
        if self.on_status:
            self.on_status(msg.level, msg.message)

    def _cb_mode(self, msg: String):
        if self.on_mode:
            self.on_mode(msg.data)

    def call_service(self, name: str):
        client = self._gui_clients.get(name)
        if not client:
            if self.on_service_done:
                self.on_service_done(False, f"未知服务: {name}")
            return
        if not client.service_is_ready():
            if self.on_service_done:
                self.on_service_done(False, f"~/{name} 不可用，电机节点是否已启动？")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._cb_service)

    def _cb_service(self, future):
        if not self.on_service_done:
            return
        try:
            r = future.result()
            self.on_service_done(r.success, r.message)
        except Exception as e:
            self.on_service_done(False, str(e))

    def publish(self, channel: str, value: float):
        pub = self._gui_pubs.get(channel)
        if pub:
            msg = Float64()
            msg.data = value
            pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
#  滑杆 + 数值框组件
# ─────────────────────────────────────────────────────────────────────────────
class SliderGroup(QWidget):
    value_changed = pyqtSignal(float)

    def __init__(self, label: str, vmin: float, vmax: float, unit: str):
        super().__init__()
        self._min      = vmin
        self._max      = vmax
        self._blocking = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        lbl = QLabel(label)
        lbl.setFixedWidth(115)
        layout.addWidget(lbl)

        layout.addWidget(QLabel(f"{vmin:.2f}"))

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, SLIDER_STEPS)
        self._slider.setValue(SLIDER_STEPS // 2)
        self._slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._slider.valueChanged.connect(self._on_slider)
        layout.addWidget(self._slider)

        layout.addWidget(QLabel(f"{vmax:.2f}"))

        self._spin = QDoubleSpinBox()
        self._spin.setRange(vmin, vmax)
        self._spin.setSingleStep((vmax - vmin) / 200)
        self._spin.setDecimals(3)
        self._spin.setFixedWidth(95)
        self._spin.setSuffix(f" {unit}")
        self._spin.setValue(0.0)
        self._spin.valueChanged.connect(self._on_spin)
        layout.addWidget(self._spin)

    def _on_slider(self, v: int):
        if self._blocking:
            return
        val = self._min + (self._max - self._min) * v / SLIDER_STEPS
        self._blocking = True
        self._spin.setValue(val)
        self._blocking = False
        self.value_changed.emit(val)

    def _on_spin(self, val: float):
        if self._blocking:
            return
        p = int((val - self._min) / (self._max - self._min) * SLIDER_STEPS)
        self._blocking = True
        self._slider.setValue(p)
        self._blocking = False
        self.value_changed.emit(val)

    def reset(self):
        self._spin.setValue(0.0)

    def value(self) -> float:
        return self._spin.value()


# ─────────────────────────────────────────────────────────────────────────────
#  单关节面板（每个 Tab 一个）
# ─────────────────────────────────────────────────────────────────────────────
class JointPanel(QWidget):
    # 状态变化时通知主窗口更新 Tab 标题颜色
    status_changed = pyqtSignal(int)   # DiagnosticStatus level

    def __init__(self, ros_node: MotorRosNode, ns: str,
                 pos_min: float = POS_MIN, pos_max: float = POS_MAX):
        super().__init__()
        self._ros     = ros_node
        self._busy    = False
        self._level   = -1
        self._pos_min = pos_min
        self._pos_max = pos_max

        ros_node.on_joint_state  = self._update_joint_state
        ros_node.on_status       = self._update_status
        ros_node.on_mode         = self._update_mode
        ros_node.on_service_done = self._on_service_done

        lay = QVBoxLayout(self)
        lay.setSpacing(8)
        lay.setContentsMargins(8, 8, 8, 8)

        lay.addWidget(self._build_status_panel())
        lay.addWidget(self._build_service_panel())
        lay.addWidget(self._build_slider_panel())
        lay.addWidget(self._build_feedback_panel())

        self._lbl_statusbar = QLabel()
        self._lbl_statusbar.setStyleSheet("color: #555; font-size: 11px;")
        lay.addWidget(self._lbl_statusbar)

        self._ip_timer = QTimer()
        self._ip_timer.setInterval(10)
        self._ip_timer.timeout.connect(
            lambda: self._ros.publish("pos", self._sl_pos.value()))

        ros_node.connect(ns)

    # ── 状态面板 ──────────────────────────────────────────────────────────────
    def _build_status_panel(self) -> QGroupBox:
        box = QGroupBox("驱动器状态")
        lay = QHBoxLayout(box)

        self._lbl_level = QLabel("等待")
        self._lbl_level.setFixedWidth(50)
        self._lbl_level.setAlignment(Qt.AlignCenter)
        self._lbl_level.setStyleSheet(
            "background:#7f8c8d; color:white; border-radius:4px; padding:2px 4px;")
        lay.addWidget(self._lbl_level)

        self._lbl_status_msg = QLabel("等待 ~/status …")
        lay.addWidget(self._lbl_status_msg)
        lay.addStretch()

        lay.addWidget(QLabel("模式:"))
        self._lbl_mode = QLabel("NONE")
        f = QFont(); f.setBold(True)
        self._lbl_mode.setFont(f)
        self._lbl_mode.setFixedWidth(42)
        lay.addWidget(self._lbl_mode)

        lay.addWidget(self._vsep())
        lay.addWidget(QLabel("位置:"))
        self._lbl_cur_pos = QLabel("—")
        self._lbl_cur_pos.setFixedWidth(90)
        lay.addWidget(self._lbl_cur_pos)

        return box

    # ── 服务按钮面板 ──────────────────────────────────────────────────────────
    def _build_service_panel(self) -> QGroupBox:
        box = QGroupBox("服务控制")
        grid = QGridLayout(box)
        grid.setSpacing(6)

        def btn(text, srv, row, col, color=None):
            b = QPushButton(text)
            if color:
                b.setStyleSheet(f"background:{color}; color:white;")
            b.clicked.connect(lambda: self._call_srv(srv))
            grid.addWidget(b, row, col)

        btn("使能",        "enable",        0, 0, "#27ae60")
        btn("禁用",        "disable",       0, 1, "#e74c3c")
        btn("故障复位",    "recover",       0, 2, "#8e44ad")

        btn("位置模式 PP", "position_mode", 1, 0)
        btn("速度模式 PV", "velocity_mode", 1, 1)
        btn("力矩模式 PT", "torque_mode",   1, 2)
        btn("插补模式 IP", "ip_mode",       1, 3)

        btn("硬件回零",    "homing",        2, 0)
        btn("软件清零",    "set_home",      2, 1)

        return box

    # ── 滑杆面板 ──────────────────────────────────────────────────────────────
    def _build_slider_panel(self) -> QGroupBox:
        box = QGroupBox("运动指令（话题实时发布，拖动即生效）")
        lay = QVBoxLayout(box)
        lay.setSpacing(10)

        self._sl_pos = SliderGroup("cmd_pos  位置", self._pos_min, self._pos_max, "rad")
        self._sl_vel = SliderGroup("cmd_vel  速度", VEL_MIN, VEL_MAX, "rad/s")
        self._sl_eff = SliderGroup("cmd_eff  力矩", EFF_MIN, EFF_MAX, "Nm")

        self._sl_pos.value_changed.connect(lambda v: self._ros.publish("pos", v))
        self._sl_vel.value_changed.connect(lambda v: self._ros.publish("vel", v))
        self._sl_eff.value_changed.connect(lambda v: self._ros.publish("eff", v))

        lay.addWidget(self._sl_pos)
        lay.addWidget(self._sl_vel)
        lay.addWidget(self._sl_eff)

        btn_row = QHBoxLayout()
        for label, sl in [("位置归零", self._sl_pos),
                           ("速度归零", self._sl_vel),
                           ("力矩归零", self._sl_eff)]:
            b = QPushButton(label)
            b.clicked.connect(sl.reset)
            btn_row.addWidget(b)
        lay.addLayout(btn_row)

        return box

    # ── 反馈面板 ──────────────────────────────────────────────────────────────
    def _build_feedback_panel(self) -> QGroupBox:
        box = QGroupBox("joint_states 反馈")
        lay = QHBoxLayout(box)

        for attr, lbl, unit in [("_fb_pos", "位置", "rad"),
                                 ("_fb_vel", "速度", "rad/s"),
                                 ("_fb_eff", "力矩", "Nm")]:
            lay.addWidget(QLabel(f"{lbl} ({unit}):"))
            label = QLabel("+0.0000")
            label.setMinimumWidth(110)
            setattr(self, attr, label)
            lay.addWidget(label)
            if attr != "_fb_eff":
                lay.addWidget(self._vsep())

        return box

    # ── 回调 ─────────────────────────────────────────────────────────────────
    def _call_srv(self, name: str):
        if self._busy:
            self._lbl_statusbar.setText("上一条服务仍在处理，请稍候")
            return
        self._busy = True
        self._lbl_statusbar.setText(f"调用 ~/{name} …")
        self._ros.call_service(name)

    def _on_service_done(self, success: bool, message: str):
        self._busy = False
        icon = "✓" if success else "✗"
        self._lbl_statusbar.setText(f"{icon}  {message}")

    def _update_joint_state(self, pos: float, vel: float, eff: float):
        self._fb_pos.setText(f"{pos:+.4f}")
        self._fb_vel.setText(f"{vel:+.4f}")
        self._fb_eff.setText(f"{eff:+.4f}")
        self._lbl_cur_pos.setText(f"{pos:+.4f} rad")

    def _update_status(self, level: int, message: str):
        color, text = _LEVEL_STYLE.get(level, ("#7f8c8d", "未知"))
        self._lbl_level.setText(text)
        self._lbl_level.setStyleSheet(
            f"background:{color}; color:white; border-radius:4px; padding:2px 4px;")
        self._lbl_status_msg.setText(message)
        if level != self._level:
            self._level = level
            self.status_changed.emit(level)

    def _update_mode(self, mode: str):
        self._lbl_mode.setText(mode)
        if mode == "IP" and not self._ip_timer.isActive():
            self._ip_timer.start()
            self._lbl_statusbar.setText("IP 模式：已启动 10ms 位置定时推送")
        elif mode != "IP" and self._ip_timer.isActive():
            self._ip_timer.stop()

    @staticmethod
    def _vsep() -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        return line


# ─────────────────────────────────────────────────────────────────────────────
#  主窗口（Tab 管理三个关节）
# ─────────────────────────────────────────────────────────────────────────────
class MotorTestWindow(QMainWindow):
    def __init__(self, ros_nodes: list):
        super().__init__()
        self.setWindowTitle("arm_motor_node 多关节调试面板")
        self.setMinimumSize(720, 580)

        self._tabs = QTabWidget()
        self._panels = []

        for (ns, label, pos_min, pos_max), ros_node in zip(JOINTS, ros_nodes):
            panel = JointPanel(ros_node, ns, pos_min, pos_max)
            self._panels.append(panel)
            idx = self._tabs.addTab(panel, label)
            panel.status_changed.connect(
                lambda level, i=idx: self._on_tab_status(i, level))

        self.setCentralWidget(self._tabs)

    def _on_tab_status(self, idx: int, level: int):
        label = JOINTS[idx][1]  # (ns, label, pos_min, pos_max)
        tab_bar = self._tabs.tabBar()
        tab_bar.setTabTextColor(idx, Qt.black)  # reset first
        # 直接在 tab text 前加颜色标记
        status_char = {
            DiagnosticStatus.OK:    "● ",
            DiagnosticStatus.WARN:  "● ",
            DiagnosticStatus.ERROR: "● ",
        }.get(level, "")
        self._tabs.setTabText(idx, f"{status_char}{label}")
        # 通过 stylesheet 给 tab bar 着色（只影响对应 tab）
        colors = {
            DiagnosticStatus.OK:    "#27ae60",
            DiagnosticStatus.WARN:  "#f39c12",
            DiagnosticStatus.ERROR: "#e74c3c",
        }
        color = colors.get(level, "#7f8c8d")
        tab_bar.setTabTextColor(idx, __import__('PyQt5.QtGui', fromlist=['QColor']).QColor(color))


# ─────────────────────────────────────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init(args=sys.argv)
    ros_nodes = [MotorRosNode() for _ in JOINTS]

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MotorTestWindow(ros_nodes)
    window.show()

    ros_timer = QTimer()
    ros_timer.setInterval(10)
    ros_timer.timeout.connect(
        lambda: [rclpy.spin_once(n, timeout_sec=0) for n in ros_nodes])
    ros_timer.start()

    exit_code = app.exec_()

    ros_timer.stop()
    for node in ros_nodes:
        node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
