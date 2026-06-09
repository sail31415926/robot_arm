#!/usr/bin/env python3
"""
@file   motor_state_control_gui.py
@brief  Motor State Control GUI for arm_motor_node (PyQt5 + rclpy)
@version 1.0
@date   2026-06-09

Only three actions are kept: Enable / Disable / Recover.
Each button acts on all three motors (J1/J2/J3) at once.

Architecture:
    Each joint owns an independent MotorRosNode connection.
    A QTimer calls spin_once() on the three nodes every 10ms on the Qt main
    thread, so all ROS2 callbacks run on the main thread with no contention.

Run:
    ros2 run robot_arm_driver motor_state_control_gui

@copyright Copyright (c) 2026 EMEET
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QFrame
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

# ── Default targets (ns, label) ──────────────────────────────────────────────
# Standalone / motor.launch.py topology: three independent arm_motor_node.
# Override via ROS parameters `namespaces` / `labels`, e.g. for real.launch.py
# which exposes a single integrated arm_node:
#   ros2 run robot_arm_driver motor_state_control_gui \
#     --ros-args -p namespaces:='[/arm_node]' -p labels:='[ARM]'
DEFAULT_TARGETS = [
    ("/joint1/arm_motor_node", "J1"),
    ("/joint2/arm_motor_node", "J2"),
    ("/joint3/arm_motor_node", "J3"),
]


def resolve_targets():
    """Read target namespaces/labels from ROS parameters, falling back to defaults."""
    cfg = rclpy.create_node("motor_state_control_gui_cfg")
    cfg.declare_parameter("namespaces", [t[0] for t in DEFAULT_TARGETS])
    cfg.declare_parameter("labels",     [t[1] for t in DEFAULT_TARGETS])
    namespaces = list(cfg.get_parameter("namespaces").value)
    labels     = list(cfg.get_parameter("labels").value)
    cfg.destroy_node()
    if len(labels) != len(namespaces):
        # derive a label from each namespace if labels are missing/mismatched
        labels = [ns.strip("/").split("/")[0] or "node" for ns in namespaces]
    return list(zip(namespaces, labels))

_LEVEL_STYLE = {
    DiagnosticStatus.OK:    ("#27ae60", "OK"),
    DiagnosticStatus.WARN:  ("#f39c12", "WARN"),
    DiagnosticStatus.ERROR: ("#e74c3c", "ERROR"),
}


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 node (one independent instance per joint)
# ─────────────────────────────────────────────────────────────────────────────
class MotorRosNode(Node):
    _counter = 0

    def __init__(self):
        MotorRosNode._counter += 1
        super().__init__(f"motor_state_control_gui_{MotorRosNode._counter}")
        self._gui_subs    = []
        self._gui_clients = {}

        self.on_status       = None
        self.on_mode         = None
        self.on_service_done = None

    def connect(self, ns: str):
        ns = ns.rstrip("/")

        for s in self._gui_subs:
            self.destroy_subscription(s)
        self._gui_subs.clear()

        for c in self._gui_clients.values():
            self.destroy_client(c)
        self._gui_clients.clear()

        def t(suffix):
            return f"{ns}/{suffix}"

        self._gui_subs.append(self.create_subscription(
            DiagnosticStatus, t("status"), self._cb_status, 10))
        self._gui_subs.append(self.create_subscription(
            String,           t("mode"),   self._cb_mode,   10))

        for name in ["enable", "disable", "recover"]:
            self._gui_clients[name] = self.create_client(Trigger, t(name))

        self.get_logger().info(f"Connected to: {ns}")
        return ns

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
                self.on_service_done(False, f"Unknown service: {name}")
            return
        if not client.service_is_ready():
            if self.on_service_done:
                self.on_service_done(False, f"~/{name} unavailable — is the motor node running?")
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


# ─────────────────────────────────────────────────────────────────────────────
#  Single-joint status row (one row per joint)
# ─────────────────────────────────────────────────────────────────────────────
class JointStatusRow(QWidget):
    def __init__(self, ros_node: MotorRosNode, ns: str, label: str):
        super().__init__()
        self._ros   = ros_node
        self._label = label
        self._level = -1
        self.on_result = None   # set by the main window to aggregate joint results

        ros_node.on_status       = self._update_status
        ros_node.on_mode         = self._update_mode
        ros_node.on_service_done = self._on_service_done

        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)

        name = QLabel(label)
        name.setFixedWidth(40)
        f = QFont(); f.setBold(True)
        name.setFont(f)
        lay.addWidget(name)

        self._lbl_level = QLabel("WAIT")
        self._lbl_level.setFixedWidth(60)
        self._lbl_level.setAlignment(Qt.AlignCenter)
        self._lbl_level.setStyleSheet(
            "background:#7f8c8d; color:white; border-radius:4px; padding:2px 4px;")
        lay.addWidget(self._lbl_level)

        self._lbl_status_msg = QLabel("Waiting for ~/status …")
        lay.addWidget(self._lbl_status_msg, 1)

        lay.addWidget(QLabel("Mode:"))
        self._lbl_mode = QLabel("NONE")
        self._lbl_mode.setFixedWidth(42)
        lay.addWidget(self._lbl_mode)

        ros_node.connect(ns)

    def _update_status(self, level: int, message: str):
        color, text = _LEVEL_STYLE.get(level, ("#7f8c8d", "UNKNOWN"))
        self._lbl_level.setText(text)
        self._lbl_level.setStyleSheet(
            f"background:{color}; color:white; border-radius:4px; padding:2px 4px;")
        self._lbl_status_msg.setText(message)
        self._level = level

    def _update_mode(self, mode: str):
        self._lbl_mode.setText(mode)

    def _on_service_done(self, success: bool, message: str):
        # update this row's own message
        icon = "✓" if success else "✗"
        self._lbl_status_msg.setText(f"{icon}  {message}")
        # report to the main window for aggregation across the three joints
        if self.on_result:
            self.on_result(self._label, success, message)


# ─────────────────────────────────────────────────────────────────────────────
#  Main window
# ─────────────────────────────────────────────────────────────────────────────
class MotorStateControlWindow(QMainWindow):
    def __init__(self, ros_nodes: list, targets: list):
        super().__init__()
        self.setWindowTitle("Motor State Control")
        self.setMinimumSize(640, 320)

        self._ros_nodes = ros_nodes
        self._targets   = targets
        self._cur_srv   = None   # action currently in progress
        self._pending   = 0      # number of joints awaiting a response
        self._results   = []     # [(label, success, message), …]

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setSpacing(10)
        lay.setContentsMargins(12, 12, 12, 12)

        # human-readable list of targets, e.g. "J1 / J2 / J3" or "ARM"
        self._targets_str = " / ".join(label for _, label in targets)

        # ── action buttons ───────────────────────────────────────────────────
        lay.addWidget(self._build_action_panel())

        # ── per-joint status ─────────────────────────────────────────────────
        lay.addWidget(self._build_status_panel(ros_nodes))

        # ── bottom status bar ────────────────────────────────────────────────
        self._lbl_statusbar = QLabel("Ready")
        self._lbl_statusbar.setStyleSheet("color: #555; font-size: 12px;")
        lay.addWidget(self._lbl_statusbar)

        lay.addStretch()
        self.setCentralWidget(central)

    def _build_action_panel(self) -> QGroupBox:
        box = QGroupBox(f"One-Click Control (applies to {self._targets_str})")
        grid = QGridLayout(box)
        grid.setSpacing(10)

        def btn(text, srv, col, color):
            b = QPushButton(text)
            b.setMinimumHeight(64)
            f = QFont(); f.setBold(True); f.setPointSize(13)
            b.setFont(f)
            b.setStyleSheet(f"background:{color}; color:white; border-radius:6px;")
            b.clicked.connect(lambda: self._call_all(srv))
            grid.addWidget(b, 0, col)

        btn("Enable",   "enable",  0, "#27ae60")
        btn("Disable",  "disable", 1, "#e74c3c")
        btn("Recover",  "recover", 2, "#8e44ad")

        return box

    def _build_status_panel(self, ros_nodes: list) -> QGroupBox:
        box = QGroupBox("Joint Status")
        lay = QVBoxLayout(box)
        lay.setSpacing(4)

        self._rows = []
        last = len(self._targets) - 1
        for i, ((ns, label), ros_node) in enumerate(zip(self._targets, ros_nodes)):
            row = JointStatusRow(ros_node, ns, label)
            row.on_result = self._on_joint_result
            self._rows.append(row)
            lay.addWidget(row)
            if i != last:
                line = QFrame()
                line.setFrameShape(QFrame.HLine)
                line.setFrameShadow(QFrame.Sunken)
                lay.addWidget(line)

        return box

    _SRV_NAME = {"enable": "Enable", "disable": "Disable", "recover": "Recover"}

    def _call_all(self, srv: str):
        if self._pending > 0:
            self._lbl_statusbar.setText("Previous command still in progress, please wait …")
            return
        name = self._SRV_NAME.get(srv, srv)
        self._cur_srv = srv
        self._results = []
        self._pending = len(self._ros_nodes)
        self._lbl_statusbar.setText(
            f"Running '{name}' on {self._targets_str} … (0/{self._pending})")
        for node in self._ros_nodes:
            node.call_service(srv)

    def _on_joint_result(self, label: str, success: bool, message: str):
        if self._pending <= 0:
            return  # result not from the current one-click action (e.g. late after timeout)
        self._results.append((label, success, message))
        self._pending -= 1
        name = self._SRV_NAME.get(self._cur_srv, self._cur_srv)
        if self._pending > 0:
            done = len(self._results)
            self._lbl_statusbar.setText(
                f"Running '{name}' on {self._targets_str} … ({done}/{len(self._ros_nodes)})")
            return
        # all returned — summarize
        ok = [l for l, s, _ in self._results if s]
        bad = [(l, m) for l, s, m in self._results if not s]
        if not bad:
            self._lbl_statusbar.setText(f"✓ '{name}' succeeded: {' / '.join(sorted(ok))} all done")
        else:
            detail = "; ".join(f"{l}: {m}" for l, m in bad)
            self._lbl_statusbar.setText(
                f"✗ '{name}' partially failed — {len(ok)}/{len(self._ros_nodes)} ok; failed {detail}")


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init(args=sys.argv)
    targets = resolve_targets()
    ros_nodes = [MotorRosNode() for _ in targets]

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MotorStateControlWindow(ros_nodes, targets)
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
