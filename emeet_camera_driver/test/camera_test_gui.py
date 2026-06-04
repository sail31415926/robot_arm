#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_test_gui.py — EMEET PTZ 摄像头调试 GUI

功能：
  - 实时视频预览（订阅 /camera/image_raw/compressed）
  - 位置控制：滑杆 + SpinBox 双向同步，100 Hz 连续发送 JointTrajectory（rad）
  - 速度控制：Pan / Tilt 滑杆（松手归零）+ 方向按钮，发布 cmd_vel（rad/s）
  - 复位按钮：发送 joint4=0, joint6=0 轨迹
  - 关节状态显示：订阅 /joint_states，差分计算速度（rad / rad/s）
  - 相机参数调节：Brightness / Exposure 滑杆（调用 set_parameters 服务）

话题：
  订阅  /camera/image_raw/compressed   sensor_msgs/CompressedImage  BEST_EFFORT
  订阅  /joint_states                  sensor_msgs/JointState       BEST_EFFORT
  发布  ~/joint_trajectory             trajectory_msgs/JointTrajectory
  发布  ~/cmd_vel                      geometry_msgs/Twist（angular.z=pan, angular.y=tilt）
  服务  /emeet_camera_node/set_parameters

启动方式：
  ros2 run emeet_camera_driver camera_test_gui.py
  ros2 launch emeet_camera_driver test_all.launch.py

版本: 1.0  日期: 2026-05-29  Copyright (c) 2026 EMEET
"""
import sys
import math
import time
import cv2
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
from sensor_msgs.msg import Image, CompressedImage, JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QLabel, QGroupBox, QSlider,
                             QDoubleSpinBox, QFormLayout, QGridLayout)
    from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
    from PyQt5.QtGui import QImage, QPixmap
except ImportError:
    print("Error: PyQt5 not installed. Run 'pip3 install PyQt5'")
    sys.exit(1)

# Position slider: 1 count = 0.001 rad
_POS_SCALE = 1000
# Velocity: all internal values in rad/s; driver converts to deg/s internally
_VEL_MAX   = math.radians(100.0)  # ≈ 1.745 rad/s  (driver hard limit)
_VEL_STEPS = 100                  # slider integer range ±100 → ±_VEL_MAX
_BTN_SPEED = math.radians(30.0)   # ≈ 0.524 rad/s


class RosWorker(QObject):
    pixmap_signal = pyqtSignal(np.ndarray)
    joint_signal = pyqtSignal(dict)

    def __init__(self, node):
        super().__init__()
        self.node = node
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.img_sub = self.node.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.img_cb, qos_profile)
        # BEST_EFFORT: compatible with both camera_driver_node (RELIABLE) and
        # joint_state_broadcaster (sensor_data / BEST_EFFORT)
        joint_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.joint_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_cb, joint_qos)

    def img_cb(self, msg):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if bgr is None or bgr.size == 0:
                return
            self.pixmap_signal.emit(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        except Exception as e:
            print(f"Image processing error: {e}")

    def joint_cb(self, msg):
        result = {name: msg.position[i]
                  for i, name in enumerate(msg.name) if i < len(msg.position)}
        if result:
            self.joint_signal.emit(result)


class EmeetIntegratedGui(QMainWindow):
    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init()
        self.node = Node('emeet_camera_gui_client')

        self.traj_pub = self.node.create_publisher(
            JointTrajectory, '/emeet_camera_node/joint_trajectory', 10)
        self.vel_pub = self.node.create_publisher(
            Twist, '/emeet_camera_node/cmd_vel', 10)
        self.param_client = self.node.create_client(
            SetParameters, '/emeet_camera_node/set_parameters')

        self._pressed_keys = set()
        self._pan_vel  = 0.0   # rad/s
        self._tilt_vel = 0.0   # rad/s
        self._pos_dirty = False
        self._last_joint_time = 0.0
        self._prev_pos = {}    # for velocity differentiation
        self._prev_pos_time = 0.0
        self.params_dirty = False

        self.init_ui()

        self.worker = RosWorker(self.node)
        self.worker.pixmap_signal.connect(self.on_image_received)
        self.worker.joint_signal.connect(self.on_joint_received)

        self.ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self.node), daemon=True)
        self.ros_thread.start()

        # 100 Hz position publisher
        self.pos_pub_timer = QTimer()
        self.pos_pub_timer.setInterval(10)
        self.pos_pub_timer.timeout.connect(self.publish_pos_if_dirty)
        self.pos_pub_timer.start()

        # 20 Hz velocity publisher
        self.vel_timer = QTimer()
        self.vel_timer.setInterval(50)
        self.vel_timer.timeout.connect(self.publish_vel)
        self.vel_timer.start()

        # 5 Hz camera parameter sync
        self.param_timer = QTimer()
        self.param_timer.setInterval(200)
        self.param_timer.timeout.connect(self.sync_parameters)
        self.param_timer.start()

        self.pos_timeout_timer = QTimer()
        self.pos_timeout_timer.setInterval(2000)
        self.pos_timeout_timer.timeout.connect(self.check_pos_timeout)
        self.pos_timeout_timer.start()

    # ------------------------------------------------------------------ UI --

    def init_ui(self):
        self.setWindowTitle("EMEET PTZ Controller (Topic Mode)")
        self.setFixedSize(1280, 590)

        central = QWidget()
        self.setCentralWidget(central)
        main = QHBoxLayout(central)
        main.setSpacing(8)
        main.setContentsMargins(8, 8, 8, 8)

        # ── Left: video + status ──────────────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(3)

        self.video_label = QLabel("Waiting for /camera/image_raw/compressed ...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setFixedSize(900, 506)
        self.video_label.setStyleSheet("background-color: black; border: 2px solid #555;")
        left.addWidget(self.video_label)

        self.pos_label = QLabel("Position: waiting for data...")
        self.pos_label.setStyleSheet("font-size: 12px; font-weight: bold; color: #008800;")
        left.addWidget(self.pos_label)

        self.status_bar = QLabel("Ready")
        self.status_bar.setStyleSheet("color: blue; border-top: 1px solid #ccc; padding: 2px;")
        left.addWidget(self.status_bar)

        main.addLayout(left)

        # ── Right: controls (fixed width) ────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(4)
        right.addWidget(self._build_angle_group())
        right.addWidget(self._build_vel_group())
        right.addWidget(self._build_cam_group())

        right_widget = QWidget()
        right_widget.setFixedWidth(352)
        right_widget.setLayout(right)
        main.addWidget(right_widget)

    def _build_angle_group(self):
        group = QGroupBox("Angle Control (Topic: ~/joint_trajectory)")
        form = QFormLayout(group)
        form.setContentsMargins(6, 4, 6, 4)
        form.setSpacing(3)

        self.pan_input = QDoubleSpinBox()
        self.pan_input.setRange(-math.pi, math.pi)
        self.pan_input.setSuffix(" rad")
        self.pan_input.setSingleStep(0.01)
        self.pan_input.setDecimals(3)
        self.pan_slider = QSlider(Qt.Horizontal)
        self.pan_slider.setRange(-int(math.pi * _POS_SCALE), int(math.pi * _POS_SCALE))
        self.pan_slider.setValue(0)
        form.addRow("Pan (joint4):", self.pan_input)
        form.addRow("", self.pan_slider)

        self.tilt_input = QDoubleSpinBox()
        self.tilt_input.setRange(-math.pi / 4, math.pi / 4)
        self.tilt_input.setSuffix(" rad")
        self.tilt_input.setSingleStep(0.01)
        self.tilt_input.setDecimals(3)
        self.tilt_slider = QSlider(Qt.Horizontal)
        self.tilt_slider.setRange(-int(math.pi / 4 * _POS_SCALE),
                                   int(math.pi / 4 * _POS_SCALE))
        self.tilt_slider.setValue(0)
        form.addRow("Tilt (joint6):", self.tilt_input)
        form.addRow("", self.tilt_slider)

        btn_row = QHBoxLayout()
        self.btn_goto = QPushButton("Send Trajectory")
        self.btn_goto.clicked.connect(self.on_goto_clicked)
        self.btn_home = QPushButton("⌂ Home (0,0)")
        self.btn_home.clicked.connect(self.on_home_clicked)
        btn_row.addWidget(self.btn_goto)
        btn_row.addWidget(self.btn_home)
        form.addRow(btn_row)

        self.pan_slider.valueChanged.connect(self._on_pan_slider_changed)
        self.pan_input.valueChanged.connect(self._on_pan_spin_changed)
        self.tilt_slider.valueChanged.connect(self._on_tilt_slider_changed)
        self.tilt_input.valueChanged.connect(self._on_tilt_spin_changed)

        return group

    def _build_vel_group(self):
        group = QGroupBox("Velocity Control (Topic: ~/cmd_vel)")
        vbox = QVBoxLayout(group)
        vbox.setContentsMargins(6, 4, 6, 4)
        vbox.setSpacing(4)

        # velocity sliders
        slider_form = QFormLayout()
        slider_form.setSpacing(3)

        self.vel_pan_slider = QSlider(Qt.Horizontal)
        self.vel_pan_slider.setRange(-_VEL_STEPS, _VEL_STEPS)
        self.vel_pan_slider.setValue(0)
        self.vel_pan_label = QLabel("0.000 rad/s")
        self.vel_pan_label.setFixedWidth(76)
        pan_row = QHBoxLayout()
        pan_row.addWidget(self.vel_pan_slider)
        pan_row.addWidget(self.vel_pan_label)
        slider_form.addRow("Pan:", pan_row)

        self.vel_tilt_slider = QSlider(Qt.Horizontal)
        self.vel_tilt_slider.setRange(-_VEL_STEPS, _VEL_STEPS)
        self.vel_tilt_slider.setValue(0)
        self.vel_tilt_label = QLabel("0.000 rad/s")
        self.vel_tilt_label.setFixedWidth(76)
        tilt_row = QHBoxLayout()
        tilt_row.addWidget(self.vel_tilt_slider)
        tilt_row.addWidget(self.vel_tilt_label)
        slider_form.addRow("Tilt:", tilt_row)

        vbox.addLayout(slider_form)

        self.vel_pan_slider.valueChanged.connect(self._on_vel_pan_changed)
        self.vel_pan_slider.sliderReleased.connect(self._on_vel_pan_released)
        self.vel_tilt_slider.valueChanged.connect(self._on_vel_tilt_changed)
        self.vel_tilt_slider.sliderReleased.connect(self._on_vel_tilt_released)

        # compact D-pad using grid layout
        self.btn_up    = QPushButton("⬆")
        self.btn_down  = QPushButton("⬇")
        self.btn_left  = QPushButton("⬅")
        self.btn_right = QPushButton("➡")
        self.btn_stop  = QPushButton("⏹ Stop")
        for btn in (self.btn_up, self.btn_down, self.btn_left,
                    self.btn_right, self.btn_stop):
            btn.setFixedSize(52, 28)

        dpad = QGridLayout()
        dpad.setSpacing(3)
        dpad.addWidget(self.btn_up,    0, 1, Qt.AlignCenter)
        dpad.addWidget(self.btn_left,  1, 0, Qt.AlignCenter)
        dpad.addWidget(self.btn_stop,  1, 1, Qt.AlignCenter)
        dpad.addWidget(self.btn_right, 1, 2, Qt.AlignCenter)
        dpad.addWidget(self.btn_down,  2, 1, Qt.AlignCenter)
        vbox.addLayout(dpad)

        self.btn_up.pressed.connect(lambda: self.on_dir_pressed('up'))
        self.btn_down.pressed.connect(lambda: self.on_dir_pressed('down'))
        self.btn_left.pressed.connect(lambda: self.on_dir_pressed('left'))
        self.btn_right.pressed.connect(lambda: self.on_dir_pressed('right'))
        self.btn_up.released.connect(lambda: self.on_dir_released('up'))
        self.btn_down.released.connect(lambda: self.on_dir_released('down'))
        self.btn_left.released.connect(lambda: self.on_dir_released('left'))
        self.btn_right.released.connect(lambda: self.on_dir_released('right'))
        self.btn_stop.clicked.connect(self.on_stop_clicked)

        return group

    def _build_cam_group(self):
        group = QGroupBox("Camera Parameters")
        form = QFormLayout(group)
        form.setContentsMargins(6, 4, 6, 4)
        form.setSpacing(3)

        self.slider_bright = QSlider(Qt.Horizontal)
        self.slider_bright.setRange(0, 255)
        self.slider_bright.setValue(128)
        self.slider_bright.valueChanged.connect(self.mark_params_dirty)
        form.addRow("Brightness:", self.slider_bright)

        self.slider_exp = QSlider(Qt.Horizontal)
        self.slider_exp.setRange(1, 500)
        self.slider_exp.setValue(156)
        self.slider_exp.valueChanged.connect(self.mark_params_dirty)
        form.addRow("Exposure:", self.slider_exp)

        return group

    # --------------------------------------------------------- image/joint --

    def on_image_received(self, rgb_image):
        h, w, ch = rgb_image.shape
        qt_img = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img)
        scaled = pixmap.scaled(self.video_label.width(), self.video_label.height(),
                               Qt.KeepAspectRatio, Qt.FastTransformation)
        self.video_label.setPixmap(scaled)

    def on_joint_received(self, joint_map):
        now = time.time()
        dt = now - self._prev_pos_time if self._prev_pos_time > 0 else 0.0
        self._last_joint_time = now

        lines = []
        for jname, label in [('Joint4', 'Pan '), ('Joint6', 'Tilt')]:
            pos = joint_map.get(jname)
            if pos is None:
                continue
            # differentiate position to get velocity (rad/s)
            if dt > 0.001 and jname in self._prev_pos:
                vel = (pos - self._prev_pos[jname]) / dt
                vel_str = f"{vel:+.3f} rad/s"
            else:
                vel_str = "-- rad/s"
            lines.append(f"{label}(/{jname}): pos={pos:.3f} rad  vel={vel_str}")

        self._prev_pos = {k: v for k, v in joint_map.items()}
        self._prev_pos_time = now

        if lines:
            self.pos_label.setText("\n".join(lines))
            self.pos_label.setStyleSheet(
                "font-size: 14px; font-weight: bold; color: #008800;")

    def check_pos_timeout(self):
        if self._last_joint_time == 0.0:
            self.pos_label.setText("Position: waiting for /joint_states ...")
            self.pos_label.setStyleSheet(
                "font-size: 16px; font-weight: bold; color: #888800;")
        elif time.time() - self._last_joint_time > 2.0:
            self.pos_label.setText("Position: DATA TIMEOUT!")
            self.pos_label.setStyleSheet(
                "font-size: 16px; font-weight: bold; color: #cc0000;")

    # ------------------------------------------------- position slider sync --

    def _on_pan_slider_changed(self, val):
        self.pan_input.blockSignals(True)
        self.pan_input.setValue(val / _POS_SCALE)
        self.pan_input.blockSignals(False)
        self._pos_dirty = True

    def _on_pan_spin_changed(self, val):
        self.pan_slider.blockSignals(True)
        self.pan_slider.setValue(int(round(val * _POS_SCALE)))
        self.pan_slider.blockSignals(False)
        self._pos_dirty = True

    def _on_tilt_slider_changed(self, val):
        self.tilt_input.blockSignals(True)
        self.tilt_input.setValue(val / _POS_SCALE)
        self.tilt_input.blockSignals(False)
        self._pos_dirty = True

    def _on_tilt_spin_changed(self, val):
        self.tilt_slider.blockSignals(True)
        self.tilt_slider.setValue(int(round(val * _POS_SCALE)))
        self.tilt_slider.blockSignals(False)
        self._pos_dirty = True

    def publish_pos_if_dirty(self):
        if not self._pos_dirty:
            return
        self._pos_dirty = False
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["Joint4", "Joint6"]
        point = JointTrajectoryPoint()
        point.positions = [self.pan_input.value(), self.tilt_input.value()]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 100 ms lookahead
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

    def on_goto_clicked(self):
        self._pan_vel  = 0.0
        self._tilt_vel = 0.0
        self.publish_stop_vel()
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["Joint4", "Joint6"]
        point = JointTrajectoryPoint()
        point.positions = [self.pan_input.value(), self.tilt_input.value()]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)
        self.status_bar.setText(
            f"Trajectory sent: joint4={self.pan_input.value():.3f} rad, "
            f"joint6={self.tilt_input.value():.3f} rad")

    # --------------------------------------------------- velocity sliders --

    def _on_vel_pan_changed(self, val):
        self._pan_vel = val / _VEL_STEPS * _VEL_MAX
        self.vel_pan_label.setText(f"{self._pan_vel:.3f} rad/s")

    def _on_vel_pan_released(self):
        self.vel_pan_slider.setValue(0)   # triggers _on_vel_pan_changed → _pan_vel=0
        self.publish_stop_vel()

    def _on_vel_tilt_changed(self, val):
        self._tilt_vel = val / _VEL_STEPS * _VEL_MAX
        self.vel_tilt_label.setText(f"{self._tilt_vel:.3f} rad/s")

    def _on_vel_tilt_released(self):
        self.vel_tilt_slider.setValue(0)
        self.publish_stop_vel()

    # --------------------------------------------------- direction buttons --

    def on_home_clicked(self):
        self._pan_vel  = 0.0
        self._tilt_vel = 0.0
        self.publish_stop_vel()
        # reset spinboxes and sliders
        self.pan_input.blockSignals(True);  self.pan_input.setValue(0.0);  self.pan_input.blockSignals(False)
        self.pan_slider.blockSignals(True); self.pan_slider.setValue(0);   self.pan_slider.blockSignals(False)
        self.tilt_input.blockSignals(True);  self.tilt_input.setValue(0.0);  self.tilt_input.blockSignals(False)
        self.tilt_slider.blockSignals(True); self.tilt_slider.setValue(0);   self.tilt_slider.blockSignals(False)
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["Joint4", "Joint6"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)
        self.status_bar.setText("Home: joint4=0.000 rad, joint6=0.000 rad")

    def on_dir_pressed(self, direction):
        self._pressed_keys.add(direction)
        self._update_vel_from_keys()

    def on_dir_released(self, direction):
        self._pressed_keys.discard(direction)
        self._update_vel_from_keys()

    def on_stop_clicked(self):
        self._pressed_keys.clear()
        self._pan_vel  = 0.0
        self._tilt_vel = 0.0
        # reset velocity sliders without retriggering sliderReleased
        self.vel_pan_slider.blockSignals(True)
        self.vel_pan_slider.setValue(0)
        self.vel_pan_slider.blockSignals(False)
        self.vel_pan_label.setText("0.00 rad/s")
        self.vel_tilt_slider.blockSignals(True)
        self.vel_tilt_slider.setValue(0)
        self.vel_tilt_slider.blockSignals(False)
        self.vel_tilt_label.setText("0.00 rad/s")
        self.publish_stop_vel()

    def _update_vel_from_keys(self):
        self._pan_vel  = 0.0
        self._tilt_vel = 0.0
        if 'left'  in self._pressed_keys: self._pan_vel  += _BTN_SPEED
        if 'right' in self._pressed_keys: self._pan_vel  -= _BTN_SPEED
        if 'up'    in self._pressed_keys: self._tilt_vel += _BTN_SPEED
        if 'down'  in self._pressed_keys: self._tilt_vel -= _BTN_SPEED

    # --------------------------------------------------------- publish vel --

    def publish_vel(self):
        if abs(self._pan_vel) < 0.01 and abs(self._tilt_vel) < 0.01:  # < 0.01 rad/s
            return
        msg = Twist()
        msg.angular.z = float(self._pan_vel)   # rad/s
        msg.angular.y = float(self._tilt_vel)  # rad/s
        self.vel_pub.publish(msg)

    def publish_stop_vel(self):
        msg = Twist()
        self.vel_pub.publish(msg)

    # --------------------------------------------------------- cam params --

    def mark_params_dirty(self):
        self.params_dirty = True

    def sync_parameters(self):
        if not self.params_dirty:
            return
        if not self.param_client.wait_for_service(timeout_sec=0.1):
            return
        self.params_dirty = False
        req = SetParameters.Request()
        req.parameters = [
            ParamMsg(name='brightness', value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER,
                integer_value=self.slider_bright.value())),
            ParamMsg(name='exposure_value', value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER,
                integer_value=self.slider_exp.value())),
        ]
        future = self.param_client.call_async(req)
        future.add_done_callback(lambda f: None)

    def closeEvent(self, event):
        self.pos_pub_timer.stop()
        self.vel_timer.stop()
        self.param_timer.stop()
        self.pos_timeout_timer.stop()
        rclpy.shutdown()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EmeetIntegratedGui()
    window.show()
    sys.exit(app.exec_())
