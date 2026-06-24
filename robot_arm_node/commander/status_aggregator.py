#!/usr/bin/env python3
"""
@file   status_aggregator.py
@brief  状态聚合器 —— 订阅 /joint_states + TF2 → 合成 ArmStatus
@version 1.0
@date   2026-06-09

职责：
  - 维护当前关节位置/速度缓存
  - 通过 TF2 查询末端位姿（ArmPose）
  - 估算末端速度（ArmTwist，数值微分或从 joint_state.velocity 推算）
  - 构建 ArmStatus 消息（供 ArmCommanderNode 周期发布）
  - 提供查询接口：pose / joint_positions / is_moving / arm_at_target

被 ArmCommanderNode 持有，不独立运行。

@copyright Copyright (c) 2026 eMeet
"""

import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer

from robot_arm_interfaces.msg import ArmPose, ArmTwist, ArmStatus

from arm_utils import quat_to_rpy


# ── 常量 ──────────────────────────────────────────────────────────────────────────
JOINT_NAMES       = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
EEF_LINK          = 'tool0'
BASE_FRAME        = 'arm_base_link'
JOINT_STATE_TOPIC = '/joint_states'

# 末端速度估算：保留最近 N 个位姿样本做数值微分
VELOCITY_WINDOW_SIZE = 10    # 100Hz 下 ≈0.1s 窗口
MIN_MOVING_VELOCITY  = 0.001  # m/s，低于此值认为静止


class StatusAggregator:
    """状态聚合器 —— 维护机械臂当前状态的权威来源。

    不继承 Node，共享 ArmCommanderNode 的 ROS 资源。
    提供线程安全的读写接口。
    """

    def __init__(self, node: Node):
        """
        Args:
            node: ArmCommanderNode 实例
        """
        self._node   = node
        self._logger = node.get_logger()

        # ── 关节状态缓存（线程安全）───────────────────────────────────────────
        self._joint_positions = {name: 0.0 for name in JOINT_NAMES}
        self._joint_velocities = {name: 0.0 for name in JOINT_NAMES}
        self._joint_lock = threading.Lock()

        node.create_subscription(
            JointState, JOINT_STATE_TOPIC,
            self._on_joint_state, 10,
        )

        # ── TF2 ───────────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        # ── 位姿历史（用于速度估算）───────────────────────────────────────────
        self._pose_history = deque(maxlen=VELOCITY_WINDOW_SIZE)

        # ── 命令执行状态（由 ArmCommanderNode 更新）───────────────────────────
        self._executing_command_id = 0
        self._command_result       = ArmStatus.RESULT_NONE
        self._current_pose_state   = ArmStatus.POSE_STATE_OBSERVE  # 默认
        self._error_code           = ArmStatus.ERR_NONE
        self._is_moving            = False   # 由 Commander 状态机维护，避免速度微分抖动
        self._at_pose_start        = False   # 由运镜 server 维护：是否已到达运镜起始点
        self._camera_ready         = False   # 机械臂到达运镜起始点→运镜结束期间为 True
        self._is_tracking          = False   # IBVS 跟随激活期间为 True
        self._tracking_img_err     = 0.0     # 当前图像误差（归一化）
        self._tracking_depth_err_m = 0.0     # 当前深度误差（m）

        self._state_lock = threading.Lock()

        # ── 就绪 ──────────────────────────────────────────────────────────────
        self._logger.info('StatusAggregator 就绪')

    # ── 关节状态订阅 ──────────────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        now = self._node.get_clock().now().nanoseconds * 1e-9
        with self._joint_lock:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                if name in self._joint_positions:
                    self._joint_positions[name] = pos
                    self._joint_velocities[name] = vel

        # 记录位姿历史（用于速度估算）
        pose = self._lookup_pose()
        if pose is not None:
            self._pose_history.append((now, pose))

    # ── 末端位姿查询 ──────────────────────────────────────────────────────────────
    @property
    def pose(self) -> ArmPose:
        """当前末端位姿（ArmPose 消息）。"""
        return self._lookup_pose() or self._zero_pose()

    def _lookup_pose(self) -> ArmPose | None:
        """从 TF2 查询末端位姿。"""
        try:
            t = self._tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            return self._transform_to_pose(t.transform)
        except Exception:
            return None

    def _zero_pose(self) -> ArmPose:
        p = ArmPose()
        p.x = p.y = p.z = 0.0
        p.roll = p.pitch = p.yaw = 0.0
        return p

    @staticmethod
    def _transform_to_pose(transform) -> ArmPose:
        """geometry_msgs/Transform → ArmPose。"""
        p = ArmPose()
        p.x = float(transform.translation.x)
        p.y = float(transform.translation.y)
        p.z = float(transform.translation.z)
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        p.roll  = float(math.degrees(roll))
        p.pitch = float(math.degrees(pitch))
        p.yaw   = float(math.degrees(yaw))
        return p

    # ── 末端速度估算 ──────────────────────────────────────────────────────────────
    @property
    def twist(self) -> ArmTwist:
        """当前末端速度（数值微分估算，ArmTwist 消息）。"""
        if len(self._pose_history) < 2:
            return ArmTwist()   # 全零

        t0, p0 = self._pose_history[0]
        t1, p1 = self._pose_history[-1]
        dt = t1 - t0
        if dt < 1e-6:
            return ArmTwist()

        tw = ArmTwist()
        tw.vx     = float((p1.x - p0.x) / dt)
        tw.vy     = float((p1.y - p0.y) / dt)
        tw.vz     = float((p1.z - p0.z) / dt)
        tw.wroll  = float((p1.roll  - p0.roll)  / dt)
        tw.wpitch = float((p1.pitch - p0.pitch) / dt)
        tw.wyaw   = float((p1.yaw   - p0.yaw)   / dt)
        return tw

    # ── 关节状态查询 ──────────────────────────────────────────────────────────────
    @property
    def joint_positions(self) -> dict:
        """当前关节位置 {Joint1: rad, ...}。"""
        with self._joint_lock:
            return dict(self._joint_positions)

    @property
    def joint_velocities(self) -> dict:
        """当前关节速度 {Joint1: rad/s, ...}。"""
        with self._joint_lock:
            return dict(self._joint_velocities)

    def joint_position_list(self, names: list = None) -> list:
        """按指定顺序返回关节位置列表。"""
        if names is None:
            names = JOINT_NAMES
        with self._joint_lock:
            return [self._joint_positions[n] for n in names]

    # ── 运动状态 ──────────────────────────────────────────────────────────────────
    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            return self._is_moving

    def set_moving(self, moving: bool):
        """由 Commander 状态机在 MOVING 进入/退出时调用，避免速度微分抖动。"""
        with self._state_lock:
            self._is_moving = moving

    @property
    def at_pose_start(self) -> bool:
        with self._state_lock:
            return self._at_pose_start

    def set_at_pose_start(self, at_start: bool):
        """由运镜 server 在到达起始点时置 True，新指令开始时（Commander）复位 False。"""
        with self._state_lock:
            self._at_pose_start = at_start

    @property
    def camera_ready(self) -> bool:
        with self._state_lock:
            return self._camera_ready

    def set_camera_ready(self, ready: bool):
        """到达运镜起始点时置 True，运镜结束（或新指令开始）时复位 False。"""
        with self._state_lock:
            self._camera_ready = ready

    def set_tracking(self, tracking: bool, img_err: float, depth_err_m: float):
        """由 TrackTargetServer 在跟随期间持续更新，退出时清零。"""
        with self._state_lock:
            self._is_tracking          = tracking
            self._tracking_img_err     = img_err
            self._tracking_depth_err_m = depth_err_m

    # ── 命令执行状态管理（由 ArmCommanderNode/action server 调用）─────────────────
    def set_command_state(self, command_id: int, result: int):
        """更新当前命令的执行状态。"""
        with self._state_lock:
            self._executing_command_id = command_id
            self._command_result       = result

    def set_pose_state(self, state: int):
        """更新当前姿态状态（STOWED / OBSERVE / SHOOTING）。"""
        with self._state_lock:
            self._current_pose_state = state

    def set_error(self, error_code: int):
        """设置错误码。"""
        with self._state_lock:
            self._error_code = error_code

    def get_error_code(self) -> int:
        """读取当前错误码。"""
        with self._state_lock:
            return self._error_code

    def clear_error(self):
        """清除错误码。"""
        with self._state_lock:
            self._error_code = ArmStatus.ERR_NONE

    # ── 构建 ArmStatus 消息 ───────────────────────────────────────────────────────
    def build_status_message(self) -> ArmStatus:
        """构建完整的 ArmStatus 消息（由 ArmCommanderNode 周期调用）。"""
        msg = ArmStatus()
        msg.header.frame_id = BASE_FRAME

        with self._state_lock:
            msg.current_pose_state  = self._current_pose_state
            msg.error_code          = self._error_code
            msg.executing_command_id = self._executing_command_id
            msg.command_result      = self._command_result

        msg.arm_pose     = self.pose
        msg.arm_twist    = self.twist
        msg.is_moving    = self.is_moving
        msg.arm_at_target = not self.is_moving   # TODO: 改为基于目标位姿的比较
        msg.arm_at_pose_start = self.at_pose_start
        msg.camera_ready      = self.camera_ready
        with self._state_lock:
            msg.is_tracking          = self._is_tracking
            msg.tracking_img_err     = float(self._tracking_img_err)
            msg.tracking_depth_err_m = float(self._tracking_depth_err_m)
        return msg
