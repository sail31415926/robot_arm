#!/usr/bin/env python3
"""
@file   joint_position_controller_node.py
@brief  eMeetArm 6轴机械臂关节位置控制 node（无 GUI，可被 GUI import 或 headless 运行）
@version 1.0
@date   2026-06-09

纯控制逻辑，不依赖任何 GUI 框架：
         - 发布 JointTrajectory 至 /arm_controller/joint_trajectory
         - 订阅 /joint_states，通过可选回调上报关节位置/速度
         - 通过 TF2 查询末端 gimbal_tool0 在 base_link 下位姿，通过可选回调上报
           （2026-07-28 云台换 V2：末端 = 云台 Joint6 后的安装板，不是臂法兰 tool0）

被 joint_position_gui.py（PyQt 调试 GUI）import 使用；也可独立运行：
  ros2 run robot_arm_node joint_position_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']


class JointPositionControllerNode(Node):
    """关节位置控制 node。

    on_joint_state(positions: list, velocities: list)        —— 收到 /joint_states 时回调（可选）
    on_end_effector(x, y, z, roll_deg, pitch_deg, yaw_deg)   —— 末端位姿更新时回调（可选）
    回调由 ROS 执行线程触发；GUI 侧应自行处理线程切换（如 Qt 信号为跨线程队列连接）。
    """

    def __init__(self, on_joint_state=None, on_end_effector=None):
        super().__init__('joint_position_controller_node')
        self._on_joint_state_cb  = on_joint_state
        self._on_end_effector_cb = on_end_effector

        self.publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._publish_end_effector)

    def _on_joint_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        positions, velocities = [], []
        for name in JOINT_NAMES:
            idx = name_to_idx.get(name)
            positions.append(msg.position[idx] if idx is not None and idx < len(msg.position) else 0.0)
            velocities.append(msg.velocity[idx] if idx is not None and idx < len(msg.velocity) else 0.0)
        if self._on_joint_state_cb:
            self._on_joint_state_cb(positions, velocities)

    def _publish_end_effector(self):
        try:
            t = self.tf_buffer.lookup_transform('arm_base_link', 'gimbal_tool0', rclpy.time.Time())
            tr = t.transform.translation
            q = t.transform.rotation
            # quaternion → RPY (rad)
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr, cosr)
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            if self._on_end_effector_cb:
                self._on_end_effector_cb(
                    tr.x, tr.y, tr.z,
                    math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                )
        except Exception:
            pass

    def publish_trajectory(self, positions: list, duration_sec: float):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        secs = int(duration_sec)
        pt.time_from_start = Duration(sec=secs, nanosec=int((duration_sec - secs) * 1e9))
        msg.points = [pt]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = JointPositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
