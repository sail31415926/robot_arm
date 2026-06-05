#!/usr/bin/env python3
"""
@file   arm_trajectory_bridge.py
@brief  JointTrajectory 实物分流桥 — 将仿真指令转发到真实硬件
@version 1.0
@date   2026-06-04

将 /arm_controller/joint_trajectory 分流到实物硬件：
  Joint1-3 → /joint{N}/arm_motor_node/cmd_pos  (std_msgs/Float64, rad)
  Joint4-6 → /gimbal_controller/joint_trajectory (JointTrajectory)

使得所有 Gazebo GUI 控制脚本（arm_slider_controller、sphere_orbit_streamer 等）
无需修改即可直接用于实物模式。

用法：
  ros2 run robot_arm_node arm_trajectory_bridge

@copyright Copyright (c) 2026 eMeet
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64


_ARM_JOINT_TOPICS = {
    'Joint1': '/joint1/arm_motor_node/cmd_pos',
    'Joint2': '/joint2/arm_motor_node/cmd_pos',
    'Joint3': '/joint3/arm_motor_node/cmd_pos',
}
_CAM_JOINTS = {'Joint4', 'Joint5', 'Joint6'}


class ArmTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('arm_trajectory_bridge')

        self._arm_pubs = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in _ARM_JOINT_TOPICS.items()
        }
        self._cam_pub = self.create_publisher(
            JointTrajectory, '/gimbal_controller/joint_trajectory', 10)

        self.create_subscription(
            JointTrajectory, '/arm_controller/joint_trajectory',
            self._on_trajectory, 10)

        # Multi-point trajectory execution state (arm joints)
        self._pending = None
        self._arm_indices: dict[str, int] = {}
        self._idx = 0
        self._start_time = None
        self._exec_timer = None

        self.get_logger().info(
            'arm_trajectory_bridge ready\n'
            '  /arm_controller/joint_trajectory → Joint1-3: cmd_pos | Joint4-6: gimbal_controller')

    # ── incoming trajectory ──────────────────────────────────────────────────

    def _on_trajectory(self, msg: JointTrajectory):
        if not msg.points or not msg.joint_names:
            return

        # ── Camera joints → forward as-is to gimbal_controller ──────────────
        cam_names, cam_idx = [], []
        for i, name in enumerate(msg.joint_names):
            if name in _CAM_JOINTS:
                cam_names.append(name)
                cam_idx.append(i)

        if cam_names:
            cam_traj = JointTrajectory()
            cam_traj.header = msg.header
            cam_traj.joint_names = cam_names
            for pt in msg.points:
                np = JointTrajectoryPoint()
                np.time_from_start = pt.time_from_start
                if pt.positions:
                    np.positions  = [pt.positions[i]  for i in cam_idx]
                if pt.velocities:
                    np.velocities = [pt.velocities[i] for i in cam_idx]
                cam_traj.points.append(np)
            self._cam_pub.publish(cam_traj)

        # ── Arm joints → execute serially via cmd_pos ────────────────────────
        arm_idx = {
            name: i
            for i, name in enumerate(msg.joint_names)
            if name in _ARM_JOINT_TOPICS
        }
        if not arm_idx:
            return

        if self._exec_timer:
            self._exec_timer.cancel()
            self._exec_timer = None

        self._pending     = msg
        self._arm_indices = arm_idx
        self._idx         = 0
        self._start_time  = self.get_clock().now()

        self._step()  # first point immediately

        if len(msg.points) > 1:
            self._exec_timer = self.create_timer(0.01, self._timer_cb)

    # ── trajectory execution ─────────────────────────────────────────────────

    def _timer_cb(self):
        self._step()

    def _step(self):
        if self._pending is None or self._idx >= len(self._pending.points):
            if self._exec_timer:
                self._exec_timer.cancel()
                self._exec_timer = None
            return

        pt = self._pending.points[self._idx]

        # Wait until time_from_start has elapsed (skip check for first point)
        if self._idx > 0:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
            tfs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if elapsed < tfs:
                return

        if pt.positions:
            for name, src_idx in self._arm_indices.items():
                if src_idx < len(pt.positions):
                    msg = Float64()
                    msg.data = pt.positions[src_idx]
                    self._arm_pubs[name].publish(msg)

        self._idx += 1
        if self._idx >= len(self._pending.points):
            if self._exec_timer:
                self._exec_timer.cancel()
                self._exec_timer = None


def main():
    rclpy.init()
    rclpy.spin(ArmTrajectoryBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
