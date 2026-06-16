#!/usr/bin/env python3
"""ROS 2 node: 订阅 /joint_states 和 /robot_arm/arm_status，Ctrl+C 时保存 CSV。"""
import csv
import datetime
import os
import signal
import sys
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robot_arm_interfaces.msg import ArmStatus


class MotionDataRecorder(Node):
    def __init__(self):
        super().__init__('motion_data_recorder')
        self.declare_parameter('output_dir', str(Path.home() / 'robot_arm_data'))

        out = self.get_parameter('output_dir').value
        os.makedirs(out, exist_ok=True)
        self._output_dir = out

        self._joint_rows: list = []
        self._status_rows: list = []
        self._joint_names: Optional[list] = None

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 100)
        self.create_subscription(ArmStatus, '/robot_arm/arm_status', self._on_arm_status, 10)
        self.get_logger().info(f'Recording → {out}  (Ctrl+C to stop and save)')

    def _on_joint_state(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._joint_names is None and msg.name:
            self._joint_names = list(msg.name)
        row = [t] + list(msg.position) + list(msg.velocity) + list(msg.effort)
        self._joint_rows.append(row)

    def _on_arm_status(self, msg: ArmStatus):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, v = msg.arm_pose, msg.arm_twist
        self._status_rows.append([
            t, p.x, p.y, p.z, p.roll, p.pitch, p.yaw,
            v.vx, v.vy, v.vz, v.wroll, v.wpitch, v.wyaw,
            int(msg.is_moving),
        ])

    def save(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        if self._joint_rows and self._joint_names:
            names = self._joint_names
            header = (['timestamp'] +
                      [f'{n}_pos' for n in names] +
                      [f'{n}_vel' for n in names] +
                      [f'{n}_eff' for n in names])
            path = os.path.join(self._output_dir, f'joint_states_{ts}.csv')
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(self._joint_rows)
            self.get_logger().info(f'Saved {len(self._joint_rows)} rows → {path}')
        else:
            self.get_logger().warn('No joint state data recorded.')

        if self._status_rows:
            header = ['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                      'vx', 'vy', 'vz', 'wroll', 'wpitch', 'wyaw', 'is_moving']
            path = os.path.join(self._output_dir, f'arm_status_{ts}.csv')
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(self._status_rows)
            self.get_logger().info(f'Saved {len(self._status_rows)} rows → {path}')
        else:
            self.get_logger().warn('No arm status data recorded.')


def main():
    rclpy.init()
    node = MotionDataRecorder()

    def _on_sigint(sig, frame):
        node.save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_sigint)
    rclpy.spin(node)
