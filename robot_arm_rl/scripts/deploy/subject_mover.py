#!/usr/bin/env python3
"""
@file   subject_mover.py
@brief  驱动 Gazebo 里的 rl_subject 红盒做路点游走（训练同款运动模型），
        并把其世界位姿转发给 RL 策略节点。

数据流：
  本节点 → /rl_subject/cmd_vel (Twist)      → planar_move 插件驱动红盒
  /rl_subject/odom (Odometry, 世界系)       → 本节点 → /rl_policy/subject_pos

参数：
  speed        游走速度 m/s（默认 0.2，训练分布 0~0.4）
  x_min/x_max/y_min/y_max   路点采样范围（默认与训练一致）
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry


class SubjectMover(Node):
    def __init__(self):
        super().__init__('rl_subject_mover')
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('x_min', 0.65)
        self.declare_parameter('x_max', 1.15)
        self.declare_parameter('y_min', -0.35)
        self.declare_parameter('y_max', 0.35)
        self._speed = float(self.get_parameter('speed').value)
        self._xr = (float(self.get_parameter('x_min').value),
                    float(self.get_parameter('x_max').value))
        self._yr = (float(self.get_parameter('y_min').value),
                    float(self.get_parameter('y_max').value))

        self._pos = None          # 世界系 (x, y, z)，来自 odom
        self._wp  = self._sample_wp()

        self._cmd_pub  = self.create_publisher(Twist, '/rl_subject/cmd_vel', 10)
        self._subj_pub = self.create_publisher(
            PointStamped, '/rl_policy/subject_pos', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/rl_subject/odom', self._odom_cb, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(f'主体游走已启动  speed={self._speed} m/s')

    def _sample_wp(self):
        return (random.uniform(*self._xr), random.uniform(*self._yr))

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y, p.z)
        out = PointStamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'   # 臂底座在世界原点，两系重合
        out.point.x, out.point.y, out.point.z = p.x, p.y, p.z
        self._subj_pub.publish(out)

    def _tick(self):
        if self._pos is None:
            return
        dx, dy = self._wp[0] - self._pos[0], self._wp[1] - self._pos[1]
        d = math.hypot(dx, dy)
        cmd = Twist()
        if d < 0.03:
            self._wp = self._sample_wp()
        else:
            # planar_move 的 cmd_vel 是体坐标系；盒子不转（yaw≈0），等同世界系
            cmd.linear.x = self._speed * dx / d
            cmd.linear.y = self._speed * dy / d
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SubjectMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
