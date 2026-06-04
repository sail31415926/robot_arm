"""
@file   motor.launch.py
@brief  三关节电机节点一键启动（multi-joint arm_motor_node）

同时启动 joint1 / joint2 / joint3 三个 arm_motor_node 实例，
各节点参数从 config/hardware/motors.yaml 中对应命名空间段读取：
  /joint1/arm_motor_node — node_id=1，负责发送主站心跳
  /joint2/arm_motor_node — node_id=2
  /joint3/arm_motor_node — node_id=3

用法：
  ros2 launch robot_arm_bringup motor.launch.py

启动后配合调试 GUI：
  ros2 run robot_arm_driver motor_test_gui

所属模块：launch/  (🔴 real only)

@date    2026-05-27
@copyright Copyright (c) 2026 EMEET
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('robot_arm_bringup'), "config", "hardware", "motors.yaml")

    def motor_node(ns):
        return Node(
            package='robot_arm_driver',
            executable="arm_motor_node",
            name="arm_motor_node",
            namespace=ns,
            output="screen",
            parameters=[cfg],
        )

    return LaunchDescription([
        motor_node("joint1"),
        motor_node("joint2"),
        motor_node("joint3"),
    ])
