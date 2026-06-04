"""
@file   test_ros.launch.py
@brief  ROS 节点基础通信测试

用法：
  ros2 launch robot_arm_bringup test_ros.launch.py

@date    2026-06-03
@copyright Copyright (c) 2026 eMeet
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_driver',
            executable='test_ros_node',
            name='test_ros_node',
            output='screen',
            parameters=[
                {'rate': 1}
            ]
        )
    ])
