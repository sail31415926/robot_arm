"""
@file   display.launch.py
@brief  eMeetArm_models 机械臂 RViz2 可视化启动文件
@version 1.0
@date   2026-06-04

启动以下节点：
         - robot_state_publisher：加载 URDF 并发布 TF 变换
         - joint_state_publisher_gui：提供关节角度滑块控制界面
         - rviz2：加载预设配置文件进行三维可视化

用法：
  ros2 launch robot_arm_bringup display.launch.py

@copyright Copyright (c) 2026 eMeet
"""

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'eMeetArm_models.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.DOTALL)
    robot_description = ' '.join(robot_description.split())

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'eMeetArm_models.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
