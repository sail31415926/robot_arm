"""
@file red_box_detect.launch.py
@brief 启动红色方块识别节点（配合已运行的 Gazebo 仿真）
@date 2026-06-03

@details 用法：
    # 终端1：先启动仿真
    ros2 launch robot_arm_gazebo gazebo.launch.py
    # 终端2：启动识别
    ros2 launch robot_arm_bringup red_box_detect.launch.py

  参数：
    show_window:=false   无图形界面时关闭弹窗，改用 rqt 看 /red_detector/image
    min_area:=300        最小轮廓面积阈值
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_window = LaunchConfiguration('show_window')
    min_area    = LaunchConfiguration('min_area')

    return LaunchDescription([
        DeclareLaunchArgument('show_window', default_value='true',
                              description='是否弹出 OpenCV 显示窗口'),
        DeclareLaunchArgument('min_area', default_value='300',
                              description='最小轮廓面积阈值(像素)'),
        Node(
            package='robot_arm_node',
            executable='red_box_detector',
            name='red_box_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'show_window': show_window,
                'min_area': min_area,
            }],
        ),
    ])
