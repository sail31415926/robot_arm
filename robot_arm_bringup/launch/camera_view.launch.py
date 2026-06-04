"""
@file   camera_view.launch.py
@brief  相机图像查看工具（Gazebo / MuJoCo 通用）

将仿真器发布的原始图像转为 compressed 格式，并用 rqt_image_view 显示。

用法：
  ros2 launch robot_arm_bringup camera_view.launch.py
  ros2 launch robot_arm_bringup camera_view.launch.py use_sim_time:=false

@date    2026-06-03
@copyright Copyright (c) 2026 eMeet
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (true for Gazebo, false for MuJoCo)',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,
        # Gazebo 的 image_transport 插件已自动发布 compressed 话题，直接订阅即可。
        # MuJoCo 发布 raw，用 republish 转为 compressed。
        Node(
            package='image_transport',
            executable='republish',
            name='camera_compress',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in',             '/camera/camera_sensor/image_raw'),
                ('out/compressed', '/camera/camera_sensor/image_raw/compressed'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        # 直接订阅 Gazebo/MuJoCo 都有的压缩话题
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='camera_view',
            arguments=['/camera/camera_sensor/image_raw/compressed'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
