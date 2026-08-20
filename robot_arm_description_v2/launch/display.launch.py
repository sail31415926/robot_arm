"""在 RViz 中查看第二版机械臂模型（robot_state_publisher + joint_state_publisher_gui）。

用法：ros2 launch robot_arm_description_v2 display.launch.py [gui:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_arm_description_v2')
    urdf_path = os.path.join(share, 'urdf', 'robot_arm_description_v2.urdf')
    rviz_config = os.path.join(share, 'rviz', 'display.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true 用 joint_state_publisher_gui 滑条调关节，false 用无界面版'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
