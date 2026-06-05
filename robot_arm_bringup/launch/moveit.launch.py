"""
@file moveit.launch.py
@brief eMeetArm_models 机械臂 MoveIt2 运动规划启动文件
@version 1.1
@date 2026-05-29

@details 启动以下节点：
         - move_group：MoveIt2 核心规划节点
         - rviz2：加载 MoveIt2 MotionPlanning 插件界面（可禁用）

参数：
  use_sim_time  true/false  Gazebo 仿真时钟（默认 true）
  rviz          true/false  是否启动 RViz2（无显示器/SSH 设 false，默认 true）

示例：
  ros2 launch robot_arm_bringup moveit.launch.py                          # Gazebo 默认
  ros2 launch robot_arm_bringup moveit.launch.py use_sim_time:=false rviz:=false  # 实物无显示器

@copyright Copyright (c) 2026 eMeet
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_share    = get_package_share_directory('robot_arm_description')
    bringup_share = get_package_share_directory('robot_arm_bringup')
    cfg           = os.path.join(bringup_share, 'config', 'moveit')

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='true = Gazebo 仿真时钟，false = 实物系统时钟',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz2（无显示器/SSH 环境设 false）',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz         = LaunchConfiguration('rviz')

    # ── Robot description ──────────────────────────────────────────────────────
    with open(os.path.join(desc_share, 'urdf', 'eMeetArm_models.urdf'), 'r') as f:
        urdf_content = f.read()
    with open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf'), 'r') as f:
        srdf_content = f.read()

    robot_description          = {'robot_description':          urdf_content}
    robot_description_semantic = {'robot_description_semantic': srdf_content}
    robot_description_kin      = {'robot_description_kinematics': load_yaml(os.path.join(desc_share, 'config', 'kinematics.yaml'))}
    robot_description_plan     = {'robot_description_planning':   load_yaml(os.path.join(desc_share, 'config', 'joint_limits.yaml'))}

    planning_pipeline  = load_yaml(os.path.join(cfg, 'planning_pipeline.yaml'))
    moveit_controllers = load_yaml(os.path.join(cfg, 'moveit_controllers.yaml'))

    # ── move_group ────────────────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kin,
            robot_description_plan,
            planning_pipeline,
            moveit_controllers,
            {'use_sim_time': use_sim_time,
             'start_state_max_bounds_error': 0.5},
        ],
    )

    # ── rviz2（条件启动）─────────────────────────────────────────────────────
    rviz_config = os.path.join(desc_share, 'rviz', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kin,
            {'use_sim_time': use_sim_time},
        ],
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD':  '1',
            '__GLX_VENDOR_LIBRARY_NAME':  'nvidia',
            '__GL_SYNC_TO_VBLANK':        '0',
            '__GL_MaxFramesAllowed':       '1',
        },
        condition=IfCondition(rviz),   # 由 LaunchConfiguration 控制，IncludeLaunchDescription 也有效
    )

    return LaunchDescription([use_sim_time_arg, rviz_arg, move_group_node, rviz_node])
