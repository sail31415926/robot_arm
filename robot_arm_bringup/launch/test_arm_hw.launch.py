"""
@file test_arm_hw.launch.py
@brief PR-1 最小验证栈：单独拉起 ArmHardwareInterface + arm_controller(JTC)

只启动臂 J1-3 的 ros2_control 栈，不含云台、不含 commander / MoveIt，
用于验证 ArmHardwareInterface（CANopen）能被 JointTrajectoryController 驱动。

用法：
  # 实物（需 can0 已 up，ros2_control_node 需 CAP_NET_RAW）
  ros2 launch robot_arm_bringup test_arm_hw.launch.py

  # 干跑（不连 CAN，命令回显为状态，验证接口/加载）
  ros2 launch robot_arm_bringup test_arm_hw.launch.py arm_sim_mode:=true

验证：
  ros2 control list_hardware_interfaces      # Joint1-3 position/velocity
  ros2 control list_controllers              # arm_controller[active], jsb[active]
  ros2 topic echo /joint_states              # 仅 Joint1-3
  ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [Joint1,Joint2,Joint3], points: [{positions: [0.1,0.0,0.0], time_from_start: {sec: 3}}]}'

@date 2026-07-01
@copyright Copyright (c) 2026 EMEET
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    desc_share = get_package_share_directory('robot_arm_description')
    xacro_path = os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro')
    controllers_yaml = os.path.join(desc_share, 'config', 'controllers_real.yaml')

    arm_sim_mode = LaunchConfiguration('arm_sim_mode').perform(context)

    # backend=real → ArmHardwareInterface；emit_camera_control=false → 只出臂段 ros2_control
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            'backend': 'real',
            'arm_sim_mode': arm_sim_mode,
            'emit_camera_control': 'false',
            'sim_mode': 'false',
            'gazebo_camera': 'false',
            'controllers_yaml': '',       # 不注入 gazebo_ros2_control 插件
        },
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    # 顺序化：先 jsb，再 arm_controller（避免并发 spawner 竞争 controller_manager 服务）
    arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_ctrl_spawner]))

    # 使能/复位伴生服务节点：提供 /arm_node/{enable,disable,recover}
    arm_driver_services = Node(
        package='robot_arm_driver', executable='arm_driver_services', output='screen',
    )

    return [robot_state_publisher, controller_manager, jsb_spawner, arm_after_jsb,
            arm_driver_services]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_sim_mode', default_value='false',
            description='true=不连 CAN，命令回显为状态（干跑）；false=连接实物 CANopen',
        ),
        OpaqueFunction(function=_setup),
    ])
