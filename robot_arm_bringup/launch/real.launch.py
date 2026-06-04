"""
@file real.launch.py
@brief eMeetArm 实物一键启动文件

启动拓扑：
  Joint1-3  →  arm_node（CANopen，JointTrajectory + joint_states 整体接口）
  Joint4-6  →  ros2_control / CameraHardwareInterface（HID）
               └─ camera_controller（JointTrajectoryController）

用法（与 Gazebo launch 参数一致）：
  ros2 launch robot_arm_bringup real.launch.py
  ros2 launch robot_arm_bringup real.launch.py controller:=slider
  ros2 launch robot_arm_bringup real.launch.py controller:=sphere_orbit
  ros2 launch robot_arm_bringup real.launch.py controller:=ruckig_ik
  ros2 launch robot_arm_bringup real.launch.py camera_type:=pixy

视频流由本 launch 直接 include emeet_camera_driver/camera.launch.py
传入 disable_hid:=true 避免与 ros2_control CameraHardwareInterface 抢 HID 设备。

@date 2026-05-29
"""

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _camera_ros2_control_urdf(camera_type: str) -> str:
    return f"""<?xml version="1.0"?>
<robot name="eMeetCamera">
  <ros2_control name="eMeetCamera_hardware" type="system">
    <hardware>
      <plugin>emeet_camera_driver/CameraHardwareInterface</plugin>
      <param name="camera_type">{camera_type}</param>
      <param name="sim_mode">false</param>
    </hardware>
    <joint name="Joint4">
      <command_interface name="position">
        <param name="min">-3.1</param><param name="max">3.1</param>
      </command_interface>
      <command_interface name="velocity"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="Joint5">
      <command_interface name="position">
        <param name="min">-0.7854</param><param name="max">0.7854</param>
      </command_interface>
      <command_interface name="velocity"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="Joint6">
      <command_interface name="position">
        <param name="min">-1.5</param><param name="max">0.5</param>
      </command_interface>
      <command_interface name="velocity"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>"""


def generate_launch_description():
    desc_share       = get_package_share_directory('robot_arm_description')
    bringup_share    = get_package_share_directory('robot_arm_bringup')
    urdf_path        = os.path.join(desc_share, 'urdf', 'eMeetArm_models.urdf')
    arm_yaml         = os.path.join(bringup_share, 'config', 'hardware', 'arm.yaml')
    controllers_yaml = os.path.join(desc_share, 'config', 'controllers.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    camera_type_arg = DeclareLaunchArgument(
        'camera_type', default_value='auto',
        description='摄像头型号: auto | pixy | e7002 | piko',
    )
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='slider',
        description='控制方式: slider | cartesian | realtime | ruckig_ik | sphere_orbit',
    )
    ctrl = LaunchConfiguration('controller')

    resolved_camera_type = 'auto'
    for arg in sys.argv:
        if arg.startswith('camera_type:='):
            resolved_camera_type = arg.split(':=', 1)[1]
    camera_urdf = _camera_ros2_control_urdf(resolved_camera_type)

    with open(urdf_path, 'r') as f:
        full_urdf = f.read()

    # ── Core nodes ────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': full_urdf, 'use_sim_time': False}],
    )

    # ros2_control 只管摄像头云台（Joint4-6）
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': camera_urdf}, controllers_yaml],
    )

    # 机械臂整体节点：Joint1-3，JointTrajectory 输入，joint_states 输出
    arm_node = Node(
        package='robot_arm_driver', executable='arm_node',
        name='arm_node', output='screen',
        parameters=[arm_yaml],
    )

    # ── Camera controller spawners ────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    camera_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['camera_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ── GUI controller nodes ──────────────────────────────────────────────────
    def ctrl_node(mode, exe):
        return Node(
            package='robot_arm_node', executable=exe, output='screen',
            condition=IfCondition(PythonExpression(["'", ctrl, "' == '", mode, "'"])),
        )

    slider_ctrl       = ctrl_node('slider',       'arm_slider_controller')
    cartesian_ctrl    = ctrl_node('cartesian',    'cartesian_controller')
    realtime_ctrl     = ctrl_node('realtime',     'cartesian_realtime_controller')
    ruckig_ik_ctrl    = ctrl_node('ruckig_ik',    'cartesian_ruckig_ik_streamer')
    sphere_orbit_ctrl = ctrl_node('sphere_orbit', 'spherical_orbit_streamer')

    # ── MoveIt（需要 IK 的控制方式：ruckig_ik / sphere_orbit / cartesian / realtime）
    needs_moveit = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['ruckig_ik','sphere_orbit','cartesian','realtime']"
        ])
    )
    # 实物模式直接创建 move_group，使用 FollowJointTrajectory action 配置
    # （不 include moveit.launch.py，避免 Ros2ControlManager 找不到 Joint1-3 控制器）
    moveit_cfg = os.path.join(bringup_share, 'config', 'software')
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': full_urdf},
            {'robot_description_semantic': open(
                os.path.join(moveit_cfg, 'srdf', 'eMeetArm_models.srdf')).read()},
            {'robot_description_kinematics': _load_yaml(
                os.path.join(moveit_cfg, 'kinematics.yaml'))},
            {'robot_description_planning': _load_yaml(
                os.path.join(moveit_cfg, 'joint_limits.yaml'))},
            _load_yaml(os.path.join(moveit_cfg, 'planning_pipeline.yaml')),
            _load_yaml(os.path.join(moveit_cfg, 'moveit_controllers_real.yaml')),
            {'use_sim_time': False,
             'start_state_max_bounds_error': 0.5},
        ],
        condition=needs_moveit,
    )

    # 摄像头控制器 2 s 后 spawn（等 ros2_control_node 初始化）
    spawn_camera = TimerAction(
        period=2.0,
        actions=[jsb_spawner, camera_ctrl_spawner],
    )

    # GUI 3 s 后启动（等 arm_node 完成 enable + PP 模式切换）
    spawn_gui = TimerAction(
        period=3.0,
        actions=[slider_ctrl, cartesian_ctrl, realtime_ctrl,
                 ruckig_ik_ctrl, sphere_orbit_ctrl],
    )

    return LaunchDescription([
        camera_type_arg,
        controller_arg,
        robot_state_publisher,
        ros2_control_node,
        arm_node,
        move_group_node,       # 仅 ruckig_ik / sphere_orbit / cartesian / realtime 时启动
        spawn_camera,
        spawn_gui,
    ])
