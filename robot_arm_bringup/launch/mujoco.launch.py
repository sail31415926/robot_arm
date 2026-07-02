"""
@file mujoco.launch.py
@brief bringup 的 MuJoCo 后端 —— MuJoCo 底座，上层栈复用共享工厂
@version 3.0
@date 2026-07-01

@note  本文件是整机启动的 mujoco 后端（统一入口见 bringup.launch.py，backend:=mujoco）。
       职责限于「底座」：mujoco_node（物理 + viewer + 轨迹接口）+ move_group（MuJoCo controllers）。
       上层栈节点定义复用 launch/_arm_launch_common.py，与 gazebo/real 共用一份。

@details 启动：mujoco_node（物理 + viewer + /arm_controller/joint_trajectory 订阅 + FJT action）、
         robot_state_publisher、move_group（需 IK 的模式）、按 controller 选择的上层节点。

         v3.0（PR-4 第二步）：上层栈节点定义改为复用 `_arm_launch_common`（与 gazebo/real 同一份），
         并**补上 commander 模式**（此前 MuJoCo 缺失的漂移）。mujoco_node 订阅
         `/arm_controller/joint_trajectory` 话题，故 commander 的 JointTrajectory 可直接驱动。
         use_sim_time 仍为 false —— MuJoCo 暂不发布 /clock（见方案 §九 P5），待其发 /clock 后再统一为 true。

         controller 参数（= 对应 GUI 名去掉 _gui 后缀）：
           joint_position | cartesian_moveit | cartesian_realtime_ik |
           cartesian_trajectory | spherical_orbit | cartesian_velocity | commander

         示例：
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=cartesian_trajectory
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=commander            # 新增
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=commander gui:=false

@note 不启动 ros2_control / controller_manager；mujoco_node 直接提供轨迹接口。

@copyright Copyright (c) 2026 eMeet
"""

import os
import sys

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 上层栈节点工厂（与 gazebo/real 共用同一份定义）
sys.path.insert(0, os.path.dirname(__file__))
import _arm_launch_common as common   # noqa: E402


def load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


# MuJoCo 暂不发布 /clock（见方案 §九 P5）→ 保持系统时钟
USE_SIM_TIME = False

_NVIDIA_ENV = {
    '__NV_PRIME_RENDER_OFFLOAD':  '1',
    '__GLX_VENDOR_LIBRARY_NAME':  'nvidia',
    '__GL_SYNC_TO_VBLANK':        '0',
    '__GL_MaxFramesAllowed':       '1',
}


def generate_launch_description():
    desc_share    = get_package_share_directory('robot_arm_description')
    bringup_share = get_package_share_directory('robot_arm_bringup')
    cfg           = os.path.join(bringup_share, 'config', 'moveit')
    xacro_path    = os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro')

    # ── 描述与配置（MuJoCo 不用 ros2_control，xacro 取默认映射）─────────────────
    robot_description_raw = xacro.process_file(xacro_path).toxml()
    srdf_content = open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf')).read()
    kinematics         = load_yaml(os.path.join(desc_share, 'config', 'kinematics.yaml'))
    joint_limits       = load_yaml(os.path.join(desc_share, 'config', 'joint_limits.yaml'))
    planning_pipeline  = load_yaml(os.path.join(cfg, 'planning_pipeline.yaml'))
    moveit_controllers = load_yaml(os.path.join(cfg, 'moveit_controllers_mujoco.yaml'))  # MuJoCo 专用
    servo_params       = {'moveit_servo': load_yaml(os.path.join(cfg, 'servo_config.yaml'))}

    # ── 参数 ──────────────────────────────────────────────────────────────────
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='joint_position',
        description='控制方式: joint_position | cartesian_moveit | cartesian_realtime_ik | '
                    'cartesian_trajectory | spherical_orbit | cartesian_velocity | commander')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='是否启动 commander_test_gui（仅 controller:=commander 生效）: true | false')
    ctrl = LaunchConfiguration('controller')
    gui  = LaunchConfiguration('gui')

    is_velocity = common.is_mode(ctrl, 'cartesian_velocity')

    # ── 基座：robot_state_publisher + MuJoCo 仿真节点 ──────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': USE_SIM_TIME}])

    mujoco_node = Node(
        package='robot_arm_node', executable='mujoco_node', name='mujoco_node',
        output='screen', additional_env=_NVIDIA_ENV)

    # ── 上层栈（复用共享工厂）──────────────────────────────────────────────────
    move_group = common.move_group_node(
        ctrl, robot_description=robot_description_raw, srdf=srdf_content,
        kinematics=kinematics, joint_limits=joint_limits,
        planning_pipeline=planning_pipeline, moveit_controllers=moveit_controllers,
        use_sim_time=USE_SIM_TIME)

    # cartesian_velocity：安全预移动（t≈0，3s 运动）→ t=4s 起 servo + GUI
    safe_pose = common.safe_pose_action(is_velocity)
    velocity_start = TimerAction(
        period=4.0,
        actions=[
            common.servo_node(is_velocity, robot_description=robot_description_raw,
                              srdf=srdf_content, kinematics=kinematics,
                              joint_limits=joint_limits, servo_params=servo_params,
                              use_sim_time=USE_SIM_TIME),
            common.gui_node(ctrl, 'cartesian_velocity', 'cartesian_velocity_gui'),
        ],
        condition=is_velocity)

    # joint_position：仅需 MuJoCo 就绪，t=2s
    joint_position_timed = TimerAction(
        period=2.0,
        actions=[common.gui_node(ctrl, 'joint_position', 'joint_position_gui')],
        condition=common.is_mode(ctrl, 'joint_position'))

    # 需 MoveIt 的 GUI（cartesian_moveit/realtime_ik/trajectory/spherical_orbit）：t=4s（各自互斥条件）
    moveit_gui_timed = TimerAction(
        period=4.0,
        actions=[
            common.gui_node(ctrl, 'cartesian_moveit',      'cartesian_moveit_gui'),
            common.gui_node(ctrl, 'cartesian_realtime_ik', 'cartesian_realtime_ik_gui'),
            common.gui_node(ctrl, 'cartesian_trajectory',  'cartesian_trajectory_gui'),
            common.gui_node(ctrl, 'spherical_orbit',       'spherical_orbit_gui'),
        ])

    # commander（新增）：等 move_group planning scene 就绪，t=8s
    commander_timed = TimerAction(
        period=8.0,
        actions=common.commander_nodes(ctrl, gui, USE_SIM_TIME),
        condition=common.is_mode(ctrl, 'commander'))

    # 摄像头画面显示（复用 camera_view.launch.py）
    camera_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'camera_view.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items())

    return LaunchDescription([
        controller_arg, gui_arg,
        robot_state_publisher,
        mujoco_node,
        move_group,
        joint_position_timed,
        moveit_gui_timed,
        safe_pose,
        velocity_start,
        commander_timed,
        camera_view,
    ])
