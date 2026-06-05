"""
@file mujoco.launch.py
@brief eMeetArm MuJoCo 仿真 + MoveIt 一键启动（替代 gazebo.launch.py + moveit.launch.py）
@version 2.2
@date 2026-06-01

@details 启动以下节点：
         - mujoco_node       : MuJoCo 物理仿真 + viewer + FollowJointTrajectory action
         - robot_state_publisher : TF（复用同一 URDF）
         - move_group        : MoveIt2 规划核心（使用 MuJoCo 专用 controller 配置）
         - 控制 GUI（根据 controller 参数选择其一）

         通过 controller 参数选择 7 种控制方式之一：
           slider       → arm_slider_controller            (PyQt5 关节滑块，无需 MoveIt)
           cartesian    → cartesian_controller             (MoveIt 笛卡尔直线规划)
           realtime     → cartesian_realtime_controller    (滑块即时 IK)
           ruckig       → cartesian_ruckig_streamer        (Ruckig 笛卡尔流式 + servo)
           ruckig_ik    → cartesian_ruckig_ik_streamer     (Ruckig 点到点+平面环绕)
           sphere_orbit → spherical_orbit_streamer         (球面坐标环绕运镜)
           velocity     → cartesian_velocity_controller    (笛卡尔速度接口，IBVS 用)

         示例：
           ros2 launch robot_arm_bringup mujoco.launch.py                              # 默认 slider
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=slider           # 关节滑块
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=cartesian        # MoveIt 笛卡尔直线
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=realtime         # 滑块即时 IK
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=ruckig           # Ruckig 笛卡尔流
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=ruckig_ik        # Ruckig 点到点+环绕
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=sphere_orbit     # 球面轨道运镜
           ros2 launch robot_arm_bringup mujoco.launch.py controller:=velocity          # 笛卡尔速度/IBVS

@note 不启动 ros2_control / controller_manager；
      mujoco_node 直接提供 /arm_controller/follow_joint_trajectory action。

@copyright Copyright (c) 2026 eMeet
"""

import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_share    = get_package_share_directory('robot_arm_description')
    bringup_share = get_package_share_directory('robot_arm_bringup')
    cfg           = os.path.join(bringup_share, 'config', 'moveit')
    urdf_path     = os.path.join(desc_share, 'urdf', 'eMeetArm_models.urdf')

    # ── URDF（去掉注释，move_group 只需 kinematic 结构）────────────────────────
    with open(urdf_path, 'r') as f:
        robot_description_raw = f.read()
    robot_description_raw = re.sub(
        r'<!--.*?-->', '', robot_description_raw, flags=re.DOTALL)
    robot_description_raw = ' '.join(robot_description_raw.split())
    robot_description = {'robot_description': robot_description_raw}

    # ── MoveIt 配置参数 ──────────────────────────────────────────────────────
    with open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf'), 'r') as f:
        srdf_content = f.read()
        robot_description_semantic = {'robot_description_semantic': srdf_content}

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(
            os.path.join(desc_share, 'config', 'kinematics.yaml'))}
    robot_description_planning = {
        'robot_description_planning': load_yaml(
            os.path.join(desc_share, 'config', 'joint_limits.yaml'))}

    planning_pipeline    = load_yaml(os.path.join(cfg, 'planning_pipeline.yaml'))
    moveit_controllers   = load_yaml(
        os.path.join(cfg, 'moveit_controllers_mujoco.yaml'))  # MuJoCo 专用

    # ── MoveIt Servo（仅 controller:=ruckig 时使用）──────────────────────────
    servo_params = {
        'moveit_servo': load_yaml(os.path.join(cfg, 'servo_config.yaml')),
    }

    # ── 控制方式参数 ──────────────────────────────────────────────────────────
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='slider',
        description='控制方式: slider | cartesian | realtime | ruckig | ruckig_ik | sphere_orbit | velocity',
    )
    ctrl = LaunchConfiguration('controller')

    def controller_node(mode_name, exe_name):
        """根据 controller 参数条件启动对应节点"""
        return Node(
            package='robot_arm_node',
            executable=exe_name,
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", ctrl, "' == '", mode_name, "'"])),
        )

    slider_ctrl       = controller_node('slider',       'arm_slider_controller')
    cartesian_ctrl    = controller_node('cartesian',    'cartesian_controller')
    realtime_ctrl     = controller_node('realtime',     'cartesian_realtime_controller')
    ruckig_ctrl       = controller_node('ruckig',       'cartesian_ruckig_streamer')
    ruckig_ik_ctrl    = controller_node('ruckig_ik',    'cartesian_ruckig_ik_streamer')
    sphere_orbit_ctrl = controller_node('sphere_orbit', 'spherical_orbit_streamer')
    velocity_ctrl     = controller_node('velocity',     'cartesian_velocity_controller')

    # ── MoveIt（需要 IK 的控制方式：cartesian / realtime / ruckig_ik / sphere_orbit）
    needs_moveit = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['cartesian','realtime','ruckig_ik','sphere_orbit']"
        ])
    )

    # ── MoveIt Servo（仅 controller:=ruckig 时启动）──────────────────────────
    is_ruckig = IfCondition(
        PythonExpression(["'", ctrl, "' == 'ruckig'"])
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params,
            {'robot_description': robot_description_raw,
             'robot_description_semantic': srdf_content,
             'use_sim_time': False},
            robot_description_kinematics,
            robot_description_planning,
        ],
        condition=is_ruckig,
    )

    # ── Ruckig 模式启动序列：先把机器人移到非奇异姿态，等 4s，再起 servo + GUI ─
    # 全零关节是运动学奇异点（雅可比秩亏），servo 会立刻紧急停车。
    # 选一组手肘弯的安全姿态：Joint2=1.0, Joint3=-1.5, Joint5=0.3。
    move_to_safe_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/arm_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            ('{joint_names: [Joint1,Joint2,Joint3,Joint4,Joint5,Joint6], '
             'points: [{positions: [0.0, 1.0, -1.5, 0.0, 0.3, 0.0], '
             'time_from_start: {sec: 3, nanosec: 0}}]}'),
        ],
        output='screen',
        condition=is_ruckig,
    )

    ruckig_start_after_pose = TimerAction(
        period=4.0,                       # 等 3s 移动 + 1s 余量
        actions=[servo_node, ruckig_ctrl],
        condition=is_ruckig,
    )

    # ── robot_state_publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{**robot_description, 'use_sim_time': False}],
    )

    # ── MuJoCo 仿真节点（提供 viewer + FollowJointTrajectory action server）──
    mujoco_node = Node(
        package='robot_arm_node',
        executable='mujoco_node',
        output='screen',
        name='mujoco_node',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD':  '1',
            '__GLX_VENDOR_LIBRARY_NAME':  'nvidia',
            '__GL_SYNC_TO_VBLANK':        '0',
            '__GL_MaxFramesAllowed':       '1',
        },
    )

    # ── MoveIt move_group（使用 MuJoCo 专用 controller 配置，不依赖 ros2_control）
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            moveit_controllers,
            {'use_sim_time': False,
             'start_state_max_bounds_error': 0.5},
        ],
        condition=needs_moveit,
    )

    # ── 控制 GUI 启动时序 ────────────────────────────────────────────────────
    # slider：仅需 MuJoCo 就绪，延迟 2s
    slider_timed = TimerAction(
        period=2.0,
        actions=[slider_ctrl],
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' == 'slider'"])),
    )

    # cartesian / realtime / ruckig_ik / sphere_orbit：需 MoveIt 就绪，延迟 4s
    cartesian_timed = TimerAction(
        period=4.0,
        actions=[cartesian_ctrl],
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' == 'cartesian'"])),
    )

    realtime_timed = TimerAction(
        period=4.0,
        actions=[realtime_ctrl],
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' == 'realtime'"])),
    )

    ruckig_ik_timed = TimerAction(
        period=4.0,
        actions=[ruckig_ik_ctrl],
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' == 'ruckig_ik'"])),
    )

    sphere_orbit_timed = TimerAction(
        period=4.0,
        actions=[sphere_orbit_ctrl],
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' == 'sphere_orbit'"])),
    )

    # velocity：MuJoCo 内部直接做雅可比反解，无需 MoveIt，延迟 2s
    velocity_timed = TimerAction(
        period=2.0,
        actions=[velocity_ctrl],
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' == 'velocity'"])),
    )

    # ── 摄像头画面显示（复用 camera_view.launch.py，与 Gazebo 一致）──────────
    camera_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'camera_view.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        controller_arg,
        robot_state_publisher,
        mujoco_node,
        move_group,
        slider_timed,
        cartesian_timed,
        realtime_timed,
        move_to_safe_pose,
        ruckig_start_after_pose,
        ruckig_ik_timed,
        sphere_orbit_timed,
        velocity_timed,
        camera_view,
    ])
