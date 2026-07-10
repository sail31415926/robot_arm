"""
@file real.launch.py
@brief bringup 的实物后端 —— 只管实物底座，上层栈复用共享工厂
@version 2.0
@date 2026-07-08

@note  本文件是整机启动的 real 后端（统一入口见 bringup.launch.py，backend:=real）。
       v2.0：臂 J1-3 从 arm_node（自研 CANopen）切换到 ros2_control HAL
       （canopen_ros2_control/RobotSystem，ros2_canopen），与云台 J4-6
       （CameraHardwareInterface，HID）合并为单个 controller_manager 管全 6 轴。

启动拓扑（单 arm_controller 管全 6 轴，与 Gazebo/MuJoCo 命令总线一致）：
  arm_controller（JTC，claim Joint1-6 position，跨两个硬件组件）
    ├─ Joint1-3  →  canopen_ros2_control/RobotSystem（ros2_canopen，CiA402/SocketCAN，position→IP）
    └─ Joint4-6  →  robot_gimbal_driver/CameraHardwareInterface（HID）
  /arm_node/{enable,disable,recover} → arm_driver_services（转发 controller_manager）
  注：gimbal_controller 仅在 controllers_real.yaml 保留定义（云台单独调试用），本 launch 不 spawn。

用法（与 Gazebo launch 参数一致）：
  ros2 launch robot_arm_bringup real.launch.py controller:=joint_position             # 关节滑块
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_moveit           # MoveIt 笛卡尔直线
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_realtime_ik      # 滑块即时 IK
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_trajectory       # Ruckig 点到点+环绕
  ros2 launch robot_arm_bringup real.launch.py controller:=spherical_orbit            # 球面轨道运镜
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_velocity         # 笛卡尔速度（MoveIt Servo）
  ros2 launch robot_arm_bringup real.launch.py controller:=visp_ibvs                  # 红色方块 IBVS 闭环
  ros2 launch robot_arm_bringup real.launch.py controller:=commander                  # Arm Commander 中间层
  ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false

  arm_sim_mode:=true   臂 J1-3 不连 CAN（mock_components 回显），云台照常，用于无臂调试

前置条件（真机）：
  sudo ip link set can0 up type can bitrate 500000 && sudo ip link set can0 txqueuelen 128

视频流由 robot_camera_node（robot_gimbal_node 包）单独启动，仅占用 V4L2，
不与 ros2_control CameraHardwareInterface（HID）冲突，可同时运行。

@copyright Copyright (c) 2026 eMeet
"""

import os
import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 上层栈节点工厂（与 gazebo/mujoco 共用同一份定义）
sys.path.insert(0, os.path.dirname(__file__))
import _arm_launch_common as common   # noqa: E402


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _default_can_interface() -> str:
    """★ CAN 口统一在 robot_arm_driver/config/can.yaml 改（test_arm.launch 也读它）★"""
    try:
        cfg = os.path.join(get_package_share_directory('robot_arm_driver'),
                           'config', 'can.yaml')
        with open(cfg) as f:
            return str(yaml.safe_load(f)['can_interface'])
    except Exception:
        return 'can0'


def _setup(context, *args, **kwargs):
    desc_share       = get_package_share_directory('robot_arm_description')
    bringup_share    = get_package_share_directory('robot_arm_bringup')
    gimbal_share     = get_package_share_directory('robot_gimbal_node')
    controllers_yaml = os.path.join(desc_share, 'config', 'controllers_real.yaml')
    moveit_cfg       = os.path.join(bringup_share, 'config', 'moveit')
    servo_params     = _load_yaml(os.path.join(moveit_cfg, 'servo_config.yaml'))
    srdf_content     = open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf')).read()

    arm_sim_mode  = LaunchConfiguration('arm_sim_mode').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)

    # 全 6 轴 URDF：J1-3 RobotSystem（CANopen）+ J4-6 CameraHardwareInterface（HID）
    _xacro_path = os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro')
    rd = xacro.process_file(
        _xacro_path,
        mappings={
            'backend': 'real',
            'arm_sim_mode': arm_sim_mode,
            'can_interface': can_interface,
            'sim_mode': 'false',            # 云台连实物 HID
            'gazebo_camera': 'false',
            'emit_camera_control': 'true',
            'controllers_yaml': '',         # 实物无 gazebo_ros2_control 插件
        },
    ).toxml()
    rds = srdf_content

    ctrl      = LaunchConfiguration('controller')
    gui       = LaunchConfiguration('gui')
    log_level = LaunchConfiguration('log_level')
    base_log  = common.log_args(log_level)   # 第三方底座节点统一追加

    # ── Core nodes ────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=base_log,
        parameters=[{'robot_description': rd, 'use_sim_time': False}],
    )

    # 单 controller_manager 管全 6 轴（臂 CANopen + 云台 HID）
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        arguments=base_log,
        parameters=[{'robot_description': rd}, controllers_yaml],
    )

    # 使能/失能/故障恢复服务：/arm_node/{enable,disable,recover}
    # （commander 及旧接口兼容，转发到 controller_manager 硬件组件状态）
    arm_driver_services = Node(
        package='robot_arm_driver', executable='arm_driver_services', output='screen',
    )

    # 相机视频流节点：仅 V4L2，不占用 HID，与 ros2_control 无冲突
    camera_params = os.path.join(gimbal_share, 'config', 'params.yaml')
    robot_camera_node = Node(
        package='robot_gimbal_node', executable='robot_camera_node',
        name='robot_camera_node', output='screen',
        parameters=[camera_params, {'publish_raw': True, 'publish_compressed': True}],
    )
    camera_view_node = Node(
        package='robot_gimbal_node', executable='camera_view',
        name='camera_view_gui', output='screen',
        condition=IfCondition(gui),
    )
    _ = (robot_camera_node, camera_view_node)   # 默认不启（保留定义，需要时加回列表）

    # ── Controller spawners ───────────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )
    arm_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )
    # gimbal_controller 不再 spawn：arm_controller 已 claim 全 6 轴（含云台 J4-6）

    # PV 模式备用：只 load+configure 不 activate（--inactive），
    # 由 /arm_node/set_mode_pv 与 arm_controller 互斥切换
    vel_ctrl_loader = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_velocity_controller', '--inactive',
                   '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )

    # ── GUI controller nodes（复用共享工厂）─────────────────────────────────────
    joint_position_ctrl    = common.gui_node(ctrl, 'joint_position',        'joint_position_gui')
    cartesian_moveit_ctrl  = common.gui_node(ctrl, 'cartesian_moveit',      'cartesian_moveit_gui')
    realtime_ik_ctrl       = common.gui_node(ctrl, 'cartesian_realtime_ik', 'cartesian_realtime_ik_gui')
    trajectory_ctrl        = common.gui_node(ctrl, 'cartesian_trajectory',  'cartesian_trajectory_gui')
    spherical_orbit_ctrl   = common.gui_node(ctrl, 'spherical_orbit',       'spherical_orbit_gui')
    velocity_ctrl          = common.gui_node(ctrl, 'cartesian_velocity',    'cartesian_velocity_gui')

    # ── MoveIt（需要 IK 的控制方式）────────────────────────────────────────────
    needs_moveit = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['cartesian_moveit','cartesian_realtime_ik',"
            "'cartesian_trajectory','spherical_orbit','commander']"
        ])
    )
    # 实物模式直接创建 move_group，使用 FollowJointTrajectory action 配置
    # （不 include moveit.launch.py，避免 Ros2ControlManager 找不到 Joint1-3 控制器）
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=base_log,
        parameters=[
            {'robot_description': rd},
            {'robot_description_semantic': rds},
            {'robot_description_kinematics': _load_yaml(
                os.path.join(desc_share, 'config', 'kinematics.yaml'))},
            {'robot_description_planning': _load_yaml(
                os.path.join(desc_share, 'config', 'joint_limits.yaml'))},
            _load_yaml(os.path.join(moveit_cfg, 'planning_pipeline.yaml')),
            _load_yaml(os.path.join(moveit_cfg, 'moveit_controllers_real.yaml')),
            {'use_sim_time': False,
             'start_state_max_bounds_error': 0.5},
        ],
        condition=needs_moveit,
    )

    # ── MoveIt Servo（cartesian_velocity 模式）─────────────────────────────────
    is_velocity  = IfCondition(PythonExpression(["'", ctrl, "' == 'cartesian_velocity'"]))
    is_visp_ibvs = IfCondition(PythonExpression(["'", ctrl, "' == 'visp_ibvs'"]))
    is_commander = IfCondition(PythonExpression(["'", ctrl, "' == 'commander'"]))
    needs_servo  = IfCondition(PythonExpression(
        ["'", ctrl, "' in ['cartesian_velocity']"]))

    def make_servo_node(condition, check_collisions=True):
        extra = {} if check_collisions else {'moveit_servo': {'check_collisions': False}}
        return Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            arguments=base_log,
            parameters=[
                servo_params,
                {'robot_description': rd,
                 'robot_description_semantic': rds,
                 'use_sim_time': False,
                 'use_gazebo': False},
                {'robot_description_kinematics': _load_yaml(
                    os.path.join(desc_share, 'config', 'kinematics.yaml'))},
                {'robot_description_planning': _load_yaml(
                    os.path.join(desc_share, 'config', 'joint_limits.yaml'))},
                extra,
            ],
            condition=condition,
        )

    # 安全姿态预移动（全零关节是运动学奇异点，Servo 启动前须先移走）
    move_to_safe_pose = common.safe_pose_action(needs_servo)

    # Servo 模式：t=3s 安全姿态预移动（3s 运动），t=7s 启动 servo_node + 控制器
    cartesian_velocity_start = TimerAction(
        period=7.0,
        actions=[make_servo_node(is_velocity, check_collisions=False), velocity_ctrl],
        condition=is_velocity,
    )

    # ── visp_ibvs 模式 ───────────────────────────────────────────────────────
    # IBVS 节点发布 /arm_controller/joint_trajectory 单点流（positions+velocities），
    # JTC（position→IP 模式）直接跟踪，无需旧 arm_node 的 PV 模式切换。
    # t=3s：发预备位姿（关节空间，3s 运动时间）
    visp_ibvs_prep = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/arm_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory',
                    ('{joint_names: [Joint1,Joint2,Joint3], '
                     'points: [{positions: [0.0, 1.0, -1.5], '
                     'time_from_start: {sec: 3, nanosec: 0}}]}'),
                ],
                output='screen',
                condition=is_visp_ibvs,
            ),
        ],
        condition=is_visp_ibvs,
    )
    # t=7s：等预备位姿到达后启动 IBVS 节点
    visp_ibvs_start = TimerAction(
        period=7.0,
        actions=common.visp_nodes(ctrl, 'visp_ibvs',
                                  robot_description=rd, use_sim_time=False,
                                  perception_topic='/ros2_algo_vision/report',
                                  control_depth=False),
        condition=is_visp_ibvs,
    )

    # ── commander 模式 ────────────────────────────────────────────────────────
    # t=10s：等 move_group planning scene 完全就绪后再启动
    commander_start = TimerAction(
        period=10.0,
        actions=common.commander_nodes(ctrl, gui, use_sim_time=False),
        condition=is_commander,
    )

    # 控制器 2 s 后 spawn（等 controller_manager / CANopen master 初始化）
    spawn_controllers = TimerAction(
        period=2.0,
        actions=[jsb_spawner, arm_ctrl_spawner, vel_ctrl_loader],
    )

    # ── 电机运行模式（402）按控制方式自动选择，经 /arm_node/set_mode_* 编排 ──────
    #   joint_position                → PP(1) 驱动器自规划（滑块点到点，6081 限速）
    #   轨迹/运镜/commander 等其余     → IP(7) 跟随上位机插补（bus.yml 默认，无需调用）
    #   cartesian_velocity/visp_ibvs  → PV(3) 速度伺服（待上层改发速度指令后接入，
    #                                   现阶段仍走 IP 位置流）
    set_mode_pp = TimerAction(
        period=6.0,   # 等 arm_controller 激活完成（t=2s spawn + 使能耗时）
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/arm_node/set_mode_pp',
                 'std_srvs/srv/Trigger', '{}'],
            output='screen',
        )],
        condition=IfCondition(PythonExpression(["'", ctrl, "' == 'joint_position'"])),
    )

    # t=3s：启动 GUI 控制器 + 安全姿态预移动（Servo 模式）
    spawn_gui = TimerAction(
        period=3.0,
        actions=[
            joint_position_ctrl, cartesian_moveit_ctrl, realtime_ik_ctrl,
            trajectory_ctrl, spherical_orbit_ctrl,
            move_to_safe_pose,   # Servo 模式专用（condition=needs_servo）
        ],
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        arm_driver_services,
        move_group_node,           # cartesian_moveit / realtime_ik / trajectory / orbit / commander
        spawn_controllers,
        set_mode_pp,               # t=6s，仅 joint_position 模式（402 切 PP）
        spawn_gui,
        cartesian_velocity_start,  # t=7s，仅 cartesian_velocity 模式
        visp_ibvs_prep,            # t=3s，visp_ibvs 预备位姿
        visp_ibvs_start,           # t=7s，visp_ibvs 启动 IBVS 节点
        commander_start,           # t=10s，仅 commander 模式
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='joint_position',
            description='控制方式: joint_position | cartesian_moveit | cartesian_realtime_ik | '
                        'cartesian_trajectory | spherical_orbit | cartesian_velocity | '
                        'visp_ibvs | commander',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否启动 commander_test_gui（仅 controller:=commander 时生效）: true | false',
        ),
        DeclareLaunchArgument(
            'arm_sim_mode', default_value='false',
            description='true=臂 J1-3 不连 CAN（mock 回显），云台照常；false=连接实物 CANopen',
        ),
        DeclareLaunchArgument(
            'can_interface', default_value=_default_can_interface(),
            description='SocketCAN 接口名，默认读 robot_arm_driver/config/can.yaml；'
                        '可临时覆盖：can0（真机）| vcan0（假从站联调）',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='第三方底座节点（move_group/servo/spawner/rsp/ros2_control 等）'
                        '日志级别，默认 warn 降噪，调试时设 info；自家业务节点不受影响',
        ),
        OpaqueFunction(function=_setup),
    ])
