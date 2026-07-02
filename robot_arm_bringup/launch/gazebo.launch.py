"""
@file gazebo.launch.py
@brief bringup 的 Gazebo 后端 —— 只管 Gazebo 底座，上层栈复用共享工厂
@version 1.5
@date 2026-07-01

@note  本文件是整机启动的 gazebo 后端（统一入口见 bringup.launch.py，backend:=gazebo）。
       职责限于「底座」：gzserver/gzclient + spawn_entity + gazebo_ros2_control 起 6 轴控制器 +
       Gazebo 环境/世界。上层栈（controller GUI / servo / commander / ibvs / visp / 安全预移动）
       的节点定义复用 launch/_arm_launch_common.py，与 mujoco/real 共用一份，避免各写一遍。
       move_group 仍走本包 moveit.launch.py（含 rviz），为 gazebo 特有。

@details 启动以下节点：
         - gzserver：物理仿真服务端（无 GPU 渲染，避免双窗口）
         - gzclient：渲染客户端（单独注入 NVIDIA PRIME 环境变量）
         - robot_state_publisher / spawn_entity / controllers / 控制 GUI
         - move_group + rviz2（需要 IK 的控制方式时自动启动）

         通过 controller 参数选择控制方式（参数名 = 对应 GUI 名去掉 _gui 后缀）：
           joint_position        → joint_position_gui          (PyQt5 关节滑块，无需 MoveIt)
           cartesian_moveit      → cartesian_moveit_gui        (MoveIt 笛卡尔直线规划)
           cartesian_realtime_ik → cartesian_realtime_ik_gui   (滑块即时 IK)
           cartesian_trajectory  → cartesian_trajectory_gui    (Ruckig 点到点 + 环绕，批量 IK)
           spherical_orbit       → spherical_orbit_gui         (球面坐标环绕运镜)
           cartesian_velocity    → cartesian_velocity_gui      (笛卡尔速度接口，手动点动，MoveIt Servo)
           visp_ibvs_control     → cartesian_velocity_gui + red_box_detector
                                   + visp_ibvs_node             (红色方块 IBVS 闭环，ViSP C++ 版)
           commander             → arm_commander_node          (Arm Commander 中间层)
                                   + commander_test_gui         (Director 视角测试 GUI，需要 MoveIt/IK，gui:=false 可关闭)
         MoveIt 由本文件自动 include，无需额外启动 moveit.launch.py。

         示例：
           ros2 launch robot_arm_bringup gazebo.launch.py                                                                         # 默认 joint_position
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=joint_position                                              # 关节滑块
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_moveit                                            # MoveIt 笛卡尔直线
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_realtime_ik                                       # 滑块即时 IK
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_trajectory                                        # Ruckig 点到点+环绕
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=spherical_orbit                                             # 球面轨道运镜
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_velocity                                          # 笛卡尔速度（手动点动）
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=visp_ibvs_control                                          # 红色方块 IBVS（ViSP C++ 版，默认世界）
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=visp_ibvs_control world:=ibvs_tracking_test                 # 红色方块 IBVS（U 形桌+圆周移动方块，推荐跟踪测试）
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=commander                                                   # Arm Commander 中间层（含 GUI）
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=commander gui:=false                                        # Arm Commander 中间层（无 GUI）

         world 参数（默认 emeet_arm）：
           world:=emeet_arm            → 标准工作台场景（默认）
           world:=ibvs_tracking_test   → IBVS 跟踪测试专用：U 形三桌 + 红色方块圆周运动（r=0.12m，周期 12s）

@copyright Copyright (c) 2026 eMeet
"""

import os
import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

# 上层栈节点工厂（与 mujoco/real 共用同一份定义）
sys.path.insert(0, os.path.dirname(__file__))
import _arm_launch_common as common   # noqa: E402


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_share       = get_package_share_directory('robot_arm_description')
    bringup_share    = get_package_share_directory('robot_arm_bringup')
    arm_share_parent = os.path.dirname(desc_share)   # Gazebo 解析 package://robot_arm_description/... 需要
    moveit_cfg       = os.path.join(bringup_share, 'config', 'moveit')
    xacro_path        = os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro')
    controllers_yaml_path = os.path.join(desc_share, 'config', 'controllers.yaml')
    worlds_dir    = os.path.join(bringup_share, 'sim', 'gazebo', 'worlds')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            'sim_mode':         'true',
            'gazebo_camera':    'true',
            'controllers_yaml': controllers_yaml_path,
        },
    ).toxml()

    # ── MoveIt Servo 用到的额外资源（servo 模式：cartesian_velocity / ibvs_control）─
    with open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf'), 'r') as f:
        srdf_content = f.read()
    servo_params = {
        'moveit_servo': _load_yaml(os.path.join(moveit_cfg, 'servo_config.yaml')),
    }
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(os.path.join(desc_share, 'config', 'kinematics.yaml')),
    }
    robot_description_planning = {
        'robot_description_planning': _load_yaml(os.path.join(desc_share, 'config', 'joint_limits.yaml')),
    }

    # ── 控制方式参数（参数名 = 对应 GUI 名去掉 _gui 后缀）──────────────────────
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='joint_position',
        description=('控制方式: joint_position | cartesian_moveit | cartesian_realtime_ik | '
                     'cartesian_trajectory | spherical_orbit | cartesian_velocity | '
                     'visp_ibvs_control | commander'),
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='是否启动 commander_test_gui（仅 controller:=commander 时生效）: true | false',
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='emeet_arm',
        description='世界文件名（不含 .world 后缀），位于 sim/gazebo/worlds/ 下。'
                    '示例: emeet_arm | ibvs_tracking_test',
    )
    ctrl       = LaunchConfiguration('controller')
    gui        = LaunchConfiguration('gui')
    world_name = LaunchConfiguration('world')
    world_file = PathJoinSubstitution([worlds_dir, [world_name, '.world']])

    joint_position_ctrl    = common.gui_node(ctrl, 'joint_position',        'joint_position_gui')
    cartesian_moveit_ctrl  = common.gui_node(ctrl, 'cartesian_moveit',      'cartesian_moveit_gui')
    realtime_ik_ctrl       = common.gui_node(ctrl, 'cartesian_realtime_ik', 'cartesian_realtime_ik_gui')
    trajectory_ctrl        = common.gui_node(ctrl, 'cartesian_trajectory',  'cartesian_trajectory_gui')
    spherical_orbit_ctrl   = common.gui_node(ctrl, 'spherical_orbit',       'spherical_orbit_gui')
    velocity_ctrl          = common.gui_node(ctrl, 'cartesian_velocity',    'cartesian_velocity_gui')

    # ── commander 模式 ─────────────────────────────────────────────────────────
    is_commander = IfCondition(PythonExpression(["'", ctrl, "' == 'commander'"]))

    # arm_commander_node + test_gui：延迟 5s 直接启动（无需等 arm_controller_spawner）
    # 它们是纯 ROS 客户端/服务端，启动后会自动等待底层资源就绪
    commander_start = TimerAction(
        period=5.0,
        actions=common.commander_nodes(ctrl, gui, use_sim_time=True),
        condition=is_commander,
    )
    # ── MoveIt（需要 IK 的控制方式，含 commander）─────────────────────────────
    needs_moveit = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['cartesian_moveit','cartesian_realtime_ik',"
            "'cartesian_trajectory','spherical_orbit','commander']"
        ])
    )
    moveit_launch_path = os.path.join(bringup_share, 'launch', 'moveit.launch.py')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path),
        launch_arguments={'use_sim_time': 'true', 'rviz': 'true'}.items(),
        condition=needs_moveit,
    )

    # ── MoveIt Servo 条件（cartesian_velocity / ibvs_control）────────────────────
    is_velocity      = IfCondition(PythonExpression(["'", ctrl, "' == 'cartesian_velocity'"]))
    is_visp_ibvs     = IfCondition(PythonExpression(["'", ctrl, "' == 'visp_ibvs_control'"]))
    def make_servo_node(condition, check_collisions=True):
        # ibvs/velocity 模式无 move_group，禁用碰撞检测避免 run_duration 超时
        extra = {} if check_collisions else \
            {'moveit_servo': {'check_collisions': False}}
        return Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[
                servo_params,
                {'robot_description': robot_description,
                 'robot_description_semantic': srdf_content,
                 'use_sim_time': True},
                robot_description_kinematics,
                robot_description_planning,
                extra,
            ],
            condition=condition,
        )

    # visp_ibvs_control：Pinocchio 加权 Jacobian，直接发 JointTrajectory，无需 Servo
    visp_ibvs_start_after_pose = TimerAction(
        period=4.0,
        actions=common.visp_nodes(ctrl, 'visp_ibvs_control',
                                  robot_description=robot_description,
                                  use_sim_time=True, control_depth=False),
        condition=is_visp_ibvs,
    )

    # ── 安全姿态预移动（全零关节是运动学奇异点，servo 启动前须先移走）──────────
    # commander 模式不需要预移动，由上层自行决定初始姿态
    needs_safe_pose = IfCondition(
        PythonExpression(["'", ctrl, "' in ['cartesian_velocity']"])
    )
    move_to_safe_pose = common.safe_pose_action(needs_safe_pose)

    velocity_start_after_pose = TimerAction(
        period=4.0,
        actions=[make_servo_node(is_velocity, check_collisions=False),
                 velocity_ctrl],
        condition=is_velocity,
    )

    # ── ibvs_tracking_test 世界：红色方块圆周运动 cmd_vel 发布 ────────────────
    # libgazebo_ros_planar_move 插件监听 /red_box/cmd_vel；
    # v=0.0524 m/s, ω=-0.5236 rad/s → 顺时针 r=0.10 m 圆，圆心 (0.65,0)，周期 12 s
    red_box_circle_mover = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--rate', '10',
                     '/red_box/cmd_vel', 'geometry_msgs/msg/Twist',
                     '{linear: {x: 0.0524, y: 0.0, z: 0.0},'
                     ' angular: {x: 0.0, y: 0.0, z: -0.5236}}'],
                output='screen',
            ),
        ],
        condition=IfCondition(
            PythonExpression(["'", world_name, "' == 'ibvs_tracking_test'"])
        ),
    )

    # ── gzserver（物理引擎，无需 GPU 渲染）────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    # ── gzclient（渲染窗口，单独注入 NVIDIA PRIME 避免双窗口）────────────────
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose', 'false'],
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD':  '1',
            '__GLX_VENDOR_LIBRARY_NAME':  'nvidia',
            '__GL_SYNC_TO_VBLANK':        '0',
            '__GL_MaxFramesAllowed':       '1',
        },
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'eMeetArm'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '30'],
    )

    camera_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'camera_view.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' not in ['visp_ibvs_control']"])
        ),
    )

    return LaunchDescription([
        controller_arg,
        gui_arg,
        world_arg,
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=(arm_share_parent
                   + ':' + os.path.join(bringup_share, 'sim', 'gazebo', 'models')
                   + ':/usr/share/gazebo-11/models'),
        ),
        gzserver,
        gzclient,
        camera_view,
        robot_state_publisher,
        spawn_entity,
        moveit_launch,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner, arm_controller_spawner],
            )
        ),
        # arm_controller 起来后启动选中的控制方式（条件互斥，仅一个生效）
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[joint_position_ctrl, cartesian_moveit_ctrl, realtime_ik_ctrl,
                         move_to_safe_pose,
                         velocity_start_after_pose,
                         visp_ibvs_start_after_pose,
                         trajectory_ctrl, spherical_orbit_ctrl],
            )
        ),
        # commander：节点独立延迟启动（不等 arm_controller_spawner，5s 后自动出现）
        commander_start,
        # ibvs_tracking_test 世界：红色方块圆周运动（5s 后开始发布 cmd_vel）
        red_box_circle_mover,
    ])
