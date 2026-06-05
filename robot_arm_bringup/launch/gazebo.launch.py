"""
@file gazebo.launch.py
@brief eMeetArm_models 机械臂 Gazebo 仿真启动文件
@version 1.3
@date 2026-06-03

@details 启动以下节点：
         - gzserver：物理仿真服务端（无 GPU 渲染，避免双窗口）
         - gzclient：渲染客户端（单独注入 NVIDIA PRIME 环境变量）
         - robot_state_publisher / spawn_entity / controllers / 控制 GUI
         - move_group + rviz2（cartesian / realtime / ruckig_ik / sphere_orbit 时自动启动）

         通过 controller 参数选择 9 种控制方式之一：
           slider             → arm_slider_controller            (PyQt5 关节滑块，无需 MoveIt)
           cartesian          → cartesian_controller             (MoveIt 笛卡尔直线规划)
           realtime           → cartesian_realtime_controller    (滑块即时 IK)
           ruckig             → cartesian_ruckig_streamer        (Ruckig 笛卡尔流式 + servo)
           ruckig_ik          → cartesian_ruckig_ik_streamer     (Ruckig 点到点+平面环绕)
           sphere_orbit       → spherical_orbit_streamer         (球面坐标环绕运镜)
           velocity           → cartesian_velocity_controller    (笛卡尔速度接口，手动点动)
           ibvs_control       → velocity + red_box_detector
                                + ibvs_control_node             (红色方块 IBVS 闭环)
           pose_command_debug → pose_command_debug               (PoseCommand 接口调试)
         MoveIt 由本文件自动 include，无需额外启动 moveit.launch.py。

         示例：
           ros2 launch robot_arm_bringup gazebo.launch.py                                         # 默认 slider
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=slider                      # 关节滑块
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian                   # MoveIt 笛卡尔直线
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=realtime                    # 滑块即时 IK
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=ruckig                      # Ruckig 笛卡尔流
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=ruckig_ik                   # Ruckig 点到点+环绕
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=sphere_orbit                # 球面轨道运镜
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=velocity                    # 笛卡尔速度（手动点动）
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=ibvs_control                # 红色方块 IBVS
           ros2 launch robot_arm_bringup gazebo.launch.py controller:=pose_command_debug          # PoseCommand 调试

@copyright Copyright (c) 2026 eMeet
"""

import os
import re
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_share       = get_package_share_directory('robot_arm_description')
    bringup_share    = get_package_share_directory('robot_arm_bringup')
    arm_share_parent = os.path.dirname(desc_share)   # Gazebo 解析 package://robot_arm_description/... 需要
    moveit_cfg       = os.path.join(bringup_share, 'config', 'moveit')
    urdf_path     = os.path.join(desc_share, 'urdf', 'eMeetArm_models.urdf')
    controllers_yaml_path = os.path.join(desc_share, 'config', 'controllers.yaml')
    world_file    = os.path.join(bringup_share, 'sim', 'gazebo', 'worlds', 'emeet_arm.world')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    with open(urdf_path, 'r') as f:
        robot_description = f.read().replace('CONTROLLERS_YAML_PATH', controllers_yaml_path)
    # Gazebo 需要 gazebo_ros2_control::GazeboSystemInterface，而 CameraHardwareInterface
    # 继承的是 hardware_interface::SystemInterface，两者不兼容。
    # 仿真时 Joint4-6 同样用 GazeboSystem 托管，sim_mode 参数同时去掉（Gazebo 不认识）。
    robot_description = robot_description.replace(
        '<plugin>emeet_camera_driver/CameraHardwareInterface</plugin>',
        '<plugin>gazebo_ros2_control/GazeboSystem</plugin>',
    )
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.DOTALL)
    robot_description = ' '.join(robot_description.split())

    # ── MoveIt Servo 用到的额外资源（仅 controller:=ruckig 时使用） ───────────
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

    # ── 控制方式参数 ──────────────────────────────────────────────────────────
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='slider',
        description='控制方式: slider | cartesian | realtime | ruckig | ruckig_ik | sphere_orbit | velocity | ibvs_control | pose_command_debug',
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

    slider_ctrl            = controller_node('slider',             'arm_slider_controller')
    cartesian_ctrl         = controller_node('cartesian',         'cartesian_controller')
    realtime_ctrl          = controller_node('realtime',          'cartesian_realtime_controller')
    ruckig_ctrl            = controller_node('ruckig',            'cartesian_ruckig_streamer')
    ruckig_ik_ctrl         = controller_node('ruckig_ik',         'cartesian_ruckig_ik_streamer')
    sphere_orbit_ctrl      = controller_node('sphere_orbit',      'spherical_orbit_streamer')
    velocity_ctrl          = controller_node('velocity',          'cartesian_velocity_controller')
    pose_command_debug_ctrl = controller_node('pose_command_debug', 'pose_command_debug')
    is_pose_cmd_debug = IfCondition(
        PythonExpression(["'", ctrl, "' == 'pose_command_debug'"])
    )
    pose_command_publisher_node = Node(
        package='robot_arm_node',
        executable='pose_command_publisher',
        output='screen',
        condition=is_pose_cmd_debug,
    )

    # ── MoveIt（需要 IK 的控制方式：cartesian / realtime / ruckig_ik / sphere_orbit）
    needs_moveit = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['cartesian','realtime','ruckig_ik','sphere_orbit','pose_command_debug']"
        ])
    )
    moveit_launch_path = os.path.join(bringup_share, 'launch', 'moveit.launch.py')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path),
        launch_arguments={'use_sim_time': 'true', 'rviz': 'true'}.items(),
        condition=needs_moveit,
    )

    # ── MoveIt Servo 条件 ────────────────────────────────────────────────────
    is_ruckig       = IfCondition(PythonExpression(["'", ctrl, "' == 'ruckig'"]))
    is_velocity     = IfCondition(PythonExpression(["'", ctrl, "' == 'velocity'"]))
    is_ibvs_control = IfCondition(PythonExpression(["'", ctrl, "' == 'ibvs_control'"]))
    needs_servo = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['ruckig','velocity','ibvs_control']"
        ])
    )

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

    # ── 安全姿态预移动（全零关节是运动学奇异点，servo 启动前须先移走）──────────
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
        condition=needs_servo,
    )

    ruckig_start_after_pose = TimerAction(
        period=4.0,
        actions=[make_servo_node(is_ruckig), ruckig_ctrl],
        condition=is_ruckig,
    )

    velocity_start_after_pose = TimerAction(
        period=4.0,
        actions=[make_servo_node(is_velocity, check_collisions=False),
                 velocity_ctrl],
        condition=is_velocity,
    )

    # ibvs_control：servo + velocity controller + 检测节点 + IBVS 控制器
    ibvs_control_start_after_pose = TimerAction(
        period=4.0,
        actions=[
            make_servo_node(is_ibvs_control, check_collisions=False),
            Node(package='robot_arm_node', executable='cartesian_velocity_controller',
                 output='screen', condition=is_ibvs_control),
            Node(package='robot_arm_node', executable='red_box_detector',
                 output='screen',
                 parameters=[{'use_sim_time': True}],
                 condition=is_ibvs_control),
            Node(package='robot_arm_node', executable='ibvs_control_node',
                 output='screen', condition=is_ibvs_control),
        ],
        condition=is_ibvs_control,
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
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    camera_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'camera_view.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", ctrl, "' != 'ibvs_control'"])
        ),
    )

    return LaunchDescription([
        controller_arg,
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
                on_exit=[slider_ctrl, cartesian_ctrl, realtime_ctrl,
                         move_to_safe_pose,
                         ruckig_start_after_pose,
                         velocity_start_after_pose,
                         ibvs_control_start_after_pose,
                         ruckig_ik_ctrl, sphere_orbit_ctrl,
                         pose_command_debug_ctrl,
                         pose_command_publisher_node],
            )
        ),
    ])
