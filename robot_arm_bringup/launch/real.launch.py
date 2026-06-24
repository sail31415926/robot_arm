"""
@file real.launch.py
@brief eMeetArm 实物一键启动文件
@version 1.6
@date 2026-06-24

启动拓扑：
  Joint1-3  →  arm_node（CANopen，JointTrajectory + joint_states 整体接口）
  Joint4-6  →  ros2_control / CameraHardwareInterface（HID）
               └─ gimbal_controller（JointTrajectoryController）

用法（与 Gazebo launch 参数一致）：
  ros2 launch robot_arm_bringup real.launch.py controller:=joint_position             # 关节滑块（PP 模式）
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_moveit           # MoveIt 笛卡尔直线
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_realtime_ik      # 滑块即时 IK
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_trajectory       # Ruckig 点到点+环绕
  ros2 launch robot_arm_bringup real.launch.py controller:=spherical_orbit            # 球面轨道运镜
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_velocity         # 笛卡尔速度（手动点动，MoveIt Servo）
  ros2 launch robot_arm_bringup real.launch.py controller:=ibvs_control               # 红色方块 IBVS 闭环（Python，含 Servo）
  ros2 launch robot_arm_bringup real.launch.py controller:=visp_ibvs                  # 红色方块 IBVS 闭环（C++ ViSP+Pinocchio，直接 PV，无 Servo）

  ros2 launch robot_arm_bringup real.launch.py controller:=commander                  # Arm Commander 中间层（含 GUI）
  ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false       # Arm Commander 中间层（无 GUI，纯话题接口，同时关闭视频流窗口）
  gui 参数（默认 true）同时控制：commander_test_gui + camera_view 窗口

视频流由 robot_camera_node（robot_gimbal_node 包）单独启动，仅占用 V4L2，
不与 ros2_control CameraHardwareInterface（HID）冲突，可同时运行。

@copyright Copyright (c) 2026 eMeet
"""

import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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
      <plugin>robot_gimbal_driver/CameraHardwareInterface</plugin>
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
        <param name="min">-1.5</param><param name="max">0.5</param><!-- [−1.5, +0.5] rad ↔ HID [+85.94°, −28.65°] -->
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
    driver_share     = get_package_share_directory('robot_arm_driver')
    gimbal_share     = get_package_share_directory('robot_gimbal_node')
    arm_yaml         = os.path.join(driver_share, 'config', 'arm.yaml')
    controllers_yaml = os.path.join(desc_share, 'config', 'controllers.yaml')
    moveit_cfg       = os.path.join(bringup_share, 'config', 'moveit')
    servo_params     = _load_yaml(os.path.join(moveit_cfg, 'servo_config.yaml'))
    srdf_content     = open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf')).read()

    _xacro_path  = os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro')
    _arm_urdf    = xacro.process_file(
        _xacro_path, mappings={'sim_mode': 'false', 'gazebo_camera': 'false'}
    ).toxml()

    # ── Launch arguments ──────────────────────────────────────────────────────
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=_arm_urdf,
        description='完整机器人 URDF/XML 字符串；默认使用 arm_sim.urdf.xacro 处理结果',
    )
    robot_description_semantic_arg = DeclareLaunchArgument(
        'robot_description_semantic',
        default_value=srdf_content,
        description='SRDF 语义描述字符串；默认使用 eMeetArm_models.srdf',
    )

    controller_arg = DeclareLaunchArgument(
        'controller', default_value='joint_position',
        description='控制方式: joint_position | cartesian_moveit | cartesian_realtime_ik | '
                    'cartesian_trajectory | spherical_orbit | cartesian_velocity | '
                    'ibvs_control | visp_ibvs | commander',
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='是否启动 commander_test_gui（仅 controller:=commander 时生效）: true | false',
    )
    ctrl = LaunchConfiguration('controller')
    gui  = LaunchConfiguration('gui')

    camera_urdf = _camera_ros2_control_urdf('auto')
    rd  = LaunchConfiguration('robot_description')
    rds = LaunchConfiguration('robot_description_semantic')

    # ── Core nodes ────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': rd, 'use_sim_time': False}],
    )

    # ros2_control 只管摄像头云台（Joint4-6）
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': camera_urdf}, controllers_yaml],
    )

    # 机械臂整体节点：Joint1-3，JointTrajectory 输入，joint_states 输出
    # joint_position 模式使用 PP（轮廓位置）模式，其余模式使用 IP（插补位置）模式
    arm_node = Node(
        package='robot_arm_driver', executable='arm_node',
        name='arm_node', output='screen',
        parameters=[arm_yaml, {
            # controller → motion_mode 映射规则：
            #   joint_position              → pp（单点目标跳转）
            #   ibvs_control / visp_ibvs    → pv（速度闭环，直接透传 velocities）
            #   其余（cartesian_* / commander / spherical_orbit）→ ip（位置轨迹插补）
            'motion_mode': PythonExpression([
                "'pp' if '", ctrl, "' in ('joint_position', 'visp_ibvs') else "
                "'pv' if '", ctrl, "' == 'ibvs_control' else "
                "'ip'"
            ])
        }],
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

    # ── Camera controller spawners ────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    camera_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['gimbal_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ── GUI controller nodes ──────────────────────────────────────────────────
    def ctrl_node(mode, exe):
        return Node(
            package='robot_arm_node', executable=exe, output='screen',
            condition=IfCondition(PythonExpression(["'", ctrl, "' == '", mode, "'"])),
        )

    joint_position_ctrl    = ctrl_node('joint_position',        'joint_position_gui')
    cartesian_moveit_ctrl  = ctrl_node('cartesian_moveit',      'cartesian_moveit_gui')
    realtime_ik_ctrl       = ctrl_node('cartesian_realtime_ik', 'cartesian_realtime_ik_gui')
    trajectory_ctrl        = ctrl_node('cartesian_trajectory',  'cartesian_trajectory_gui')
    spherical_orbit_ctrl   = ctrl_node('spherical_orbit',       'spherical_orbit_gui')
    velocity_ctrl          = ctrl_node('cartesian_velocity',    'cartesian_velocity_gui')

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

    # ── MoveIt Servo（cartesian_velocity / ibvs_control / commander 模式）──────
    is_velocity     = IfCondition(PythonExpression(["'", ctrl, "' == 'cartesian_velocity'"]))
    is_ibvs_control = IfCondition(PythonExpression(["'", ctrl, "' == 'ibvs_control'"]))
    is_visp_ibvs    = IfCondition(PythonExpression(["'", ctrl, "' == 'visp_ibvs'"]))
    is_commander    = IfCondition(PythonExpression(["'", ctrl, "' == 'commander'"]))
    needs_servo     = IfCondition(PythonExpression(
        ["'", ctrl, "' in ['cartesian_velocity','ibvs_control']"]))

    def make_servo_node(condition, check_collisions=True):
        extra = {} if check_collisions else {'moveit_servo': {'check_collisions': False}}
        return Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
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

    # 电机状态控制 GUI：一键 使能/失能/故障复位（作用于整体 arm_node 的 Joint1-3）
    # commander 模式由 commander_test_gui 统一管理使能/复位，不需要此窗口
    motor_state_gui = Node(
        package='robot_arm_driver', executable='motor_state_control_gui',
        name='motor_state_control_gui', output='screen',
        parameters=[{'namespaces': ['/arm_node'], 'labels': ['ARM']}],
        condition=IfCondition(PythonExpression(["'", ctrl, "' != 'commander'"])),
    )

    # Servo 模式：t=3s 安全姿态预移动（3s 运动），t=7s 启动 servo_node + 控制器
    cartesian_velocity_start = TimerAction(
        period=7.0,
        actions=[make_servo_node(is_velocity, check_collisions=False), velocity_ctrl],
        condition=is_velocity,
    )
    ibvs_start = TimerAction(
        period=7.0,
        actions=[
            make_servo_node(is_ibvs_control, check_collisions=False),
            Node(package='robot_arm_node', executable='cartesian_velocity_gui',
                 output='screen', condition=is_ibvs_control),
            Node(package='robot_arm_node', executable='red_box_detector',
                 output='screen',
                 parameters=[{'use_sim_time': False}],
                 condition=is_ibvs_control),
            Node(package='robot_arm_node', executable='ibvs_control_node',
                 output='screen', condition=is_ibvs_control),
        ],
        condition=is_ibvs_control,
    )

    # ── visp_ibvs 模式 ───────────────────────────────────────────────────────
    # arm_node 以 IP 模式启动（可接收位置指令）
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
    # t=7s：切换到 PV 模式后启动 IBVS（等预备位姿到达）
    visp_ibvs_start = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/arm_node/pv_mode',
                     'std_srvs/srv/Trigger', '{}'],
                output='screen',
                condition=is_visp_ibvs,
            ),
            Node(package='robot_arm_node', executable='red_box_detector',
                 output='screen',
                 parameters=[{'use_sim_time': False}],
                 condition=is_visp_ibvs),
            Node(package='robot_arm_node', executable='visp_ibvs_node',
                 output='screen',
                 parameters=[{'robot_description': rd,
                               'use_sim_time': False,
                               'perception_topic': '/ros2_algo_vision/report',
                               'control_depth': False}],
                 condition=is_visp_ibvs),
            Node(package='robot_arm_node', executable='visp_ibvs_gui',
                 output='screen',
                 condition=is_visp_ibvs),
        ],
        condition=is_visp_ibvs,
    )

    # ── commander 模式 ────────────────────────────────────────────────────────
    # t=10s：等 move_group planning scene 完全就绪后再启动
    commander_start = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='robot_arm_node',
                executable='arm_commander_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
                condition=is_commander,
            ),
            Node(
                package='robot_arm_node',
                executable='commander_test_gui',
                output='screen',
                condition=IfCondition(PythonExpression(
                    ["'", ctrl, "' == 'commander' and '", gui, "' == 'true'"]
                )),
            ),
        ],
        condition=is_commander,
    )

    # 摄像头控制器 2 s 后 spawn（等 ros2_control_node 初始化）
    spawn_camera = TimerAction(
        period=2.0,
        actions=[jsb_spawner, camera_ctrl_spawner],
    )

    # t=3s：启动 GUI 控制器 + 安全姿态预移动（Servo 模式）
    spawn_gui = TimerAction(
        period=3.0,
        actions=[
            joint_position_ctrl, cartesian_moveit_ctrl, realtime_ik_ctrl,
            trajectory_ctrl, spherical_orbit_ctrl,
            motor_state_gui,     # 电机状态控制 GUI（所有控制方式通用）
            move_to_safe_pose,   # Servo 模式专用（condition=needs_servo）
        ],
    )

    return LaunchDescription([
        robot_description_arg,
        robot_description_semantic_arg,
        controller_arg,
        gui_arg,
        robot_state_publisher,
        ros2_control_node,
        arm_node,
        # robot_camera_node,
        #camera_view_node,
        move_group_node,           # cartesian_moveit / cartesian_realtime_ik / cartesian_trajectory / spherical_orbit
        spawn_camera,
        spawn_gui,
        cartesian_velocity_start,  # t=7s，仅 cartesian_velocity 模式
        ibvs_start,                # t=7s，仅 ibvs_control 模式
        visp_ibvs_prep,            # t=3s，visp_ibvs 预备位姿（IP 模式）
        visp_ibvs_start,           # t=7s，visp_ibvs 切 PV + 启动 IBVS 节点
        commander_start,           # t=10s，仅 commander 模式
    ])
