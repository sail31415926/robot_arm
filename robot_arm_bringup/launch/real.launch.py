"""
@file real.launch.py
@brief eMeetArm 实物一键启动文件

启动拓扑：
  Joint1-3  →  arm_node（CANopen，JointTrajectory + joint_states 整体接口）
  Joint4-6  →  ros2_control / CameraHardwareInterface（HID）
               └─ gimbal_controller（JointTrajectoryController）

用法（与 Gazebo launch 参数一致，共 9 种控制方式）：
  ros2 launch robot_arm_bringup real.launch.py controller:=slider             # 关节滑块（PP 模式）
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian          # MoveIt 笛卡尔直线
  ros2 launch robot_arm_bringup real.launch.py controller:=realtime           # 滑块即时 IK
  ros2 launch robot_arm_bringup real.launch.py controller:=ruckig             # Ruckig 笛卡尔流 + Servo
  ros2 launch robot_arm_bringup real.launch.py controller:=ruckig_ik          # Ruckig 点到点+环绕
  ros2 launch robot_arm_bringup real.launch.py controller:=sphere_orbit       # 球面轨道运镜
  ros2 launch robot_arm_bringup real.launch.py controller:=velocity           # 笛卡尔速度（手动点动）
  ros2 launch robot_arm_bringup real.launch.py controller:=ibvs_control       # 红色方块 IBVS 闭环
  ros2 launch robot_arm_bringup real.launch.py controller:=pose_command_debug # PoseCommand 接口调试
  ros2 launch robot_arm_bringup real.launch.py camera_type:=pixy

视频流由 robot_camera_node（robot_gimbal_node 包）单独启动，仅占用 V4L2，
不与 ros2_control CameraHardwareInterface（HID）冲突，可同时运行。

@date 2026-05-29
"""

import os
import sys
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
    urdf_path        = os.path.join(desc_share, 'urdf', 'eMeetArm_models.urdf')
    arm_yaml         = os.path.join(driver_share, 'config', 'arm.yaml')
    controllers_yaml = os.path.join(desc_share, 'config', 'controllers.yaml')
    moveit_cfg       = os.path.join(bringup_share, 'config', 'moveit')
    servo_params     = _load_yaml(os.path.join(moveit_cfg, 'servo_config.yaml'))
    srdf_content     = open(os.path.join(desc_share, 'srdf', 'eMeetArm_models.srdf')).read()

    # ── Launch arguments ──────────────────────────────────────────────────────
    camera_type_arg = DeclareLaunchArgument(
        'camera_type', default_value='auto',
        description='摄像头型号: auto | pixy | e7002 | piko',
    )
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='slider',
        description='控制方式: slider | cartesian | realtime | ruckig | ruckig_ik | sphere_orbit | velocity | ibvs_control | pose_command_debug',
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
    # slider 模式使用 PP（轮廓位置）模式，其余模式使用 IP（插补位置）模式
    arm_node = Node(
        package='robot_arm_driver', executable='arm_node',
        name='arm_node', output='screen',
        parameters=[arm_yaml, {
            'motion_mode': PythonExpression(
                ["'pp' if '", ctrl, "' == 'slider' else 'ip'"]
            )
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

    slider_ctrl            = ctrl_node('slider',             'arm_slider_controller')
    cartesian_ctrl         = ctrl_node('cartesian',          'cartesian_controller')
    realtime_ctrl          = ctrl_node('realtime',           'cartesian_realtime_controller')
    ruckig_ctrl            = ctrl_node('ruckig',             'cartesian_ruckig_streamer')
    ruckig_ik_ctrl         = ctrl_node('ruckig_ik',          'cartesian_ruckig_ik_streamer')
    sphere_orbit_ctrl      = ctrl_node('sphere_orbit',       'spherical_orbit_streamer')
    velocity_ctrl          = ctrl_node('velocity',           'cartesian_velocity_controller')
    pose_command_debug_ctrl = ctrl_node('pose_command_debug', 'pose_command_debug')

    # ── MoveIt（需要 IK 的控制方式：cartesian / realtime / ruckig_ik / sphere_orbit / pose_command_debug）
    needs_moveit = IfCondition(
        PythonExpression([
            "'", ctrl, "' in ['cartesian','realtime','ruckig_ik','sphere_orbit','pose_command_debug']"
        ])
    )
    # 实物模式直接创建 move_group，使用 FollowJointTrajectory action 配置
    # （不 include moveit.launch.py，避免 Ros2ControlManager 找不到 Joint1-3 控制器）
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': full_urdf},
            {'robot_description_semantic': srdf_content},
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

    # ── MoveIt Servo（ruckig / velocity / ibvs_control 模式）────────────────────
    is_ruckig       = IfCondition(PythonExpression(["'", ctrl, "' == 'ruckig'"]))
    is_velocity     = IfCondition(PythonExpression(["'", ctrl, "' == 'velocity'"]))
    is_ibvs_control = IfCondition(PythonExpression(["'", ctrl, "' == 'ibvs_control'"]))
    needs_servo     = IfCondition(PythonExpression(
        ["'", ctrl, "' in ['ruckig','velocity','ibvs_control']"]))
    is_pose_cmd_debug = IfCondition(PythonExpression(["'", ctrl, "' == 'pose_command_debug'"]))

    def make_servo_node(condition, check_collisions=True):
        extra = {} if check_collisions else {'moveit_servo': {'check_collisions': False}}
        return Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[
                servo_params,
                {'robot_description': full_urdf,
                 'robot_description_semantic': srdf_content,
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

    pose_command_publisher_node = Node(
        package='robot_arm_node', executable='pose_command_publisher',
        output='screen', condition=is_pose_cmd_debug,
    )

    # Servo 模式：t=3s 安全姿态预移动（3s 运动），t=7s 启动 servo_node + 控制器
    ruckig_start = TimerAction(
        period=7.0,
        actions=[make_servo_node(is_ruckig), ruckig_ctrl],
        condition=is_ruckig,
    )
    velocity_start = TimerAction(
        period=7.0,
        actions=[make_servo_node(is_velocity, check_collisions=False), velocity_ctrl],
        condition=is_velocity,
    )
    ibvs_start = TimerAction(
        period=7.0,
        actions=[
            make_servo_node(is_ibvs_control, check_collisions=False),
            Node(package='robot_arm_node', executable='cartesian_velocity_controller',
                 output='screen', condition=is_ibvs_control),
            Node(package='robot_arm_node', executable='red_box_detector',
                 output='screen', condition=is_ibvs_control),
            Node(package='robot_arm_node', executable='ibvs_control_node',
                 output='screen', condition=is_ibvs_control),
        ],
        condition=is_ibvs_control,
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
            slider_ctrl, cartesian_ctrl, realtime_ctrl,
            ruckig_ik_ctrl, sphere_orbit_ctrl,
            pose_command_debug_ctrl, pose_command_publisher_node,
            move_to_safe_pose,   # Servo 模式专用（condition=needs_servo）
        ],
    )

    return LaunchDescription([
        camera_type_arg,
        controller_arg,
        robot_state_publisher,
        ros2_control_node,
        arm_node,
        robot_camera_node,
        camera_view_node,
        move_group_node,     # cartesian / realtime / ruckig_ik / sphere_orbit / pose_command_debug
        spawn_camera,
        spawn_gui,
        ruckig_start,        # t=7s，仅 ruckig 模式
        velocity_start,      # t=7s，仅 velocity 模式
        ibvs_start,          # t=7s，仅 ibvs_control 模式
    ])
