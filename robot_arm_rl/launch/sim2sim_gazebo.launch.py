"""
@file   sim2sim_gazebo.launch.py
@brief  RL Sim2Sim 独立 Gazebo 工程：专用世界 + 臂 + RL 策略 + 运动主体，一键起

用法（须在 source 了 venv 的终端里，SB3/mujoco 在 venv 中）：
  ros2 launch robot_arm_rl sim2sim_gazebo.launch.py                    # 完整演示
  ros2 launch robot_arm_rl sim2sim_gazebo.launch.py gui:=false        # 无头验证
  ros2 launch robot_arm_rl sim2sim_gazebo.launch.py subject_speed:=0.0  # 静止主体
  ros2 launch robot_arm_rl sim2sim_gazebo.launch.py camera_view:=true   # 附带相机之眼窗口

组成（全部由本包资产驱动，不依赖 robot_arm_gazebo 的 world）：
  rl_world.world   净室场景：矮台(0.45) + planar_move 红盒 @(0.90,0,0.47)，
                   几何与训练场景 rl_env.xml 对齐，全 visual-only 无碰撞陷阱
  gzserver/gzclient + robot_state_publisher + spawn_entity + JTC 控制器
                   （robot_description 由 robot_arm_description 的 xacro 只读生成）
  rl_policy_node   50Hz 策略 → /arm_controller/joint_trajectory
  subject_mover    路点游走驱动红盒(/rl_subject/cmd_vel) + 位姿转发给策略

验收数据：节点日志 [验收] 行，或 ros2 topic echo /rl_policy/framing

实现说明：RL 脚本经 realpath 从源码目录运行（rl_env.xml 的跨包网格相对路径
按源码布局写，install 空间解析不了）；解释器优先取 VIRTUAL_ENV。

@copyright Copyright (c) 2026 eMeet
"""

import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# realpath 穿透 colcon symlink-install，定位本包源码树
_SRC_LAUNCH = os.path.dirname(os.path.realpath(__file__))
_PKG_SRC    = os.path.normpath(os.path.join(_SRC_LAUNCH, '..'))
_SCRIPTS    = os.path.join(_PKG_SRC, 'scripts')
_MODELS     = os.path.join(_PKG_SRC, 'models')
_WORLD      = os.path.join(_PKG_SRC, 'sim', 'gazebo', 'rl_world.world')

# ros2 launch 本身跑在系统 python 上，看不见 venv 的 SB3/mujoco
_PY = (os.path.join(os.environ['VIRTUAL_ENV'], 'bin', 'python3')
       if 'VIRTUAL_ENV' in os.environ else sys.executable)


def generate_launch_description():
    desc_share    = get_package_share_directory('robot_arm_description')
    bringup_share = get_package_share_directory('robot_arm_bringup')
    gazebo_ros    = get_package_share_directory('gazebo_ros')
    # Gazebo 解析 package://<pkg>/... 需要各 share 的父目录
    arm_share_parent    = os.path.dirname(desc_share)
    gimbal_share_parent = os.path.dirname(
        get_package_share_directory('robot_gimbal_description_v2'))

    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    camera_arg = DeclareLaunchArgument(
        'camera_view', default_value='false',
        description='是否弹 rqt_image_view 看相机之眼（构图效果）')
    speed_arg = DeclareLaunchArgument(
        'subject_speed', default_value='0.2',
        description='主体游走速度 m/s（0 = 静止）')
    model_arg = DeclareLaunchArgument(
        'model_path', default_value=os.path.join(_MODELS, 'best', 'best_model'))
    vecnorm_arg = DeclareLaunchArgument(
        'vec_norm_path',
        default_value=os.path.join(_MODELS, 'vec_normalize_best.pkl'))

    # robot_description：只读引用 robot_arm_description 的 xacro（含 Gazebo 插件与
    # ros2_control 配置，controllers.yaml 沿用 bringup 的正式配置保持后端一致）
    robot_description = xacro.process_file(
        os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro'),
        mappings={
            'sim_mode':         'true',
            'gazebo_camera':    'true',
            # 臂落地（默认 0.40 垫高是配 emeet_arm.world 高桌的）：
            # base_link 与世界原点重合 → 与训练场景 rl_env.xml 布局 1:1，
            # 策略的 base 系坐标即世界坐标，主体位姿零转换
            'base_height':      '0.0',
            'controllers_yaml': os.path.join(bringup_share, 'config',
                                             'controllers.yaml'),
        }).toxml()

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': _WORLD, 'verbose': 'false'}.items())

    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose', 'false'],
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__GL_SYNC_TO_VBLANK':       '0',
            '__GL_MaxFramesAllowed':     '1',
        },
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'eMeetArm'],
        output='screen')

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen')
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '60'],
        output='screen')

    camera_view = Node(
        package='rqt_image_view', executable='rqt_image_view',
        arguments=['/camera/camera_sensor/image_raw'],
        condition=IfCondition(LaunchConfiguration('camera_view')),
        output='screen')

    rl_node = TimerAction(period=12.0, actions=[ExecuteProcess(
        cmd=[_PY, os.path.join(_SCRIPTS, 'deploy', 'rl_policy_node.py'),
             '--ros-args',
             '-p', ['model_path:=',    LaunchConfiguration('model_path')],
             '-p', ['vec_norm_path:=', LaunchConfiguration('vec_norm_path')],
             '-p', 'subject_x:=0.90', '-p', 'subject_y:=0.00', '-p', 'subject_z:=0.47'],
        output='screen')])

    mover = TimerAction(period=15.0, actions=[ExecuteProcess(
        cmd=[_PY, os.path.join(_SCRIPTS, 'deploy', 'subject_mover.py'),
             '--ros-args', '-p', ['speed:=', LaunchConfiguration('subject_speed')]],
        output='screen')])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            arm_share_parent + ':' + gimbal_share_parent
            + ':/usr/share/gazebo-11/models'),
        gui_arg, camera_arg, speed_arg, model_arg, vecnorm_arg,
        gzserver, gzclient, rsp, spawn, jsb_spawner, arm_spawner,
        camera_view, rl_node, mover,
    ])
