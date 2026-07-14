"""
@file   bringup.launch.py
@brief  整机启动唯一入口：选后端 + 选控制方式，一条命令拉起整套栈
@version 1.1
@date   2026-07-01

用法：
  ros2 launch robot_arm_bringup bringup.launch.py backend:=gazebo controller:=commander
  ros2 launch robot_arm_bringup bringup.launch.py backend:=real   controller:=commander gui:=false
  ros2 launch robot_arm_bringup bringup.launch.py backend:=mujoco controller:=cartesian_trajectory

参数：
  backend      gazebo（默认，sim 主路）| mujoco | real（实物）
  controller   控制方式：joint_position | cartesian_moveit | cartesian_realtime_ik |
               cartesian_trajectory | spherical_orbit | cartesian_velocity |
               ibvs_control | visp_ibvs(_control) | commander
  gui          true/false，是否起 GUI（backend=gazebo/real 生效）
  world        Gazebo 世界名，不含 .world（仅 backend=gazebo 生效）
  log_level    第三方底座节点（rviz2/move_group/servo/spawner/rsp 等）日志级别，
               默认 warn 降噪；调试时 log_level:=info 恢复。自家业务节点恒为 info。
               完整日志始终落 ~/.ros/log/<run>/launch.log。

分层结构（PR-4，2026-07-10 起仿真后端拆为独立包）：
  bringup.launch.py              ← 本文件：按 backend 分发、透传参数，别的什么都不做
    ├─ robot_arm_gazebo/launch/gazebo.launch.py   ← Gazebo 底座（含 worlds/models 资产）
    ├─ robot_arm_mujoco/launch/mujoco.launch.py   ← MuJoCo 底座（含 mujoco_node 仿真桥）
    └─ 本包 launch/real.launch.py                 ← 实物底座
         └─ robot_arm_bringup.launch_common（本包 Python 模块）← 三后端共用的「上层栈」节点定义：
                                                     controller GUI / move_group / servo /
                                                     commander / ibvs / visp / 安全预移动
  上层只写一份、由三后端复用，从源头消除「三处各抄一遍 → 必然漂移」。

参数按各后端声明的项透传（未声明的不透传，避免 launch 报未知参数）：
  backend=gazebo → controller, gui, world, log_level
  backend=mujoco → controller, log_level
  backend=real   → controller, gui, log_level

@copyright Copyright (c) 2026 eMeet
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend', default_value='gazebo',
        description='后端: gazebo（默认，sim 主路）| mujoco | real（实物）')
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='joint_position',
        description='控制方式（透传后端）: joint_position | cartesian_moveit | '
                    'cartesian_realtime_ik | cartesian_trajectory | spherical_orbit | '
                    'cartesian_velocity | ibvs_control | visp_ibvs(_control) | commander')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='是否启动 GUI（backend=gazebo/real 生效）: true | false')
    world_arg = DeclareLaunchArgument(
        'world', default_value='emeet_arm',
        description='Gazebo 世界名，不含 .world（仅 backend=gazebo 生效）: emeet_arm | ibvs_tracking_test')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='第三方底座节点日志级别，默认 warn 降噪，调试时设 info（透传后端）')

    backend    = LaunchConfiguration('backend')
    controller = LaunchConfiguration('controller')
    gui        = LaunchConfiguration('gui')
    world      = LaunchConfiguration('world')
    log_level  = LaunchConfiguration('log_level')

    def _is_backend(name):
        return IfCondition(PythonExpression(["'", backend, "' == '", name, "'"]))

    def _include(package, filename, launch_arguments, name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', filename)),
            launch_arguments=launch_arguments,
            condition=_is_backend(name))

    gazebo = _include('robot_arm_gazebo', 'gazebo.launch.py',
                      {'controller': controller, 'gui': gui, 'world': world,
                       'log_level': log_level}.items(),
                      'gazebo')
    mujoco = _include('robot_arm_mujoco', 'mujoco.launch.py',
                      {'controller': controller, 'log_level': log_level}.items(),
                      'mujoco')
    real   = _include('robot_arm_bringup', 'real.launch.py',
                      {'controller': controller, 'gui': gui,
                       'log_level': log_level}.items(),
                      'real')

    return LaunchDescription([
        backend_arg, controller_arg, gui_arg, world_arg, log_level_arg,
        gazebo, mujoco, real,
    ])
