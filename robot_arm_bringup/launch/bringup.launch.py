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

分层结构（PR-4）：
  bringup.launch.py              ← 本文件：按 backend 分发、透传参数，别的什么都不做
    └─ {gazebo,mujoco,real}.launch.py   ← 各后端只管「底座」：仿真器/实物驱动 + 起该后端的控制器
         └─ _arm_launch_common.py       ← 三后端共用的「上层栈」节点定义：
                                            controller GUI / move_group / servo / commander /
                                            ibvs / visp / 安全预移动
  上层只写一份、由三后端复用，从源头消除「三处各抄一遍 → 必然漂移」。

参数按各后端声明的项透传（未声明的不透传，避免 launch 报未知参数）：
  backend=gazebo → controller, gui, world      backend=mujoco → controller      backend=real → controller, gui

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
    launch_dir = os.path.join(
        get_package_share_directory('robot_arm_bringup'), 'launch')

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

    backend    = LaunchConfiguration('backend')
    controller = LaunchConfiguration('controller')
    gui        = LaunchConfiguration('gui')
    world      = LaunchConfiguration('world')

    def _is_backend(name):
        return IfCondition(PythonExpression(["'", backend, "' == '", name, "'"]))

    def _include(filename, launch_arguments, name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, filename)),
            launch_arguments=launch_arguments,
            condition=_is_backend(name))

    gazebo = _include('gazebo.launch.py',
                      {'controller': controller, 'gui': gui, 'world': world}.items(),
                      'gazebo')
    mujoco = _include('mujoco.launch.py',
                      {'controller': controller}.items(),
                      'mujoco')
    real   = _include('real.launch.py',
                      {'controller': controller, 'gui': gui}.items(),
                      'real')

    return LaunchDescription([
        backend_arg, controller_arg, gui_arg, world_arg,
        gazebo, mujoco, real,
    ])
