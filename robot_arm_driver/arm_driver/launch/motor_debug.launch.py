#!/usr/bin/env python3
"""单电机调试平台入口 —— 一个 launch 拉起一个电机的「C++ 总线后端 + PyQt5 前端」。

用法:
  ros2 launch robot_arm_driver motor_debug.launch.py node_id:=1
  ros2 launch robot_arm_driver motor_debug.launch.py node_id:=2 can_interface:=vcan0

⚠️ 后端（motor_debug_backend）直接占用总线控制字（SocketCAN + SDO），运行前必须
   停掉 ros2_control 栈（test_arm.launch / real.launch）。
任一进程退出（关 GUI 窗口 / 后端连不上 CAN）整个 launch 一起退，后端退出路径
自带失能兜底（清零 PV/PT 目标 + Shutdown）。
CAN 口默认值与 test_arm/real 一样统一读 config/can.yaml（★ 换口只改那一个文件 ★）。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_can_interface() -> str:
    """★ CAN 口统一在 robot_arm_driver/config/can.yaml 改（与 test_arm/real 同源）★"""
    try:
        path = os.path.join(get_package_share_directory('robot_arm_driver'),
                            'config', 'can.yaml')
        with open(path, encoding='utf-8') as f:
            return str(yaml.safe_load(f)['can_interface'])
    except Exception:
        return 'can0'


def _setup(context):
    can_interface = LaunchConfiguration('can_interface').perform(context)
    node_id = int(LaunchConfiguration('node_id').perform(context))
    master_id = int(LaunchConfiguration('master_id').perform(context))
    sdo_timeout_ms = int(LaunchConfiguration('sdo_timeout_ms').perform(context))

    # 一机一实例：后端节点名带 node_id，多开互不串（GUI 经 backend 参数对上）
    backend_name = f'motor_debug_backend_{node_id}'

    return [
        Node(
            package='robot_arm_driver',
            executable='motor_debug_backend',
            name=backend_name,
            parameters=[{
                'can_interface': can_interface,
                'node_id': node_id,
                'master_id': master_id,
                'sdo_timeout_ms': sdo_timeout_ms,
            }],
            output='screen',
            on_exit=Shutdown(),   # 连不上 CAN 直接整体退出，别留一个空 GUI
        ),
        Node(
            package='robot_arm_driver',
            executable='motor_debug_gui.py',
            name=f'motor_debug_gui_{node_id}',
            parameters=[{'backend': backend_name}],
            output='screen',
            on_exit=Shutdown(),   # 关窗即整个 launch 退出（后端退出路径自带失能）
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_id', default_value='1',
                              description='CANopen 节点号（J1=1 J2=2 J3=3）'),
        DeclareLaunchArgument('can_interface', default_value=default_can_interface(),
                              description='CAN 口（默认读 config/can.yaml）'),
        DeclareLaunchArgument('master_id', default_value='127',
                              description='主站节点号（心跳用）'),
        DeclareLaunchArgument('sdo_timeout_ms', default_value='500',
                              description='SDO 应答超时 ms'),
        OpaqueFunction(function=_setup),
    ])
