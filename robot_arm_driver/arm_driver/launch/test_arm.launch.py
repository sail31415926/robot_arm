"""
@file test_arm.launch.py
@brief robot_arm_driver 自测栈：一条命令验证 ros2_canopen HAL 是否跑通

自包含（内置最小 URDF + 自带 test_controllers.yaml），不依赖 robot_arm_description /
robot_arm_bringup，只测「驱动这一层」：
  ros2_control_node(RobotSystem/CANopen) + jsb + arm_controller(JTC) + arm_driver_services

═══ 用法（按验证层次从低到高）═══════════════════════════════════════════════

  # ① mock 干跑（不碰 CAN，验证配置/加载）
  ros2 launch robot_arm_driver test_arm.launch.py mock:=true

  # ② vcan0 假从站全链路（NMT/SDO/PDO/402 状态机全真，从站仿真）
  #    先建虚拟 CAN（一次性）：
  #      sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
  #    一条命令：假从站×3 + 主站栈 + 10s 后自动发测试轨迹：
  ros2 launch robot_arm_driver test_arm.launch.py can_interface:=vcan0 fake_slaves:=true demo:=true

  # ③ 真机（RB200-CA ×3，node 1/2/3，波特率 500k）
  #      sudo ip link set can0 up type can bitrate 500000 && sudo ip link set can0 txqueuelen 128
  ros2 launch robot_arm_driver test_arm.launch.py            # 默认 can0
  ros2 launch robot_arm_driver test_arm.launch.py demo:=true # 敢让它动了再加 demo

═══ 通过标准 ═══════════════════════════════════════════════════════════════
  1. 日志出现 "Initialisation successful"（CANopen 主站+从站配置完成）
  2. jsb / arm_controller 均 "Configured and activated"
  3. ros2 topic echo /joint_states --once 有 Joint1-3
  4. demo:=true 时关节 3s 平滑走到 [0.1, 0, 0] 再回零
  急停/使能： ros2 service call /arm_node/disable std_srvs/srv/Trigger（enable/recover 同）

@date 2026-07-08
@copyright Copyright (c) 2026 EMEET
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_can_interface() -> str:
    """★ CAN 口统一在 robot_arm_driver/config/can.yaml 改（real.launch 也读它）★"""
    try:
        cfg = os.path.join(get_package_share_directory('robot_arm_driver'),
                           'config', 'can.yaml')
        with open(cfg) as f:
            return str(yaml.safe_load(f)['can_interface'])
    except Exception:
        return 'can0'

# 关节限位与 robot_arm_description/urdf/arm.urdf.xacro 保持一致
JOINT_LIMITS = {
    'Joint1': (-2.618, 2.618),
    'Joint2': (-0.981, 2.959),
    'Joint3': (-2.5, 0.02),
}


def _test_urdf(mock: bool, can_interface: str, share: str) -> str:
    """最小 robot_description：只含 ros2_control 块（驱动层测试不需要连杆/网格）。"""
    if mock:
        hardware = '<plugin>mock_components/GenericSystem</plugin>'
    else:
        hardware = f"""<plugin>robot_arm_driver/UnwrapRobotSystem</plugin>
      <param name="bus_config">{share}/config/canopen/bus.yml</param>
      <param name="master_config">{share}/config/canopen/master.dcf</param>
      <param name="can_interface_name">{can_interface}</param>"""

    joints = '\n'.join(f"""    <joint name="{name}">
      <param name="device_name">joint_{i}</param>
      <command_interface name="position">
        <param name="min">{lo}</param><param name="max">{hi}</param>
      </command_interface>
      <command_interface name="velocity"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>""" for i, (name, (lo, hi)) in enumerate(JOINT_LIMITS.items(), start=1))

    return f"""<?xml version="1.0"?>
<robot name="eMeetArmDriverTest">
  <ros2_control name="eMeetArm_hardware" type="system">
    <hardware>
      {hardware}
    </hardware>
{joints}
  </ros2_control>
</robot>"""


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('robot_arm_driver')

    can_interface = LaunchConfiguration('can_interface').perform(context)
    mock          = LaunchConfiguration('mock').perform(context).lower() == 'true'
    fake_slaves   = LaunchConfiguration('fake_slaves')
    demo          = LaunchConfiguration('demo')
    with_slaves   = LaunchConfiguration('fake_slaves').perform(context).lower() == 'true'

    robot_description = _test_urdf(mock, can_interface, share)
    controllers_yaml  = os.path.join(share, 'config', 'test_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
    )
    # 假从站模式：从站 lifecycle 起来要 1~2s，主站必须等它们在总线上就位，
    # 否则 CANopen boot 三次超时直接放弃（实测竞速踩过）
    cm_start = TimerAction(period=3.0, actions=[controller_manager]) \
        if with_slaves else controller_manager

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    # 顺序化：先 jsb，再 arm_controller（避免并发 spawner 竞争）
    arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_ctrl_spawner]))

    # /arm_node/{enable,disable,recover}
    arm_driver_services = Node(
        package='robot_arm_driver', executable='arm_driver_services', output='screen',
    )

    # 假从站 ×3（fake_slaves:=true 时，与主站同一份 RB200-CA.eds、同一 CAN 口）
    slave_launch = os.path.join(
        get_package_share_directory('canopen_fake_slaves'), 'launch', 'cia402_slave.launch.py')
    slave_eds = os.path.join(share, 'config', 'canopen', 'RB200-CA.eds')
    slaves = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slave_launch),
            launch_arguments={
                'node_id': str(node_id),
                'node_name': f'fake_joint_{node_id}',
                'slave_config': slave_eds,
                'can_interface_name': can_interface,
            }.items(),
            condition=IfCondition(fake_slaves),
        )
        for node_id in (1, 2, 3)
    ]

    # demo:=true：t=10s 发测试轨迹（0.1rad/3s），t=16s 回零
    def _traj(positions, sec):
        return ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/arm_controller/joint_trajectory',
                 'trajectory_msgs/msg/JointTrajectory',
                 ('{joint_names: [Joint1,Joint2,Joint3], '
                  f'points: [{{positions: {positions}, '
                  f'time_from_start: {{sec: {sec}}}}}]}}')],
            output='screen', condition=IfCondition(demo),
        )
    demo_move = TimerAction(period=10.0, actions=[_traj('[0.1, 0.0, 0.0]', 3)],
                            condition=IfCondition(demo))
    demo_home = TimerAction(period=16.0, actions=[_traj('[0.0, 0.0, 0.0]', 3)],
                            condition=IfCondition(demo))

    return slaves + [cm_start, jsb_spawner, arm_after_jsb,
                     arm_driver_services, demo_move, demo_home]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface', default_value=default_can_interface(),
            description='SocketCAN 接口名，默认读 robot_arm_driver/config/can.yaml；'
                        '可临时覆盖：can0（真机）| vcan0（假从站联调）',
        ),
        DeclareLaunchArgument(
            'mock', default_value='false',
            description='true=不连 CAN，mock_components 回显命令（干跑）',
        ),
        DeclareLaunchArgument(
            'fake_slaves', default_value='false',
            description='true=同时启动 3 个 CiA402 假从站（配合 vcan0 用）',
        ),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true=t=10s 自动发测试轨迹（J1→0.1rad→回零）；真机确认安全后再开',
        ),
        OpaqueFunction(function=_setup),
    ])
