"""
@file   teach.launch.py
@brief  示教节点单独启动 —— 只拉 arm_teach_node，不碰底座
@version 1.0
@date   2026-08-24

★ 本 launch **刻意只启动示教节点一个**。
  示教依赖的东西（ros2_control / arm_controller / mode_manager_node / arm_commander /
  robot_state_publisher）全部由既有的 bringup 提供，在这里重复拉起只会双重驱动
  —— 那正是 2026-07-31 去掉 gimbal_v2_bridge 的原因（见 real.launch.py 的 v3.1 说明）。
  所以正确用法是**两条命令**：

    终端①  ros2 launch robot_arm_bringup bringup.launch.py backend:=gazebo controller:=commander
    终端②  ros2 launch robot_arm_teach   teach.launch.py

  实机把 backend 换成 real 即可。⚠️ 注意 `start_system.sh` 已经把机械臂包含进去了，
  别再叠一套 bringup（robot_arm/CLAUDE.md 的「调试时先确认只有一套栈在跑」）。

参数：
  params_file   参数 YAML 路径，默认本包 config/teach_params.yaml
  storage_dir   轨迹存储目录，覆盖 YAML 里的 teach.storage_directory；
                留空则用节点默认值 $HOME/.ros/robot_arm_teach
  allow_drag    是否允许手拖示教（仅仿真，默认 false）。实机 effort_controller 不存在，
                开了也会被 ModeManager 拒绝，见 doc/实机能力限制说明.md
  gui           是否同时起示教面板 teach_gui（PyQt5），默认 false。
                面板是纯客户端：只调本包的 13 个服务 + 发 JogCommand，不碰任何总线，
                所以关掉它对示教功能没有任何影响，命令行照样能用。
                无显示环境（DISPLAY 未设/ssh 无 X 转发）时别开 —— Qt 起不来会直接退出。
  log_level     本节点日志级别，默认 info

⚠️ 不要在这里传 use_sim_time。本节点所有间隔/超时判断都走 steady_clock，不读 /clock；
   而 `use_sim_time:=true` 会让**所有 ROS 定时器在实物上永不触发**
   （robot_arm/CLAUDE.md 记过这个坑，症状是订阅回调都正常、只有 timer 驱动的逻辑静默死掉）。

@copyright Copyright (c) 2026 eMeet
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_teach')
    default_params = os.path.join(pkg_share, 'config', 'teach_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='参数 YAML 路径（顶层键必须是节点名 arm_teach）')
    storage_dir_arg = DeclareLaunchArgument(
        'storage_dir', default_value='',
        description='轨迹存储目录；留空则用 $HOME/.ros/robot_arm_teach')
    allow_drag_arg = DeclareLaunchArgument(
        'allow_drag', default_value='false',
        description='是否允许手拖示教（仅仿真）: true | false')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='是否同时起示教面板 teach_gui（PyQt5）: true | false')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='本节点日志级别: debug | info | warn | error')

    params_file = LaunchConfiguration('params_file')
    storage_dir = LaunchConfiguration('storage_dir')
    allow_drag  = LaunchConfiguration('allow_drag')
    gui         = LaunchConfiguration('gui')
    log_level   = LaunchConfiguration('log_level')

    teach_node = Node(
        package='robot_arm_teach',
        executable='arm_teach_node',
        # 节点名必须与 YAML 顶层键一致，改这里就要同时改 config/teach_params.yaml
        name='arm_teach',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        # 命令行覆盖放在 YAML 之后 —— 后者赢，这样 storage_dir / allow_drag 才能生效。
        # ★ allow_drag 必须用 ParameterValue 显式声明 value_type=bool：
        #   LaunchConfiguration 求值出来是字符串 "false"，直接塞给一个 bool 参数
        #   会在节点启动时报参数类型不匹配（而且报得很不明显）。
        # ★ storage_directory 传空串是允许的 —— 节点侧把空串当作"用默认目录"
        #   （见 declare_params()），所以不需要在 launch 里做条件分支。
        parameters=[
            params_file,
            {
                'teach.storage_directory': storage_dir,
                'teach.allow_drag': ParameterValue(allow_drag, value_type=bool),
            },
        ],
    )

    # 示教面板。GUI 与节点之间只有服务/话题，没有任何进程内耦合 ——
    # 面板崩了机械臂不受影响（点动会被 0.3s 断流看门狗停住），节点没起面板也能开着等。
    # 刻意不设 respawn：Qt 起不来（无 DISPLAY）时反复重启只会刷屏。
    teach_gui = Node(
        package='robot_arm_teach',
        executable='teach_gui',
        name='arm_teach_gui',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        params_file_arg,
        storage_dir_arg,
        allow_drag_arg,
        gui_arg,
        log_level_arg,
        teach_node,
        teach_gui,
    ])
