"""
@file real.launch.py
@brief bringup 的实物后端 —— 只管实物底座，上层栈复用共享工厂
@version 2.0
@date 2026-07-08

@note  本文件是整机启动的 real 后端（统一入口见 bringup.launch.py，backend:=real）。
       v2.0：臂 J1-3 从 arm_node（自研 CANopen）切换到 ros2_control HAL
       （canopen_ros2_control/RobotSystem，ros2_canopen），与云台 J4-6 合并为
       单个 controller_manager 管全 6 轴。
       v2.1（2026-07-13 控制路径融合）：J4-6 换 GimbalForwardingInterface（无 HID
       转发插件），robot_gimbal_node 为唯一 HID 拥有者、本 launch 常驻拉起。
       v3.0（2026-07-28 云台换 V2）：云台由 V1（二轴 eMeetCamera / USB HID）换为
       V2（C-200T 三轴 GCU 云台，robot_gimbal_V2）。**云台执行节点不在本工作空间运行**
       （串口唯一拥有者跑在云台板端），臂侧只经话题收发。
       v3.1（2026-07-31）：**去掉 gimbal_v2_bridge**。板端 robot_gimbal_node_v2 已原生收发
       臂侧转发约定（直接订阅 forward_cmd、直接发布 joint_states_raw），再经桥翻译一遍会
       双重驱动：命令送两遍，且桥把「转发流」升级成 GimbalCommand.POSITION —— 板端 POSITION
       会解冻 FROZEN，等于让 JTC 的保持流能解冻 FREEZE，破坏仲裁优先级。现在转发插件直连板端。
       （桥的源码保留在 robot_arm_driver 里未删，其 absolute_mode 相对⇄绝对姿态换算日后若要
        重启用，须改成不与板端原生话题重叠的接法，见 docs/云台控制路径融合方案.md 第 0 节。）
       v3.2（2026-08-21 关停失能开关）：新增 auto_disable_on_shutdown（**默认 false**）。
       true 时挂 OnProcessExit(ros2_control_node) + OnShutdown 两个钩子调 robot_arm_driver
       的 disable_motors（独立 SocketCAN 工具，减速停稳→断力矩→确认→NMT 兜底）。
       默认 false 保持既有行为：Ctrl-C 后臂停在原地、保持力矩、不下沉 —— 代价是驱动器
       仍带电且无人控制（RB200-CA 的 1016 消费者心跳默认禁用，不会自我保护，会一直
       持续到断电）。两边风险与选择依据详见下方钩子处的注释。

启动拓扑（单 arm_controller 管全 6 轴，与 Gazebo/MuJoCo 命令总线一致）：
  arm_controller（JTC，claim Joint1-6 position，跨两个硬件组件）
    ├─ Joint1-3  →  canopen_ros2_control/RobotSystem（ros2_canopen，CiA402/SocketCAN，position→IP）
    └─ Joint4-6  →  robot_gimbal_driver_v2/GimbalForwardingInterface（转发插件，无串口）
                      ├ 指令：变化检测 → /robot_gimbal_v2/forward_cmd ──┐
                      └ 状态：/robot_gimbal_v2/joint_states_raw 回填 ←──┤（跨机 DDS）
                              → jsb 统一发 6 轴 /joint_states           │
  云台板端 robot_gimbal_node_v2：**另行在云台板上启动** ←────────────────┘
                     （串口 /dev/ttyS0 唯一拥有者，原生收发上面两个话题），本 launch 不拉起。
                     前提：与本机同网段、同 ROS_DOMAIN_ID，且两侧都不能设 ROS_LOCALHOST_ONLY=1，
                     否则转发插件收不到回读，会刷 "No feedback from robot_gimbal_node yet"
                     并退化为指令回显（J4-6 的 /joint_states 是开环值，不是真实回读）。
  /arm_node/{enable,disable,recover} → arm_driver_services（转发 controller_manager）
  注：gimbal_controller 仅在 controllers_real.yaml 保留定义（云台单独调试用），本 launch 不 spawn。

用法（与 Gazebo launch 参数一致）：
  ros2 launch robot_arm_bringup real.launch.py controller:=joint_position             # 关节滑块
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_moveit           # MoveIt 笛卡尔直线
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_realtime_ik      # 滑块即时 IK
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_trajectory       # Ruckig 点到点+环绕
  ros2 launch robot_arm_bringup real.launch.py controller:=spherical_orbit            # 球面轨道运镜
  ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_velocity         # 笛卡尔速度（MoveIt Servo）
  ros2 launch robot_arm_bringup real.launch.py controller:=visp_ibvs                  # 红色方块 IBVS 闭环
  ros2 launch robot_arm_bringup real.launch.py controller:=commander                  # Arm Commander 中间层
  ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false

  arm_sim_mode:=true   臂 J1-3 不连 CAN（mock_components 回显），云台照常，用于无臂调试
  arm_sim_offset:=0.03 配合 arm_sim_mode：回显叠 0.03rad 常量静差，复现到位超时 / 验证超时兜底

前置条件（真机）：
  sudo ip link set can0 up type can bitrate 500000 && sudo ip link set can0 txqueuelen 128

视频流：云台 V2 的主相机（Cam0）由云台板端自行发布，本 launch 不再拉相机节点
（V1 的 robot_camera_node / camera_view 随云台换代一并下线）。

@copyright Copyright (c) 2026 eMeet
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 上层栈节点工厂（与 gazebo/mujoco 共用同一份定义，ament_python 正式模块）
from robot_arm_bringup import launch_common as common


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _default_can_interface() -> str:
    """★ CAN 口统一在 robot_arm_driver/config/can.yaml 改（test_arm.launch 也读它）★"""
    try:
        cfg = os.path.join(get_package_share_directory('robot_arm_driver'),
                           'config', 'can.yaml')
        with open(cfg) as f:
            return str(yaml.safe_load(f)['can_interface'])
    except Exception:
        return 'can0'


def _setup(context, *args, **kwargs):
    desc_share       = get_package_share_directory('robot_arm_description')
    bringup_share    = get_package_share_directory('robot_arm_bringup')
    moveit_cfg       = os.path.join(
        get_package_share_directory('robot_arm_moveit_config'), 'config')
    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers_real.yaml')
    # moveit_servo 命名空间必须显式包上：Humble 只从 moveit_servo. 前缀读参数，
    # 裸传会整体静默回退 Panda 默认值（错规划组/错输出话题）
    servo_params     = {'moveit_servo': _load_yaml(
        os.path.join(moveit_cfg, 'servo_config.yaml'))}
    srdf_content     = open(os.path.join(moveit_cfg, 'eMeetArm_models.srdf')).read()

    arm_sim_mode  = LaunchConfiguration('arm_sim_mode').perform(context)
    arm_sim_offset = LaunchConfiguration('arm_sim_offset').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)
    auto_disable  = LaunchConfiguration('auto_disable_on_shutdown').perform(context)

    # 全 6 轴 URDF：J1-3 RobotSystem（CANopen）+ J4-6 GimbalForwardingInterface（转发）
    _xacro_path = os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro')
    rd = xacro.process_file(
        _xacro_path,
        mappings={
            'backend': 'real',
            'arm_sim_mode': arm_sim_mode,
            'arm_sim_offset': arm_sim_offset,
            'can_interface': can_interface,
            'sim_mode': 'false',            # 云台连实物 HID
            'gazebo_camera': 'false',
            'emit_camera_control': 'true',
            'controllers_yaml': '',         # 实物无 gazebo_ros2_control 插件
        },
    ).toxml()
    rds = srdf_content

    ctrl      = LaunchConfiguration('controller')
    gui       = LaunchConfiguration('gui')
    log_level = LaunchConfiguration('log_level')
    base_log  = common.log_args(log_level)   # 第三方底座节点统一追加

    # ── Core nodes ────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=base_log,
        parameters=[{'robot_description': rd, 'use_sim_time': False}],
    )

    # 单 controller_manager 管全 6 轴（臂 CANopen + 云台 HID）
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        arguments=base_log,
        parameters=[{'robot_description': rd}, controllers_yaml],
    )

    # 使能/失能/故障恢复服务：/arm_node/{enable,disable,recover}
    # （commander 及旧接口兼容，转发到 controller_manager 硬件组件状态）
    arm_driver_services = Node(
        package='robot_arm_driver', executable='arm_driver_services', output='screen',
    )

    # 云台 V2 无需臂侧适配节点：板端 robot_gimbal_node_v2 原生订阅 /robot_gimbal_v2/forward_cmd、
    # 原生发布 /robot_gimbal_v2/joint_states_raw，转发插件直接与它对接（见文件头 v3.1）。
    # 原 gimbal_v2_bridge 已停止启用（双重驱动 + 破坏 FREEZE 仲裁），源码保留未删。

    # ── Controller spawners ───────────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )
    arm_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )
    # 云台控制器：默认（velocity_backend=trajectory）用不到 —— 速度也走 arm_controller
    # 那条 joint_trajectory，J4-6 跟着同一条轨迹走。只有切到 PV 后端
    #（velocity_backend:=velocity_controller，arm_controller 被停）时，J4-6 才需要它
    # 顶上命令通道，由 mode_manager_node 的 hold_controllers 激活。故只 load 不 activate。
    gimbal_ctrl_loader = Node(
        package='controller_manager', executable='spawner',
        arguments=['gimbal_controller', '--inactive',
                   '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )

    # JOINT_VELOCITY 模式备用：只 load+configure 不 activate（--inactive），
    # 由 mode_manager_node 经 /robot_arm/switch_control_mode 与 arm_controller 互斥切换
    # （旧 /arm_node/set_mode_pv 仍可用，但推荐走 mode_manager 的语义模式接口）
    vel_ctrl_loader = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_velocity_controller', '--inactive',
                   '--controller-manager', '/controller_manager'] + base_log,
        output='screen',
    )

    # 控制模式仲裁器（常驻基础设施，实物无 /clock → use_sim_time=False）
    # hold_controllers 只在 PV 后端生效（默认 trajectory 后端不切控制器，也就不需要保持控制器）
    mode_manager = common.mode_manager_node(real=True, use_sim_time=False,
                                            hold_controllers=['gimbal_controller'])

    # ── GUI controller nodes（复用共享工厂）─────────────────────────────────────
    # use_sim_time=False：实物无 /clock，若为 True 节点内定时器永不触发（TF 位姿面板卡 '--'）
    joint_position_ctrl    = common.gui_node(ctrl, 'joint_position',        'joint_position_gui',        use_sim_time=False)
    cartesian_moveit_ctrl  = common.gui_node(ctrl, 'cartesian_moveit',      'cartesian_moveit_gui',      use_sim_time=False)
    realtime_ik_ctrl       = common.gui_node(ctrl, 'cartesian_realtime_ik', 'cartesian_realtime_ik_gui', use_sim_time=False)
    trajectory_ctrl        = common.gui_node(ctrl, 'cartesian_trajectory',  'cartesian_trajectory_gui',  use_sim_time=False)
    spherical_orbit_ctrl   = common.gui_node(ctrl, 'spherical_orbit',       'spherical_orbit_gui',       use_sim_time=False)
    velocity_ctrl          = common.gui_node(ctrl, 'cartesian_velocity',    'cartesian_velocity_gui',    use_sim_time=False)

    # ── MoveIt（需要 IK 的控制方式；模式分组唯一来源 = common.MOVEIT_MODES）──────
    kinematics_yaml   = _load_yaml(os.path.join(moveit_cfg, 'kinematics.yaml'))
    joint_limits_yaml = _load_yaml(os.path.join(moveit_cfg, 'joint_limits.yaml'))
    # 实物模式直接创建 move_group，使用 FollowJointTrajectory action 配置
    # （不 include moveit.launch.py，避免 Ros2ControlManager 找不到 Joint1-3 控制器）
    move_group_node = common.move_group_node(
        ctrl,
        robot_description=rd, srdf=rds,
        kinematics=kinematics_yaml, joint_limits=joint_limits_yaml,
        planning_pipeline=_load_yaml(os.path.join(moveit_cfg, 'planning_pipeline.yaml')),
        moveit_controllers=_load_yaml(os.path.join(moveit_cfg, 'moveit_controllers_real.yaml')),
        use_sim_time=False, log_level=log_level,
    )

    # ── MoveIt Servo（cartesian_velocity 模式）─────────────────────────────────
    is_velocity  = common.is_mode(ctrl, 'cartesian_velocity')
    is_visp_ibvs = common.is_mode(ctrl, 'visp_ibvs')
    is_commander = common.is_mode(ctrl, 'commander')
    needs_servo  = common.in_modes(ctrl, common.SERVO_MODES)

    # 安全姿态预移动（全零关节是运动学奇异点，Servo 启动前须先移走）
    move_to_safe_pose = common.safe_pose_action(needs_servo)

    # Servo 模式：t=3s 安全姿态预移动（3s 运动），t=7s 启动 servo_node + 控制器
    cartesian_velocity_start = TimerAction(
        period=7.0,
        actions=[
            common.servo_node(
                is_velocity,
                robot_description=rd, srdf=rds,
                kinematics=kinematics_yaml, joint_limits=joint_limits_yaml,
                servo_params=servo_params,
                use_sim_time=False, use_gazebo=False, log_level=log_level,
            ),
            velocity_ctrl,
        ],
        condition=is_velocity,
    )

    # ── visp_ibvs 模式 ───────────────────────────────────────────────────────
    # IBVS 节点发布 /arm_controller/joint_trajectory 单点流（positions+velocities），
    # JTC（position→IP 模式）直接跟踪，无需旧 arm_node 的 PV 模式切换。
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
    # t=7s：等预备位姿到达后启动 IBVS 节点
    visp_ibvs_start = TimerAction(
        period=7.0,
        actions=common.visp_nodes(ctrl, 'visp_ibvs',
                                  robot_description=rd, use_sim_time=False,
                                  perception_topic='/ros2_algo_vision/report',
                                  control_depth=False),
        condition=is_visp_ibvs,
    )

    # ── commander 模式 ────────────────────────────────────────────────────────
    # t=10s：等 move_group planning scene 完全就绪后再启动
    commander_start = TimerAction(
        period=10.0,
        actions=common.commander_nodes(ctrl, gui, use_sim_time=False, real=True),
        condition=is_commander,
    )

    # 控制器 2 s 后 spawn（等 controller_manager / CANopen master 初始化）
    spawn_controllers = TimerAction(
        period=2.0,
        actions=[jsb_spawner, arm_ctrl_spawner, vel_ctrl_loader, gimbal_ctrl_loader],
    )

    # ── 关停自动失能：**默认关闭**（auto_disable_on_shutdown，2026-08-21）─────────
    # 这是一个取舍，两边都有真实风险，默认选了「停在原地」：
    #
    #   关闭（默认）= Ctrl-C 后臂**停在原地、保持力矩、不下沉**，但驱动器停在
    #     Operation Enabled 带着力矩而**没人在控制它**了（进程已死）。三个后果：
    #       ① 你以为关了去手动搬臂时，它会顶回来（伤手 / 顶坏谐波减速器）
    #       ② 臂若刚好压住东西，推不开且越推越用力
    #       ③ CANopen master 没干净关闭 → 下次 launch 报 SDO timed out / can0 NO-CARRIER
    #          （CLAUDE.md 高频坑第 1 条）。这条**与下沉无关**，手动跑一次
    #          `ros2 run robot_arm_driver disable_motors can0` 就是它的解药。
    #     ⚠️ 且 RB200-CA 不会自我保护：EDS 的 1016:01（消费者心跳，即驱动器监测**主站**
    #        心跳的超时）默认 0 = 禁用，bus.yml 也没配 —— 驱动器不知道上位机死了，
    #        这个带力矩状态会**一直持续到断电**。想加第二道防线就配 1016（未验证）。
    #
    #   开启 = Ctrl-C 后先 Quick Stop 按 6085 斜坡减速停稳，再 Shutdown 断力矩并读状态字
    #     确认，失败则 NMT Reset Node 兜底。代价：断力矩那一刻 J2/J3 失去支撑**下沉**
    #     （60FE 抱闸输出在 RB200-CA 上只读、驱动器自管，上位机控制不了）。
    #     幅度未实机实测 —— 仿真是「J2 从 +0.500 砸到下限」，但那是 Gazebo 关节无摩擦的
    #     最坏情况，真机谐波减速器摩擦大得多，可能只是慢慢垂下来。
    #
    # 要开启：ros2 launch robot_arm_bringup real.launch.py auto_disable_on_shutdown:=true
    # 开启后要免下沉，得先把臂移到机械自稳的收纳位再关停（posture.stowed_joints 尚未
    # 在实机标定，见 arm_params_real.yaml 候选④），那是上层逻辑，不是这两个钩子的事。
    #
    # 钩子机制（开启时）：ros2_control_node 关停时以 SIGABRT 挂掉（vendored 0.2.13
    # DeviceContainer 析构 bug），这条路径上任何进程内钩子都拿不到执行机会（生命周期
    # on_shutdown/on_cleanup 与 rclcpp::on_shutdown 均实测无效），所以只能进程外收拾。
    #   主路径 OnProcessExit：ros2_control_node 一死就跑 —— 此刻总线空闲，没有主站
    #                         200Hz 的 RPDO 抢控制字，SDO 通路最干净。
    #   兜底 OnShutdown    ：覆盖 ros2_control_node 从未起来、或 launch 整体被 SIGTERM
    #                         的情况；此时 master 可能还在写 0x001F，SDO 抢不过，
    #                         工具会自动升级到 NMT 层（NMT 不是 402 控制字，抢得过）。
    # 两条都触发是幂等的（第二次读状态字发现已无力矩就直接放过）。工具自己屏蔽
    # SIGINT/SIGTERM/SIGHUP/SIGPIPE，否则会在写控制字之前被关停信号打死。
    # arm_sim_mode 下臂不连 CAN（mock 回显），没有从站可失能，故一并跳过。
    disable_motors_cmd = [
        os.path.join(get_package_prefix('robot_arm_driver'),
                     'lib', 'robot_arm_driver', 'disable_motors'),
        can_interface, '1', '2', '3',   # bus.yml 的三个关节模组
    ]
    shutdown_disable = []
    if (auto_disable.lower() in ('true', '1')
            and arm_sim_mode.lower() not in ('true', '1')):
        shutdown_disable = [
            RegisterEventHandler(OnProcessExit(
                target_action=ros2_control_node,
                on_exit=[ExecuteProcess(cmd=disable_motors_cmd, output='screen')],
            )),
            RegisterEventHandler(OnShutdown(
                on_shutdown=[ExecuteProcess(cmd=disable_motors_cmd, output='screen')],
            )),
        ]

    # ── 电机运行模式（402）按控制方式自动选择，经 /arm_node/set_mode_* 编排 ──────
    #   joint_position                → PP(1) 驱动器自规划（滑块点到点，6081 限速）
    #   轨迹/运镜/commander 等其余     → IP(7) 跟随上位机插补（bus.yml 默认，无需调用）
    #   cartesian_velocity/visp_ibvs  → PV(3) 速度伺服（待上层改发速度指令后接入，
    #                                   现阶段仍走 IP 位置流）
    set_mode_pp = TimerAction(
        period=6.0,   # 等 arm_controller 激活完成（t=2s spawn + 使能耗时）
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/arm_node/set_mode_pp',
                 'std_srvs/srv/Trigger', '{}'],
            output='screen',
        )],
        condition=IfCondition(PythonExpression(["'", ctrl, "' == 'joint_position'"])),
    )

    # t=3s：启动 GUI 控制器 + 安全姿态预移动（Servo 模式）
    spawn_gui = TimerAction(
        period=3.0,
        actions=[
            joint_position_ctrl, cartesian_moveit_ctrl, realtime_ik_ctrl,
            trajectory_ctrl, spherical_orbit_ctrl,
            move_to_safe_pose,   # Servo 模式专用（condition=needs_servo）
        ],
    )

    return [
        *shutdown_disable,         # 关停失能钩子（须早于 ros2_control_node 注册）
        robot_state_publisher,
        ros2_control_node,
        arm_driver_services,
        mode_manager,              # 控制模式仲裁器（常驻，/robot_arm/switch_control_mode）
        move_group_node,           # cartesian_moveit / realtime_ik / trajectory / orbit / commander
        spawn_controllers,
        set_mode_pp,               # t=6s，仅 joint_position 模式（402 切 PP）
        spawn_gui,
        cartesian_velocity_start,  # t=7s，仅 cartesian_velocity 模式
        visp_ibvs_prep,            # t=3s，visp_ibvs 预备位姿
        visp_ibvs_start,           # t=7s，visp_ibvs 启动 IBVS 节点
        commander_start,           # t=10s，仅 commander 模式
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='joint_position',
            description='控制方式: joint_position | cartesian_moveit | cartesian_realtime_ik | '
                        'cartesian_trajectory | spherical_orbit | cartesian_velocity | '
                        'visp_ibvs | commander',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否启动 commander_test_gui（仅 controller:=commander 时生效）: true | false',
        ),
        DeclareLaunchArgument(
            'arm_sim_mode', default_value='false',
            description='true=臂 J1-3 不连 CAN（mock 回显），云台照常；false=连接实物 CANopen',
        ),
        DeclareLaunchArgument(
            'arm_sim_offset', default_value='0.0',
            description='仅 arm_sim_mode:=true 生效：mock 回显位置 = 命令 + 此偏移（rad），'
                        '模拟伺服静差，用于复现到位超时 / 验证超时兜底；0 = 精确回显',
        ),
        DeclareLaunchArgument(
            'can_interface', default_value=_default_can_interface(),
            description='SocketCAN 接口名，默认读 robot_arm_driver/config/can.yaml；'
                        '可临时覆盖：can0（真机）| vcan0（假从站联调）',
        ),
        DeclareLaunchArgument(
            'auto_disable_on_shutdown', default_value='false',
            description='关停时是否自动失能电机。false（默认）= Ctrl-C 后臂停在原地、'
                        '保持力矩、不下沉，但驱动器仍带电且无人控制（且下次 launch 可能'
                        '报 SDO timed out）；true = 减速停稳后断力矩，代价是 J2/J3 会下沉。'
                        '手动失能随时可用：ros2 run robot_arm_driver disable_motors can0',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='第三方底座节点（move_group/servo/spawner/rsp/ros2_control 等）'
                        '日志级别，默认 warn 降噪，调试时设 info；自家业务节点不受影响',
        ),
        OpaqueFunction(function=_setup),
    ])
