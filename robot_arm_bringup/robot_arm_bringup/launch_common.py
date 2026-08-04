"""
@file   launch_common.py（robot_arm_bringup.launch_common）
@brief  上层栈节点工厂 —— controller GUI / move_group / servo / 安全预移动 / commander / ibvs / visp 的单一来源
@version 1.1
@date   2026-07-14

PR-4 launch 重构·第二步：把三后端各抄一遍的「上层栈」节点定义收成一份。

设计取舍：不做「一个 common.launch.py 用 IncludeLaunchDescription 统一时序」——因为三后端
的启动时序本质不同（gazebo 用 OnProcessExit 事件链、mujoco/real 用 TimerAction），且
robot_description / use_sim_time / moveit_controllers 各异。改用**工厂函数模块**：每个
backend 保留自己的时序编排，只调用这里的工厂拿「同一份节点定义」，从而消除复制粘贴漂移，
风险最低（不重写已调好的时序）。

注意：本模块经 ament_python_install_package 作为正式 Python 包安装
（v1.1 前是 launch/_arm_launch_common.py + sys.path hack）。backend launch 用
`from robot_arm_bringup import launch_common as common` 导入。

约定：
  ctrl / gui 传 LaunchConfiguration；use_sim_time 传 bool（各 backend 自己的固定值）；
  robot_description / srdf 传字符串；kinematics/joint_limits/planning_pipeline/moveit_controllers/
  servo_params 传已 load 的 dict。

日志规范（三层）：
  - 自家业务节点（GUI / controllers / commander / mujoco_node 等）：info + screen，不动；
  - 第三方底座节点（move_group / servo / rviz2 / spawner / robot_state_publisher 等）：
    默认 warn 降噪，向工厂传 log_level（通常为 LaunchConfiguration('log_level')）；
  - 调试时 launch 加 log_level:=info 一键恢复。完整日志始终落 ~/.ros/log/。

@copyright Copyright (c) 2026 eMeet
"""

from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ── 模式分组（单一来源，供 backend 复用）─────────────────────────────────────────
# 需要 IK / move_group 的模式（含 commander；visp 两模式靠 move_group 的
# /check_state_validity 提供 IBVS 碰撞守护，缺了则守护 fail-open 无保护）
MOVEIT_MODES = ['cartesian_moveit', 'cartesian_realtime_ik',
                'cartesian_trajectory', 'spherical_orbit', 'commander',
                'visp_ibvs', 'visp_ibvs_control']
# 需要 MoveIt Servo + 安全预移动的模式
SERVO_MODES  = ['cartesian_velocity']

# 安全姿态预移动指令（全零关节是运动学奇异点，Servo 启动前须先移走）
SAFE_POSE_CMD = ('{joint_names: [Joint1,Joint2,Joint3,Joint4,Joint5,Joint6], '
                 'points: [{positions: [0.0, 1.0, -1.5, 0.0, 0.3, 0.0], '
                 'time_from_start: {sec: 3, nanosec: 0}}]}')


# ── 日志工具 ────────────────────────────────────────────────────────────────────
def log_args(log_level):
    """第三方底座节点的日志级别参数（log_level=None 则不加，保持默认 info）。"""
    return ['--ros-args', '--log-level', log_level] if log_level is not None else []


# ── 条件工具 ────────────────────────────────────────────────────────────────────
def is_mode(ctrl, name):
    """controller == name 的 IfCondition。"""
    return IfCondition(PythonExpression(["'", ctrl, "' == '", name, "'"]))


def in_modes(ctrl, names):
    """controller ∈ names 的 IfCondition。"""
    lst = "[" + ",".join(f"'{n}'" for n in names) + "]"
    return IfCondition(PythonExpression(["'", ctrl, "' in ", lst]))


# ── controller GUI（基础 5 种；cartesian_velocity 的 GUI 随 servo 时序单独起）──────
# 调试控制器/GUI 在 robot_arm_debug 包（产品栈 arm_commander_node 在 robot_arm_node）
# use_sim_time 必须按后端传入（默认 True 仅适配 gazebo 调用点；mujoco/real 传 False）：
# 实物无 /clock 时若为 True，节点内所有 ROS 定时器永不触发（位姿面板卡 '--'）。
def gui_node(ctrl, mode, exe, use_sim_time=True):
    return Node(package='robot_arm_debug', executable=exe, output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=is_mode(ctrl, mode))


def controller_gui_nodes(ctrl):
    """joint_position / cartesian_moveit / realtime_ik / trajectory / spherical_orbit 的 GUI。"""
    return [
        gui_node(ctrl, 'joint_position',        'joint_position_gui'),
        gui_node(ctrl, 'cartesian_moveit',      'cartesian_moveit_gui'),
        gui_node(ctrl, 'cartesian_realtime_ik', 'cartesian_realtime_ik_gui'),
        gui_node(ctrl, 'cartesian_trajectory',  'cartesian_trajectory_gui'),
        gui_node(ctrl, 'spherical_orbit',       'spherical_orbit_gui'),
    ]


# ── move_group（需要 IK 的模式，含 commander）─────────────────────────────────────
def move_group_node(ctrl, *, robot_description, srdf, kinematics, joint_limits,
                    planning_pipeline, moveit_controllers, use_sim_time,
                    log_level=None):
    return Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        arguments=log_args(log_level),
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            planning_pipeline,
            moveit_controllers,
            {'use_sim_time': use_sim_time, 'start_state_max_bounds_error': 0.5},
        ],
        condition=in_modes(ctrl, MOVEIT_MODES),
    )


# ── MoveIt Servo（cartesian_velocity / ibvs_control）──────────────────────────────
def servo_node(condition, *, robot_description, srdf, kinematics, joint_limits,
               servo_params, use_sim_time, use_gazebo=None, log_level=None):
    """servo_node_main。ibvs/velocity 无 move_group，恒禁碰撞检测避免 run_duration 超时。

    servo_params 必须已包在 {'moveit_servo': ...} 命名空间下——Humble 的
    makeServoParameters 只从 moveit_servo. 前缀读参数，裸传会整体静默回退
    Panda 默认值（panda_arm 规划组、/panda_arm_controller 输出话题）。
    use_gazebo 同属 moveit_servo.* 参数，在此并入正确命名空间。
    """
    servo_overrides = {'check_collisions': False}
    if use_gazebo is not None:
        servo_overrides['use_gazebo'] = use_gazebo
    return Node(
        package='moveit_servo', executable='servo_node_main', name='servo_node', output='screen',
        arguments=log_args(log_level),
        parameters=[
            servo_params,
            {'robot_description': robot_description,
             'robot_description_semantic': srdf,
             'use_sim_time': use_sim_time},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            {'moveit_servo': servo_overrides},
        ],
        condition=condition,
    )


# ── 安全姿态预移动 ────────────────────────────────────────────────────────────────
def safe_pose_action(condition):
    return ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/arm_controller/joint_trajectory',
             'trajectory_msgs/msg/JointTrajectory', SAFE_POSE_CMD],
        output='screen', condition=condition)


# ── 控制模式仲裁器（基础设施，所有模式常驻）──────────────────────────────────────
def mode_manager_node(use_sim_time, hold_controllers=None):
    """语义控制模式的唯一权威（TRAJECTORY/JOINT_VELOCITY/...），后端无感。

    非模式相关基础设施，与 ctrl 无关、常驻启动：上层通过
    /robot_arm/switch_control_mode 切模式，本节点做 switch_controller + 播种 + 看门狗
    + 速度总线限幅/限位刹车。
    仅需 controller_manager 存在（gazebo/real 均有；mujoco 无 → 切换会明确报错，不崩）。

    hold_controllers：**仅 PV 后端（velocity_backend:=velocity_controller）生效** ——
    进入速度/力矩模式时与流式控制器一起激活、回轨迹模式时一起停用的「保持控制器」。
    默认 trajectory 后端不切控制器，速度和位置共用 arm_controller，用不到它。
    """
    params = {'use_sim_time': use_sim_time}
    if hold_controllers:
        params['hold_controllers'] = list(hold_controllers)
    return Node(package='robot_arm_node', executable='mode_manager_node', output='screen',
                parameters=[params])


# ── commander + test_gui ─────────────────────────────────────────────────────────
def commander_nodes(ctrl, gui, use_sim_time):
    return [
        Node(package='robot_arm_node', executable='arm_commander_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             condition=is_mode(ctrl, 'commander')),
        Node(package='robot_arm_debug', executable='commander_test_gui', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(PythonExpression(
                 ["'", ctrl, "' == 'commander' and '", gui, "' == 'true'"]))),
    ]


# 注：Python 版 ibvs_control（ibvs_control_node + IBVS_Controller）已被 C++ visp_ibvs_node.cpp
# 全面取代并删除，故不再提供 ibvs_nodes 工厂；红方块 IBVS 统一走 visp_nodes()。


# ── visp_ibvs（ViSP C++ 版；mode 名各 backend 不同，作参数传入）────────────────────
def visp_nodes(ctrl, mode, *, robot_description, use_sim_time,
               perception_topic=None, control_depth=False):
    cond = is_mode(ctrl, mode)
    visp_params = {'robot_description': robot_description,
                   'use_sim_time': use_sim_time,
                   'control_depth': control_depth}
    if perception_topic:
        visp_params['perception_topic'] = perception_topic
    return [
        Node(package='robot_arm_debug', executable='red_box_detector', output='screen',
             parameters=[{'use_sim_time': use_sim_time}], condition=cond),
        Node(package='robot_arm_debug', executable='visp_ibvs_node', output='screen',
             parameters=[visp_params], condition=cond),
        Node(package='robot_arm_debug', executable='visp_ibvs_gui', output='screen',
             condition=cond),
    ]
