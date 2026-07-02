"""
@file   _arm_launch_common.py
@brief  上层栈节点工厂 —— controller GUI / move_group / servo / 安全预移动 / commander / ibvs / visp 的单一来源
@version 1.0
@date   2026-07-01

PR-4 launch 重构·第二步：把三后端各抄一遍的「上层栈」节点定义收成一份。

设计取舍：不做「一个 common.launch.py 用 IncludeLaunchDescription 统一时序」——因为三后端
的启动时序本质不同（gazebo 用 OnProcessExit 事件链、mujoco/real 用 TimerAction），且
robot_description / use_sim_time / moveit_controllers 各异。改用**工厂函数模块**：每个
backend 保留自己的时序编排，只调用这里的工厂拿「同一份节点定义」，从而消除复制粘贴漂移，
风险最低（不重写已调好的时序）。

注意：本文件以下划线开头、非 `*.launch.py`，不会被 `ros2 launch` 当作入口。backend launch
用 `sys.path.insert(0, os.path.dirname(__file__)); import _arm_launch_common` 导入。

约定：
  ctrl / gui 传 LaunchConfiguration；use_sim_time 传 bool（各 backend 自己的固定值）；
  robot_description / srdf 传字符串；kinematics/joint_limits/planning_pipeline/moveit_controllers/
  servo_params 传已 load 的 dict。

@copyright Copyright (c) 2026 eMeet
"""

from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ── 模式分组（单一来源，供 backend 复用）─────────────────────────────────────────
# 需要 IK / move_group 的模式（含 commander）
MOVEIT_MODES = ['cartesian_moveit', 'cartesian_realtime_ik',
                'cartesian_trajectory', 'spherical_orbit', 'commander']
# 需要 MoveIt Servo + 安全预移动的模式
SERVO_MODES  = ['cartesian_velocity']

# 安全姿态预移动指令（全零关节是运动学奇异点，Servo 启动前须先移走）
SAFE_POSE_CMD = ('{joint_names: [Joint1,Joint2,Joint3,Joint4,Joint5,Joint6], '
                 'points: [{positions: [0.0, 1.0, -1.5, 0.0, 0.3, 0.0], '
                 'time_from_start: {sec: 3, nanosec: 0}}]}')


# ── 条件工具 ────────────────────────────────────────────────────────────────────
def is_mode(ctrl, name):
    """controller == name 的 IfCondition。"""
    return IfCondition(PythonExpression(["'", ctrl, "' == '", name, "'"]))


def in_modes(ctrl, names):
    """controller ∈ names 的 IfCondition。"""
    lst = "[" + ",".join(f"'{n}'" for n in names) + "]"
    return IfCondition(PythonExpression(["'", ctrl, "' in ", lst]))


# ── controller GUI（基础 5 种；cartesian_velocity 的 GUI 随 servo 时序单独起）──────
def gui_node(ctrl, mode, exe):
    return Node(package='robot_arm_node', executable=exe, output='screen',
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
                    planning_pipeline, moveit_controllers, use_sim_time):
    return Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
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
               servo_params, use_sim_time, use_gazebo=None):
    """servo_node_main。ibvs/velocity 无 move_group，恒禁碰撞检测避免 run_duration 超时。"""
    rd_params = {'robot_description': robot_description,
                 'robot_description_semantic': srdf,
                 'use_sim_time': use_sim_time}
    if use_gazebo is not None:
        rd_params['use_gazebo'] = use_gazebo
    return Node(
        package='moveit_servo', executable='servo_node_main', name='servo_node', output='screen',
        parameters=[
            servo_params,
            rd_params,
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            {'moveit_servo': {'check_collisions': False}},
        ],
        condition=condition,
    )


# ── 安全姿态预移动 ────────────────────────────────────────────────────────────────
def safe_pose_action(condition):
    return ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/arm_controller/joint_trajectory',
             'trajectory_msgs/msg/JointTrajectory', SAFE_POSE_CMD],
        output='screen', condition=condition)


# ── commander + test_gui ─────────────────────────────────────────────────────────
def commander_nodes(ctrl, gui, use_sim_time):
    return [
        Node(package='robot_arm_node', executable='arm_commander_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             condition=is_mode(ctrl, 'commander')),
        Node(package='robot_arm_node', executable='commander_test_gui', output='screen',
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
        Node(package='robot_arm_node', executable='red_box_detector', output='screen',
             parameters=[{'use_sim_time': use_sim_time}], condition=cond),
        Node(package='robot_arm_node', executable='visp_ibvs_node', output='screen',
             parameters=[visp_params], condition=cond),
        Node(package='robot_arm_node', executable='visp_ibvs_gui', output='screen',
             condition=cond),
    ]
