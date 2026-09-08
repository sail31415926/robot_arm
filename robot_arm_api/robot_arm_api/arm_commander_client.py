# -*- coding: utf-8 -*-
"""
@file      arm_commander_client.py
@brief     机械臂 Arm Commander 对外接口的 Python 客户端库（Director / 大模型运镜规划视角）：
           把 /robot_arm 的 action / service / topic 包成阻塞式方法，附云台 V2 直连客户端与
           「JSON 运镜步骤表」执行器。命令行 demo 在同包 arm_commander_demo.py。
@version   1.1
@date      2026-09-08
@copyright Copyright (c) 2026 eMeet

定位：给「大模型解析提示词 → 得到运镜参数（推拉多少、平移多少、从哪到哪、直线还是弧线）」
      的上层模块一个**不用关心 ROS2 细节**的调用面。每次下发都会把等价的 `ros2 action send_goal`
      / `ros2 service call` / `ros2 topic pub` 打印到日志，方便复制到终端复现。

对象（一个 ROS 节点 + 后台执行器，由 ArmApi 持有、两个客户端共用）：
  · ArmCommanderClient  机械臂 Arm Commander 接口（/robot_arm/...，服务端 arm_commander_node）
                          位姿到点 / 关节到点 / 直线运镜 / 球面环绕运镜 / 视觉跟随 /
                          关节速度流 / 末端速度流 / 使能 / 回零 / 清错 / 急停 / 控制模式切换
  · GimbalV2Client      云台 V2 直连接口（/robot_gimbal_v2/...，执行节点 robot_gimbal_node_v2 在云台板端）
                          朝向到位（action）/ 朝向流式 / 速度点动 / 冻结 / 回中 / 电机启停 / 转发流开关
                          —— 云台是机械臂的 J4-6，机械臂位置类动作本身就会带着云台一起动；
                          这里是绕开 IK、直接指定云台角的补充通路
  · ArmApi              门面：`with ArmApi() as api: api.arm.dolly(0.1)`（gimbal 挂在 api.gimbal）
  · execute_plan        把大模型输出的 JSON 步骤表逐条映射到上面的方法（schema 见 README）

依赖接口包：robot_arm_interfaces（必需）；robot_gimbal_interfaces_v2 可选 —— 缺失时 ArmApi.gimbal 为 None。

上层用法：
  from robot_arm_api import ArmApi, make_pose
  with ArmApi() as api:
      api.arm.wait_ready()
      api.arm.enable()
      api.arm.move_to_observe()
      api.arm.dolly(0.10, speed='slow')                        # 推镜 10cm（base_link +x）
      api.arm.move_to_pose(make_pose(0.25, 0.0, 0.65, 0, 0, 0))
      api.arm.arc_around((0.6, 0.0, 0.7), 0.4, -30, 30, on_camera_ready=start_record)
      if api.gimbal:
          api.gimbal.rotate_to_deg(pan_deg=20, tilt_deg=-10)

═══════════════════════════════ 本文件函数 / 方法汇总 ═══════════════════════════════

【模块级工具】
  speed_code(speed)                      速度档位 'slow'/'normal'/'fast' 或 0/1/2 → uint8
  make_pose(x, y, z, roll, pitch, yaw)   构造 ArmPose（米 / 度，base_link 系）
  offset_pose(pose, dx, ...)             在已有 ArmPose 上叠加增量，返回新对象
  pose_to_str(pose)                      ArmPose → 可读字符串
  msg_to_yaml(msg) / _yaml_value(v)      任意 ROS 消息 → ros2 CLI 可用的 YAML 内联字符串
  ros_type_name(cls)                     接口类 → 'pkg/kind/Name'

【CallResult】                           所有阻塞调用统一返回：success / reason / result

【_Action】                              ActionClient + 名字 / 类型的小包装

【_RosClientBase（两个客户端的公共基类）】
  cancel()                               取消当前正在执行的 action goal
  _wait_future / _show_cli / _call_service / _service_result / _send_goal / _wrap_feedback / _stream

【ArmCommanderClient —— 机械臂】
  状态：
    wait_ready(timeout_sec)              等 Commander action server + 首帧 ArmStatus
    get_status()                         最近一帧 ArmStatus（None = 还没收到）
    get_pose()                           末端当前位姿 ArmPose（副本）
    get_joints()                         {Joint1..Joint6: rad}（来自 /joint_states）
    get_control_mode()                   当前语义控制模式（ControlMode 常量）
    is_moving() / has_error() / is_idle()
    wait_until_idle(timeout_sec)         等到 is_moving=false 且命令不在执行中
    wait_camera_ready(timeout_sec)       等运镜到达起拍点（camera_ready 上升沿，触发录像用）
    status_summary()                     一行状态摘要
  运维（service）：
    enable(on) / disable()               伺服上电 / 下电
    homing(timeout_sec)                  回零（阻塞）
    reset_error()                        清驱动故障 + Commander ERROR/STOPPED → IDLE
    stop()                               软件急停（停 action + 停速度流；之后需 reset_error）
    switch_control_mode(mode)            切 TRAJECTORY / JOINT_VELOCITY（速度流前置）
    enter_velocity_mode() / exit_velocity_mode()
  位置类（action，阻塞到到位）：
    move_to_stowed(speed, return_to_start)   收纳位（6 轴关节回零）
    move_to_observe(speed, return_to_start)  观察位（Commander 内置预设）
    move_to_pose(pose, speed, return_to_start, timeout_sec)   绝对末端位姿（拍摄位）
    move_relative(dx, dy, dz, droll, dpitch, dyaw, speed)     相对当前末端位姿
    move_to_joint(j1, j2, j3, speed, relative, duration_sec)  关节空间点到点（不过 IK）
  运镜（action，阻塞到结束）：
    shot_linear(start, end, speed, return_to_start, on_camera_ready)   直线运镜（两端绝对位姿）
    shot_linear_from_current(dx, dy, dz, droll, dpitch, dyaw, ...)     从当前位姿出发的直线运镜
    dolly(distance_m, ...)               推 / 拉（沿 +x，正 = 靠近主体）
    truck(distance_m, ...)               横移（沿 +y，正 = 向左）
    crane(distance_m, ...)               升 / 降（沿 +z，正 = 上升）
    shot_orbit(center, az_start, az_end, el_start, el_end, r_start, r_end, ...) 球面环绕运镜
    arc_around(center, radius_m, az_start, az_end, elevation_deg, ...)  等半径等仰角的环绕简写
  视觉跟随（action，非阻塞）：
    track_target_start(...) / track_target_stop()
  速度流（topic，需 JOINT_VELOCITY 模式，≥3Hz 持续发布）：
    publish_cartesian_velocity(vx, vy, vz, wroll, wpitch, wyaw)   单帧末端 twist
    publish_joint_velocity(v1, v2, v3)                            单帧 J1-3 角速度
    jog_cartesian(..., duration_sec, auto_mode)   按时长持续发末端速度，结束补 0 帧
    jog_joint(v1, v2, v3, duration_sec, auto_mode) 按时长持续发关节速度，结束补 0 帧
  内部：
    _move_to_pose_goal(...) / _run_shot(...) / _current_pose_or_zero()
    _on_status / _on_joint_states / _on_control_mode   订阅回调（缓存状态）

【GimbalV2Client —— 云台 V2 直连】
    wait_ready(timeout_sec) / get_status() / get_angles() / get_angles_deg()
    rotate_to(pan, roll, tilt, timeout_sec, wait)      转到指定角（rad，None = 该轴不动，action）
    rotate_to_deg(pan_deg, roll_deg, tilt_deg, ...)    同上，度制
    set_position_stream(pan, roll, tilt, max_vel)      流式位置指令（后到覆盖先到，不等到位）
    publish_velocity(pan_vel, roll_vel, tilt_vel)      单帧速度（rad/s）
    jog(pan_vel, roll_vel, tilt_vel, duration_sec)     按时长速度点动，结束补 0 帧
    freeze() / go_zero() / start_motor() / stop_motor() / gyro_calib()
    set_forward_cmd_enable(enable)                      开/关机械臂转发流对云台的控制权
  内部：_publish_cmd(mode, **fields) / _on_status / _on_joint_states_raw

【ArmApi —— 门面】
    ArmApi(node_name, dry_run, print_cli, with_gimbal)
    shutdown() / __enter__ / __exit__

【运镜步骤表】
    execute_plan(api, steps, stop_on_error)     逐条执行 JSON 步骤（op + 参数）
    run_plan_step(api, step)                    执行单条步骤（demo 的 CLI 子命令也走这里）
    _pose_from_dict(d, fallback)                {x,y,z,roll,pitch,yaw} → ArmPose
"""

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

from robot_arm_interfaces.action import (ArmMoveToJoint, ArmMoveToPose, ArmTrackTarget,
                                         ArmTrajectoryShot)
from robot_arm_interfaces.msg import (ArmFollowCommand, ArmJointVelocityCommand, ArmPose,
                                      ArmStatus, ArmTwist, ControlMode)
from robot_arm_interfaces.srv import (ArmEnable, ArmHoming, ArmResetError, ArmStop,
                                      SwitchControlMode)

try:  # 云台 V2 接口（执行节点在云台板端）—— 可选
    from robot_gimbal_interfaces_v2.action import RotateToAngle
    from robot_gimbal_interfaces_v2.msg import GimbalCommand, GimbalStatus
    from robot_gimbal_interfaces_v2.srv import SetForwardCmdEnable
    HAS_GIMBAL_IFACE = True
except ImportError:  # pragma: no cover - 取决于工作空间是否编译了该包
    HAS_GIMBAL_IFACE = False

__all__ = [
    'ArmCommanderClient', 'GimbalV2Client', 'ArmApi', 'CallResult', 'HAS_GIMBAL_IFACE',
    'make_pose', 'offset_pose', 'pose_to_str', 'speed_code', 'msg_to_yaml', 'ros_type_name',
    'execute_plan', 'run_plan_step',
]

# ══════════════════════════════════════════════════════════════════════════════
#  接口名常量 —— 与各节点源码一一对应（改名时只改这里）
# ══════════════════════════════════════════════════════════════════════════════
# 机械臂：robot_arm_node/src/commander/arm_commander_node.cpp、src/mode/mode_manager_node.cpp
ARM_TOPIC_STATUS = '/robot_arm/arm_status'
ARM_TOPIC_CONTROL_MODE = '/robot_arm/control_mode'          # latched（TRANSIENT_LOCAL）
ARM_TOPIC_FOLLOW_COMMAND = '/robot_arm/follow_command'      # ArmFollowCommand（末端 twist）
ARM_TOPIC_JOINT_VELOCITY = '/robot_arm/cmd/joint_velocity'  # ArmJointVelocityCommand（J1-3）
ARM_TOPIC_JOINT_STATES = '/joint_states'
ARM_ACTION_MOVE_TO_POSE = '/robot_arm/move_to_pose'
ARM_ACTION_MOVE_TO_JOINT = '/robot_arm/move_to_joint'
ARM_ACTION_TRAJECTORY_SHOT = '/robot_arm/trajectory_shot'
ARM_ACTION_TRACK_TARGET = '/robot_arm/track_target'
ARM_SRV_STOP = '/robot_arm/stop'
ARM_SRV_ENABLE = '/robot_arm/enable'
ARM_SRV_HOMING = '/robot_arm/homing'
ARM_SRV_RESET_ERROR = '/robot_arm/reset_error'
ARM_SRV_SWITCH_MODE = '/robot_arm/switch_control_mode'
ARM_JOINT_NAMES = ('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6')

# 云台 V2：robot_gimbal_node_v2/src/robot_gimbal_node.cpp（注意都带 _v2，V1 名字已废）
GIMBAL_TOPIC_CMD = '/robot_gimbal_v2/gimbal_cmd'
GIMBAL_TOPIC_CMD_VEL = '/robot_gimbal_v2/cmd_vel'           # geometry_msgs/Twist
GIMBAL_TOPIC_STATUS = '/robot_gimbal_v2/status'
GIMBAL_TOPIC_JOINT_STATES_RAW = '/robot_gimbal_v2/joint_states_raw'
GIMBAL_ACTION_ROTATE = '/robot_gimbal_v2/rotate_to_angle'
GIMBAL_SRV_FORWARD_ENABLE = '/robot_gimbal_v2/set_forward_cmd_enable'

# 枚举名映射（打印用）
SPEED_NAMES = {'slow': 0, 'normal': 1, 'fast': 2}
MODE_NAMES = {0: 'TRAJECTORY', 1: 'JOINT_VELOCITY', 2: 'JOINT_EFFORT', 3: 'ADMITTANCE'}
POSE_STATE_NAMES = {0: 'STOWED', 1: 'OBSERVE', 2: 'SHOOTING'}
ARM_ERROR_NAMES = {0: 'ERR_NONE', 1: 'ERR_LIMIT', 2: 'ERR_DRIVER', 3: 'ERR_TIMEOUT'}
ARM_RESULT_NAMES = {0: 'NONE', 1: 'EXECUTING', 2: 'SUCCEEDED', 3: 'FAILED', 4: 'ABORTED'}
GOAL_STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                     4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

# 默认超时
DEFAULT_SERVICE_TIMEOUT_SEC = 5.0
DEFAULT_ACTION_TIMEOUT_SEC = 60.0
DEFAULT_SHOT_TIMEOUT_SEC = 180.0
DEFAULT_HOMING_TIMEOUT_SEC = 120.0
# 速度流发布频率：Commander 断流看门狗 0.3s，JTC 位置流 50Hz，这里与 commander_test_gui 一致取 50Hz
VELOCITY_STREAM_RATE_HZ = 50.0
# 速度点动结束后的静置时间：Commander 的 VelocityStreamServer 在收到停帧后仍会按断流看门狗
# （command_timeout 0.3s）把最后一点位置流发完；若此时立刻发轨迹类 goal（如 stow），那几帧
# 位置流会覆盖 JTC 刚收到的轨迹，动作原地不动直到 Commander 超时进 ERROR（2026-09-08 mock 臂实测）。
VELOCITY_STREAM_SETTLE_SEC = 0.5
GIMBAL_STREAM_RATE_HZ = 20.0


# ══════════════════════════════════════════════════════════════════════════════
#  模块级工具
# ══════════════════════════════════════════════════════════════════════════════
def speed_code(speed: Any) -> int:
    """@brief 把速度档位统一成接口用的 uint8 常量。

    @param speed 'slow' / 'normal' / 'fast'（不区分大小写）或 0 / 1 / 2
    @return SPEED_SLOW=0 / SPEED_NORMAL=1 / SPEED_FAST=2
    @throws ValueError 档位非法
    """
    if isinstance(speed, str):
        key = speed.strip().lower()
        if key not in SPEED_NAMES:
            raise ValueError(f"速度档位必须是 slow/normal/fast，收到 '{speed}'")
        return SPEED_NAMES[key]
    code = int(speed)
    if code not in (0, 1, 2):
        raise ValueError(f'速度档位必须是 0/1/2，收到 {code}')
    return code


def make_pose(x: float, y: float, z: float,
              roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> ArmPose:
    """@brief 构造末端位姿 ArmPose（机械臂 base_link 系）。

    @param x,y,z            位置，米（x 前 / y 左 / z 上）
    @param roll,pitch,yaw   姿态，度（pitch 正值抬头；yaw 按 ArmPose.msg 注释：正值向右转）
    @return ArmPose
    """
    pose = ArmPose()
    pose.x, pose.y, pose.z = float(x), float(y), float(z)
    pose.roll, pose.pitch, pose.yaw = float(roll), float(pitch), float(yaw)
    return pose


def offset_pose(pose: ArmPose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0) -> ArmPose:
    """@brief 在已有位姿上叠加增量，返回新的 ArmPose（不修改入参）。

    @param pose   基准位姿
    @param dx,dy,dz          位置增量，米
    @param droll,dpitch,dyaw 姿态增量，度
    @return 新的 ArmPose
    """
    return make_pose(pose.x + dx, pose.y + dy, pose.z + dz,
                     pose.roll + droll, pose.pitch + dpitch, pose.yaw + dyaw)


def pose_to_str(pose: Optional[ArmPose]) -> str:
    """@brief ArmPose → 可读字符串（米保留 3 位、度保留 1 位）。

    @param pose ArmPose 或 None
    @return 形如 'xyz=(0.250, 0.000, 0.650)m rpy=(0.0, 0.0, 0.0)deg'
    """
    if pose is None:
        return 'pose=None'
    return (f'xyz=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})m '
            f'rpy=({pose.roll:.1f}, {pose.pitch:.1f}, {pose.yaw:.1f})deg')


def _yaml_value(value: Any) -> str:
    """@brief 单个字段值 → YAML 内联写法（msg_to_yaml 的递归辅助）。

    @param value bool / int / float / str / list / 嵌套消息
    @return YAML 片段
    """
    if isinstance(value, bool):                      # bool 是 int 子类，必须先判
        return 'true' if value else 'false'
    if isinstance(value, float):
        if math.isnan(value):
            return '.nan'
        return repr(round(value, 6))                 # 留小数点，避免 YAML 读成 int
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str):
        return f"'{value}'"
    if hasattr(value, 'get_fields_and_field_types'):  # 嵌套消息（ArmPose / Vector3 …）
        return msg_to_yaml(value)
    if hasattr(value, 'tolist'):                     # array.array / numpy
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(_yaml_value(v) for v in value) + ']'
    return str(value)


def msg_to_yaml(msg: Any) -> str:
    """@brief 任意 rosidl 消息对象 → `ros2 topic pub / action send_goal` 可直接用的 YAML 字符串。

    从**真正要发出去的消息对象**序列化，不手写模板，接口改字段时不会漂开。
    @param msg 消息 / Goal / Request 对象
    @return 形如 '{x: 0.25, y: 0.0, ...}'
    """
    fields = msg.get_fields_and_field_types().keys()
    return '{' + ', '.join(f'{name}: {_yaml_value(getattr(msg, name))}' for name in fields) + '}'


def ros_type_name(cls: Any) -> str:
    """@brief 接口类 → CLI 用的类型名 'pkg/kind/Name'（kind = msg | srv | action）。

    @param cls 例如 ArmMoveToPose（action）、ArmEnable（srv）、ArmFollowCommand（msg）
    @return 例如 'robot_arm_interfaces/action/ArmMoveToPose'
    """
    pkg, kind = cls.__module__.split('.')[:2]
    return f'{pkg}/{kind}/{cls.__name__}'


# ══════════════════════════════════════════════════════════════════════════════
#  统一返回值
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class CallResult:
    """@brief 所有阻塞式调用的统一返回值。

    @var success 是否成功（action：SUCCEEDED 且 result.success；service：response.success）
    @var reason  人类可读原因：action 的 exit_reason（reached / timeout / unreachable …）
                 或 service 的 message；dry-run 时为 'dry-run'
    @var result  原始 Result / Response 对象（需要细节时用），非阻塞调用时为 goal handle
    """
    success: bool
    reason: str = ''
    result: Any = None

    def __bool__(self) -> bool:
        """@brief 允许 `if api.arm.dolly(0.1):` 这样直接判真。
        @return success
        """
        return self.success

    def __str__(self) -> str:
        """@brief 打印用摘要。
        @return 'OK(reached)' / 'FAIL(timeout)'
        """
        return f"{'OK' if self.success else 'FAIL'}({self.reason})"


class _Action:
    """@brief ActionClient 及其名字 / 类型的小包装（rclpy 的 ActionClient 不公开这两项）。"""

    def __init__(self, node: Node, action_type: Any, name: str):
        """@brief 创建 action client。
        @param node        宿主节点
        @param action_type action 接口类
        @param name        action 名
        """
        self.name = name
        self.type = action_type
        self.client = ActionClient(node, action_type, name)


# ══════════════════════════════════════════════════════════════════════════════
#  公共基类
# ══════════════════════════════════════════════════════════════════════════════
class _RosClientBase:
    """@brief 两个客户端的公共部分：等待 future、打印等效指令、service / action 阻塞调用。

    线程模型：ROS 回调由 ArmApi 里的后台执行器线程驱动；本类所有阻塞方法只在
    调用者线程里轮询 future，**不要**在 ROS 回调里调用这些阻塞方法（会死锁）。
    """

    def __init__(self, node: Node, dry_run: bool = False, print_cli: bool = True):
        """@brief 记录节点与开关。
        @param node      共用的 rclpy 节点
        @param dry_run   True = 只打印等效 ros2 指令、不下发、不等待
        @param print_cli 是否打印等效 ros2 指令
        """
        self._node = node
        self._log = node.get_logger()
        self.dry_run = dry_run
        self.print_cli = print_cli
        self._goal_lock = threading.Lock()
        self._active_goal = None

    # ── 基础设施 ────────────────────────────────────────────────────────────
    def _wait_future(self, future: Any, timeout_sec: Optional[float]) -> bool:
        """@brief 在调用者线程里轮询等待 future 完成（执行器在后台线程 spin）。
        @param future      rclpy Future
        @param timeout_sec 超时秒数；None 或 <=0 表示一直等
        @return True = 完成；False = 超时或 rclpy 已关闭
        """
        deadline = (time.monotonic() + timeout_sec) if timeout_sec and timeout_sec > 0 else None
        while rclpy.ok() and not future.done():
            if deadline is not None and time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return future.done()

    def _show_cli(self, text: str) -> None:
        """@brief 打印等效 ros2 CLI 指令（受 print_cli 开关控制）。
        @param text 完整指令文本
        """
        if self.print_cli:
            self._log.info('等效指令: ' + text)

    def _call_service(self, client: Any, request: Any,
                      timeout_sec: float) -> Tuple[Optional[Any], str]:
        """@brief 阻塞调用 service。
        @param client      rclpy service client
        @param request     Request 对象
        @param timeout_sec 等服务上线 + 等应答的超时
        @return (response, '') 或 (None, 失败原因)
        """
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None, f'服务 {client.srv_name} 不可用（对应节点未启动？）'
        future = client.call_async(request)
        if not self._wait_future(future, timeout_sec):
            return None, f'服务 {client.srv_name} 应答超时（{timeout_sec}s）'
        return future.result(), ''

    def _service_result(self, client: Any, request: Any,
                        timeout_sec: float = DEFAULT_SERVICE_TIMEOUT_SEC) -> CallResult:
        """@brief 调 service 并把 success / message 风格的应答整理成 CallResult。
        @param client      rclpy service client
        @param request     Request 对象
        @param timeout_sec 超时
        @return CallResult（dry-run 时直接 success）
        """
        self._show_cli(f'ros2 service call {client.srv_name} {ros_type_name(client.srv_type)} '
                       f'"{msg_to_yaml(request)}"')
        if self.dry_run:
            return CallResult(True, 'dry-run')
        response, err = self._call_service(client, request, timeout_sec)
        if response is None:
            self._log.error(err)
            return CallResult(False, err)
        ok = bool(getattr(response, 'success', True))
        message = str(getattr(response, 'message', ''))
        return CallResult(ok, message or ('ok' if ok else 'rejected'), response)

    def _send_goal(self, action: _Action, goal: Any, timeout_sec: float,
                   feedback_cb: Optional[Callable[[Any], None]] = None,
                   wait_result: bool = True) -> CallResult:
        """@brief 发送 action goal；默认阻塞到结果，超时则自动取消。
        @param action      _Action 包装
        @param goal        Goal 对象
        @param timeout_sec 等结果的超时（秒）；超时会 cancel
        @param feedback_cb 反馈回调 f(feedback_msg)；None = 每秒打印一次 progress
        @param wait_result False = goal 被接受后立即返回（result 字段放 goal handle）
        @return CallResult
        """
        self._show_cli(f'ros2 action send_goal {action.name} {ros_type_name(action.type)} '
                       f'"{msg_to_yaml(goal)}"')
        if self.dry_run:
            return CallResult(True, 'dry-run')
        if not action.client.wait_for_server(timeout_sec=DEFAULT_SERVICE_TIMEOUT_SEC):
            reason = f'action server {action.name} 不可用（对应节点未启动？）'
            self._log.error(reason)
            return CallResult(False, reason)

        send_future = action.client.send_goal_async(
            goal, feedback_callback=self._wrap_feedback(action.name, feedback_cb))
        if not self._wait_future(send_future, DEFAULT_SERVICE_TIMEOUT_SEC):
            return CallResult(False, 'goal 应答超时')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            reason = ('goal 被拒绝：执行端非空闲（正在执行 / STOPPED / ERROR）。'
                      '机械臂请先 stop 或 reset_error，云台请先 cancel')
            self._log.warning(reason)
            return CallResult(False, reason)
        with self._goal_lock:
            self._active_goal = goal_handle
        if not wait_result:
            return CallResult(True, 'accepted', goal_handle)

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec):
            self._log.warning(f'{action.name} 等结果超时（{timeout_sec}s），取消 goal')
            self._wait_future(goal_handle.cancel_goal_async(), 3.0)
            with self._goal_lock:
                self._active_goal = None
            return CallResult(False, f'client timeout {timeout_sec}s')
        with self._goal_lock:
            self._active_goal = None
        wrapped = result_future.result()
        status, result = wrapped.status, wrapped.result
        ok = status == GoalStatus.STATUS_SUCCEEDED and bool(getattr(result, 'success', True))
        reason = str(getattr(result, 'exit_reason', '') or getattr(result, 'message', ''))
        if not reason:
            reason = GOAL_STATUS_NAMES.get(status, str(status))
            if status == GoalStatus.STATUS_ABORTED:
                # Commander 在非空闲态（ERROR / STOPPED / 正忙）会接受后立刻 abort 且不带 exit_reason
                reason += '（执行端直接中止：Commander 处于 ERROR/STOPPED 或正忙，先 reset_error）'
        return CallResult(ok, reason, result)

    def _wrap_feedback(self, label: str,
                       user_cb: Optional[Callable[[Any], None]]) -> Callable[[Any], None]:
        """@brief 生成 feedback 回调：有用户回调就转发，否则每秒打印一次进度。
        @param label   action 名（打印用）
        @param user_cb 用户回调或 None
        @return rclpy 需要的 feedback_callback
        """
        last_print = [0.0]

        def _cb(fb_msg: Any) -> None:
            """@brief rclpy feedback 回调：转发给用户或节流打印进度。
            @param fb_msg 带 .feedback 的反馈消息
            """
            feedback = fb_msg.feedback
            if user_cb is not None:
                user_cb(feedback)
                return
            now = time.monotonic()
            if now - last_print[0] >= 1.0:
                last_print[0] = now
                progress = getattr(feedback, 'progress_percent', None)
                if progress is not None:
                    self._log.info(f'{label} 进度 {float(progress):.0f}%')
        return _cb

    def _stream(self, publish_once: Callable[[], None], duration_sec: float,
                rate_hz: float) -> None:
        """@brief 以 wall-clock 节拍持续调用 publish_once（速度流用）。

        用 time.monotonic 而不是 ROS 定时器：实机没有 /clock，use_sim_time 一旦为 true
        ROS 定时器会静默不触发（见 robot_arm/CLAUDE.md）。
        @param publish_once 发一帧
        @param duration_sec 持续时长
        @param rate_hz      频率
        """
        period = 1.0 / max(rate_hz, 1.0)
        end = time.monotonic() + max(duration_sec, 0.0)
        next_tick = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            publish_once()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    # ── 公共操作 ────────────────────────────────────────────────────────────
    def cancel(self) -> bool:
        """@brief 取消当前正在执行的 action goal（阻塞式调用超时时也会自动调用）。
        @return True = 已发出取消并得到应答
        """
        with self._goal_lock:
            goal_handle = self._active_goal
            self._active_goal = None
        if goal_handle is None:
            return False
        return self._wait_future(goal_handle.cancel_goal_async(), 3.0)


# ══════════════════════════════════════════════════════════════════════════════
#  机械臂
# ══════════════════════════════════════════════════════════════════════════════
class ArmCommanderClient(_RosClientBase):
    """@brief 机械臂 Arm Commander（arm_commander_node）对外接口的客户端封装（Director 视角）。

    坐标 / 单位约定（与 robot_arm_interfaces 一致）：位姿在机械臂 base_link 系，位置米、姿态度；
    速度流线速度 m/s、角速度 **度/秒**；关节角 rad。规划末端是 gimbal_tool0（云台之后），
    位置类动作 6 轴一起动、到位判据只看臂 J1-3。
    """

    def __init__(self, node: Node, dry_run: bool = False, print_cli: bool = True):
        """@brief 建立机械臂全部 action / service / topic 端点。
        @param node      共用节点
        @param dry_run   只打印不下发
        @param print_cli 是否打印等效指令
        """
        super().__init__(node, dry_run, print_cli)
        self._status: Optional[ArmStatus] = None
        self._joints: Dict[str, float] = {}
        self._control_mode: Optional[int] = None
        self._data_lock = threading.Lock()

        node.create_subscription(ArmStatus, ARM_TOPIC_STATUS, self._on_status, 10)
        node.create_subscription(JointState, ARM_TOPIC_JOINT_STATES, self._on_joint_states, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        node.create_subscription(ControlMode, ARM_TOPIC_CONTROL_MODE,
                                 self._on_control_mode, latched)

        self._act_move_to_pose = _Action(node, ArmMoveToPose, ARM_ACTION_MOVE_TO_POSE)
        self._act_move_to_joint = _Action(node, ArmMoveToJoint, ARM_ACTION_MOVE_TO_JOINT)
        self._act_shot = _Action(node, ArmTrajectoryShot, ARM_ACTION_TRAJECTORY_SHOT)
        self._act_track = _Action(node, ArmTrackTarget, ARM_ACTION_TRACK_TARGET)

        self._srv_stop = node.create_client(ArmStop, ARM_SRV_STOP)
        self._srv_enable = node.create_client(ArmEnable, ARM_SRV_ENABLE)
        self._srv_homing = node.create_client(ArmHoming, ARM_SRV_HOMING)
        self._srv_reset_error = node.create_client(ArmResetError, ARM_SRV_RESET_ERROR)
        self._srv_switch_mode = node.create_client(SwitchControlMode, ARM_SRV_SWITCH_MODE)

        self._pub_follow = node.create_publisher(ArmFollowCommand, ARM_TOPIC_FOLLOW_COMMAND, 10)
        self._pub_joint_vel = node.create_publisher(ArmJointVelocityCommand,
                                                    ARM_TOPIC_JOINT_VELOCITY, 10)
        self._track_goal_handle = None

    # ── 订阅回调 ────────────────────────────────────────────────────────────
    def _on_status(self, msg: ArmStatus) -> None:
        """@brief 缓存最新 ArmStatus（10Hz）。
        @param msg ArmStatus
        """
        with self._data_lock:
            self._status = msg

    def _on_joint_states(self, msg: JointState) -> None:
        """@brief 缓存本臂 6 轴关节角（忽略 /joint_states 里的其他关节）。
        @param msg JointState
        """
        with self._data_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in ARM_JOINT_NAMES:
                    self._joints[name] = float(pos)

    def _on_control_mode(self, msg: ControlMode) -> None:
        """@brief 缓存 ModeManager 广播的当前语义控制模式。
        @param msg ControlMode
        """
        with self._data_lock:
            self._control_mode = int(msg.mode)

    # ── 状态查询 ────────────────────────────────────────────────────────────
    def wait_ready(self, timeout_sec: float = 10.0) -> bool:
        """@brief 等 Commander 的 action server 上线并收到首帧 ArmStatus。
        @param timeout_sec 超时
        @return True = 就绪；dry-run 恒为 True
        """
        if self.dry_run:
            # dry-run 不要求 server 在线，但若 ArmStatus 正在广播就等它最多 3s：
            # 这样相对运动打印出的等效指令带的是真实起点，而不是零位姿
            deadline = time.monotonic() + min(timeout_sec, 3.0)   # 实机 DDS 发现偶尔要 2s 以上
            while time.monotonic() < deadline and self.get_status() is None:
                time.sleep(0.05)
            return True
        if not self._act_move_to_pose.client.wait_for_server(timeout_sec=timeout_sec):
            self._log.error(f'{ARM_ACTION_MOVE_TO_POSE} 不可用：arm_commander_node 未启动？'
                            '（实机：ros2 launch robot_arm_bringup real.launch.py '
                            'controller:=commander gui:=false）')
            return False
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.get_status() is not None:
                return True
            time.sleep(0.05)
        self._log.error(f'{timeout_sec}s 内未收到 {ARM_TOPIC_STATUS}')
        return False

    def get_status(self) -> Optional[ArmStatus]:
        """@brief 最近一帧 ArmStatus。
        @return ArmStatus；尚未收到时 None
        """
        with self._data_lock:
            return self._status

    def get_pose(self) -> Optional[ArmPose]:
        """@brief 末端当前位姿（ArmStatus.arm_pose 的副本，含云台在内的真实末端）。
        @return ArmPose；尚未收到状态时 None
        """
        status = self.get_status()
        if status is None:
            return None
        p = status.arm_pose
        return make_pose(p.x, p.y, p.z, p.roll, p.pitch, p.yaw)

    def get_joints(self) -> Dict[str, float]:
        """@brief 当前 6 轴关节角（rad），来自 /joint_states。
        @return {'Joint1': rad, ..., 'Joint6': rad}（收到多少给多少）
        """
        with self._data_lock:
            return dict(self._joints)

    def get_control_mode(self) -> Optional[int]:
        """@brief 当前语义控制模式。
        @return ControlMode.TRAJECTORY / JOINT_VELOCITY …；未知时 None
        """
        with self._data_lock:
            if self._control_mode is not None:
                return self._control_mode
            return int(self._status.active_control_mode) if self._status else None

    def is_moving(self) -> bool:
        """@brief 机械臂是否在运动（ArmStatus.is_moving）。
        @return bool
        """
        status = self.get_status()
        return bool(status and status.is_moving)

    def has_error(self) -> bool:
        """@brief 是否有未清除的错误码（ERR_LIMIT / ERR_DRIVER / ERR_TIMEOUT）。
        @return bool
        """
        status = self.get_status()
        return bool(status and status.error_code != ArmStatus.ERR_NONE)

    def is_idle(self) -> bool:
        """@brief 是否空闲：不在运动且没有命令在执行。

        注意 STOPPED（急停后）也会显示空闲，但会拒收新 goal —— 收到 'goal 被拒绝' 就 reset_error。
        @return bool
        """
        status = self.get_status()
        if status is None:
            return False
        return (not status.is_moving
                and status.command_result != ArmStatus.RESULT_EXECUTING)

    def wait_until_idle(self, timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC) -> bool:
        """@brief 轮询等待到空闲。
        @param timeout_sec 超时
        @return True = 已空闲
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.is_idle():
                return True
            time.sleep(0.05)
        return False

    def wait_camera_ready(self, timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC) -> bool:
        """@brief 等运镜到达起拍点：ArmStatus.camera_ready 变为 true（录像应在此刻开始）。

        camera_ready 从到达运镜起点起、到整条轨迹结束为 true（成功 / 取消 / 异常都会复位）。
        @param timeout_sec 超时
        @return True = 已就绪
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            status = self.get_status()
            if status is not None and status.camera_ready:
                return True
            time.sleep(0.02)
        return False

    def status_summary(self) -> str:
        """@brief 一行状态摘要（demo 的 `status` 子命令用）。
        @return 字符串
        """
        status = self.get_status()
        if status is None:
            return f'ArmStatus: 未收到（{ARM_TOPIC_STATUS}）'
        mode = self.get_control_mode()
        joints = self.get_joints()
        joint_txt = ' '.join(f'{n[-1]}={joints[n]:+.3f}' for n in ARM_JOINT_NAMES if n in joints)
        return (f'pose_state={POSE_STATE_NAMES.get(status.current_pose_state, "?")} '
                f'mode={MODE_NAMES.get(mode, mode)} '
                f'error={ARM_ERROR_NAMES.get(status.error_code, status.error_code)} '
                f'result={ARM_RESULT_NAMES.get(status.command_result, "?")} '
                f'moving={status.is_moving} at_target={status.arm_at_target} '
                f'camera_ready={status.camera_ready} tracking={status.is_tracking}\n'
                f'  末端 {pose_to_str(status.arm_pose)}\n'
                f'  关节(rad) {joint_txt or "未收到 /joint_states"}')

    # ── 运维 service ────────────────────────────────────────────────────────
    def enable(self, on: bool = True) -> CallResult:
        """@brief 伺服上电 / 下电（Commander 透传驱动层）。
        @param on True = 上电使能，False = 下电
        @return CallResult
        """
        request = ArmEnable.Request()
        request.enable = bool(on)
        return self._service_result(self._srv_enable, request, timeout_sec=15.0)

    def disable(self) -> CallResult:
        """@brief 伺服下电（= enable(False)）。
        @return CallResult
        """
        return self.enable(False)

    def homing(self, timeout_sec: float = DEFAULT_HOMING_TIMEOUT_SEC) -> CallResult:
        """@brief 回零（阻塞，驱动层回零结束后才应答）。
        @param timeout_sec 超时
        @return CallResult
        """
        return self._service_result(self._srv_homing, ArmHoming.Request(), timeout_sec)

    def reset_error(self) -> CallResult:
        """@brief 清除驱动层故障，并把 Commander 从 ERROR / STOPPED 复位回 IDLE（速度流随之解禁）。
        @return CallResult（result.cleared_error_code = 清除前的错误码）
        """
        return self._service_result(self._srv_reset_error, ArmResetError.Request(), 10.0)

    def stop(self) -> CallResult:
        """@brief 软件急停：中止当前 action（MOVING → STOPPED）并停掉速度流，停后保持当前位置。

        之后新 goal 会被拒收，需 reset_error() 解除。非运动状态下调用是空操作。
        @return CallResult
        """
        return self._service_result(self._srv_stop, ArmStop.Request(), 3.0)

    def switch_control_mode(self, mode: int) -> CallResult:
        """@brief 切换语义控制模式（唯一入口 ModeManager）。

        默认后端下 TRAJECTORY ↔ JOINT_VELOCITY 只更新语义标志、不切控制器，机械臂原地不动，
        随时可切。JOINT_EFFORT / ADMITTANCE 在实机会被拒绝。
        @param mode ControlMode.TRAJECTORY(0) / JOINT_VELOCITY(1) …
        @return CallResult（result.active_mode = 切换后实际模式）
        """
        request = SwitchControlMode.Request()
        request.target_mode = int(mode)
        result = self._service_result(self._srv_switch_mode, request)
        if result and not self.dry_run:
            with self._data_lock:
                self._control_mode = int(result.result.active_mode)
        return result

    def enter_velocity_mode(self) -> CallResult:
        """@brief 切到 JOINT_VELOCITY（速度流的模式闸；不切则速度指令被忽略并告警）。
        @return CallResult
        """
        return self.switch_control_mode(ControlMode.JOINT_VELOCITY)

    def exit_velocity_mode(self) -> CallResult:
        """@brief 切回 TRAJECTORY（用完速度流后调用）。
        @return CallResult
        """
        return self.switch_control_mode(ControlMode.TRAJECTORY)

    # ── 位置类 action ───────────────────────────────────────────────────────
    def _move_to_pose_goal(self, pose_state: int, pose: Optional[ArmPose], speed: Any,
                           return_to_start: bool, timeout_sec: float) -> CallResult:
        """@brief ArmMoveToPose 的公共发送逻辑。
        @param pose_state      POSE_STATE_STOWED / OBSERVE / SHOOTING
        @param pose            SHOOTING 时的绝对位姿；其余可 None
        @param speed           档位
        @param return_to_start 到位后自动原路返回
        @param timeout_sec     超时
        @return CallResult（result.actual_pose = 实际末端位姿）
        """
        goal = ArmMoveToPose.Goal()
        goal.target_pose_state = int(pose_state)
        goal.transition_speed = speed_code(speed)
        goal.return_to_start = bool(return_to_start)
        if pose is not None:
            goal.target_pose = pose
        return self._send_goal(self._act_move_to_pose, goal, timeout_sec)

    def move_to_stowed(self, speed: Any = 'normal', return_to_start: bool = False,
                       timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC) -> CallResult:
        """@brief 回收纳位：关节空间回零（6 轴含云台全部归零，不走 IK）。
        @param speed           slow / normal / fast
        @param return_to_start 到位后自动返回出发位姿
        @param timeout_sec     超时
        @return CallResult
        """
        return self._move_to_pose_goal(ArmMoveToPose.Goal.POSE_STATE_STOWED, None, speed,
                                       return_to_start, timeout_sec)

    def move_to_observe(self, speed: Any = 'normal', return_to_start: bool = False,
                        timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC) -> CallResult:
        """@brief 去观察位（Commander 内置预设，可由 pose_observe_* 参数覆盖）。
        @param speed           slow / normal / fast
        @param return_to_start 到位后自动返回出发位姿
        @param timeout_sec     超时
        @return CallResult
        """
        return self._move_to_pose_goal(ArmMoveToPose.Goal.POSE_STATE_OBSERVE, None, speed,
                                       return_to_start, timeout_sec)

    def move_to_pose(self, pose: ArmPose, speed: Any = 'normal', return_to_start: bool = False,
                     timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC) -> CallResult:
        """@brief 到绝对末端位姿（拍摄位，IK 求解后 6 轴一起动）。

        IK 无解时 exit_reason='unreachable'，Commander 回 IDLE 不进 ERROR。
        @param pose            目标位姿（make_pose 构造，base_link 系，米 / 度）
        @param speed           slow / normal / fast（末端线速度约 0.02 / 0.05 / 0.10 m/s）
        @param return_to_start 到位后自动原路返回
        @param timeout_sec     超时
        @return CallResult
        """
        return self._move_to_pose_goal(ArmMoveToPose.Goal.POSE_STATE_SHOOTING, pose, speed,
                                       return_to_start, timeout_sec)

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                      droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0,
                      speed: Any = 'normal', timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC
                      ) -> CallResult:
        """@brief 相对当前末端位姿平移 / 转动（基于最近一帧 ArmStatus.arm_pose）。
        @param dx,dy,dz          位置增量，米（base_link 系：+x 前 / +y 左 / +z 上）
        @param droll,dpitch,dyaw 姿态增量，度
        @param speed             档位
        @param timeout_sec       超时
        @return CallResult
        """
        base = self._current_pose_or_zero()
        return self.move_to_pose(offset_pose(base, dx, dy, dz, droll, dpitch, dyaw),
                                 speed, False, timeout_sec)

    def move_to_joint(self, j1: float, j2: float, j3: float, speed: Any = 'normal',
                      relative: bool = False, duration_sec: float = 0.0,
                      timeout_sec: float = DEFAULT_ACTION_TIMEOUT_SEC) -> CallResult:
        """@brief 关节空间点到点（只动臂 J1-3、不过 IK，云台 J4-6 保持不动）。

        限位 / 自碰撞校验不过时不下发任何指令：exit_reason = out_of_range / collision。
        @param j1,j2,j3     目标关节角 rad（relative=True 时为增量）
        @param speed        档位（关节角速度 0.3 / 0.6 / 1.2 rad/s，duration_sec<=0 时生效）
        @param relative     True = 相对当前关节角
        @param duration_sec >0 直接指定运动时长（覆盖档位），夹在 [0.5, 30]s
        @param timeout_sec  超时
        @return CallResult（result.actual_joints = 实际到达的 J1-3）
        """
        goal = ArmMoveToJoint.Goal()
        goal.target_joints = [float(j1), float(j2), float(j3)]
        goal.transition_speed = speed_code(speed)
        goal.relative = bool(relative)
        goal.duration_sec = float(duration_sec)
        return self._send_goal(self._act_move_to_joint, goal, timeout_sec)

    # ── 运镜 action ─────────────────────────────────────────────────────────
    def _run_shot(self, goal: Any, timeout_sec: float,
                  on_camera_ready: Optional[Callable[[], None]]) -> CallResult:
        """@brief 发送 ArmTrajectoryShot，并可在 camera_ready 上升沿触发回调（开录像）。
        @param goal            ArmTrajectoryShot.Goal
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点时调用一次（None = 不监听）
        @return CallResult
        """
        stop_event = threading.Event()
        watcher = None
        if on_camera_ready is not None and not self.dry_run:
            def _watch() -> None:
                """@brief 后台轮询 camera_ready，上升沿调一次 on_camera_ready 后退出。"""
                while not stop_event.is_set():
                    status = self.get_status()
                    if status is not None and status.camera_ready:
                        try:
                            on_camera_ready()
                        finally:
                            return
                    time.sleep(0.02)
            watcher = threading.Thread(target=_watch, daemon=True, name='camera_ready_watch')
            watcher.start()
        try:
            return self._send_goal(self._act_shot, goal, timeout_sec)
        finally:
            stop_event.set()
            if watcher is not None:
                watcher.join(timeout=0.5)

    def shot_linear(self, start: ArmPose, end: ArmPose, speed: Any = 'normal',
                    return_to_start: bool = False, timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
                    on_camera_ready: Optional[Callable[[], None]] = None) -> CallResult:
        """@brief 直线运镜：先 PTP 到 start，起点停顿 1s（camera_ready=true），再直线到 end。
        @param start,end       两端绝对末端位姿（base_link 系）
        @param speed           档位
        @param return_to_start 结束后原路返回 start
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点的回调（开始录像）
        @return CallResult
        """
        goal = ArmTrajectoryShot.Goal()
        goal.motion_type = ArmTrajectoryShot.Goal.MOTION_LINEAR
        goal.transition_speed = speed_code(speed)
        goal.return_to_start = bool(return_to_start)
        goal.linear_start_pose = start
        goal.linear_end_pose = end
        return self._run_shot(goal, timeout_sec, on_camera_ready)

    def shot_linear_from_current(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                                 droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0,
                                 speed: Any = 'normal', return_to_start: bool = False,
                                 timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
                                 on_camera_ready: Optional[Callable[[], None]] = None
                                 ) -> CallResult:
        """@brief 从当前末端位姿出发的直线运镜（start = 当前位姿，end = 当前 + 增量）。
        @param dx,dy,dz          位移，米
        @param droll,dpitch,dyaw 姿态增量，度
        @param speed             档位
        @param return_to_start   结束后返回起点
        @param timeout_sec       超时
        @param on_camera_ready   到达起拍点回调
        @return CallResult
        """
        start = self._current_pose_or_zero()
        end = offset_pose(start, dx, dy, dz, droll, dpitch, dyaw)
        return self.shot_linear(start, end, speed, return_to_start, timeout_sec, on_camera_ready)

    def dolly(self, distance_m: float, speed: Any = 'normal', return_to_start: bool = False,
              timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
              on_camera_ready: Optional[Callable[[], None]] = None) -> CallResult:
        """@brief 推 / 拉镜：沿 base_link +x 直线运镜（正 = 推进靠近主体，负 = 拉远）。
        @param distance_m      位移，米
        @param speed           档位
        @param return_to_start 结束后返回起点
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点回调
        @return CallResult
        """
        return self.shot_linear_from_current(dx=distance_m, speed=speed,
                                             return_to_start=return_to_start,
                                             timeout_sec=timeout_sec,
                                             on_camera_ready=on_camera_ready)

    def truck(self, distance_m: float, speed: Any = 'normal', return_to_start: bool = False,
              timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
              on_camera_ready: Optional[Callable[[], None]] = None) -> CallResult:
        """@brief 横移镜：沿 base_link +y 直线运镜（正 = 向机械臂左侧，负 = 右侧）。
        @param distance_m      位移，米
        @param speed           档位
        @param return_to_start 结束后返回起点
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点回调
        @return CallResult
        """
        return self.shot_linear_from_current(dy=distance_m, speed=speed,
                                             return_to_start=return_to_start,
                                             timeout_sec=timeout_sec,
                                             on_camera_ready=on_camera_ready)

    def crane(self, distance_m: float, speed: Any = 'normal', return_to_start: bool = False,
              timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
              on_camera_ready: Optional[Callable[[], None]] = None) -> CallResult:
        """@brief 升 / 降镜：沿 base_link +z 直线运镜（正 = 上升，负 = 下降）。
        @param distance_m      位移，米
        @param speed           档位
        @param return_to_start 结束后返回起点
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点回调
        @return CallResult
        """
        return self.shot_linear_from_current(dz=distance_m, speed=speed,
                                             return_to_start=return_to_start,
                                             timeout_sec=timeout_sec,
                                             on_camera_ready=on_camera_ready)

    def shot_orbit(self, center: Sequence[float], az_start_deg: float, az_end_deg: float,
                   el_start_deg: float, el_end_deg: float, r_start_m: float, r_end_m: float,
                   speed: Any = 'normal', return_to_start: bool = False,
                   timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
                   on_camera_ready: Optional[Callable[[], None]] = None) -> CallResult:
        """@brief 球面环绕运镜：以 center 为球心，相机始终朝向球心，从起始球坐标运动到终止球坐标。

        球坐标约定（ArmTrajectoryShot.action）：方位角 0° = 近侧，正值顺时针；俯仰角正值向上，
        范围 (-90°, 90°)；半径米。起止点任一 IK 无解 → exit_reason='unreachable'。
        @param center          球心 (x, y, z)，base_link 系，米（被摄主体位置）
        @param az_start_deg    起始方位角
        @param az_end_deg      终止方位角
        @param el_start_deg    起始俯仰角
        @param el_end_deg      终止俯仰角
        @param r_start_m       起始半径
        @param r_end_m         终止半径（≠ r_start 即边环绕边推拉）
        @param speed           档位
        @param return_to_start 结束后原路返回起始球坐标
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点回调
        @return CallResult
        """
        goal = ArmTrajectoryShot.Goal()
        goal.motion_type = ArmTrajectoryShot.Goal.MOTION_ORBIT
        goal.transition_speed = speed_code(speed)
        goal.return_to_start = bool(return_to_start)
        goal.orbit_center_x, goal.orbit_center_y, goal.orbit_center_z = (
            float(center[0]), float(center[1]), float(center[2]))
        goal.azimuth_start_deg = float(az_start_deg)
        goal.azimuth_end_deg = float(az_end_deg)
        goal.elevation_start_deg = float(el_start_deg)
        goal.elevation_end_deg = float(el_end_deg)
        goal.radius_start_m = float(r_start_m)
        goal.radius_end_m = float(r_end_m)
        return self._run_shot(goal, timeout_sec, on_camera_ready)

    def arc_around(self, center: Sequence[float], radius_m: float, az_start_deg: float,
                   az_end_deg: float, elevation_deg: float = 0.0, speed: Any = 'normal',
                   return_to_start: bool = False, timeout_sec: float = DEFAULT_SHOT_TIMEOUT_SEC,
                   on_camera_ready: Optional[Callable[[], None]] = None) -> CallResult:
        """@brief 等半径、等仰角的水平环绕（shot_orbit 的常用简写）。
        @param center          球心 (x, y, z)，米
        @param radius_m        环绕半径
        @param az_start_deg    起始方位角
        @param az_end_deg      终止方位角
        @param elevation_deg   俯仰角（全程不变）
        @param speed           档位
        @param return_to_start 结束后返回
        @param timeout_sec     超时
        @param on_camera_ready 到达起拍点回调
        @return CallResult
        """
        return self.shot_orbit(center, az_start_deg, az_end_deg, elevation_deg, elevation_deg,
                               radius_m, radius_m, speed, return_to_start, timeout_sec,
                               on_camera_ready)

    # ── 视觉跟随 action ─────────────────────────────────────────────────────
    def track_target_start(self, desired_depth_m: float = 0.0, desired_x: float = 0.0,
                           desired_y: float = 0.0, constrain_height: bool = False,
                           desired_height_m: float = 0.0, hold_on_converge: bool = True,
                           total_timeout_sec: float = 0.0) -> CallResult:
        """@brief 启动 IBVS 视觉跟随（非阻塞：goal 被接受即返回；跟随期间 ArmStatus.is_tracking=true）。

        特征丢失超 0.5s / 达到 total_timeout_sec 会自动退出。
        @param desired_depth_m   期望保持距离，0 = 节点默认
        @param desired_x         期望目标在图像的 x（归一化，0 = 画面中心）
        @param desired_y         期望目标在图像的 y
        @param constrain_height  是否锁定相机高度
        @param desired_height_m  锁定高度（arm_base 系 Z）
        @param hold_on_converge  True = 收敛后继续跟随等 stop；False = 收敛即结束
        @param total_timeout_sec 总超时，0 = 永不超时
        @return CallResult（result = goal handle）
        """
        goal = ArmTrackTarget.Goal()
        goal.desired_depth = float(desired_depth_m)
        goal.desired_x = float(desired_x)
        goal.desired_y = float(desired_y)
        goal.constrain_height = bool(constrain_height)
        goal.desired_height = float(desired_height_m)
        goal.hold_on_converge = bool(hold_on_converge)
        goal.total_timeout_sec = float(total_timeout_sec)
        result = self._send_goal(self._act_track, goal, 0.0, wait_result=False)
        if result and not self.dry_run:
            self._track_goal_handle = result.result
        return result

    def track_target_stop(self) -> CallResult:
        """@brief 停止视觉跟随（cancel goal，机械臂保持当前位置）。
        @return CallResult（result = ArmTrackTarget.Result，exit_reason='cancelled'）
        """
        if self.dry_run:
            self._log.info(f'等效指令: ros2 action send_goal 的 cancel（{ARM_ACTION_TRACK_TARGET}）'
                           ' —— 请在发 goal 的终端 Ctrl-C')
            return CallResult(True, 'dry-run')
        goal_handle = self._track_goal_handle
        self._track_goal_handle = None
        if goal_handle is None:
            return CallResult(False, '没有活跃的跟随 goal')
        self._wait_future(goal_handle.cancel_goal_async(), 3.0)
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, 5.0):
            return CallResult(False, '取消后等结果超时')
        result = result_future.result().result
        return CallResult(True, str(result.exit_reason), result)

    # ── 速度流 topic ────────────────────────────────────────────────────────
    def publish_cartesian_velocity(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                                   wroll: float = 0.0, wpitch: float = 0.0, wyaw: float = 0.0,
                                   show_cli: bool = False) -> None:
        """@brief 发一帧末端 6 维 twist（需 JOINT_VELOCITY 模式；必须 ≥3Hz 持续发，停发 0.3s 即停流）。

        线速度由臂 J1-3 承担（默认限幅 0.2 m/s），角速度**度/秒**、绕 base 系 X/Y/Z，由云台
        J4-6 承担；Commander 用 6×6 Jacobian 一次解算。全 0 帧 = 原地停住。
        @param vx,vy,vz          线速度 m/s（前 / 左 / 上）
        @param wroll,wpitch,wyaw 角速度 deg/s
        @param show_cli          是否打印等效指令（流式发送时只在首帧打印）
        """
        msg = ArmFollowCommand()
        msg.twist = ArmTwist(vx=float(vx), vy=float(vy), vz=float(vz),
                             wroll=float(wroll), wpitch=float(wpitch), wyaw=float(wyaw))
        if show_cli:
            self._show_cli(f'ros2 topic pub -r {VELOCITY_STREAM_RATE_HZ:.0f} '
                           f'{ARM_TOPIC_FOLLOW_COMMAND} {ros_type_name(ArmFollowCommand)} '
                           f'"{msg_to_yaml(msg)}"')
        if not self.dry_run:
            self._pub_follow.publish(msg)

    def publish_joint_velocity(self, v1: float = 0.0, v2: float = 0.0, v3: float = 0.0,
                               show_cli: bool = False) -> None:
        """@brief 发一帧 J1-3 关节角速度（需 JOINT_VELOCITY 模式；≥3Hz 持续发；云台保持不动）。
        @param v1,v2,v3 rad/s，正方向与 URDF 关节轴一致；逐轴限幅 max_joint_speed（默认 1.0）
        @param show_cli 是否打印等效指令
        """
        msg = ArmJointVelocityCommand()
        msg.velocities = [float(v1), float(v2), float(v3)]
        if show_cli:
            self._show_cli(f'ros2 topic pub -r {VELOCITY_STREAM_RATE_HZ:.0f} '
                           f'{ARM_TOPIC_JOINT_VELOCITY} {ros_type_name(ArmJointVelocityCommand)} '
                           f'"{msg_to_yaml(msg)}"')
        if not self.dry_run:
            self._pub_joint_vel.publish(msg)

    def jog_cartesian(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                      wroll: float = 0.0, wpitch: float = 0.0, wyaw: float = 0.0,
                      duration_sec: float = 1.0, auto_mode: bool = True,
                      rate_hz: float = VELOCITY_STREAM_RATE_HZ) -> CallResult:
        """@brief 末端速度点动：按 rate_hz 持续发 duration_sec，结束补一帧全 0（原地停住）。
        @param vx,vy,vz          线速度 m/s
        @param wroll,wpitch,wyaw 角速度 deg/s
        @param duration_sec      持续时长
        @param auto_mode         True = 自动切 JOINT_VELOCITY，结束后切回 TRAJECTORY
        @param rate_hz           发布频率（建议 50）
        @return CallResult
        """
        if auto_mode:
            switched = self.enter_velocity_mode()
            if not switched:
                return switched
        self.publish_cartesian_velocity(vx, vy, vz, wroll, wpitch, wyaw, show_cli=True)
        if not self.dry_run:
            self._stream(lambda: self.publish_cartesian_velocity(vx, vy, vz, wroll, wpitch, wyaw),
                         duration_sec, rate_hz)
            self.publish_cartesian_velocity()          # 停帧
            time.sleep(VELOCITY_STREAM_SETTLE_SEC)      # 等速度流收干净（见常量注释）
        if auto_mode:
            self.exit_velocity_mode()
        return CallResult(True, f'jog {duration_sec:.2f}s')

    def jog_joint(self, v1: float = 0.0, v2: float = 0.0, v3: float = 0.0,
                  duration_sec: float = 1.0, auto_mode: bool = True,
                  rate_hz: float = VELOCITY_STREAM_RATE_HZ) -> CallResult:
        """@brief 关节速度点动：按 rate_hz 持续发 duration_sec，结束补一帧全 0。
        @param v1,v2,v3     rad/s
        @param duration_sec 持续时长
        @param auto_mode    True = 自动切 JOINT_VELOCITY，结束后切回 TRAJECTORY
        @param rate_hz      发布频率
        @return CallResult
        """
        if auto_mode:
            switched = self.enter_velocity_mode()
            if not switched:
                return switched
        self.publish_joint_velocity(v1, v2, v3, show_cli=True)
        if not self.dry_run:
            self._stream(lambda: self.publish_joint_velocity(v1, v2, v3), duration_sec, rate_hz)
            self.publish_joint_velocity()              # 停帧
            time.sleep(VELOCITY_STREAM_SETTLE_SEC)      # 等速度流收干净（见常量注释）
        if auto_mode:
            self.exit_velocity_mode()
        return CallResult(True, f'jog {duration_sec:.2f}s')

    # ── 内部 ────────────────────────────────────────────────────────────────
    def _current_pose_or_zero(self) -> ArmPose:
        """@brief 取当前末端位姿；没收到状态时（如 dry-run）退化为零位姿并告警。
        @return ArmPose
        """
        pose = self.get_pose()
        if pose is None:
            self._log.warning('尚未收到 ArmStatus，相对运动以零位姿为基准（仅 dry-run 可接受）')
            return make_pose(0.0, 0.0, 0.0)
        return pose


# ══════════════════════════════════════════════════════════════════════════════
#  云台 V2 直连
# ══════════════════════════════════════════════════════════════════════════════
class GimbalV2Client(_RosClientBase):
    """@brief 云台 V2（三轴 GCU，执行节点 robot_gimbal_node_v2 跑在云台板端）的直连客户端。

    单位 / 坐标：对外一律 rad、rad/s，URDF 关节系：pan = Joint4(Yaw)、roll = Joint5、
    tilt = Joint6(Pitch)。

    与机械臂的关系：云台就是机械臂的 J4-6。臂侧 arm_controller 经 /robot_gimbal_v2/forward_cmd
    驱动它（只在指令变化 >0.01rad 时发）；板端仲裁优先级 FREEZE > 显式位置（gimbal_cmd POSITION /
    action）> 转发流 > 速度流。所以：用本类直接转云台后，下一次机械臂位置类动作会把云台带回
    轨迹规划的角度；要让云台完全听本类的，先 set_forward_cmd_enable(False)，用完再打开。
    """

    def __init__(self, node: Node, dry_run: bool = False, print_cli: bool = True):
        """@brief 建立云台端点。
        @param node      共用节点
        @param dry_run   只打印不下发
        @param print_cli 是否打印等效指令
        @throws RuntimeError robot_gimbal_interfaces_v2 未编译
        """
        if not HAS_GIMBAL_IFACE:
            raise RuntimeError('robot_gimbal_interfaces_v2 未编译/未 source，GimbalV2Client 不可用')
        super().__init__(node, dry_run, print_cli)
        self._status: Optional[Any] = None
        self._angles: Dict[str, float] = {}
        self._data_lock = threading.Lock()
        node.create_subscription(GimbalStatus, GIMBAL_TOPIC_STATUS, self._on_status, 10)
        node.create_subscription(JointState, GIMBAL_TOPIC_JOINT_STATES_RAW,
                                 self._on_joint_states_raw, 10)
        self._act_rotate = _Action(node, RotateToAngle, GIMBAL_ACTION_ROTATE)
        self._srv_forward_enable = node.create_client(SetForwardCmdEnable,
                                                     GIMBAL_SRV_FORWARD_ENABLE)
        self._pub_cmd = node.create_publisher(GimbalCommand, GIMBAL_TOPIC_CMD, 10)
        self._pub_cmd_vel = node.create_publisher(Twist, GIMBAL_TOPIC_CMD_VEL, 10)

    def _on_status(self, msg: Any) -> None:
        """@brief 缓存 GimbalStatus。
        @param msg GimbalStatus
        """
        with self._data_lock:
            self._status = msg

    def _on_joint_states_raw(self, msg: JointState) -> None:
        """@brief 缓存板端真实回读的三轴角（rad）。
        @param msg JointState（name 为 Joint4/5/6）
        """
        with self._data_lock:
            for name, pos in zip(msg.name, msg.position):
                self._angles[name] = float(pos)

    def wait_ready(self, timeout_sec: float = 5.0) -> bool:
        """@brief 等云台 action server 上线。
        @param timeout_sec 超时
        @return True = 就绪
        """
        if self.dry_run:
            return True
        ok = self._act_rotate.client.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            self._log.error(f'{GIMBAL_ACTION_ROTATE} 不可用：云台板端 robot_gimbal_node_v2 未启动，'
                            '或两机 ROS_DOMAIN_ID 不同 / 设了 ROS_LOCALHOST_ONLY=1')
        return ok

    def get_status(self) -> Optional[Any]:
        """@brief 最近一帧 GimbalStatus（含 gbc_stat / tca_ready / has_hw_fault 等）。
        @return GimbalStatus 或 None
        """
        with self._data_lock:
            return self._status

    def get_angles(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """@brief 当前 (pan, roll, tilt) rad —— 优先 GimbalStatus，其次 joint_states_raw。
        @return 三元组；未知轴为 None
        """
        status = self.get_status()
        if status is not None:
            return float(status.pan), float(status.roll), float(status.tilt)
        with self._data_lock:
            return (self._angles.get('Joint4'), self._angles.get('Joint5'),
                    self._angles.get('Joint6'))

    def get_angles_deg(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """@brief 当前 (pan, roll, tilt) 度。
        @return 三元组；未知轴为 None
        """
        return tuple(None if a is None else math.degrees(a) for a in self.get_angles())

    def rotate_to(self, pan: Optional[float] = None, roll: Optional[float] = None,
                  tilt: Optional[float] = None, timeout_sec: float = 5.0,
                  wait: bool = True) -> CallResult:
        """@brief 转到指定角并等到位（RotateToAngle action，feedback 基于串口真实回读）。
        @param pan,roll,tilt rad；None = 该轴保持不动（接口里用 NaN 表达）
        @param timeout_sec   板端动作超时（0 = 板端默认 5s）；客户端再多等 5s
        @param wait          False = 接受即返回
        @return CallResult（result.actual_pan/roll/tilt）
        """
        goal = RotateToAngle.Goal()
        goal.target_pan = float('nan') if pan is None else float(pan)
        goal.target_roll = float('nan') if roll is None else float(roll)
        goal.target_tilt = float('nan') if tilt is None else float(tilt)
        goal.timeout_sec = float(timeout_sec)
        client_timeout = (timeout_sec if timeout_sec > 0 else 5.0) + 5.0
        return self._send_goal(self._act_rotate, goal, client_timeout, wait_result=wait)

    def rotate_to_deg(self, pan_deg: Optional[float] = None, roll_deg: Optional[float] = None,
                      tilt_deg: Optional[float] = None, timeout_sec: float = 5.0,
                      wait: bool = True) -> CallResult:
        """@brief rotate_to 的度制版本。
        @param pan_deg,roll_deg,tilt_deg 度；None = 不动
        @param timeout_sec 超时
        @param wait        是否等到位
        @return CallResult
        """
        conv = (lambda d: None if d is None else math.radians(d))
        return self.rotate_to(conv(pan_deg), conv(roll_deg), conv(tilt_deg), timeout_sec, wait)

    def _publish_cmd(self, mode: int, **fields: float) -> None:
        """@brief 发一条 GimbalCommand。
        @param mode   GimbalCommand.POSITION / FREEZE / START / STOP / GYRO_CALIB / GO_ZERO
        @param fields pan / roll / tilt / max_angular_vel 等字段
        """
        msg = GimbalCommand()
        msg.mode = int(mode)
        for key, value in fields.items():
            setattr(msg, key, float(value))
        self._show_cli(f'ros2 topic pub --once {GIMBAL_TOPIC_CMD} {ros_type_name(GimbalCommand)} '
                       f'"{msg_to_yaml(msg)}"')
        if not self.dry_run:
            self._pub_cmd.publish(msg)

    def set_position_stream(self, pan: float, roll: float, tilt: float,
                            max_vel: float = 0.0) -> None:
        """@brief 流式位置指令：后到覆盖先到，不等到位、无结果（要到位反馈用 rotate_to）。
        @param pan,roll,tilt rad
        @param max_vel       最大角速度 rad/s，0 = 驱动层默认
        """
        self._publish_cmd(GimbalCommand.POSITION, pan=pan, roll=roll, tilt=tilt,
                          max_angular_vel=max_vel)

    def publish_velocity(self, pan_vel: float = 0.0, roll_vel: float = 0.0,
                         tilt_vel: float = 0.0, show_cli: bool = False) -> None:
        """@brief 发一帧速度（/robot_gimbal_v2/cmd_vel，Twist：angular.z=pan, .x=roll, .y=tilt）。

        板端看门狗 0.3s，须持续发；全 0 或停发即停。
        @param pan_vel,roll_vel,tilt_vel rad/s
        @param show_cli 是否打印等效指令
        """
        msg = Twist()
        msg.angular.z = float(pan_vel)
        msg.angular.x = float(roll_vel)
        msg.angular.y = float(tilt_vel)
        if show_cli:
            self._show_cli(f'ros2 topic pub -r {GIMBAL_STREAM_RATE_HZ:.0f} {GIMBAL_TOPIC_CMD_VEL} '
                           f'{ros_type_name(Twist)} "{msg_to_yaml(msg)}"')
        if not self.dry_run:
            self._pub_cmd_vel.publish(msg)

    def jog(self, pan_vel: float = 0.0, roll_vel: float = 0.0, tilt_vel: float = 0.0,
            duration_sec: float = 1.0, rate_hz: float = GIMBAL_STREAM_RATE_HZ) -> CallResult:
        """@brief 速度点动：持续发 duration_sec，结束补一帧全 0。
        @param pan_vel,roll_vel,tilt_vel rad/s
        @param duration_sec 持续时长
        @param rate_hz      发布频率
        @return CallResult
        """
        self.publish_velocity(pan_vel, roll_vel, tilt_vel, show_cli=True)
        if not self.dry_run:
            self._stream(lambda: self.publish_velocity(pan_vel, roll_vel, tilt_vel),
                         duration_sec, rate_hz)
            self.publish_velocity()
        return CallResult(True, f'jog {duration_sec:.2f}s')

    def freeze(self) -> None:
        """@brief 冻结：锁定当前姿态，忽略速度流 / 转发流（显式 POSITION 或 action 解冻）。"""
        self._publish_cmd(GimbalCommand.FREEZE)

    def go_zero(self) -> None:
        """@brief 三轴回中（GCU go_zero；yaw 无绝对零参考，实际只回 roll/pitch）。"""
        self._publish_cmd(GimbalCommand.GO_ZERO)

    def start_motor(self) -> None:
        """@brief 启动云台电机（GCU cmd=2）。"""
        self._publish_cmd(GimbalCommand.START)

    def stop_motor(self) -> None:
        """@brief 停止云台电机（GCU cmd=3）；手动转云台前先发这个。"""
        self._publish_cmd(GimbalCommand.STOP)

    def gyro_calib(self) -> None:
        """@brief 陀螺仪校准（需 tca_ready 且云台静止，持续数秒）。"""
        self._publish_cmd(GimbalCommand.GYRO_CALIB)

    def set_forward_cmd_enable(self, enable: bool) -> CallResult:
        """@brief 开 / 关机械臂转发流（forward_cmd）对云台的控制权（默认开）。

        关掉后云台只听 gimbal_cmd / rotate_to_angle / cmd_vel；机械臂的位置类动作仍会规划
        6 轴，但 J4-6 不会被执行 —— 用完记得打开，否则臂侧 TF 与云台实物脱节。
        @param enable True = 允许转发流
        @return CallResult（result.was_enabled = 之前的状态）
        """
        request = SetForwardCmdEnable.Request()
        request.enable = bool(enable)
        return self._service_result(self._srv_forward_enable, request)


# ══════════════════════════════════════════════════════════════════════════════
#  门面
# ══════════════════════════════════════════════════════════════════════════════
class ArmApi:
    """@brief 机械臂 API 门面：一个 rclpy 节点 + 后台多线程执行器，挂 arm（必有）/ gimbal（可选）。

    用法：
        with ArmApi() as api:
            api.arm.wait_ready()
            api.arm.enable()
            api.arm.move_to_observe()
            api.arm.dolly(0.10, speed='slow')
    gimbal 在 robot_gimbal_interfaces_v2 未编译或 with_gimbal=False 时为 None。
    """

    def __init__(self, node_name: str = 'robot_arm_api', dry_run: bool = False,
                 print_cli: bool = True, with_gimbal: bool = True):
        """@brief 初始化 rclpy（如未初始化）、创建节点与客户端、启动后台执行器线程。
        @param node_name   节点名
        @param dry_run     只打印等效指令、不下发
        @param print_cli   是否打印等效指令
        @param with_gimbal 是否创建 GimbalV2Client
        """
        self._own_context = not rclpy.ok()
        if self._own_context:
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.arm = ArmCommanderClient(self.node, dry_run, print_cli)
        self.gimbal: Optional[GimbalV2Client] = None
        if with_gimbal:
            if HAS_GIMBAL_IFACE:
                self.gimbal = GimbalV2Client(self.node, dry_run, print_cli)
            else:
                self.node.get_logger().warning('robot_gimbal_interfaces_v2 不可用，云台直连接口关闭')
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True,
                                             name='robot_arm_api_spin')
        self._spin_thread.start()

    def shutdown(self) -> None:
        """@brief 停执行器、销毁节点；若 rclpy 是本对象初始化的则一并 shutdown。"""
        self._executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._own_context and rclpy.ok():
            rclpy.shutdown()

    def __enter__(self) -> 'ArmApi':
        """@brief 支持 with 语法。
        @return self
        """
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        """@brief 退出 with 时收尾。
        @param exc_type,exc,tb 异常信息（原样传播）
        """
        self.shutdown()


# ══════════════════════════════════════════════════════════════════════════════
#  运镜步骤表（大模型输出 → 接口调用）
# ══════════════════════════════════════════════════════════════════════════════
def _pose_from_dict(d: Dict[str, Any], fallback: Optional[ArmPose] = None) -> ArmPose:
    """@brief {x,y,z,roll,pitch,yaw} → ArmPose；缺的字段用 fallback（无则 0）补。
    @param d        字典
    @param fallback 基准位姿
    @return ArmPose
    """
    base = fallback or make_pose(0.0, 0.0, 0.0)
    return make_pose(d.get('x', base.x), d.get('y', base.y), d.get('z', base.z),
                     d.get('roll', base.roll), d.get('pitch', base.pitch),
                     d.get('yaw', base.yaw))


def run_plan_step(api: ArmApi, step: Dict[str, Any]) -> CallResult:
    """@brief 执行一条步骤：{'op': <名字>, ...参数}，参数名与对应方法形参一致（见 README schema）。

    op 一览：
      机械臂  enable / disable / homing / reset_error / stop / mode / stow / observe / pose /
              move_rel / joint / dolly / truck / crane / linear / orbit / arc /
              jog / jog_joint / track_start / track_stop
      云台    gimbal_rotate（度）/ gimbal_rotate_rad / gimbal_jog / gimbal_freeze / gimbal_go_zero /
              gimbal_start / gimbal_stop / gimbal_gyro_calib / gimbal_forward_enable
      其他    wait / wait_camera_ready
    @param api  ArmApi
    @param step 步骤字典
    @return CallResult
    """
    p = dict(step)
    op = str(p.pop('op', '')).strip().lower()
    arm = api.arm
    speed = p.get('speed', 'normal')
    rts = bool(p.get('return_to_start', False))

    # ── 机械臂 ──
    if op == 'enable':
        return arm.enable(bool(p.get('on', True)))
    if op == 'disable':
        return arm.disable()
    if op == 'homing':
        return arm.homing(float(p.get('timeout_sec', DEFAULT_HOMING_TIMEOUT_SEC)))
    if op == 'reset_error':
        return arm.reset_error()
    if op == 'stop':
        return arm.stop()
    if op == 'mode':
        mode = p.get('mode', 'trajectory')
        code = {'trajectory': 0, 'velocity': 1, 'joint_velocity': 1}.get(str(mode).lower(), mode)
        return arm.switch_control_mode(int(code))
    if op == 'stow':
        return arm.move_to_stowed(speed, rts)
    if op == 'observe':
        return arm.move_to_observe(speed, rts)
    if op == 'pose':
        return arm.move_to_pose(_pose_from_dict(p, arm.get_pose()), speed, rts)
    if op == 'move_rel':
        return arm.move_relative(p.get('dx', 0.0), p.get('dy', 0.0), p.get('dz', 0.0),
                                 p.get('droll', 0.0), p.get('dpitch', 0.0), p.get('dyaw', 0.0),
                                 speed)
    if op == 'joint':
        return arm.move_to_joint(p['j1'], p['j2'], p['j3'], speed, bool(p.get('relative', False)),
                                 float(p.get('duration_sec', 0.0)))
    if op in ('dolly', 'truck', 'crane'):
        return getattr(arm, op)(float(p['distance_m']), speed, rts)
    if op == 'linear':
        start = _pose_from_dict(p['start'], arm.get_pose()) if 'start' in p else None
        if start is None:
            return arm.shot_linear_from_current(
                p.get('dx', 0.0), p.get('dy', 0.0), p.get('dz', 0.0),
                p.get('droll', 0.0), p.get('dpitch', 0.0), p.get('dyaw', 0.0), speed, rts)
        end = _pose_from_dict(p['end'], start)
        return arm.shot_linear(start, end, speed, rts)
    if op in ('orbit', 'arc'):
        center = p['center']
        if op == 'arc' or 'radius_m' in p:
            return arm.arc_around(center, float(p['radius_m']), float(p['az_start_deg']),
                                  float(p['az_end_deg']), float(p.get('elevation_deg', 0.0)),
                                  speed, rts)
        return arm.shot_orbit(center, p['az_start_deg'], p['az_end_deg'],
                              p.get('el_start_deg', 0.0), p.get('el_end_deg', 0.0),
                              p['r_start_m'], p.get('r_end_m', p['r_start_m']), speed, rts)
    if op == 'jog':
        return arm.jog_cartesian(p.get('vx', 0.0), p.get('vy', 0.0), p.get('vz', 0.0),
                                 p.get('wroll', 0.0), p.get('wpitch', 0.0), p.get('wyaw', 0.0),
                                 float(p.get('duration_sec', 1.0)), bool(p.get('auto_mode', True)))
    if op == 'jog_joint':
        return arm.jog_joint(p.get('v1', 0.0), p.get('v2', 0.0), p.get('v3', 0.0),
                             float(p.get('duration_sec', 1.0)), bool(p.get('auto_mode', True)))
    if op == 'track_start':
        return arm.track_target_start(p.get('desired_depth_m', 0.0), p.get('desired_x', 0.0),
                                      p.get('desired_y', 0.0),
                                      bool(p.get('constrain_height', False)),
                                      p.get('desired_height_m', 0.0),
                                      bool(p.get('hold_on_converge', True)),
                                      p.get('total_timeout_sec', 0.0))
    if op == 'track_stop':
        return arm.track_target_stop()

    # ── 云台 ──
    if op.startswith('gimbal_'):
        gimbal = api.gimbal
        if gimbal is None:
            return CallResult(False, '云台直连接口不可用（robot_gimbal_interfaces_v2 未编译）')
        if op == 'gimbal_rotate':
            return gimbal.rotate_to_deg(p.get('pan'), p.get('roll'), p.get('tilt'),
                                        float(p.get('timeout_sec', 5.0)),
                                        bool(p.get('wait', True)))
        if op == 'gimbal_rotate_rad':
            return gimbal.rotate_to(p.get('pan'), p.get('roll'), p.get('tilt'),
                                    float(p.get('timeout_sec', 5.0)), bool(p.get('wait', True)))
        if op == 'gimbal_jog':
            return gimbal.jog(p.get('pan_vel', 0.0), p.get('roll_vel', 0.0),
                              p.get('tilt_vel', 0.0), float(p.get('duration_sec', 1.0)))
        if op == 'gimbal_forward_enable':
            return gimbal.set_forward_cmd_enable(bool(p.get('enable', True)))
        simple = {'gimbal_freeze': gimbal.freeze, 'gimbal_go_zero': gimbal.go_zero,
                  'gimbal_start': gimbal.start_motor, 'gimbal_stop': gimbal.stop_motor,
                  'gimbal_gyro_calib': gimbal.gyro_calib}
        if op in simple:
            simple[op]()
            return CallResult(True, 'published')
        return CallResult(False, f'未知云台 op: {op}')

    # ── 其他 ──
    if op == 'wait':
        time.sleep(float(p.get('seconds', 1.0)))
        return CallResult(True, f"waited {p.get('seconds', 1.0)}s")
    if op == 'wait_camera_ready':
        ok = arm.wait_camera_ready(float(p.get('timeout_sec', DEFAULT_ACTION_TIMEOUT_SEC)))
        return CallResult(ok, 'camera_ready' if ok else 'timeout')
    return CallResult(False, f'未知 op: {op}')


def execute_plan(api: ArmApi, steps: List[Dict[str, Any]],
                 stop_on_error: bool = True) -> List[CallResult]:
    """@brief 顺序执行运镜步骤表（大模型输出的 JSON 数组），逐条打日志。
    @param api           ArmApi
    @param steps         [{'op': 'observe'}, {'op': 'dolly', 'distance_m': 0.1}, ...]
    @param stop_on_error 某步失败是否中止后续
    @return 每步的 CallResult 列表（中止时长度 < len(steps)）
    """
    log = api.node.get_logger()
    results: List[CallResult] = []
    for index, step in enumerate(steps, start=1):
        log.info(f'── 步骤 {index}/{len(steps)}: {json.dumps(step, ensure_ascii=False)}')
        try:
            result = run_plan_step(api, step)
        except (KeyError, ValueError, TypeError) as exc:
            result = CallResult(False, f'参数错误: {exc!r}')
        results.append(result)
        # rclpy 的 logger 不允许同一调用点换 severity，成功 / 失败分两行写
        if result:
            log.info(f'   → {result}')
        else:
            log.error(f'   → {result}')
        if not result and stop_on_error:
            log.error('步骤失败，中止后续步骤')
            break
    return results
