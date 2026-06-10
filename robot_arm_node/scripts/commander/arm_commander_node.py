#!/usr/bin/env python3
"""
@file   arm_commander_node.py
@brief  Arm Commander 主节点 —— 机械臂中间层，承接 Director 语义指令，翻译为关节级控制
@version 1.0
@date   2026-06-09

职责：
  - 承载 ArmMoveToPose / ArmTrajectoryShot 两个 Action Server
  - 订阅 ArmFollowCommand（持续速度流）
  - 发布 ArmStatus（10Hz 周期广播）
  - 管理高层状态机：IDLE → MOVING → REACHED / STOPPED / ERROR

分层位置（三明治中间层）：
  Director (上层)  ←→  ArmCommanderNode (本节点)  ←→  Driver (下层)

用法（三个终端）：
  # 终端 1：启动仿真（选其一）
  ros2 launch robot_arm_bringup gazebo.launch.py      # Gazebo
  ros2 launch robot_arm_bringup mujoco.launch.py      # MuJoCo

  # 终端 2：启动本节点（headless，作为 action server）
  ros2 run robot_arm_node arm_commander_node

  # 终端 3：启动测试 GUI（Director 视角，验证接口）
  ros2 run robot_arm_node commander_test_gui

监听 / 查看状态：
  ros2 topic echo /robot_arm/arm_status
  ros2 action info /robot_arm/move_to_pose

覆盖预定义位姿参数（启动时）：
  ros2 run robot_arm_node arm_commander_node \
    --ros-args -p pose_observe_x:=0.3 -p pose_observe_z:=0.6

@copyright Copyright (c) 2026 eMeet
"""

import queue
import threading
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from robot_arm_interfaces.action import ArmMoveToPose, ArmTrajectoryShot
from robot_arm_interfaces.msg import ArmFollowCommand, ArmStatus
from robot_arm_interfaces.srv import ArmStop

from commander.move_to_pose_server import MoveToPoseServer
from commander.trajectory_shot_server import TrajectoryShotServer
from commander.motion_executor import MotionExecutor
from commander.status_aggregator import StatusAggregator


# ── 状态机 ───────────────────────────────────────────────────────────────────────
class CommanderState(IntEnum):
    """Arm Commander 高层状态。

    状态转换图：
                ┌──────────────────────────────────┐
                │                                  │
        ┌───────┴────────┐                         │
        ▼                │                         │
    IDLE ──(goal)──→ MOVING ──(done)──→ REACHED ───┘
     │                 │  │                    (reset / new goal)
     │                 │  ├──(cancel)──→ STOPPED ──→ IDLE
     │                 │  └──(timeout)─→ ERROR ────→ IDLE
     │                 │       (driver_err)
     └──(stop)──→ STOPPED ──→ IDLE
    """
    IDLE      = 0   # 空闲，等待指令
    MOVING    = 1   # 执行运动中
    REACHED   = 2   # 已到达目标（等待下一指令或复位）
    STOPPED   = 3   # 已急停
    ERROR     = 4   # 故障（需清错）


STATE_NAME = {v: k for k, v in CommanderState.__members__.items()}


# ── 预定义姿态（可通过 ROS param 覆盖）─────────────────────────────────────────────
# STOWED 收纳位 → 关节空间回零 [0,0,0,0,0,0]，不走 IK，无需 Cartesian 参数
DEFAULT_POSE_OBSERVE = dict(x=0.20, y=0.00, z=0.75, roll=90.0, pitch=0.0, yaw=0.0)


# ── 话题 / 动作名称常量 ───────────────────────────────────────────────────────────
TOPIC_FOLLOW_CMD   = '/robot_arm/follow_command'
TOPIC_ARM_STATUS   = '/robot_arm/arm_status'
ACTION_MOVE_TO_POSE   = '/robot_arm/move_to_pose'
ACTION_TRAJECTORY_SHOT  = '/robot_arm/trajectory_shot'
SERVICE_ARM_STOP      = '/robot_arm/stop'


class ArmCommanderNode(Node):
    """Arm Commander 主节点。

    作为 ROS 2 Node 运行，承载：
      - ActionServer: ArmMoveToPose, ArmTrajectoryShot
      - Subscription: ArmFollowCommand
      - Publisher:    ArmStatus (10Hz)

    对外接口：
      - ROS topic/action/service（供外部 Director）
      - gui_q（供 GUI import 模式，遵循现有 controller 模式）
    """

    # ── 初始化 ────────────────────────────────────────────────────────────────────
    def __init__(self, gui_q: queue.Queue = None):
        """初始化 Arm Commander 节点。

        Args:
            gui_q: 可选 queue.Queue，用于向 GUI 上报状态（遵循现有 controller 模式）。
                   传 None 则仅通过 ArmStatus topic 上报，适用于 headless 模式。
        """
        super().__init__('arm_commander')

        # ── 状态 ────────────────────────────────────────────────────────────────
        self._state        = CommanderState.IDLE
        self._state_lock   = threading.Lock()
        self._gui_q        = gui_q
        self._active_goal  = None              # 当前执行的 goal handle（用于 cancel）
        self._goal_lock    = threading.Lock()

        # ── 回调组（允许 action + topic 并行）──────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()

        # ── 子系统 ──────────────────────────────────────────────────────────────
        self.status     = StatusAggregator(self)
        self.motion     = MotionExecutor(self)
        self.move_to_pose_srv    = MoveToPoseServer(self, self.motion, self.status)
        self.trajectory_shot_srv = TrajectoryShotServer(self, self.motion, self.status)

        # ── Action Servers ──────────────────────────────────────────────────────
        self._mtp_server = ActionServer(
            self, ArmMoveToPose, ACTION_MOVE_TO_POSE,
            execute_callback = self._on_mtp_execute,
            cancel_callback  = self._on_mtp_cancel,
            callback_group   = self._cb_group,
        )
        # ArmTrajectoryShot — 骨架预留
        self._em_server = ActionServer(
            self, ArmTrajectoryShot, ACTION_TRAJECTORY_SHOT,
            execute_callback = self._on_em_execute,
            cancel_callback  = self._on_em_cancel,
            callback_group   = self._cb_group,
        )

        # ── 订阅：ArmFollowCommand ──────────────────────────────────────────────
        self._follow_sub = self.create_subscription(
            ArmFollowCommand, TOPIC_FOLLOW_CMD,
            self._on_follow_command, 10,
            callback_group=self._cb_group,
        )

        # ── 发布：ArmStatus ─────────────────────────────────────────────────────
        self._status_pub  = self.create_publisher(ArmStatus, TOPIC_ARM_STATUS, 10)
        self._status_timer = self.create_timer(0.1, self._publish_status)   # 10Hz

        # ── 服务：ArmStop（急停 + 从 ERROR/STOPPED 复位到 IDLE）───────────────
        self._stop_srv = self.create_service(
            ArmStop, SERVICE_ARM_STOP,
            self._on_arm_stop,
            callback_group=self._cb_group,
        )

        # ── 参数声明 ────────────────────────────────────────────────────────────
        self.declare_parameter('pose_observe_x',  DEFAULT_POSE_OBSERVE['x'])
        self.declare_parameter('pose_observe_y',  DEFAULT_POSE_OBSERVE['y'])
        self.declare_parameter('pose_observe_z',  DEFAULT_POSE_OBSERVE['z'])
        self.declare_parameter('pose_observe_roll',  DEFAULT_POSE_OBSERVE['roll'])
        self.declare_parameter('pose_observe_pitch', DEFAULT_POSE_OBSERVE['pitch'])
        self.declare_parameter('pose_observe_yaw',   DEFAULT_POSE_OBSERVE['yaw'])

        self.get_logger().info('Arm Commander 已就绪  |  '
                               f'状态={STATE_NAME[self._state]}  |  '
                               f'action: {ACTION_MOVE_TO_POSE} / {ACTION_TRAJECTORY_SHOT}  |  '
                               f'topic: {TOPIC_FOLLOW_CMD} → {TOPIC_ARM_STATUS}')

    # ── 状态机接口 ────────────────────────────────────────────────────────────────
    @property
    def state(self) -> CommanderState:
        with self._state_lock:
            return self._state

    def _transition(self, new_state: CommanderState):
        """状态转换，记录日志并通知 GUI。"""
        with self._state_lock:
            old = self._state
            self._state = new_state
        self.get_logger().info(f'状态: {STATE_NAME[old]} → {STATE_NAME[new_state]}')
        self._emit_gui('state', STATE_NAME[new_state])

    def _is_idle(self) -> bool:
        """是否可接受新指令。"""
        return self.state in (CommanderState.IDLE, CommanderState.REACHED)

    # ── ArmMoveToPose Action Server ───────────────────────────────────────────────
    def _on_mtp_execute(self, goal_handle):
        """执行 ArmMoveToPose goal（在专用线程中调用，可阻塞）。

        Humble ActionServer 简单模式：goal 自动 ACCEPT，在此方法内完成全部执行
        并返回 result。如需拒绝，调用 goal_handle.abort() 并返回。
        """
        goal = goal_handle.request
        state_name = {0: 'STOWED', 1: 'OBSERVE', 2: 'SHOOTING'}.get(
            goal.target_pose_state, 'UNKNOWN')
        self.get_logger().info(f'收到 MoveToPose goal: {state_name}')

        # ── 状态检查 ──────────────────────────────────────────────────────────
        if not self._is_idle():
            self.get_logger().warn(f'拒绝 goal: 当前状态={STATE_NAME[self.state]}，非空闲')
            goal_handle.abort()
            return ArmMoveToPose.Result()

        self.get_logger().info('MoveToPose goal 已接受，开始执行')
        self._transition(CommanderState.MOVING)

        with self._goal_lock:
            self._active_goal = goal_handle

        try:
            result = self.move_to_pose_srv.execute(goal_handle)
            self._transition(CommanderState.REACHED if result.success else CommanderState.ERROR)
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as e:
            self.get_logger().error(f'MoveToPose 执行异常: {e}')
            goal_handle.abort()
            self._transition(CommanderState.ERROR)
            return ArmMoveToPose.Result()
        finally:
            with self._goal_lock:
                self._active_goal = None

    def _on_mtp_cancel(self, cancel_request):
        """取消请求回调。"""
        self.get_logger().info('收到 MoveToPose 取消请求')
        self.motion.stop()
        self._transition(CommanderState.STOPPED)
        return CancelResponse.ACCEPT

    # ── ArmTrajectoryShot Action Server ──────────────────────────────────────────
    def _on_em_execute(self, goal_handle):
        goal = goal_handle.request
        type_name = {0: 'LINEAR', 1: 'ORBIT'}.get(goal.motion_type, 'UNKNOWN')
        self.get_logger().info(f'收到 TrajectoryShot goal: {type_name}')

        if not self._is_idle():
            self.get_logger().warn(f'拒绝 goal: 当前状态={STATE_NAME[self.state]}，非空闲')
            goal_handle.abort()
            return ArmTrajectoryShot.Result()

        self._transition(CommanderState.MOVING)
        with self._goal_lock:
            self._active_goal = goal_handle

        try:
            result = self.trajectory_shot_srv.execute(goal_handle)
            self._transition(CommanderState.REACHED if result.success else CommanderState.ERROR)
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as e:
            self.get_logger().error(f'TrajectoryShot 执行异常: {e}')
            goal_handle.abort()
            self._transition(CommanderState.ERROR)
            return ArmTrajectoryShot.Result()
        finally:
            with self._goal_lock:
                self._active_goal = None

    def _on_em_cancel(self, cancel_request):
        self.get_logger().info('收到 TrajectoryShot 取消请求')
        self.motion.stop()
        self._transition(CommanderState.STOPPED)
        return CancelResponse.ACCEPT

    # ── ArmStop 服务（急停 + 复位）────────────────────────────────────────────────
    def _on_arm_stop(self, _request, response):
        """ArmStop service handler：
        - MOVING → 急停 → STOPPED
        - ERROR / STOPPED → 复位 → IDLE（不发额外停止指令）
        """
        current = self.state
        if current == CommanderState.MOVING:
            self.motion.stop()
            self._transition(CommanderState.STOPPED)
            response.success = True
            response.message = f'已急停，状态 MOVING → STOPPED'
        elif current in (CommanderState.ERROR, CommanderState.STOPPED):
            self._transition(CommanderState.IDLE)
            self.status.clear_error()
            response.success = True
            response.message = f'已复位，状态 {STATE_NAME[current]} → IDLE'
        else:
            response.success = True
            response.message = f'当前状态={STATE_NAME[current]}，无需操作'
        self.get_logger().info(f'ArmStop: {response.message}')
        return response

    # ── ArmFollowCommand 订阅 ─────────────────────────────────────────────────────
    def _on_follow_command(self, msg: ArmFollowCommand):
        """接收 Director 下发的持续速度流（点动 / IBVS），故障时屏蔽。"""
        if self.state == CommanderState.ERROR:
            return
        self.motion.execute_twist(msg.twist)

    # ── ArmStatus 周期发布 ────────────────────────────────────────────────────────
    def _publish_status(self):
        """10Hz 周期广播 ArmStatus。"""
        msg = self.status.build_status_message()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

    # ── GUI 队列接口 ──────────────────────────────────────────────────────────────
    def _emit_gui(self, event_type: str, *args):
        """向 GUI 队列发送事件（遵循现有 controller 的 queue.Queue 模式）。"""
        if self._gui_q is not None:
            try:
                self._gui_q.put_nowait((event_type, *args))
            except queue.Full:
                pass

    # ── 公共接口 ──────────────────────────────────────────────────────────────────
    def stop(self):
        """外部急停接口（供 GUI 或 service 调用）。"""
        self.get_logger().info('执行急停')
        self.motion.stop()
        self._transition(CommanderState.STOPPED)

    def reset(self):
        """从 STOPPED / ERROR 恢复到 IDLE。"""
        if self.state in (CommanderState.STOPPED, CommanderState.ERROR):
            self._transition(CommanderState.IDLE)
        else:
            self.get_logger().warn(f'当前状态 {STATE_NAME[self.state]} 无需 reset')

    def get_pose_observe(self) -> dict:
        """读取 ROS param 中的 OBSERVE 预定义姿态。"""
        return {k: self.get_parameter(f'pose_observe_{k}').value
                for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}


# ── headless 入口 ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ArmCommanderNode()          # gui_q=None → headless 模式
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
