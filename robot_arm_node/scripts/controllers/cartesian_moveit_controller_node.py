#!/usr/bin/env python3
"""
@file   cartesian_moveit_controller_node.py
@brief  eMeetArm 末端笛卡尔精密路径控制 node（无 GUI，可被 GUI import 或 headless 运行）
@version 2.1
@date   2026-06-09

精密笛卡尔直线运动，不依赖任何 GUI 框架：
           compute_cartesian_path → 笛卡尔直线路径规划（末端严格走直线）
           execute_trajectory     → MoveIt 执行轨迹
           move_action            → 关节空间规划回零位
         状态/位姿通过 gui_q 队列上报（headless 运行时传空队列即可）。

被 cartesian_moveit_gui.py（tkinter 调试 GUI）import 使用；也可独立运行：
  ros2 run robot_arm_node cartesian_moveit_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    MoveItErrorCodes, JointConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener, Buffer

from arm_utils import rpy_to_quat, quat_to_rpy

# ── 参数 ──────────────────────────────────────────────────────────────────────
# (名称, 单位, 最小值, 最大值)
PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'base_link'

# ── 运动参数默认值（在此修改即可）────────────────────────────────────────────
DEFAULT_VEL      = 0.9    # 速度缩放比例   (0.01 ~ 1.0)
DEFAULT_ACC      = 0.5    # 加速度缩放比例 (0.01 ~ 1.0)
DEFAULT_MAX_STEP = 0.001  # 笛卡尔插值步长 m，越小越精确，规划越慢 (0.001 ~ 0.05)

# ── 预备位置（启动后自动执行）────────────────────────────────────────────────
READY_POSE = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianMoveitControllerNode(Node):
    """
    笛卡尔直线运动：
      /compute_cartesian_path (service) → 生成末端严格直线轨迹
      /execute_trajectory     (action)  → 执行轨迹
    回零使用：
      /move_action            (action)  → 关节空间规划回零位
    """
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_moveit_controller_node',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q              = gui_q
        self._busy             = False
        self._dispatch_timer   = None
        self._retry            = 0
        self._mode             = ''       # 'cartesian' | 'home'
        self._cp_req           = None     # GetCartesianPath.Request
        self._home_mg_req      = None     # MotionPlanRequest
        self._vel_scale        = 0.1
        self._acc_scale        = 0.1
        self._seq              = 0        # 每次新指令递增，让旧回调自动失效
        self._exec_goal_handle = None     # 当前执行中的 action goal handle
        self._cp_in_flight     = False    # compute_cartesian_path 是否正在飞行中

        # 笛卡尔路径服务 + 执行 action
        self._cp_cli  = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._exec_ac = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        # 回零用 MoveGroup action
        self._mg_ac   = ActionClient(self, MoveGroup, '/move_action')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)


    # ── TF → GUI ──────────────────────────────────────────────────────────────
    def _pub_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
            self._q.put(('pose', tr.x, tr.y, tr.z,
                         math.degrees(r), math.degrees(p), math.degrees(y)))
        except Exception:
            pass

    def get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
            return (tr.x, tr.y, tr.z,
                    math.degrees(r), math.degrees(p), math.degrees(y))
        except Exception:
            return None

    # ── 外部调用（GUI 线程） ───────────────────────────────────────────────────
    def go_ready(self):
        p = READY_POSE
        self.send_goal(p['x'], p['y'], p['z'],
                       p['roll'], p['pitch'], p['yaw'],
                       DEFAULT_VEL, DEFAULT_ACC, DEFAULT_MAX_STEP)

    def _preempt(self):
        self._seq += 1
        self._stop_dispatch()
        if self._exec_goal_handle is not None:
            self._exec_goal_handle.cancel_goal_async()
            self._exec_goal_handle = None

    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg, vel, acc, max_step):
        self._preempt()
        self._busy      = True
        self._retry     = 0
        self._mode      = 'cartesian'
        self._vel_scale = vel
        self._acc_scale = acc
        self._cp_req    = self._build_cp_req(x, y, z, roll_deg, pitch_deg, yaw_deg, max_step)
        self._q.put(('status', '连接服务...'))
        self._dispatch_timer = self.create_timer(0.05, self._dispatch)

    def send_home(self, vel, acc):
        self._preempt()
        self._busy        = True
        self._retry       = 0
        self._mode        = 'home'
        self._vel_scale   = vel
        self._acc_scale   = acc
        self._home_mg_req = self._build_home_req(vel, acc)
        self._q.put(('status', '回零规划中...'))
        self._dispatch_timer = self.create_timer(0.05, self._dispatch)

    # ── executor 线程：等待服务/action 就绪后派发 ──────────────────────────────
    def _dispatch(self):
        seq = self._seq
        if self._mode == 'cartesian':
            ready = self._cp_cli.service_is_ready()
        else:
            ready = self._mg_ac.server_is_ready()

        if not ready:
            self._retry += 1
            if self._retry >= 60:   # 60 × 50ms = 3s
                self._stop_dispatch()
                self._busy = False
                self._q.put(('status', '✗ MoveIt 服务未响应，请确认 MoveIt 已启动'))
            return

        self._stop_dispatch()

        if self._mode == 'cartesian':
            if self._cp_in_flight:
                return  # 已有请求在途，等它返回后自动触发新请求
            self._q.put(('status', '计算笛卡尔路径...'))
            self._cp_in_flight = True
            self._cp_cli.call_async(self._cp_req).add_done_callback(
                lambda f, s=seq: self._on_cartesian_path(f, s))
        else:
            goal = MoveGroup.Goal()
            goal.request                          = self._home_mg_req
            goal.planning_options.plan_only       = False
            goal.planning_options.replan          = True
            goal.planning_options.replan_attempts = 3
            self._mg_ac.send_goal_async(goal).add_done_callback(
                lambda f, s=seq: self._on_mg_goal_resp(f, s))

    def _stop_dispatch(self):
        if self._dispatch_timer:
            self._dispatch_timer.cancel()
            self._dispatch_timer.destroy()
            self._dispatch_timer = None

    # ── 笛卡尔路径回调 ────────────────────────────────────────────────────────
    def _on_cartesian_path(self, future, seq):
        self._cp_in_flight = False
        if seq != self._seq:
            # 旧结果丢弃，但有新指令等待：立刻用最新参数重新规划
            if self._busy and self._mode == 'cartesian':
                self._stop_dispatch()
                self._q.put(('status', '计算笛卡尔路径...'))
                self._cp_in_flight = True
                cur_seq = self._seq
                self._cp_cli.call_async(self._cp_req).add_done_callback(
                    lambda f, s=cur_seq: self._on_cartesian_path(f, s))
            return
        resp = future.result()
        if resp.fraction < 0.95:
            self._busy = False
            self._q.put(('status',
                f'✗ 笛卡尔路径仅覆盖 {resp.fraction*100:.0f}%，'
                '目标可能超出工作空间或存在奇异点'))
            return

        traj = resp.solution
        self._scale_trajectory(traj, self._vel_scale)

        self._q.put(('status', f'路径覆盖 {resp.fraction*100:.0f}%，执行中...'))
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        self._exec_ac.send_goal_async(goal).add_done_callback(
            lambda f, s=seq: self._on_exec_goal_resp(f, s))

    def _on_exec_goal_resp(self, future, seq):
        if seq != self._seq:
            return
        handle = future.result()
        if not handle.accepted:
            self._busy = False
            self._q.put(('status', '✗ 执行被拒绝'))
            return
        self._exec_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self._on_exec_result(f, s))

    def _on_exec_result(self, future, seq):
        if seq != self._seq:
            return
        self._exec_goal_handle = None
        self._busy = False
        val = future.result().result.error_code.val
        self._q.put(('status', '✓ 执行完成' if val == MoveItErrorCodes.SUCCESS
                     else f'✗ 执行失败，错误码: {val}'))

    # ── 回零 MoveGroup 回调 ───────────────────────────────────────────────────
    def _on_mg_goal_resp(self, future, seq):
        if seq != self._seq:
            return
        handle = future.result()
        if not handle.accepted:
            self._busy = False
            self._q.put(('status', '✗ 回零目标被拒绝'))
            return
        self._q.put(('status', '回零执行中...'))
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self._on_mg_result(f, s))

    def _on_mg_result(self, future, seq):
        if seq != self._seq:
            return
        self._busy = False
        val = future.result().result.error_code.val
        self._q.put(('status', '✓ 回零完成' if val == MoveItErrorCodes.SUCCESS
                     else f'✗ 回零失败，错误码: {val}'))

    # ── 构造 GetCartesianPath 请求 ────────────────────────────────────────────
    def _build_cp_req(self, x, y, z, roll_deg, pitch_deg, yaw_deg, max_step):
        qx, qy, qz, qw = rpy_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        target = Pose()
        target.position.x    = x;  target.position.y    = y;  target.position.z    = z
        target.orientation.x = qx; target.orientation.y = qy
        target.orientation.z = qz; target.orientation.w = qw

        req = GetCartesianPath.Request()
        req.header.frame_id  = BASE_FRAME
        req.header.stamp     = self.get_clock().now().to_msg()
        req.group_name       = PLANNING_GROUP
        req.link_name        = EEF_LINK
        req.waypoints        = [target]
        req.max_step         = max_step
        req.jump_threshold   = 0.0
        req.avoid_collisions = True
        return req

    def _build_home_req(self, vel, acc):
        req = MotionPlanRequest()
        req.group_name                      = PLANNING_GROUP
        req.num_planning_attempts           = 5
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = vel
        req.max_acceleration_scaling_factor = acc
        c = Constraints()
        for name in ('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = 0.0
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return req

    # ── 轨迹速度缩放（compute_cartesian_path 不支持缩放参数，手动处理）─────────
    @staticmethod
    def _scale_trajectory(traj, vel_scale):
        if vel_scale <= 0.0 or vel_scale >= 1.0:
            return
        factor = 1.0 / vel_scale   # > 1，时间拉长，速度降低
        for pt in traj.joint_trajectory.points:
            ns = pt.time_from_start.sec * 1_000_000_000 + pt.time_from_start.nanosec
            ns = int(ns * factor)
            pt.time_from_start.sec     = ns // 1_000_000_000
            pt.time_from_start.nanosec = ns %  1_000_000_000
            pt.velocities     = [v / factor         for v in pt.velocities]
            pt.accelerations  = [a / (factor*factor) for a in pt.accelerations]


def main():
    rclpy.init()
    node     = CartesianMoveitControllerNode(queue.Queue())
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
