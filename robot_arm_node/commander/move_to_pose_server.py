#!/usr/bin/env python3
"""
@file   move_to_pose_server.py
@brief  ArmMoveToPose Action Server —— 姿态切换执行逻辑
@version 1.0
@date   2026-06-09

职责：
  - 解析 Goal (target_pose_state / transition_speed / target_pose)
  - 将预定义姿态（STOWED / OBSERVE）解析为 Cartesian 目标
  - 委托 MotionExecutor 做 Ruckig OTG → batch IK → JointTrajectory
  - 周期发送 Feedback（progress_percent / current_pose）
  - 检测到位/超时 → 返回 Result

被 ArmCommanderNode 持有，不独立运行。

@copyright Copyright (c) 2026 eMeet
"""

import math
import time

from rclpy.node import Node
from robot_arm_interfaces.action import ArmMoveToPose
from robot_arm_interfaces.msg import ArmPose, ArmStatus

from .motion_executor import MotionExecutor
from .status_aggregator import StatusAggregator


# ── 到位判定参数 ──────────────────────────────────────────────────────────────────
DEFAULT_POSITION_TOLERANCE_M = 0.01    # 位置容差（米）
DEFAULT_ORIENTATION_TOLERANCE_DEG = 2.0  # 姿态容差（度）
DEFAULT_JOINT_TOLERANCE_RAD  = 0.02    # 关节容差（rad）
DEFAULT_TIMEOUT_SEC = 30.0             # 运动超时（秒）
FEEDBACK_RATE_HZ = 10.0                # 反馈频率

# STOWED 收纳位 —— 全关节回零
STOWED_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
STOWED_DURATION_SEC = 2.0              # 回零耗时


# ── 速度档位 → Ruckig 限制映射 ────────────────────────────────────────────────────
SPEED_PROFILES = {
    ArmMoveToPose.Goal.SPEED_SLOW:   dict(v_pos=0.02, a_pos=0.05, j_pos=0.50,
                                           v_ori=0.05, a_ori=0.10, j_ori=1.00),
    ArmMoveToPose.Goal.SPEED_NORMAL: dict(v_pos=0.05, a_pos=0.10, j_pos=1.00,
                                           v_ori=0.10, a_ori=0.20, j_ori=2.00),
    ArmMoveToPose.Goal.SPEED_FAST:   dict(v_pos=0.10, a_pos=0.20, j_pos=2.00,
                                           v_ori=0.20, a_ori=0.40, j_ori=4.00),
}


class MoveToPoseServer:
    """ArmMoveToPose Action 的执行逻辑。

    不持有 ROS 对象（publisher/subscriber 等）——全部通过 MotionExecutor 和
    StatusAggregator 间接访问。这样便于单元测试和替换执行策略。
    """

    def __init__(self, node: Node, motion: MotionExecutor, status: StatusAggregator):
        """
        Args:
            node:    ArmCommanderNode 实例（用于日志 / clock）
            motion:  共享运动执行引擎
            status:  共享状态聚合器（获取当前位姿、检测到位）
        """
        self._node   = node
        self._motion = motion
        self._status = status
        self._logger = node.get_logger()

    # ── 入口 ──────────────────────────────────────────────────────────────────────
    def execute(self, goal_handle) -> ArmMoveToPose.Result:
        """执行一条 ArmMoveToPose goal（在 action 专用线程中调用，可阻塞）。"""
        goal  = goal_handle.request
        state = goal.target_pose_state
        speed = SPEED_PROFILES.get(goal.transition_speed,
                                   SPEED_PROFILES[ArmMoveToPose.Goal.SPEED_NORMAL])

        # ── STOWED：关节空间回零（不走 IK，不支持 return_to_start）──────────────
        if state == ArmMoveToPose.Goal.POSE_STATE_STOWED:
            return self._execute_stowed(goal_handle)

        # ── OBSERVE / SHOOTING：Cartesian 目标 → IK → JointTrajectory ──────────
        # 记录出发位姿（用于 return_to_start）
        start_pose = self._status.pose

        target = self._resolve_target(goal)
        if target is None:
            result = ArmMoveToPose.Result()
            result.success     = False
            result.exit_reason = 'unreachable'
            result.error_code  = ArmStatus.ERR_LIMIT
            return result
        self._logger.info(f'MoveToPose 目标: x={target.x:.3f} y={target.y:.3f} '
                          f'z={target.z:.3f} R={target.roll:.1f} P={target.pitch:.1f} Y={target.yaw:.1f}')

        exec_result = self._motion.plan_and_execute(
            dict(x=target.x, y=target.y, z=target.z,
                 roll=target.roll, pitch=target.pitch, yaw=target.yaw),
            speed)
        if not exec_result['success']:
            result = ArmMoveToPose.Result()
            result.success     = False
            result.exit_reason = exec_result.get('exit_reason', 'error')
            result.error_code  = ArmStatus.ERR_DRIVER
            result.actual_pose = self._status.pose
            return result

        # 等待到位（进度 0→50% 对应前往目标，50→100% 对应返回起始点）
        result = self._wait_arrival(goal_handle, target, speed,
                                    progress_range=(0.0, 50.0 if goal.return_to_start else 100.0))
        if not result.success or not goal.return_to_start:
            return result

        # ── return_to_start：原路返回出发位姿 ────────────────────────────────────
        self._logger.info('return_to_start: 返回出发位姿')
        exec_back = self._motion.plan_and_execute(
            dict(x=start_pose.x, y=start_pose.y, z=start_pose.z,
                 roll=start_pose.roll, pitch=start_pose.pitch, yaw=start_pose.yaw),
            speed)
        if not exec_back['success']:
            result.success     = False
            result.exit_reason = 'error'
            result.error_code  = ArmStatus.ERR_DRIVER
            return result

        return self._wait_arrival(goal_handle, start_pose, speed,
                                  progress_range=(50.0, 100.0))

    # ── STOWED 执行（关节空间）────────────────────────────────────────────────────
    def _execute_stowed(self, goal_handle) -> ArmMoveToPose.Result:
        """收纳位：全关节回零。"""
        self._logger.info('STOWED: 全关节回零')
        self._motion.go_to_joints(STOWED_JOINTS, STOWED_DURATION_SEC)

        t_start   = time.time()
        fb_period = 1.0 / FEEDBACK_RATE_HZ
        last_fb   = 0.0
        result    = ArmMoveToPose.Result()

        while time.time() - t_start < DEFAULT_TIMEOUT_SEC:
            if goal_handle.is_cancel_requested:
                self._motion.stop()
                result.exit_reason = 'cancelled'
                self._logger.info('STOWED 被取消')
                break

            # 反馈（基于关节接近度）
            now = time.time()
            if now - last_fb >= fb_period:
                current = self._motion.get_current_joints()
                max_err = max(abs(c - t) for c, t in zip(current, STOWED_JOINTS))
                progress = max(0.0, 100.0 - max_err / 0.1 * 100.0)  # 粗略

                feedback = ArmMoveToPose.Feedback()
                feedback.progress_percent = float(min(progress, 99.9))
                feedback.current_pose     = self._status.pose
                goal_handle.publish_feedback(feedback)
                last_fb = now

            if self._is_at_joints(STOWED_JOINTS):
                result.success     = True
                result.exit_reason = 'reached'
                self._logger.info('STOWED 到位')
                break

            time.sleep(0.01)

        result.error_code  = (ArmStatus.ERR_NONE if result.success
                              else ArmStatus.ERR_TIMEOUT)
        result.actual_pose = self._status.pose
        return result

    # ── 等待到位（Cartesian）──────────────────────────────────────────────────────
    def _wait_arrival(self, goal_handle, target, speed,
                      progress_range=(0.0, 100.0)) -> ArmMoveToPose.Result:
        """轨迹下发后轮询等待到位。

        Args:
            progress_range: (start%, end%) 进度映射区间，支持分段反馈
                            例如去程 (0, 50)、回程 (50, 100)
        """
        t_start   = time.time()
        fb_period = 1.0 / FEEDBACK_RATE_HZ
        last_fb   = 0.0
        result    = ArmMoveToPose.Result()
        p_lo, p_hi = progress_range

        dist_xyz = math.sqrt(
            (target.x - self._status.pose.x)**2 +
            (target.y - self._status.pose.y)**2 +
            (target.z - self._status.pose.z)**2
        )
        total_duration = max(dist_xyz / max(speed['v_pos'], 1e-6), 0.5)

        while time.time() - t_start < DEFAULT_TIMEOUT_SEC:
            if goal_handle.is_cancel_requested:
                self._motion.stop()
                result.exit_reason = 'cancelled'
                self._logger.info('MoveToPose 被取消')
                break

            now = time.time()
            if now - last_fb >= fb_period:
                elapsed  = now - t_start
                ratio    = min(elapsed / max(total_duration, 1e-6), 0.999)
                progress = p_lo + ratio * (p_hi - p_lo)
                feedback = ArmMoveToPose.Feedback()
                feedback.progress_percent = float(progress)
                feedback.current_pose     = self._status.pose
                goal_handle.publish_feedback(feedback)
                last_fb = now

            if self._is_at_target(target):
                result.success     = True
                result.exit_reason = 'reached'
                self._logger.info('MoveToPose 已到达目标')
                break

            time.sleep(0.01)

        result.error_code  = (ArmStatus.ERR_NONE if result.success
                              else ArmStatus.ERR_TIMEOUT)
        result.actual_pose = self._status.pose
        return result

    # ── 目标解析 ──────────────────────────────────────────────────────────────────
    def _resolve_target(self, goal: ArmMoveToPose.Goal) -> ArmPose | None:
        """根据 target_pose_state 解析为 ArmPose 目标。

        Returns:
            ArmPose 目标，或 None（非法参数）
        """
        if goal.target_pose_state == ArmMoveToPose.Goal.POSE_STATE_OBSERVE:
            p = self._node.get_pose_observe()
            target = ArmPose()
            target.x     = float(p['x'])
            target.y     = float(p['y'])
            target.z     = float(p['z'])
            target.roll  = float(p['roll'])
            target.pitch = float(p['pitch'])
            target.yaw   = float(p['yaw'])
            return target

        elif goal.target_pose_state == ArmMoveToPose.Goal.POSE_STATE_SHOOTING:
            return goal.target_pose   # 绝对位姿，直接使用

        else:
            self._logger.error(f'非法 target_pose_state: {goal.target_pose_state}')
            return None

    # ── 到位判定（关节空间）─────────────────────────────────────────────────────────
    def _is_at_joints(self, target_joints: list) -> bool:
        """判断所有关节是否到达目标角度。"""
        current = self._motion.get_current_joints()
        for c, t in zip(current, target_joints):
            if abs(c - t) > DEFAULT_JOINT_TOLERANCE_RAD:
                return False
        return True

    # ── 到位判定（Cartesian）───────────────────────────────────────────────────────
    def _is_at_target(self, target: ArmPose) -> bool:
        """判断末端是否到达目标（位置 + 姿态双重容差）。

        Returns:
            True 表示已到位
        """
        cur = self._status.pose
        pos_ok = (abs(cur.x - target.x) < DEFAULT_POSITION_TOLERANCE_M and
                  abs(cur.y - target.y) < DEFAULT_POSITION_TOLERANCE_M and
                  abs(cur.z - target.z) < DEFAULT_POSITION_TOLERANCE_M)

        # 角度差（考虑环绕 -180/180）
        def angular_diff(a, b):
            d = abs(a - b) % 360.0
            return d if d <= 180.0 else 360.0 - d

        ori_ok = (angular_diff(cur.roll,  target.roll)  < DEFAULT_ORIENTATION_TOLERANCE_DEG and
                  angular_diff(cur.pitch, target.pitch) < DEFAULT_ORIENTATION_TOLERANCE_DEG and
                  angular_diff(cur.yaw,   target.yaw)   < DEFAULT_ORIENTATION_TOLERANCE_DEG)

        return pos_ok and ori_ok
