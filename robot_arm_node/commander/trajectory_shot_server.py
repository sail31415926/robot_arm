#!/usr/bin/env python3
"""
@file   trajectory_shot_server.py
@brief  ArmTrajectoryShot Action Server —— 直线运镜 / 球面环绕运镜执行逻辑
@version 1.0
@date   2026-06-10

职责：
  MOTION_LINEAR：
    1. 移动到 linear_start_pose（IK → JointTrajectory）
    2. 移动到 linear_end_pose
    3. 若 return_to_start=True，返回 linear_start_pose

  MOTION_ORBIT：
    1. 移动到起始球坐标（IK → JointTrajectory）
    2. 沿球面轨道运动到终止球坐标（Ruckig 1-DOF + IK 批量求解）
    3. 若 return_to_start=True，原路返回起始球坐标

被 ArmCommanderNode 持有，不独立运行。

@copyright Copyright (c) 2026 eMeet
"""

import math
import time

from rclpy.node import Node
from robot_arm_interfaces.action import ArmTrajectoryShot
from robot_arm_interfaces.msg import ArmPose, ArmStatus

from .motion_executor import MotionExecutor
from .status_aggregator import StatusAggregator
from arm_utils import aim_quat, sphere_to_cart, cart_to_sphere, quat_to_rpy


# ── 参数常量 ──────────────────────────────────────────────────────────────────────
DEFAULT_POSITION_TOLERANCE_M     = 0.01   # 到位判定：位置容差（米）
DEFAULT_ORIENTATION_TOLERANCE_DEG = 2.0  # 到位判定：姿态容差（度）
DEFAULT_TIMEOUT_SEC = 60.0               # 运动超时（秒）
FEEDBACK_RATE_HZ    = 10.0               # 反馈频率
DWELL_AT_START_SEC  = 1.0                # 到达起始点后停顿时长（秒），停顿结束再执行运镜

# 速度档位 → motion_executor 速度参数映射
SPEED_PROFILES = {
    ArmTrajectoryShot.Goal.SPEED_SLOW:   dict(v_pos=0.02, a_pos=0.05, j_pos=0.50,
                                              v_ori=0.05, a_ori=0.10, j_ori=1.00),
    ArmTrajectoryShot.Goal.SPEED_NORMAL: dict(v_pos=0.05, a_pos=0.10, j_pos=1.00,
                                              v_ori=0.10, a_ori=0.20, j_ori=2.00),
    ArmTrajectoryShot.Goal.SPEED_FAST:   dict(v_pos=0.10, a_pos=0.20, j_pos=2.00,
                                              v_ori=0.20, a_ori=0.40, j_ori=4.00),
}


class TrajectoryShotServer:
    """ArmTrajectoryShot Action 的执行逻辑。"""

    def __init__(self, node: Node, motion: MotionExecutor, status: StatusAggregator):
        self._node   = node
        self._motion = motion
        self._status = status
        self._logger = node.get_logger()

    # ── 主入口 ────────────────────────────────────────────────────────────────────
    def execute(self, goal_handle) -> ArmTrajectoryShot.Result:
        """执行一条 ArmTrajectoryShot goal（在 action 专用线程中调用，可阻塞）。"""
        goal  = goal_handle.request
        speed = SPEED_PROFILES.get(goal.transition_speed,
                                   SPEED_PROFILES[ArmTrajectoryShot.Goal.SPEED_NORMAL])

        if goal.motion_type == ArmTrajectoryShot.Goal.MOTION_LINEAR:
            return self._execute_linear(goal_handle, goal, speed)
        elif goal.motion_type == ArmTrajectoryShot.Goal.MOTION_ORBIT:
            return self._execute_orbit(goal_handle, goal, speed)
        else:
            self._logger.error(f'非法 motion_type: {goal.motion_type}')
            result = ArmTrajectoryShot.Result()
            result.success     = False
            result.exit_reason = 'error'
            result.error_code  = ArmStatus.ERR_LIMIT
            return result

    # ── MOTION_LINEAR ─────────────────────────────────────────────────────────────
    def _execute_linear(self, goal_handle, goal, speed) -> ArmTrajectoryShot.Result:
        """直线运镜：start_pose → end_pose（→ start_pose if return_to_start）。"""
        start = goal.linear_start_pose
        end   = goal.linear_end_pose

        self._logger.info(
            f'LINEAR 起始=({start.x:.3f},{start.y:.3f},{start.z:.3f}) '
            f'终止=({end.x:.3f},{end.y:.3f},{end.z:.3f})')

        # 步骤 1：移动到起始位姿（0 → 33%）
        r = self._move_and_wait(goal_handle, start, speed,
                                label='LINEAR 起始位',
                                progress_range=(0.0, 33.0 if goal.return_to_start else 50.0))
        if not r.success:
            return r
        # 到达起始点：置位信号 → 停顿 → 复位信号，再执行轨迹
        if not self._dwell_at_start(goal_handle, 'LINEAR'):
            res = ArmTrajectoryShot.Result()
            res.success     = False
            res.exit_reason = 'cancelled'
            return res

        # 步骤 2：移动到终止位姿（33 → 67% 或 50 → 100%）
        p2_end = 67.0 if goal.return_to_start else 100.0
        r = self._move_and_wait(goal_handle, end, speed,
                                label='LINEAR 终止位',
                                progress_range=(33.0 if goal.return_to_start else 50.0, p2_end))
        if not r.success or not goal.return_to_start:
            return r

        # 步骤 3：返回起始位姿（67 → 100%）
        self._logger.info('LINEAR return_to_start: 返回起始位姿')
        return self._move_and_wait(goal_handle, start, speed,
                                   label='LINEAR 返回起始',
                                   progress_range=(67.0, 100.0))

    # ── MOTION_ORBIT ─────────────────────────────────────────────────────────────
    def _execute_orbit(self, goal_handle, goal, speed) -> ArmTrajectoryShot.Result:
        """球面环绕运镜：start 球坐标 → end 球坐标（→ start if return_to_start）。

        执行流程：
          1. PTP 移动到起始球坐标（IK + JointTrajectory）
          2. Ruckig 1-DOF 球面轨道运动（批量 IK + 多路点 JointTrajectory）
          3. 若 return_to_start，Ruckig 原路返回起始球坐标
        """
        import threading as _th
        ox  = goal.orbit_center_x
        oy  = goal.orbit_center_y
        oz  = goal.orbit_center_z
        az0 = math.radians(goal.azimuth_start_deg)
        el0 = math.radians(goal.elevation_start_deg)
        r0  = goal.radius_start_m
        az1 = math.radians(goal.azimuth_end_deg)
        el1 = math.radians(goal.elevation_end_deg)
        r1  = goal.radius_end_m

        self._logger.info(
            f'ORBIT 球心=({ox:.3f},{oy:.3f},{oz:.3f})  '
            f'起({goal.azimuth_start_deg:.1f}°,{goal.elevation_start_deg:.1f}°,{r0:.3f}m) '
            f'→ 终({goal.azimuth_end_deg:.1f}°,{goal.elevation_end_deg:.1f}°,{r1:.3f}m)')

        # Ruckig 速度参数（从 speed 字典映射到球面归一化参数）
        # v_ori 对应球面轨道的归一化速度（已归一化到 [0,1] 路径参数）
        s_vel  = speed.get('v_ori', 0.10)
        s_acc  = speed.get('a_ori', 0.20)
        s_jerk = speed.get('j_ori', 2.00)

        cancel_ev = _th.Event()
        result    = ArmTrajectoryShot.Result()

        def _cancelled():
            return goal_handle.is_cancel_requested

        # ── 步骤 1：PTP 移到起始球坐标 ──────────────────────────────────────
        start_pose = self._sphere_to_pose(
            goal.azimuth_start_deg, goal.elevation_start_deg, r0, ox, oy, oz)
        r_ptp = self._move_and_wait(
            goal_handle, start_pose, speed,
            label='ORBIT PTP→起点',
            progress_range=(0.0, 30.0 if goal.return_to_start else 20.0),
            azimuth=goal.azimuth_start_deg,
            elevation=goal.elevation_start_deg,
            radius=r0)
        if not r_ptp.success:
            return r_ptp
        # 到达起始点：置位信号 → 停顿 → 复位信号，再执行轨道
        if not self._dwell_at_start(goal_handle, 'ORBIT'):
            result.exit_reason = 'cancelled'
            return result

        # ── 步骤 2：Ruckig 1-DOF 球面轨道（起 → 终）─────────────────────────
        if _cancelled():
            self._motion.stop()
            result.exit_reason = 'cancelled'
            return result

        self._logger.info('ORBIT 球面轨道开始（Ruckig 1-DOF）')
        ok = self._motion.plan_orbit_ruckig(
            ox, oy, oz, az0, el0, r0, az1, el1, r1,
            s_vel, s_acc, s_jerk, cancel_ev)
        if not ok:
            result.success     = False
            result.exit_reason = 'cancelled' if cancel_ev.is_set() else 'error'
            result.error_code  = ArmStatus.ERR_DRIVER
            return result

        if not goal.return_to_start:
            # 等待到终止位
            end_pose = self._sphere_to_pose(
                goal.azimuth_end_deg, goal.elevation_end_deg, r1, ox, oy, oz)
            return self._wait_at_pose(
                goal_handle, end_pose, speed,
                label='ORBIT 终止到位',
                progress_range=(60.0, 100.0),
                azimuth=goal.azimuth_end_deg,
                elevation=goal.elevation_end_deg,
                radius=r1)

        # ── 步骤 3：Ruckig 1-DOF 原路返回（终 → 起）─────────────────────────
        if _cancelled():
            self._motion.stop()
            result.exit_reason = 'cancelled'
            return result

        self._logger.info('ORBIT return_to_start: 原路返回')
        ok = self._motion.plan_orbit_ruckig(
            ox, oy, oz, az1, el1, r1, az0, el0, r0,
            s_vel, s_acc, s_jerk, cancel_ev)
        if not ok:
            result.success     = False
            result.exit_reason = 'cancelled' if cancel_ev.is_set() else 'error'
            result.error_code  = ArmStatus.ERR_DRIVER
            return result

        return self._wait_at_pose(
            goal_handle, start_pose, speed,
            label='ORBIT 返回到位',
            progress_range=(90.0, 100.0),
            azimuth=goal.azimuth_start_deg,
            elevation=goal.elevation_start_deg,
            radius=r0)

    # ── 到达起始点后的停顿 ────────────────────────────────────────────────────────
    def _dwell_at_start(self, goal_handle, label: str = '') -> bool:
        """到达运镜起始点后：置位 arm_at_pose_start → 停顿 DWELL_AT_START_SEC（期间可取消）
        → 复位 arm_at_pose_start，再返回。

        arm_at_pose_start 的 True 窗口 = 这段"停在起点等待"的时间；停顿结束即复位，
        因此轨迹执行期间及到达目标后均为 False。

        Returns:
            True  正常结束停顿，可继续执行轨迹
            False 停顿期间被取消（已发 stop，已复位信号）
        """
        self._status.set_at_pose_start(True)
        self._status.set_camera_ready(True)
        self._logger.info(f'{label} 已到达起始点，停顿 {DWELL_AT_START_SEC:.1f}s 后执行运镜')

        t_end     = time.time() + DWELL_AT_START_SEC
        cancelled = False
        while time.time() < t_end:
            if goal_handle.is_cancel_requested:
                self._motion.stop()
                self._logger.info(f'{label} 起点停顿期间被取消')
                cancelled = True
                break
            time.sleep(0.02)

        self._status.set_at_pose_start(False)
        return not cancelled

    # ── 共用：IK 移动 + 等待到位 ──────────────────────────────────────────────────
    def _move_and_wait(self, goal_handle, target: ArmPose, speed: dict,
                       label: str = '',
                       progress_range: tuple = (0.0, 100.0),
                       azimuth: float = 0.0,
                       elevation: float = 0.0,
                       radius: float = 0.0) -> ArmTrajectoryShot.Result:
        """下发一段 IK 轨迹并等待到位，期间发送 Feedback。"""
        result = ArmTrajectoryShot.Result()

        exec_r = self._motion.plan_and_execute(
            dict(x=target.x, y=target.y, z=target.z,
                 roll=target.roll, pitch=target.pitch, yaw=target.yaw),
            speed)
        if not exec_r['success']:
            result.success     = False
            result.exit_reason = exec_r.get('exit_reason', 'error')
            result.error_code  = ArmStatus.ERR_DRIVER
            return result

        # 轮询等待到位
        t_start   = time.time()
        fb_period = 1.0 / FEEDBACK_RATE_HZ
        last_fb   = 0.0
        p_lo, p_hi = progress_range

        dist_xyz = math.sqrt(
            (target.x - self._status.pose.x)**2 +
            (target.y - self._status.pose.y)**2 +
            (target.z - self._status.pose.z)**2
        )
        total_dur = max(dist_xyz / max(speed['v_pos'], 1e-6), 0.5)

        while time.time() - t_start < DEFAULT_TIMEOUT_SEC:
            if goal_handle.is_cancel_requested:
                self._motion.stop()
                result.exit_reason = 'cancelled'
                self._logger.info(f'{label} 被取消')
                break

            now = time.time()
            if now - last_fb >= fb_period:
                ratio    = min((now - t_start) / max(total_dur, 1e-6), 0.999)
                progress = p_lo + ratio * (p_hi - p_lo)

                fb = ArmTrajectoryShot.Feedback()
                fb.progress_percent   = float(progress)
                fb.elapsed_sec        = float(now - t_start)
                fb.current_pose       = self._status.pose
                fb.current_azimuth_deg   = float(azimuth)
                fb.current_elevation_deg = float(elevation)
                fb.current_radius_m      = float(radius)
                goal_handle.publish_feedback(fb)
                last_fb = now

            if self._is_at_target(target):
                result.success     = True
                result.exit_reason = 'reached'
                self._logger.info(f'{label} 到位')
                break

            time.sleep(0.01)

        result.error_code = (ArmStatus.ERR_NONE if result.success
                             else ArmStatus.ERR_TIMEOUT)
        return result

    # ── 等待已下发轨迹执行完毕（不再发新 IK，只轮询到位）────────────────────────────
    def _wait_at_pose(self, goal_handle, target: ArmPose, speed: dict,
                      label: str = '', progress_range: tuple = (0.0, 100.0),
                      azimuth: float = 0.0, elevation: float = 0.0,
                      radius: float = 0.0) -> ArmTrajectoryShot.Result:
        """轨迹已由 plan_orbit_ruckig 下发，此处仅轮询等到位并发 Feedback。"""
        result = ArmTrajectoryShot.Result()
        t_start   = time.time()
        fb_period = 1.0 / FEEDBACK_RATE_HZ
        last_fb   = 0.0
        p_lo, p_hi = progress_range

        while time.time() - t_start < DEFAULT_TIMEOUT_SEC:
            if goal_handle.is_cancel_requested:
                self._motion.stop()
                result.exit_reason = 'cancelled'
                self._logger.info(f'{label} 被取消')
                break
            now = time.time()
            if now - last_fb >= fb_period:
                ratio    = min((now - t_start) / max(DEFAULT_TIMEOUT_SEC * 0.1, 1e-6), 0.999)
                fb = ArmTrajectoryShot.Feedback()
                fb.progress_percent   = float(p_lo + ratio * (p_hi - p_lo))
                fb.elapsed_sec        = float(now - t_start)
                fb.current_pose       = self._status.pose
                fb.current_azimuth_deg   = float(azimuth)
                fb.current_elevation_deg = float(elevation)
                fb.current_radius_m      = float(radius)
                goal_handle.publish_feedback(fb)
                last_fb = now
            if self._is_at_target(target):
                result.success     = True
                result.exit_reason = 'reached'
                self._logger.info(f'{label} 到位')
                break
            time.sleep(0.01)

        result.error_code = (ArmStatus.ERR_NONE if result.success
                             else ArmStatus.ERR_TIMEOUT)
        return result

    # ── 球坐标 → Cartesian 位姿 ──────────────────────────────────────────────────
    @staticmethod
    def _sphere_to_pose(azimuth_deg: float, elevation_deg: float,
                        radius_m: float, ox: float = 0.0,
                        oy: float = 0.0, oz: float = 0.0) -> ArmPose:
        """球坐标（方位角/俯仰角/半径）→ 末端 Cartesian 位姿。

        使用 arm_utils.sphere_to_cart + aim_quat，与 spherical_orbit_controller 保持一致。
        ox/oy/oz 为球心坐标（默认 base_link 原点）。
        """
        az_rad = math.radians(azimuth_deg)
        el_rad = math.radians(elevation_deg)
        px, py, pz = sphere_to_cart(az_rad, el_rad, radius_m, ox, oy, oz)
        qx, qy, qz, qw = aim_quat(px, py, pz, ox, oy, oz)
        roll, pitch, yaw = [math.degrees(v) for v in quat_to_rpy(qx, qy, qz, qw)]
        pose = ArmPose()
        pose.x = float(px); pose.y = float(py); pose.z = float(pz)
        pose.roll = float(roll); pose.pitch = float(pitch); pose.yaw = float(yaw)
        return pose

    # ── 到位判定 ──────────────────────────────────────────────────────────────────
    def _is_at_target(self, target: ArmPose) -> bool:
        cur = self._status.pose
        pos_ok = (abs(cur.x - target.x) < DEFAULT_POSITION_TOLERANCE_M and
                  abs(cur.y - target.y) < DEFAULT_POSITION_TOLERANCE_M and
                  abs(cur.z - target.z) < DEFAULT_POSITION_TOLERANCE_M)

        def ang_diff(a, b):
            d = abs(a - b) % 360.0
            return d if d <= 180.0 else 360.0 - d

        ori_ok = (ang_diff(cur.roll,  target.roll)  < DEFAULT_ORIENTATION_TOLERANCE_DEG and
                  ang_diff(cur.pitch, target.pitch) < DEFAULT_ORIENTATION_TOLERANCE_DEG and
                  ang_diff(cur.yaw,   target.yaw)   < DEFAULT_ORIENTATION_TOLERANCE_DEG)

        return pos_ok and ori_ok
