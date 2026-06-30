#!/usr/bin/env python3
"""
@file   track_target_server.py
@brief  ArmTrackTarget Action Server —— 视觉伺服目标跟随执行逻辑

职责：
  - 接收 goal 后通过 ROS2 参数服务启动 visp_ibvs_node（paused=False）
  - 订阅 feature 话题，10Hz 计算图像误差 / 深度误差，发送 Feedback
  - 监测收敛 / 特征丢失 / 超时 / cancel，退出时挂起 IBVS（paused=True）
  - 通过 StatusAggregator 将跟随状态写入 ArmStatus 广播

被 ArmCommanderNode 持有，不独立运行。

@version 1.0
@date 2026-06-23
@copyright Copyright (c) 2026 eMeet
"""

import math
import time
import threading

from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PointStamped

from robot_arm_interfaces.action import ArmTrackTarget
from robot_arm_interfaces.msg import ArmStatus

from .status_aggregator import StatusAggregator


# ── 常量（与 visp_ibvs_node.cpp 保持一致）────────────────────────────────────
IMG_STOP_TH      = 0.005   # 图像误差收敛阈值（归一化）
DEPTH_STOP_TH    = 0.02    # 深度误差收敛阈值（m）
FEATURE_TIMEOUT  = 0.5     # 特征丢失判定时长（s）
FEEDBACK_HZ      = 10.0    # Feedback 发布频率
DEFAULT_DEPTH    = 0.3     # 未指定 desired_depth 时的默认值（m）

IBVS_NODE        = 'visp_ibvs_node'
FEATURE_TOPIC    = '/red_detector/feature'


class TrackTargetServer:
    """ArmTrackTarget Action 的执行逻辑。"""

    def __init__(self, node: Node, status: StatusAggregator):
        self._node   = node
        self._status = status
        self._logger = node.get_logger()

        # 特征缓存（线程安全）
        self._feat_lock      = threading.Lock()
        self._feat_x         = 0.0
        self._feat_y         = 0.0
        self._feat_z         = DEFAULT_DEPTH
        self._last_feat_time = None
        self._has_feat       = False

        # 取消标志（供 cancel 回调写入）
        self._cancel_flag = threading.Event()

        # 订阅特征话题
        node.create_subscription(
            PointStamped, FEATURE_TOPIC,
            self._on_feature, 10,
        )

        # visp_ibvs_node 参数服务客户端
        self._param_cli = node.create_client(
            SetParameters, f'/{IBVS_NODE}/set_parameters',
        )

    # ── 特征订阅回调 ──────────────────────────────────────────────────────────
    def _on_feature(self, msg: PointStamped):
        if not math.isfinite(msg.point.z) or msg.point.z <= 0.0:
            return
        with self._feat_lock:
            self._feat_x         = msg.point.x
            self._feat_y         = msg.point.y
            self._feat_z         = msg.point.z
            self._last_feat_time = time.time()
            self._has_feat       = True

    # ── IBVS 参数设置 ─────────────────────────────────────────────────────────
    def _set_params(self, **kwargs) -> bool:
        """向 visp_ibvs_node 写入 ROS2 参数。服务不可用时（仿真）直接返回 True。"""
        if not self._param_cli.wait_for_service(timeout_sec=1.0):
            self._logger.info(f'{IBVS_NODE} 参数服务不可用，仿真模式跳过')
            return True

        params = []
        for name, value in kwargs.items():
            p = Parameter(name=name)
            if isinstance(value, bool):
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=value)
            elif isinstance(value, float):
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            params.append(p)

        req = SetParameters.Request(parameters=params)
        event  = threading.Event()
        result = [None]

        def _done(future):
            try:
                result[0] = future.result()
            except Exception:
                pass
            event.set()

        self._param_cli.call_async(req).add_done_callback(_done)
        if event.wait(timeout=2.0) and result[0]:
            return all(r.successful for r in result[0].results)
        return False

    # ── 取消接口（供 Commander cancel 回调调用）───────────────────────────────
    def cancel(self):
        self._cancel_flag.set()

    # ── 主执行入口 ────────────────────────────────────────────────────────────
    def execute(self, goal_handle) -> ArmTrackTarget.Result:
        """在 action 专用线程中阻塞执行，直到退出条件满足。"""
        goal = goal_handle.request
        self._cancel_flag.clear()

        result    = ArmTrackTarget.Result()
        img_err   = 999.0
        depth_err = 999.0

        # ── 配置并启动 IBVS ──────────────────────────────────────────────────
        desired_depth = float(goal.desired_depth) if goal.desired_depth > 0.0 \
                        else DEFAULT_DEPTH
        params = {
            'paused':     False,
            'desired_x':  float(goal.desired_x),
            'desired_y':  float(goal.desired_y),
            'desired_depth': desired_depth,
            'constrain_height': bool(goal.constrain_height),
        }
        if goal.constrain_height and goal.desired_height > 0.0:
            params['desired_height'] = float(goal.desired_height)

        if not self._set_params(**params):
            self._logger.error('IBVS 参数设置失败，无法启动跟随')
            result.success    = False
            result.exit_code  = ArmTrackTarget.Result.EXIT_ERROR
            result.exit_reason = 'error'
            goal_handle.abort()
            return result

        self._status.set_tracking(True, 0.0, 0.0)
        self._logger.info(
            f'目标跟随启动  depth={desired_depth:.2f}m  '
            f'hold={goal.hold_on_converge}  timeout={goal.total_timeout_sec:.1f}s')

        # ── 主控制循环 ───────────────────────────────────────────────────────
        t_start   = time.time()
        fb_period = 1.0 / FEEDBACK_HZ
        last_fb   = 0.0

        while True:
            now     = time.time()
            elapsed = now - t_start

            # 急停检测（commander 进入 STOPPED）：立即退出，清理段会 paused=True 停 IBVS
            if self._node.is_stopped():
                self._logger.info('目标跟随期间被急停，退出')
                result.exit_code   = ArmTrackTarget.Result.EXIT_ERROR
                result.exit_reason = 'stopped'
                result.success     = False
                break

            # 取消检测
            if goal_handle.is_cancel_requested or self._cancel_flag.is_set():
                self._logger.info('目标跟随被取消')
                result.exit_code   = ArmTrackTarget.Result.EXIT_CANCELLED
                result.exit_reason = 'cancelled'
                result.success     = True
                break

            # 总超时
            if goal.total_timeout_sec > 0.0 and elapsed >= goal.total_timeout_sec:
                self._logger.warn(f'目标跟随总超时（{goal.total_timeout_sec:.1f}s）')
                result.exit_code   = ArmTrackTarget.Result.EXIT_TIMEOUT
                result.exit_reason = 'timeout'
                result.success     = False
                break

            # 读取特征快照
            with self._feat_lock:
                has_feat  = self._has_feat
                last_t    = self._last_feat_time
                fx, fy, fz = self._feat_x, self._feat_y, self._feat_z

            # 特征丢失
            if has_feat and (now - last_t) > FEATURE_TIMEOUT:
                self._logger.warn('特征丢失超时，退出跟随')
                result.exit_code   = ArmTrackTarget.Result.EXIT_FEATURE_LOST
                result.exit_reason = 'feature_lost'
                result.success     = False
                break

            # 误差计算
            if has_feat:
                img_err   = math.hypot(fx - goal.desired_x, fy - goal.desired_y)
                depth_err = abs(math.log(fz / desired_depth)) if fz > 0 else 999.0
            else:
                img_err = depth_err = 999.0

            self._status.set_tracking(True, img_err, depth_err)

            # 收敛检测
            is_converged = img_err < IMG_STOP_TH and depth_err < DEPTH_STOP_TH
            if is_converged and not goal.hold_on_converge:
                self._logger.info(
                    f'目标收敛  img_err={img_err:.4f}  depth_err={depth_err:.3f}m')
                result.exit_code   = ArmTrackTarget.Result.EXIT_CONVERGED
                result.exit_reason = 'converged'
                result.success     = True
                break

            # Feedback
            if now - last_fb >= fb_period:
                fb = ArmTrackTarget.Feedback()
                fb.img_err      = float(img_err)
                fb.depth_err_m  = float(depth_err)
                fb.elapsed_sec  = float(elapsed)
                fb.is_converged = bool(is_converged)
                fb.current_pose = self._status.pose
                goal_handle.publish_feedback(fb)
                last_fb = now

            time.sleep(0.02)

        # ── 退出清理 ─────────────────────────────────────────────────────────
        self._set_params(paused=True)
        self._status.set_tracking(False, 0.0, 0.0)

        result.final_img_err     = float(img_err)
        result.final_depth_err_m = float(depth_err)
        result.final_pose        = self._status.pose

        if result.exit_code == ArmTrackTarget.Result.EXIT_CANCELLED:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._logger.info(
            f'目标跟随结束  reason={result.exit_reason}  '
            f'img_err={result.final_img_err:.4f}')
        return result
