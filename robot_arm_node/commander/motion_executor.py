#!/usr/bin/env python3
"""
@file   motion_executor.py
@brief  共享运动执行引擎 —— Cartesian → Ruckig OTG → 批量 IK → JointTrajectory
@version 1.0
@date   2026-06-09

职责：
  - 封装 Ruckig 在线轨迹生成 + /compute_ik (pick_ik) 批量求解 + JointTrajectory 下发
  - 提供两种执行模式：
      plan_and_execute(target_pose, speed)  → 点到点（供 MoveToPose / ExecuteMotion）
      execute_twist(twist)                  → 速度流（供 ArmFollowCommand）
  - 提供 stop() 急停

复用了 cartesian_trajectory_controller_node.py 的核心算法：
  - _ik_sync() —— 同步 IK 包装
  - _solve_and_send() —— 批量 IK + 轨迹发布
  - Ruckig 4-DOF / 1-DOF OTG

不依赖 GUI 框架，不 import tkinter / PyQt5。

@copyright Copyright (c) 2026 eMeet
"""

import math
import time
import threading
import queue

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer

# Ruckig 可选依赖 —— 如果未安装，plan_and_execute 将报错
try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result as RuckigResult
    _HAS_RUCKIG = True
except ImportError:
    _HAS_RUCKIG = False

from arm_utils import rpy_to_quat, quat_to_rpy, sphere_to_cart, aim_quat


# ── 常量 ──────────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'arm_base_link'

STREAM_DT    = 0.01     # s，Ruckig 内部步长（100Hz）
IK_SAMPLE_DT = 0.03     # s，IK 采样步长：每 ~30ms 取一个路点喂 IK，下游控制器插值（降低 IK 点数）
IK_DECIMATE  = max(1, round(IK_SAMPLE_DT / STREAM_DT))   # =3，球面轨道路点数 ÷3
IK_TIMEOUT_S = 0.05     # 单次 IK 超时

# 默认运动限制
DEFAULT_V_POS = 0.05;  DEFAULT_A_POS = 0.10;  DEFAULT_J_POS = 1.00
DEFAULT_V_ORI = 0.10;  DEFAULT_A_ORI = 0.20;  DEFAULT_J_ORI = 2.00

# 速度流限制
MAX_V_LIN = 0.30    # m/s
MAX_V_ANG = 1.00    # rad/s

# 话题
TRAJ_TOPIC         = '/arm_controller/joint_trajectory'
TWIST_TOPIC        = '/servo_node/delta_twist_cmds'
IK_SERVICE         = '/compute_ik'
JOINT_STATE_TOPIC  = '/joint_states'


class MotionExecutor:
    """运动执行引擎 —— 被 ArmCommanderNode 持有，共享 ROS 资源。

    注意：不继承 Node，所有 publisher/subscriber/client 使用传入的 node 创建。
    这保证了整个 Arm Commander 的 ROS 资源归属于同一个 Node。
    """

    def __init__(self, node: Node):
        """
        Args:
            node: ArmCommanderNode 实例
        """
        self._node   = node
        self._logger = node.get_logger()

        # ── 关节状态缓存 ──────────────────────────────────────────────────────
        self._joint_positions = {name: 0.0 for name in JOINT_NAMES}
        self._joint_velocities = {name: 0.0 for name in JOINT_NAMES}
        self._joint_lock = threading.Lock()

        node.create_subscription(
            JointState, JOINT_STATE_TOPIC,
            self._on_joint_state, 10,
        )

        # ── TF2 ───────────────────────────────────────────────────────────────
        self._tf_buffer    = Buffer()
        self._tf_listener  = TransformListener(self._tf_buffer, node)

        # ── Publisher：JointTrajectory ────────────────────────────────────────
        self._traj_pub = node.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        # ── Publisher：TwistStamped（速度流）─────────────────────────────────
        self._twist_pub = node.create_publisher(TwistStamped, TWIST_TOPIC, 10)

        # ── Client：/compute_ik ──────────────────────────────────────────────
        self._ik_client = node.create_client(GetPositionIK, IK_SERVICE)

        # ── Ruckig 可用性 ────────────────────────────────────────────────────
        if not _HAS_RUCKIG:
            self._logger.warn('Ruckig 未安装，plan_and_execute() 不可用；仅 execute_twist() 可用')

        self._logger.info(f'MotionExecutor 就绪  |  '
                          f'JointTrajectory → {TRAJ_TOPIC}  |  '
                          f'TwistStamped → {TWIST_TOPIC}  |  '
                          f'IK → {IK_SERVICE}  |  '
                          f'Ruckig={"✓" if _HAS_RUCKIG else "✗"}')

    # ── 关节状态订阅 ──────────────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        with self._joint_lock:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                if name in self._joint_positions:
                    self._joint_positions[name] = pos
                    self._joint_velocities[name] = vel

    # ── 公共：点对点运动 ──────────────────────────────────────────────────────────
    def plan_and_execute(self, target_pose: dict, speed: dict,
                         feedback_cb=None, cancel_event=None) -> dict:
        """规划并执行一条 Cartesian 轨迹（阻塞调用，在独立线程中运行）。

        当前实现：单点 IK → JointTrajectory（由 joint_trajectory_controller 插值）。
        后续迭代将接入 Ruckig OTG + 批量 IK，实现 jerk-limited 轨迹。

        Args:
            target_pose:  dict with keys x, y, z, roll, pitch, yaw (m, deg)
            speed:        dict with keys v_pos, a_pos, j_pos, v_ori, a_ori, j_ori
            feedback_cb:  optional callable(progress_0_1, current_pose_dict) for progress
            cancel_event: optional threading.Event to signal cancellation

        Returns:
            dict with keys: success (bool), exit_reason (str),
                            actual_pose (dict), error_code (int)
        """
        start_pose = self.get_ee_pose()
        self._logger.info(f'plan_and_execute: 起点=({start_pose["x"]:.3f},{start_pose["y"]:.3f},'
                          f'{start_pose["z"]:.3f}) → 目标=({target_pose["x"]:.3f},'
                          f'{target_pose["y"]:.3f},{target_pose["z"]:.3f})')

        # ── 1. 构建 PoseStamped ────────────────────────────────────────────────
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_FRAME
        pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(target_pose['x'])
        pose_stamped.pose.position.y = float(target_pose['y'])
        pose_stamped.pose.position.z = float(target_pose['z'])
        qx, qy, qz, qw = rpy_to_quat(
            math.radians(target_pose['roll']),
            math.radians(target_pose['pitch']),
            math.radians(target_pose['yaw']),
        )
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        # ── 2. IK 求解 ─────────────────────────────────────────────────────────
        if cancel_event and cancel_event.is_set():
            return {'success': False, 'exit_reason': 'cancelled',
                    'actual_pose': start_pose, 'error_code': 0}

        joints, error_code = self.ik_sync(pose_stamped)
        if joints is None:
            self._logger.warn(f'IK 无解，目标不可达 (error_code={error_code})')
            return {'success': False, 'exit_reason': 'unreachable',
                    'actual_pose': start_pose, 'error_code': error_code}

        self._logger.info(f'IK 成功: {[f"{j:.3f}" for j in joints]}')

        # ── 3. 计算到达时间 ─────────────────────────────────────────────────────
        dist_xyz = math.sqrt(
            (target_pose['x'] - start_pose['x'])**2 +
            (target_pose['y'] - start_pose['y'])**2 +
            (target_pose['z'] - start_pose['z'])**2
        )
        # 按速度档位估算耗时（含加减速余量）
        v = speed.get('v_pos', DEFAULT_V_POS)
        duration = max(dist_xyz / max(v, 1e-6) * 1.5, 0.5)  # 1.5x 加减速余量，下限 0.5s

        # ── 4. 下发 JointTrajectory ─────────────────────────────────────────────
        # 必须包含起始点（t=0）+ 目标点（t=duration）：
        #   IP 模式：需要两点之间做线性插补，单点会导致缓冲下溢→驱动失能
        #   PP 模式：取最后一个 waypoint，起始点不影响行为
        msg = JointTrajectory()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES

        pt0 = JointTrajectoryPoint()
        pt0.positions      = [float(j) for j in self.get_current_joints()]
        pt0.velocities     = [0.0] * len(JOINT_NAMES)
        pt0.time_from_start = Duration(sec=0, nanosec=0)

        pt1 = JointTrajectoryPoint()
        pt1.positions      = [float(j) for j in joints]
        pt1.velocities     = [0.0] * len(JOINT_NAMES)
        pt1.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))

        msg.points = [pt0, pt1]
        self._traj_pub.publish(msg)

        self._logger.info(f'JointTrajectory 已下发 (duration={duration:.2f}s)')

        # ── 5. 发送 100% feedback ──────────────────────────────────────────────
        if feedback_cb:
            feedback_cb(1.0, target_pose)

        return {
            'success': True,
            'exit_reason': 'reached',
            'actual_pose': target_pose,
            'error_code': 0,
        }

    # ── 公共：速度流 ──────────────────────────────────────────────────────────────
    def execute_twist(self, twist) -> None:
        """发布 TwistStamped 速度指令（供 ArmFollowCommand 使用）。

        Args:
            twist: robot_arm_interfaces.msg.ArmTwist — 注意单位是度/秒，需转弧度
        """
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME

        # 限幅
        msg.twist.linear.x  = max(-MAX_V_LIN, min(MAX_V_LIN, float(twist.vx)))
        msg.twist.linear.y  = max(-MAX_V_LIN, min(MAX_V_LIN, float(twist.vy)))
        msg.twist.linear.z  = max(-MAX_V_LIN, min(MAX_V_LIN, float(twist.vz)))
        msg.twist.angular.x = max(-MAX_V_ANG, min(MAX_V_ANG, math.radians(float(twist.wroll))))
        msg.twist.angular.y = max(-MAX_V_ANG, min(MAX_V_ANG, math.radians(float(twist.wpitch))))
        msg.twist.angular.z = max(-MAX_V_ANG, min(MAX_V_ANG, math.radians(float(twist.wyaw))))

        self._twist_pub.publish(msg)

    # ── 公共：急停 ────────────────────────────────────────────────────────────────
    def stop(self) -> None:
        """急停：在当前位置发布零速度 JointTrajectory。"""
        with self._joint_lock:
            positions = [self._joint_positions[name] for name in JOINT_NAMES]

        msg = JointTrajectory()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions     = positions
        pt.velocities    = [0.0] * len(JOINT_NAMES)
        pt.time_from_start = Duration(sec=0, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self._logger.info('急停指令已发送')

    # ── IK 工具 ────────────────────────────────────────────────────────────────────
    def ik_sync(self, pose_stamped: PoseStamped) -> tuple:
        """同步 IK 求解（复用 cartesian_trajectory_controller_node 的 Event 模式）。

        Args:
            pose_stamped: 目标位姿（base_link 坐标系）

        Returns:
            (joint_list: list[float] | None, error_code: int)
            error_code == MoveItErrorCodes.SUCCESS (1) 表示成功
        """
        if not self._ik_client.wait_for_service(timeout_sec=0.5):
            self._logger.warn('/compute_ik 服务不可用')
            return None, -1

        req = GetPositionIK.Request()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = int(IK_TIMEOUT_S)
        req.ik_request.avoid_collisions = False

        # 填入当前关节角（种子）
        with self._joint_lock:
            seed = RobotState()
            seed.joint_state.name     = JOINT_NAMES
            seed.joint_state.position = [self._joint_positions[n] for n in JOINT_NAMES]
        req.ik_request.robot_state = seed

        # 同步等待
        event = threading.Event()
        result_holder = {'joints': None, 'error': -1}

        def _done(future):
            try:
                resp = future.result()
                if (resp is not None and
                    resp.error_code.val == MoveItErrorCodes.SUCCESS):
                    result_holder['joints'] = list(resp.solution.joint_state.position)
                    result_holder['error']  = MoveItErrorCodes.SUCCESS
                else:
                    result_holder['error'] = (resp.error_code.val
                                              if resp else -1)
            except Exception as e:
                self._logger.error(f'IK 调用异常: {e}')
            finally:
                event.set()

        self._ik_client.call_async(req).add_done_callback(_done)
        if not event.wait(timeout=IK_TIMEOUT_S * 2):
            self._logger.warn(f'IK 超时 ({IK_TIMEOUT_S*2:.1f}s)')
            return None, -1

        return result_holder['joints'], result_holder['error']

    # ── 末端位姿查询 ──────────────────────────────────────────────────────────────
    def get_ee_pose(self) -> dict:
        """从 TF2 查询当前末端位姿。

        Returns:
            dict: {x, y, z (m), roll, pitch, yaw (deg)} 或零位姿
        """
        try:
            t = self._tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tx, ty, tz = (t.transform.translation.x,
                          t.transform.translation.y,
                          t.transform.translation.z)
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
            return dict(x=tx, y=ty, z=tz,
                        roll=math.degrees(roll),
                        pitch=math.degrees(pitch),
                        yaw=math.degrees(yaw))
        except Exception:
            return dict(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)

    def get_ee_quat(self) -> tuple:
        """从 TF2 查询当前末端姿态（四元数）。

        Returns:
            (qx, qy, qz, qw) 或 (0, 0, 0, 1)
        """
        try:
            t = self._tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            return (t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w)
        except Exception:
            return (0.0, 0.0, 0.0, 1.0)

    # ── 关节空间直驱（STOWED 收纳位：全关节回零）───────────────────────────────────
    def go_to_joints(self, target_joints: list, duration_sec: float = 2.0) -> dict:
        """直接下发关节目标（跳过 IK，供 STOWED 等预定义关节位姿使用）。

        Args:
            target_joints: 6 个关节目标角（rad）
            duration_sec:  到达时间

        Returns:
            dict: {success, exit_reason, actual_pose, error_code}
        """
        self._logger.info(f'go_to_joints: {[f"{j:.3f}" for j in target_joints]} '
                          f'duration={duration_sec:.2f}s')

        msg = JointTrajectory()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES

        pt0 = JointTrajectoryPoint()
        pt0.positions      = [float(j) for j in self.get_current_joints()]
        pt0.velocities     = [0.0] * len(JOINT_NAMES)
        pt0.time_from_start = Duration(sec=0, nanosec=0)

        pt1 = JointTrajectoryPoint()
        pt1.positions      = [float(j) for j in target_joints]
        pt1.velocities     = [0.0] * len(target_joints)
        pt1.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        msg.points = [pt0, pt1]
        self._traj_pub.publish(msg)

        return {'success': True, 'exit_reason': 'sent', 'error_code': 0}

    def get_current_joints(self) -> list:
        """获取当前关节位置列表（rad）。"""
        with self._joint_lock:
            return [self._joint_positions[n] for n in JOINT_NAMES]

    # ── 低级 IK（显式种子，供批量求解用）─────────────────────────────────────────────
    def _ik_sync_with_seed(self, x, y, z, qx, qy, qz, qw, seed: list):
        """同步 IK，种子由调用方显式传入（批量 IK 时用上一帧解作种子）。

        Returns:
            (joint_list | None, error_code)
        """
        # 服务可用性由 solve_and_send 在批量开始时检查一次；此处仅做廉价的本地就绪判断，
        # 避免每个路点都阻塞式 wait_for_service（上千点时这是显著开销）。
        if not self._ik_client.service_is_ready():
            return None, -1

        ps = PoseStamped()
        ps.header.frame_id     = BASE_FRAME
        ps.header.stamp        = self._node.get_clock().now().to_msg()
        ps.pose.position.x     = float(x);  ps.pose.position.y    = float(y)
        ps.pose.position.z     = float(z);  ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y  = float(qy); ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w  = float(qw)

        rs = RobotState()
        rs.joint_state.name     = JOINT_NAMES
        rs.joint_state.position = list(seed)

        req = GetPositionIK.Request()
        req.ik_request.group_name       = PLANNING_GROUP
        req.ik_request.ik_link_name     = EEF_LINK
        req.ik_request.pose_stamped     = ps
        req.ik_request.robot_state      = rs
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec      = 0
        req.ik_request.timeout.nanosec  = int(IK_TIMEOUT_S * 1e9)

        ev = threading.Event(); box = [None]
        def _cb(f): box[0] = f; ev.set()
        self._ik_client.call_async(req).add_done_callback(_cb)
        ev.wait(timeout=IK_TIMEOUT_S + 0.05)

        if box[0] is None:
            return None, -1
        try:
            resp = box[0].result()
        except Exception:
            return None, -1
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, resp.error_code.val
        n2p = dict(zip(resp.solution.joint_state.name,
                       resp.solution.joint_state.position))
        return [n2p.get(n, 0.0) for n in JOINT_NAMES], MoveItErrorCodes.SUCCESS

    # ── 路点降采样（减少 IK 求解次数）───────────────────────────────────────────────
    @staticmethod
    def _decimate(all_pts: list, k: int) -> list:
        """对 Ruckig 100Hz 路点降采样：每 k 个取 1 个，并始终保留首/末点。

        下游 joint_trajectory_controller 会在路点之间做插值，因此降采样只减少需要
        求解 IK 的点数（k=3 → IK 调用数 ÷3），不改变轨迹的起止位姿与总时长。
        """
        if k <= 1 or len(all_pts) <= 2:
            return all_pts
        sampled = all_pts[::k]
        if sampled[-1] is not all_pts[-1]:
            sampled.append(all_pts[-1])
        return sampled

    # ── 批量 IK + JointTrajectory 下发（从 spherical_orbit_controller 提取）─────────
    def solve_and_send(self, all_pts: list, cancel_event: threading.Event = None) -> bool:
        """对轨迹点列表批量求 IK，计算关节速度，下发 JointTrajectory。

        Args:
            all_pts: list of (t_sec, x, y, z, qx, qy, qz, qw)
            cancel_event: 设置后中途放弃

        Returns:
            True 表示成功下发，False 表示失败/中止
        """
        if not all_pts:
            return False

        # 服务可用性只在批量开始检查一次（之后逐点用廉价的 service_is_ready）
        if not self._ik_client.wait_for_service(timeout_sec=1.0):
            self._logger.error('solve_and_send: /compute_ik 服务不可用')
            return False

        # 降采样：100Hz 路点 → 每 IK_DECIMATE 个取 1 个，IK 求解次数随之下降
        n_raw   = len(all_pts)
        all_pts = self._decimate(all_pts, IK_DECIMATE)
        t_plan0 = time.time()

        with self._joint_lock:
            seed = [self._joint_positions[n] for n in JOINT_NAMES]

        joint_pos = []
        joint_t   = []

        IK_ERR = {1:'OK', -1:'PLANNING_FAILED', -6:'TIMED_OUT',
                  -31:'NO_IK_SOLUTION', -1:'TIMEOUT'}

        for idx, (t_pt, wx, wy, wz, qx, qy, qz, qw) in enumerate(all_pts):
            if cancel_event and cancel_event.is_set():
                self._logger.info('solve_and_send: 中止（cancel_event）')
                return False

            sol, err = self._ik_sync_with_seed(wx, wy, wz, qx, qy, qz, qw, seed)

            # 首帧失败时用零种子重试
            if sol is None and idx == 0:
                sol, err = self._ik_sync_with_seed(wx, wy, wz, qx, qy, qz, qw,
                                                   [0.0] * 6)
            if sol is None:
                err_name = IK_ERR.get(err, str(err))
                if joint_pos:
                    sol = joint_pos[-1]   # 降级：沿用上一帧
                    self._logger.warn(f'IK 失败 step={idx} err={err_name}  '
                                      f'pos=({wx:.3f},{wy:.3f},{wz:.3f})，沿用上帧')
                else:
                    self._logger.error(f'IK 首帧失败 err={err_name}  '
                                       f'pos=({wx:.3f},{wy:.3f},{wz:.3f})，放弃')
                    return False

            joint_pos.append(sol)
            joint_t.append(t_pt)
            seed = sol

            if (idx + 1) % 20 == 0:
                self._logger.debug(f'solve_and_send 规划中 {idx+1}/{len(all_pts)}')

        # 中央差分计算关节速度，端点为零
        n    = len(joint_pos)
        jvel = [[0.0] * 6 for _ in range(n)]
        for i in range(1, n - 1):
            dt2 = joint_t[i+1] - joint_t[i-1]
            if dt2 > 1e-9:
                for j in range(6):
                    jvel[i][j] = (joint_pos[i+1][j] - joint_pos[i-1][j]) / dt2

        msg = JointTrajectory()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        for i in range(n):
            pt = JointTrajectoryPoint()
            pt.positions  = joint_pos[i]
            pt.velocities = jvel[i]
            ns = int(joint_t[i] * 1e9)
            pt.time_from_start = Duration(sec=ns // 1_000_000_000,
                                          nanosec=ns % 1_000_000_000)
            msg.points.append(pt)

        self._traj_pub.publish(msg)
        self._logger.info(f'solve_and_send: 下发 {n} 个路点（原 {n_raw}，降采样 1/{IK_DECIMATE}），'
                          f'时长={joint_t[-1]:.2f}s，IK 规划耗时={(time.time()-t_plan0)*1000:.0f}ms')
        return True

    # ── 球面轨道规划（Ruckig 1-DOF，从 spherical_orbit_controller 提取）─────────────
    def plan_orbit_ruckig(self, ox: float, oy: float, oz: float,
                          theta0: float, phi0: float, r0: float,
                          theta1: float, phi1: float, r1: float,
                          s_vel: float, s_acc: float, s_jerk: float,
                          cancel_event: threading.Event = None) -> bool:
        """球面轨道运镜：Ruckig 1-DOF(s) → 球坐标插值 → Cartesian → 批量 IK → JointTrajectory。

        相机始终朝向球心 (ox, oy, oz)。

        Args:
            ox, oy, oz: 球心（被摄主体位置，base_link 坐标系，米）
            theta0, phi0, r0: 起始球坐标（theta/phi 弧度，r 米）
            theta1, phi1, r1: 终止球坐标
            s_vel, s_acc, s_jerk: Ruckig 对归一化参数 s 的速度/加速/加加速限制（1/s, 1/s², 1/s³）
            cancel_event: 设置后中途放弃

        Returns:
            True 表示成功下发 JointTrajectory
        """
        if not _HAS_RUCKIG:
            self._logger.error('plan_orbit_ruckig: Ruckig 未安装')
            return False

        d_theta = theta1 - theta0
        d_phi   = phi1   - phi0
        d_r     = r1     - r0

        otg = Ruckig(1, STREAM_DT)
        inp = InputParameter(1); out = OutputParameter(1)
        inp.current_position     = [0.]
        inp.current_velocity     = [0.]
        inp.current_acceleration = [0.]
        inp.target_position      = [1.]
        inp.target_velocity      = [0.]
        inp.target_acceleration  = [0.]
        inp.max_velocity         = [float(s_vel)]
        inp.max_acceleration     = [float(s_acc)]
        inp.max_jerk             = [float(s_jerk)]

        all_pts = []; t_acc = 0.
        while True:
            if cancel_event and cancel_event.is_set():
                return False
            res = otg.update(inp, out); t_acc += STREAM_DT
            s = max(0., min(1., out.new_position[0]))

            theta = theta0 + s * d_theta
            phi   = phi0   + s * d_phi
            r     = r0     + s * d_r

            px, py, pz     = sphere_to_cart(theta, phi, r, ox, oy, oz)
            qx, qy, qz, qw = aim_quat(px, py, pz, ox, oy, oz)
            all_pts.append((t_acc, px, py, pz, qx, qy, qz, qw))
            out.pass_to_input(inp)
            if res == RuckigResult.Finished:
                break
            if res == RuckigResult.Error:
                self._logger.error('plan_orbit_ruckig: Ruckig 求解失败')
                return False

        self._logger.info(f'plan_orbit_ruckig: 生成 {len(all_pts)} 个路点，'
                          f'时长={all_pts[-1][0]:.2f}s  '
                          f'Δθ={abs(math.degrees(d_theta)):.1f}°  '
                          f'Δφ={abs(math.degrees(d_phi)):.1f}°  '
                          f'Δr={abs(d_r)*1000:.0f}mm')
        return self.solve_and_send(all_pts, cancel_event)

    # ── 关节轨迹下发（多点）─────────────────────────────────────────────────────────
    def publish_trajectory(self, points: list, joint_names: list = None) -> None:
        """下发一条 JointTrajectory。

        Args:
            points: list of dict {positions: [...], velocities: [...], time_from_start: (sec, nsec)}
            joint_names: 默认 JOINT_NAMES
        """
        if joint_names is None:
            joint_names = JOINT_NAMES

        msg = JointTrajectory()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names  = joint_names

        for p in points:
            pt = JointTrajectoryPoint()
            pt.positions = p['positions']
            pt.velocities = p.get('velocities', [0.0] * len(joint_names))
            sec, nsec = p['time_from_start']
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            msg.points.append(pt)

        self._traj_pub.publish(msg)
