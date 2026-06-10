#!/usr/bin/env python3
"""
@file   cartesian_trajectory_controller_node.py
@brief  eMeetArm Ruckig 笛卡尔轨迹 + 批量 IK 执行 node（无 GUI，可被 GUI import 或 headless 运行）
@version 1.2
@date   2026-06-09

两种运镜模式，共用 IK 批量求解 + JointTrajectory 下发流程，不依赖任何 GUI 框架：
         【点到点】Ruckig 4-DOF(x,y,z,s) → 100Hz 笛卡尔位姿序列
         【环  绕】Ruckig 1-DOF(θ) → 圆弧位置 + look-at 朝向
                         ↓ 共用
                  /compute_ik × N（pick_ik，种子延续）→ JointTrajectory → /arm_controller
         状态/位姿通过 gui_q 队列上报（headless 运行时传空队列即可）。

被 cartesian_trajectory_gui.py（tkinter 调试 GUI）import 使用；也可独立运行：
  ros2 run robot_arm_node cartesian_trajectory_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer

try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
except ImportError as e:
    raise SystemExit('✗ 未找到 ruckig 库，请先安装：pip install ruckig') from e

from arm_utils import (rpy_to_quat, quat_to_rpy, quat_normalize, quat_dot, quat_slerp,
                        quat_mul, look_at_quat)

# ── 常量 ──────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'base_link'

STREAM_DT    = 0.01    # s，Ruckig 内部步长（100Hz）
IK_TIMEOUT_S = 0.05    # 单次 IK 超时（100ms）

# 点到点默认限制
DEFAULT_V_POS = 0.05;  DEFAULT_A_POS = 0.10;  DEFAULT_J_POS = 1.00
DEFAULT_V_ORI = 0.10;  DEFAULT_A_ORI = 0.20;  DEFAULT_J_ORI = 2.00

# 环绕默认限制（角度空间）
DEFAULT_W_ORB = 0.30   # rad/s   轨道角速度上限
DEFAULT_A_ORB = 0.30   # rad/s²
DEFAULT_J_ORB = 1.00   # rad/s³

READY_POSE = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)

PTP_PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianTrajectoryControllerNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_trajectory_controller_node',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q = gui_q

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        self._joint_pos = [0.0] * 6
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)

        self._planning = False
        self._stop_req = False

    def _on_js(self, msg: JointState):
        n2i = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(JOINT_NAMES):
            idx = n2i.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_pos[i] = msg.position[idx]

    def _pub_pose(self):
        pose = self.get_ee_pose()
        if pose is None:
            return
        x, y, z, r, p, yw = pose
        self._q.put(('pose', x, y, z, math.degrees(r),
                     math.degrees(p), math.degrees(yw)))

    def get_ee_pose(self):
        try:
            t  = self.tf_buffer.lookup_transform(BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, yw = quat_to_rpy(q.x, q.y, q.z, q.w)
            return tr.x, tr.y, tr.z, r, p, yw
        except Exception:
            return None

    def get_ee_quat(self):
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, EEF_LINK, rclpy.time.Time())
            q = t.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except Exception:
            return None

    # ── 同步 IK（后台线程专用）────────────────────────────────────────────────
    # 返回 (joint_list, error_code_val)；成功时 error_code=1，失败时 joint_list=None
    _IK_ERR = {1: 'OK', -1: 'PLANNING_FAILED', -6: 'TIMED_OUT',
               -15: 'INVALID_GROUP_NAME', -31: 'NO_IK_SOLUTION',
               99999: 'TIMEOUT_WAIT'}

    def _ik_sync(self, x, y, z, qx, qy, qz, qw, seed):
        if not self._ik_cli.service_is_ready():
            return None, -1
        ps = PoseStamped()
        ps.header.frame_id    = BASE_FRAME
        ps.header.stamp       = self.get_clock().now().to_msg()
        ps.pose.position.x    = x;  ps.pose.position.y    = y
        ps.pose.position.z    = z;  ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy; ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
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
        self._ik_cli.call_async(req).add_done_callback(_cb)
        ev.wait(timeout=IK_TIMEOUT_S + 0.05)
        if box[0] is None:
            return None, 99999
        try:
            resp = box[0].result()
        except Exception:
            return None, -1
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, resp.error_code.val
        n2p = dict(zip(resp.solution.joint_state.name,
                       resp.solution.joint_state.position))
        return [n2p.get(n, 0.0) for n in JOINT_NAMES], MoveItErrorCodes.SUCCESS

    # ── 停止 ──────────────────────────────────────────────────────────────────
    def stop_motion(self):
        self._stop_req = True
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions       = list(self._joint_pos)
        pt.velocities      = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=int(0.1e9))
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self._q.put(('status', '■ 已停止'))

    # ── 共用：IK 批量求解 + JointTrajectory 下发 ─────────────────────────────
    def _solve_and_send(self, all_pts, desc):
        """
        all_pts: list of (t, x, y, z, qx, qy, qz, qw)
        对每个点求 IK，计算关节速度，下发完整 JointTrajectory。
        """
        n_ik   = len(all_pts)
        t_traj = all_pts[-1][0]
        self._q.put(('status', f'⚙ 规划中... {n_ik} 个 IK 点  {desc}  时长={t_traj:.2f}s'))

        seed      = list(self._joint_pos)
        joint_pos = []
        joint_t   = []

        for idx, (t_pt, wx, wy, wz, qx, qy, qz, qw) in enumerate(all_pts):
            if self._stop_req:
                self._q.put(('status', '■ 规划中止'))
                return

            sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, seed)

            # 第一步失败时用零种子重试一次（种子可能离解太远）
            if sol is None and idx == 0:
                sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw,
                                         [0.0] * 6)

            if sol is None:
                err_name = self._IK_ERR.get(err, str(err))
                if joint_pos:
                    sol = joint_pos[-1]
                    self.get_logger().warn(
                        f'IK 失败 step={idx} err={err_name}  '
                        f'pos=({wx:.3f},{wy:.3f},{wz:.3f})')
                else:
                    self._q.put(('status',
                                 f'✗ IK 失败（step=0 err={err_name}）'
                                 f'  pos=({wx:.3f},{wy:.3f},{wz:.3f})'
                                 f'  quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})'))
                    return

            joint_pos.append(sol)
            joint_t.append(t_pt)
            seed = sol

            if (idx + 1) % 20 == 0:
                self._q.put(('status', f'⚙ 规划中 {idx+1}/{n_ik}...'))

        if self._stop_req:
            return

        # 中央差分计算关节速度，端点为零
        n    = len(joint_pos)
        jvel = [[0.0] * 6 for _ in range(n)]
        for i in range(1, n - 1):
            dt2 = joint_t[i+1] - joint_t[i-1]
            for j in range(6):
                jvel[i][j] = (joint_pos[i+1][j] - joint_pos[i-1][j]) / dt2

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        for i in range(n):
            pt           = JointTrajectoryPoint()
            pt.positions = joint_pos[i]
            pt.velocities = jvel[i]
            ns = int(joint_t[i] * 1e9)
            pt.time_from_start = Duration(sec=ns // 1_000_000_000,
                                          nanosec=ns % 1_000_000_000)
            msg.points.append(pt)
        self._traj_pub.publish(msg)
        self._q.put(('status', f'● 执行中  {n} 个路点  时长 {t_traj:.2f}s  {desc}'))

    # ── 点到点 ────────────────────────────────────────────────────────────────
    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(target=self._plan_ptp,
                         args=(x, y, z, roll_deg, pitch_deg, yaw_deg,
                               v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
                         daemon=True).start()

    def go_ready(self, v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        p = READY_POSE
        self.send_goal(p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw'],
                       v_pos, a_pos, j_pos, v_ori, a_ori, j_ori)

    def _plan_ptp(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        self._planning = True
        try:
            start = self.get_ee_pose();  q_cur = self.get_ee_quat()
            if start is None or q_cur is None:
                self._q.put(('status', '✗ TF 未就绪')); return
            x0, y0, z0 = start[0], start[1], start[2]
            q_end = rpy_to_quat(math.radians(roll_deg),
                                 math.radians(pitch_deg),
                                 math.radians(yaw_deg))
            if quat_dot(q_cur, q_end) < 0:
                q_end = (-q_end[0], -q_end[1], -q_end[2], -q_end[3])
            theta_total = 2.0 * math.acos(
                max(-1., min(1., abs(quat_dot(quat_normalize(q_cur),
                                              quat_normalize(q_end))))))
            otg = Ruckig(4, STREAM_DT)
            inp = InputParameter(4);  out = OutputParameter(4)
            inp.current_position     = [x0, y0, z0, 0.]
            inp.current_velocity     = [0., 0., 0., 0.]
            inp.current_acceleration = [0., 0., 0., 0.]
            inp.target_position      = [x,  y,  z,  theta_total]
            inp.target_velocity      = [0., 0., 0., 0.]
            inp.target_acceleration  = [0., 0., 0., 0.]
            s_v = v_ori if theta_total > 1e-6 else max(v_ori, 1e-3)
            s_a = a_ori if theta_total > 1e-6 else max(a_ori, 1e-3)
            s_j = j_ori if theta_total > 1e-6 else max(j_ori, 1e-3)
            inp.max_velocity     = [v_pos, v_pos, v_pos, s_v]
            inp.max_acceleration = [a_pos, a_pos, a_pos, s_a]
            inp.max_jerk         = [j_pos, j_pos, j_pos, s_j]

            all_pts = [];  t_acc = 0.
            while True:
                res = otg.update(inp, out);  t_acc += STREAM_DT
                rx, ry, rz, rs = out.new_position
                if theta_total > 1e-6:
                    t_s = max(0., min(1., rs / theta_total))
                    qx, qy, qz, qw = quat_slerp(q_cur, q_end, t_s)
                else:
                    qx, qy, qz, qw = q_end
                all_pts.append((t_acc, rx, ry, rz, qx, qy, qz, qw))
                out.pass_to_input(inp)
                if res == Result.Finished: break
                if res == Result.Error:
                    self._q.put(('status', '✗ Ruckig 求解失败')); return

            dist = math.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)
            self._solve_and_send(
                all_pts, f'Δs={dist*1000:.1f}mm  Δθ={math.degrees(theta_total):.1f}°')
        finally:
            self._planning = False

    # ── 环绕 ──────────────────────────────────────────────────────────────────
    def send_orbit(self, cx, cy, z_sub, radius, h_cam,
                   theta0_deg, theta1_deg, w_max, a_max, j_max,
                   roll_deg=0.):
        """
        cx, cy    : 被摄主体 XY 坐标
        z_sub     : 被摄主体高度（base_link z）
        radius    : 轨道半径
        h_cam     : 相机高于主体的距离（h_cam > 0 = 相机在主体上方）
        theta0/1  : 轨道起止角（度）
        """
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(target=self._plan_orbit,
                         args=(cx, cy, z_sub, radius, h_cam,
                               theta0_deg, theta1_deg, w_max, a_max, j_max,
                               roll_deg),
                         daemon=True).start()

    def _plan_orbit(self, cx, cy, z_sub, radius, h_cam,
                    theta0_deg, theta1_deg, w_max, a_max, j_max,
                    roll_deg):
        self._planning = True
        try:
            theta0    = math.radians(theta0_deg)
            theta1    = math.radians(theta1_deg)
            roll_rad  = math.radians(roll_deg)
            pitch_rad = math.atan2(h_cam, radius)
            z_cam     = z_sub + h_cam

            otg = Ruckig(1, STREAM_DT)
            inp = InputParameter(1);  out = OutputParameter(1)
            inp.current_position     = [theta0]
            inp.current_velocity     = [0.]
            inp.current_acceleration = [0.]
            inp.target_position      = [theta1]
            inp.target_velocity      = [0.]
            inp.target_acceleration  = [0.]
            inp.max_velocity         = [w_max]
            inp.max_acceleration     = [a_max]
            inp.max_jerk             = [j_max]

            all_pts = [];  t_acc = 0.
            while True:
                res = otg.update(inp, out);  t_acc += STREAM_DT
                theta = out.new_position[0]
                px = cx + radius * math.cos(theta + math.pi)
                py = cy + radius * math.sin(theta + math.pi)
                # yaw=θ 跟踪轨道角，roll/pitch 由用户固定
                qx, qy, qz, qw = rpy_to_quat(roll_rad, pitch_rad, theta)
                all_pts.append((t_acc, px, py, z_cam, qx, qy, qz, qw))
                out.pass_to_input(inp)
                if res == Result.Finished: break
                if res == Result.Error:
                    self._q.put(('status', '✗ Ruckig 求解失败')); return

            sweep = abs(theta1_deg - theta0_deg)
            self._solve_and_send(
                all_pts,
                f'r={radius:.2f}m  h={h_cam:+.2f}m  '
                f'θ:{theta0_deg:.0f}°→{theta1_deg:.0f}°({sweep:.0f}°)')
        finally:
            self._planning = False


def main():
    rclpy.init()
    node     = CartesianTrajectoryControllerNode(queue.Queue())
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
