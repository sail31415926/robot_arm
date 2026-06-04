#!/usr/bin/env python3
"""
@file   spherical_orbit_streamer.py
@brief  eMeetArm 球面坐标环绕运镜控制器
@version 1.1
@date   2026-06-04

球面坐标系环绕运镜，相机始终对准被摄主体。

  球面坐标（Z-up，θ=0° 为近侧，即主体朝向世界原点方向）：
    θ ∈ (-180°, 180°)  方位角；θ=0° 相机在主体近侧（靠近底座）
                                 θ=±180° 相机在远侧（主体背后）
    φ ∈ ( -90°,  90°)  仰角；φ=0° 与主体等高，φ>0 相机在主体上方
    r > 0               轨道半径（m）

  相机世界坐标（theta_ref = atan2(-oy, -ox) 为近侧参考方位角）：
    p = o + r * ( cos(φ)·cos(theta_ref+θ),
                  cos(φ)·sin(theta_ref+θ),
                  sin(φ) )

  朝向：aim_quat，EEF X 轴朝向主体，roll 固定 90°（相机光轴沿 EEF X）。
    yaw   = atan2(dy, dx)      — 水平跟踪主体方向
    pitch = asin(-dz)          — 俯仰对准主体高度
    roll  = 90°                — 固定，匹配机械臂自然姿态

  1-DOF Ruckig 规划归一化参数 s ∈ [0, 1]，
  θ/φ/r 随 s 线性插值，整条弧线具有单一 jerk-limited S 形速度包络。

  pipeline：
    Ruckig(s) → (θ,φ,r) 线性插值 → 球坐标→笛卡尔 → aim_quat
      → /compute_ik × N（种子延续）→ JointTrajectory → /arm_controller

用法：
  ros2 run robot_arm_node spherical_orbit_streamer
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=sphere_orbit
  ros2 launch robot_arm_bringup mujoco.launch.py controller:=sphere_orbit
  ros2 launch robot_arm_bringup real.launch.py   controller:=sphere_orbit

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk

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


# ── 常量 ──────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'base_link'

STREAM_DT    = 0.01    # Ruckig 步长，100 Hz
IK_TIMEOUT_S = 0.05    # 单次 IK 超时

# 球面轨道 Ruckig 默认限制（作用于归一化参数 s，单位 1/s, 1/s², 1/s³）
DEFAULT_S_VEL  = 0.10
DEFAULT_S_ACC  = 0.10
DEFAULT_S_JERK = 1.50

# 移到起始位置 PTP 默认限制
DEFAULT_V_POS = 0.05;  DEFAULT_A_POS = 0.10;  DEFAULT_J_POS = 1.00
DEFAULT_V_ORI = 0.10;  DEFAULT_A_ORI = 0.20;  DEFAULT_J_ORI = 2.00

READY_POSE = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)


from arm_utils import rpy_to_quat, quat_to_rpy, quat_normalize, quat_dot, quat_slerp


def aim_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z):
    """EEF X 轴朝向目标，roll 固定 90°（相机光轴沿 EEF X 的安装方式）。"""
    dx, dy, dz = tgt_x - cam_x, tgt_y - cam_y, tgt_z - cam_z
    n = math.sqrt(dx*dx + dy*dy + dz*dz)
    if n < 1e-9:
        return rpy_to_quat(math.pi/2, 0., 0.)
    dx, dy, dz = dx/n, dy/n, dz/n
    pitch = math.asin(max(-1., min(1., -dz)))
    yaw   = math.atan2(dy, dx)
    return rpy_to_quat(math.pi/2, pitch, yaw)


def look_at_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z):
    """EEF Z 轴指向目标（look-at），world Z-up hint。返回 (qx,qy,qz,qw)。"""
    dx, dy, dz = tgt_x - cam_x, tgt_y - cam_y, tgt_z - cam_z
    n = math.sqrt(dx*dx + dy*dy + dz*dz)
    if n < 1e-9:
        return (0., 0., 0., 1.)
    zx, zy, zz = dx/n, dy/n, dz/n          # EEF Z（forward，指向目标）

    # world Z-up hint；φ ≈ ±90° 时退回 X 轴避免退化
    if abs(zz) < 0.999:
        ux, uy, uz = 0., 0., 1.
    else:
        ux, uy, uz = 1., 0., 0.

    # EEF X（right）= forward × up
    xx = zy*uz - zz*uy
    xy = zz*ux - zx*uz
    xz = zx*uy - zy*ux
    xn = math.sqrt(xx*xx + xy*xy + xz*xz)
    xx, xy, xz = xx/xn, xy/xn, xz/xn

    # EEF Y（up corrected）= right × forward
    yx = xy*zz - xz*zy
    yy = xz*zx - xx*zz
    yz = xx*zy - xy*zx

    # 旋转矩阵列向量（EEF X, Y, Z）→ 四元数（Shepperd）
    R = [[xx, yx, zx],
         [xy, yy, zy],
         [xz, yz, zz]]
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        s  = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2][1] - R[1][2]) * s
        qy = (R[0][2] - R[2][0]) * s
        qz = (R[1][0] - R[0][1]) * s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s  = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        qw = (R[2][1] - R[1][2]) / s
        qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s
        qz = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s  = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        qw = (R[0][2] - R[2][0]) / s
        qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s
        qz = (R[1][2] + R[2][1]) / s
    else:
        s  = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        qw = (R[1][0] - R[0][1]) / s
        qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s
        qz = 0.25 * s
    return quat_normalize((qx, qy, qz, qw))


def _theta_ref(ox, oy):
    """主体→世界原点在 XY 平面的方位角，作为 θ=0° 的参考方向（近侧）。"""
    return math.atan2(-oy, -ox)


def sphere_to_cart(theta_rad, phi_rad, r, ox, oy, oz):
    """Z-up 球坐标 → 世界系笛卡尔位置。
    theta=0  : 相机在近侧（主体朝向世界原点方向）
    theta=±π : 相机在远侧
    phi=+π/2 : 正上方，phi=-π/2 : 正下方
    """
    theta_world = _theta_ref(ox, oy) + theta_rad
    cp = math.cos(phi_rad)
    return (
        ox + r * cp * math.cos(theta_world),
        oy + r * cp * math.sin(theta_world),
        oz + r * math.sin(phi_rad),
    )


def cart_to_sphere(px, py, pz, ox, oy, oz):
    """笛卡尔位置 → Z-up 球坐标（θ 以近侧为 0°）。返回 (theta_rad, phi_rad, r)。"""
    dx, dy, dz = px - ox, py - oy, pz - oz
    r = math.sqrt(dx*dx + dy*dy + dz*dz)
    if r < 1e-9:
        return 0., 0., 0.
    phi   = math.asin(max(-1., min(1., dz / r)))
    theta = math.atan2(dy, dx) - _theta_ref(ox, oy)
    theta = (theta + math.pi) % (2 * math.pi) - math.pi   # 归一化到 (-π, π]
    return theta, phi, r


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class SphericalOrbitNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('spherical_orbit_streamer',
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

    # ── TF / 关节状态 ──────────────────────────────────────────────────────────
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
        self._q.put(('pose', x, y, z,
                     math.degrees(r), math.degrees(p), math.degrees(yw)))

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

    # ── 同步 IK ────────────────────────────────────────────────────────────────
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

    # ── 停止 ───────────────────────────────────────────────────────────────────
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
        """all_pts: list of (t, x, y, z, qx, qy, qz, qw)"""
        n_ik   = len(all_pts)
        t_traj = all_pts[-1][0]
        self._q.put(('status',
                     f'⚙ 规划中... {n_ik} 个 IK 点  {desc}  时长={t_traj:.2f}s'))

        seed      = list(self._joint_pos)
        joint_pos = []
        joint_t   = []

        for idx, (t_pt, wx, wy, wz, qx, qy, qz, qw) in enumerate(all_pts):
            if self._stop_req:
                self._q.put(('status', '■ 规划中止'))
                return
            sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, seed)
            if sol is None and idx == 0:
                sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, [0.0]*6)
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
            pt            = JointTrajectoryPoint()
            pt.positions  = joint_pos[i]
            pt.velocities = jvel[i]
            ns = int(joint_t[i] * 1e9)
            pt.time_from_start = Duration(sec=ns // 1_000_000_000,
                                          nanosec=ns % 1_000_000_000)
            msg.points.append(pt)
        self._traj_pub.publish(msg)
        self._q.put(('status',
                     f'● 执行中  {n} 个路点  时长 {t_traj:.2f}s  {desc}'))

    # ── 移到起始位置（4-DOF PTP Ruckig）──────────────────────────────────────
    def goto_start(self, ox, oy, oz, theta0_rad, phi0_rad, r0,
                   v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        px, py, pz = sphere_to_cart(theta0_rad, phi0_rad, r0, ox, oy, oz)
        q_end = aim_quat(px, py, pz, ox, oy, oz)
        threading.Thread(
            target=self._plan_ptp,
            args=(px, py, pz, q_end,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
            daemon=True).start()

    def _build_ptp_plan(self, tx, ty, tz, q_end_in,
                        v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        """构建 PTP 路径点列表，不发送。失败返回 None。"""
        start = self.get_ee_pose();  q_cur = self.get_ee_quat()
        if start is None or q_cur is None:
            self._q.put(('status', '✗ TF 未就绪')); return None
        x0, y0, z0 = start[0], start[1], start[2]
        q_end = q_end_in
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
        inp.target_position      = [tx, ty, tz, theta_total]
        inp.target_velocity      = [0., 0., 0., 0.]
        inp.target_acceleration  = [0., 0., 0., 0.]
        sv = v_ori if theta_total > 1e-6 else max(v_ori, 1e-3)
        sa = a_ori if theta_total > 1e-6 else max(a_ori, 1e-3)
        sj = j_ori if theta_total > 1e-6 else max(j_ori, 1e-3)
        inp.max_velocity     = [v_pos, v_pos, v_pos, sv]
        inp.max_acceleration = [a_pos, a_pos, a_pos, sa]
        inp.max_jerk         = [j_pos, j_pos, j_pos, sj]

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
                self._q.put(('status', '✗ Ruckig 求解失败')); return None
        return all_pts

    def _plan_ptp(self, tx, ty, tz, q_end_in,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        self._planning = True
        try:
            all_pts = self._build_ptp_plan(tx, ty, tz, q_end_in,
                                           v_pos, a_pos, j_pos,
                                           v_ori, a_ori, j_ori)
            if all_pts is None:
                return
            dist = math.sqrt((tx - all_pts[0][1])**2 +
                             (ty - all_pts[0][2])**2 +
                             (tz - all_pts[0][3])**2)
            # 计算 Δθ 用于日志
            q_cur = self.get_ee_quat()
            d = abs(quat_dot(quat_normalize(q_cur) if q_cur else (0.,0.,0.,1.),
                             quat_normalize(q_end_in)))
            ang = 2.0 * math.acos(max(-1., min(1., d)))
            self._solve_and_send(
                all_pts,
                f'移到起始位  Δs={dist*1000:.1f}mm  Δθ={math.degrees(ang):.1f}°')
        finally:
            self._planning = False

    def go_ready(self, v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        p = READY_POSE
        q = rpy_to_quat(math.radians(p['roll']),
                        math.radians(p['pitch']),
                        math.radians(p['yaw']))
        threading.Thread(
            target=self._plan_ptp,
            args=(p['x'], p['y'], p['z'], q,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
            daemon=True).start()

    # ── 球面轨道规划（1-DOF Ruckig on s）────────────────────────────────────
    def send_orbit(self, ox, oy, oz,
                   theta0_rad, phi0_rad, r0,
                   theta1_rad, phi1_rad, r1,
                   s_vel, s_acc, s_jerk):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(
            target=self._plan_orbit,
            args=(ox, oy, oz,
                  theta0_rad, phi0_rad, r0,
                  theta1_rad, phi1_rad, r1,
                  s_vel, s_acc, s_jerk),
            daemon=True).start()

    def send_orbit_chained(self, ox, oy, oz,
                           theta0_rad, phi0_rad, r0,
                           theta1_rad, phi1_rad, r1,
                           s_vel, s_acc, s_jerk,
                           v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        """先 PTP 移到球面轨道起点，再执行轨道规划。"""
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(
            target=self._plan_chained,
            args=(ox, oy, oz,
                  theta0_rad, phi0_rad, r0,
                  theta1_rad, phi1_rad, r1,
                  s_vel, s_acc, s_jerk,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
            daemon=True).start()

    def _plan_chained(self, ox, oy, oz,
                      theta0, phi0, r0,
                      theta1, phi1, r1,
                      s_vel, s_acc, s_jerk,
                      v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        self._planning = True
        try:
            # Step 1: PTP 到起点
            px, py, pz = sphere_to_cart(theta0, phi0, r0, ox, oy, oz)
            q_end = aim_quat(px, py, pz, ox, oy, oz)
            self._q.put(('status',
                         f'● 先移到起始位置 …  '
                         f'({px:.3f},{py:.3f},{pz:.3f})'))
            ptp_pts = self._build_ptp_plan(px, py, pz, q_end,
                                           v_pos, a_pos, j_pos,
                                           v_ori, a_ori, j_ori)
            if ptp_pts is None or self._stop_req:
                if not self._stop_req:
                    self._q.put(('status', '✗ 移到起始位置失败，放弃轨道执行'))
                return
            self._solve_and_send(ptp_pts, 'PTP → 起点')
            if self._stop_req:
                return

            # Step 2: 执行球面轨道
            self._q.put(('status', '● 已到达起点，开始执行球面轨道 …'))
            self._plan_orbit_inner(ox, oy, oz,
                                   theta0, phi0, r0,
                                   theta1, phi1, r1,
                                   s_vel, s_acc, s_jerk)
        finally:
            self._planning = False

    def _plan_orbit(self, ox, oy, oz,
                    theta0, phi0, r0,
                    theta1, phi1, r1,
                    s_vel, s_acc, s_jerk):
        self._planning = True
        try:
            self._plan_orbit_inner(ox, oy, oz,
                                   theta0, phi0, r0,
                                   theta1, phi1, r1,
                                   s_vel, s_acc, s_jerk)
        finally:
            self._planning = False

    def _plan_orbit_inner(self, ox, oy, oz,
                          theta0, phi0, r0,
                          theta1, phi1, r1,
                          s_vel, s_acc, s_jerk):
        d_theta = theta1 - theta0
        d_phi   = phi1   - phi0
        d_r     = r1     - r0

        otg = Ruckig(1, STREAM_DT)
        inp = InputParameter(1);  out = OutputParameter(1)
        inp.current_position     = [0.]
        inp.current_velocity     = [0.]
        inp.current_acceleration = [0.]
        inp.target_position      = [1.]
        inp.target_velocity      = [0.]
        inp.target_acceleration  = [0.]
        inp.max_velocity         = [s_vel]
        inp.max_acceleration     = [s_acc]
        inp.max_jerk             = [s_jerk]

        all_pts = [];  t_acc = 0.
        while True:
            res = otg.update(inp, out);  t_acc += STREAM_DT
            s = max(0., min(1., out.new_position[0]))

            theta = theta0 + s * d_theta
            phi   = phi0   + s * d_phi
            r     = r0     + s * d_r

            px, py, pz         = sphere_to_cart(theta, phi, r, ox, oy, oz)
            qx, qy, qz, qw     = aim_quat(px, py, pz, ox, oy, oz)
            all_pts.append((t_acc, px, py, pz, qx, qy, qz, qw))
            out.pass_to_input(inp)
            if res == Result.Finished: break
            if res == Result.Error:
                self._q.put(('status', '✗ Ruckig 求解失败')); return

        self._solve_and_send(
            all_pts,
            f'球面轨道  Δθ={abs(math.degrees(d_theta)):.1f}°  '
            f'Δφ={abs(math.degrees(d_phi)):.1f}°  '
            f'Δr={abs(d_r)*1000:.0f}mm')


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: SphericalOrbitNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('eMeet 球面轨道运镜控制器')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 当前末端位姿 ──────────────────────────────────────────────────────
        cf = ttk.LabelFrame(
            main, text=f'当前末端位姿（{BASE_FRAME} → {EEF_LINK}，TF 实时）', padding=6)
        cf.pack(fill=tk.X, **pad)
        self.cvars = {}
        for col, (k, u) in enumerate([('X','m'),('Y','m'),('Z','m'),
                                       ('Roll','°'),('Pitch','°'),('Yaw','°')]):
            ttk.Label(cf, text=f'{k}({u}):').grid(row=0, column=col*2, sticky='e', padx=4)
            v = tk.StringVar(value='--')
            self.cvars[k] = v
            ttk.Entry(cf, textvariable=v, width=9, state='readonly',
                      justify='center').grid(row=0, column=col*2+1, padx=2)

        # ── 被摄主体位置 ──────────────────────────────────────────────────────
        sf = ttk.LabelFrame(main, text='被摄主体位置（世界系）', padding=6)
        sf.pack(fill=tk.X, **pad)
        self.obj_vars = {}
        for col, (k, default) in enumerate([('ox', 0.6), ('oy', -0.12), ('oz', 0.5)]):
            ttk.Label(sf, text=f'{k}(m):').grid(row=0, column=col*2, sticky='e', padx=4)
            var = tk.DoubleVar(value=default)
            self.obj_vars[k] = var
            ttk.Spinbox(sf, from_=-2.0, to=2.0, increment=0.01,
                        textvariable=var, width=9, format='%.3f').grid(
                row=0, column=col*2+1, padx=4)
        ttk.Button(sf, text='主体位置=当前末端',
                   command=self._set_obj_from_ee).grid(row=0, column=6, padx=8)

        # ── 球面坐标参数 ──────────────────────────────────────────────────────
        bf = ttk.LabelFrame(main, text='球面坐标参数（Z-up）', padding=6)
        bf.pack(fill=tk.X, **pad)

        for col, header in enumerate(['', 'θ 方位角(°)', 'φ 仰角(°)', 'r 半径(m)']):
            ttk.Label(bf, text=header, font=('', 9, 'bold'),
                      anchor='center').grid(row=0, column=col, padx=8, pady=2)

        self.sph_vars = {}
        # 默认：起点在近侧（θ=0），终点绕 60°
        sph_defaults = {'th0': -30., 'ph0': -10., 'r0': 0.45,
                        'th1':  30., 'ph1':  30., 'r1': 0.20}
        for row, (label, th_k, ph_k, r_k) in enumerate(
                [('起点', 'th0', 'ph0', 'r0'),
                 ('终点', 'th1', 'ph1', 'r1')], start=1):
            ttk.Label(bf, text=label, width=4, anchor='e').grid(
                row=row, column=0, padx=6, pady=3, sticky='e')
            for col, (key, lo, hi, inc, fmt) in enumerate([
                (th_k, -720., 720.,  1.0, '%.1f'),
                (ph_k,  -89.,  89.,  1.0, '%.1f'),
                (r_k,   0.05,  2.0, 0.01, '%.3f'),
            ], start=1):
                var = tk.DoubleVar(value=sph_defaults[key])
                self.sph_vars[key] = var
                ttk.Spinbox(bf, from_=lo, to=hi, increment=inc,
                            textvariable=var, width=10, format=fmt).grid(
                    row=row, column=col, padx=6, pady=3)

        ttk.Button(bf, text='从当前位姿推算起点',
                   command=self._infer_start, width=18).grid(
            row=3, column=0, columnspan=4, pady=(4, 2))

        # ── 球面轨道 Ruckig 限制 ──────────────────────────────────────────────
        rl = ttk.LabelFrame(
            main,
            text='Ruckig 限制（归一化参数 s ∈[0,1]，单位 1/s · 1/s² · 1/s³）',
            padding=6)
        rl.pack(fill=tk.X, **pad)
        self.s_vel  = tk.DoubleVar(value=DEFAULT_S_VEL)
        self.s_acc  = tk.DoubleVar(value=DEFAULT_S_ACC)
        self.s_jerk = tk.DoubleVar(value=DEFAULT_S_JERK)
        for col, (label, var, lo, hi, inc) in enumerate([
            ('s_vel',  self.s_vel,  0.01, 5.0,  0.05),
            ('s_acc',  self.s_acc,  0.01, 20.0, 0.05),
            ('s_jerk', self.s_jerk, 0.05, 50.0, 0.25),
        ]):
            ttk.Label(rl, text=f'{label}:').grid(row=0, column=col*2, sticky='e', padx=6)
            ttk.Spinbox(rl, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=8, format='%.2f').grid(
                row=0, column=col*2+1, padx=4)

        # ── 移到起始位置 PTP 限制 ─────────────────────────────────────────────
        pl = ttk.LabelFrame(
            main,
            text='移到起始位置限制：位置 (m/s · m/s² · m/s³) / 姿态 (rad/s · rad/s² · rad/s³)',
            padding=6)
        pl.pack(fill=tk.X, **pad)
        self.ptp_v_pos = tk.DoubleVar(value=DEFAULT_V_POS)
        self.ptp_a_pos = tk.DoubleVar(value=DEFAULT_A_POS)
        self.ptp_j_pos = tk.DoubleVar(value=DEFAULT_J_POS)
        self.ptp_v_ori = tk.DoubleVar(value=DEFAULT_V_ORI)
        self.ptp_a_ori = tk.DoubleVar(value=DEFAULT_A_ORI)
        self.ptp_j_ori = tk.DoubleVar(value=DEFAULT_J_ORI)
        for col, (label, var, lo, hi, inc, fmt) in enumerate([
            ('v_pos', self.ptp_v_pos, 0.001, 1.0,  0.01, '%.3f'),
            ('a_pos', self.ptp_a_pos, 0.01,  5.0,  0.05, '%.2f'),
            ('j_pos', self.ptp_j_pos, 0.05,  50.0, 0.25, '%.2f'),
            ('v_ori', self.ptp_v_ori, 0.01,  3.14, 0.05, '%.2f'),
            ('a_ori', self.ptp_a_ori, 0.05,  10.0, 0.25, '%.2f'),
            ('j_ori', self.ptp_j_ori, 0.1,   50.0, 0.5,  '%.2f'),
        ]):
            ttk.Label(pl, text=f'{label}:').grid(row=0, column=col*2, sticky='e', padx=2)
            ttk.Spinbox(pl, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=7, format=fmt).grid(
                row=0, column=col*2+1, padx=2)

        # ── 按钮行 ────────────────────────────────────────────────────────────
        btf = ttk.Frame(main)
        btf.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(btf, text='预 备 位 置',
                   command=self._ready, width=14).pack(side=tk.LEFT, padx=4)
        ttk.Button(btf, text='移到起始位置',
                   command=self._goto_start, width=14).pack(side=tk.LEFT, padx=4)
        ttk.Button(btf, text='规划执行',
                   command=self._execute, width=12).pack(side=tk.LEFT, padx=4)
        ttk.Button(btf, text='■ 停  止',
                   command=self.node.stop_motion, width=12).pack(side=tk.LEFT, padx=4)

        # ── 状态栏 ────────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(
            value='就绪  |  Ruckig(s) → sphere_to_cart → look_at_quat → IK → /arm_controller')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(4, 0))

        self._poll()
        self.root.after(3000, self._auto_ready)

    # ── 辅助 ──────────────────────────────────────────────────────────────────
    def _get_obj(self):
        return (self.obj_vars['ox'].get(),
                self.obj_vars['oy'].get(),
                self.obj_vars['oz'].get())

    def _get_start_rad(self):
        return (math.radians(self.sph_vars['th0'].get()),
                math.radians(self.sph_vars['ph0'].get()),
                self.sph_vars['r0'].get())

    def _get_end_rad(self):
        return (math.radians(self.sph_vars['th1'].get()),
                math.radians(self.sph_vars['ph1'].get()),
                self.sph_vars['r1'].get())

    def _set_obj_from_ee(self):
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法设置主体坐标'); return
        self.obj_vars['ox'].set(round(pose[0], 3))
        self.obj_vars['oy'].set(round(pose[1], 3))
        self.obj_vars['oz'].set(round(pose[2], 3))
        self.status_var.set(
            f'主体位置已设为 ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})')

    def _infer_start(self):
        """从当前末端位姿反算起点球坐标；终点默认 θ₁=θ₀+90°，φ/r 不变。"""
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法推算起点'); return
        ox, oy, oz = self._get_obj()
        theta, phi, r = cart_to_sphere(pose[0], pose[1], pose[2], ox, oy, oz)
        if r < 1e-4:
            self.status_var.set('末端与主体位置重合，无法推算'); return
        th_deg = math.degrees(theta)
        ph_deg = math.degrees(phi)
        self.sph_vars['th0'].set(round(th_deg, 1))
        self.sph_vars['ph0'].set(round(ph_deg, 1))
        self.sph_vars['r0'].set(round(r, 3))
        self.sph_vars['th1'].set(round(th_deg + 90., 1))
        self.sph_vars['ph1'].set(round(ph_deg, 1))
        self.sph_vars['r1'].set(round(r, 3))
        self.status_var.set(
            f'已推算起点：θ={th_deg:.1f}°  φ={ph_deg:.1f}°  r={r:.3f}m'
            f'  （终点默认 θ₁={th_deg+90:.1f}°）')

    def _ready(self):
        self.node.go_ready(
            self.ptp_v_pos.get(), self.ptp_a_pos.get(), self.ptp_j_pos.get(),
            self.ptp_v_ori.get(), self.ptp_a_ori.get(), self.ptp_j_ori.get())

    def _auto_ready(self):
        if self.node.get_ee_pose() is None:
            self.root.after(1000, self._auto_ready); return
        self.status_var.set('自动移到 READY_POSE...')
        self._ready()

    def _goto_start(self):
        ox, oy, oz   = self._get_obj()
        th0, ph0, r0 = self._get_start_rad()
        self.node.goto_start(
            ox, oy, oz, th0, ph0, r0,
            self.ptp_v_pos.get(), self.ptp_a_pos.get(), self.ptp_j_pos.get(),
            self.ptp_v_ori.get(), self.ptp_a_ori.get(), self.ptp_j_ori.get())

    def _execute(self):
        ox, oy, oz   = self._get_obj()
        th0, ph0, r0 = self._get_start_rad()
        th1, ph1, r1 = self._get_end_rad()

        # 检查当前位置是否已在起点附近（位置 < 1 cm，姿态 < 2°）
        pose = self.node.get_ee_pose()
        need_ptp = True
        if pose is not None:
            spx, spy, spz = sphere_to_cart(th0, ph0, r0, ox, oy, oz)
            dist = math.sqrt((pose[0]-spx)**2 + (pose[1]-spy)**2 + (pose[2]-spz)**2)
            if dist < 0.01:
                need_ptp = False

        if need_ptp:
            self.status_var.set(
                f'当前位置距起点 {dist*1000:.1f}mm，先 PTP 移到起点再执行轨道 ...')
            self.node.send_orbit_chained(
                ox, oy, oz,
                th0, ph0, r0,
                th1, ph1, r1,
                self.s_vel.get(), self.s_acc.get(), self.s_jerk.get(),
                self.ptp_v_pos.get(), self.ptp_a_pos.get(), self.ptp_j_pos.get(),
                self.ptp_v_ori.get(), self.ptp_a_ori.get(), self.ptp_j_ori.get())
        else:
            self.node.send_orbit(
                ox, oy, oz,
                th0, ph0, r0,
                th1, ph1, r1,
                self.s_vel.get(), self.s_acc.get(), self.s_jerk.get())

    # ── queue 轮询 ────────────────────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                item = self.gui_q.get_nowait()
                if item[0] == 'status':
                    self.status_var.set(item[1])
                elif item[0] == 'pose':
                    _, x, y, z, roll, pitch, yaw = item
                    self.cvars['X'].set(f'{x:.4f}');    self.cvars['Y'].set(f'{y:.4f}')
                    self.cvars['Z'].set(f'{z:.4f}');    self.cvars['Roll'].set(f'{roll:.2f}')
                    self.cvars['Pitch'].set(f'{pitch:.2f}')
                    self.cvars['Yaw'].set(f'{yaw:.2f}')
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q    = queue.Queue()
    node     = SphericalOrbitNode(gui_q)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=executor.spin, daemon=True).start()

    root = tk.Tk()
    App(root, node, gui_q)
    root.mainloop()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
