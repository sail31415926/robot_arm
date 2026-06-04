#!/usr/bin/env python3
"""
@file   cartesian_ruckig_ik_streamer.py
@brief  eMeetArm Ruckig 笛卡尔轨迹 + 批量 IK 执行控制器
@version 1.2
@date   2026-06-04

两种运镜模式，共用 IK 批量求解 + JointTrajectory 下发流程：

         【点到点】Ruckig 4-DOF(x,y,z,s) → 100Hz 笛卡尔位姿序列
         【环  绕】Ruckig 1-DOF(θ) → 圆弧位置 + look-at 朝向
                         ↓ 共用
                  /compute_ik × N（pick_ik，种子延续 <2ms/次）
                  → JointTrajectory（位置+速度）→ /arm_controller

         环绕模式说明：
           - 轨道为水平圆（XY 平面），相机始终 look-at 中心点
           - 1-DOF Ruckig 规划轨道角 θ，保证角速度 jerk-limited
           - look-at：EEF z 轴指向中心，世界 Z 为 up hint

用法：
  ros2 run robot_arm_node cartesian_ruckig_ik_streamer
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=ruckig_ik
  ros2 launch robot_arm_bringup mujoco.launch.py controller:=ruckig_ik
  ros2 launch robot_arm_bringup real.launch.py   controller:=ruckig_ik

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


from arm_utils import rpy_to_quat, quat_to_rpy, quat_normalize, quat_dot, quat_slerp


def quat_mul(q1, q2):
    """四元数乘法 q1 * q2（body-frame 叠加旋转）"""
    x1, y1, z1, w1 = q1;  x2, y2, z2, w2 = q2
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def look_at_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z):
    """
    EEF z 轴从相机位置指向目标中心（look-at）。
    使用世界 Z 作为 up hint；若 forward 接近竖直则退回世界 X。
    返回 (qx, qy, qz, qw)。
    """
    dx, dy, dz = tgt_x - cam_x, tgt_y - cam_y, tgt_z - cam_z
    n = math.sqrt(dx*dx + dy*dy + dz*dz)
    if n < 1e-9:
        return (0., 0., 0., 1.)
    zx, zy, zz = dx/n, dy/n, dz/n          # z 列（forward）

    # up hint：接近竖直时换 X 轴避免退化
    if abs(zz) < 0.999:
        ux, uy, uz = 0., 0., 1.
    else:
        ux, uy, uz = 1., 0., 0.

    # x 列（right）= z × up
    xx = zy*uz - zz*uy
    xy = zz*ux - zx*uz
    xz = zx*uy - zy*ux
    xn = math.sqrt(xx*xx + xy*xy + xz*xz)
    xx, xy, xz = xx/xn, xy/xn, xz/xn

    # y 列（up corrected）= x × z  （右手系）
    yx = xy*zz - xz*zy
    yy = xz*zx - xx*zz
    yz = xx*zy - xy*zx

    # 旋转矩阵列向量 → 四元数（Shepperd's method）
    # R 列优先：R[行][列]，列 0=x, 列 1=y, 列 2=z
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


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianRuckigIKNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_ruckig_ik_streamer',
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


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: CartesianRuckigIKNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('eMeet 机械臂 Ruckig+IK 笛卡尔控制器')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 当前末端位姿（公共，两个模式都显示）─────────────────────────────
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

        # ── Notebook：点到点 / 环绕 ───────────────────────────────────────────
        nb = ttk.Notebook(main)
        nb.pack(fill=tk.BOTH, expand=True, **pad)

        self._build_ptp_tab(nb, pad)
        self._build_orbit_tab(nb, pad)

        # ── 공통 停止按钮 + 状态栏 ────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=(2, 0))
        ttk.Button(bf, text='■ 停  止', command=self.node.stop_motion,
                   width=14).pack(side=tk.LEFT, padx=4)

        self.status_var = tk.StringVar(
            value='就绪  |  Ruckig → /compute_ik → /arm_controller/joint_trajectory')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(4, 0))

        self._poll()
        self.root.after(3000, self._auto_ready)

    # ── 点到点 Tab ────────────────────────────────────────────────────────────
    def _build_ptp_tab(self, nb, pad):
        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text='  点 到 点  ')

        # 目标位姿滑块
        tf = ttk.LabelFrame(tab, text='目标末端位姿', padding=6)
        tf.pack(fill=tk.X, **pad)
        for col, h in enumerate(['自由度', '滑块', '目标值', '单位']):
            ttk.Label(tf, text=h, font=('', 9, 'bold')).grid(
                row=0, column=col, **pad, sticky='ew')

        self.svars = [];  self.evars = []

        def make_row(i, name, unit, lo, hi):
            row = i + 1
            ttk.Label(tf, text=name, width=6, anchor='center').grid(
                row=row, column=0, **pad)
            sv = tk.DoubleVar(value=0.0);  self.svars.append(sv)
            dec = 3 if unit == 'm' else 1
            ev = tk.StringVar(value=f'0.{"0"*dec}');  self.evars.append(ev)
            ttk.Scale(tf, from_=lo, to=hi, orient='horizontal',
                      variable=sv, length=300).grid(
                row=row, column=1, padx=4, pady=3, sticky='ew')
            entry = ttk.Entry(tf, textvariable=ev, width=10, justify='right')
            entry.grid(row=row, column=2, **pad)
            ttk.Label(tf, text=unit, width=3).grid(row=row, column=3, **pad)
            sv.trace_add('write', lambda *_, v=sv, e=ev, d=dec:
                         e.set(f'{v.get():.{d}f}'))
            def apply_entry(*_, v=sv, e=ev, lo=lo, hi=hi):
                try: v.set(max(lo, min(hi, float(e.get()))))
                except ValueError: pass
            entry.bind('<Return>', apply_entry);  entry.bind('<FocusOut>', apply_entry)

        for i, (name, unit, lo, hi) in enumerate(PTP_PARAMS):
            make_row(i, name, unit, lo, hi)
        tf.columnconfigure(1, weight=1)

        # Ruckig 限制
        lf = ttk.LabelFrame(tab,
            text='Ruckig 限制：位置 (m/s, m/s², m/s³)  /  姿态 (rad/s, rad/s², rad/s³)',
            padding=6)
        lf.pack(fill=tk.X, **pad)
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
            ttk.Label(lf, text=f'{label}:').grid(row=0, column=col*2, sticky='e', padx=2)
            ttk.Spinbox(lf, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=7, format=fmt).grid(
                row=0, column=col*2+1, padx=2)

        # 按钮
        bf = ttk.Frame(tab);  bf.pack(fill=tk.X, pady=6)
        ttk.Button(bf, text='规划执行', command=self._ptp_execute, width=12).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(bf, text='同步当前位姿', command=self._ptp_sync, width=14).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(bf, text='预 备 位 置', command=self._ready, width=14).pack(
            side=tk.LEFT, padx=4)

    # ── 环绕 Tab ──────────────────────────────────────────────────────────────
    def _build_orbit_tab(self, nb, pad):
        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text='  环  绕  ')

        # 轨道参数
        of = ttk.LabelFrame(tab, text='轨道参数', padding=6)
        of.pack(fill=tk.X, **pad)

        orbit_fields = [
            # (label, key, default, lo,    hi,   inc,  fmt,    unit)
            ('主体 X',     'cx',   0.6, -1.5,  1.5,  0.01, '%.3f', 'm'),
            ('主体 Y',     'cy',   -0.12, -1.5,  1.5,  0.01, '%.3f', 'm'),
            ('主体高度',   'cz',   0.47, -0.5,  1.5,  0.01, '%.3f', 'm'),
            ('轨道半径',   'r',    0.2,  0.05, 1.5,  0.01, '%.3f', 'm'),
            ('相机高于主体','h',    0.1, -0.5,  1.0,  0.01, '%.3f', 'm'),
            ('起始角',     'th0',  -30,-360.0,360.0, 1.0,  '%.1f', '°'),
            ('终止角',     'th1', 30,-360.0,360.0, 1.0,  '%.1f', '°'),
            ('Roll 固定',  'roll',   90.0,-180.0,180.0, 5.0, '%.1f', '°'),
        ]
        self.orbit_vars = {}
        for row, (label, key, default, lo, hi, inc, fmt, unit) in enumerate(orbit_fields):
            ttk.Label(of, text=label, width=10, anchor='e').grid(
                row=row, column=0, sticky='e', padx=4, pady=2)
            var = tk.DoubleVar(value=default)
            self.orbit_vars[key] = var
            sp = ttk.Spinbox(of, from_=lo, to=hi, increment=inc,
                             textvariable=var, width=10, format=fmt)
            sp.grid(row=row, column=1, padx=4, pady=2, sticky='w')
            ttk.Label(of, text=unit).grid(row=row, column=2, sticky='w', padx=2)

        # 辅助按钮行
        hf = ttk.Frame(of)
        hf.grid(row=len(orbit_fields), column=0, columnspan=3, pady=(6, 2))
        ttk.Button(hf, text='从当前位姿推算轨道', command=self._orbit_infer,
                   width=18).pack(side=tk.LEFT, padx=4)
        ttk.Button(hf, text='主体位置=当前末端', command=self._orbit_set_center,
                   width=16).pack(side=tk.LEFT, padx=4)

        # Ruckig 限制（角度空间）
        lf = ttk.LabelFrame(
            tab, text='Ruckig 限制：ω (rad/s)  /  α (rad/s²)  /  jerk (rad/s³)',
            padding=6)
        lf.pack(fill=tk.X, **pad)
        self.orb_w = tk.DoubleVar(value=DEFAULT_W_ORB)
        self.orb_a = tk.DoubleVar(value=DEFAULT_A_ORB)
        self.orb_j = tk.DoubleVar(value=DEFAULT_J_ORB)
        for col, (label, var, lo, hi, inc) in enumerate([
            ('ω_max', self.orb_w, 0.01, 3.14, 0.05),
            ('α_max', self.orb_a, 0.01, 10.0, 0.05),
            ('j_max', self.orb_j, 0.05, 50.0, 0.25),
        ]):
            ttk.Label(lf, text=f'{label}:').grid(row=0, column=col*2, sticky='e', padx=4)
            ttk.Spinbox(lf, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=8, format='%.2f').grid(
                row=0, column=col*2+1, padx=4)

        # 按钮
        bf = ttk.Frame(tab);  bf.pack(fill=tk.X, pady=6)
        ttk.Button(bf, text='规划执行', command=self._orbit_execute, width=12).pack(
            side=tk.LEFT, padx=4)

    # ── 辅助：从当前位姿推算轨道参数 ─────────────────────────────────────────
    def _orbit_infer(self):
        """
        以当前 orbit_vars 里的主体坐标为基准，
        从当前末端位姿反算 轨道半径、相机高于主体、起始角，
        并把终止角设为 起始角+90°（默认示例）。
        """
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法推算'); return
        ex, ey, ez = pose[0], pose[1], pose[2]
        cx  = self.orbit_vars['cx'].get()
        cy  = self.orbit_vars['cy'].get()
        cz  = self.orbit_vars['cz'].get()   # 主体高度
        r   = math.sqrt((ex-cx)**2 + (ey-cy)**2)
        h   = ez - cz                        # 相机高于主体
        th0 = math.degrees(math.atan2(ey - cy, ex - cx))
        self.orbit_vars['r'].set(round(r, 3))
        self.orbit_vars['h'].set(round(h, 3))
        self.orbit_vars['th0'].set(round(th0, 1))
        self.orbit_vars['th1'].set(round(th0 + 90.0, 1))
        self.status_var.set(
            f'已推算：r={r:.3f}m  h={h:+.3f}m  θ₀={th0:.1f}°  θ₁={th0+90:.1f}°')

    def _orbit_set_center(self):
        """把当前末端位置设为被摄主体坐标（XY + 主体高度）。"""
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法设置主体坐标'); return
        self.orbit_vars['cx'].set(round(pose[0], 3))
        self.orbit_vars['cy'].set(round(pose[1], 3))
        self.orbit_vars['cz'].set(round(pose[2], 3))
        self.status_var.set(
            f'主体位置已设为 ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})')

    # ── 按钮回调 ──────────────────────────────────────────────────────────────
    def _ptp_execute(self):
        v = [s.get() for s in self.svars]
        self.node.send_goal(v[0], v[1], v[2], v[3], v[4], v[5],
                            self.ptp_v_pos.get(), self.ptp_a_pos.get(), self.ptp_j_pos.get(),
                            self.ptp_v_ori.get(), self.ptp_a_ori.get(), self.ptp_j_ori.get())



    def _orbit_execute(self):
        ov = self.orbit_vars
        self.node.send_orbit(
            ov['cx'].get(), ov['cy'].get(), ov['cz'].get(),
            ov['r'].get(),  ov['h'].get(),
            ov['th0'].get(), ov['th1'].get(),
            self.orb_w.get(), self.orb_a.get(), self.orb_j.get(),
            ov['roll'].get())

    def _ready(self):
        self.node.go_ready(
            self.ptp_v_pos.get(), self.ptp_a_pos.get(), self.ptp_j_pos.get(),
            self.ptp_v_ori.get(), self.ptp_a_ori.get(), self.ptp_j_ori.get())

    def _ptp_sync(self):
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法同步'); return
        x, y, z, r, p, yw = pose
        deg = (x, y, z, math.degrees(r), math.degrees(p), math.degrees(yw))
        for i, (val, (_, unit, lo, hi)) in enumerate(zip(deg, PTP_PARAMS)):
            c = max(lo, min(hi, val))
            self.svars[i].set(c)
            dec = 3 if unit == 'm' else 1
            self.evars[i].set(f'{c:.{dec}f}')
        self.status_var.set('已同步当前末端位姿到滑块')

    def _auto_ready(self):
        if self.node.get_ee_pose() is None:
            self.root.after(1000, self._auto_ready); return
        self.status_var.set('自动移到 READY_POSE...')
        self._ready()

    # ── queue 轮询 ────────────────────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                item = self.gui_q.get_nowait()
                if item[0] == 'status':
                    self.status_var.set(item[1])
                elif item[0] == 'pose':
                    _, x, y, z, roll, pitch, yaw = item
                    self.cvars['X'].set(f'{x:.4f}');   self.cvars['Y'].set(f'{y:.4f}')
                    self.cvars['Z'].set(f'{z:.4f}');   self.cvars['Roll'].set(f'{roll:.2f}')
                    self.cvars['Pitch'].set(f'{pitch:.2f}'); self.cvars['Yaw'].set(f'{yaw:.2f}')
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q    = queue.Queue()
    node     = CartesianRuckigIKNode(gui_q)
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
