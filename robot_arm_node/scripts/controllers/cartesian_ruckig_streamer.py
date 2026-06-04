#!/usr/bin/env python3
"""
@file   cartesian_ruckig_streamer.py
@brief  eMeetArm 笛卡尔 Ruckig 流式控制器（对接 MoveIt Servo，TwistStamped）
@version 1.1
@date   2026-06-04

Ruckig OTG 在笛卡尔空间生成 jerk-limited 轨迹，按固定频率（默认 100Hz）
         流式发布 TwistStamped 到 /servo_node/delta_twist_cmds。
         MoveIt Servo 内部做 DLS 雅可比反解 + 奇异点处理 + 关节限位，
         最终输出 JointTrajectory 到 /arm_controller/joint_trajectory。

         数据流：
           Ruckig(4-DOF) → 笛卡尔 Twist → MoveIt Servo → JointTrajectory
                                         (DLS 雅可比)    → /arm_controller

         规划维度：
           位置  3-DOF Ruckig: x, y, z      → linear velocity
           姿态  1-DOF Ruckig: s ∈ [0, θ]   → angular velocity = ṡ × n̂
           n̂ 为 q_start → q_end 的固定旋转轴（SLERP 性质）

         前置要求：
           - MoveIt 已启动（servo 需要 planning scene）
           - servo_node 已启动并 start_servo（launch 已自动）
           - arm_controller (joint_trajectory_controller) active
           - pip install ruckig

         与其他脚本的区别：
           cartesian_controller.py             → MoveIt 直线规划，整段下发
           cartesian_realtime_controller.py    → 滑块即时 IK，无平滑约束
           本脚本                              → Ruckig 平滑流 + Servo 笛卡尔闭环

         按钮说明：
           执  行       — 用当前 TF 位姿作起点，按 Ruckig 规划流式运动到目标
           停  止       — 立即停止流（发零速 Twist 让 Servo 停止）
           同步当前位姿 — 当前末端位姿写回目标滑块
           预 备 位 置  — 用 Ruckig 移动到 READY_POSE

用法：
  ros2 run robot_arm_node cartesian_ruckig_streamer
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=ruckig
  ros2 launch robot_arm_bringup mujoco.launch.py controller:=ruckig

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
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer

try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
except ImportError as e:
    raise SystemExit(
        '✗ 未找到 ruckig 库，请先安装：pip install ruckig'
    ) from e


# ── 常量 ──────────────────────────────────────────────────────────────────────
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'base_link'

# MoveIt Servo 默认 topic / service（节点名 servo_node）
TWIST_TOPIC          = '/servo_node/delta_twist_cmds'
START_SERVO_SERVICE  = '/servo_node/start_servo'

# 流式发布周期（Ruckig 步长）。100Hz 与 servo_config.yaml 的 publish_period 匹配
STREAM_DT = 0.01   # s

# 默认运动学限制（位置 / 姿态）
DEFAULT_V_POS = 0.10   # m/s
DEFAULT_A_POS = 0.50   # m/s²
DEFAULT_J_POS = 2.00   # m/s³
DEFAULT_V_ORI = 0.50   # rad/s
DEFAULT_A_ORI = 2.00   # rad/s²
DEFAULT_J_ORI = 5.00   # rad/s³

READY_POSE = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)

# (名称, 单位, 最小值, 最大值)
PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]


from arm_utils import rpy_to_quat, quat_to_rpy, quat_normalize, quat_dot


def quat_angle(q0, q1):
    """两个单位四元数间的最短旋转角 θ ∈ [0, π]"""
    d = abs(quat_dot(quat_normalize(q0), quat_normalize(q1)))
    d = max(-1.0, min(1.0, d))
    return 2.0 * math.acos(d)


def quat_rotation_axis(q0, q1):
    """
    q0 → q1 的旋转轴 n̂（单位向量，世界坐标系下表示）和总转角 θ。
    SLERP 性质：q(t) 是绕 n̂ 的等角速旋转，故 ω(t) = ṡ · n̂。
    退化情况（θ≈0）返回 ((0,0,0), 0)，调用方应跳过角速度。
    """
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    # 选最短路径：dot < 0 时翻转 q1
    if quat_dot(q0, q1) < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
    # q_rel = q1 · q0^(-1) = q1 · conj(q0)
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    cx, cy, cz, cw = -x0, -y0, -z0, w0       # conj(q0)
    rx = w1*cx + x1*cw + y1*cz - z1*cy
    ry = w1*cy - x1*cz + y1*cw + z1*cx
    rz = w1*cz + x1*cy - y1*cx + z1*cw
    rw = w1*cw - x1*cx - y1*cy - z1*cz
    rw = max(-1.0, min(1.0, rw))
    theta = 2.0 * math.acos(rw)
    sin_half = math.sqrt(max(0.0, 1.0 - rw*rw))
    if sin_half < 1e-8:
        return (0.0, 0.0, 0.0), 0.0
    return (rx / sin_half, ry / sin_half, rz / sin_half), theta


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianRuckigNode(Node):
    """
    Ruckig 在笛卡尔空间生成 jerk-limited 轨迹，按 STREAM_DT 流式发布
    TwistStamped 到 MoveIt Servo (/servo_node/delta_twist_cmds)。
    """
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_ruckig_streamer',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q = gui_q

        self.declare_parameter('twist_topic', TWIST_TOPIC)
        self.declare_parameter('stream_dt',   STREAM_DT)
        twist_topic = self.get_parameter('twist_topic').value
        self._dt    = float(self.get_parameter('stream_dt').value)

        self._twist_pub = self.create_publisher(TwistStamped, twist_topic, 10)

        # MoveIt Servo 启动服务
        self._start_servo_cli = self.create_client(Trigger, START_SERVO_SERVICE)
        self._servo_started = False
        self.create_timer(0.5, self._try_start_servo)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        # Ruckig 4-DOF: [x, y, z, s]
        self._otg = Ruckig(4, self._dt)
        self._inp = InputParameter(4)
        self._out = OutputParameter(4)

        # 流式状态
        self._stream_timer   = None
        self._halt_timer     = None             # 停止阶段持续发零速 Twist
        self._halt_count     = 0
        self._converge_timer = None             # Ruckig 完成后的笛卡尔闭环收敛阶段
        self._converge_count = 0
        self._target_xyz     = (0.0, 0.0, 0.0)  # 目标笛卡尔位置（闭环用）
        self._q_start  = (0.0, 0.0, 0.0, 1.0)   # 起始四元数
        self._q_end    = (0.0, 0.0, 0.0, 1.0)   # 目标四元数（IK 用）
        self._theta    = 0.0                    # 总旋转角
        self._axis     = (0.0, 0.0, 0.0)        # SLERP 固定旋转轴（世界系）

        self.get_logger().info(
            f'发布 Twist 到 {twist_topic}，周期 {self._dt*1000:.1f}ms '
            f'({1.0/self._dt:.0f}Hz)')

    # ── 启动 MoveIt Servo（servo 默认不接收命令，需要调用 start_servo）─────────
    def _try_start_servo(self):
        if self._servo_started:
            return
        if not self._start_servo_cli.service_is_ready():
            return
        future = self._start_servo_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_servo_resp)
        self._servo_started = True   # 标记已发请求，避免重复

    def _on_start_servo_resp(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'✓ MoveIt Servo 已启动: {resp.message}')
            else:
                self.get_logger().warn(f'⚠ start_servo 返回失败: {resp.message}')
                self._servo_started = False   # 允许重试
        except Exception as e:
            self.get_logger().warn(f'⚠ start_servo 调用异常: {e}')
            self._servo_started = False

    # ── TF → GUI 当前位姿（独立于流，便于显示） ──────────────────────────────
    def _pub_pose(self):
        pose = self.get_ee_pose()
        if pose is None:
            return
        x, y, z, r, p, yw = pose
        self._q.put(('pose', x, y, z, math.degrees(r),
                     math.degrees(p), math.degrees(yw)))

    def get_ee_pose(self):
        """返回 (x, y, z, roll_rad, pitch_rad, yaw_rad)，TF 不可用时 None"""
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, yw = quat_to_rpy(q.x, q.y, q.z, q.w)
            return tr.x, tr.y, tr.z, r, p, yw
        except Exception:
            return None

    def get_ee_quat(self):
        """返回当前末端四元数 (x, y, z, w)，TF 不可用时 None"""
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            q = t.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except Exception:
            return None

    # ── 启动一条新流 ──────────────────────────────────────────────────────────
    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        # 取 TF 当前位姿作起点（保证起始连续，不跳变）
        start = self.get_ee_pose()
        q_start = self.get_ee_quat()
        if start is None or q_start is None:
            self._q.put(('status', '✗ TF 未就绪，无法获取当前位姿作起点'))
            return

        x0, y0, z0, _, _, _ = start
        q_end = rpy_to_quat(math.radians(roll_deg),
                            math.radians(pitch_deg),
                            math.radians(yaw_deg))
        axis, theta = quat_rotation_axis(q_start, q_end)

        # Ruckig 输入：4-DOF [x, y, z, s]
        self._inp.current_position     = [x0, y0, z0, 0.0]
        self._inp.current_velocity     = [0.0, 0.0, 0.0, 0.0]
        self._inp.current_acceleration = [0.0, 0.0, 0.0, 0.0]
        self._inp.target_position      = [x,  y,  z,  theta]
        self._inp.target_velocity      = [0.0, 0.0, 0.0, 0.0]
        self._inp.target_acceleration  = [0.0, 0.0, 0.0, 0.0]
        # 第 4 个 DOF (s) 退化时给一个非零最小限，避免除零（不影响实际运动）
        s_v = v_ori if theta > 1e-6 else max(v_ori, 1e-3)
        s_a = a_ori if theta > 1e-6 else max(a_ori, 1e-3)
        s_j = j_ori if theta > 1e-6 else max(j_ori, 1e-3)
        self._inp.max_velocity         = [v_pos, v_pos, v_pos, s_v]
        self._inp.max_acceleration     = [a_pos, a_pos, a_pos, s_a]
        self._inp.max_jerk             = [j_pos, j_pos, j_pos, s_j]

        self._q_start    = q_start
        self._q_end      = q_end
        self._theta      = theta
        self._axis       = axis
        self._target_xyz = (x, y, z)            # 闭环收敛用

        # 抢占旧流（直接停掉，不走 halt 阶段，因为新流立即接管）
        self._cancel_timers()

        # 启动新流
        self._stream_timer = self.create_timer(self._dt, self._stream_step)
        dist = math.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)
        self._q.put((
            'status',
            f'● 流式运行  | Δs={dist*1000:.1f}mm  Δθ={math.degrees(theta):.1f}°  '
            f'v={v_pos:.2f}m/s  a={a_pos:.2f}m/s²  j={j_pos:.2f}m/s³'
        ))

    def go_ready(self, v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        p = READY_POSE
        self.send_goal(p['x'], p['y'], p['z'],
                       p['roll'], p['pitch'], p['yaw'],
                       v_pos, a_pos, j_pos, v_ori, a_ori, j_ori)

    def _cancel_timers(self):
        """取消所有运行中的定时器（stream / converge / halt）"""
        for attr in ('_stream_timer', '_converge_timer', '_halt_timer'):
            t = getattr(self, attr)
            if t is not None:
                t.cancel()
                t.destroy()
                setattr(self, attr, None)

    def stop_stream(self, silent=False):
        """停止运动：先停所有定时器，再启动 halt 阶段持续发零速 Twist"""
        self._cancel_timers()
        # 启动 halt 阶段：连续 ~300ms 发零速 Twist，覆盖 servo 的命令缓冲
        self._halt_count = 0
        self._halt_timer = self.create_timer(self._dt, self._halt_step)
        if not silent:
            self._q.put(('status', '■ 已停止'))

    def _halt_step(self):
        self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._halt_count += 1
        if self._halt_count >= 30:   # 30 × STREAM_DT(10ms) = 300ms，足够覆盖 servo 周期
            self._halt_timer.cancel()
            self._halt_timer.destroy()
            self._halt_timer = None

    def _publish_twist(self, vx, vy, vz, wx, wy, wz):
        msg = TwistStamped()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self._twist_pub.publish(msg)

    # ── Ruckig 单步 → 发布 TwistStamped ───────────────────────────────────────
    def _stream_step(self):
        result = self._otg.update(self._inp, self._out)

        # 取出 Ruckig 当前速度：[vx, vy, vz, ṡ]
        vx, vy, vz, s_dot = self._out.new_velocity

        # 角速度 = ṡ · n̂（SLERP 是绕固定轴的等角速旋转）
        if self._theta > 1e-6:
            ax, ay, az = self._axis
            wx, wy, wz = s_dot * ax, s_dot * ay, s_dot * az
        else:
            wx = wy = wz = 0.0

        self._publish_twist(vx, vy, vz, wx, wy, wz)

        # 准备下一步：把 out 复制进 in
        self._out.pass_to_input(self._inp)

        # 完成判定
        if result == Result.Finished:
            # Ruckig 算的速度曲线跑完了，但 servo 开环积分会有残留误差，
            # 切到笛卡尔 P 控制器闭环收敛阶段
            self._cancel_timers()
            self._converge_count = 0
            self._converge_timer = self.create_timer(self._dt, self._converge_step)
            self._q.put(('status', '⚙ 闭环收敛中...'))
        elif result == Result.Error:
            self.stop_stream(silent=True)
            self._q.put(('status', '✗ Ruckig 求解失败（限制不合理？）'))

    # ── 笛卡尔 P 控制器：闭环把残留位姿误差消掉 ──────────────────────────────
    def _converge_step(self):
        cur = self.get_ee_pose()
        cur_q = self.get_ee_quat()
        if cur is None or cur_q is None:
            return

        # 位置误差（世界系）
        dx = self._target_xyz[0] - cur[0]
        dy = self._target_xyz[1] - cur[1]
        dz = self._target_xyz[2] - cur[2]
        pos_err = math.sqrt(dx*dx + dy*dy + dz*dz)

        # 姿态误差（轴角）
        axis_err, theta_err = quat_rotation_axis(cur_q, self._q_end)

        # P 控制律
        K_lin = 3.0
        K_ang = 3.0
        vx, vy, vz = K_lin * dx, K_lin * dy, K_lin * dz
        wx = K_ang * theta_err * axis_err[0]
        wy = K_ang * theta_err * axis_err[1]
        wz = K_ang * theta_err * axis_err[2]

        # 限幅
        MAX_V = 0.05
        MAX_W = 0.3
        v_mag = math.sqrt(vx*vx + vy*vy + vz*vz)
        if v_mag > MAX_V:
            s = MAX_V / v_mag
            vx, vy, vz = vx*s, vy*s, vz*s
        w_mag = math.sqrt(wx*wx + wy*wy + wz*wz)
        if w_mag > MAX_W:
            s = MAX_W / w_mag
            wx, wy, wz = wx*s, wy*s, wz*s

        self._publish_twist(vx, vy, vz, wx, wy, wz)

        # 收敛 / 超时判定（1mm 位置 + 0.5° 姿态阈值，最多 2 秒）
        self._converge_count += 1
        converged = pos_err < 1e-3 and abs(theta_err) < math.radians(0.5)
        timeout   = self._converge_count >= 200          # 200 × 10ms = 2s

        if converged or timeout:
            self._cancel_timers()
            self._halt_count = 0
            self._halt_timer = self.create_timer(self._dt, self._halt_step)
            tag = '✓ 收敛完成' if converged else '⚠ 收敛超时'
            self._q.put((
                'status',
                f'{tag}  位置残差 {pos_err*1000:.2f}mm  姿态残差 {math.degrees(abs(theta_err)):.2f}°'
            ))


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: CartesianRuckigNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('eMeet 机械臂 Ruckig 笛卡尔流式控制器')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 目标位姿 ──────────────────────────────────────────────────────────
        tf = ttk.LabelFrame(main, text='目标末端位姿', padding=6)
        tf.pack(fill=tk.X, **pad)

        for col, h in enumerate(['自由度', '滑块', '目标值', '单位']):
            ttk.Label(tf, text=h, font=('', 9, 'bold')).grid(
                row=0, column=col, **pad, sticky='ew')

        self.svars = []
        self.evars = []

        def make_row(i, name, unit, lo, hi):
            row = i + 1
            ttk.Label(tf, text=name, width=6, anchor='center').grid(
                row=row, column=0, **pad)

            sv = tk.DoubleVar(value=0.0)
            self.svars.append(sv)

            scale = ttk.Scale(tf, from_=lo, to=hi, orient='horizontal',
                              variable=sv, length=320)
            scale.grid(row=row, column=1, padx=4, pady=3, sticky='ew')

            dec = 3 if unit == 'm' else 1
            ev = tk.StringVar(value=f'0.{"0"*dec}')
            self.evars.append(ev)

            entry = ttk.Entry(tf, textvariable=ev, width=10, justify='right')
            entry.grid(row=row, column=2, **pad)
            ttk.Label(tf, text=unit, width=3).grid(row=row, column=3, **pad)

            sv.trace_add('write', lambda *_, v=sv, e=ev, d=dec:
                         e.set(f'{v.get():.{d}f}'))

            def apply_entry(*_, v=sv, e=ev, lo=lo, hi=hi):
                try:
                    v.set(max(lo, min(hi, float(e.get()))))
                except ValueError:
                    pass
            entry.bind('<Return>',   apply_entry)
            entry.bind('<FocusOut>', apply_entry)

        for i, (name, unit, lo, hi) in enumerate(PARAMS):
            make_row(i, name, unit, lo, hi)
        tf.columnconfigure(1, weight=1)

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

        # ── 运动学限制（位置/姿态各三档） ─────────────────────────────────────
        lf = ttk.LabelFrame(main, text='Ruckig 限制：位置 (m/s, m/s², m/s³)  /  姿态 (rad/s, rad/s², rad/s³)',
                            padding=6)
        lf.pack(fill=tk.X, **pad)

        self.v_pos = tk.DoubleVar(value=DEFAULT_V_POS)
        self.a_pos = tk.DoubleVar(value=DEFAULT_A_POS)
        self.j_pos = tk.DoubleVar(value=DEFAULT_J_POS)
        self.v_ori = tk.DoubleVar(value=DEFAULT_V_ORI)
        self.a_ori = tk.DoubleVar(value=DEFAULT_A_ORI)
        self.j_ori = tk.DoubleVar(value=DEFAULT_J_ORI)

        for col, (label, var, lo, hi, inc, fmt) in enumerate([
            ('v_pos', self.v_pos, 0.001, 1.0,  0.01,  '%.3f'),
            ('a_pos', self.a_pos, 0.01,  5.0,  0.05,  '%.2f'),
            ('j_pos', self.j_pos, 0.05,  50.0, 0.25,  '%.2f'),
            ('v_ori', self.v_ori, 0.01,  3.14, 0.05,  '%.2f'),
            ('a_ori', self.a_ori, 0.05,  10.0, 0.25,  '%.2f'),
            ('j_ori', self.j_ori, 0.1,   50.0, 0.5,   '%.2f'),
        ]):
            ttk.Label(lf, text=f'{label}:').grid(row=0, column=col*2, sticky='e', padx=2)
            ttk.Spinbox(lf, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=7, format=fmt).grid(
                row=0, column=col*2+1, padx=2)

        # ── 按钮 ──────────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=6)

        ttk.Button(bf, text='执  行', command=self._execute, width=12).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(bf, text='停  止', command=self.node.stop_stream, width=12).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(bf, text='同步当前位姿', command=self._sync, width=14).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(bf, text='预 备 位 置', command=self._ready, width=14).pack(
            side=tk.LEFT, padx=4)

        # ── 状态栏 ────────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(
            value=f'就绪  |  Twist → {TWIST_TOPIC}  (MoveIt Servo)')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._poll()

        # 启动后自动移动到 READY_POSE（等 TF 就绪，给 servo 一些预热时间）
        self.root.after(3000, self._auto_initial_ready)

    def _auto_initial_ready(self):
        """启动后自动触发"预备位置"。TF 没就绪则 1 秒后重试。"""
        if self.node.get_ee_pose() is None:
            self.root.after(1000, self._auto_initial_ready)
            return
        self.status_var.set('🚀 启动后自动移到 READY_POSE...')
        self._ready()

    # ── 按钮回调 ──────────────────────────────────────────────────────────────
    def _limits(self):
        return (self.v_pos.get(), self.a_pos.get(), self.j_pos.get(),
                self.v_ori.get(), self.a_ori.get(), self.j_ori.get())

    def _execute(self):
        v = [s.get() for s in self.svars]
        self.node.send_goal(v[0], v[1], v[2], v[3], v[4], v[5], *self._limits())

    def _ready(self):
        self.node.go_ready(*self._limits())

    def _sync(self):
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法同步')
            return
        x, y, z, r, p, yw = pose
        deg = (x, y, z, math.degrees(r), math.degrees(p), math.degrees(yw))
        for i, (val, (_, unit, lo, hi)) in enumerate(zip(deg, PARAMS)):
            clamped = max(lo, min(hi, val))
            self.svars[i].set(clamped)
            dec = 3 if unit == 'm' else 1
            self.evars[i].set(f'{clamped:.{dec}f}')
        self.status_var.set('已同步当前末端位姿到滑块')

    # ── queue 轮询 ────────────────────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                item = self.gui_q.get_nowait()
                if item[0] == 'status':
                    self.status_var.set(item[1])
                elif item[0] == 'pose':
                    _, x, y, z, roll, pitch, yaw = item
                    self.cvars['X'].set(f'{x:.4f}')
                    self.cvars['Y'].set(f'{y:.4f}')
                    self.cvars['Z'].set(f'{z:.4f}')
                    self.cvars['Roll'].set(f'{roll:.2f}')
                    self.cvars['Pitch'].set(f'{pitch:.2f}')
                    self.cvars['Yaw'].set(f'{yaw:.2f}')
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q    = queue.Queue()
    node     = CartesianRuckigNode(gui_q)
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
