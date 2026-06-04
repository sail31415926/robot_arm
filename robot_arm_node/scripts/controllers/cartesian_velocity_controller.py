#!/usr/bin/env python3
"""
@file   cartesian_velocity_controller.py
@brief  eMeetArm 笛卡尔速度控制器 — IBVS 接口验证 / 手动点动测试
@version 1.0
@date   2026-06-04

数据流（两种输入，共用同一输出）：

           A. 外部 IBVS：
              /arm_vel_cmd (TwistStamped)
                ↓ 看门狗 + 限幅

           B. GUI 手动点动：
              按住按钮 → 对应轴速度
                ↓

           /servo_node/delta_twist_cmds (100Hz)

         下游两种模式（本节点无需关心）：
           - MuJoCo 仿真：mujoco_node 直接订阅此 topic，内部做 DLS 雅可比反解
                         → 无需 MoveIt，无需 servo_node
                         → ros2 launch robot_arm_bringup mujoco.launch.py controller:=velocity
           - 真机/Gazebo：MoveIt Servo 订阅此 topic，输出 JointTrajectory
                         → 需要 MoveIt + servo_node 已启动

         看门狗：/arm_vel_cmd 超过 WD_TIMEOUT 秒未更新 → 自动发零速停止

用法：
  ros2 run robot_arm_node cartesian_velocity_controller
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=velocity
  ros2 launch robot_arm_bringup mujoco.launch.py controller:=velocity

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer


# ── 常量 ──────────────────────────────────────────────────────────────────────
EEF_LINK   = 'tool0'
BASE_FRAME = 'base_link'

JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
READY_POSE     = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)
READY_JOINTS   = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]   # READY_POSE 对应的关节角（IK 不可用时的回退）

TWIST_TOPIC         = '/servo_node/delta_twist_cmds'
TRAJ_TOPIC          = '/arm_controller/joint_trajectory'
VEL_CMD_TOPIC       = '/arm_vel_cmd'
START_SERVO_SERVICE = '/servo_node/start_servo'

STREAM_DT  = 0.01   # s，100Hz 发布周期
WD_TIMEOUT = 1.0    # s，IBVS 指令看门狗超时（ibvs_controller 200ms 心跳，留足余量）

MAX_V_LIN = 0.30    # m/s   线速度上限（接收外部指令时裁剪）
MAX_V_ANG = 1.00    # rad/s 角速度上限

DEFAULT_JOG_LIN = 0.05   # m/s   GUI 点动默认线速度
DEFAULT_JOG_ANG = 0.20   # rad/s GUI 点动默认角速度


from arm_utils import rpy_to_quat, quat_to_rpy


def clamp(v, limit):
    return max(-limit, min(limit, v))


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianVelocityNode(Node):
    """
    100Hz 速度流节点。

    接受两种速度来源：
      - set_jog_vel()：GUI 按钮点动调用
      - /arm_vel_cmd 订阅：外部 IBVS 控制器输入（带看门狗）

    统一输出到 /servo_node/delta_twist_cmds（MoveIt Servo）。
    """

    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_velocity_controller',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q = gui_q

        self._lock            = threading.Lock()
        self._vel             = [0.0] * 6   # [vx, vy, vz, wx, wy, wz]
        self._cmd_time        = 0.0
        self._source          = 'idle'      # 'jog' | 'ibvs' | 'idle'
        self._suppress_stream = False       # 发轨迹时暂停速度流，避免 servo 冲突

        self._twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self._traj_pub  = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        self.create_subscription(
            TwistStamped, VEL_CMD_TOPIC, self._on_vel_cmd, 10)

        self._start_cli     = self.create_client(Trigger, START_SERVO_SERVICE)
        self._stop_cli      = self.create_client(Trigger, '/servo_node/stop_servo')
        self._servo_started = False
        self.create_timer(0.5, self._try_start_servo)

        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self._joint_positions  = [0.0] * 6
        self._joint_velocities = [0.0] * 6
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        self.create_timer(STREAM_DT, self._stream_step)

        self.get_logger().info(
            f'CartesianVelocityController 已启动 | '
            f'IBVS输入={VEL_CMD_TOPIC} | Servo输出={TWIST_TOPIC} | '
            f'看门狗={WD_TIMEOUT}s')

    # ── MoveIt Servo 启动（Gazebo/真机模式；MuJoCo 模式下服务不存在，跳过即可）──
    def _try_start_servo(self):
        if self._servo_started:
            return
        if not self._start_cli.service_is_ready():
            # MuJoCo 模式下 servo_node 不存在，首次检测后停止轮询并标记就绪
            self._no_servo_count = getattr(self, '_no_servo_count', 0) + 1
            if self._no_servo_count == 4:   # ~2s 后确认服务不存在
                self._servo_started = True  # 停止轮询
                self._q.put(('status', '就绪（MuJoCo 直驱模式，无需 servo_node）'))
            return
        self._start_cli.call_async(Trigger.Request()).add_done_callback(
            self._on_start_servo)
        self._servo_started = True

    def _on_start_servo(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('✓ MoveIt Servo 已启动')
                self._q.put(('status', '✓ MoveIt Servo 就绪'))
            else:
                self.get_logger().warn(f'⚠ start_servo 失败: {resp.message}')
                self._servo_started = False
        except Exception as e:
            self.get_logger().warn(f'⚠ start_servo 异常: {e}')
            self._servo_started = False

    # ── 外部 IBVS 速度输入 ────────────────────────────────────────────────────
    def _on_vel_cmd(self, msg: TwistStamped):
        vx = clamp(msg.twist.linear.x,  MAX_V_LIN)
        vy = clamp(msg.twist.linear.y,  MAX_V_LIN)
        vz = clamp(msg.twist.linear.z,  MAX_V_LIN)
        wx = clamp(msg.twist.angular.x, MAX_V_ANG)
        wy = clamp(msg.twist.angular.y, MAX_V_ANG)
        wz = clamp(msg.twist.angular.z, MAX_V_ANG)
        with self._lock:
            self._vel      = [vx, vy, vz, wx, wy, wz]
            self._cmd_time = time.monotonic()
            self._source   = 'ibvs'

    # ── GUI 点动接口（由 GUI 线程调用）──────────────────────────────────────
    def set_jog_vel(self, vx=0., vy=0., vz=0., wx=0., wy=0., wz=0.):
        with self._lock:
            self._vel      = [vx, vy, vz, wx, wy, wz]
            self._cmd_time = time.monotonic()
            self._source   = 'jog'

    def stop(self):
        with self._lock:
            self._vel    = [0.0] * 6
            self._source = 'idle'

    def _on_joint_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        vels = []
        for i, name in enumerate(JOINT_NAMES):
            idx = name_to_idx.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_positions[i] = msg.position[idx]
            vel = 0.0
            if idx is not None and idx < len(msg.velocity):
                vel = msg.velocity[idx]
                self._joint_velocities[i] = vel
            vels.append(vel)
        self._q.put(('jvel', *vels))

    def go_ready(self):
        self.stop()
        self._start_suppress()
        if not self._ik_cli.service_is_ready():
            # MuJoCo 直驱模式：无 MoveIt，回退到预计算关节角
            threading.Timer(1.2, lambda: self._send_joint_traj(
                READY_JOINTS, duration_sec=3)).start()
            threading.Timer(5.5, self._end_suppress).start()
            self._q.put(('status', '移动至预备位置（关节空间，无 IK）...'))
            return
        p = READY_POSE
        qx, qy, qz, qw = rpy_to_quat(
            math.radians(p['roll']), math.radians(p['pitch']), math.radians(p['yaw']))

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id        = BASE_FRAME
        pose_stamped.header.stamp           = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x        = p['x']
        pose_stamped.pose.position.y        = p['y']
        pose_stamped.pose.position.z        = p['z']
        pose_stamped.pose.orientation.x     = qx
        pose_stamped.pose.orientation.y     = qy
        pose_stamped.pose.orientation.z     = qz
        pose_stamped.pose.orientation.w     = qw

        rs = RobotState()
        rs.joint_state.name     = JOINT_NAMES
        rs.joint_state.position = list(self._joint_positions)

        req = GetPositionIK.Request()
        req.ik_request.group_name       = PLANNING_GROUP
        req.ik_request.ik_link_name     = EEF_LINK
        req.ik_request.pose_stamped     = pose_stamped
        req.ik_request.robot_state      = rs
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec      = 0
        req.ik_request.timeout.nanosec  = 50_000_000

        self._ik_cli.call_async(req).add_done_callback(self._on_ready_ik)
        self._q.put(('status', '求解预备位置 IK...（速度流已暂停）'))

    def _on_ready_ik(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self._q.put(('status', f'✗ IK 调用异常: {e}'))
            return
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self._q.put(('status', '✗ IK 无解（目标超出工作空间？）'))
            return
        name_to_pos = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
        positions = [name_to_pos.get(n, 0.0) for n in JOINT_NAMES]
        threading.Timer(1.2, lambda: self._send_joint_traj(
            positions, duration_sec=3)).start()
        threading.Timer(5.5, self._end_suppress).start()
        self._q.put(('status', '移动至预备位置...'))

    def go_home(self):
        self.stop()
        self._start_suppress()
        threading.Timer(1.2, lambda: self._send_joint_traj(
            [0.0] * 6, duration_sec=3)).start()
        threading.Timer(5.5, self._end_suppress).start()
        self._q.put(('status', '回零中...（速度流已暂停）'))

    def _send_joint_traj(self, positions, duration_sec: int):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    # ── 速度流暂停（发轨迹时使用）────────────────────────────────────────────
    def _start_suppress(self):
        with self._lock:
            self._suppress_stream = True
        # 显式停止 servo，防止 servo 最后一帧轨迹覆盖我们的目标轨迹
        if self._stop_cli.service_is_ready():
            self._stop_cli.call_async(Trigger.Request())

    def _end_suppress(self):
        with self._lock:
            self._suppress_stream = False
        # 重新启动 servo
        if self._start_cli.service_is_ready():
            self._servo_started = False   # 允许 _try_start_servo 重新调用
            self._start_cli.call_async(Trigger.Request())

    # ── 100Hz 发布步骤 ────────────────────────────────────────────────────────
    def _stream_step(self):
        with self._lock:
            suppress = self._suppress_stream
            vel      = list(self._vel)
            source   = self._source
            t        = self._cmd_time

        # 暂停中：不向 servo 发命令，让 servo 的命令超时机制自然停止输出
        if suppress:
            return

        if source == 'ibvs' and (time.monotonic() - t) > WD_TIMEOUT:
            vel = [0.0] * 6
            with self._lock:
                self._vel    = vel
                self._source = 'idle'
            self._q.put(('status', '⚠ IBVS 指令超时，已停止'))

        self._publish_twist(*vel)

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

    # ── TF → GUI 位姿 ────────────────────────────────────────────────────────
    def _pub_pose(self):
        pose = self.get_ee_pose()
        if pose is None:
            return
        x, y, z, r, p, yw = pose
        self._q.put(('pose', x, y, z,
                     math.degrees(r), math.degrees(p), math.degrees(yw)))

    def get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, yw = quat_to_rpy(q.x, q.y, q.z, q.w)
            return tr.x, tr.y, tr.z, r, p, yw
        except Exception:
            return None


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    """
    GUI 分三区：
      1. 当前末端位姿（TF 实时）
      2. 线速度点动（按住按钮 → 发送，松开 → 停止）
      3. 角速度点动（同上）
    底部：紧急停止 + 状态栏
    """

    def __init__(self, root: tk.Tk, node: CartesianVelocityNode,
                 gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('eMeet 机械臂笛卡尔速度控制器')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 当前末端位姿 ──────────────────────────────────────────────────────
        cf = ttk.LabelFrame(
            main, text=f'当前末端位姿（{BASE_FRAME} → {EEF_LINK}，TF 实时）',
            padding=6)
        cf.pack(fill=tk.X, **pad)

        self.cvars = {}
        for col, (k, u) in enumerate([('X', 'm'), ('Y', 'm'), ('Z', 'm'),
                                       ('Roll', '°'), ('Pitch', '°'), ('Yaw', '°')]):
            ttk.Label(cf, text=f'{k}({u}):').grid(
                row=0, column=col*2, sticky='e', padx=4)
            v = tk.StringVar(value='--')
            self.cvars[k] = v
            ttk.Entry(cf, textvariable=v, width=9, state='readonly',
                      justify='center').grid(row=0, column=col*2+1, padx=2)

        # ── 关节角速度（实时）────────────────────────────────────────────────
        jf = ttk.LabelFrame(main, text='关节角速度 (rad/s，/joint_states 实时)', padding=6)
        jf.pack(fill=tk.X, **pad)

        self.jvel_vars = []
        for col, name in enumerate(JOINT_NAMES):
            ttk.Label(jf, text=f'{name}:').grid(row=0, column=col*2, sticky='e', padx=4)
            v = tk.StringVar(value='0.000')
            self.jvel_vars.append(v)
            ttk.Entry(jf, textvariable=v, width=8, state='readonly',
                      justify='center').grid(row=0, column=col*2+1, padx=2)

        # ── 线速度点动 ────────────────────────────────────────────────────────
        lf = ttk.LabelFrame(main, text='线速度点动（按住发送，松开停止）', padding=6)
        lf.pack(fill=tk.X, **pad)

        ttk.Label(lf, text='速度 (m/s):').grid(row=0, column=0, **pad, sticky='e')
        self.jog_lin = tk.DoubleVar(value=DEFAULT_JOG_LIN)
        ttk.Spinbox(lf, from_=0.001, to=MAX_V_LIN, increment=0.005,
                    textvariable=self.jog_lin, width=7, format='%.3f').grid(
            row=0, column=1, **pad)

        lin_axes = [
            ('+X', 'x', +1, 2), ('-X', 'x', -1, 3),
            ('+Y', 'y', +1, 4), ('-Y', 'y', -1, 5),
            ('+Z', 'z', +1, 6), ('-Z', 'z', -1, 7),
        ]
        for label, axis, sign, col in lin_axes:
            btn = tk.Button(lf, text=label, width=5, bg='#cce', relief='raised')
            btn.grid(row=0, column=col, padx=4, pady=4)
            btn.bind('<ButtonPress-1>',
                     lambda e, a=axis, s=sign: self._lin_press(a, s))
            btn.bind('<ButtonRelease-1>', lambda e: self.node.stop())

        # ── 角速度点动 ────────────────────────────────────────────────────────
        af = ttk.LabelFrame(main, text='角速度点动（按住发送，松开停止）', padding=6)
        af.pack(fill=tk.X, **pad)

        ttk.Label(af, text='速度 (rad/s):').grid(row=0, column=0, **pad, sticky='e')
        self.jog_ang = tk.DoubleVar(value=DEFAULT_JOG_ANG)
        ttk.Spinbox(af, from_=0.01, to=MAX_V_ANG, increment=0.02,
                    textvariable=self.jog_ang, width=7, format='%.2f').grid(
            row=0, column=1, **pad)

        ang_axes = [
            ('+Rx', 'x', +1, 2), ('-Rx', 'x', -1, 3),
            ('+Ry', 'y', +1, 4), ('-Ry', 'y', -1, 5),
            ('+Rz', 'z', +1, 6), ('-Rz', 'z', -1, 7),
        ]
        for label, axis, sign, col in ang_axes:
            btn = tk.Button(af, text=label, width=5, bg='#ecc', relief='raised')
            btn.grid(row=0, column=col, padx=4, pady=4)
            btn.bind('<ButtonPress-1>',
                     lambda e, a=axis, s=sign: self._ang_press(a, s))
            btn.bind('<ButtonRelease-1>', lambda e: self.node.stop())

        # ── 功能按钮 ──────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=6)

        ttk.Button(bf, text='预 备 位 置', command=self.node.go_ready,
                   width=14).pack(side=tk.LEFT, padx=6)
        ttk.Button(bf, text='回  零  位', command=self.node.go_home,
                   width=14).pack(side=tk.LEFT, padx=6)
        tk.Button(bf, text='■  紧急停止', command=self.node.stop,
                  bg='#e33', fg='white', font=('', 10, 'bold'),
                  width=14).pack(side=tk.LEFT, padx=6)

        # ── 状态栏 ────────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(
            value=f'就绪  |  {VEL_CMD_TOPIC}  →  {TWIST_TOPIC}')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._poll()

    # ── 点动按钮回调 ──────────────────────────────────────────────────────────
    def _lin_press(self, axis, sign):
        v   = self.jog_lin.get() * sign
        kw  = {'vx': 0., 'vy': 0., 'vz': 0.}
        kw[{'x': 'vx', 'y': 'vy', 'z': 'vz'}[axis]] = v
        self.node.set_jog_vel(**kw)

    def _ang_press(self, axis, sign):
        v  = self.jog_ang.get() * sign
        kw = {'wx': 0., 'wy': 0., 'wz': 0.}
        kw[{'x': 'wx', 'y': 'wy', 'z': 'wz'}[axis]] = v
        self.node.set_jog_vel(**kw)

    # ── queue 轮询 ────────────────────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                item = self.gui_q.get_nowait()
                if item[0] == 'status':
                    self.status_var.set(item[1])
                elif item[0] == 'jvel':
                    for v, val in zip(self.jvel_vars, item[1:]):
                        v.set(f'{val:.3f}')
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
    node     = CartesianVelocityNode(gui_q)
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
