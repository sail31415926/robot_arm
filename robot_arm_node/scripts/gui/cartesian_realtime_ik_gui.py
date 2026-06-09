#!/usr/bin/env python3
"""
@file   cartesian_realtime_controller.py
@brief  eMeetArm 末端笛卡尔实时控制 GUI（IK 直发关节指令）
@version 1.1
@date   2026-06-04

拖动 X/Y/Z/Roll/Pitch/Yaw 滑块实时控制末端位姿：
           滑块变化 → pick_ik（local 模式）→ 直发 /arm_controller/joint_trajectory
         特点：
           - 响应延迟 ~50ms，适合实时交互与视觉引导
           - 无碰撞检测，无路径规划，末端走关节插值曲线
           - 与 cartesian_controller.py 互补：本脚本用于实时交互，
             后者用于精密直线路径规划
         按钮说明：
           实时模式 ON/OFF  — 开启后拖动滑块即时发送指令
           同步当前位姿     — 将当前末端位姿同步到滑块和数值框
           预备位置         — 移动到预设的 READY_POSE
           回零位           — 所有关节回到 0

用法：
  ros2 run robot_arm_node cartesian_realtime_controller
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=realtime
  ros2 launch robot_arm_bringup mujoco.launch.py controller:=realtime
  ros2 launch robot_arm_bringup real.launch.py   controller:=realtime

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

# ── 常量 ──────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'base_link'

TRAJ_DURATION  = 0.3    # s，控制器平滑插值窗口，越长运动越平滑
DEBOUNCE_MS    = 10     # ms，滑块防抖延迟，越短响应越及时

READY_POSE = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)

PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]


from arm_utils import rpy_to_quat, quat_to_rpy


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class RealtimeCartesianNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_realtime_controller',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q   = gui_q
        self._seq = 0  # 请求序号，过时响应自动丢弃

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        # 保存当前关节状态，作为 IK 初始解参考（提高连续性）
        self._joint_positions = [0.0] * 6
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

    # ── 关节状态回调 ──────────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(JOINT_NAMES):
            idx = name_to_idx.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_positions[i] = msg.position[idx]

    # ── TF → GUI 末端位姿 ─────────────────────────────────────────────────────
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

    # ── IK 请求（异步）───────────────────────────────────────────────────────
    def send_ik(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        if not self._ik_cli.service_is_ready():
            self._q.put(('status', '✗ /compute_ik 服务未就绪，请确认 MoveIt 已启动'))
            return

        self._seq += 1
        seq = self._seq

        qx, qy, qz, qw = rpy_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_FRAME
        pose_stamped.header.stamp    = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x    = x
        pose_stamped.pose.position.y    = y
        pose_stamped.pose.position.z    = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        # 以当前关节状态为初始解，提高 IK 成功率和解的连续性
        rs = RobotState()
        rs.joint_state.name     = JOINT_NAMES
        rs.joint_state.position = list(self._joint_positions)

        req = GetPositionIK.Request()
        req.ik_request.group_name      = PLANNING_GROUP
        req.ik_request.ik_link_name    = EEF_LINK
        req.ik_request.pose_stamped    = pose_stamped
        req.ik_request.robot_state      = rs
        req.ik_request.avoid_collisions = False       # 实时控制不做碰撞检测，提升速度
        req.ik_request.timeout.sec      = 0
        req.ik_request.timeout.nanosec  = 30_000_000  # local 模式 30ms 足够

        self._ik_cli.call_async(req).add_done_callback(
            lambda f, s=seq: self._on_ik_result(f, s))

    # ── IK 结果回调 → 发布关节轨迹 ───────────────────────────────────────────
    def _on_ik_result(self, future, seq):
        if seq != self._seq:
            return  # 过时响应，丢弃

        try:
            resp = future.result()
        except Exception as e:
            self._q.put(('status', f'✗ IK 调用异常: {e}'))
            return

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self._q.put(('status', f'✗ IK 无解（目标超出工作空间或奇异点附近）'))
            return

        name_to_pos = dict(zip(
            resp.solution.joint_state.name,
            resp.solution.joint_state.position))
        positions = [name_to_pos.get(n, 0.0) for n in JOINT_NAMES]

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=int(TRAJ_DURATION * 1e9))
        msg.points = [pt]
        self._traj_pub.publish(msg)

        self._q.put(('status', '● 实时运行中'))

    def go_ready(self):
        p = READY_POSE
        self.send_ik(p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw'])

    # ── 回零（直发关节零位）──────────────────────────────────────────────────
    def go_home(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * 6
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self._q.put(('status', '回零中...'))


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: RealtimeCartesianNode, gui_q: queue.Queue):
        self.root          = root
        self.node          = node
        self.gui_q         = gui_q
        self._debounce_id  = None
        self._realtime_on  = False

        root.title('eMeet 机械臂末端笛卡尔实时控制器')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 目标位姿滑块 ──────────────────────────────────────────────────────
        tf = ttk.LabelFrame(main, text='目标末端位姿（实时模式开启后拖动即发送）', padding=6)
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

            dec = 3 if unit == 'm' else 1
            ev = tk.StringVar(value=f'0.{"0"*dec}')
            self.evars.append(ev)

            def on_scale(val, e=ev, d=dec):
                e.set(f'{float(val):.{d}f}')
                self._on_slider_change()

            scale = ttk.Scale(tf, from_=lo, to=hi, orient='horizontal',
                              variable=sv, length=320, command=on_scale)
            scale.grid(row=row, column=1, padx=4, pady=3, sticky='ew')

            entry = ttk.Entry(tf, textvariable=ev, width=10, justify='right')
            entry.grid(row=row, column=2, **pad)
            ttk.Label(tf, text=unit, width=3).grid(row=row, column=3, **pad)

            def apply_entry(*_, v=sv, e=ev, lo=lo, hi=hi):
                try:
                    v.set(max(lo, min(hi, float(e.get()))))
                    self._on_slider_change()  # 手动输入也触发 IK
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

        # ── 按钮 ──────────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=6)

        self.rt_btn = ttk.Button(
            bf, text='实时模式 OFF', command=self._toggle_realtime, width=14)
        self.rt_btn.pack(side=tk.LEFT, padx=6)

        ttk.Button(bf, text='同步当前位姿', command=self._sync, width=14).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(bf, text='预 备 位 置', command=self.node.go_ready, width=14).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(bf, text='回  零  位', command=self.node.go_home, width=14).pack(
            side=tk.LEFT, padx=6)

        # ── 状态栏 ────────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(
            value='实时模式已关闭  |  /compute_ik → /arm_controller/joint_trajectory')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._poll()

    # ── 实时模式开关 ──────────────────────────────────────────────────────────
    def _toggle_realtime(self):
        self._realtime_on = not self._realtime_on
        if self._realtime_on:
            self.rt_btn.configure(text='实时模式  ON')
            ready = self.node._ik_cli.service_is_ready()
            if ready:
                self.status_var.set('● 实时模式已开启  |  拖动滑块即时发送 IK 指令')
            else:
                self.status_var.set('⚠ 实时模式已开启，但 /compute_ik 不可用（MoveIt 未启动？）')
            print(f'[realtime] ON, /compute_ik ready={ready}')
        else:
            self.rt_btn.configure(text='实时模式 OFF')
            if self._debounce_id:
                self.root.after_cancel(self._debounce_id)
                self._debounce_id = None
            self.status_var.set('实时模式已关闭')
            print('[realtime] OFF')

    # ── 滑块防抖 ──────────────────────────────────────────────────────────────
    def _on_slider_change(self):
        if not self._realtime_on:
            return
        print(f'[slider] 变化检测，安排 IK 请求')
        self.status_var.set('⏳ 滑块变化，求解中...')
        if self._debounce_id:
            self.root.after_cancel(self._debounce_id)
        self._debounce_id = self.root.after(DEBOUNCE_MS, self._send_ik)

    def _send_ik(self):
        self._debounce_id = None
        v = [s.get() for s in self.svars]
        print(f'[send_ik] X={v[0]:.3f} Y={v[1]:.3f} Z={v[2]:.3f} '
              f'R={v[3]:.1f} P={v[4]:.1f} Yaw={v[5]:.1f}')
        self.node.send_ik(v[0], v[1], v[2], v[3], v[4], v[5])

    # ── 同步当前末端位姿到滑块 ────────────────────────────────────────────────
    def _sync(self):
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法同步')
            return
        for i, (val, (_, unit, lo, hi)) in enumerate(zip(pose, PARAMS)):
            clamped = max(lo, min(hi, val))
            self.svars[i].set(clamped)
            dec = 3 if unit == 'm' else 1
            self.evars[i].set(f'{clamped:.{dec}f}')
        self.status_var.set('已同步当前末端位姿到滑块')

    # ── queue 轮询（tkinter 主线程）──────────────────────────────────────────
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
    node     = RealtimeCartesianNode(gui_q)
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
