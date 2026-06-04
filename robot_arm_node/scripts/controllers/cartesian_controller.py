#!/usr/bin/env python3
"""
@file   cartesian_controller.py
@brief  eMeetArm 末端笛卡尔空间精密路径控制 GUI (tkinter)
@version 2.1
@date   2026-06-04

通过 X/Y/Z/Roll/Pitch/Yaw 滑块设定目标末端位姿，点击执行后进行精密运动：
           compute_cartesian_path → 笛卡尔直线路径规划（末端严格走直线）
           execute_trajectory     → MoveIt 执行轨迹
         特点：
           - 末端在笛卡尔空间走直线，位置与姿态均受控
           - 支持速度/加速度缩放及插值步长调节
           - 支持指令抢占：新指令自动取消正在执行的旧指令
           - 与 cartesian_realtime_controller.py 互补：本脚本用于精密路径，
             后者用于实时交互滑块控制
         按钮说明：
           执行         — 规划并执行当前滑块目标位姿
           同步当前位姿 — 将当前末端位姿同步到滑块
           预备位置     — 移动到预设的 READY_POSE
           回零位       — 所有关节回到 0（关节空间规划）

用法：
  ros2 run robot_arm_node cartesian_controller
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian
  ros2 launch robot_arm_bringup mujoco.launch.py controller:=cartesian
  ros2 launch robot_arm_bringup real.launch.py   controller:=cartesian

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    MoveItErrorCodes, JointConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener, Buffer

# ── 参数 ──────────────────────────────────────────────────────────────────────
# (名称, 单位, 最小值, 最大值)
PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]
PLANNING_GROUP = 'arm'
EEF_LINK       = 'tool0'
BASE_FRAME     = 'base_link'

# ── 运动参数默认值（在此修改即可）────────────────────────────────────────────
DEFAULT_VEL      = 0.9    # 速度缩放比例   (0.01 ~ 1.0)
DEFAULT_ACC      = 0.5    # 加速度缩放比例 (0.01 ~ 1.0)
DEFAULT_MAX_STEP = 0.001  # 笛卡尔插值步长 m，越小越精确，规划越慢 (0.001 ~ 0.05)

# ── 预备位置（启动后自动执行）────────────────────────────────────────────────
READY_POSE = dict(x=0.3, y=0.0, z=0.6, roll=90.0, pitch=10.0, yaw=0.0)


from arm_utils import rpy_to_quat, quat_to_rpy


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianNode(Node):
    """
    笛卡尔直线运动：
      /compute_cartesian_path (service) → 生成末端严格直线轨迹
      /execute_trajectory     (action)  → 执行轨迹
    回零使用：
      /move_action            (action)  → 关节空间规划回零位
    """
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_controller',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q              = gui_q
        self._busy             = False
        self._dispatch_timer   = None
        self._retry            = 0
        self._mode             = ''       # 'cartesian' | 'home'
        self._cp_req           = None     # GetCartesianPath.Request
        self._home_mg_req      = None     # MotionPlanRequest
        self._vel_scale        = 0.1
        self._acc_scale        = 0.1
        self._seq              = 0        # 每次新指令递增，让旧回调自动失效
        self._exec_goal_handle = None     # 当前执行中的 action goal handle
        self._cp_in_flight     = False    # compute_cartesian_path 是否正在飞行中

        # 笛卡尔路径服务 + 执行 action
        self._cp_cli  = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._exec_ac = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        # 回零用 MoveGroup action
        self._mg_ac   = ActionClient(self, MoveGroup, '/move_action')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)


    # ── TF → GUI ──────────────────────────────────────────────────────────────
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

    # ── 外部调用（GUI 线程） ───────────────────────────────────────────────────
    def go_ready(self):
        p = READY_POSE
        self.send_goal(p['x'], p['y'], p['z'],
                       p['roll'], p['pitch'], p['yaw'],
                       DEFAULT_VEL, DEFAULT_ACC, DEFAULT_MAX_STEP)

    def _preempt(self):
        self._seq += 1
        self._stop_dispatch()
        if self._exec_goal_handle is not None:
            self._exec_goal_handle.cancel_goal_async()
            self._exec_goal_handle = None

    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg, vel, acc, max_step):
        self._preempt()
        self._busy      = True
        self._retry     = 0
        self._mode      = 'cartesian'
        self._vel_scale = vel
        self._acc_scale = acc
        self._cp_req    = self._build_cp_req(x, y, z, roll_deg, pitch_deg, yaw_deg, max_step)
        self._q.put(('status', '连接服务...'))
        self._dispatch_timer = self.create_timer(0.05, self._dispatch)

    def send_home(self, vel, acc):
        self._preempt()
        self._busy        = True
        self._retry       = 0
        self._mode        = 'home'
        self._vel_scale   = vel
        self._acc_scale   = acc
        self._home_mg_req = self._build_home_req(vel, acc)
        self._q.put(('status', '回零规划中...'))
        self._dispatch_timer = self.create_timer(0.05, self._dispatch)

    # ── executor 线程：等待服务/action 就绪后派发 ──────────────────────────────
    def _dispatch(self):
        seq = self._seq
        if self._mode == 'cartesian':
            ready = self._cp_cli.service_is_ready()
        else:
            ready = self._mg_ac.server_is_ready()

        if not ready:
            self._retry += 1
            if self._retry >= 60:   # 60 × 50ms = 3s
                self._stop_dispatch()
                self._busy = False
                self._q.put(('status', '✗ MoveIt 服务未响应，请确认 MoveIt 已启动'))
            return

        self._stop_dispatch()

        if self._mode == 'cartesian':
            if self._cp_in_flight:
                return  # 已有请求在途，等它返回后自动触发新请求
            self._q.put(('status', '计算笛卡尔路径...'))
            self._cp_in_flight = True
            self._cp_cli.call_async(self._cp_req).add_done_callback(
                lambda f, s=seq: self._on_cartesian_path(f, s))
        else:
            goal = MoveGroup.Goal()
            goal.request                          = self._home_mg_req
            goal.planning_options.plan_only       = False
            goal.planning_options.replan          = True
            goal.planning_options.replan_attempts = 3
            self._mg_ac.send_goal_async(goal).add_done_callback(
                lambda f, s=seq: self._on_mg_goal_resp(f, s))

    def _stop_dispatch(self):
        if self._dispatch_timer:
            self._dispatch_timer.cancel()
            self._dispatch_timer.destroy()
            self._dispatch_timer = None

    # ── 笛卡尔路径回调 ────────────────────────────────────────────────────────
    def _on_cartesian_path(self, future, seq):
        self._cp_in_flight = False
        if seq != self._seq:
            # 旧结果丢弃，但有新指令等待：立刻用最新参数重新规划
            if self._busy and self._mode == 'cartesian':
                self._stop_dispatch()
                self._q.put(('status', '计算笛卡尔路径...'))
                self._cp_in_flight = True
                cur_seq = self._seq
                self._cp_cli.call_async(self._cp_req).add_done_callback(
                    lambda f, s=cur_seq: self._on_cartesian_path(f, s))
            return
        resp = future.result()
        if resp.fraction < 0.95:
            self._busy = False
            self._q.put(('status',
                f'✗ 笛卡尔路径仅覆盖 {resp.fraction*100:.0f}%，'
                '目标可能超出工作空间或存在奇异点'))
            return

        traj = resp.solution
        self._scale_trajectory(traj, self._vel_scale)

        self._q.put(('status', f'路径覆盖 {resp.fraction*100:.0f}%，执行中...'))
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        self._exec_ac.send_goal_async(goal).add_done_callback(
            lambda f, s=seq: self._on_exec_goal_resp(f, s))

    def _on_exec_goal_resp(self, future, seq):
        if seq != self._seq:
            return
        handle = future.result()
        if not handle.accepted:
            self._busy = False
            self._q.put(('status', '✗ 执行被拒绝'))
            return
        self._exec_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self._on_exec_result(f, s))

    def _on_exec_result(self, future, seq):
        if seq != self._seq:
            return
        self._exec_goal_handle = None
        self._busy = False
        val = future.result().result.error_code.val
        self._q.put(('status', '✓ 执行完成' if val == MoveItErrorCodes.SUCCESS
                     else f'✗ 执行失败，错误码: {val}'))

    # ── 回零 MoveGroup 回调 ───────────────────────────────────────────────────
    def _on_mg_goal_resp(self, future, seq):
        if seq != self._seq:
            return
        handle = future.result()
        if not handle.accepted:
            self._busy = False
            self._q.put(('status', '✗ 回零目标被拒绝'))
            return
        self._q.put(('status', '回零执行中...'))
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self._on_mg_result(f, s))

    def _on_mg_result(self, future, seq):
        if seq != self._seq:
            return
        self._busy = False
        val = future.result().result.error_code.val
        self._q.put(('status', '✓ 回零完成' if val == MoveItErrorCodes.SUCCESS
                     else f'✗ 回零失败，错误码: {val}'))

    # ── 构造 GetCartesianPath 请求 ────────────────────────────────────────────
    def _build_cp_req(self, x, y, z, roll_deg, pitch_deg, yaw_deg, max_step):
        qx, qy, qz, qw = rpy_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        target = Pose()
        target.position.x    = x;  target.position.y    = y;  target.position.z    = z
        target.orientation.x = qx; target.orientation.y = qy
        target.orientation.z = qz; target.orientation.w = qw

        req = GetCartesianPath.Request()
        req.header.frame_id  = BASE_FRAME
        req.header.stamp     = self.get_clock().now().to_msg()
        req.group_name       = PLANNING_GROUP
        req.link_name        = EEF_LINK
        req.waypoints        = [target]
        req.max_step         = max_step
        req.jump_threshold   = 0.0
        req.avoid_collisions = True
        return req

    def _build_home_req(self, vel, acc):
        req = MotionPlanRequest()
        req.group_name                      = PLANNING_GROUP
        req.num_planning_attempts           = 5
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = vel
        req.max_acceleration_scaling_factor = acc
        c = Constraints()
        for name in ('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = 0.0
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return req

    # ── 轨迹速度缩放（compute_cartesian_path 不支持缩放参数，手动处理）─────────
    @staticmethod
    def _scale_trajectory(traj, vel_scale):
        if vel_scale <= 0.0 or vel_scale >= 1.0:
            return
        factor = 1.0 / vel_scale   # > 1，时间拉长，速度降低
        for pt in traj.joint_trajectory.points:
            ns = pt.time_from_start.sec * 1_000_000_000 + pt.time_from_start.nanosec
            ns = int(ns * factor)
            pt.time_from_start.sec     = ns // 1_000_000_000
            pt.time_from_start.nanosec = ns %  1_000_000_000
            pt.velocities     = [v / factor         for v in pt.velocities]
            pt.accelerations  = [a / (factor*factor) for a in pt.accelerations]


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: CartesianNode, gui_q: queue.Queue):
        self.root = root
        self.node = node
        self.gui_q = gui_q

        root.title('eMeet 机械臂末端笛卡尔控制器')
        root.resizable(True, False)

        pad = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 目标位姿 ──────────────────────────────────────────────────────────
        tf = ttk.LabelFrame(main, text='目标末端位姿（滑块拖动 或 直接输入后回车）', padding=6)
        tf.pack(fill=tk.X, **pad)

        for col, h in enumerate(['自由度', '滑块', '目标值', '单位']):
            ttk.Label(tf, text=h, font=('', 9, 'bold')).grid(
                row=0, column=col, **pad, sticky='ew')

        self.svars  = []   # tk.DoubleVar  for sliders
        self.evars  = []   # tk.StringVar  for entry boxes

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

            # slider → entry
            sv.trace_add('write', lambda *_, v=sv, e=ev, d=dec:
                         e.set(f'{v.get():.{d}f}'))

            # entry → slider (on Enter / focus-out)
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
        keys = [('X','m'),('Y','m'),('Z','m'),('Roll','°'),('Pitch','°'),('Yaw','°')]
        for col, (k, u) in enumerate(keys):
            ttk.Label(cf, text=f'{k}({u}):').grid(row=0, column=col*2, sticky='e', padx=4)
            v = tk.StringVar(value='--')
            self.cvars[k] = v
            ttk.Entry(cf, textvariable=v, width=9, state='readonly',
                      justify='center').grid(row=0, column=col*2+1, padx=2)

        # ── 速度/加速度缩放 ───────────────────────────────────────────────────
        sf = ttk.LabelFrame(main, text='运动缩放比例', padding=6)
        sf.pack(fill=tk.X, **pad)

        self.vel_var      = tk.DoubleVar(value=DEFAULT_VEL)
        self.acc_var      = tk.DoubleVar(value=DEFAULT_ACC)
        self.max_step_var = tk.DoubleVar(value=DEFAULT_MAX_STEP)

        for label, var, lo, hi, inc, fmt, unit in [
            ('速度',          self.vel_var,      0.01,  1.0,  0.05,  '%.2f', ''),
            ('加速度',        self.acc_var,      0.01,  1.0,  0.05,  '%.2f', ''),
            ('插值步长',      self.max_step_var, 0.001, 0.05, 0.001, '%.3f', 'm'),
        ]:
            ttk.Label(sf, text=f'{label}:').pack(side=tk.LEFT, padx=4)
            ttk.Spinbox(sf, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=8, format=fmt).pack(side=tk.LEFT, padx=2)
            if unit:
                ttk.Label(sf, text=unit).pack(side=tk.LEFT)

        # ── 按钮 ──────────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=6)

        ttk.Button(bf, text='执  行', command=self._execute, width=14).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(bf, text='同步当前位姿', command=self._sync, width=14).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(bf, text='预 备 位 置', command=self._ready, width=14).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(bf, text='回  零  位', command=self._home, width=14).pack(
            side=tk.LEFT, padx=6)

        # ── 状态栏 ────────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(value='就绪  |  /move_action  |  IK: pick_ik')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._poll()

    # ── queue 轮询（tkinter 主线程） ──────────────────────────────────────────
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

    # ── 按钮回调 ──────────────────────────────────────────────────────────────
    def _execute(self):
        v = [s.get() for s in self.svars]
        self.node.send_goal(
            v[0], v[1], v[2], v[3], v[4], v[5],
            self.vel_var.get(), self.acc_var.get(),
            self.max_step_var.get(),
        )

    def _ready(self):
        self.node.go_ready()

    def _home(self):
        self.node.send_home(self.vel_var.get(), self.acc_var.get())

    def _sync(self):
        pose = self.node.get_ee_pose()
        if pose is None:
            self.status_var.set('TF 未就绪，无法同步')
            return
        lims = [(lo, hi) for _, _, lo, hi in PARAMS]
        for i, val in enumerate(pose):
            lo, hi = lims[i]
            self.svars[i].set(max(lo, min(hi, val)))
        self.status_var.set('已同步当前末端位姿到滑块')


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q    = queue.Queue()
    node     = CartesianNode(gui_q)
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
