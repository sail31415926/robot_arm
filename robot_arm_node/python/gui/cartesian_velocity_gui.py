#!/usr/bin/env python3
"""
@file   cartesian_velocity_gui.py
@brief  eMeetArm 笛卡尔速度控制 GUI — 手动点动测试
@version 1.0
@date   2026-06-09

控制逻辑见 cartesian_velocity_controller_node.py，本文件仅 tkinter GUI 外壳：
           按住线/角速度点动按钮 → 调 node.set_jog_vel；松开 → node.stop
         显示当前末端位姿与关节角速度。

用法：
  ros2 run robot_arm_node cartesian_velocity_gui
  ros2 launch robot_arm_gazebo gazebo.launch.py controller:=velocity
  ros2 launch robot_arm_mujoco mujoco.launch.py controller:=velocity

@copyright Copyright (c) 2026 eMeet
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor

from cartesian_velocity_controller_node import (
    CartesianVelocityControllerNode,
    BASE_FRAME, EEF_LINK, JOINT_NAMES,
    DEFAULT_JOG_LIN, DEFAULT_JOG_ANG, MAX_V_LIN, MAX_V_ANG,
    VEL_CMD_TOPIC, TWIST_TOPIC,
)


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    """
    GUI 分三区：
      1. 当前末端位姿（TF 实时）
      2. 线速度点动（按住按钮 → 发送，松开 → 停止）
      3. 角速度点动（同上）
    底部：紧急停止 + 状态栏
    """

    def __init__(self, root: tk.Tk, node: CartesianVelocityControllerNode,
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
    node     = CartesianVelocityControllerNode(gui_q)
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
