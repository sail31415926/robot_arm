#!/usr/bin/env python3
"""
@file   cartesian_moveit_gui.py
@brief  eMeetArm 末端笛卡尔空间精密路径控制 GUI (tkinter)
@version 2.1
@date   2026-06-09

控制逻辑见 cartesian_moveit_controller_node.py，本文件仅 tkinter GUI 外壳：
           X/Y/Z/Roll/Pitch/Yaw 滑块设定目标 → 调 node 精密笛卡尔直线运动
         按钮说明：
           执行         — 规划并执行当前滑块目标位姿
           同步当前位姿 — 将当前末端位姿同步到滑块
           预备位置     — 移动到预设的 READY_POSE
           回零位       — 所有关节回到 0（关节空间规划）

用法：
  ros2 run robot_arm_node cartesian_moveit_gui
  ros2 launch robot_arm_gazebo gazebo.launch.py controller:=cartesian
  ros2 launch robot_arm_mujoco mujoco.launch.py controller:=cartesian
  ros2 launch robot_arm_bringup real.launch.py   controller:=cartesian

@copyright Copyright (c) 2026 eMeet
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor

from cartesian_moveit_controller_node import (
    CartesianMoveitControllerNode,
    PARAMS, BASE_FRAME, EEF_LINK,
    DEFAULT_VEL, DEFAULT_ACC, DEFAULT_MAX_STEP,
)


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: CartesianMoveitControllerNode, gui_q: queue.Queue):
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
    node     = CartesianMoveitControllerNode(gui_q)
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
