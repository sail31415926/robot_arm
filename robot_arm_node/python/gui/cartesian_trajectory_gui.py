#!/usr/bin/env python3
"""
@file   cartesian_trajectory_gui.py
@brief  eMeetArm Ruckig 笛卡尔轨迹 + 批量 IK GUI（点到点 / 环绕）
@version 1.2
@date   2026-06-09

控制逻辑见 cartesian_trajectory_controller_node.py，本文件仅 tkinter GUI 外壳：
         点到点 Tab → node.send_goal；环绕 Tab → node.send_orbit。

用法：
  ros2 run robot_arm_node cartesian_trajectory_gui
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
from rclpy.executors import MultiThreadedExecutor

from cartesian_trajectory_controller_node import (
    CartesianTrajectoryControllerNode,
    BASE_FRAME, EEF_LINK, PTP_PARAMS,
    DEFAULT_V_POS, DEFAULT_A_POS, DEFAULT_J_POS,
    DEFAULT_V_ORI, DEFAULT_A_ORI, DEFAULT_J_ORI,
    DEFAULT_W_ORB, DEFAULT_A_ORB, DEFAULT_J_ORB,
)


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: CartesianTrajectoryControllerNode, gui_q: queue.Queue):
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

        # ── 公共 停止按钮 + 状态栏 ────────────────────────────────────────────
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
    node     = CartesianTrajectoryControllerNode(gui_q)
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
