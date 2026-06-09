#!/usr/bin/env python3
"""
@file   spherical_orbit_gui.py
@brief  eMeetArm 球面坐标环绕运镜 GUI
@version 1.1
@date   2026-06-09

控制逻辑见 spherical_orbit_controller_node.py，本文件仅 tkinter GUI 外壳：
         设定被摄主体位置 + 球面坐标起止点 → 调 node 规划执行球面轨道运镜。

用法：
  ros2 run robot_arm_node spherical_orbit_gui
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
from rclpy.executors import MultiThreadedExecutor

from spherical_orbit_controller_node import (
    SphericalOrbitControllerNode,
    BASE_FRAME, EEF_LINK,
    DEFAULT_S_VEL, DEFAULT_S_ACC, DEFAULT_S_JERK,
    DEFAULT_V_POS, DEFAULT_A_POS, DEFAULT_J_POS,
    DEFAULT_V_ORI, DEFAULT_A_ORI, DEFAULT_J_ORI,
    sphere_to_cart, cart_to_sphere,
)


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: SphericalOrbitControllerNode, gui_q: queue.Queue):
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
    node     = SphericalOrbitControllerNode(gui_q)
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
