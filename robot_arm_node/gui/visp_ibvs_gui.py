#!/usr/bin/env python3
"""
@file visp_ibvs_gui.py
@brief visp_ibvs_control 模式专用调试 GUI

实时显示：
  - 红色方块检测状态（归一化坐标、深度）
  - 图像误差与深度误差进度条
  - 六个关节当前位置；云台 J4-J6 限位使用率及动态权重
  - 期望目标参数（可调后一键推送到 visp_ibvs_node）

话题订阅：
  /red_detector/feature   (geometry_msgs/PointStamped)
  /joint_states           (sensor_msgs/JointState)

参数推送目标节点：/visp_ibvs_node

@version 1.0
@date 2026-06-16
@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tkinter as tk
from tkinter import ttk

# ── 与 visp_ibvs_node.cpp 常量保持一致 ───────────────────────────────────────
W_GIMBAL      = 3.0     # 云台基础权重
W_DYN_K       = 15.0    # 动态涨价斜率
IMG_STOP_TH   = 0.005
DEPTH_STOP_TH = 0.02
FEATURE_TIMEOUT = 1.5   # s，超时则指示灯变灰
VISP_NODE     = '/visp_ibvs_node'

JOINT_NAMES  = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
TRAJ_TOPIC   = '/arm_controller/joint_trajectory'
# 与 cartesian_velocity_controller_node.py 保持一致
READY_JOINTS = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
HOME_JOINTS  = [0.0, 0.0,  0.0, 0.0, 0.0, 0.0]

# 云台关节限位（下限, 上限）rad
GIMBAL_LIMITS = {
    'Joint4': (-3.1,    3.1   ),
    'Joint5': (-0.7854, 0.7854),
    'Joint6': (-1.5,    0.5   ),
}

# 进度条样式颜色分三档（low/mid/high 按使用率）
_S_LOW  = 'low.Horizontal.TProgressbar'
_S_MID  = 'mid.Horizontal.TProgressbar'
_S_HIGH = 'high.Horizontal.TProgressbar'
# ─────────────────────────────────────────────────────────────────────────────


def _gimbal_usage(name: str, q: float):
    """返回 (ratio 0-1, effective_weight)"""
    lo, hi = GIMBAL_LIMITS[name]
    mid        = (lo + hi) / 2
    half_range = (hi - lo) / 2
    ratio = min(abs(q - mid) / half_range, 1.0) if half_range > 1e-6 else 0.0
    w = W_GIMBAL * (1.0 + W_DYN_K * ratio * ratio)
    return ratio, w


class _DebugNode(Node):
    """轻量 ROS2 节点：订阅话题转 queue，并提供关节轨迹发布。"""

    def __init__(self, gui_q: queue.Queue):
        super().__init__('visp_ibvs_gui')
        self._q = gui_q
        self.create_subscription(PointStamped, '/red_detector/feature',
                                 self._on_feat, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joints, 10)
        self._traj_pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

    def _on_feat(self, msg: PointStamped):
        self._q.put(('feat', msg.point.x, msg.point.y, msg.point.z))

    def _on_joints(self, msg: JointState):
        self._q.put(('joints', dict(zip(msg.name, msg.position))))

    def _send_joint_traj(self, positions: list, duration_sec: int = 3):
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def go_ready(self):
        self._send_joint_traj(READY_JOINTS, duration_sec=3)

    def go_home(self):
        self._send_joint_traj(HOME_JOINTS, duration_sec=3)


class App:
    def __init__(self, root: tk.Tk, node: '_DebugNode', gui_q: queue.Queue):
        self._root  = root
        self._node  = node
        self._q     = gui_q
        self._last_feat_t = 0.0

        root.title('visp_ibvs 调试面板')
        root.resizable(False, False)

        self._desired_x     = tk.DoubleVar(value=0.0)
        self._desired_y     = tk.DoubleVar(value=0.0)
        self._desired_depth = tk.DoubleVar(value=0.3)
        self._status_var    = tk.StringVar(value='等待节点启动…')

        self._setup_styles()
        self._build()
        self._poll()

    # ── 进度条颜色样式 ────────────────────────────────────────────────────────
    def _setup_styles(self):
        s = ttk.Style()
        s.theme_use('clam')
        s.configure(_S_LOW,  background='#4caf50', troughcolor='#e0e0e0')  # 绿
        s.configure(_S_MID,  background='#ff9800', troughcolor='#e0e0e0')  # 橙
        s.configure(_S_HIGH, background='#f44336', troughcolor='#e0e0e0')  # 红

    # ── 布局构建 ─────────────────────────────────────────────────────────────
    def _build(self):
        r = self._root
        self._build_top(r)
        self._build_errors(r)
        self._build_joints(r)
        self._build_buttons(r)
        ttk.Label(r, textvariable=self._status_var, relief='sunken',
                  anchor='w').grid(row=4, column=0, sticky='ew', padx=6, pady=(2, 4))
        r.grid_columnconfigure(0, weight=1)

    def _build_buttons(self, parent):
        bf = tk.Frame(parent)
        bf.grid(row=3, column=0, pady=(2, 4))
        tk.Button(bf, text='预 备 位 置', width=14, height=1,
                  bg='#1976d2', fg='white', font=('', 10, 'bold'),
                  relief='flat', cursor='hand2',
                  command=self._go_ready).pack(side=tk.LEFT, padx=8)
        tk.Button(bf, text='回  零  位', width=14, height=1,
                  bg='#555', fg='white', font=('', 10),
                  relief='flat', cursor='hand2',
                  command=self._go_home).pack(side=tk.LEFT, padx=8)

    def _build_top(self, parent):
        top = tk.Frame(parent)
        top.grid(row=0, column=0, sticky='ew', padx=6, pady=4)

        # ── 检测状态 ──────────────────────────────────────────────────────────
        det = ttk.LabelFrame(top, text='检测状态')
        det.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))

        self._indicator = tk.Label(det, text='●', font=('', 18), fg='gray')
        self._indicator.grid(row=0, column=0, columnspan=2, pady=(4, 2))

        self._det_vars: dict[str, tk.StringVar] = {}
        for i, (lbl, key) in enumerate([('深度 (m)', 'z'),
                                         ('归一化 x', 'x'),
                                         ('归一化 y', 'y')], 1):
            ttk.Label(det, text=lbl + ':').grid(row=i, column=0, sticky='e', padx=6, pady=2)
            v = tk.StringVar(value='—')
            ttk.Entry(det, textvariable=v, width=10, state='readonly').grid(
                row=i, column=1, padx=4, pady=2)
            self._det_vars[key] = v

        # ── 期望目标 ──────────────────────────────────────────────────────────
        des = ttk.LabelFrame(top, text='期望目标（可调后点应用）')
        des.pack(side=tk.LEFT, fill=tk.Y, padx=6)

        fields = [
            ('图像位置 x',  self._desired_x,     -1.0,  1.0, 0.05),
            ('图像位置 y',  self._desired_y,     -1.0,  1.0, 0.05),
            ('距离   (m)',  self._desired_depth,  0.05, 2.0, 0.05),
        ]
        for i, (lbl, var, lo, hi, inc) in enumerate(fields):
            ttk.Label(des, text=lbl + ':').grid(row=i, column=0, sticky='e', padx=6, pady=3)
            sb = ttk.Spinbox(des, from_=lo, to=hi, increment=inc,
                             textvariable=var, width=9, format='%.3f')
            sb.grid(row=i, column=1, padx=4, pady=3)
            sb.bind('<Return>', lambda _e: self._apply_desired())

        ttk.Button(des, text='✔  应用', command=self._apply_desired).grid(
            row=len(fields), column=0, columnspan=2, pady=(6, 4), ipadx=8)

    def _build_errors(self, parent):
        fr = ttk.LabelFrame(parent, text='控制误差')
        fr.grid(row=1, column=0, sticky='ew', padx=6, pady=2)

        self._img_err_var   = tk.StringVar(value='—')
        self._depth_err_var = tk.StringVar(value='—')
        self._img_bar       = ttk.Progressbar(fr, length=220, maximum=1.0,
                                               style=_S_LOW)
        self._depth_bar     = ttk.Progressbar(fr, length=220, maximum=1.0,
                                               style=_S_LOW)

        rows = [
            ('图像误差', self._img_err_var,   self._img_bar,   IMG_STOP_TH),
            ('深度误差', self._depth_err_var, self._depth_bar, DEPTH_STOP_TH),
        ]
        for i, (lbl, var, bar, thr) in enumerate(rows):
            ttk.Label(fr, text=lbl + ':',  width=8).grid(row=i, column=0, padx=6, pady=3,
                                                          sticky='e')
            ttk.Entry(fr, textvariable=var, width=9, state='readonly').grid(
                row=i, column=1, padx=4)
            bar.grid(row=i, column=2, padx=6, pady=3)
            ttk.Label(fr, text=f'阈值 {thr}', foreground='gray',
                      font=('', 8)).grid(row=i, column=3, padx=4)

    def _build_joints(self, parent):
        fr = ttk.LabelFrame(parent, text='关节状态  |  机械臂 J1-J3  ·  云台 J4-J6')
        fr.grid(row=2, column=0, sticky='ew', padx=6, pady=4)

        for col, txt in enumerate(['关节', '位置(rad)',
                                    '关节', '位置(rad)', '限位使用率', '动态权重']):
            ttk.Label(fr, text=txt, font=('', 9, 'bold')).grid(
                row=0, column=col, padx=8, pady=(4, 2))
        ttk.Separator(fr, orient='horizontal').grid(
            row=1, column=0, columnspan=6, sticky='ew', padx=4)

        self._jpos: dict[str, tk.StringVar] = {}
        self._gbars: dict[str, ttk.Progressbar] = {}
        self._gw_vars: dict[str, tk.StringVar]  = {}

        arm_joints    = JOINT_NAMES[:3]
        gimbal_joints = JOINT_NAMES[3:]

        for row_idx, (arm, gim) in enumerate(zip(arm_joints, gimbal_joints), 2):
            # 机械臂列
            ttk.Label(fr, text=arm, width=7).grid(row=row_idx, column=0, padx=6, pady=3)
            av = tk.StringVar(value='  0.000')
            ttk.Entry(fr, textvariable=av, width=9, state='readonly').grid(
                row=row_idx, column=1, padx=4)
            self._jpos[arm] = av

            # 云台列
            ttk.Label(fr, text=gim, width=7).grid(row=row_idx, column=2, padx=6)
            gv = tk.StringVar(value='  0.000')
            ttk.Entry(fr, textvariable=gv, width=9, state='readonly').grid(
                row=row_idx, column=3, padx=4)
            self._jpos[gim] = gv

            bar = ttk.Progressbar(fr, length=130, maximum=100.0, style=_S_LOW)
            bar.grid(row=row_idx, column=4, padx=6, pady=3)
            self._gbars[gim] = bar

            wv = tk.StringVar(value='w=3.0')
            ttk.Label(fr, textvariable=wv, width=10, anchor='center').grid(
                row=row_idx, column=5, padx=6)
            self._gw_vars[gim] = wv

    # ── 位置指令 ──────────────────────────────────────────────────────────────
    def _go_ready(self):
        threading.Thread(target=self._send_pose, args=(READY_JOINTS, '预备位置', 3), daemon=True).start()

    def _go_home(self):
        threading.Thread(target=self._send_pose, args=(HOME_JOINTS, '零位', 3), daemon=True).start()

    def _send_pose(self, joints: list, label: str, duration_sec: int):
        """后台线程：暂停 IBVS → 发轨迹 → 等待完成 → 恢复 IBVS。"""
        # 1. 暂停控制循环，防止每 20ms 覆盖目标轨迹
        subprocess.run(['ros2', 'param', 'set', VISP_NODE, 'paused', 'true'],
                       capture_output=True)
        self._root.after(0, lambda: self._status_var.set(f'移动至{label}…（{duration_sec} s）'))

        # 2. 发送轨迹
        self._node._send_joint_traj(joints, duration_sec=duration_sec)

        # 3. 等待轨迹执行完毕（多留 0.5 s 余量）
        time.sleep(duration_sec + 0.5)

        # 4. 恢复控制循环
        subprocess.run(['ros2', 'param', 'set', VISP_NODE, 'paused', 'false'],
                       capture_output=True)
        self._root.after(0, lambda: self._status_var.set(f'已到达{label}，IBVS 已恢复'))

    # ── 参数推送 ──────────────────────────────────────────────────────────────
    def _apply_desired(self):
        x = self._desired_x.get()
        y = self._desired_y.get()
        d = self._desired_depth.get()
        for param, val in [('desired_x', x), ('desired_y', y), ('desired_depth', d)]:
            subprocess.Popen(
                ['ros2', 'param', 'set', VISP_NODE, param, f'{val:.4f}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._status_var.set(
            f'已推送 → desired_x={x:.3f}  desired_y={y:.3f}  desired_depth={d:.3f} m')

    # ── 100 ms 轮询 ───────────────────────────────────────────────────────────
    def _poll(self):
        now = time.monotonic()

        while True:
            try:
                msg = self._q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == 'feat':
                _, x, y, z = msg
                self._last_feat_t = now

                self._det_vars['x'].set(f'{x:+.4f}')
                self._det_vars['y'].set(f'{y:+.4f}')
                self._det_vars['z'].set(f'{z:.4f}')
                self._indicator.config(fg='#22aa22')

                dx  = self._desired_x.get()
                dy  = self._desired_y.get()
                dz  = max(self._desired_depth.get(), 1e-3)
                img_err   = math.hypot(x - dx, y - dy)
                depth_err = abs(math.log(max(z, 1e-3) / dz))

                self._img_err_var.set(f'{img_err:.5f}')
                self._depth_err_var.set(f'{depth_err:.5f}')
                self._img_bar['value']   = min(img_err,   1.0)
                self._depth_bar['value'] = min(depth_err, 1.0)
                self._img_bar.configure(
                    style=_S_LOW if img_err < 0.05 else (_S_MID if img_err < 0.2 else _S_HIGH))
                self._depth_bar.configure(
                    style=_S_LOW if depth_err < 0.1 else (_S_MID if depth_err < 0.3 else _S_HIGH))

            elif msg[0] == 'joints':
                joints: dict = msg[1]
                for name in JOINT_NAMES:
                    q = joints.get(name)
                    if q is not None and name in self._jpos:
                        self._jpos[name].set(f'{q:+.4f}')

                for gim in ['Joint4', 'Joint5', 'Joint6']:
                    q = joints.get(gim, 0.0)
                    ratio, w = _gimbal_usage(gim, q)
                    pct = ratio * 100.0
                    self._gbars[gim]['value'] = pct
                    self._gbars[gim].configure(
                        style=_S_LOW if pct < 50 else (_S_MID if pct < 80 else _S_HIGH))
                    self._gw_vars[gim].set(f'w={w:.1f}')

        # 检测超时：指示灯变灰
        if now - self._last_feat_t > FEATURE_TIMEOUT and self._last_feat_t > 0:
            self._indicator.config(fg='gray')
            for k in self._det_vars:
                self._det_vars[k].set('—')

        self._root.after(100, self._poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q = queue.Queue()
    node  = _DebugNode(gui_q)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    root = tk.Tk()
    App(root, node, gui_q)
    root.mainloop()

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
