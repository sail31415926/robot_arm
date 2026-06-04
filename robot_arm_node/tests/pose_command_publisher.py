#!/usr/bin/env python3
"""
@file   pose_command_publisher.py
@brief  ArmPoseCommand 发布测试工具（GUI）
@version 1.0
@date   2026-06-04

发布 /robot_arm/pose_command，配合 pose_command_debug 执行器使用。
支持 STOWED / OBSERVE / SHOOTING 三种姿态、SLOW / NORMAL / FAST 速度档，
以及 pan_deg / tilt_deg 云台角度。

用法（配套使用，需分别启动）：
  # 终端 1：启动仿真 + 执行端（订阅话题，驱动机械臂）
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=pose_command_debug
  # 终端 2：启动本发布 GUI（发送指令）
  ros2 run robot_arm_node pose_command_publisher

@copyright Copyright (c) 2026 eMeet
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

try:
    from robot_arm_interfaces.msg import ArmPoseCommand
except ImportError as e:
    raise SystemExit('✗ 未找到 robot_arm_interfaces.msg，请先构建：colcon build --packages-select robot_arm_interfaces') from e

POSE_CMD_TOPIC = '/robot_arm/pose_command'

STATE_LABELS = [
    (ArmPoseCommand.POSE_STATE_STOWED,   'STOWED  (0)  收纳位'),
    (ArmPoseCommand.POSE_STATE_OBSERVE,  'OBSERVE (1)  观察位'),
    (ArmPoseCommand.POSE_STATE_SHOOTING, 'SHOOTING(2)  拍摄位  ← 需填写 XYZ'),
]
SPEED_LABELS = [
    (ArmPoseCommand.SPEED_SLOW,   'SLOW   (0)  缓慢'),
    (ArmPoseCommand.SPEED_NORMAL, 'NORMAL (1)  正常'),
    (ArmPoseCommand.SPEED_FAST,   'FAST   (2)  快速'),
]


# ── ROS 节点（只发布） ────────────────────────────────────────────────────────
class PublisherNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('pose_command_publisher',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q   = gui_q
        self._pub = self.create_publisher(ArmPoseCommand, POSE_CMD_TOPIC, 10)
        self.create_subscription(ArmPoseCommand, POSE_CMD_TOPIC, self._on_echo, 10)

    def publish(self, state: int, speed: int,
                tx: float, ty: float, tz: float,
                pan_deg: float, tilt_deg: float):
        msg = ArmPoseCommand()
        msg.target_pose_state = state
        msg.transition_speed  = speed
        msg.target_pose_x     = float(tx)
        msg.target_pose_y     = float(ty)
        msg.target_pose_z     = float(tz)
        msg.pan_deg           = float(pan_deg)
        msg.tilt_deg          = float(tilt_deg)
        self._pub.publish(msg)

    def _on_echo(self, msg: ArmPoseCommand):
        state_name = {0: 'STOWED', 1: 'OBSERVE', 2: 'SHOOTING'}.get(
            msg.target_pose_state, str(msg.target_pose_state))
        speed_name = {0: 'SLOW', 1: 'NORMAL', 2: 'FAST'}.get(
            msg.transition_speed, str(msg.transition_speed))
        info = f'✓ 已发布: {state_name}  速度={speed_name}'
        if msg.target_pose_state == ArmPoseCommand.POSE_STATE_SHOOTING:
            info += (f'  xyz=({msg.target_pose_x:.3f},'
                     f'{msg.target_pose_y:.3f},{msg.target_pose_z:.3f})'
                     f'  pan={msg.pan_deg:.1f}°  tilt={msg.tilt_deg:.1f}°')
        self._q.put(info)


# ── GUI ───────────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: PublisherNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('ArmPoseCommand 发布测试工具')
        root.resizable(False, False)

        pad  = dict(padx=8, pady=4)
        main = ttk.Frame(root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 目标姿态 ──────────────────────────────────────────────────────────
        sf = ttk.LabelFrame(main, text='目标姿态  target_pose_state', padding=8)
        sf.pack(fill=tk.X, **pad)
        self.state_var = tk.IntVar(value=ArmPoseCommand.POSE_STATE_OBSERVE)
        for val, label in STATE_LABELS:
            ttk.Radiobutton(sf, text=label, variable=self.state_var, value=val,
                            command=self._on_state_change).pack(
                anchor='w', padx=6, pady=2)

        # ── SHOOTING 目标坐标 ─────────────────────────────────────────────────
        self._xyz_frame = ttk.LabelFrame(
            main,
            text='拍摄位坐标  target_pose_x/y/z（base_link，仅 SHOOTING 有效）',
            padding=8)
        self._xyz_frame.pack(fill=tk.X, **pad)
        self.xyz_vars     = {}
        self._xyz_widgets = []
        for col, (k, default) in enumerate([('X', 0.30), ('Y', 0.00), ('Z', 0.50)]):
            ttk.Label(self._xyz_frame, text=f'{k} (m):', width=6,
                      anchor='e').grid(row=0, column=col*2, sticky='e', padx=6)
            var = tk.DoubleVar(value=default)
            self.xyz_vars[k] = var
            sp = ttk.Spinbox(self._xyz_frame, from_=-1.5, to=1.5, increment=0.01,
                             textvariable=var, width=10, format='%.3f')
            sp.grid(row=0, column=col*2+1, padx=4, pady=6)
            self._xyz_widgets.append(sp)

        # ── 云台角度（pan / tilt） ────────────────────────────────────────────
        pf = ttk.LabelFrame(main, text='云台角度  pan_deg / tilt_deg', padding=8)
        pf.pack(fill=tk.X, **pad)
        self.pan_var  = tk.DoubleVar(value=0.0)
        self.tilt_var = tk.DoubleVar(value=0.0)
        for col, (label, var, lo, hi) in enumerate([
            ('Pan (°)',  self.pan_var,  -180.0, 180.0),
            ('Tilt (°)', self.tilt_var,  -90.0,  45.0),
        ]):
            ttk.Label(pf, text=label, width=8, anchor='e').grid(
                row=0, column=col*2, sticky='e', padx=6)
            ttk.Spinbox(pf, from_=lo, to=hi, increment=1.0,
                        textvariable=var, width=10, format='%.1f').grid(
                row=0, column=col*2+1, padx=4, pady=6)

        # ── 速度 ──────────────────────────────────────────────────────────────
        vf = ttk.LabelFrame(main, text='转换速度  transition_speed', padding=8)
        vf.pack(fill=tk.X, **pad)
        self.speed_var = tk.IntVar(value=ArmPoseCommand.SPEED_NORMAL)
        spf = ttk.Frame(vf)
        spf.pack(anchor='w')
        for val, label in SPEED_LABELS:
            ttk.Radiobutton(spf, text=label, variable=self.speed_var,
                            value=val).pack(side=tk.LEFT, padx=14, pady=4)

        # ── 发布按钮 ──────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(bf, text='发  布  ArmPoseCommand',
                   command=self._publish, width=28).pack(pady=2)

        # ── 话题 + 状态 ───────────────────────────────────────────────────────
        ttk.Label(main, text=f'话题: {POSE_CMD_TOPIC}',
                  foreground='gray').pack(anchor='w', padx=8, pady=(4, 0))
        self.status_var = tk.StringVar(value='就绪')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._on_state_change()
        self._poll()

    def _on_state_change(self):
        is_shooting = self.state_var.get() == ArmPoseCommand.POSE_STATE_SHOOTING
        state = 'normal' if is_shooting else 'disabled'
        for w in self._xyz_widgets:
            w.configure(state=state)

    def _publish(self):
        self.node.publish(
            state    = self.state_var.get(),
            speed    = self.speed_var.get(),
            tx       = self.xyz_vars['X'].get(),
            ty       = self.xyz_vars['Y'].get(),
            tz       = self.xyz_vars['Z'].get(),
            pan_deg  = self.pan_var.get(),
            tilt_deg = self.tilt_var.get(),
        )

    def _poll(self):
        try:
            while True:
                self.status_var.set(self.gui_q.get_nowait())
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q    = queue.Queue()
    node     = PublisherNode(gui_q)
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
