#!/usr/bin/env python3
"""
@file   pose_command_publisher.py
@brief  ArmMoveToPose action 客户端测试工具（GUI）
@version 2.0
@date   2026-06-09

通过 action /robot_arm/move_to_pose 发送姿态切换目标，配合 pose_command_debug
（action server）使用。支持 STOWED / OBSERVE / SHOOTING 三种姿态、
SLOW / NORMAL / FAST 速度档；SHOOTING 可指定目标 XYZ 与末端朝向 RPY。
实时显示动作的 feedback（进度）与 result（成功/退出原因）。

用法（配套使用，需分别启动）：
  # 终端 1：启动仿真 + 执行端（action server，驱动机械臂）
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=pose_command_debug
  # 终端 2：启动本发布 GUI（action client）
  ros2 run robot_arm_node pose_command_publisher

@copyright Copyright (c) 2026 eMeet
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

try:
    from robot_arm_interfaces.action import ArmMoveToPose
except ImportError as e:
    raise SystemExit('✗ 未找到 robot_arm_interfaces.action，请先构建：colcon build --packages-select robot_arm_interfaces') from e

ACTION_NAME = '/robot_arm/move_to_pose'

STATE_LABELS = [
    (ArmMoveToPose.Goal.POSE_STATE_STOWED,   'STOWED  (0)  收纳位'),
    (ArmMoveToPose.Goal.POSE_STATE_OBSERVE,  'OBSERVE (1)  观察位'),
    (ArmMoveToPose.Goal.POSE_STATE_SHOOTING, 'SHOOTING(2)  拍摄位  ← 需填写 XYZ/RPY'),
]
SPEED_LABELS = [
    (ArmMoveToPose.Goal.SPEED_SLOW,   'SLOW   (0)  缓慢'),
    (ArmMoveToPose.Goal.SPEED_NORMAL, 'NORMAL (1)  正常'),
    (ArmMoveToPose.Goal.SPEED_FAST,   'FAST   (2)  快速'),
]


# ── ROS 节点（action client） ─────────────────────────────────────────────────
class PublisherNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('pose_command_publisher',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q      = gui_q
        self._client = ActionClient(self, ArmMoveToPose, ACTION_NAME)

    # ── 发送目标 ────────────────────────────────────────────────────────────────
    def send_goal(self, state: int, speed: int,
                  tx: float, ty: float, tz: float,
                  roll_deg: float, pitch_deg: float, yaw_deg: float):
        if not self._client.wait_for_server(timeout_sec=1.0):
            self._q.put(f'✗ 未发现 action server: {ACTION_NAME}')
            return
        goal = ArmMoveToPose.Goal()
        goal.target_pose_state = state
        goal.transition_speed  = speed
        goal.target_pose.x     = float(tx)
        goal.target_pose.y     = float(ty)
        goal.target_pose.z     = float(tz)
        goal.target_pose.roll  = float(roll_deg)
        goal.target_pose.pitch = float(pitch_deg)
        goal.target_pose.yaw   = float(yaw_deg)
        self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        ).add_done_callback(self._on_goal_response)
        state_name = {0: 'STOWED', 1: 'OBSERVE', 2: 'SHOOTING'}.get(state, str(state))
        speed_name = {0: 'SLOW', 1: 'NORMAL', 2: 'FAST'}.get(speed, str(speed))
        self._q.put(f'→ 已发送目标: {state_name}  速度={speed_name}')

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._q.put('✗ 目标被拒绝（执行端忙或非法）')
            return
        self._q.put('✓ 目标已接受，执行中…')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._q.put(f'… 进度 {fb.progress_percent:5.1f}%  '
                    f'当前高度 z={fb.current_pose.z:.3f}m')

    def _on_result(self, future):
        result = future.result().result
        flag = '✓' if result.success else '✗'
        info = f'{flag} 完成: {result.exit_reason}  err={result.error_code}'
        if result.success:
            info += (f'  实际=({result.actual_pose.x:.3f},'
                     f'{result.actual_pose.y:.3f},{result.actual_pose.z:.3f})')
        self._q.put(info)


# ── GUI ───────────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: PublisherNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('ArmMoveToPose 发布测试工具')
        root.resizable(False, False)

        pad  = dict(padx=8, pady=4)
        main = ttk.Frame(root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 目标姿态 ──────────────────────────────────────────────────────────
        sf = ttk.LabelFrame(main, text='目标姿态  target_pose_state', padding=8)
        sf.pack(fill=tk.X, **pad)
        self.state_var = tk.IntVar(value=ArmMoveToPose.Goal.POSE_STATE_OBSERVE)
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
        self._cond_widgets = []
        for col, (k, default) in enumerate([('X', 0.30), ('Y', 0.00), ('Z', 0.50)]):
            ttk.Label(self._xyz_frame, text=f'{k} (m):', width=6,
                      anchor='e').grid(row=0, column=col*2, sticky='e', padx=6)
            var = tk.DoubleVar(value=default)
            self.xyz_vars[k] = var
            sp = ttk.Spinbox(self._xyz_frame, from_=-1.5, to=1.5, increment=0.01,
                             textvariable=var, width=10, format='%.3f')
            sp.grid(row=0, column=col*2+1, padx=4, pady=6)
            self._cond_widgets.append(sp)

        # ── 末端朝向（roll / pitch / yaw） ────────────────────────────────────
        rf = ttk.LabelFrame(
            main,
            text='末端朝向  roll/pitch/yaw（°，仅 SHOOTING 有效）', padding=8)
        rf.pack(fill=tk.X, **pad)
        self.rpy_vars = {}
        for col, (k, default, lo, hi) in enumerate([
            ('Roll',  90.0, -180.0, 180.0),
            ('Pitch', 10.0,  -90.0,  90.0),
            ('Yaw',    0.0, -180.0, 180.0),
        ]):
            ttk.Label(rf, text=f'{k} (°):', width=8,
                      anchor='e').grid(row=0, column=col*2, sticky='e', padx=6)
            var = tk.DoubleVar(value=default)
            self.rpy_vars[k] = var
            sp = ttk.Spinbox(rf, from_=lo, to=hi, increment=1.0,
                             textvariable=var, width=10, format='%.1f')
            sp.grid(row=0, column=col*2+1, padx=4, pady=6)
            self._cond_widgets.append(sp)

        # ── 速度 ──────────────────────────────────────────────────────────────
        vf = ttk.LabelFrame(main, text='转换速度  transition_speed', padding=8)
        vf.pack(fill=tk.X, **pad)
        self.speed_var = tk.IntVar(value=ArmMoveToPose.Goal.SPEED_NORMAL)
        spf = ttk.Frame(vf)
        spf.pack(anchor='w')
        for val, label in SPEED_LABELS:
            ttk.Radiobutton(spf, text=label, variable=self.speed_var,
                            value=val).pack(side=tk.LEFT, padx=14, pady=4)

        # ── 发送按钮 ──────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(bf, text='发  送  ArmMoveToPose 目标',
                   command=self._send, width=28).pack(pady=2)

        # ── action + 状态 ─────────────────────────────────────────────────────
        ttk.Label(main, text=f'action: {ACTION_NAME}',
                  foreground='gray').pack(anchor='w', padx=8, pady=(4, 0))
        self.status_var = tk.StringVar(value='就绪')
        ttk.Label(main, textvariable=self.status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._on_state_change()
        self._poll()

    def _on_state_change(self):
        is_shooting = self.state_var.get() == ArmMoveToPose.Goal.POSE_STATE_SHOOTING
        state = 'normal' if is_shooting else 'disabled'
        for w in self._cond_widgets:
            w.configure(state=state)

    def _send(self):
        self.node.send_goal(
            state     = self.state_var.get(),
            speed     = self.speed_var.get(),
            tx        = self.xyz_vars['X'].get(),
            ty        = self.xyz_vars['Y'].get(),
            tz        = self.xyz_vars['Z'].get(),
            roll_deg  = self.rpy_vars['Roll'].get(),
            pitch_deg = self.rpy_vars['Pitch'].get(),
            yaw_deg   = self.rpy_vars['Yaw'].get(),
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
