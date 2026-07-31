#!/usr/bin/env python3
"""
@file   commander_test_gui.py
@brief  Arm Commander 调试 GUI —— Director 视角的完整测试工具
@version 2.1
@date   2026-07-31

功能（面板自上而下，与 Commander 的 4 个 action 对应）：
  1. ArmStatus 实时监控 —— 位姿/速度/状态/错误码 + 到位指示灯 + 急停/清错/回零/使能
  2. ArmMoveToPose      —— 姿态切换（STOWED/OBSERVE/SHOOTING）+ return_to_start
  3. ArmMoveToJoint     —— 关节空间点到点（2026-07-31 新增）：J1-3 滑块 + 当前值只读框
                           + 「↧ 读当前值」（把 /joint_states 实测灌进滑块，示教先手摆再微调）
                           + 档位 / Δ相对增量 / 时长（0=按档位）
                           云台 J4-6 由 Commander 保持不动，滑块范围 = URDF 限位
  4. ArmTrajectoryShot  —— 直线运镜（LINEAR）/ 球面环绕运镜（ORBIT）+ return_to_start
  （ArmTrackTarget 由 visp_ibvs_gui 单独覆盖，不在本 GUI）

订阅：/robot_arm/arm_status（语义状态）+ /joint_states（关节角 —— ArmStatus 只报末端
      位姿，关节角按总线约定从 /joint_states 读）

用法：
  ros2 launch robot_arm_gazebo gazebo.launch.py controller:=commander
  ros2 launch robot_arm_bringup real.launch.py  controller:=commander

@copyright Copyright (c) 2026 eMeet
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import math
import os
import sys

# arm_utils 与本脚本安装在同一目录（lib/robot_arm_node/），但 ROS2 不自动加入 sys.path
_this_dir = os.path.dirname(os.path.abspath(__file__))
if _this_dir not in sys.path:
    sys.path.insert(0, _this_dir)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from robot_arm_interfaces.action import ArmMoveToPose, ArmMoveToJoint, ArmTrajectoryShot
from robot_arm_interfaces.msg import ArmStatus
from robot_arm_interfaces.srv import ArmStop, ArmEnable, ArmHoming, ArmResetError
from sensor_msgs.msg import JointState

try:
    from arm_utils import aim_quat, sphere_to_cart, quat_to_rpy
    _HAS_ARM_UTILS = True
except ImportError:
    _HAS_ARM_UTILS = False


# ── 常量 ──────────────────────────────────────────────────────────────────────────
ACTION_MTP   = '/robot_arm/move_to_pose'
ACTION_MTJ   = '/robot_arm/move_to_joint'
ACTION_TSS   = '/robot_arm/trajectory_shot'
TOPIC_STATUS = '/robot_arm/arm_status'
TOPIC_JOINTS = '/joint_states'

# 关节滑块范围 = URDF 里 J1-3 的机械限位（2026-07-28 J2/J3 零点重标定后的值）。
# 这里只是 GUI 的输入范围，真正的拦截在 Commander 侧（从 /robot_description 解析
# URDF 校验，越界回 exit_reason="out_of_range" 且不下发）——两处不一致时以后者为准。
ARM_JOINT_RANGE = [
    ('Joint1', -2.618, 2.618),
    ('Joint2', -0.981, 2.959),
    ('Joint3', -2.500, 0.020),
]

STATE_LABELS = [
    (ArmMoveToPose.Goal.POSE_STATE_STOWED,   'STOWED  (0)  收纳位'),
    (ArmMoveToPose.Goal.POSE_STATE_OBSERVE,  'OBSERVE (1)  观察位'),
    (ArmMoveToPose.Goal.POSE_STATE_SHOOTING, 'SHOOTING(2)  拍摄位  ← 需填 XYZ/RPY'),
]
SPEED_LABELS = [
    (ArmMoveToPose.Goal.SPEED_SLOW,   'SLOW'),
    (ArmMoveToPose.Goal.SPEED_NORMAL, 'NORMAL'),
    (ArmMoveToPose.Goal.SPEED_FAST,   'FAST'),
]
POSE_STATE_NAMES = {0: 'STOWED', 1: 'OBSERVE', 2: 'SHOOTING'}
CMD_RESULT_NAMES = {0: 'NONE', 1: 'EXECUTING', 2: 'SUCCEEDED', 3: 'FAILED', 4: 'ABORTED'}
ERR_NAMES        = {0: 'NONE', 1: 'LIMIT', 2: 'DRIVER', 3: 'TIMEOUT'}


# ── ROS 节点 ─────────────────────────────────────────────────────────────────────
class CommanderTestNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('commander_test_gui')
        # use_sim_time 由 launch 按后端传入（gazebo/mujoco=true，real=false）。
        # 切勿在此硬编码覆盖：实物无 /clock 时 use_sim_time=true 会让所有
        # ROS 定时器永不触发（位姿面板卡 '--' 的教训）。
        self._q = gui_q

        self._mtp_client          = ActionClient(self, ArmMoveToPose, ACTION_MTP)
        self._mtj_client          = ActionClient(self, ArmMoveToJoint, ACTION_MTJ)
        self._tss_client          = ActionClient(self, ArmTrajectoryShot, ACTION_TSS)
        self._stop_client         = self.create_client(ArmStop,       '/robot_arm/stop')
        self._enable_client       = self.create_client(ArmEnable,     '/robot_arm/enable')
        self._homing_client       = self.create_client(ArmHoming,     '/robot_arm/homing')
        self._reset_error_client  = self.create_client(ArmResetError, '/robot_arm/reset_error')
        self._status_sub  = self.create_subscription(ArmStatus, TOPIC_STATUS, self._on_status, 10)
        # 关节反馈：ArmStatus 只报末端位姿，关节角按总线约定从 /joint_states 读
        self._joints_sub  = self.create_subscription(JointState, TOPIC_JOINTS, self._on_joints, 10)
        self._last_arm_joints = None   # [J1,J2,J3]，供 GUI「读当前值」用

    # ── ArmMoveToPose ─────────────────────────────────────────────────────────────
    def send_mtp_goal(self, state, speed, return_to_start,
                      tx, ty, tz, roll_deg, pitch_deg, yaw_deg):
        if not self._mtp_client.wait_for_server(timeout_sec=1.0):
            self._q.put(('error', f'✗ 未发现 server: {ACTION_MTP}')); return
        goal = ArmMoveToPose.Goal()
        goal.target_pose_state = state
        goal.transition_speed  = speed
        goal.return_to_start   = bool(return_to_start)
        goal.target_pose.x     = float(tx);  goal.target_pose.y = float(ty)
        goal.target_pose.z     = float(tz);  goal.target_pose.roll  = float(roll_deg)
        goal.target_pose.pitch = float(pitch_deg); goal.target_pose.yaw = float(yaw_deg)
        name = {0:'STOWED',1:'OBSERVE',2:'SHOOTING'}.get(state, str(state))
        spd  = {0:'SLOW',1:'NORMAL',2:'FAST'}.get(speed, str(speed))
        rts  = ' ↩return' if return_to_start else ''
        self._q.put(('log', f'→ MTP {name} speed={spd}{rts}'))
        self._mtp_client.send_goal_async(
            goal, feedback_callback=self._on_mtp_fb
        ).add_done_callback(self._on_mtp_response)

    def _on_mtp_response(self, f):
        gh = f.result()
        if not gh.accepted:
            self._q.put(('error', '✗ MTP 目标被拒绝')); return
        self._q.put(('log', '✓ MTP 已接受，执行中…'))
        gh.get_result_async().add_done_callback(self._on_mtp_result)

    def _on_mtp_fb(self, msg):
        fb = msg.feedback
        self._q.put(('fb_mtp', fb.progress_percent, fb.current_pose))

    def _on_mtp_result(self, f):
        r = f.result().result
        flag = '✓' if r.success else '✗'
        self._q.put(('log', f'{flag} MTP {r.exit_reason}  err={r.error_code}  '
                     f'实际z={r.actual_pose.z:.3f}m'))

    # ── ArmMoveToJoint ────────────────────────────────────────────────────────────
    def send_mtj_goal(self, joints, speed, relative, duration_sec):
        if not self._mtj_client.wait_for_server(timeout_sec=1.0):
            self._q.put(('error', f'✗ 未发现 server: {ACTION_MTJ}')); return
        goal = ArmMoveToJoint.Goal()
        goal.target_joints    = [float(j) for j in joints]   # 必须 3 个（J1-3）
        goal.transition_speed = speed
        goal.relative         = bool(relative)
        goal.duration_sec     = float(duration_sec)
        spd  = {0:'SLOW',1:'NORMAL',2:'FAST'}.get(speed, str(speed))
        mode = '相对' if relative else '绝对'
        dur  = f' {duration_sec:.2f}s' if duration_sec > 0 else f' speed={spd}'
        self._q.put(('log', f'→ MTJ {mode} [{joints[0]:.3f}, {joints[1]:.3f}, '
                            f'{joints[2]:.3f}]{dur}'))
        self._mtj_client.send_goal_async(
            goal, feedback_callback=self._on_mtj_fb
        ).add_done_callback(self._on_mtj_response)

    def _on_mtj_response(self, f):
        gh = f.result()
        if not gh.accepted:
            self._q.put(('error', '✗ MTJ 目标被拒绝')); return
        self._q.put(('log', '✓ MTJ 已接受，执行中…'))
        gh.get_result_async().add_done_callback(self._on_mtj_result)

    def _on_mtj_fb(self, msg):
        fb = msg.feedback
        self._q.put(('fb_mtj', fb.progress_percent, list(fb.current_joints)))

    def _on_mtj_result(self, f):
        r = f.result().result
        flag = '✓' if r.success else '✗'
        j = ', '.join(f'{v:.3f}' for v in r.actual_joints)
        self._q.put(('log', f'{flag} MTJ {r.exit_reason}  err={r.error_code}  实际[{j}]'))

    def _on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        try:
            self._last_arm_joints = [pos[n] for n, _, _ in ARM_JOINT_RANGE]
        except KeyError:
            return
        self._q.put(('joints', list(self._last_arm_joints)))

    def current_arm_joints(self):
        return self._last_arm_joints

    # ── ArmTrajectoryShot ─────────────────────────────────────────────────────────
    def send_tss_linear(self, speed, return_to_start,
                        sx, sy, sz, sr, sp, syw,
                        ex, ey, ez, er, epitch, eyw):
        if not self._tss_client.wait_for_server(timeout_sec=1.0):
            self._q.put(('error', f'✗ 未发现 server: {ACTION_TSS}')); return
        goal = ArmTrajectoryShot.Goal()
        goal.motion_type       = ArmTrajectoryShot.Goal.MOTION_LINEAR
        goal.transition_speed  = speed
        goal.return_to_start   = bool(return_to_start)
        goal.linear_start_pose.x = float(sx); goal.linear_start_pose.y = float(sy)
        goal.linear_start_pose.z = float(sz); goal.linear_start_pose.roll  = float(sr)
        goal.linear_start_pose.pitch = float(sp); goal.linear_start_pose.yaw = float(syw)
        goal.linear_end_pose.x   = float(ex); goal.linear_end_pose.y   = float(ey)
        goal.linear_end_pose.z   = float(ez); goal.linear_end_pose.roll   = float(er)
        goal.linear_end_pose.pitch = float(epitch); goal.linear_end_pose.yaw = float(eyw)
        spd = {0:'SLOW',1:'NORMAL',2:'FAST'}.get(speed, str(speed))
        rts = ' ↩return' if return_to_start else ''
        self._q.put(('log', f'→ TSS LINEAR speed={spd}{rts}  '
                     f'起({sx:.2f},{sy:.2f},{sz:.2f})→终({ex:.2f},{ey:.2f},{ez:.2f})'))
        self._tss_client.send_goal_async(
            goal, feedback_callback=self._on_tss_fb
        ).add_done_callback(self._on_tss_response)

    def send_tss_orbit(self, speed, return_to_start,
                       cx, cy, cz,
                       az_s, el_s, r_s, az_e, el_e, r_e):
        if not self._tss_client.wait_for_server(timeout_sec=1.0):
            self._q.put(('error', f'✗ 未发现 server: {ACTION_TSS}')); return
        goal = ArmTrajectoryShot.Goal()
        goal.motion_type         = ArmTrajectoryShot.Goal.MOTION_ORBIT
        goal.transition_speed    = speed
        goal.return_to_start     = bool(return_to_start)
        goal.orbit_center_x      = float(cx)
        goal.orbit_center_y      = float(cy)
        goal.orbit_center_z      = float(cz)
        goal.azimuth_start_deg   = float(az_s); goal.elevation_start_deg = float(el_s)
        goal.radius_start_m      = float(r_s)
        goal.azimuth_end_deg     = float(az_e); goal.elevation_end_deg   = float(el_e)
        goal.radius_end_m        = float(r_e)
        spd = {0:'SLOW',1:'NORMAL',2:'FAST'}.get(speed, str(speed))
        rts = ' ↩return' if return_to_start else ''
        self._q.put(('log', f'→ TSS ORBIT speed={spd}{rts}  '
                     f'球心=({cx:.2f},{cy:.2f},{cz:.2f})  '
                     f'az({az_s:.1f}°→{az_e:.1f}°) el({el_s:.1f}°→{el_e:.1f}°) '
                     f'r({r_s:.3f}→{r_e:.3f}m)'))
        self._tss_client.send_goal_async(
            goal, feedback_callback=self._on_tss_fb
        ).add_done_callback(self._on_tss_response)

    def _on_tss_response(self, f):
        gh = f.result()
        if not gh.accepted:
            self._q.put(('error', '✗ TSS 目标被拒绝')); return
        self._q.put(('log', '✓ TSS 已接受，执行中…'))
        gh.get_result_async().add_done_callback(self._on_tss_result)

    def _on_tss_fb(self, msg):
        fb = msg.feedback
        self._q.put(('fb_tss', fb.progress_percent, fb.elapsed_sec,
                     fb.current_pose, fb.current_azimuth_deg,
                     fb.current_elevation_deg, fb.current_radius_m))

    def _on_tss_result(self, f):
        r = f.result().result
        flag = '✓' if r.success else '✗'
        self._q.put(('log', f'{flag} TSS {r.exit_reason}  err={r.error_code}'))

    def call_arm_stop(self):
        if not self._stop_client.wait_for_service(timeout_sec=0.5):
            self._q.put(('error', '✗ /robot_arm/stop 服务不可用')); return
        self._stop_client.call_async(ArmStop.Request()).add_done_callback(
            lambda f: self._on_simple_result(f, 'Stop'))

    def call_arm_enable(self, enable: bool):
        if not self._enable_client.wait_for_service(timeout_sec=0.5):
            self._q.put(('error', '✗ /robot_arm/enable 服务不可用')); return
        req = ArmEnable.Request(); req.enable = enable
        self._enable_client.call_async(req).add_done_callback(
            lambda f: self._on_simple_result(f, 'Enable'))

    def call_arm_homing(self):
        if not self._homing_client.wait_for_service(timeout_sec=0.5):
            self._q.put(('error', '✗ /robot_arm/homing 服务不可用')); return
        self._homing_client.call_async(ArmHoming.Request()).add_done_callback(
            lambda f: self._on_simple_result(f, 'Homing'))

    def call_arm_reset_error(self):
        if not self._reset_error_client.wait_for_service(timeout_sec=0.5):
            self._q.put(('error', '✗ /robot_arm/reset_error 服务不可用')); return
        self._reset_error_client.call_async(ArmResetError.Request()).add_done_callback(
            lambda f: self._on_reset_error_result(f))

    def _on_simple_result(self, future, label: str):
        try:
            resp = future.result()
            flag = '✓' if resp.success else '✗'
            self._q.put(('log', f'{flag} {label}: {resp.message}'))
        except Exception as e:
            self._q.put(('error', f'✗ {label} 调用异常: {e}'))

    def _on_reset_error_result(self, future):
        try:
            resp = future.result()
            flag = '✓' if resp.success else '✗'
            self._q.put(('log', f'{flag} ResetError: {resp.message}  '
                         f'cleared_err={resp.cleared_error_code}'))
        except Exception as e:
            self._q.put(('error', f'✗ ResetError 调用异常: {e}'))

    def _on_status(self, msg: ArmStatus):
        self._q.put(('status', msg))


# ── GUI ───────────────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: CommanderTestNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('Arm Commander 调试 GUI')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        self._build_status_panel(main, pad)
        self._build_mtp_panel(main, pad)
        self._build_mtj_panel(main, pad)
        self._build_tss_panel(main, pad)
        self._build_log_panel(main, pad)
        self._poll()

    # ── ArmStatus ─────────────────────────────────────────────────────────────────
    def _build_status_panel(self, parent, pad):
        sf = ttk.LabelFrame(parent, text='ArmStatus  (/robot_arm/arm_status)', padding=6)
        sf.pack(fill=tk.X, **pad)

        row1 = ttk.Frame(sf); row1.pack(fill=tk.X)
        self._pose_vars = {}
        for k, u in [('X','m'),('Y','m'),('Z','m'),('Roll','°'),('Pitch','°'),('Yaw','°')]:
            ttk.Label(row1, text=f'{k}({u}):').pack(side=tk.LEFT, padx=(4, 1))
            v = tk.StringVar(value='--')
            self._pose_vars[k] = v
            ttk.Entry(row1, textvariable=v, width=8, state='readonly',
                      justify='center').pack(side=tk.LEFT, padx=(0, 6))

        row2 = ttk.Frame(sf); row2.pack(fill=tk.X, pady=(4, 0))
        for label, var_name in [('姿态','pose_state'),('错误','error_code'),
                                 ('命令结果','cmd_result'),('运动中','is_moving')]:
            ttk.Label(row2, text=f'{label}:').pack(side=tk.LEFT, padx=(4, 1))
            v = tk.StringVar(value='--')
            setattr(self, f'_sv_{var_name}', v)
            ttk.Entry(row2, textvariable=v, width=11, state='readonly',
                      justify='center').pack(side=tk.LEFT, padx=(0, 8))

        # 到位指示灯：到达目标 / 到达运镜起始点 / 摄像头录制就绪（绿=是，灰=否）
        row3 = ttk.Frame(sf); row3.pack(fill=tk.X, pady=(4, 0))
        self._led_at_target     = self._make_led(row3, '到达目标')
        self._led_at_start      = self._make_led(row3, '到达起点')
        self._led_camera_ready  = self._make_led(row3, '摄像头就绪')

        # 控制按钮行
        btn_row = ttk.Frame(sf); btn_row.pack(fill=tk.X, pady=(6, 0))
        tk.Button(btn_row, text='■ 急  停', width=10, bg='#cc3333', fg='white',
                  font=('', 9, 'bold'),
                  command=self._cmd_stop).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text='↺ 清除故障', width=10, bg='#3366cc', fg='white',
                  font=('', 9, 'bold'),
                  command=self._cmd_reset_error).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text='⌂ 回  零', width=10, bg='#336633', fg='white',
                  font=('', 9, 'bold'),
                  command=self._cmd_homing).pack(side=tk.LEFT, padx=4)
        self._enable_btn = tk.Button(btn_row, text='⚡ 使  能', width=10,
                                      bg='#666666', fg='white', font=('', 9, 'bold'),
                                      command=self._cmd_toggle_enable)
        self._enable_btn.pack(side=tk.LEFT, padx=4)
        self._servo_enabled = False

    # ── 到位指示灯辅助 ───────────────────────────────────────────────────────────
    @staticmethod
    def _make_led(parent, text):
        """创建一个布尔指示灯（绿=是 / 灰=否），返回可更新的 Label。"""
        ttk.Label(parent, text=f'{text}:').pack(side=tk.LEFT, padx=(4, 1))
        lbl = tk.Label(parent, text='否', width=6, relief='groove',
                       bg='#bbbbbb', fg='white', font=('', 9, 'bold'))
        lbl.pack(side=tk.LEFT, padx=(0, 12))
        return lbl

    @staticmethod
    def _set_led(lbl, on: bool):
        lbl.configure(text='是' if on else '否',
                      bg='#2e9e3f' if on else '#bbbbbb')

    # ── ArmMoveToPose ─────────────────────────────────────────────────────────────
    def _build_mtp_panel(self, parent, pad):
        mf = ttk.LabelFrame(parent, text='ArmMoveToPose  (/robot_arm/move_to_pose)', padding=6)
        mf.pack(fill=tk.X, **pad)

        self._mtp_state_var = tk.IntVar(value=ArmMoveToPose.Goal.POSE_STATE_OBSERVE)
        for val, label in STATE_LABELS:
            ttk.Radiobutton(mf, text=label, variable=self._mtp_state_var, value=val,
                            command=self._on_mtp_state_change).pack(anchor='w', padx=6)

        # SHOOTING 坐标
        self._mtp_shooting_frame = ttk.LabelFrame(
            mf, text='拍摄位 XYZ(m) + RPY(°)  —  仅 SHOOTING 有效', padding=4)
        self._mtp_shooting_frame.pack(fill=tk.X, pady=(4, 0), padx=4)
        self._mtp_cond_w = []
        row = ttk.Frame(self._mtp_shooting_frame); row.pack(fill=tk.X)
        self._mtp_xyz, self._mtp_rpy = {}, {}
        for col, (k, dflt, lo, hi, unit) in enumerate([
            ('X',0.30,-0.8,0.8,'m'),('Y',0.00,-0.8,0.8,'m'),('Z',0.50,-0.1,0.8,'m'),
            # 2026-07-29 云台换 V2：画面水平的 EEF roll 由 90° 变为 0°
            #（见 arm_utils.EEF_LEVEL_ROLL）；位置与 pitch 不变，只改 roll。
            ('Roll',0.,-180.,180.,'°'),('Pitch',10.,-90.,90.,'°'),('Yaw',0.,-180.,180.,'°'),
        ]):
            ttk.Label(row, text=f'{k}({unit}):').pack(side=tk.LEFT, padx=(4, 1))
            v = tk.DoubleVar(value=dflt)
            (self._mtp_xyz if col < 3 else self._mtp_rpy)[k] = v
            sp = ttk.Spinbox(row, from_=lo, to=hi,
                             increment=0.01 if col < 3 else 1.0,
                             textvariable=v, width=7,
                             format='%.2f' if col < 3 else '%.1f')
            sp.pack(side=tk.LEFT, padx=(0, 4))
            self._mtp_cond_w.append(sp)

        # 速度 + return_to_start + 发送
        bf = ttk.Frame(mf); bf.pack(fill=tk.X, pady=(6, 0))
        self._mtp_speed_var = tk.IntVar(value=ArmMoveToPose.Goal.SPEED_NORMAL)
        for val, label in SPEED_LABELS:
            ttk.Radiobutton(bf, text=label, variable=self._mtp_speed_var,
                            value=val).pack(side=tk.LEFT, padx=6)
        self._mtp_rts_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bf, text='↩ 回起点', variable=self._mtp_rts_var).pack(
            side=tk.LEFT, padx=10)
        ttk.Button(bf, text='▶ 发送', command=self._send_mtp,
                   width=10).pack(side=tk.RIGHT, padx=4)

        self._on_mtp_state_change()

    def _cmd_stop(self):
        self.node.call_arm_stop()
        self._log('■ 急停指令已发送')

    def _cmd_reset_error(self):
        self.node.call_arm_reset_error()
        self._log('↺ 清除故障指令已发送')

    def _cmd_homing(self):
        self.node.call_arm_homing()
        self._log('⌂ 回零指令已发送（阻塞，约 4s）')

    def _cmd_toggle_enable(self):
        self._servo_enabled = not self._servo_enabled
        self.node.call_arm_enable(self._servo_enabled)
        if self._servo_enabled:
            self._enable_btn.configure(text='⊘ 下  电', bg='#cc6600')
            self._log('⚡ 伺服使能')
        else:
            self._enable_btn.configure(text='⚡ 使  能', bg='#666666')
            self._log('⊘ 伺服下电')

    def _on_mtp_state_change(self):
        is_shooting = self._mtp_state_var.get() == ArmMoveToPose.Goal.POSE_STATE_SHOOTING
        s = 'normal' if is_shooting else 'disabled'
        for w in self._mtp_cond_w:
            w.configure(state=s)

    def _send_mtp(self):
        self.node.send_mtp_goal(
            state          = self._mtp_state_var.get(),
            speed          = self._mtp_speed_var.get(),
            return_to_start = self._mtp_rts_var.get(),
            tx=self._mtp_xyz['X'].get(), ty=self._mtp_xyz['Y'].get(),
            tz=self._mtp_xyz['Z'].get(),
            roll_deg=self._mtp_rpy['Roll'].get(),
            pitch_deg=self._mtp_rpy['Pitch'].get(),
            yaw_deg=self._mtp_rpy['Yaw'].get(),
        )

    # ── ArmMoveToJoint ────────────────────────────────────────────────────────────
    # 关节空间点到点：只动臂 J1-3（云台 J4-6 由 Commander 用当前回读原样保持）。
    # 「读当前值」把 /joint_states 的实测灌进滑块 —— 示教时先摆到位再微调最顺手。
    def _build_mtj_panel(self, parent, pad):
        jf = ttk.LabelFrame(parent, text='ArmMoveToJoint  (/robot_arm/move_to_joint)  '
                                         '—  关节空间，只动臂 J1-3，云台保持', padding=6)
        jf.pack(fill=tk.X, **pad)

        # 目标 / 当前 两行：滑块给目标，只读框显示实测，便于对比
        self._mtj_vars, self._mtj_cur_vars = {}, {}
        for name, lo, hi in ARM_JOINT_RANGE:
            row = ttk.Frame(jf); row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=f'{name}', width=7).pack(side=tk.LEFT, padx=(4, 2))
            v = tk.DoubleVar(value=0.0)
            self._mtj_vars[name] = v
            ttk.Scale(row, from_=lo, to=hi, variable=v, orient=tk.HORIZONTAL,
                      length=260).pack(side=tk.LEFT, padx=2)
            ttk.Spinbox(row, from_=lo, to=hi, increment=0.01, textvariable=v,
                        width=8, format='%.3f').pack(side=tk.LEFT, padx=4)
            ttk.Label(row, text=f'[{lo:.2f}, {hi:.2f}] rad').pack(side=tk.LEFT, padx=(2, 6))
            cv = tk.StringVar(value='--')
            self._mtj_cur_vars[name] = cv
            ttk.Label(row, text='当前:').pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=cv, width=8, state='readonly',
                      justify='center').pack(side=tk.LEFT, padx=(1, 4))

        bf = ttk.Frame(jf); bf.pack(fill=tk.X, pady=(6, 0))
        self._mtj_speed_var = tk.IntVar(value=ArmMoveToJoint.Goal.SPEED_NORMAL)
        for val, label in SPEED_LABELS:
            ttk.Radiobutton(bf, text=label, variable=self._mtj_speed_var,
                            value=val).pack(side=tk.LEFT, padx=6)
        self._mtj_rel_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bf, text='Δ 相对增量', variable=self._mtj_rel_var).pack(
            side=tk.LEFT, padx=10)
        ttk.Label(bf, text='时长(s，0=按档位):').pack(side=tk.LEFT, padx=(6, 1))
        self._mtj_dur_var = tk.DoubleVar(value=0.0)
        ttk.Spinbox(bf, from_=0.0, to=30.0, increment=0.5,
                    textvariable=self._mtj_dur_var, width=6,
                    format='%.1f').pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(bf, text='▶ 发送', command=self._send_mtj,
                   width=10).pack(side=tk.RIGHT, padx=4)
        ttk.Button(bf, text='↧ 读当前值', command=self._mtj_load_current,
                   width=11).pack(side=tk.RIGHT, padx=4)

    def _mtj_load_current(self):
        cur = self.node.current_arm_joints()
        if cur is None:
            self._log('✗ 还没收到 /joint_states，无法读当前值'); return
        for (name, _, _), val in zip(ARM_JOINT_RANGE, cur):
            self._mtj_vars[name].set(round(val, 4))
        self._log('↧ 已把当前关节角灌入滑块')

    def _send_mtj(self):
        self.node.send_mtj_goal(
            joints       = [self._mtj_vars[n].get() for n, _, _ in ARM_JOINT_RANGE],
            speed        = self._mtj_speed_var.get(),
            relative     = self._mtj_rel_var.get(),
            duration_sec = self._mtj_dur_var.get(),
        )

    # ── ArmTrajectoryShot ─────────────────────────────────────────────────────────
    def _build_tss_panel(self, parent, pad):
        tf = ttk.LabelFrame(parent, text='ArmTrajectoryShot  (/robot_arm/trajectory_shot)', padding=6)
        tf.pack(fill=tk.X, **pad)

        nb = ttk.Notebook(tf)
        nb.pack(fill=tk.X, padx=2, pady=2)

        self._build_tss_linear_tab(nb)
        self._build_tss_orbit_tab(nb)

        # 共用底部控件
        bf = ttk.Frame(tf); bf.pack(fill=tk.X, pady=(4, 0))
        self._tss_speed_var = tk.IntVar(value=ArmTrajectoryShot.Goal.SPEED_NORMAL)
        for val, label in SPEED_LABELS:
            ttk.Radiobutton(bf, text=label, variable=self._tss_speed_var,
                            value=val).pack(side=tk.LEFT, padx=6)
        self._tss_rts_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bf, text='↩ 回起点', variable=self._tss_rts_var).pack(
            side=tk.LEFT, padx=10)
        ttk.Button(bf, text='▶ 发送', command=lambda: self._send_tss(nb),
                   width=10).pack(side=tk.RIGHT, padx=4)

    def _pose_row(self, parent, defaults):
        """构建一行位姿输入：X Y Z Roll Pitch Yaw，返回变量字典。"""
        row = ttk.Frame(parent); row.pack(fill=tk.X, pady=2)
        vars_ = {}
        for k, dflt, lo, hi, unit in [
            ('X',defaults[0],-0.8,0.8,'m'), ('Y',defaults[1],-0.8,0.8,'m'),
            ('Z',defaults[2],-0.1,0.8,'m'),
            ('Roll',defaults[3],-180.,180.,'°'), ('Pitch',defaults[4],-90.,90.,'°'),
            ('Yaw',defaults[5],-180.,180.,'°'),
        ]:
            ttk.Label(row, text=f'{k}({unit}):').pack(side=tk.LEFT, padx=(4, 1))
            v = tk.DoubleVar(value=dflt)
            vars_[k] = v
            ttk.Spinbox(row, from_=lo, to=hi,
                        increment=0.01 if unit == 'm' else 1.0,
                        textvariable=v, width=7,
                        format='%.2f' if unit == 'm' else '%.1f').pack(
                side=tk.LEFT, padx=(0, 4))
        return vars_

    def _build_tss_linear_tab(self, nb):
        tab = ttk.Frame(nb, padding=6); nb.add(tab, text='直线运镜 LINEAR')

        # 起始位姿
        r1 = ttk.Frame(tab); r1.pack(fill=tk.X)
        ttk.Label(r1, text='起始位姿', foreground='#2266aa', width=8).pack(side=tk.LEFT)
        ttk.Button(r1, text='↩ 移到起始位', width=12,
                   command=self._goto_linear_start).pack(side=tk.RIGHT, padx=4)
        # roll 90.→0.：云台 V2 下 0° 才是画面水平（见 arm_utils.EEF_LEVEL_ROLL）
        self._tss_lin_start = self._pose_row(tab, [0.30, 0.0, 0.60, 0., 0., 0.])

        # 终止位姿
        r2 = ttk.Frame(tab); r2.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(r2, text='终止位姿', foreground='#aa4422', width=8).pack(side=tk.LEFT)
        ttk.Button(r2, text='→ 移到终止位', width=12,
                   command=self._goto_linear_end).pack(side=tk.RIGHT, padx=4)
        self._tss_lin_end = self._pose_row(tab, [0.30, 0.0, 0.40, 0., 0., 0.])

    def _build_tss_orbit_tab(self, nb):
        tab = ttk.Frame(nb, padding=6); nb.add(tab, text='球面环绕 ORBIT')

        # ── 被摄主体位置（球心）──────────────────────────────────────────────
        cf = ttk.LabelFrame(tab, text='被摄主体位置（球心，base_link 坐标系）', padding=4)
        cf.pack(fill=tk.X, pady=(0, 6))
        center_row = ttk.Frame(cf); center_row.pack(fill=tk.X)
        self._tss_orb_center = {}
        for k, dflt in [('ox', 0.60), ('oy', -0.12), ('oz', 0.47)]:
            ttk.Label(center_row, text=f'{k}(m):').pack(side=tk.LEFT, padx=(4, 1))
            v = tk.DoubleVar(value=dflt)
            self._tss_orb_center[k] = v
            ttk.Spinbox(center_row, from_=-2.0, to=2.0, increment=0.01,
                        textvariable=v, width=8, format='%.3f').pack(
                side=tk.LEFT, padx=(0, 10))
        ttk.Button(center_row, text='设为当前末端',
                   command=self._set_orbit_center_from_ee).pack(side=tk.LEFT, padx=4)

        # 移到起始球坐标按钮
        top_row = ttk.Frame(tab); top_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(top_row, text='↩ 移到起始球坐标', width=16,
                   command=self._goto_orbit_start).pack(side=tk.RIGHT, padx=4)

        def sphere_row(parent, label, az_dflt, el_dflt, r_dflt):
            ttk.Label(parent, text=label).pack(anchor='w')
            row = ttk.Frame(parent); row.pack(fill=tk.X, pady=2)
            vars_ = {}
            for k, dflt, lo, hi, unit in [
                ('方位角Az', az_dflt, -360., 360., '°'),
                ('俯仰角El', el_dflt,  -89.,  89., '°'),
                ('半径r',    r_dflt,    0.05,  2.0, 'm'),
            ]:
                ttk.Label(row, text=f'{k}({unit}):').pack(side=tk.LEFT, padx=(4, 1))
                v = tk.DoubleVar(value=dflt)
                vars_[k] = v
                ttk.Spinbox(row, from_=lo, to=hi,
                            increment=1.0 if unit == '°' else 0.05,
                            textvariable=v, width=8,
                            format='%.1f' if unit == '°' else '%.3f').pack(
                    side=tk.LEFT, padx=(0, 8))
            return vars_

        self._tss_orb_start = sphere_row(tab, '起始球坐标', -30.0, 10., 0.30)
        self._tss_orb_end   = sphere_row(tab, '终止球坐标',  30.0, 10., 0.30)

    def _send_tss(self, nb):
        tab_idx = nb.index(nb.select())
        speed = self._tss_speed_var.get()
        rts   = self._tss_rts_var.get()

        if tab_idx == 0:   # LINEAR
            s = self._tss_lin_start;  e = self._tss_lin_end
            self.node.send_tss_linear(speed, rts,
                s['X'].get(), s['Y'].get(), s['Z'].get(),
                s['Roll'].get(), s['Pitch'].get(), s['Yaw'].get(),
                e['X'].get(), e['Y'].get(), e['Z'].get(),
                e['Roll'].get(), e['Pitch'].get(), e['Yaw'].get())
        else:               # ORBIT
            s  = self._tss_orb_start;  e = self._tss_orb_end
            cx = self._tss_orb_center['ox'].get()
            cy = self._tss_orb_center['oy'].get()
            cz = self._tss_orb_center['oz'].get()
            self.node.send_tss_orbit(speed, rts, cx, cy, cz,
                s['方位角Az'].get(), s['俯仰角El'].get(), s['半径r'].get(),
                e['方位角Az'].get(), e['俯仰角El'].get(), e['半径r'].get())

    # ── 移到起始 / 终止位姿（手动定位，不执行轨迹）─────────────────────────────────
    def _goto_linear_start(self):
        """移到直线运镜起始位姿（ArmMoveToPose SHOOTING，NORMAL 速度）。"""
        s = self._tss_lin_start
        self.node.send_mtp_goal(
            ArmMoveToPose.Goal.POSE_STATE_SHOOTING,
            ArmMoveToPose.Goal.SPEED_NORMAL, False,
            s['X'].get(), s['Y'].get(), s['Z'].get(),
            s['Roll'].get(), s['Pitch'].get(), s['Yaw'].get())
        self._log('↩ 移到直线运镜起始位')

    def _goto_linear_end(self):
        """移到直线运镜终止位姿（ArmMoveToPose SHOOTING，NORMAL 速度）。"""
        e = self._tss_lin_end
        self.node.send_mtp_goal(
            ArmMoveToPose.Goal.POSE_STATE_SHOOTING,
            ArmMoveToPose.Goal.SPEED_NORMAL, False,
            e['X'].get(), e['Y'].get(), e['Z'].get(),
            e['Roll'].get(), e['Pitch'].get(), e['Yaw'].get())
        self._log('→ 移到直线运镜终止位')

    def _set_orbit_center_from_ee(self):
        """将当前末端位姿设为球心（从 ArmStatus 读取）。"""
        for k, pose_k in [('ox','X'), ('oy','Y'), ('oz','Z')]:
            v = self._pose_vars.get(pose_k)
            if v and v.get() != '--':
                try:
                    self._tss_orb_center[k].set(round(float(v.get()), 3))
                except ValueError:
                    pass
        cx = self._tss_orb_center['ox'].get()
        cy = self._tss_orb_center['oy'].get()
        cz = self._tss_orb_center['oz'].get()
        self._log(f'球心已设为当前末端 ({cx:.3f},{cy:.3f},{cz:.3f})')

    def _goto_orbit_start(self):
        """移到环绕运镜起始球坐标对应的 Cartesian 位姿（ArmMoveToPose SHOOTING）。"""
        if not _HAS_ARM_UTILS:
            self._log('✗ arm_utils 未加载，无法计算球坐标'); return
        s  = self._tss_orb_start
        ox = self._tss_orb_center['ox'].get()
        oy = self._tss_orb_center['oy'].get()
        oz = self._tss_orb_center['oz'].get()
        az_rad = math.radians(s['方位角Az'].get())
        el_rad = math.radians(s['俯仰角El'].get())
        r      = s['半径r'].get()
        px, py, pz = sphere_to_cart(az_rad, el_rad, r, ox, oy, oz)
        qx, qy, qz, qw = aim_quat(px, py, pz, ox, oy, oz)
        roll_r, pitch_r, yaw_r = quat_to_rpy(qx, qy, qz, qw)
        self.node.send_mtp_goal(
            ArmMoveToPose.Goal.POSE_STATE_SHOOTING,
            ArmMoveToPose.Goal.SPEED_NORMAL, False,
            px, py, pz,
            math.degrees(roll_r), math.degrees(pitch_r), math.degrees(yaw_r))
        self._log(f'↩ 移到环绕起始  球心=({ox:.3f},{oy:.3f},{oz:.3f})  '
                  f'az={s["方位角Az"].get():.1f}° el={s["俯仰角El"].get():.1f}° '
                  f'r={r:.3f}m → ({px:.3f},{py:.3f},{pz:.3f})')

    # ── 日志 ──────────────────────────────────────────────────────────────────────
    def _build_log_panel(self, parent, pad):
        lf = ttk.LabelFrame(parent, text='日志', padding=4)
        lf.pack(fill=tk.BOTH, expand=True, padx=pad['padx'], pady=(6, 0))
        self._log_text = tk.Text(lf, height=8, state='disabled',
                                  font=('Courier', 9), wrap=tk.WORD)
        sc = ttk.Scrollbar(lf, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=sc.set)
        self._log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sc.pack(side=tk.RIGHT, fill=tk.Y)

    def _log(self, msg: str):
        self._log_text.configure(state='normal')
        self._log_text.insert(tk.END, msg + '\n')
        self._log_text.see(tk.END)
        self._log_text.configure(state='disabled')

    # ── 消息轮询 ──────────────────────────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                self._handle(self.gui_q.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def _handle(self, item):
        t = item[0]
        if t == 'status':
            s = item[1]; p = s.arm_pose
            for k, val in zip(('X','Y','Z','Roll','Pitch','Yaw'),
                               (p.x,p.y,p.z,p.roll,p.pitch,p.yaw)):
                self._pose_vars[k].set(f'{val:.3f}' if k in ('X','Y','Z') else f'{val:.1f}')
            self._sv_pose_state.set(POSE_STATE_NAMES.get(s.current_pose_state,'?'))
            self._sv_error_code.set(ERR_NAMES.get(s.error_code, str(s.error_code)))
            self._sv_cmd_result.set(CMD_RESULT_NAMES.get(s.command_result,'?'))
            self._sv_is_moving.set('是' if s.is_moving else '否')
            self._set_led(self._led_at_target,    s.arm_at_target)
            self._set_led(self._led_at_start,     s.arm_at_pose_start)
            self._set_led(self._led_camera_ready, s.camera_ready)
        elif t == 'fb_mtp':
            _, prog, pose = item
            self._log(f'… MTP {prog:5.1f}%  z={pose.z:.3f}m')
        elif t == 'joints':
            for (name, _, _), val in zip(ARM_JOINT_RANGE, item[1]):
                self._mtj_cur_vars[name].set(f'{val:.3f}')
        elif t == 'fb_mtj':
            _, prog, joints = item
            j = ', '.join(f'{v:.3f}' for v in joints)
            self._log(f'… MTJ {prog:5.1f}%  [{j}]')
        elif t == 'fb_tss':
            _, prog, elapsed, pose, az, el, r = item
            self._log(f'… TSS {prog:5.1f}%  {elapsed:.1f}s  '
                      f'z={pose.z:.3f}m  az={az:.1f}° el={el:.1f}° r={r:.3f}m')
        elif t in ('log', 'error'):
            self._log(item[1])


# ── 入口 ──────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q = queue.Queue()
    node  = CommanderTestNode(gui_q)
    exe   = MultiThreadedExecutor()
    exe.add_node(node)
    threading.Thread(target=exe.spin, daemon=True).start()

    root = tk.Tk()
    App(root, node, gui_q)
    root.mainloop()

    exe.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
