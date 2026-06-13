#!/usr/bin/env python3
"""
@file   ibvs_control_node.py
@brief  eMeetArm IBVS 视觉伺服 — 红色方块居中控制
@version 3.0
@date   2026-06-04

策略：订阅 red_box_detector 发布的特征点 → IBVS 驱动机械臂使质心保持在画面中心

  特征点：质心 (x_norm, y_norm, depth) — 1 点
  期望位置：(0, 0, depth_current)  — 图像中心，深度每帧跟随（仅修正平移偏移）
  控制模式：4xyzy — Vx Vy Vz ωy（伪逆优先用 Vx/Vy 居中）

  数据流：
    red_box_detector
        → /red_detector/feature (PointStamped: x_norm, y_norm, depth)
        → IBVS_Controller (4xyzy, curr, 1点)
        → TF 变换 (camera_optical_frame → base_link)
        → /arm_vel_cmd (TwistStamped)

    /camera/camera_sensor/image_raw
        → 仅用于调试可视化 → /ibvs/debug_image（误差箭头）

  目标丢失保护：超过 1s 未收到特征点则自动刹车

用法：
  ros2 run robot_arm_node ibvs_controller
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=ibvs_control

@copyright Copyright (c) 2026 eMeet
"""

import math
import os
import queue
import sys
import threading
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, TwistStamped
from sensor_msgs.msg import Image as ImageMsg
from tf2_ros import TransformListener, Buffer

import importlib.util
from ament_index_python.packages import get_package_share_directory

# 按绝对路径加载第三方算法库，避免 sys.path 操纵和同名模块歧义
_ibvs_path = os.path.join(
    get_package_share_directory('robot_arm_node'),
    'IBVS_Controller', 'ibvs_controller.py')
_spec = importlib.util.spec_from_file_location('ibvs_lib', _ibvs_path)
_mod  = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
IBVS_Controller = _mod.IBVS_Controller

# ── 调试图像绘制用相机内参（反算像素坐标）────────────────────────────────────
CAM_W    = 640
CAM_H    = 480
_fy      = (CAM_H / 2) / math.tan(math.radians(70.9 / 2))
CAM_K    = np.array([[_fy, 0,  CAM_W / 2],
                     [0,  _fy, CAM_H / 2],
                     [0,   0,  1        ]], dtype=np.float32)
CAM_DIST = np.zeros((4, 1), dtype=np.float32)

# ── 常量 ──────────────────────────────────────────────────────────────────────
CAMERA_FRAME = 'camera_optical_frame'
BASE_FRAME   = 'arm_base_link'
IMAGE_TOPIC   = '/camera/camera_sensor/image_raw'
FEATURE_TOPIC = '/red_detector/feature'   # 由 red_box_detector 发布
VEL_TOPIC     = '/arm_vel_cmd'
DEBUG_TOPIC   = '/ibvs/debug_image'

DEFAULT_LAMBDA = 1.0
ERROR_STOP_TH  = 0.005   # 质心误差小于此值停止（归一化坐标）
MAX_V_LIN      = 0.10    # m/s
MAX_V_ANG      = 0.30    # rad/s
MIN_V_CAM      = 0.005   # m/s，相机系最小线速度（防止末段速度衰减到零）
ADAPTIVE_TH    = 0.05    # 误差低于此值时启动自适应增益
ADAPTIVE_MAX   = 5.0     # 自适应 Lambda 最大倍数
DEPTH_GAIN     = 1.0     # 深度比例增益：Vz_cam = DEPTH_GAIN × (depth - desired_depth)


# ── 工具函数 ──────────────────────────────────────────────────────────────────
def _quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),     2*(x*z+y*w)    ],
        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)    ],
        [2*(x*z-y*w),   2*(y*z+x*w),     1-2*(x*x+y*y)  ],
    ], dtype=np.float64)


def _clamp_vel(vx, vy, vz, wx, wy, wz):
    lm = math.sqrt(vx*vx+vy*vy+vz*vz)
    if lm > MAX_V_LIN: s=MAX_V_LIN/lm; vx,vy,vz=vx*s,vy*s,vz*s
    am = math.sqrt(wx*wx+wy*wy+wz*wz)
    if am > MAX_V_ANG: s=MAX_V_ANG/am; wx,wy,wz=wx*s,wy*s,wz*s
    return vx, vy, vz, wx, wy, wz


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class IBVSColorNode(Node):
    """
    红色方块居中 IBVS 节点。

    质心作为唯一特征点，期望位置固定为图像中心 (0, 0)。
    深度每帧从 solvePnP 实时获取并作为期望深度，
    使控制器只关注 x/y 居中，不强制改变距离。
    """

    def __init__(self, gui_q: queue.Queue):
        super().__init__('ibvs_controller',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        self._q = gui_q

        # 1 个质心特征点，4xyzy 模式
        self._ibvs = IBVS_Controller(
            control_mode='4xyzy', interaction_mode='curr', num_pts=1)
        self._ibvs.set_lambda_matrix([DEFAULT_LAMBDA] * 4)

        self._lock           = threading.Lock()
        self._ibvs_active    = False
        self._lambda         = DEFAULT_LAMBDA
        self._last_feat      = None    # (x_norm, y_norm, depth) 最新质心特征
        self._last_feat_time = -float('inf')   # 用于检测目标丢失
        self._desired_depth  = 0.3     # 默认保持 0.3m；None = 跟随当前深度不控制距离

        self._vel_pub   = self.create_publisher(TwistStamped, VEL_TOPIC,   10)
        self._debug_pub = self.create_publisher(ImageMsg,     DEBUG_TOPIC, 10)

        # 特征点由 red_box_detector 检测后发布，这里只订阅结果
        self.create_subscription(
            PointStamped, FEATURE_TOPIC, self._on_feature, 10)
        # 图像仅用于调试可视化（不做检测）
        self.create_subscription(
            ImageMsg, IMAGE_TOPIC, self._on_image_debug, 10)
        # 目标丢失超时检查（每 200ms）
        self.create_timer(0.2, self._check_feat_timeout)
        # 看门狗心跳：IBVS 激活时每 200ms 发一次零速，防止速度控制器看门狗超时
        self.create_timer(0.2, self._keepalive)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'IBVSColorNode 就绪 | {FEATURE_TOPIC} → {VEL_TOPIC}')

    # ── 特征点回调（IBVS 控制，由 red_box_detector 发布）────────────────────
    def _on_feature(self, msg: PointStamped):
        x_norm = msg.point.x
        y_norm = msg.point.y
        depth  = msg.point.z

        # 过滤无效深度（solvePnP 偶发 NaN/Inf）
        if not math.isfinite(depth) or depth <= 0:
            return

        feat = (x_norm, y_norm, depth)
        now  = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._last_feat      = feat
            self._last_feat_time = now
            active         = self._ibvs_active
            lam            = self._lambda
            desired_depth  = self._desired_depth   # None or float

        self._q.put(('detect', True, depth, x_norm, y_norm))

        if not active:
            return

        # ── IBVS 居中控制 ─────────────────────────────────────────────────────
        # 期望深度：固定值 or 跟随当前（不控制距离）
        d_des   = desired_depth if desired_depth is not None else depth
        desired = [(0.0, 0.0, d_des)]
        current = [(x_norm, y_norm, depth)]

        err_pre = math.sqrt(x_norm*x_norm + y_norm*y_norm)
        eff_lam = lam * min(ADAPTIVE_TH / err_pre, ADAPTIVE_MAX) \
            if (err_pre < ADAPTIVE_TH and err_pre > 1e-6) else lam

        self._ibvs.set_lambda_matrix([eff_lam] * 4)
        self._ibvs.set_desired_points(desired)
        self._ibvs.set_current_points(current)
        self._ibvs.calculate_interaction_matrix()
        vels     = self._ibvs.calculate_velocities()
        err_norm = float(np.linalg.norm(self._ibvs.errs))
        self._q.put(('error', err_norm))

        # ── X/Y 居中速度（IBVS 计算）─────────────────────────────────────────
        if err_norm > ERROR_STOP_TH:
            vx_c = float(vels[0][0])
            vy_c = float(vels[1][0])
            vz_c = float(vels[2][0])
            wy_c = float(vels[3][0])
        else:
            vx_c = vy_c = vz_c = wy_c = 0.0

        # ── 深度控制叠加（独立比例控制器）────────────────────────────────────
        # IBVS 交互矩阵中深度只影响 L 矩阵计算，不产生 Vz 误差驱动。
        # 在 IBVS 输出 Vz 的基础上叠加深度比例项：
        #   Vz_depth = DEPTH_GAIN × (depth_current - depth_desired)
        #   depth > desired → Vz > 0 → 相机前进 → 深度减小 ✓
        if desired_depth is not None:
            depth_err = depth - desired_depth
            vz_c += depth_err * DEPTH_GAIN
            vz_c  = max(-MAX_V_LIN, min(MAX_V_LIN, vz_c))

        # ── 最小速度保证 + 发布 ───────────────────────────────────────────────
        v_lin = math.sqrt(vx_c*vx_c + vy_c*vy_c + vz_c*vz_c)
        if v_lin > 0:
            if v_lin < MIN_V_CAM:
                s = MIN_V_CAM / v_lin
                vx_c, vy_c, vz_c = vx_c*s, vy_c*s, vz_c*s
            v_b, w_b = self._transform_to_base(vx_c, vy_c, vz_c, wy_c)
            if v_b is not None:
                self._publish_vel(*_clamp_vel(*v_b, *w_b))
            else:
                self._publish_vel(0, 0, 0, 0, 0, 0)
        else:
            self._publish_vel(0, 0, 0, 0, 0, 0)

    # ── 看门狗心跳（每 200ms）────────────────────────────────────────────────
    def _keepalive(self):
        """IBVS 激活且最近没有特征到来时，持续发零速让速度控制器看门狗不超时。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            active = self._ibvs_active
            ftime  = self._last_feat_time
        # 有特征时 _on_feature 已经负责发命令；这里只在特征超时（>0.3s）时补发心跳
        if active and (now - ftime) > 0.3:
            self._publish_vel(0, 0, 0, 0, 0, 0)

    # ── 目标丢失超时检查（每 200ms）─────────────────────────────────────────
    def _check_feat_timeout(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            lost   = (now - self._last_feat_time) > 1.0
            active = self._ibvs_active
        if lost:
            with self._lock:
                self._last_feat = None
            self._q.put(('detect', False, None, None, None))
            if active:
                self._publish_vel(0, 0, 0, 0, 0, 0)

    # ── 图像回调（仅调试可视化，不做检测）────────────────────────────────────
    def _on_image_debug(self, msg: ImageMsg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            feat   = self._last_feat
            active = self._ibvs_active
            ftime  = self._last_feat_time
        feat_valid = feat is not None and (now - ftime) < 1.0

        img_cx, img_cy = CAM_W // 2, CAM_H // 2
        cv2.drawMarker(bgr, (img_cx, img_cy), (255, 100, 0),
                       cv2.MARKER_CROSS, 20, 2)

        if feat_valid:
            x_norm, y_norm, depth = feat
            # 归一化坐标反算像素位置
            cx_i = int(x_norm * float(CAM_K[0, 0]) + float(CAM_K[0, 2]))
            cy_i = int(y_norm * float(CAM_K[1, 1]) + float(CAM_K[1, 2]))
            cv2.drawMarker(bgr, (cx_i, cy_i), (0, 255, 0),
                           cv2.MARKER_CROSS, 16, 2)
            cv2.arrowedLine(bgr, (cx_i, cy_i), (img_cx, img_cy),
                            (0, 80, 255), 2, tipLength=0.2)
            if active:
                err   = math.sqrt(x_norm*x_norm + y_norm*y_norm)
                color = (0, 255, 0) if err <= ERROR_STOP_TH else (0, 200, 255)
                cv2.putText(bgr, f'err={err:.4f}  z={depth:.3f}m',
                            (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            else:
                cv2.putText(bgr,
                            f'dx={x_norm:.3f}  dy={y_norm:.3f}  z={depth:.3f}m',
                            (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (180, 180, 0), 1)
        elif active:
            cv2.putText(bgr, 'TARGET LOST', (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self._publish_debug(bgr, msg.header)

    # ── 坐标变换 ──────────────────────────────────────────────────────────────
    def _transform_to_base(self, vx_c, vy_c, vz_c, wy_c):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, CAMERA_FRAME, rclpy.time.Time())
            q = t.transform.rotation
            R = _quat_to_rot(q.x, q.y, q.z, q.w)
        except Exception:
            return None, None
        v_b = (R @ np.array([vx_c, vy_c, vz_c])).tolist()
        w_b = (R @ np.array([0.0,  wy_c, 0.0 ])).tolist()
        return v_b, w_b

    def _publish_vel(self, vx, vy, vz, wx, wy, wz):
        msg = TwistStamped()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.twist.linear.x  = float(vx)
        msg.twist.linear.y  = float(vy)
        msg.twist.linear.z  = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)
        self._vel_pub.publish(msg)

    def _publish_debug(self, bgr, header):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        m = ImageMsg()
        m.header       = header
        m.height       = rgb.shape[0]
        m.width        = rgb.shape[1]
        m.encoding     = 'rgb8'
        m.is_bigendian = False
        m.step         = rgb.shape[1] * 3
        m.data         = rgb.tobytes()
        self._debug_pub.publish(m)

    # ── 外部控制接口 ──────────────────────────────────────────────────────────
    def start_ibvs(self):
        with self._lock:
            if self._last_feat is None:
                self._q.put(('status', '⚠ 未检测到红色方块，无法启动'))
                return
            self._ibvs_active = True
            depth = self._last_feat[2]
        self._q.put(('status',
                     f'● IBVS 居中运行中（期望深度 {depth:.3f}m）'))

    def stop_ibvs(self):
        with self._lock:
            self._ibvs_active = False
        self._publish_vel(0, 0, 0, 0, 0, 0)
        self._q.put(('status', '■ IBVS 已停止'))

    def set_lambda(self, lam: float):
        with self._lock:
            self._lambda = lam

    def set_desired_depth(self, d):
        """设置期望深度。d=None 表示跟随当前深度（不控制距离）；d=float 表示固定目标距离(m)。"""
        with self._lock:
            self._desired_depth = d


# ── tkinter GUI ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, node: IBVSColorNode, gui_q: queue.Queue):
        self.root  = root
        self.node  = node
        self.gui_q = gui_q

        root.title('eMeet IBVS 红色方块居中控制器')
        root.resizable(True, False)

        pad  = dict(padx=6, pady=3)
        main = ttk.Frame(root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ── 检测状态 ──────────────────────────────────────────────────────────
        df = ttk.LabelFrame(main, text='红色方块检测', padding=6)
        df.pack(fill=tk.X, **pad)

        self._det_ind = tk.Label(df, text='●', fg='gray', font=('', 14))
        self._det_ind.grid(row=0, column=0, padx=6)

        for col, (lbl, attr) in enumerate(
                [('深度(m)', '_depth_v'), ('norm X', '_dx_v'), ('norm Y', '_dy_v')],
                start=1):
            ttk.Label(df, text=f'{lbl}:').grid(
                row=0, column=col*2-1, sticky='e', padx=3)
            v = tk.StringVar(value='--')
            setattr(self, attr, v)
            ttk.Entry(df, textvariable=v, width=8, state='readonly',
                      justify='center').grid(row=0, column=col*2, padx=2)

        # ── IBVS 误差 ─────────────────────────────────────────────────────────
        ef = ttk.LabelFrame(main, text='居中误差（目标：0.000）', padding=6)
        ef.pack(fill=tk.X, **pad)

        self._err_var = tk.StringVar(value='--')
        ttk.Entry(ef, textvariable=self._err_var, width=12,
                  state='readonly', justify='center',
                  font=('', 11)).pack(side=tk.LEFT, padx=6)

        # 简单误差进度条
        self._err_bar = ttk.Progressbar(ef, length=200, maximum=1.0)
        self._err_bar.pack(side=tk.LEFT, padx=4)

        # ── 参数 ──────────────────────────────────────────────────────────────
        pf = ttk.LabelFrame(main, text='控制参数', padding=6)
        pf.pack(fill=tk.X, **pad)

        ttk.Label(pf, text='Lambda:').grid(row=0, column=0, sticky='e', padx=4)
        self._lam_var = tk.DoubleVar(value=DEFAULT_LAMBDA)
        ttk.Spinbox(pf, from_=0.1, to=10.0, increment=0.1,
                    textvariable=self._lam_var, width=7, format='%.1f',
                    command=lambda: self.node.set_lambda(
                        self._lam_var.get())).grid(row=0, column=1, padx=2)

        ttk.Label(pf, text='收敛阈值:').grid(
            row=0, column=2, sticky='e', padx=8)
        ttk.Label(pf, text=f'{ERROR_STOP_TH:.3f}').grid(
            row=0, column=3, padx=2)
        ttk.Label(pf, text='调试图像:').grid(
            row=0, column=4, sticky='e', padx=8)
        ttk.Label(pf, text=DEBUG_TOPIC, foreground='blue').grid(
            row=0, column=5, padx=2)

        # ── 期望深度控制（第二行）─────────────────────────────────────────────
        self._depth_lock_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pf, text='固定深度', variable=self._depth_lock_var,
                        command=self._on_depth_lock).grid(
            row=1, column=0, columnspan=2, sticky='w', padx=4, pady=(4, 0))

        ttk.Label(pf, text='目标(m):').grid(
            row=1, column=2, sticky='e', padx=8, pady=(4, 0))
        self._depth_target_var = tk.DoubleVar(value=0.30)
        self._depth_spinbox = ttk.Spinbox(
            pf, from_=0.10, to=2.00, increment=0.01,
            textvariable=self._depth_target_var, width=7, format='%.2f',
            state='normal',
            command=self._on_depth_change)
        self._depth_spinbox.grid(row=1, column=3, padx=2, pady=(4, 0))

        ttk.Label(pf, text='（未勾选 = 跟随当前，不控制距离）',
                  foreground='gray').grid(
            row=1, column=4, columnspan=2, sticky='w', padx=4, pady=(4, 0))

        # ── 按钮 ──────────────────────────────────────────────────────────────
        bf = ttk.Frame(main)
        bf.pack(fill=tk.X, pady=8)

        tk.Button(bf, text='▶ 启动居中 IBVS',
                  command=self.node.start_ibvs,
                  bg='#4a4', fg='white', font=('', 11, 'bold'),
                  width=16).pack(side=tk.LEFT, padx=6)
        tk.Button(bf, text='■ 停止',
                  command=self.node.stop_ibvs,
                  bg='#e33', fg='white', font=('', 11, 'bold'),
                  width=8).pack(side=tk.LEFT, padx=6)

        # ── 状态栏 ────────────────────────────────────────────────────────────
        self._status_var = tk.StringVar(
            value=f'就绪 | norm X/Y = 归一化图像坐标（非米制），cam= 才是米制')
        ttk.Label(main, textvariable=self._status_var,
                  relief='sunken', anchor='w', padding=(4, 2)).pack(
            fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

        self._poll()

    def _on_depth_lock(self):
        locked = self._depth_lock_var.get()
        self._depth_spinbox.config(state='normal' if locked else 'disabled')
        if locked:
            self.node.set_desired_depth(self._depth_target_var.get())
        else:
            self.node.set_desired_depth(None)

    def _on_depth_change(self):
        if self._depth_lock_var.get():
            self.node.set_desired_depth(self._depth_target_var.get())

    def _poll(self):
        try:
            while True:
                item = self.gui_q.get_nowait()
                t = item[0]
                if t == 'status':
                    self._status_var.set(item[1])
                elif t == 'detect':
                    _, det, depth, dx, dy = item
                    if det:
                        self._det_ind.config(fg='#2a2')
                        self._depth_v.set(f'{depth:.3f}')
                        self._dx_v.set(f'{dx:.3f}')
                        self._dy_v.set(f'{dy:.3f}')
                    else:
                        self._det_ind.config(fg='gray')
                        self._depth_v.set('--')
                        self._dx_v.set('--')
                        self._dy_v.set('--')
                elif t == 'error':
                    val = item[1]
                    self._err_var.set(f'{val:.5f}')
                    self._err_bar['value'] = min(val, 1.0)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    gui_q    = queue.Queue()
    node     = IBVSColorNode(gui_q)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=executor.spin, daemon=True).start()

    root = tk.Tk()
    App(root, node, gui_q)
    root.mainloop()

    node.stop_ibvs()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
