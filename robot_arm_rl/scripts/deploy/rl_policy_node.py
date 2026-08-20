#!/usr/bin/env python3
"""
@file   rl_policy_node.py
@brief  robot_arm RL 构图跟拍策略部署节点（sim2sim 已打通；实机仍有 TODO）

工作原理（每个策略周期，默认 50Hz 与训练口径一致）：
  /joint_states(执行态 q,qv) ─┐
  主体位置(参数或话题)        ─┼→ MuJoCo 运动学孪生(rl_env.xml)做 FK/投影 → 34 维观测
  内部球坐标+视轴偏置状态     ─┘
  → VecNormalize → SAC 推理 → 增量更新球坐标/偏置(含可达域守卫，与训练一致)
  → 孪生 IK(6关节) → /arm_controller/joint_trajectory 单点位置流

sim2sim（Gazebo）已闭环：感知 = 用执行态关节角在孪生里做几何投影，
反馈经真实执行链（JTC → gazebo_ros2_control）回来，是跨仿真器验证。

⚠️ 实机上线前仍需（按 README/迁移路线）：
  1. 真实感知：u,v,s 必须来自检测框（如 /red_detector/feature），
     kinematic 感知在实机上等于相信指令即执行，遮挡/标定误差全被忽略；
  2. 实测 JTC 50Hz 位置流在实机跟踪率只有 ~71-74%（2026-08-12），
     需降频+缩放增量或建模执行滞后重训；
  3. 安全壳：工作空间/自碰撞检查、丢检测框冻结、急停接 ModeManager。

参数：
  model_path / vec_norm_path   模型与归一化统计量（.zip 不含后缀 / .pkl）
  policy_hz        策略频率（默认 50，与训练一致；wall clock 驱动，
                    不用 sim time——Gazebo /clock 10Hz 会饿死定时器）
  traj_duration_s  每条轨迹段时长（默认 0.1s）
  subject_x/y/z    主体在 base_link 系的位置（可被话题覆盖）
  goal_u/v/s       构图目标（默认三分法）
  auto_start       true=收到 /joint_states 即开始控制；false=等 /rl_policy/enable

话题：
  订阅  /joint_states, /rl_policy/subject_pos(PointStamped, base_link 系)
  发布  /arm_controller/joint_trajectory, /rl_policy/debug(PoseStamped),
        /rl_policy/framing(PointStamped: x=u误差, y=v误差, z=构图误差模长)

@copyright Copyright (c) 2026 eMeet
"""

import math
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from stable_baselines3 import SAC
    from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
except ImportError as e:
    raise SystemExit('✗ 请先安装 stable-baselines3：pip install stable-baselines3') from e

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))   # scripts/

from envs.emeet_arm_env import (
    eMeetArmEnv,
    JOINT_NAMES, MAX_VEL,
    DTHETA_MAX, DPHI_MAX, DR_MAX, DPAN_MAX, DTILT_MAX, OFF_MAX,
    THETA_MIN, THETA_MAX, PHI_MIN, PHI_MAX, R_MIN, R_MAX,
    WS_H_MIN, WS_H_MAX, WS_Z_MIN, WS_Z_MAX,
    R_MID, R_HALF, S_MID, S_HALF, DEFAULT_GOAL,
    sphere_to_cart, aim_mat, ws_violation, _theta_ref,
)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('rl_policy_node')

        self.declare_parameter('model_path',      '')
        self.declare_parameter('vec_norm_path',   '')
        self.declare_parameter('policy_hz',       50.0)
        # 段时长必须≈策略周期：过长会被 50Hz 抢占稀释增量（每段只执行前 20ms）
        self.declare_parameter('traj_duration_s', 0.02)
        self.declare_parameter('subject_x',       0.90)
        self.declare_parameter('subject_y',       0.00)
        self.declare_parameter('subject_z',       0.47)
        self.declare_parameter('goal_u',          DEFAULT_GOAL[0])
        self.declare_parameter('goal_v',          DEFAULT_GOAL[1])
        self.declare_parameter('goal_s',          DEFAULT_GOAL[2])
        self.declare_parameter('auto_start',      True)
        self.declare_parameter('metrics_window',  250)

        model_path    = self.get_parameter('model_path').value
        vec_norm_path = self.get_parameter('vec_norm_path').value
        policy_hz     = float(self.get_parameter('policy_hz').value)
        self._traj_dt = float(self.get_parameter('traj_duration_s').value)
        self._goal = (float(self.get_parameter('goal_u').value),
                      float(self.get_parameter('goal_v').value),
                      float(self.get_parameter('goal_s').value))
        self._enabled = bool(self.get_parameter('auto_start').value)
        self._metrics_window = int(self.get_parameter('metrics_window').value)

        if not model_path:
            self.get_logger().error('参数 model_path 未设置，节点退出')
            raise SystemExit(1)

        self.get_logger().info(f'加载 SAC 模型: {model_path}')
        self._model = SAC.load(model_path, device='cpu')

        # 运动学孪生：FK / IK / 投影全部复用训练环境的实现，保证口径逐位一致
        self._kin = eMeetArmEnv(subject_motion=False, goal=DEFAULT_GOAL)

        self._vec_norm = None
        if vec_norm_path and os.path.exists(vec_norm_path):
            dummy = DummyVecEnv([lambda: eMeetArmEnv(subject_motion=False)])
            self._vec_norm = VecNormalize.load(vec_norm_path, dummy)
            self._vec_norm.training = False
            self._vec_norm.norm_reward = False
            self.get_logger().info(f'  VecNormalize 已加载: {vec_norm_path}')
        else:
            self.get_logger().warn('  未找到 VecNormalize，观测分布将与训练不一致')

        self._lock = threading.Lock()
        self._q    = np.zeros(6)
        self._qv   = np.zeros(6)
        self._js_ready = False
        self._subj = np.array([
            float(self.get_parameter('subject_x').value),
            float(self.get_parameter('subject_y').value),
            float(self.get_parameter('subject_z').value)])

        # 策略内部状态
        self._seeded = False
        self._theta, self._phi, self._r = 0.0, 0.2, 0.50
        self._pan_off, self._tilt_off = 0.0, 0.0
        self._prev_action = np.zeros(5, dtype=np.float32)
        self._prev_uv = (0.5, 0.5)

        # 验收统计
        self._stat_frames, self._stat_err, self._stat_vis = 0, 0.0, 0

        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self._subj_sub = self.create_subscription(
            PointStamped, '/rl_policy/subject_pos', self._subj_cb, 10)
        self._enable_sub = self.create_subscription(
            Bool, '/rl_policy/enable', self._enable_cb, 1)

        self._traj_pub  = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._debug_pub = self.create_publisher(
            PoseStamped, '/rl_policy/debug', 10)
        self._framing_pub = self.create_publisher(
            PointStamped, '/rl_policy/framing', 10)

        # wall timer：见文件头，不能用 sim time 驱动
        self._policy_timer = self.create_timer(
            1.0 / max(policy_hz, 1.0), self._policy_step,
            clock=rclpy.clock.Clock())
        self.get_logger().info(
            f'RL 策略节点就绪  hz={policy_hz:.0f}  traj_dt={self._traj_dt:.2f}s  '
            f'goal={self._goal}  subject={tuple(self._subj)}  '
            f'{"自动开始" if self._enabled else "等待 /rl_policy/enable"}')

    # ── 回调 ────────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        n2i = {n: i for i, n in enumerate(msg.name)}
        with self._lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in n2i:
                    j = n2i[name]
                    self._q[i] = msg.position[j]
                    if j < len(msg.velocity):
                        self._qv[i] = msg.velocity[j]
            self._js_ready = True

    def _subj_cb(self, msg: PointStamped):
        with self._lock:
            self._subj = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _enable_cb(self, msg: Bool):
        self._enabled = bool(msg.data)
        self.get_logger().info(f'RL 控制 {"启动" if self._enabled else "停止"}')

    # ── 主循环 ──────────────────────────────────────────────────────────────

    def _policy_step(self):
        with self._lock:
            if not self._js_ready or not self._enabled:
                return
            q, qv, subj = self._q.copy(), self._qv.copy(), self._subj.copy()

        # 孪生同步到执行态
        self._kin._set_subject_pos(*subj)
        self._kin._set_joints(q)

        if not self._seeded:
            self._seed_spherical(subj)
            self._seeded = True

        # 观测（口径与 eMeetArmEnv._get_obs 逐维一致）
        u, v, s = self._kin._project_subject()
        obs = self._build_obs(q, qv, u, v, s)
        if self._vec_norm is not None:
            obs = self._vec_norm.normalize_obs(obs.reshape(1, -1))[0]

        action, _ = self._model.predict(obs, deterministic=True)
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # 球坐标增量 + 可达域守卫（与 env.step 一致）
        ox, oy, oz = subj
        theta_new = float(np.clip(self._theta + action[0]*DTHETA_MAX, THETA_MIN, THETA_MAX))
        phi_new   = float(np.clip(self._phi   + action[1]*DPHI_MAX,   PHI_MIN,   PHI_MAX))
        r_new     = float(np.clip(self._r     + action[2]*DR_MAX,     R_MIN,     R_MAX))
        px, py, pz = sphere_to_cart(theta_new, phi_new, r_new, ox, oy, oz)
        cur = sphere_to_cart(self._theta, self._phi, self._r, ox, oy, oz)
        if ws_violation(px, py, pz) <= ws_violation(*cur):   # 单调回归守卫，与 env 一致
            self._theta, self._phi, self._r = theta_new, phi_new, r_new
        else:
            px, py, pz = cur

        self._pan_off  = float(np.clip(self._pan_off  + action[3]*DPAN_MAX,  -OFF_MAX, OFF_MAX))
        self._tilt_off = float(np.clip(self._tilt_off + action[4]*DTILT_MAX, -OFF_MAX, OFF_MAX))

        # IK（孪生 qpos 已在执行态 → 天然热启动）
        q_target = self._kin._ik_solve(
            np.array([px, py, pz]),
            aim_mat((px, py, pz), (ox, oy, oz), self._pan_off, self._tilt_off))

        self._send_trajectory(q_target, q)
        self._publish_debug(px, py, pz)
        self._prev_action = action
        self._collect_metrics(u, v)
        self._prev_uv = (u, v)

    # ── 起步播种：从执行态相机位姿反解球坐标，避免第一拍跳变 ────────────────

    def _seed_spherical(self, subj):
        cam = self._kin.data.xpos[self._kin._cam_body_id].copy()
        d = cam - subj
        r = float(np.linalg.norm(d))
        self._r     = float(np.clip(r, R_MIN, R_MAX))
        self._phi   = float(np.clip(math.asin(np.clip(d[2] / max(r, 1e-6), -1, 1)),
                                    PHI_MIN, PHI_MAX))
        theta_world = math.atan2(d[1], d[0])
        self._theta = float(np.clip(_wrap(theta_world - _theta_ref(subj[0], subj[1])),
                                    THETA_MIN, THETA_MAX))
        self._pan_off, self._tilt_off = 0.0, 0.0

        # 播种点可能在可达域外（如臂处于收纳位姿），沿 r 投影回域内，
        # 保证策略从有效状态起步（域外死锁的教训，2026-08-19 sim2sim 实测）
        if ws_violation(*sphere_to_cart(self._theta, self._phi, self._r,
                                        *subj)) > 0:
            grid = np.linspace(R_MIN, R_MAX, 61)
            viols = [ws_violation(*sphere_to_cart(self._theta, self._phi, float(r),
                                                  *subj)) for r in grid]
            order = sorted(range(len(grid)),
                           key=lambda i: (viols[i], abs(grid[i] - self._r)))
            self._r = float(grid[order[0]])
        self.get_logger().info(
            f'球坐标播种: θ={self._theta:+.2f} φ={self._phi:+.2f} r={self._r:.2f}  '
            f'(域外违规量 {ws_violation(*sphere_to_cart(self._theta, self._phi, self._r, *subj)):.3f})')

    # ── 观测构建 ─────────────────────────────────────────────────────────────

    def _build_obs(self, q, qv, u, v, s) -> np.ndarray:
        kin = self._kin
        qpos_norm = np.clip(2*(q - kin._jnt_lo)/(kin._jnt_hi - kin._jnt_lo) - 1, -1, 1)
        qvel_norm = np.clip(qv / MAX_VEL, -1., 1.)
        du = float(np.clip((u - self._prev_uv[0]) * 25.0, -1., 1.))
        dv = float(np.clip((v - self._prev_uv[1]) * 25.0, -1., 1.))
        u_tgt, v_tgt, s_tgt = self._goal
        return np.array([
            math.sin(self._theta), math.cos(self._theta),
            self._phi / (math.pi / 2),
            (self._r - R_MID) / R_HALF,
            self._pan_off / OFF_MAX,
            self._tilt_off / OFF_MAX,
            *self._prev_action,
            u - u_tgt, v - v_tgt, (s - s_tgt) / S_HALF,
            u*2 - 1, v*2 - 1, (s - S_MID) / S_HALF,
            u_tgt*2 - 1, v_tgt*2 - 1, (s_tgt - S_MID) / S_HALF,
            du, dv,
            1.0,               # 检测有效标志（v3 口径；接真实检测器后按丢帧置 0）
            *qpos_norm,
            *qvel_norm,
        ], dtype=np.float32)

    # ── 输出 ────────────────────────────────────────────────────────────────

    def _send_trajectory(self, q_target, q_now):
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions  = [float(x) for x in q_target]
        pt.velocities = [float((t - c) / self._traj_dt)
                         for t, c in zip(q_target, q_now)]
        sec = int(self._traj_dt)
        pt.time_from_start = DurationMsg(
            sec=sec, nanosec=int((self._traj_dt - sec) * 1e9))
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _publish_debug(self, px, py, pz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
        R = aim_mat((px, py, pz), tuple(self._subj), self._pan_off, self._tilt_off)
        import mujoco
        qwxyz = np.zeros(4)
        mujoco.mju_mat2Quat(qwxyz, R.flatten())
        (msg.pose.orientation.w, msg.pose.orientation.x,
         msg.pose.orientation.y, msg.pose.orientation.z) = [float(x) for x in qwxyz]
        self._debug_pub.publish(msg)

    def _collect_metrics(self, u, v):
        u_tgt, v_tgt, _ = self._goal
        err = math.hypot(u - u_tgt, v - v_tgt)
        vis = 0.05 < u < 0.95 and 0.05 < v < 0.95

        fmsg = PointStamped()
        fmsg.header.stamp = self.get_clock().now().to_msg()
        fmsg.point.x, fmsg.point.y, fmsg.point.z = u - u_tgt, v - v_tgt, err
        self._framing_pub.publish(fmsg)

        self._stat_frames += 1
        self._stat_err += err
        self._stat_vis += vis
        if self._stat_frames >= self._metrics_window:
            self.get_logger().info(
                f'[验收] 近{self._stat_frames}步  构图误差均值={self._stat_err/self._stat_frames:.4f}  '
                f'可见率={self._stat_vis/self._stat_frames*100:.1f}%')
            self._stat_frames, self._stat_err, self._stat_vis = 0, 0.0, 0


def main():
    rclpy.init()
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
