#!/usr/bin/env python3
"""
@file   rl_policy_node.py
@brief  eMeetArm RL 策略 ROS2 部署节点

加载训练好的 SAC 模型，在实机（或 MuJoCo 仿真）上运行摄影构图策略。

接口：
  订阅  /joint_states                        sensor_msgs/JointState
  订阅  /rl_policy/subject_pos               geometry_msgs/PointStamped  — 被摄主体世界坐标
  发布  /arm_controller/joint_trajectory     trajectory_msgs/JointTrajectory
  发布  /rl_policy/debug                     geometry_msgs/PoseStamped   — 目标 EEF 位姿（可视化）

参数（ROS2 params）：
  model_path        SAC 模型路径（.zip，不含后缀）
  vec_norm_path     VecNormalize pkl 路径（可选）
  policy_hz         策略频率（默认 10 Hz）
  traj_duration_s   每条轨迹段时长（默认 0.15 s）
  subject_x/y/z     固定主体坐标（当无订阅数据时使用）

前置：
  cd ~/eMeet_ws
  colcon build --packages-select robot_arm_rl robot_arm_description --symlink-install
  source install/setup.bash

用法（MuJoCo 仿真）：
  ros2 launch robot_arm_bringup mujoco.launch.py
  ros2 run robot_arm_rl rl_policy_node \
      --ros-args -p model_path:=/path/to/models/sac_emeet_arm_final \
                 -p vec_norm_path:=/path/to/models/vec_normalize_best.pkl

用法（实机）：
  ros2 launch robot_arm_bringup real.launch.py
  ros2 run robot_arm_rl rl_policy_node \
      --ros-args -p model_path:=/path/to/models/sac_emeet_arm_final \
                 -p subject_x:=0.60 -p subject_y:=-0.12 -p subject_z:=0.47

@copyright Copyright (c) 2026 eMeet
"""

import math
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# stable-baselines3 + VecNormalize
try:
    from stable_baselines3 import SAC
    from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
except ImportError as e:
    raise SystemExit('✗ 请先安装 stable-baselines3：pip install stable-baselines3') from e

# 把 envs/ 加入路径（rl_policy_node 与 envs/ 在同一 lib 目录下）
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))   # scripts/

from envs.emeet_arm_env import (
    eMeetArmEnv,
    JOINT_NAMES, MAX_VEL,
    DTHETA_MAX, DPHI_MAX, DR_MAX,
    THETA_MIN, THETA_MAX, PHI_MIN, PHI_MAX, R_MIN, R_MAX,
    sphere_to_cart, aim_quat,
    R_MID, R_HALF,
)

TRAJ_DT = 0.15   # 每条轨迹段时长 s（足够短以实现平滑在线追踪）


class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('rl_policy_node')

        # ── ROS2 参数 ─────────────────────────────────────────────────────────
        self.declare_parameter('model_path',     '')
        self.declare_parameter('vec_norm_path',  '')
        self.declare_parameter('policy_hz',      10.0)
        self.declare_parameter('traj_duration_s', TRAJ_DT)
        self.declare_parameter('subject_x',      0.60)
        self.declare_parameter('subject_y',     -0.12)
        self.declare_parameter('subject_z',      0.47)

        model_path    = self.get_parameter('model_path').value
        vec_norm_path = self.get_parameter('vec_norm_path').value
        policy_hz     = self.get_parameter('policy_hz').value
        self._traj_dt = self.get_parameter('traj_duration_s').value

        # ── 加载模型 ──────────────────────────────────────────────────────────
        if not model_path:
            self.get_logger().error('参数 model_path 未设置，节点退出')
            raise SystemExit(1)

        self.get_logger().info(f'加载 SAC 模型: {model_path}')
        self._model = SAC.load(model_path)

        # VecNormalize（观测归一化）
        self._vec_norm = None
        if vec_norm_path and os.path.exists(vec_norm_path):
            dummy = DummyVecEnv([lambda: eMeetArmEnv()])
            self._vec_norm = VecNormalize.load(vec_norm_path, dummy)
            self._vec_norm.training = False
            self._vec_norm.norm_reward = False
            self.get_logger().info(f'  VecNormalize 已加载: {vec_norm_path}')
        else:
            self.get_logger().warn('  未找到 VecNormalize，将直接使用原始观测（可能影响性能）')

        # ── 共享状态 ──────────────────────────────────────────────────────────
        self._lock = threading.Lock()

        # 关节状态（来自 /joint_states）
        self._joint_pos = np.zeros(6)
        self._js_ready  = False

        # 被摄主体位置
        self._subj_x = self.get_parameter('subject_x').value
        self._subj_y = self.get_parameter('subject_y').value
        self._subj_z = self.get_parameter('subject_z').value

        # 当前球坐标（策略内部状态）
        self._theta = 0.0
        self._phi   = 0.2
        self._r     = 0.50
        self._prev_action = np.zeros(3, dtype=np.float32)

        # ── ROS2 接口 ─────────────────────────────────────────────────────────
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self._subj_sub = self.create_subscription(
            PointStamped, '/rl_policy/subject_pos', self._subj_cb, 10)

        self._traj_pub  = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._debug_pub = self.create_publisher(
            PoseStamped, '/rl_policy/debug', 10)

        # 策略定时器
        period = 1.0 / max(policy_hz, 1.0)
        self._policy_timer = self.create_timer(period, self._policy_step)

        self.get_logger().info(
            f'RL 策略节点就绪  hz={policy_hz:.1f}  traj_dt={self._traj_dt:.2f}s')

    # ── 订阅回调 ────────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        n2p = dict(zip(msg.name, msg.position))
        with self._lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in n2p:
                    self._joint_pos[i] = n2p[name]
            self._js_ready = True

    def _subj_cb(self, msg: PointStamped):
        with self._lock:
            self._subj_x = msg.point.x
            self._subj_y = msg.point.y
            self._subj_z = msg.point.z

    # ── 策略主循环 ──────────────────────────────────────────────────────────────

    def _policy_step(self):
        with self._lock:
            if not self._js_ready:
                return
            q    = self._joint_pos.copy()
            ox   = self._subj_x
            oy   = self._subj_y
            oz   = self._subj_z
            theta, phi, r = self._theta, self._phi, self._r
            prev = self._prev_action.copy()

        # 构建观测（与 eMeetArmEnv._get_obs 一致）
        obs = self._build_obs(q, theta, phi, r, prev, ox, oy, oz)

        # 归一化
        if self._vec_norm is not None:
            obs_norm = self._vec_norm.normalize_obs(obs.reshape(1, -1))[0]
        else:
            obs_norm = obs

        # 策略推理（确定性）
        action, _ = self._model.predict(obs_norm, deterministic=True)
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # 更新球坐标
        new_theta = float(np.clip(theta + action[0] * DTHETA_MAX, THETA_MIN, THETA_MAX))
        new_phi   = float(np.clip(phi   + action[1] * DPHI_MAX,   PHI_MIN,   PHI_MAX))
        new_r     = float(np.clip(r     + action[2] * DR_MAX,     R_MIN,     R_MAX))

        # 目标笛卡尔 + 姿态
        px, py, pz = sphere_to_cart(new_theta, new_phi, new_r, ox, oy, oz)
        qx, qy, qz, qw = aim_quat(px, py, pz, ox, oy, oz)

        # 发布目标轨迹（单路点，持续 traj_dt 秒）
        self._send_trajectory([(px, py, pz, qx, qy, qz, qw)], q)
        self._publish_debug(px, py, pz, qx, qy, qz, qw)

        with self._lock:
            self._theta = new_theta
            self._phi   = new_phi
            self._r     = new_r
            self._prev_action = action

    # ── 辅助：构建观测向量 ─────────────────────────────────────────────────────

    def _build_obs(self, q, theta, phi, r, prev_action, ox, oy, oz) -> np.ndarray:
        from envs.emeet_arm_env import (
            eMeetArmEnv, U_TGT, V_TGT, S_TGT, S_MID, S_HALF,
            CAM_FOVX, CAM_FOVY,
        )
        # 关节归一化（使用 eMeetArmEnv 中的关节限位）
        env_tmp = _get_dummy_env()
        jlo, jhi = env_tmp._jnt_lo, env_tmp._jnt_hi
        qpos_norm = np.clip(2 * (q - jlo) / (jhi - jlo) - 1, -1., 1.)
        qvel_norm = np.zeros(6)   # 部署时关节速度从 JointState 读取（此处简化为零）

        # 相机投影：暂用几何估算（部署时应接入真实感知）
        u, v, s = self._estimate_projection(theta, phi, r, ox, oy, oz)

        u_err = u - U_TGT
        v_err = v - V_TGT
        s_err = (s - S_TGT) / S_HALF

        return np.array([
            math.sin(theta), math.cos(theta),
            phi / (math.pi / 2),
            (r - R_MID) / R_HALF,
            prev_action[0], prev_action[1], prev_action[2],
            u_err, v_err, s_err,
            u * 2 - 1, v * 2 - 1, (s - S_MID) / S_HALF,
            *qpos_norm,
            *qvel_norm,
        ], dtype=np.float32)

    def _estimate_projection(self, theta, phi, r, ox, oy, oz):
        """几何估算主体在画面中的位置（无需渲染）。
        当启用真实感知时，此函数应替换为检测框结果。
        """
        from envs.emeet_arm_env import sphere_to_cart, aim_quat, CAM_FOVX, CAM_FOVY
        import numpy as np

        px, py, pz = sphere_to_cart(theta, phi, r, ox, oy, oz)
        # 相机朝向由 aim_quat 确定，光轴对准主体时误差约为 0
        # 此处近似：u=0.5, v=V_TGT（相机对准时的理想值）
        # 实际部署中应使用 YOLO / ToF 检测结果
        dx = ox - px; dy = oy - py; dz = oz - pz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz) + 1e-6
        # 计算到主体的方向在相机 xy 平面的投影角
        u_err_rad = math.atan2(dy, dx) - math.atan2(oy - py, ox - px)
        v_err_rad = math.asin(max(-1., min(1., -dz / dist)))
        u = 0.5 + u_err_rad / CAM_FOVX
        v = 0.5 + v_err_rad / CAM_FOVY
        # 面积估算
        obj_r = 0.025
        proj_r = obj_r / (dist * math.tan(CAM_FOVY / 2))
        s = math.pi * proj_r ** 2
        return float(u), float(v), float(s)

    # ── 辅助：发布轨迹 ─────────────────────────────────────────────────────────

    def _send_trajectory(self, waypoints, current_q):
        """发布一段短轨迹（通过 MoveIt IK 获取关节目标）。
        当前简化版本：只发目标关节角（需 IK，此处省略 MoveIt 调用，
        用 cartesian_ruckig_ik_streamer 接管实际 IK）。
        实际部署建议接入 /compute_ik 服务。
        """
        # 直接向 spherical_orbit_streamer 风格发送空轨迹作为占位
        # 完整 IK 对接请参考 spherical_orbit_streamer._ik_sync
        pass

    def _publish_debug(self, px, py, pz, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._debug_pub.publish(msg)


# ── 全局缓存（避免重复构建 env 对象）──────────────────────────────────────────
_dummy_env_cache = None

def _get_dummy_env():
    global _dummy_env_cache
    if _dummy_env_cache is None:
        _dummy_env_cache = eMeetArmEnv.__new__(eMeetArmEnv)
        import mujoco
        from envs.emeet_arm_env import MJCF_PATH, JOINT_NAMES
        _dummy_env_cache.model = mujoco.MjModel.from_xml_path(MJCF_PATH)
        _dummy_env_cache.data  = mujoco.MjData(_dummy_env_cache.model)
        _dummy_env_cache._jnt_ids = [
            mujoco.mj_name2id(_dummy_env_cache.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in JOINT_NAMES]
        _dummy_env_cache._jnt_lo = np.array(
            [_dummy_env_cache.model.jnt_range[j, 0]
             for j in _dummy_env_cache._jnt_ids])
        _dummy_env_cache._jnt_hi = np.array(
            [_dummy_env_cache.model.jnt_range[j, 1]
             for j in _dummy_env_cache._jnt_ids])
    return _dummy_env_cache


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
