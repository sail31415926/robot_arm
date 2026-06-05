#!/usr/bin/env python3
"""
@file   emeet_arm_env.py
@brief  eMeetArm 摄影机械臂 Gymnasium 强化学习环境（纯 MuJoCo，无 ROS 依赖）

任务：给定被摄主体位置，控制相机机械臂在球坐标空间内运动，
      使主体始终出现在画面目标位置（三分之一构图），运动平滑。

动作空间（3维连续，归一化 [-1, 1]）：
    a[0] = Δθ  方位角增量 → 实际 ±DTHETA_MAX rad
    a[1] = Δφ  仰角增量   → 实际 ±DPHI_MAX   rad
    a[2] = Δr  半径增量   → 实际 ±DR_MAX      m

观测空间（25维）：
    [0]   sin(θ)                     — 方位角正弦（消除 ±π 不连续）
    [1]   cos(θ)                     — 方位角余弦
    [2]   φ / (π/2)                  — 仰角归一化 [-1,1]
    [3]   (r - R_MID) / R_HALF       — 半径归一化 [-1,1]
    [4:7] prev_action                — 上一步动作（平滑感知）
    [7]   u_err  = u - u_tgt         — 水平构图误差 [-1,1]
    [8]   v_err  = v - v_tgt         — 垂直构图误差 [-1,1]
    [9]   s_err  = s - s_tgt         — 尺寸误差
    [10]  u                          — 主体在画面中的归一化 x [0,1]
    [11]  v                          — 主体在画面中的归一化 y [0,1]
    [12]  s                          — 主体在画面中的归一化面积
    [13:19] qpos_norm                — 关节角归一化 [-1,1]
    [19:25] qvel_norm                — 关节速度归一化 [-1,1]

@copyright Copyright (c) 2026 eMeet
"""

import math
import os

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import mujoco
import mujoco.viewer

# ── 路径 ────────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))

def _find_mjcf() -> str:
    # 优先使用 ament_index（ros2 run / colcon install 环境）
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('robot_arm_description'), 'xml', 'eMeetArm.xml')
    except Exception:
        pass
    # 回退：从源码目录直接运行（python3 train_sac.py）
    return os.path.normpath(os.path.join(
        _HERE, '..', '..', '..', 'robot_arm_description', 'xml', 'eMeetArm.xml'))

MJCF_PATH = _find_mjcf()

# ── 机械臂常量 ────────────────────────────────────────────────────────────────
JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
MAX_VEL     = np.array([3.14, 3.14, 3.14, 1.0, 1.0, 1.0])   # rad/s

_KP = np.array([500., 800., 300., 80., 50., 50.])
_KV = np.array([ 60., 100.,  40., 12.,  8.,  8.])
_KV_OVER_KP = _KV / _KP

# ── 球坐标参数 ────────────────────────────────────────────────────────────────
THETA_MIN, THETA_MAX = -math.pi, math.pi   # 方位角
PHI_MIN,   PHI_MAX   = -0.5,     0.7       # 仰角 rad（摄影常用范围）
R_MIN,     R_MAX     = 0.20,     0.80      # 轨道半径 m
R_MID  = (R_MIN + R_MAX) / 2.0
R_HALF = (R_MAX - R_MIN) / 2.0

# 每步最大增量
DTHETA_MAX = 0.15    # rad  (~8.6°)
DPHI_MAX   = 0.08    # rad  (~4.6°)
DR_MAX     = 0.04    # m

# ── 相机参数（与 eMeetArm.xml 一致）─────────────────────────────────────────
CAM_FOVY  = math.radians(70.9)          # 垂直 FOV
CAM_W, CAM_H = 640, 480
CAM_FOVX  = 2 * math.atan(math.tan(CAM_FOVY / 2) * CAM_W / CAM_H)  # ~87°

# 目标构图参数（三分之一构图：主体在画面 (0.5, 0.67)）
U_TGT = 0.50   # 水平居中
V_TGT = 0.67   # 垂直偏上（三分之一）
S_TGT = 0.06   # 主体面积占画面比例（典型值）
S_MID = 0.05
S_HALF = 0.05

# ── 仿真参数 ─────────────────────────────────────────────────────────────────
SIM_STEPS_PER_ACTION = 10   # 每次 action 推进的物理步数（timestep=0.002 → 20ms/action）
MAX_EP_STEPS = 500
IK_MAX_ITER  = 30
IK_TOL_POS   = 1e-3   # m
IK_DLS_LAMBDA = 0.02


# ── 球坐标数学（复用 spherical_orbit_streamer 中的公式）────────────────────────
def _theta_ref(ox: float, oy: float) -> float:
    return math.atan2(-oy, -ox)


def sphere_to_cart(theta: float, phi: float, r: float,
                   ox: float, oy: float, oz: float):
    """球坐标 → 世界笛卡尔坐标。"""
    theta_world = _theta_ref(ox, oy) + theta
    cp = math.cos(phi)
    return (ox + r * cp * math.cos(theta_world),
            oy + r * cp * math.sin(theta_world),
            oz + r * math.sin(phi))


def aim_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z):
    """EEF X 轴朝向目标，roll 固定 90°（相机光轴沿 EEF X）。返回 (qx,qy,qz,qw)。"""
    dx, dy, dz = tgt_x - cam_x, tgt_y - cam_y, tgt_z - cam_z
    n = math.sqrt(dx*dx + dy*dy + dz*dz)
    if n < 1e-9:
        return _rpy_to_quat(math.pi / 2, 0., 0.)
    dx, dy, dz = dx/n, dy/n, dz/n
    pitch = math.asin(max(-1., min(1., -dz)))
    yaw   = math.atan2(dy, dx)
    return _rpy_to_quat(math.pi / 2, pitch, yaw)


def _rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll  * .5); sr = math.sin(roll  * .5)
    cp = math.cos(pitch * .5); sp = math.sin(pitch * .5)
    cy = math.cos(yaw   * .5); sy = math.sin(yaw   * .5)
    return (sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy)


class eMeetArmEnv(gym.Env):
    """
    eMeetArm 摄影构图强化学习环境。

    参数
    ----
    xml_path        MuJoCo MJCF 路径（默认自动定位）
    subject_body    被摄主体 body 名称（默认 obj_red_box）
    subject_motion  是否允许主体随机游走（训练多样性）
    render_mode     'human' 开启 MuJoCo viewer，None 关闭
    """

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self,
                 xml_path: str = MJCF_PATH,
                 subject_body: str = 'obj_red_box',
                 subject_motion: bool = False,
                 render_mode=None):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)

        # 关节 / 执行器 ID
        self._jnt_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in JOINT_NAMES]
        self._act_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'act_{n}')
            for n in JOINT_NAMES]
        self._dof_addrs = [self.model.jnt_dofadr[jid] for jid in self._jnt_ids]
        self._qpos_addrs = [self.model.jnt_qposadr[jid] for jid in self._jnt_ids]

        # 末端 site / body
        self._ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
        self._ee_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'tool0')
        self._cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'ee_cam')

        # 关节限位
        self._jnt_lo = np.array([self.model.jnt_range[jid, 0] for jid in self._jnt_ids])
        self._jnt_hi = np.array([self.model.jnt_range[jid, 1] for jid in self._jnt_ids])

        # 被摄主体
        self._subj_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, subject_body)
        self._subject_motion = subject_motion

        # 状态
        self._theta = 0.0
        self._phi   = 0.0
        self._r     = 0.5
        self._prev_action = np.zeros(3)
        self._step_count  = 0

        # Gym spaces
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(25,), dtype=np.float32)

        self.render_mode = render_mode
        self._viewer = None

    # ── 公共接口 ────────────────────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        # 随机化主体位置（工作台区域）
        ox = self.np_random.uniform(0.65, 1.15)
        oy = self.np_random.uniform(-0.35, 0.35)
        oz = 0.47
        self._set_subject_pos(ox, oy, oz)

        # 随机化起始球坐标
        self._theta = self.np_random.uniform(-1.2, 1.2)
        self._phi   = self.np_random.uniform(-0.3, 0.5)
        self._r     = self.np_random.uniform(0.30, 0.65)

        # IK 到初始位姿
        px, py, pz = sphere_to_cart(self._theta, self._phi, self._r, ox, oy, oz)
        qx, qy, qz, qw = aim_quat(px, py, pz, ox, oy, oz)
        q_init = self._ik_solve(
            np.array([px, py, pz]),
            np.array([qx, qy, qz, qw]))
        self._set_joints(q_init)

        # 步进几步让仿真稳定
        for _ in range(20):
            self._apply_ctrl(q_init)
            mujoco.mj_step(self.model, self.data)

        self._prev_action = np.zeros(3)
        self._step_count  = 0

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # 球坐标增量
        dtheta = float(action[0]) * DTHETA_MAX
        dphi   = float(action[1]) * DPHI_MAX
        dr     = float(action[2]) * DR_MAX

        self._theta = float(np.clip(self._theta + dtheta, THETA_MIN, THETA_MAX))
        self._phi   = float(np.clip(self._phi   + dphi,   PHI_MIN,   PHI_MAX))
        self._r     = float(np.clip(self._r     + dr,     R_MIN,     R_MAX))

        # 主体位置
        ox, oy, oz = self._get_subject_pos()

        # 目标 EEF 笛卡尔 + 姿态
        px, py, pz = sphere_to_cart(self._theta, self._phi, self._r, ox, oy, oz)
        qx, qy, qz, qw = aim_quat(px, py, pz, ox, oy, oz)

        # IK → 关节目标
        q_target = self._ik_solve(
            np.array([px, py, pz]),
            np.array([qx, qy, qz, qw]))

        # 物理仿真
        for _ in range(SIM_STEPS_PER_ACTION):
            self._apply_ctrl(q_target)
            mujoco.mj_step(self.model, self.data)

        # 主体随机游走（可选）
        if self._subject_motion:
            self._step_subject()

        obs    = self._get_obs()
        reward = self._compute_reward(action)
        self._prev_action = action.copy()
        self._step_count += 1

        terminated = False
        truncated  = self._step_count >= MAX_EP_STEPS

        if self.render_mode == 'human':
            self.render()

        return obs, reward, terminated, truncated, {}

    def render(self):
        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        if self._viewer.is_running():
            self._viewer.sync()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    # ── 观测 ────────────────────────────────────────────────────────────────────

    def _get_obs(self) -> np.ndarray:
        # 关节状态
        q    = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        qvel = np.array([self.data.qvel[a] for a in self._dof_addrs])
        qpos_norm = 2 * (q - self._jnt_lo) / (self._jnt_hi - self._jnt_lo) - 1
        qvel_norm = np.clip(qvel / MAX_VEL, -1., 1.)

        # 画面构图
        u, v, s = self._project_subject()
        u_err = u - U_TGT
        v_err = v - V_TGT
        s_err = (s - S_TGT) / S_HALF

        return np.array([
            math.sin(self._theta),                          # 0
            math.cos(self._theta),                          # 1
            self._phi / (math.pi / 2),                      # 2
            (self._r - R_MID) / R_HALF,                     # 3
            self._prev_action[0],                           # 4
            self._prev_action[1],                           # 5
            self._prev_action[2],                           # 6
            u_err,                                          # 7
            v_err,                                          # 8
            s_err,                                          # 9
            u * 2 - 1,                                      # 10  u 归一化到 [-1,1]
            v * 2 - 1,                                      # 11
            (s - S_MID) / S_HALF,                           # 12
            *qpos_norm,                                     # 13-18
            *qvel_norm,                                     # 19-24
        ], dtype=np.float32)

    # ── 奖励 ────────────────────────────────────────────────────────────────────

    def _compute_reward(self, action: np.ndarray) -> float:
        u, v, s = self._project_subject()

        # 1. 构图奖励：主体靠近三分之一构图点
        # 截断防止偏移画面时误差爆炸（max 误差 = 2 帧宽，≈ 1.5²+1.5² ≈ 4.5）
        framing_err2 = min((u - U_TGT)**2 + (v - V_TGT)**2, 4.5)
        r_framing = -framing_err2 * 8.0

        # 2. 可见性奖励：主体在画面内
        if 0.05 < u < 0.95 and 0.05 < v < 0.95:
            r_visible = 1.0
        else:
            r_visible = -5.0

        # 3. 尺寸奖励：主体大小适中（误差截断到 ±3，防止过近时爆炸）
        s_err_clip = max(-3., min(3., (s - S_TGT) / S_HALF))
        r_size = -s_err_clip ** 2 * 0.5

        # 4. 平滑惩罚：抑制大幅动作（防止画面抖动）
        r_smooth = -float(np.sum(action**2)) * 0.3

        # 5. 抖动惩罚：连续帧动作变化
        r_jerk = -float(np.sum((action - self._prev_action)**2)) * 0.5

        # 6. 工作空间边界惩罚
        q = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        margin = 0.1
        lo_pen = np.sum(np.maximum(0, margin - (q - self._jnt_lo))**2)
        hi_pen = np.sum(np.maximum(0, margin - (self._jnt_hi - q))**2)
        r_workspace = -(lo_pen + hi_pen) * 0.5

        return r_framing + r_visible + r_size + r_smooth + r_jerk + r_workspace

    # ── 辅助：相机投影 ─────────────────────────────────────────────────────────

    def _project_subject(self):
        """将主体世界坐标投影到相机归一化图像坐标 (u, v, s)。
        u, v ∈ [0,1]；s = 主体等效面积占画面比例（估算）。
        若主体在相机后方则返回 (-0.5, -0.5, 0)。
        """
        subj_pos = self._get_subject_pos()

        # 相机位姿（世界系）
        cam_pos  = self.data.cam_xpos[self._cam_id].copy()   # (3,)
        cam_xmat = self.data.cam_xmat[self._cam_id].reshape(3, 3)  # 列=相机轴

        # 主体在相机坐标系中的位置
        p_cam = cam_xmat.T @ (np.array(subj_pos) - cam_pos)

        if p_cam[2] < 0.01:
            return -0.5, -0.5, 0.0

        # 针孔投影（MuJoCo 相机：X 右，Y 下，Z 前）
        # 截断到 ±3 倍视野宽度，防止主体偏离时误差爆炸
        u_ndc = max(-3., min(3., p_cam[0] / (p_cam[2] * math.tan(CAM_FOVX / 2))))
        v_ndc = max(-3., min(3., p_cam[1] / (p_cam[2] * math.tan(CAM_FOVY / 2))))
        u = (1 + u_ndc) / 2   # 截断后范围 [-1, 2]
        v = (1 + v_ndc) / 2

        # 主体等效面积：按距离估算，截断防止过近时爆炸
        obj_radius = 0.025
        proj_radius = obj_radius / (p_cam[2] * math.tan(CAM_FOVY / 2))
        s = min(math.pi * proj_radius**2, 1.0)

        return float(u), float(v), float(s)

    # ── 辅助：IK（雅可比 DLS）──────────────────────────────────────────────────

    def _ik_solve(self, target_pos: np.ndarray,
                  target_quat_xyzw: np.ndarray) -> np.ndarray:
        """Jacobian DLS 迭代 IK，从当前关节角出发。返回关节角数组 (6,)。"""
        q = np.array([self.data.qpos[a] for a in self._qpos_addrs], dtype=float)

        # MuJoCo 四元数约定 [w,x,y,z]，aim_quat 返回 [x,y,z,w]
        qx, qy, qz, qw = target_quat_xyzw
        target_quat_mj = np.array([qw, qx, qy, qz])

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        for _ in range(IK_MAX_ITER):
            # 前向运动学
            for i, a in enumerate(self._qpos_addrs):
                self.data.qpos[a] = q[i]
            mujoco.mj_fwdPosition(self.model, self.data)

            # 位置误差
            ee_pos = self.data.site_xpos[self._ee_site_id].copy()
            pos_err = target_pos - ee_pos
            if np.linalg.norm(pos_err) < IK_TOL_POS:
                break

            # 姿态误差（轴角）
            ee_mat  = self.data.site_xmat[self._ee_site_id].reshape(3, 3)
            tgt_mat = np.zeros(9)
            mujoco.mju_quat2Mat(tgt_mat, target_quat_mj)
            tgt_mat = tgt_mat.reshape(3, 3)
            R_err   = tgt_mat @ ee_mat.T
            trace   = R_err[0, 0] + R_err[1, 1] + R_err[2, 2]
            angle   = math.acos(max(-1., min(1., (trace - 1) / 2)))
            if abs(angle) < 1e-6:
                ori_err = np.zeros(3)
            else:
                ori_err = (angle / (2 * math.sin(angle))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1]])

            # 雅可比（取关节列）
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self._ee_site_id)
            J = np.vstack([jacp, jacr])[:, self._dof_addrs]

            err6 = np.concatenate([pos_err, ori_err * 0.5])  # 降低姿态权重
            JJT  = J @ J.T
            lam2 = IK_DLS_LAMBDA ** 2
            dq   = J.T @ np.linalg.solve(JJT + lam2 * np.eye(6), err6)
            q    = np.clip(q + 0.6 * dq, self._jnt_lo, self._jnt_hi)

        return q

    # ── 辅助：主体 / 关节操作 ──────────────────────────────────────────────────

    def _get_subject_pos(self):
        return tuple(self.data.xpos[self._subj_body_id].copy())

    def _set_subject_pos(self, x, y, z):
        """通过 qpos 直接设置自由体位置（free joint: 3 pos + 4 quat）。"""
        # 自由关节无名称，通过 body_jntadr 找到关节地址
        jntadr = self.model.body_jntadr[self._subj_body_id]
        qadr   = self.model.jnt_qposadr[jntadr]
        self.data.qpos[qadr]     = x
        self.data.qpos[qadr + 1] = y
        self.data.qpos[qadr + 2] = z
        # 保持四元数单位（w=1，旋转为零）
        self.data.qpos[qadr + 3] = 1.0
        self.data.qpos[qadr + 4] = 0.0
        self.data.qpos[qadr + 5] = 0.0
        self.data.qpos[qadr + 6] = 0.0

    def _step_subject(self):
        """让主体在工作台上做缓慢随机游走（速度 0.02 m/step）."""
        ox, oy, oz = self._get_subject_pos()
        ox = float(np.clip(ox + self.np_random.uniform(-0.005, 0.005), 0.65, 1.15))
        oy = float(np.clip(oy + self.np_random.uniform(-0.005, 0.005), -0.35, 0.35))
        self._set_subject_pos(ox, oy, oz)

    def _set_joints(self, q: np.ndarray):
        for i, a in enumerate(self._qpos_addrs):
            self.data.qpos[a] = q[i]
            self.data.qvel[self._dof_addrs[i]] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _apply_ctrl(self, q_target: np.ndarray):
        """写入位置执行器指令（含速度前馈）。"""
        cur_q = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        for i, aid in enumerate(self._act_ids):
            vel_ff = (q_target[i] - cur_q[i]) / (self.model.opt.timestep * SIM_STEPS_PER_ACTION)
            self.data.ctrl[aid] = q_target[i] + _KV_OVER_KP[i] * vel_ff
