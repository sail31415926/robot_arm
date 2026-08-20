#!/usr/bin/env python3
"""
@file   emeet_arm_env.py
@brief  robot_arm 摄影机械臂 Gymnasium 强化学习环境 v3（纯 MuJoCo，无 ROS 依赖）

任务：运动主体的可配置构图跟拍（视觉在环 + 执行受限版）。
      每轮随机给定构图目标 (u*, v*, s*)，被摄主体以随机速度游走；
      策略控制相机球坐标轨道 + 视轴偏置，使主体保持在目标构图点且运动平滑。

v3 相对 v2 的两处 sim2real 升级（2026-08-19，用户要求重训）：
  1. 主体位置不再用真值——模拟真实视觉链路（对齐 red_box_detector 的
     接口口径：归一化像素坐标 + 深度）：
       真值投影 → 加噪声/丢帧/1 帧延迟 → (u,v,s,depth) 测量
       → 反投影重建主体 3D 位置 → EMA 滤波 → 供 look-at/轨道中心使用
     观测里的 u,v,s 全部换成测量值（含 valid 标志位）；**真值只用于算奖励**
     （奖励评的是"画面里实际在哪"，这在真实世界也是客观事实）。
  2. 执行链加关节速度+加速度双限幅（joint_limits.yaml 实机口径）：
     IK 目标先过指令整形器（rate limiter）再进 PD，策略必须在可实现的
     动力学内学会平滑跟拍，不能再依赖瞬间大角速度。

动作空间（5维连续，[-1,1]，增量式）：
    a[0]=Δθ  a[1]=Δφ  a[2]=Δr  a[3]=Δpan  a[4]=Δtilt

观测空间（35维）：
    [0]     sin(θ)
    [1]     cos(θ)
    [2]     φ / (π/2)
    [3]     (r - R_MID) / R_HALF
    [4]     pan_off  / OFF_MAX
    [5]     tilt_off / OFF_MAX
    [6:11]  prev_action
    [11]    u_meas - u*            — 构图误差（测量）
    [12]    v_meas - v*
    [13]    (s_meas - s*) / S_HALF
    [14]    u_meas*2-1             — 当前投影（测量）
    [15]    v_meas*2-1
    [16]    (s_meas - S_MID) / S_HALF
    [17]    u**2-1                 — 构图目标（goal-conditioned）
    [18]    v**2-1
    [19]    (s* - S_MID) / S_HALF
    [20]    du（测量图像速度 ×25 截断）
    [21]    dv
    [22]    检测有效标志（1=本步有检测 / 0=丢帧，持有上次测量）
    [23:29] qpos_norm
    [29:35] qvel_norm（按 VEL_LIMIT 归一）
    [35:41] (q_cmd - q) 指令-实际偏差（×5 截断）—— 指令整形器状态①
    [41:47] qd_cmd / VEL_LIMIT               —— 指令整形器状态②
    整形器状态必须可观测：它是动作与执行之间的隐藏动力系统，
    不进观测就是 POMDP 破洞（v3 三轮横盘的元凶之一）。

坐标约定：投影/视轴以 Cam0 body 系 = ROS 光学系（+Z 光轴/+Y 下/+X 右）为准；
v 向下增大，v*=0.67 = 主体在画面下三分之一。

@copyright Copyright (c) 2026 eMeet
"""

import math
import os
from collections import deque

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import mujoco
import mujoco.viewer

# ── 路径 ────────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))

def _find_mjcf() -> str:
    # RL 包本地场景（robot_arm_rl/sim/rl_env.xml），与其他节点解耦
    local_xml = os.path.normpath(os.path.join(_HERE, '..', '..', 'sim', 'rl_env.xml'))
    if os.path.isfile(local_xml):
        return local_xml
    try:
        from ament_index_python.packages import get_package_share_directory
        share_xml = os.path.join(
            get_package_share_directory('robot_arm_rl'), 'sim', 'rl_env.xml')
        if os.path.isfile(share_xml):
            return share_xml
    except Exception:
        pass
    raise FileNotFoundError(f'rl_env.xml 未找到: {local_xml}')

MJCF_PATH = _find_mjcf()

# ── 机械臂常量 ────────────────────────────────────────────────────────────────
JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

# 速度限 = 实机口径（robot_arm_moveit_config/config/joint_limits.yaml）
VEL_LIMIT = np.array([3.14, 3.14, 3.14, 1.0, 1.0, 1.0])       # rad/s
# 加速度限：joint_limits.yaml 的值([1,1,1,0.5,0.25,0.25])是 MoveIt 规划用的
# 保守参数，非驱动器真值——照抄会让云台 tilt 挪 0.1rad 耗时 ~1s，视轴通道
# 动作→效果延迟 50 步，策略学不动（v3 实测三轮横盘的元凶之一）。
# 暂取 8× 规划值，实机辨识后回填。
ACC_LIMIT = 8.0 * np.array([1.0, 1.0, 1.0, 0.5, 0.25, 0.25])  # rad/s²
MAX_VEL   = VEL_LIMIT   # 兼容旧名（观测归一化用）

# 与 rl_env.xml <actuator> 的 kp/kv 一致（改 XML 记得同步）
_KP = np.array([500., 800., 300., 80., 50., 50.])
_KV = np.array([ 60., 100.,  40., 12.,  8.,  8.])
_KV_OVER_KP = _KV / _KP

# ── 球坐标轨道参数 ───────────────────────────────────────────────────────────
THETA_MIN, THETA_MAX = -math.pi, math.pi
PHI_MIN,   PHI_MAX   = -0.5,     0.7
R_MIN,     R_MAX     = 0.20,     0.80
R_MID  = (R_MIN + R_MAX) / 2.0
R_HALF = (R_MAX - R_MIN) / 2.0

# 每步最大增量（每步 = 20ms，50Hz 决策）
DTHETA_MAX = 0.15
DPHI_MAX   = 0.08
DR_MAX     = 0.04
DPAN_MAX   = 0.05
DTILT_MAX  = 0.05
OFF_MAX    = 0.35

# 相机目标位置的可达内域（Cam0 实测臂展 0.674m）
WS_H_MIN, WS_H_MAX = 0.20, 0.60
WS_Z_MIN, WS_Z_MAX = 0.15, 0.85

# ── 相机参数（与 rl_env.xml ee_cam 一致）────────────────────────────────────
CAM_FOVY  = math.radians(70.9)
CAM_W, CAM_H = 640, 480
CAM_FOVX  = 2 * math.atan(math.tan(CAM_FOVY / 2) * CAM_W / CAM_H)

# ── 构图目标采样范围（goal-conditioned）─────────────────────────────────────
U_TGT_RANGE = (0.35, 0.65)
V_TGT_RANGE = (0.30, 0.70)
S_TGT_RANGE = (0.02, 0.10)
DEFAULT_GOAL = (0.50, 0.67, 0.06)
S_MID  = 0.05
S_HALF = 0.05

# ── 被摄主体运动 ─────────────────────────────────────────────────────────────
SUBJ_X_RANGE = (0.65, 1.15)
SUBJ_Y_RANGE = (-0.35, 0.35)
SUBJ_Z       = 0.47
SUBJ_SPEED_MAX = 0.008    # m/step（50Hz → 0.4 m/s）
SUBJ_STATIC_PROB = 0.3

# ── 视觉测量模型（对齐 red_box_detector：归一化像素 + 深度）──────────────────
VIS_SIGMA_UV        = 0.008   # u,v 高斯噪声（≈5px/640）
VIS_SIGMA_S_REL     = 0.10    # 面积相对噪声
VIS_SIGMA_DEPTH_REL = 0.02    # 深度相对噪声（ToF 量级）
VIS_DROPOUT_P       = 0.05    # 单步丢帧概率（丢帧持有上次测量）
VIS_LATENCY_STEPS   = 1       # 检测延迟（步 = 20ms，≈相机一帧）
# 主体 3D 估计的 α-β 跟踪器（look-at 中心的数据源）。
# 不能用轻滤波：估计噪声会直接变成相机物理抖动——苛刻加速度限时代
# 被"顺便"滤掉了，放宽执行后立刻现形（静止基线 +115→-349 实测）。
# α=0.1 实测：静止零动作基线 +199（α=0.2 时 -144——滤波不够，噪声直通执行）
EST_ALPHA = 0.1
EST_BETA  = 0.01
# α-β 跟踪器（检测后处理，观测吃滤波值而非原始测量）——
# 原始测量差分当图像速度时噪声被放大到信号的 20 倍（du 噪声 σ≈0.28 vs
# 信号 ~0.01），策略要么抖要么学会无视预判通道；部署侧同样应接跟踪器
TRACK_ALPHA = 0.5
TRACK_BETA  = 0.15

# ── 仿真参数 ─────────────────────────────────────────────────────────────────
SIM_STEPS_PER_ACTION = 10   # timestep=0.002 → 20ms/action
CTRL_DT = SIM_STEPS_PER_ACTION * 0.002
MAX_EP_STEPS = 500
LOST_STEPS_TERMINATE = 50
IK_MAX_ITER  = 30
IK_TOL_POS   = 1e-3
IK_TOL_ANG   = 1e-2
IK_DLS_LAMBDA = 0.02


# ── 球坐标 / 朝向数学 ────────────────────────────────────────────────────────
def _theta_ref(ox: float, oy: float) -> float:
    return math.atan2(-oy, -ox)


def sphere_to_cart(theta: float, phi: float, r: float,
                   ox: float, oy: float, oz: float):
    """球坐标 → 世界笛卡尔坐标（θ=0 时相机在主体与底座连线上）。"""
    theta_world = _theta_ref(ox, oy) + theta
    cp = math.cos(phi)
    return (ox + r * cp * math.cos(theta_world),
            oy + r * cp * math.sin(theta_world),
            oz + r * math.sin(phi))


def ws_violation(px: float, py: float, pz: float) -> float:
    """相机目标越出可达内域的程度（0 = 在域内）。"""
    h = math.hypot(px, py)
    dh = max(WS_H_MIN - h, h - WS_H_MAX, 0.0)
    dz = max(WS_Z_MIN - pz, pz - WS_Z_MAX, 0.0)
    return dh + dz


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def aim_mat(cam_pos, tgt_pos, pan_off: float = 0.0, tilt_off: float = 0.0):
    """构造 Cam0 光学系目标旋转矩阵（列 = 光学系各轴在世界系）。

    基准：+Z 光轴指向目标、+X 保持水平（roll=0）；再叠加视轴偏置。
    """
    z = np.asarray(tgt_pos, dtype=float) - np.asarray(cam_pos, dtype=float)
    n = np.linalg.norm(z)
    if n < 1e-9:
        return np.eye(3)
    z = z / n
    x = np.cross(z, np.array([0., 0., 1.]))
    xn = np.linalg.norm(x)
    if xn < 1e-6:
        x = np.array([1., 0., 0.]) - z * z[0]
        xn = np.linalg.norm(x)
    x = x / xn
    y = np.cross(z, x)
    R0 = np.column_stack([x, y, z])
    return R0 @ _rot_y(pan_off) @ _rot_x(tilt_off)


def aim_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z,
             pan_off: float = 0.0, tilt_off: float = 0.0):
    """aim_mat 的四元数封装，返回 (qx,qy,qz,qw)。"""
    R = aim_mat((cam_x, cam_y, cam_z), (tgt_x, tgt_y, tgt_z), pan_off, tilt_off)
    q_wxyz = np.zeros(4)
    mujoco.mju_mat2Quat(q_wxyz, R.flatten())
    return (float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]), float(q_wxyz[0]))


class eMeetArmEnv(gym.Env):
    """
    robot_arm 摄影构图跟拍强化学习环境 v3（视觉在环 + 执行受限）。

    参数
    ----
    xml_path        MuJoCo MJCF 路径
    subject_body    被摄主体 mocap body 名称
    subject_motion  是否允许主体运动
    goal            构图目标 (u*, v*, s*)；None = 每轮随机
    vision_noise    视觉噪声总开关/倍率（1.0=标称，0.0=理想视觉，调试用）
    render_mode     'human' 开 MuJoCo viewer
    """

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self,
                 xml_path: str = MJCF_PATH,
                 subject_body: str = 'obj_red_box',
                 subject_motion: bool = True,
                 goal=None,
                 vision_noise: float = 1.0,
                 render_mode=None):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)

        self._jnt_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in JOINT_NAMES]
        self._act_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'act_{n}')
            for n in JOINT_NAMES]
        self._dof_addrs  = [self.model.jnt_dofadr[jid] for jid in self._jnt_ids]
        self._qpos_addrs = [self.model.jnt_qposadr[jid] for jid in self._jnt_ids]

        # IK / 投影基准帧：Cam0 body（ROS 光学系，在云台 J4-6 之后）。
        # 挂 tool0 的 ee_site 是 V2 换代前的旧口径——site 位姿只含 J1-3，
        # 云台无人控制、相机朝向放飞。
        self._cam_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'Cam0')

        self._jnt_lo = np.array([self.model.jnt_range[jid, 0] for jid in self._jnt_ids])
        self._jnt_hi = np.array([self.model.jnt_range[jid, 1] for jid in self._jnt_ids])
        self._q_home = 0.5 * (self._jnt_lo + self._jnt_hi)

        self._subj_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, subject_body)
        self._subj_mocap_id = self.model.body_mocapid[self._subj_body_id]
        assert self._subj_mocap_id >= 0, f'{subject_body} 必须是 mocap body'
        self._subject_motion = subject_motion
        self._vision_noise = float(vision_noise)
        # 课程式训练句柄（训练回调经 env_method('set_difficulty', ...) 动态调整）
        self._static_prob  = SUBJ_STATIC_PROB
        self._speed_scale  = 1.0
        self._lost_terminate = LOST_STEPS_TERMINATE

        self._fixed_goal = tuple(goal) if goal is not None else None
        self._u_tgt, self._v_tgt, self._s_tgt = DEFAULT_GOAL

        # 内部状态
        self._theta = 0.0
        self._phi   = 0.0
        self._r     = 0.5
        self._pan_off  = 0.0
        self._tilt_off = 0.0
        self._prev_action = np.zeros(5, dtype=np.float32)
        self._step_count  = 0
        self._lost_count  = 0
        self._subj_waypoint = np.zeros(2)
        self._subj_speed  = 0.0

        # 指令整形器（速度/加速度限幅）状态
        self._q_cmd  = np.zeros(6)
        self._qd_cmd = np.zeros(6)

        # 视觉测量状态
        self._vis_queue = deque()          # 延迟队列，元素 (u,v,s,depth) 或 None
        self._meas = (0.5, 0.5, S_MID, 0.5)   # 最近有效原始测量 (u,v,s,depth)
        self._meas_valid = True
        self._subj_est = np.array([0.9, 0.0, SUBJ_Z])   # 主体 3D 估计（α-β）
        self._subj_est_vel = np.zeros(3)                # 估计速度（m/step）
        # α-β 跟踪器状态（滤波后的图像位置/速度/面积，观测的数据源）
        self._trk_uv  = np.array([0.5, 0.5])
        self._trk_duv = np.zeros(2)        # 单步（20ms）图像速度
        self._trk_s   = S_MID

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(47,), dtype=np.float32)

        self.render_mode = render_mode
        self._viewer = None

    # ── 公共接口 ────────────────────────────────────────────────────────────

    @property
    def goal(self):
        return (self._u_tgt, self._v_tgt, self._s_tgt)

    @property
    def subject_estimate(self):
        """当前主体 3D 位置估计（视觉重建 + EMA），部署侧对应检测反投影。"""
        return tuple(self._subj_est)

    def set_difficulty(self, static_prob: float = None,
                       vision_noise: float = None,
                       speed_scale: float = None,
                       lost_steps: int = None):
        """课程式训练接口：动态调任务难度（下一次 reset 生效）。

        lost_steps：丢失主体多少步提前终止。课程早期应放大（如 500=不终止），
        否则"出画→检测丢→估计冻结→look-at 指向陈旧位置"的闭环让 episode
        秒死，策略永远吃不到"丢失-惩罚-找回"的恢复经验（v3 实测 125 步均亡）。"""
        if static_prob is not None:
            self._static_prob = float(static_prob)
        if vision_noise is not None:
            self._vision_noise = float(vision_noise)
        if speed_scale is not None:
            self._speed_scale = float(speed_scale)
        if lost_steps is not None:
            self._lost_terminate = int(lost_steps)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        if self._fixed_goal is not None:
            self._u_tgt, self._v_tgt, self._s_tgt = self._fixed_goal
        else:
            self._u_tgt = self.np_random.uniform(*U_TGT_RANGE)
            self._v_tgt = self.np_random.uniform(*V_TGT_RANGE)
            self._s_tgt = self.np_random.uniform(*S_TGT_RANGE)

        if self._subject_motion and self.np_random.uniform() > self._static_prob:
            self._subj_speed = (self.np_random.uniform(0.2, 1.0)
                                * SUBJ_SPEED_MAX * self._speed_scale)
        else:
            self._subj_speed = 0.0

        # 拒绝采样直到 IK 验证可达（超臂展目标会让 episode 从第一帧就是坏样本）
        for _ in range(30):
            ox = self.np_random.uniform(*SUBJ_X_RANGE)
            oy = self.np_random.uniform(*SUBJ_Y_RANGE)
            self._set_subject_pos(ox, oy, SUBJ_Z)

            self._theta = self.np_random.uniform(-1.2, 1.2)
            self._phi   = self.np_random.uniform(-0.3, 0.5)
            self._r     = self.np_random.uniform(0.30, 0.65)

            px, py, pz = sphere_to_cart(self._theta, self._phi, self._r,
                                        ox, oy, SUBJ_Z)
            if ws_violation(px, py, pz) > 0:
                continue

            self._set_joints(self._q_home)
            q_init = self._ik_solve(np.array([px, py, pz]),
                                    aim_mat((px, py, pz), (ox, oy, SUBJ_Z)))
            self._set_joints(q_init)

            cam_pos = self.data.xpos[self._cam_body_id]
            R = self.data.xmat[self._cam_body_id].reshape(3, 3)
            d = np.array([ox, oy, SUBJ_Z]) - cam_pos
            d /= max(np.linalg.norm(d), 1e-9)
            if (np.linalg.norm(np.array([px, py, pz]) - cam_pos) < 0.02
                    and float(R[:, 2] @ d) > math.cos(math.radians(5.0))):
                break

        self._subj_waypoint = np.array([
            self.np_random.uniform(*SUBJ_X_RANGE),
            self.np_random.uniform(*SUBJ_Y_RANGE)])

        # 指令整形器初始化
        self._q_cmd  = q_init.copy()
        self._qd_cmd = np.zeros(6)

        for _ in range(20):
            self._apply_ctrl(self._q_cmd)
            mujoco.mj_step(self.model, self.data)

        self._pan_off  = 0.0
        self._tilt_off = 0.0
        self._prev_action = np.zeros(5, dtype=np.float32)
        self._step_count  = 0
        self._lost_count  = 0

        # 视觉初始化：强制一帧有效测量播种估计器/跟踪器/延迟队列
        raw = self._sense_raw()
        if raw is None:            # 极端情况兜底：用真值
            u, v, s = self._project_subject()
            raw = (u, v, s, self._true_depth())
        self._meas, self._meas_valid = raw, True
        self._subj_est = self._reconstruct(raw)
        self._subj_est_vel = np.zeros(3)
        self._trk_uv  = np.array([raw[0], raw[1]])
        self._trk_duv = np.zeros(2)
        self._trk_s   = raw[2]
        self._vis_queue = deque([raw] * VIS_LATENCY_STEPS)

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # 球坐标增量（轨道中心 = 主体视觉估计，不是真值）
        theta_new = float(np.clip(self._theta + float(action[0]) * DTHETA_MAX,
                                  THETA_MIN, THETA_MAX))
        phi_new   = float(np.clip(self._phi   + float(action[1]) * DPHI_MAX,
                                  PHI_MIN,   PHI_MAX))
        r_new     = float(np.clip(self._r     + float(action[2]) * DR_MAX,
                                  R_MIN,     R_MAX))

        ox, oy, oz = self._subj_est
        px, py, pz = sphere_to_cart(theta_new, phi_new, r_new, ox, oy, oz)
        cur = sphere_to_cart(self._theta, self._phi, self._r, ox, oy, oz)
        ws_penalty = 0.0
        # 单调回归守卫：域内等价于"必须留在域内"，域外接受不恶化的更新
        if ws_violation(px, py, pz) <= ws_violation(*cur):
            self._theta, self._phi, self._r = theta_new, phi_new, r_new
        else:
            ws_penalty = -0.5
            px, py, pz = cur

        self._pan_off  = float(np.clip(
            self._pan_off  + float(action[3]) * DPAN_MAX,  -OFF_MAX, OFF_MAX))
        self._tilt_off = float(np.clip(
            self._tilt_off + float(action[4]) * DTILT_MAX, -OFF_MAX, OFF_MAX))

        # IK → 指令整形（实机速度/加速度限幅）→ PD
        q_target = self._ik_solve(
            np.array([px, py, pz]),
            aim_mat((px, py, pz), (ox, oy, oz), self._pan_off, self._tilt_off))

        qd_des = (q_target - self._q_cmd) / CTRL_DT
        qd_des = np.clip(qd_des, -VEL_LIMIT, VEL_LIMIT)
        # 预刹车：靠近关节限位时许可速度按 v≤√(2·a·d) 收缩（梯形减速），
        # 否则撞硬限位的瞬时刹车会产生几十倍于 ACC_LIMIT 的加速度尖峰。
        # 刹车加速度取 0.8 倍上限：离散时间贴曲线所需减速度恰为曲线用的 a，
        # 留余量后加速度窗口（满上限）才永远追得上收缩中的速度界
        dist_hi = np.maximum(self._jnt_hi - self._q_cmd, 0.0)
        dist_lo = np.maximum(self._q_cmd - self._jnt_lo, 0.0)
        qd_des = np.clip(qd_des,
                         -np.sqrt(2 * 0.8 * ACC_LIMIT * dist_lo),
                         np.sqrt(2 * 0.8 * ACC_LIMIT * dist_hi))
        qd_new = np.clip(qd_des,
                         self._qd_cmd - ACC_LIMIT * CTRL_DT,
                         self._qd_cmd + ACC_LIMIT * CTRL_DT)
        q_new = np.clip(self._q_cmd + qd_new * CTRL_DT,
                        self._jnt_lo, self._jnt_hi)
        # 速度状态用实际位移反推，保证与 _q_cmd 一致（撞限位后不脱节）
        self._qd_cmd = (q_new - self._q_cmd) / CTRL_DT
        self._q_cmd  = q_new

        for _ in range(SIM_STEPS_PER_ACTION):
            self._apply_ctrl(self._q_cmd)
            mujoco.mj_step(self.model, self.data)

        if self._subj_speed > 0:
            self._step_subject()

        # ── 视觉测量链：真值投影→噪声/丢帧→延迟→跟踪滤波/重建 ──────────────
        self._vis_queue.append(self._sense_raw())
        delayed = self._vis_queue.popleft()
        # α-β 跟踪器：预测一步，有检测则用残差修正，丢帧则纯外推
        pred = self._trk_uv + self._trk_duv
        if delayed is not None:
            self._meas = delayed
            self._meas_valid = True
            resid = np.array([delayed[0], delayed[1]]) - pred
            self._trk_uv  = pred + TRACK_ALPHA * resid
            self._trk_duv = self._trk_duv + TRACK_BETA * resid
            self._trk_s   = (1 - TRACK_ALPHA) * self._trk_s + TRACK_ALPHA * delayed[2]
            est_pred = self._subj_est + self._subj_est_vel
            est_resid = self._reconstruct(delayed) - est_pred
            self._subj_est     = est_pred + EST_ALPHA * est_resid
            self._subj_est_vel = self._subj_est_vel + EST_BETA * est_resid
        else:
            self._meas_valid = False
            self._trk_uv  = pred                # 丢帧：按速度外推
            self._trk_duv = self._trk_duv * 0.98
            self._subj_est = self._subj_est + self._subj_est_vel
            self._subj_est_vel = self._subj_est_vel * 0.99

        # 奖励用真值（画面里实际在哪是客观事实）
        u_true, v_true, s_true = self._project_subject()
        reward = self._compute_reward(action, u_true, v_true, s_true) + ws_penalty
        obs    = self._get_obs()

        self._prev_action = action.copy()
        self._step_count += 1

        # 提前终止按"系统自己知道的"口径：检测无效或测得出画
        u_m, v_m = self._meas[0], self._meas[1]
        seen = self._meas_valid and 0.05 < u_m < 0.95 and 0.05 < v_m < 0.95
        self._lost_count = 0 if seen else self._lost_count + 1
        terminated = self._lost_count >= self._lost_terminate
        if terminated:
            reward -= 10.0
        truncated = self._step_count >= MAX_EP_STEPS

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

    # ── 视觉测量模型 ─────────────────────────────────────────────────────────

    def _true_depth(self) -> float:
        subj = np.array(self._get_subject_pos())
        cam_pos  = self.data.xpos[self._cam_body_id]
        cam_xmat = self.data.xmat[self._cam_body_id].reshape(3, 3)
        return float((cam_xmat.T @ (subj - cam_pos))[2])

    def _sense_raw(self):
        """生成本步检测：真值投影 + 噪声；不可检测（后方/大幅出画/丢帧）返回 None。"""
        u, v, s = self._project_subject()
        depth = self._true_depth()
        if depth < 0.01 or not (-0.2 < u < 1.2 and -0.2 < v < 1.2):
            return None                        # 物理上检测不到
        k = self._vision_noise
        if k > 0 and self.np_random.uniform() < VIS_DROPOUT_P * k:
            return None                        # 随机丢帧
        u = u + self.np_random.normal(0, VIS_SIGMA_UV * k)
        v = v + self.np_random.normal(0, VIS_SIGMA_UV * k)
        s = max(1e-6, s * (1 + self.np_random.normal(0, VIS_SIGMA_S_REL * k)))
        depth = max(0.02, depth * (1 + self.np_random.normal(0, VIS_SIGMA_DEPTH_REL * k)))
        return (float(u), float(v), float(s), float(depth))

    def _reconstruct(self, meas) -> np.ndarray:
        """(u,v,depth) 反投影到世界系（光学系针孔模型，深度沿 +Z）。"""
        u, v, _, depth = meas
        xn = (2 * u - 1) * math.tan(CAM_FOVX / 2)
        yn = (2 * v - 1) * math.tan(CAM_FOVY / 2)
        p_cam = np.array([xn * depth, yn * depth, depth])
        cam_pos  = self.data.xpos[self._cam_body_id]
        cam_xmat = self.data.xmat[self._cam_body_id].reshape(3, 3)
        return cam_pos + cam_xmat @ p_cam

    # ── 观测 ────────────────────────────────────────────────────────────────

    def _get_obs(self) -> np.ndarray:
        q    = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        qvel = np.array([self.data.qvel[a] for a in self._dof_addrs])
        qpos_norm = 2 * (q - self._jnt_lo) / (self._jnt_hi - self._jnt_lo) - 1
        qvel_norm = np.clip(qvel / VEL_LIMIT, -1., 1.)

        # 观测吃跟踪器输出（滤波位置/速度），不吃原始测量
        u, v = float(self._trk_uv[0]), float(self._trk_uv[1])
        s = float(self._trk_s)
        du = float(np.clip(self._trk_duv[0] * 25.0, -1., 1.))
        dv = float(np.clip(self._trk_duv[1] * 25.0, -1., 1.))

        return np.array([
            math.sin(self._theta),
            math.cos(self._theta),
            self._phi / (math.pi / 2),
            (self._r - R_MID) / R_HALF,
            self._pan_off / OFF_MAX,
            self._tilt_off / OFF_MAX,
            *self._prev_action,                       # 6:11
            u - self._u_tgt,                          # 11
            v - self._v_tgt,                          # 12
            (s - self._s_tgt) / S_HALF,               # 13
            u * 2 - 1,                                # 14
            v * 2 - 1,                                # 15
            (s - S_MID) / S_HALF,                     # 16
            self._u_tgt * 2 - 1,                      # 17
            self._v_tgt * 2 - 1,                      # 18
            (self._s_tgt - S_MID) / S_HALF,           # 19
            du,                                       # 20
            dv,                                       # 21
            1.0 if self._meas_valid else 0.0,         # 22
            *qpos_norm,                               # 23:29
            *qvel_norm,                               # 29:35
            *np.clip((self._q_cmd - q) * 5.0, -1., 1.),   # 35:41 整形器状态①
            *np.clip(self._qd_cmd / VEL_LIMIT, -1., 1.),  # 41:47 整形器状态②
        ], dtype=np.float32)

    # ── 奖励（用真值口径）────────────────────────────────────────────────────

    def _compute_reward(self, action: np.ndarray, u, v, s) -> float:
        framing_err2 = min((u - self._u_tgt)**2 + (v - self._v_tgt)**2, 4.5)
        r_framing = -framing_err2 * 8.0

        if 0.05 < u < 0.95 and 0.05 < v < 0.95:
            r_visible = 1.0
        else:
            r_visible = -5.0

        s_err_clip = max(-3., min(3., (s - self._s_tgt) / S_HALF))
        r_size = -s_err_clip ** 2 * 0.5

        r_smooth = -float(np.sum(action**2)) * 0.2
        r_jerk   = -float(np.sum((action - self._prev_action)**2)) * 0.5

        q = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        margin = 0.1
        lo_pen = np.sum(np.maximum(0, margin - (q - self._jnt_lo))**2)
        hi_pen = np.sum(np.maximum(0, margin - (self._jnt_hi - q))**2)
        r_workspace = -(lo_pen + hi_pen) * 0.5

        return r_framing + r_visible + r_size + r_smooth + r_jerk + r_workspace

    # ── 相机投影（真值，仅奖励/测量生成用）───────────────────────────────────

    def _project_subject(self):
        """主体真值 → 图像归一化坐标 (u, v, s)。Cam0 body 系 = ROS 光学系。"""
        subj_pos = np.array(self._get_subject_pos())
        cam_pos  = self.data.xpos[self._cam_body_id]
        cam_xmat = self.data.xmat[self._cam_body_id].reshape(3, 3)

        p_cam = cam_xmat.T @ (subj_pos - cam_pos)
        if p_cam[2] < 0.01:
            return -0.5, -0.5, 0.0

        u_ndc = max(-3., min(3., p_cam[0] / (p_cam[2] * math.tan(CAM_FOVX / 2))))
        v_ndc = max(-3., min(3., p_cam[1] / (p_cam[2] * math.tan(CAM_FOVY / 2))))
        u = (1 + u_ndc) / 2
        v = (1 + v_ndc) / 2

        obj_radius = 0.025
        proj_radius = obj_radius / (p_cam[2] * math.tan(CAM_FOVY / 2))
        s = min(math.pi * proj_radius**2, 1.0)
        return float(u), float(v), float(s)

    # ── IK（雅可比 DLS，目标帧 = Cam0 光学系）──────────────────────────────

    def _ik_solve(self, target_pos: np.ndarray, target_mat: np.ndarray) -> np.ndarray:
        """DLS 迭代 IK。迭代借用 data.qpos 做 FK，返回前必须整体还原——
        否则物理状态被 IK 解覆盖，动力学等效每步瞬移（PD 被绕过）。"""
        q = np.array([self.data.qpos[a] for a in self._qpos_addrs], dtype=float)
        qpos_snapshot = self.data.qpos.copy()

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        for _ in range(IK_MAX_ITER):
            for i, a in enumerate(self._qpos_addrs):
                self.data.qpos[a] = q[i]
            mujoco.mj_fwdPosition(self.model, self.data)

            ee_pos  = self.data.xpos[self._cam_body_id].copy()
            pos_err = target_pos - ee_pos

            ee_mat = self.data.xmat[self._cam_body_id].reshape(3, 3)
            R_err  = target_mat @ ee_mat.T
            trace  = R_err[0, 0] + R_err[1, 1] + R_err[2, 2]
            angle  = math.acos(max(-1., min(1., (trace - 1) / 2)))
            if abs(angle) < 1e-6:
                ori_err = np.zeros(3)
            else:
                ori_err = (angle / (2 * math.sin(angle))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1]])

            # 收敛判据必须含姿态——只看位置会带着几十度朝向误差提前退出
            if np.linalg.norm(pos_err) < IK_TOL_POS and angle < IK_TOL_ANG:
                break

            mujoco.mj_jac(self.model, self.data, jacp, jacr,
                          ee_pos, self._cam_body_id)
            J = np.vstack([jacp, jacr])[:, self._dof_addrs]

            err6 = np.concatenate([pos_err, ori_err * 0.5])
            JJT  = J @ J.T
            lam2 = IK_DLS_LAMBDA ** 2
            dq   = J.T @ np.linalg.solve(JJT + lam2 * np.eye(6), err6)
            q    = np.clip(q + 0.6 * dq, self._jnt_lo, self._jnt_hi)

        self.data.qpos[:] = qpos_snapshot
        mujoco.mj_fwdPosition(self.model, self.data)
        return q

    # ── 主体 / 关节操作 ──────────────────────────────────────────────────────

    def _get_subject_pos(self):
        return tuple(self.data.mocap_pos[self._subj_mocap_id])

    def _set_subject_pos(self, x, y, z):
        self.data.mocap_pos[self._subj_mocap_id] = (x, y, z)

    def _step_subject(self):
        """路点式游走：匀速走向随机路点，到达后换下一个。"""
        ox, oy, oz = self._get_subject_pos()
        to_wp = self._subj_waypoint - np.array([ox, oy])
        dist = float(np.linalg.norm(to_wp))
        if dist < 0.02:
            self._subj_waypoint = np.array([
                self.np_random.uniform(*SUBJ_X_RANGE),
                self.np_random.uniform(*SUBJ_Y_RANGE)])
            return
        step = to_wp / dist * min(self._subj_speed, dist)
        self._set_subject_pos(ox + step[0], oy + step[1], oz)

    def _set_joints(self, q: np.ndarray):
        for i, a in enumerate(self._qpos_addrs):
            self.data.qpos[a] = q[i]
            self.data.qvel[self._dof_addrs[i]] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _apply_ctrl(self, q_target: np.ndarray):
        cur_q = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        for i, aid in enumerate(self._act_ids):
            vel_ff = (q_target[i] - cur_q[i]) / CTRL_DT
            self.data.ctrl[aid] = q_target[i] + _KV_OVER_KP[i] * vel_ff
