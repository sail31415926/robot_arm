#!/usr/bin/env python3
"""
@file   orbit_env.py
@brief  robot_arm 环绕运镜 RL 环境（eye-in-hand，末端笛卡尔速度动作，纯 MuJoCo）

任务：目标物体静止（阶段4允许微移）；相机从弧线一端出发，以近似恒定
      半径绕目标水平环绕一段弧，全程保持目标在画面中心、俯仰基本水平。

为什么不是 180°：实机 Cam0 臂展仅 0.674 m 且底座固定，环绕弧上限由
主体位置与半径决定。2026-08-20 用本环境同款 IK 扫描（判据：位置<2cm
且光轴对准<5°，φ=0）：
      ox=0.60: r=0.35→145°  r=0.45→130°  r=0.55→120°
      ox=0.70: r=0.35→115°  r=0.50→110°
      ox=0.80: r=0.40→ 90°  r=0.55→ 95°
      ox=0.90: r=0.35→ 50°  r=0.50→ 75°   ← 构图跟拍任务的桌心，环绕不适用
  → 训练默认主体 x∈[0.60,0.75]，期望半径 RHO_D=0.40，课程弧度上限 120°；
    每轮 reset 先按工作空间几何裁剪出连续可行弧，再用 IK 验证端点，
    弧度目标 = min(课程采样值, 可行弧跨度)。

坐标口径（与 emeet_arm_env 一致）：
  * 相机帧 = Cam0 body 系 = ROS 光学系（+Z 光轴 / +Y 下 / +X 右）。
  * θ=0 表示相机位于"底座→主体"连线的底座一侧；θ 沿世界 Z 逆时针为正。
  * roll=0（图像 +X 保持水平）是仓库统一口径（robot_arm_matlab 定义）。

动作空间（6维连续，[-1,1]，**Cam0 光学系下的末端笛卡尔速度**）：
    a[0:3] = v_xyz  线速度 → ±V_MAX  (0.2 m/s)
    a[3:6] = ω_xyz  角速度 → ±W_MAX  (0.5 rad/s)
  经 DLS 微分逆解 → 关节速度（逐关节限幅）→ 积分为位置目标 → PD 执行器。
  决策频率 20 Hz（每步 = 25 × 2ms 物理步），与实机 JTC 低频位置流一致。

观测空间（30维）：
    [0:3]   target_pos_in_cam / 0.8      目标在相机系的位置（规格必选项）
    [3:5]   u_n, v_n                     目标像素坐标归一化 [-1,1]（必选项）
    [5]     (rho - rho_d) / 0.3          距离误差（必选项 rho 的中心化版本）
    [6:8]   sin(θ), cos(θ)               环绕方位角（必选项，消 ±π 不连续）
    [8]     φ / (π/2)                    俯仰角（必选项）
    [9]     progress / π                 已完成弧度（带方向符号累计）
    [10]    (goal_arc - progress) / π    剩余弧度
    [11]    dir                          环绕方向 ±1（不给方向策略无从知道往哪转）
    [12:18] last_action                  上一帧动作（必选项，平滑感知）
    [18:24] qpos_norm                    关节角（可选项，限位感知）
    [24:30] qvel_norm                    关节速度（可选项）

奖励（详见 _compute_reward；权重 = 规格默认值，w1..w5 = 10/5/2/0.5/1）：
    r_center / r_progress / r_distance / r_smooth / r_safe(限位+奇异+水平)
    事件：碰撞/出视野/持续偏心/关节顶限位 → -10 终止；完成 → +50 终止。

课程阶段（set_stage 热切换，供训练回调用）：
    1  静态对准：goal_arc=0，只启用 r_center + r_smooth
    2  小弧环绕：goal_arc ∈ [10°, 30°]，加入 r_progress + r_distance
    3  长弧环绕：goal_arc ∈ [30°, arc_max]，arc_max 由回调从 60° 提到 120°，全奖励
    4  域随机化：主体微移 + 初始位姿扰动 + 观测噪声 + rho_d 抖动

已知简化（上实机前必读）：
  * MJCF 碰撞对只有 臂↔世界（contype/conaffinity 未配自碰撞），自碰撞
    检测缺失；底座 geom 与地面常接触，已从碰撞判定中排除。
  * 主体是 mocap 体不参与碰撞，相机撞穿主体只能靠 r_distance 间接抑制。
  * 感知是几何真值投影；实机要换检测框 + 标定内参。

@copyright Copyright (c) 2026 eMeet
"""

import math
import os

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import mujoco
import mujoco.viewer

# ── 路径（与 emeet_arm_env 共用同一场景）─────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, '..', '..', 'sim', 'rl_env.xml'))

# ── 机械臂常量 ────────────────────────────────────────────────────────────────
JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
MAX_VEL     = np.array([3.14, 3.14, 3.14, 1.0, 1.0, 1.0])   # rad/s

# 与 rl_env.xml <actuator> 的 kp/kv 一致（改 XML 记得同步）
_KP = np.array([500., 800., 300., 80., 50., 50.])
_KV = np.array([ 60., 100.,  40., 12.,  8.,  8.])
_KV_OVER_KP = _KV / _KP

# ── 动作 / 控制 ──────────────────────────────────────────────────────────────
V_MAX = 0.2     # m/s   线速度上限（规格值）
W_MAX = 0.5     # rad/s 角速度上限（规格值）
CTRL_HZ = 20    # 决策频率（规格建议 10~20Hz，取上限贴近实机 JTC 流）
SIM_STEPS_PER_ACTION = 25          # timestep=0.002 → 50ms/action
DT_ACTION = 0.002 * SIM_STEPS_PER_ACTION
DLS_LAMBDA = 0.05                  # 微分逆解阻尼（奇异附近自动降速）

# ── 相机内参（由 rl_env.xml ee_cam 的 fovy/resolution 推得，方形像素）────────
CAM_W, CAM_H = 640, 480
CAM_FOVY = math.radians(70.9)
FY = (CAM_H / 2.0) / math.tan(CAM_FOVY / 2.0)
FX = FY
CX, CY = CAM_W / 2.0, CAM_H / 2.0
K = np.array([[FX, 0., CX],
              [0., FY, CY],
              [0., 0., 1.]])       # 实机部署时整体替换为标定值

# ── 轨道 / 工作空间 ──────────────────────────────────────────────────────────
RHO_D_DEFAULT = 0.40   # 期望环绕半径（规格 0.5 在实机只剩 ~110° 弧，0.40 留裕量）
SUBJ_X_RANGE  = (0.60, 0.75)   # 比构图任务(0.65~1.15)更近：弧度上限 115~145°
SUBJ_Y_RANGE  = (-0.10, 0.10)
SUBJ_Z        = 0.47
PHI_D         = 0.20   # 默认俯仰 ≈11°：φ=0 时相机与桌面(0.45)几乎同高，易蹭桌

# 可达内域（与 emeet_arm_env 同口径：Cam0 臂展 0.674m，边缘需奇异位形不进）
WS_H_MIN, WS_H_MAX = 0.20, 0.60
WS_Z_MIN, WS_Z_MAX = 0.15, 0.85
ARC_MARGIN = math.radians(5)   # 可行弧两端安全余量

# ── 奖励权重（规格默认值）────────────────────────────────────────────────────
W_CENTER   = 10.0
W_PROGRESS = 5.0
W_DISTANCE = 2.0
W_SMOOTH   = 0.5
W_SAFE     = 1.0
R_COLLISION = -10.0
R_DONE      = 50.0
DONE_CENTER_TOL = 0.05     # 完成判据：中心误差（归一化）
SIGMA_MIN_THRESH = 0.05    # 奇异惩罚阈值：J 最小奇异值低于此值开始惩罚

# ── 终止参数 ─────────────────────────────────────────────────────────────────
MAX_EP_STEPS       = 500
OFFCENTER_TOL      = 0.5   # 偏心阈值（归一化）
OFFCENTER_STEPS    = 50    # 持续偏心步数 → 终止
LIMIT_PIN_STEPS    = 25    # 关节持续顶限位步数 → 终止
STAGE1_HOLD_STEPS  = 25    # 阶段1：居中保持步数 → 成功

# ── 课程默认 ─────────────────────────────────────────────────────────────────
ARC_MAX_INIT = math.radians(60)
ARC_MAX_FULL = math.radians(120)


def _wrap(a: float) -> float:
    """角度归一化到 [-π, π]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _theta_ref(ox: float, oy: float) -> float:
    """θ=0 的世界方位：主体指向底座的方向。"""
    return math.atan2(-oy, -ox)


def orbit_to_cart(theta, phi, r, ox, oy, oz):
    """轨道坐标 (θ,φ,r) → 相机世界坐标。"""
    tw = _theta_ref(ox, oy) + theta
    cp = math.cos(phi)
    return (ox + r * cp * math.cos(tw),
            oy + r * cp * math.sin(tw),
            oz + r * math.sin(phi))


def lookat_mat(cam_pos, tgt_pos):
    """roll=0 的 look-at 旋转矩阵（列 = 光学系各轴在世界系）。"""
    z = np.asarray(tgt_pos, float) - np.asarray(cam_pos, float)
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
    return np.column_stack([x, y, z])


class ArmOrbitEnv(gym.Env):
    """
    环绕运镜环境。

    参数
    ----
    xml_path      MuJoCo MJCF 路径（默认 sim/rl_env.xml，与构图任务共用场景）
    stage         课程阶段 1~4（可训练中经 set_stage 热切换）
    rho_d         期望环绕半径（m）
    goal_arc      固定弧度目标（rad）；None = 按阶段采样（评估时可固定口径）
    render_mode   'human' 开 MuJoCo viewer
    """

    metadata = {"render_modes": ["human"], "render_fps": CTRL_HZ}

    def __init__(self,
                 xml_path: str = MJCF_PATH,
                 stage: int = 3,
                 rho_d: float = RHO_D_DEFAULT,
                 goal_arc=None,
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
        self._jnt_lo = np.array([self.model.jnt_range[jid, 0] for jid in self._jnt_ids])
        self._jnt_hi = np.array([self.model.jnt_range[jid, 1] for jid in self._jnt_ids])
        self._q_home = 0.5 * (self._jnt_lo + self._jnt_hi)

        self._cam_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'Cam0')

        # 目标物体（mocap 体，位置由 env 直写）
        self._subj_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'obj_red_box')
        self._subj_mocap_id = self.model.body_mocapid[self._subj_body_id]

        # 碰撞判定 geom 集：臂上的 robot_col geom（contype==2），
        # 排除 base_link——底座 geom 与地面平面常态接触（安装面），不算碰撞。
        base_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self._col_geom_ids = {
            g for g in range(self.model.ngeom)
            if self.model.geom_contype[g] == 2
            and self.model.geom_bodyid[g] != base_bid}

        # 课程 / 任务参数
        self._stage    = int(stage)
        self._rho_d0   = float(rho_d)     # 名义值；stage4 每轮抖动
        self._rho_d    = float(rho_d)
        self._arc_max  = ARC_MAX_FULL if stage >= 3 else ARC_MAX_INIT
        self._fixed_goal_arc = goal_arc

        # 轮内状态
        self._q_cmd = self._q_home.copy()   # 速度积分的指令状态（见 step 注释）
        self._subj = np.array([0.9, 0.0, SUBJ_Z])
        self._dir  = 1.0
        self._goal_arc = 0.0
        self._theta_prev = 0.0
        self._progress   = 0.0
        self._prev_action = np.zeros(6, dtype=np.float32)
        self._step_count = 0
        self._offcenter_count = 0
        self._pin_count = 0
        self._center_hold = 0
        self._center_err_sum = 0.0
        self._sigma_min = 1.0
        self._subj_drift = np.zeros(2)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(30,),
                                            dtype=np.float32)
        self.render_mode = render_mode
        self._viewer = None

        # 雅可比缓冲
        self._jacp = np.zeros((3, self.model.nv))
        self._jacr = np.zeros((3, self.model.nv))

    # ── 课程接口（训练回调经 VecEnv.env_method 调用）─────────────────────────

    def set_stage(self, stage: int):
        self._stage = int(stage)
        if self._stage >= 3:
            self._arc_max = max(self._arc_max, ARC_MAX_INIT)

    def set_arc_max(self, arc_max: float):
        self._arc_max = float(np.clip(arc_max, ARC_MAX_INIT, ARC_MAX_FULL))

    # ── 几何工具 ─────────────────────────────────────────────────────────────

    def _feasible_arc(self, ox, oy, r, phi):
        """工作空间几何裁剪：返回包含 θ=0 的连续可行弧 [lo, hi]。

        判据与 IK 扫描高度吻合（水平距 ≤0.60 是主导约束），端点另做 IK 复核。
        """
        step = math.radians(2)

        def ok(theta):
            px, py, pz = orbit_to_cart(theta, phi, r, ox, oy, SUBJ_Z)
            return (WS_H_MIN <= math.hypot(px, py) <= WS_H_MAX
                    and WS_Z_MIN <= pz <= WS_Z_MAX)

        if not ok(0.0):
            return 0.0, 0.0
        hi = 0.0
        while hi + step <= math.pi and ok(hi + step):
            hi += step
        lo = 0.0
        while lo - step >= -math.pi and ok(lo - step):
            lo -= step
        return lo, hi

    def _ik_solve(self, target_pos, target_mat, max_iter=30):
        """DLS 迭代 IK（reset 摆位专用；训练步内不用 IK，用微分逆解）。

        迭代借用 data.qpos 做 FK，返回前必须整体还原——否则物理状态被
        IK 解覆盖，动力学等效瞬移（emeet_arm_env 的历史教训）。
        """
        q = np.array([self.data.qpos[a] for a in self._qpos_addrs], dtype=float)
        snapshot = self.data.qpos.copy()

        for _ in range(max_iter):
            for i, a in enumerate(self._qpos_addrs):
                self.data.qpos[a] = q[i]
            mujoco.mj_fwdPosition(self.model, self.data)

            ee_pos  = self.data.xpos[self._cam_body_id].copy()
            pos_err = target_pos - ee_pos
            ee_mat  = self.data.xmat[self._cam_body_id].reshape(3, 3)
            R_err   = target_mat @ ee_mat.T
            trace   = R_err[0, 0] + R_err[1, 1] + R_err[2, 2]
            angle   = math.acos(max(-1., min(1., (trace - 1) / 2)))
            if abs(angle) < 1e-6:
                ori_err = np.zeros(3)
            else:
                ori_err = (angle / (2 * math.sin(angle))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1]])
            if np.linalg.norm(pos_err) < 1e-3 and angle < 1e-2:
                break

            mujoco.mj_jac(self.model, self.data, self._jacp, self._jacr,
                          ee_pos, self._cam_body_id)
            J = np.vstack([self._jacp, self._jacr])[:, self._dof_addrs]
            err6 = np.concatenate([pos_err, ori_err * 0.5])
            dq = J.T @ np.linalg.solve(J @ J.T + 0.02**2 * np.eye(6), err6)
            q = np.clip(q + 0.6 * dq, self._jnt_lo, self._jnt_hi)

        self.data.qpos[:] = snapshot
        mujoco.mj_fwdPosition(self.model, self.data)
        return q

    def _verify_pose(self, theta, phi, r, ox, oy):
        """IK 到位复核：位置 <2cm 且光轴对准 <5°（与可达性扫描同判据）。"""
        px, py, pz = orbit_to_cart(theta, phi, r, ox, oy, SUBJ_Z)
        self._set_joints(self._q_home)
        q = self._ik_solve(np.array([px, py, pz]),
                           lookat_mat((px, py, pz), (ox, oy, SUBJ_Z)))
        self._set_joints(q)
        cam = self.data.xpos[self._cam_body_id]
        R = self.data.xmat[self._cam_body_id].reshape(3, 3)
        d = np.array([ox, oy, SUBJ_Z]) - cam
        d /= max(np.linalg.norm(d), 1e-9)
        ok = (np.linalg.norm(np.array([px, py, pz]) - cam) < 0.02
              and float(R[:, 2] @ d) > math.cos(math.radians(5.0)))
        return ok, q

    # ── Gym 接口 ─────────────────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        # stage4：期望半径每轮抖动（域随机化）
        self._rho_d = (self._rho_d0 + self.np_random.uniform(-0.05, 0.05)
                       if self._stage >= 4 else self._rho_d0)

        # 弧度目标采样（rad）
        if self._fixed_goal_arc is not None:
            want_arc = float(self._fixed_goal_arc)
        elif self._stage <= 1:
            want_arc = 0.0
        elif self._stage == 2:
            want_arc = self.np_random.uniform(math.radians(10), math.radians(30))
        else:
            want_arc = self.np_random.uniform(math.radians(30), self._arc_max)

        # 拒绝采样：主体位置 + 起点弧位，直到 几何可行 → IK 复核通过 →
        # 扰动落位并 PD 稳定后无碰撞。落位碰撞检查不能省——云台 Link4 悬在
        # Cam0 下方，φ 偏低的起点会贴着桌面，episode 从第 0 步就是坏样本。
        for _ in range(30):
            ox = self.np_random.uniform(*SUBJ_X_RANGE)
            oy = self.np_random.uniform(*SUBJ_Y_RANGE)
            lo, hi = self._feasible_arc(ox, oy, self._rho_d, PHI_D)
            span = (hi - lo) - 2 * ARC_MARGIN
            if span <= math.radians(5):
                continue
            self._goal_arc = min(want_arc, span)
            self._dir = 1.0 if self.np_random.uniform() < 0.5 else -1.0

            # 起点必须保证整段弧都在可行区间内
            if self._dir > 0:
                s_lo, s_hi = lo + ARC_MARGIN, hi - ARC_MARGIN - self._goal_arc
            else:
                s_lo, s_hi = lo + ARC_MARGIN + self._goal_arc, hi - ARC_MARGIN
            theta_s = self.np_random.uniform(s_lo, s_hi)
            theta_e = theta_s + self._dir * self._goal_arc

            self._set_subject(ox, oy, SUBJ_Z)
            ok_s, q_s = self._verify_pose(theta_s, PHI_D, self._rho_d, ox, oy)
            ok_e, _   = self._verify_pose(theta_e, PHI_D, self._rho_d, ox, oy)
            if not (ok_s and ok_e):
                continue

            # 初始视轴扰动：阶段1 大偏移（学对准），其余小偏移（起点不完美）
            pan_amp = 0.20 if self._stage == 1 else 0.05
            q0 = q_s.copy()
            q0[3] += self.np_random.uniform(-pan_amp, pan_amp)   # Joint4 = pan
            q0[5] += self.np_random.uniform(-pan_amp, pan_amp)   # Joint6 = tilt
            if self._stage >= 4:                                  # 全关节位姿噪声
                q0 += self.np_random.normal(0, 0.02, 6)
            q0 = np.clip(q0, self._jnt_lo, self._jnt_hi)
            self._set_joints(q0)

            # PD 稳定几步后检查落位质量：无碰撞且目标在画面内
            for _ in range(20):
                self._apply_ctrl(q0)
                mujoco.mj_step(self.model, self.data)
            _, u0, v0, front0 = self._project()
            if not self._in_collision() and front0 \
                    and abs(u0) < 0.9 and abs(v0) < 0.9:
                break
        # 30 次全失败极罕见（几何预筛已排除主因）；用最后一次落位兜底继续

        self._subj = np.array([ox, oy, SUBJ_Z])
        self._q_cmd = np.array([self.data.qpos[a] for a in self._qpos_addrs])

        # 阶段4：轮内目标微移剧本（≤1cm/s 慢漂移）
        if self._stage >= 4:
            ang = self.np_random.uniform(-math.pi, math.pi)
            spd = self.np_random.uniform(0, 0.01) * DT_ACTION
            self._subj_drift = spd * np.array([math.cos(ang), math.sin(ang)])
        else:
            self._subj_drift = np.zeros(2)

        self._theta_prev = self._orbit_state()[0]
        self._progress = 0.0
        self._prev_action = np.zeros(6, dtype=np.float32)
        self._step_count = 0
        self._offcenter_count = 0
        self._pin_count = 0
        self._center_hold = 0
        self._center_err_sum = 0.0
        self._sigma_min = 1.0

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # ── 笛卡尔速度 → 关节速度（DLS 微分逆解，相机系→世界系）────────────
        v_cam = action[:3] * V_MAX
        w_cam = action[3:] * W_MAX
        R = self.data.xmat[self._cam_body_id].reshape(3, 3)
        twist = np.concatenate([R @ v_cam, R @ w_cam])

        cam_pos = self.data.xpos[self._cam_body_id].copy()
        mujoco.mj_jac(self.model, self.data, self._jacp, self._jacr,
                      cam_pos, self._cam_body_id)
        J = np.vstack([self._jacp, self._jacr])[:, self._dof_addrs]
        self._sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
        qdot = J.T @ np.linalg.solve(J @ J.T + DLS_LAMBDA**2 * np.eye(6), twist)

        # 逐关节速度限幅（等比缩放保方向，部署安全层同款逻辑）
        scale = float(np.max(np.abs(qdot) / MAX_VEL))
        if scale > 1.0:
            qdot = qdot / scale

        # 积分基准 = 持久指令状态，不是实测位置：以实测为基准时 PD 稳态误差
        # 会被逐步"确认"，零动作下臂每步下垂一点，70 步后云台蹭到桌面
        # （2026-08-20 冒烟实测）。指令积分与实机 JTC"保持最后指令"行为一致。
        self._q_cmd = np.clip(self._q_cmd + qdot * DT_ACTION,
                              self._jnt_lo, self._jnt_hi)

        # ── 物理仿真 ─────────────────────────────────────────────────────────
        for _ in range(SIM_STEPS_PER_ACTION):
            self._apply_ctrl(self._q_cmd)
            mujoco.mj_step(self.model, self.data)

        # 阶段4：目标微移
        if np.any(self._subj_drift != 0):
            ox = float(np.clip(self._subj[0] + self._subj_drift[0], *SUBJ_X_RANGE))
            oy = float(np.clip(self._subj[1] + self._subj_drift[1], *SUBJ_Y_RANGE))
            self._subj = np.array([ox, oy, SUBJ_Z])
            self._set_subject(ox, oy, SUBJ_Z)

        # ── 状态量 ───────────────────────────────────────────────────────────
        theta, phi, rho = self._orbit_state()
        d_theta = _wrap(theta - self._theta_prev)
        self._theta_prev = theta
        self._progress += self._dir * d_theta      # 方向符号化累计进度

        p_cam, u_n, v_n, in_front = self._project()
        center_err = math.hypot(u_n, v_n)
        self._center_err_sum += center_err

        # ── 奖励 ─────────────────────────────────────────────────────────────
        reward = self._compute_reward(action, u_n, v_n, rho, d_theta)

        # ── 终止判定 ─────────────────────────────────────────────────────────
        terminated, success, reason = False, False, ''

        # 失败：目标出视野（相机后方或出图像边界）
        if not in_front or abs(u_n) > 1.0 or abs(v_n) > 1.0:
            reward += R_COLLISION
            terminated, reason = True, 'out_of_view'

        # 失败：碰撞
        if not terminated and self._in_collision():
            reward += R_COLLISION
            terminated, reason = True, 'collision'

        # 失败：持续偏心（规格：偏离 >0.5 持续 50 步）
        self._offcenter_count = self._offcenter_count + 1 \
            if center_err > OFFCENTER_TOL else 0
        if not terminated and self._offcenter_count >= OFFCENTER_STEPS:
            reward += R_COLLISION
            terminated, reason = True, 'offcenter'

        # 失败：关节持续顶限位（指令积分被 clip 钉在边界 = 策略持续要求越界；
        # 看 _q_cmd 而非实测——PD 稳态误差让实测到不了精确边界）
        pinned = np.any((self._q_cmd - self._jnt_lo < 1e-6)
                        | (self._jnt_hi - self._q_cmd < 1e-6))
        self._pin_count = self._pin_count + 1 if pinned else 0
        if not terminated and self._pin_count >= LIMIT_PIN_STEPS:
            reward += R_COLLISION
            terminated, reason = True, 'joint_limit'

        # 成功
        if not terminated:
            if self._stage <= 1:
                self._center_hold = self._center_hold + 1 \
                    if center_err < DONE_CENTER_TOL else 0
                if self._center_hold >= STAGE1_HOLD_STEPS:
                    reward += R_DONE
                    terminated, success, reason = True, True, 'success'
            elif (self._progress >= self._goal_arc
                  and center_err < DONE_CENTER_TOL):
                reward += R_DONE
                terminated, success, reason = True, True, 'success'

        self._prev_action = action.copy()
        self._step_count += 1
        truncated = (not terminated) and self._step_count >= MAX_EP_STEPS

        info = {
            'success':      float(success),
            'progress_deg': math.degrees(max(self._progress, 0.0)),
            'goal_deg':     math.degrees(self._goal_arc),
            'center_err':   self._center_err_sum / self._step_count,
            'reason':       reason,
        }

        if self.render_mode == 'human':
            self.render()
        return self._get_obs(), float(reward), terminated, truncated, info

    def render(self):
        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        if self._viewer.is_running():
            self._viewer.sync()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    # ── 奖励 ─────────────────────────────────────────────────────────────────

    def _compute_reward(self, action, u_n, v_n, rho, d_theta) -> float:
        # 1. 居中（二次惩罚，截断防出画面时爆炸）
        r_center = -W_CENTER * min(u_n**2 + v_n**2, 4.0)

        # 2. 环绕进度（带方向的角度增量；倒退即负分）
        r_progress = W_PROGRESS * self._dir * d_theta

        # 3. 距离保持
        r_distance = -W_DISTANCE * abs(rho - self._rho_d)

        # 4. 动作平滑（变化率；能量项由速度限幅+进度权衡隐式约束）
        r_smooth = -W_SMOOTH * float(np.sum((action - self._prev_action)**2))

        # 5. 安全：限位余量 + 奇异接近 + 水平保持（roll≈0 口径）
        q = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        margin = 0.1
        lim = (np.sum(np.maximum(0, margin - (q - self._jnt_lo))**2)
               + np.sum(np.maximum(0, margin - (self._jnt_hi - q))**2)) / margin**2
        sing = max(0.0, (SIGMA_MIN_THRESH - self._sigma_min) / SIGMA_MIN_THRESH)
        R = self.data.xmat[self._cam_body_id].reshape(3, 3)
        level = R[2, 0] ** 2          # 相机 X 轴的世界 Z 分量：0 = 图像水平
        r_safe = -W_SAFE * (lim + sing + level)

        # 课程门控：阶段1 只学对准与平滑；阶段2 加进度与距离；阶段3+ 全量
        if self._stage <= 1:
            return r_center + r_smooth
        if self._stage == 2:
            return r_center + r_smooth + r_progress + r_distance
        return r_center + r_smooth + r_progress + r_distance + r_safe

    # ── 感知 ─────────────────────────────────────────────────────────────────

    def _project(self):
        """目标 → 相机系位置 + 归一化像素坐标（内参 K 针孔投影）。

        返回 (p_cam, u_n, v_n, in_front)。u_n/v_n ∈ [-1,1] 对应图像边界；
        阶段4 叠加观测噪声（位置 5mm / 像素 1% 高斯）。
        """
        cam_pos  = self.data.xpos[self._cam_body_id]
        cam_xmat = self.data.xmat[self._cam_body_id].reshape(3, 3)
        p_cam = cam_xmat.T @ (self._subj - cam_pos)

        if self._stage >= 4:
            p_cam = p_cam + self.np_random.normal(0, 0.005, 3)

        if p_cam[2] < 0.01:
            return p_cam, -2.0, -2.0, False

        u_pix = FX * p_cam[0] / p_cam[2] + CX
        v_pix = FY * p_cam[1] / p_cam[2] + CY
        u_n = (u_pix - CX) / CX
        v_n = (v_pix - CY) / CY
        if self._stage >= 4:
            u_n += self.np_random.normal(0, 0.01)
            v_n += self.np_random.normal(0, 0.01)
        u_n = float(np.clip(u_n, -1.5, 1.5))
        v_n = float(np.clip(v_n, -1.5, 1.5))
        return p_cam, u_n, v_n, True

    def _orbit_state(self):
        """相机实际位置 → 轨道坐标 (θ, φ, ρ)。"""
        cam = self.data.xpos[self._cam_body_id]
        d = cam - self._subj
        rho = float(np.linalg.norm(d))
        phi = math.asin(float(np.clip(d[2] / max(rho, 1e-9), -1., 1.)))
        theta = _wrap(math.atan2(d[1], d[0]) - _theta_ref(self._subj[0],
                                                          self._subj[1]))
        return theta, phi, rho

    def _get_obs(self) -> np.ndarray:
        p_cam, u_n, v_n, _ = self._project()
        theta, phi, rho = self._orbit_state()
        q    = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        qvel = np.array([self.data.qvel[a] for a in self._dof_addrs])
        qpos_norm = 2 * (q - self._jnt_lo) / (self._jnt_hi - self._jnt_lo) - 1
        qvel_norm = np.clip(qvel / MAX_VEL, -1., 1.)

        return np.array([
            *(np.clip(p_cam / 0.8, -2., 2.)),           # 0:3
            u_n, v_n,                                    # 3:5
            (rho - self._rho_d) / 0.3,                   # 5
            math.sin(theta), math.cos(theta),            # 6:8
            phi / (math.pi / 2),                         # 8
            self._progress / math.pi,                    # 9
            (self._goal_arc - self._progress) / math.pi, # 10
            self._dir,                                   # 11
            *self._prev_action,                          # 12:18
            *qpos_norm,                                  # 18:24
            *qvel_norm,                                  # 24:30
        ], dtype=np.float32)

    # ── 碰撞 / 底层操作 ──────────────────────────────────────────────────────

    def _in_collision(self) -> bool:
        """臂体(除底座)与世界的任何有效接触即判碰撞。"""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.dist < 1e-4 and (c.geom1 in self._col_geom_ids
                                  or c.geom2 in self._col_geom_ids):
                return True
        return False

    def _set_subject(self, x, y, z):
        self.data.mocap_pos[self._subj_mocap_id] = (x, y, z)

    def _set_joints(self, q: np.ndarray):
        for i, a in enumerate(self._qpos_addrs):
            self.data.qpos[a] = q[i]
            self.data.qvel[self._dof_addrs[i]] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _apply_ctrl(self, q_target: np.ndarray):
        """位置执行器指令（含速度前馈），与 emeet_arm_env 同款。"""
        cur_q = np.array([self.data.qpos[a] for a in self._qpos_addrs])
        for i, aid in enumerate(self._act_ids):
            vel_ff = (q_target[i] - cur_q[i]) / DT_ACTION
            self.data.ctrl[aid] = q_target[i] + _KV_OVER_KP[i] * vel_ff
