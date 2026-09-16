# -*- coding: utf-8 -*-
"""
@file      shot_compiler.py
@brief     规范 JSON（shot_spec.ShotSpec，A 型）→ Arm Commander 可执行原语（LINEAR / ORBIT / PTP goal）的编译器。
           纯 numpy（可达性判定复用 reach_check 的 ArmModel），不依赖 ROS；执行在 shot_executor。
@version   0.1
@date      2026-09-16
@copyright Copyright (c) 2026 eMeet

它要做的四件事（对照规范 4.1 / 5.3 / 8.4）：
  ① 坐标系：世界（odom）→ 机械臂基座系（WorldFrame）；受控体从相机光心换到 Commander 的法兰 gimbal_tool0
     （CameraFrames，固定变换取自 URDF：光轴 +Z = 法兰 +X，偏移约 4.4 cm）。
  ② 几何：每一段先试"一条原语能不能在容差内复现参考轨迹"——直线段 / 纯摇镜 → LINEAR（位置线性 + 姿态 slerp）；
     look_at 恒定的圆弧 → ORBIT（球面环绕，相机对准球心）；都不行就按弧长细分成多条 LINEAR 弦，
     直到与参考位姿的偏差落进 tolerance × tolerance_budget。偏差按**法兰**插值后再换回光心来算，
     所以 4.4 cm 的偏置带来的误差也被计入。
  ③ 时间：Commander 只有 slow / normal / fast 三档（arm_params.yaml speed_profiles），把 duration / speed 折成
     最接近的档位；比 fast 还快 → degradation="slowed"；比 slow 还慢 → 告警（会比要求的快）。
     law 只有 s_curve / ease_in_out 是 Ruckig 原生形态，其余告警；末段 law 末端带速度按 4.1 标 slowed。
  ④ 可达性：每个 waypoint 过 check_pose（规则 25），每条原语沿其自身法兰路径连续采样过 IK（规则 26）；
     不可达返回结构化 SpecError，不生成 goal。

不做的事：B 型闭环（先用 shot_spec.tracking_to_static 按目标快照展开）、av（只透传）、避障。

═══════════════════════════════ 本文件函数 / 类汇总 ═══════════════════════════════
  SpeedLimit / SPEED_PROFILES                 三档笛卡尔速度上限（与 arm_params.yaml 一致）
  DEFAULT_TOOL0_TO_OPTICAL                    gimbal_tool0 → camera_optical_frame（robot_gimbal_description_v2）
  WorldFrame                                  odom ↔ 机械臂基座系（identity / from_chassis / from_matrix）
  CameraFrames                                光心位姿 ↔ 法兰 ArmPose（from_urdf / flange_pose / optical_from_flange）
  Primitive                                   一条 Commander goal（linear / orbit / ptp）+ 它覆盖的段与进度区间
  primitive_progress(prim, flange_pose)       当前法兰位姿在原语几何上的投影进度（执行期反馈用）
  CompiledShot                                编译结果：原语序列 / 各点 hold / 告警 / 降级 / 名义时长 / plan_steps()
  ShotCompiler.compile(spec, start_joints)    主入口
  ShotCompiler.pose_error(ref, flange_pose)   实测法兰位姿相对参考位姿的 (位置, 指向, roll) 误差（执行期反馈用）
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import shot_spec as ss
from .reach_check import (ArmModel, Margins, cart_to_sphere, check_pose, matrix_to_pose,
                          pose_from_look_at, pose_to_matrix, sphere_to_cart)

# ─────────────────────────────── 常量 ───────────────────────────────


@dataclass(frozen=True)
class SpeedLimit:
    """@brief 一个档位的笛卡尔速度上限：末端线速度（m/s）与角速度（rad/s）。"""
    v_pos: float
    v_ori: float


#: 与 robot_arm_bringup/config/arm_params.yaml 的 speed_profiles 一致（只取 v_pos / v_ori）
SPEED_PROFILES: Dict[str, SpeedLimit] = {
    'slow': SpeedLimit(0.02, 0.05),
    'normal': SpeedLimit(0.05, 0.10),
    'fast': SpeedLimit(0.10, 0.20),
}

#: robot_gimbal_description_v2 gimbal.urdf.xacro：Cam0_joint origin xyz="0.0362 0 -0.0244029322052897"
#: rpy="-π/2 0 -π/2"，Cam0 即光学系（camera_optical_frame 是它的零偏移别名）。列 = 光学 X/Y/Z 在法兰系的方向。
DEFAULT_TOOL0_TO_OPTICAL = np.array([
    [0.0, 0.0, 1.0, 0.0362],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, -0.0244029322052897],
    [0.0, 0.0, 0.0, 1.0],
])

#: law 里 Ruckig 从静止到静止的 S 曲线能直接复现的两种
_NATIVE_LAWS = ('s_curve', 'ease_in_out')
#: 末端带速度的 law（规范 4.1：执行层减速停住并标 slowed）
_END_VELOCITY_LAWS = ('constant', 'ease_in')
#: Commander progress_percent 里运镜段**之前**（搬到起点 + 起点停顿 [+ ORBIT 的规划期]）占的份额
#: （trajectory_shot_server：LINEAR 0→50 PTP，50→100 运镜；ORBIT 0→20 PTP，20→60 规划，60→100 运镜；均不含返回）。
#: 只用来判断"还没开始动"；运镜段内的 progress_percent 是按 0.1×timeout 归一化的**时间**，不是路径进度，
#: 路径进度用 primitive_progress() 从 current_pose 在原语几何上投影得到。
PROGRESS_OFFSET = {'linear': 50.0, 'orbit': 60.0, 'ptp': 0.0}
#: primitive_progress：位置噪声 σ_p 与姿态噪声 σ_r 的比值（m / rad ≈ 1），决定 LINEAR 进度用位置投影还是转角占比
_PROGRESS_RAD_PER_M = 1.0


# ─────────────────────────────── 旋转小工具 ───────────────────────────────

def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """@brief R = Rz(yaw)·Ry(pitch)·Rx(roll)（rad），与 motion::rpy_to_quat 一致。"""
    return pose_to_matrix({'x': 0, 'y': 0, 'z': 0, 'roll': math.degrees(roll),
                           'pitch': math.degrees(pitch), 'yaw': math.degrees(yaw)})[:3, :3]


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    """@brief 两向量夹角。"""
    return ss._angle_between(a, b)


# ─────────────────────────────── 坐标系 ───────────────────────────────

class WorldFrame:
    """@brief 世界系（odom）与机械臂基座系（Commander 的 base_link = arm_base_link）之间的固定变换。
           规范的所有坐标在世界系里；Commander goal 在基座系里。"""

    def __init__(self, tf: Optional[np.ndarray] = None):
        """@brief 构造。
        @param tf 4×4：机械臂基座系在世界系里的位姿；None = 单位阵（世界系就是基座系）
        """
        self.tf = np.eye(4) if tf is None else np.asarray(tf, dtype=float)
        self._inv = np.linalg.inv(self.tf)

    @classmethod
    def identity(cls) -> 'WorldFrame':
        """@brief 世界系 = 基座系（单臂台架、离线测试）。
        @return WorldFrame
        """
        return cls(None)

    @classmethod
    def from_matrix(cls, tf: np.ndarray) -> 'WorldFrame':
        """@brief 直接给 4×4（例如 tf2 查出来的 odom → arm_base_link）。
        @param tf 4×4
        @return WorldFrame
        """
        return cls(tf)

    @classmethod
    def from_chassis(cls, x: float, y: float, yaw: float, arm_base_height: float = 0.31) -> 'WorldFrame':
        """@brief 由底盘位姿构造：odom →（x, y, ψ）→ base_link →（固定，离地 arm_base_height）→ arm_base_link（规范 2.1）。
        @param x,y             底盘在 odom 系的位置
        @param yaw             底盘朝向 ψ，rad
        @param arm_base_height 臂基座离地高，m（规范给 0.31）
        @return WorldFrame
        """
        tf = np.eye(4)
        tf[:3, :3] = ss._rodrigues(np.array([0.0, 0.0, 1.0]), yaw)
        tf[:3, 3] = [x, y, arm_base_height]
        return cls(tf)

    @property
    def rotation(self) -> np.ndarray:
        """@brief 基座系轴在世界系里的方向（3×3）。
        @return 3×3
        """
        return self.tf[:3, :3]

    def to_arm_point(self, p_world: Sequence[float]) -> np.ndarray:
        """@brief 世界点 → 基座系点。"""
        return (self._inv @ np.append(np.asarray(p_world, dtype=float), 1.0))[:3]

    def to_world_point(self, p_arm: Sequence[float]) -> np.ndarray:
        """@brief 基座系点 → 世界点。"""
        return (self.tf @ np.append(np.asarray(p_arm, dtype=float), 1.0))[:3]

    def to_arm_rotation(self, rot_world: np.ndarray) -> np.ndarray:
        """@brief 世界系下的旋转 → 基座系下的旋转。"""
        return self.rotation.T @ np.asarray(rot_world)

    def to_world_rotation(self, rot_arm: np.ndarray) -> np.ndarray:
        """@brief 基座系下的旋转 → 世界系下的旋转。"""
        return self.rotation @ np.asarray(rot_arm)


class CameraFrames:
    """@brief 受控体换算：规范管相机光心（camera_optical_frame），Commander 管法兰（gimbal_tool0）。"""

    def __init__(self, tool0_to_optical: Optional[np.ndarray] = None):
        """@brief 构造。
        @param tool0_to_optical 4×4，法兰系 → 光学系；None 用 URDF 常量
        """
        self.tool0_to_optical = (DEFAULT_TOOL0_TO_OPTICAL.copy() if tool0_to_optical is None
                                 else np.asarray(tool0_to_optical, dtype=float))
        self._optical_to_tool0 = np.linalg.inv(self.tool0_to_optical)

    @classmethod
    def from_urdf(cls, xml: str, tool_link: str = 'gimbal_tool0',
                  optical_link: str = 'camera_optical_frame') -> 'CameraFrames':
        """@brief 从完整 URDF 现算法兰 → 光学系的固定变换（两者之间只有 fixed 关节，任何关节角结果相同）。
        @param xml          URDF 字符串（/robot_description）
        @param tool_link    法兰 link
        @param optical_link 光学系 link
        @return CameraFrames
        """
        model = ArmModel.from_urdf_string(xml, tip_link=optical_link)
        q = np.zeros(len(model.joint_names))
        rel = np.linalg.inv(model.fk(q, tool_link)) @ model.fk(q, optical_link)
        return cls(rel)

    @property
    def offset_m(self) -> float:
        """@brief 法兰到光心的距离。
        @return m
        """
        return float(np.linalg.norm(self.tool0_to_optical[:3, 3]))

    def flange_matrix(self, rot_cam: np.ndarray, p_cam: Sequence[float]) -> np.ndarray:
        """@brief 光心位姿（基座系）→ 法兰 4×4。"""
        t_cam = np.eye(4)
        t_cam[:3, :3] = rot_cam
        t_cam[:3, 3] = np.asarray(p_cam, dtype=float)
        return t_cam @ self._optical_to_tool0

    def flange_pose(self, rot_cam: np.ndarray, p_cam: Sequence[float]) -> Dict[str, float]:
        """@brief 光心位姿（基座系）→ 法兰 ArmPose 字典（米 / 度，Commander 语义）。
        @param rot_cam 光学系 3×3（列 = 右 / 下 / 前）
        @param p_cam   光心位置
        @return {x, y, z, roll, pitch, yaw}
        """
        return self.pose_from_flange_matrix(self.flange_matrix(rot_cam, p_cam))

    @staticmethod
    def pose_from_flange_matrix(t_tool: np.ndarray) -> Dict[str, float]:
        """@brief 法兰 4×4 → ArmPose 字典。pitch=±90° 的万向锁下 matrix_to_pose 会丢掉 roll，这里检测并改用
               yaw=0 重解 roll，保证 pose_to_matrix 能还原出同一个旋转。
        @param t_tool 法兰 4×4（基座系）
        @return {x, y, z, roll, pitch, yaw}
        """
        pose = matrix_to_pose(t_tool)
        back = pose_to_matrix(pose)[:3, :3]
        if np.max(np.abs(back - t_tool[:3, :3])) > 1e-6:
            rot = t_tool[:3, :3]
            pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
            rx = _rpy_matrix(0.0, pitch, 0.0).T @ rot        # 应为纯 Rx(roll)
            roll = math.atan2(rx[2, 1], rx[1, 1])
            pose.update({'roll': math.degrees(roll), 'pitch': math.degrees(pitch), 'yaw': 0.0})
        return pose

    def optical_from_flange(self, pose: Any) -> Tuple[np.ndarray, np.ndarray]:
        """@brief 法兰 ArmPose（字典 / msg）→ 光心（旋转 3×3, 位置），基座系。
        @param pose ArmPose 语义
        @return (rot_cam, p_cam)
        """
        t_cam = pose_to_matrix(pose) @ self.tool0_to_optical
        return t_cam[:3, :3], t_cam[:3, 3]


# ─────────────────────────────── 输出结构 ───────────────────────────────

@dataclass
class Primitive:
    """@brief 一条 Commander goal。kind：linear（ArmTrajectoryShot MOTION_LINEAR）/ orbit（MOTION_ORBIT）/
           ptp（ArmMoveToPose）。segment_index 是它复现的规范段号（起始 PTP 为 −1），[s0, s1] 是覆盖的进度区间。
           位姿全是法兰 ArmPose 字典（基座系，米 / 度）；orbit 的角度为度、半径为米（与 action 一致）。"""
    kind: str
    segment_index: int
    s0: float
    s1: float
    speed: str
    nominal_sec: float = 0.0
    start: Optional[Dict[str, float]] = None
    end: Optional[Dict[str, float]] = None
    pose: Optional[Dict[str, float]] = None
    center: Optional[np.ndarray] = None
    az0: float = 0.0
    el0: float = 0.0
    r0: float = 0.0
    az1: float = 0.0
    el1: float = 0.0
    r1: float = 0.0

    @property
    def progress_offset(self) -> float:
        """@brief Commander 反馈 progress_percent 里属于"搬到起点"的份额（%），之后才是本原语的运镜进度。
        @return 百分数
        """
        return PROGRESS_OFFSET[self.kind]

    def to_plan_step(self) -> Dict[str, Any]:
        """@brief 转成 run_plan_step / execute_plan 认识的一条步骤。
        @return 步骤字典
        """
        if self.kind == 'ptp':
            step = {'op': 'pose'}
            step.update({k: float(v) for k, v in self.pose.items()})
            step['speed'] = self.speed
            return step
        if self.kind == 'linear':
            return {'op': 'linear', 'start': {k: float(v) for k, v in self.start.items()},
                    'end': {k: float(v) for k, v in self.end.items()}, 'speed': self.speed}
        return {'op': 'orbit', 'center': [float(v) for v in self.center],
                'az_start_deg': float(self.az0), 'az_end_deg': float(self.az1),
                'el_start_deg': float(self.el0), 'el_end_deg': float(self.el1),
                'r_start_m': float(self.r0), 'r_end_m': float(self.r1), 'speed': self.speed}


def primitive_progress(prim: Primitive, flange_pose: Any) -> float:
    """@brief 由当前法兰位姿在原语几何上投影得到该原语的局部进度 u∈[0,1]（不用 Commander 的 progress_percent，
           那是按时间归一化的）。LINEAR：转角（rad）大于弦长（m）时取转角占比（纯摇镜 / 细分弦），否则位置投到弦上；
           ORBIT：取方位角占比，方位不变时取仰角 / 半径占比；PTP：无路径，恒 0。
    @param prim        原语
    @param flange_pose 当前法兰 ArmPose（msg / 字典）
    @return u
    """
    if prim.kind == 'ptp':
        return 0.0
    t_now = pose_to_matrix(flange_pose)
    if prim.kind == 'linear':
        t0, t1 = pose_to_matrix(prim.start), pose_to_matrix(prim.end)
        chord = t1[:3, 3] - t0[:3, 3]
        length2 = float(np.dot(chord, chord))
        total_ang = float(np.linalg.norm(ss.log_so3(t0[:3, :3].T @ t1[:3, :3])))
        chord_len = math.sqrt(length2)
        # 位置投影的噪声敏感度 ~ σ_p / 弦长，转角占比的 ~ σ_r / 总转角；按 σ_p≈2 mm、σ_r≈2 mrad（1 m/rad）取更稳的一个：
        # 转角（rad）大于弦长（m）就用转角占比（slerp 的转角随进度线性增长）——纯摇镜的法兰弦只有几厘米，靠这条
        if chord_len < 1e-9 and total_ang < 1e-9:
            return 1.0
        if total_ang > chord_len * _PROGRESS_RAD_PER_M:
            u = float(np.linalg.norm(ss.log_so3(t0[:3, :3].T @ t_now[:3, :3]))) / total_ang
        else:
            u = float(np.dot(t_now[:3, 3] - t0[:3, 3], chord)) / length2
        return min(1.0, max(0.0, u))
    th, ph, r = cart_to_sphere(t_now[:3, 3], prim.center)
    th0, th1 = math.radians(prim.az0), math.radians(prim.az1)
    d_th = th1 - th0
    if abs(d_th) > 1e-6:
        best = None
        for k in (-1, 0, 1):
            u = (th + 2 * math.pi * k - th0) / d_th
            score = 0.0 if 0.0 <= u <= 1.0 else min(abs(u), abs(u - 1.0))
            if best is None or score < best[0]:
                best = (score, u)
        u = best[1]
    elif abs(prim.el1 - prim.el0) > 1e-6:
        u = (math.degrees(ph) - prim.el0) / (prim.el1 - prim.el0)
    elif abs(prim.r1 - prim.r0) > 1e-6:
        u = (r - prim.r0) / (prim.r1 - prim.r0)
    else:
        u = 1.0
    return min(1.0, max(0.0, float(u)))


@dataclass
class CompiledShot:
    """@brief 编译结果。primitives 按执行顺序；holds[i] 是 waypoints[i] 的停留秒数。"""
    spec: ss.ShotSpec
    primitives: List[Primitive]
    holds: List[float]
    warnings: List[str] = field(default_factory=list)
    degradation: str = 'none'
    nominal_total_sec: float = 0.0
    dwell_sec: float = 1.0

    def primitives_of_segment(self, i: int) -> List[Primitive]:
        """@brief 第 i 段对应的原语（按顺序）。
        @param i 段号
        @return list
        """
        return [p for p in self.primitives if p.segment_index == i]

    def plan_steps(self) -> List[Dict[str, Any]]:
        """@brief 转成 JSON 步骤表（op 列表）：原语 + 各点 hold 的 wait，可直接喂 execute_plan。
        @return 步骤列表
        """
        steps: List[Dict[str, Any]] = []
        n_seg = len(self.spec.segments)

        def _wait(hold: float, shorten: bool) -> None:
            """@brief 与 ShotExecutor 同口径：后面还有 goal 的 hold 扣掉 Commander 自带的起点停顿。"""
            seconds = hold - (self.dwell_sec if shorten else 0.0)
            if hold > 0 and seconds > 1e-9:
                steps.append({'op': 'wait', 'seconds': float(seconds)})

        for p in self.primitives_of_segment(-1):
            steps.append(p.to_plan_step())
        if self.holds:
            _wait(self.holds[0], shorten=n_seg > 0)
        for i in range(n_seg):
            for p in self.primitives_of_segment(i):
                steps.append(p.to_plan_step())
            _wait(self.holds[i + 1], shorten=i < n_seg - 1)
        return steps

    def summary(self) -> str:
        """@brief 一段人话汇总（日志用）。
        @return 文本
        """
        kinds = ' → '.join(f'{p.kind}[{p.segment_index}]' for p in self.primitives)
        lines = [f'{len(self.spec.waypoints)} 点 {len(self.spec.segments)} 段 → {len(self.primitives)} 条原语：{kinds}',
                 f'名义时长 {self.nominal_total_sec:.1f} s，降级 {self.degradation}']
        lines += [f'⚠ {w}' for w in self.warnings]
        return '\n'.join(lines)


# ─────────────────────────────── 编译器 ───────────────────────────────

class ShotCompiler:
    """@brief 规范 A 型 ShotSpec → CompiledShot。"""

    def __init__(self, model: ArmModel, world: Optional[WorldFrame] = None,
                 frames: Optional[CameraFrames] = None,
                 speed_profiles: Optional[Dict[str, SpeedLimit]] = None,
                 margins: Optional[Margins] = None, tolerance_budget: float = 0.5,
                 max_chords: int = 8, dwell_sec: float = 1.0, reach_step_m: float = 0.02,
                 reach_step_rad: float = 0.05):
        """@brief 构造。
        @param model            机械臂模型（末端 gimbal_tool0），用于可达性判定
        @param world            世界系 → 基座系；None = 单位阵
        @param frames           法兰 ↔ 光心；None 用 URDF 常量
        @param speed_profiles   三档速度上限；None 用 SPEED_PROFILES
        @param margins          可达判定余量；None 用 reach_check 默认
        @param tolerance_budget 编译期允许吃掉多少容差（0~1），剩下的留给执行误差
        @param max_chords       一段最多细分成多少条 LINEAR 弦
        @param dwell_sec        Commander 每条运镜 goal 起点的停顿（arm_params posture.dwell_at_start_sec）
        @param reach_step_m     沿路径采样 IK 的位置步长
        @param reach_step_rad   沿路径采样 IK 的角度步长
        """
        self.model = model
        self.world = world or WorldFrame.identity()
        self.frames = frames or CameraFrames()
        self.speed_profiles = speed_profiles or SPEED_PROFILES
        self.margins = margins or Margins()
        self.tolerance_budget = tolerance_budget
        self.max_chords = max_chords
        self.dwell_sec = dwell_sec
        self.reach_step_m = reach_step_m
        self.reach_step_rad = reach_step_rad

    # ── 参考位姿（基座系、法兰） ──
    def _ref_optical_arm(self, spec: ss.ShotSpec, i: int, s: float) -> Tuple[np.ndarray, np.ndarray]:
        """@brief 第 i 段进度 s 的参考光心位姿，换到基座系。
        @return (rot_cam, p_cam)
        """
        ref = ss.reference_pose(spec, i, s)
        return self.world.to_arm_rotation(ref.rotation), self.world.to_arm_point(ref.position)

    def _waypoint_flange(self, wp: ss.Waypoint) -> Dict[str, float]:
        """@brief waypoint → 法兰 ArmPose（基座系）。"""
        rot = ss.camera_rotation(wp.position, wp.look_at, wp.roll)
        return self.frames.flange_pose(self.world.to_arm_rotation(rot), self.world.to_arm_point(wp.position))

    def _ref_flange(self, spec: ss.ShotSpec, i: int, s: float) -> Dict[str, float]:
        """@brief 第 i 段进度 s 的参考法兰 ArmPose（基座系）。"""
        rot, pos = self._ref_optical_arm(spec, i, s)
        return self.frames.flange_pose(rot, pos)

    # ── 误差 ──
    def pose_error(self, ref: ss.CamRef, flange_pose: Any) -> Tuple[float, float, float]:
        """@brief 实测（或候选）法兰位姿相对参考位姿的误差：光心位置偏差（m）、光轴指向夹角（rad）、
               绕光轴的 roll 偏差（rad）。参考在世界系，法兰位姿在基座系。
        @param ref         参考位姿（shot_spec.reference_pose）
        @param flange_pose ArmPose 字典 / msg
        @return (position_err, aim_err, roll_err)
        """
        rot_a, p_a = self.frames.optical_from_flange(flange_pose)
        rot_w = self.world.to_world_rotation(rot_a)
        p_w = self.world.to_world_point(p_a)
        pos_err = float(np.linalg.norm(p_w - ref.position))
        aim_err = _angle(rot_w[:, 2], ref.rotation[:, 2])
        roll_err = abs(float(ss.log_so3(ref.rotation.T @ rot_w)[2]))
        return pos_err, aim_err, roll_err

    def _deviation(self, spec: ss.ShotSpec, i: int, sampler, s_values: Sequence[float]) -> Tuple[float, float, float]:
        """@brief 候选原语位姿函数 sampler(s) → 法兰 4×4（基座系）相对参考的最大 (位置, 指向, roll) 偏差。"""
        worst = [0.0, 0.0, 0.0]
        for s in s_values:
            t_tool = sampler(s)
            errs = self.pose_error(ss.reference_pose(spec, i, s), self.frames.pose_from_flange_matrix(t_tool))
            worst = [max(w, e) for w, e in zip(worst, errs)]
        return worst[0], worst[1], worst[2]

    def _within(self, dev: Tuple[float, float, float], tol: ss.Tolerance) -> bool:
        """@brief 偏差是否在预算内。"""
        b = self.tolerance_budget
        return dev[0] <= b * tol.position and dev[1] <= b * tol.aim and dev[2] <= b * tol.roll

    # ── 候选原语 ──
    @staticmethod
    def _linear_sampler(t0: np.ndarray, t1: np.ndarray):
        """@brief Commander LINEAR 的几何：法兰位置线性、姿态 slerp。"""
        def _at(u: float) -> np.ndarray:
            out = np.eye(4)
            out[:3, :3] = ss.slerp(t0[:3, :3], t1[:3, :3], u)
            out[:3, 3] = t0[:3, 3] + u * (t1[:3, 3] - t0[:3, 3])
            return out
        return _at

    @staticmethod
    def _orbit_sampler(center: np.ndarray, sph0: Tuple[float, float, float], sph1: Tuple[float, float, float]):
        """@brief Commander ORBIT 的几何：(θ, φ, r) 线性插值，法兰 +X 对准球心、roll 0（aim_quat）。"""
        def _at(u: float) -> np.ndarray:
            th = sph0[0] + u * (sph1[0] - sph0[0])
            ph = sph0[1] + u * (sph1[1] - sph0[1])
            r = sph0[2] + u * (sph1[2] - sph0[2])
            pos = sphere_to_cart(th, ph, r, center)
            return pose_to_matrix(pose_from_look_at(pos, center))
        return _at

    def _chords(self, spec: ss.ShotSpec, i: int, n: int) -> Tuple[List[Tuple[float, float]], List[np.ndarray]]:
        """@brief 把第 i 段按进度等分成 n 条弦：返回 [(s_k, s_k+1)] 与 n+1 个端点法兰 4×4。"""
        knots = np.linspace(0.0, 1.0, n + 1)
        mats = []
        for s in knots:
            rot, pos = self._ref_optical_arm(spec, i, float(s))
            mats.append(self.frames.flange_matrix(rot, pos))
        return [(float(knots[k]), float(knots[k + 1])) for k in range(n)], mats

    def _piecewise_sampler(self, spans: List[Tuple[float, float]], mats: List[np.ndarray]):
        """@brief 多条 LINEAR 弦拼成的整段位姿函数 s → 法兰 4×4。"""
        samplers = [self._linear_sampler(mats[k], mats[k + 1]) for k in range(len(spans))]

        def _at(s: float) -> np.ndarray:
            for k, (a, b) in enumerate(spans):
                if s <= b or k == len(spans) - 1:
                    u = 0.0 if b - a < 1e-12 else (s - a) / (b - a)
                    return samplers[k](min(1.0, max(0.0, u)))
            return samplers[-1](1.0)
        return _at

    def _try_orbit(self, spec: ss.ShotSpec, i: int, s_values: Sequence[float]) -> Optional[Tuple[Any, Tuple]]:
        """@brief 试 ORBIT：look_at 恒定且 roll 恒为 0 才有意义。球心取三条法兰光轴射线的最近点；
               θ1 在 {θ1, θ1±2π} 里挑与参考路径偏差最小的（对应 arc 的 axis 转向）。
        @return (sampler, (center, sph0, sph1)) 或 None
        """
        w0, w1 = spec.waypoints[i], spec.waypoints[i + 1]
        if np.linalg.norm(w1.look_at - w0.look_at) > 1e-6 or abs(w0.roll) > 1e-9 or abs(w1.roll) > 1e-9:
            return None
        if ss.pure_pan(spec, i):
            return None
        rays = []
        for s in (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0):    # 四条射线：整圈环绕时 s=0/1 重合，中间两条补秩
            rot, pos = self._ref_optical_arm(spec, i, s)
            t_tool = self.frames.flange_matrix(rot, pos)
            rays.append((t_tool[:3, 3], t_tool[:3, 0]))
        # 最小二乘：min Σ |(I − d dᵀ)(c − p)|²
        a_mat = np.zeros((3, 3))
        b_vec = np.zeros(3)
        for p, d in rays:
            proj = np.eye(3) - np.outer(d, d)
            a_mat += proj
            b_vec += proj @ p
        if np.linalg.matrix_rank(a_mat, tol=1e-9) < 3:
            return None
        center = np.linalg.solve(a_mat, b_vec)
        t0 = self.frames.flange_matrix(*self._ref_optical_arm(spec, i, 0.0))
        t1 = self.frames.flange_matrix(*self._ref_optical_arm(spec, i, 1.0))
        th0, ph0, r0 = cart_to_sphere(t0[:3, 3], center)
        th1, ph1, r1 = cart_to_sphere(t1[:3, 3], center)
        if r0 < 1e-6 or r1 < 1e-6:
            return None
        # 期望的方位扫角：B 型球坐标段就是 Δaz（可超过 2π）；A 型竖直轴圆弧是 sweep×sign(axis_z)。
        # 用它把 θ1 解回绕到正确的圈上，再在 ±2π 邻域里挑与参考偏差最小的。
        seg = spec.segments[i]
        expected = None
        if seg.track is not None and seg.track.sph is not None:
            expected = seg.track.sph[i + 1][1] - seg.track.sph[i][1]
        elif seg.path == 'arc':
            _, axis, _, sweep = ss._arc_params(spec, i)
            if abs(axis[2]) > 0.99:
                expected = sweep * (1.0 if axis[2] > 0 else -1.0)
        base = th1
        if expected is not None:
            base = th0 + expected + 2 * math.pi * round((th1 - th0 - expected) / (2 * math.pi))
        best = None
        for th1c in (base, base + 2 * math.pi, base - 2 * math.pi):
            sampler = self._orbit_sampler(center, (th0, ph0, r0), (th1c, ph1, r1))
            dev = self._deviation(spec, i, sampler, s_values)
            if best is None or dev[0] < best[0][0]:
                best = (dev, sampler, (center, (th0, ph0, r0), (th1c, ph1, r1)))
        dev, sampler, params = best
        if not self._within(dev, spec.tolerance):
            return None
        return sampler, params

    # ── 时间 ──
    def _nominal_sec(self, kind: str, level: str, prims: List[Dict[str, Any]]) -> float:
        """@brief 一段在某档位下的名义运动时长（不含 dwell）：每条原语取位置 / 姿态两组约束的更严者。"""
        lim = self.speed_profiles[level]
        total = 0.0
        for p in prims:
            total += max(p['dist'] / lim.v_pos, p['ang'] / lim.v_ori)
        return total

    def _pick_speed(self, spec: ss.ShotSpec, i: int, prims: List[Dict[str, Any]],
                    warnings: List[str]) -> Tuple[str, float, str]:
        """@brief 由 duration / speed 选档位。
        @return (level, 该段名义时长（含弦间 dwell）, 'none'|'slowed')
        """
        seg = spec.segments[i]
        length = ss.segment_length(spec, i)
        t_req = seg.duration if seg.duration is not None else length / seg.speed
        extra = (len(prims) - 1) * self.dwell_sec
        best_level, best_t, best_score = None, 0.0, math.inf
        for level in ('slow', 'normal', 'fast'):
            t_nom = self._nominal_sec('', level, prims) + extra
            score = abs(math.log(max(t_nom, 1e-9) / max(t_req, 1e-9)))
            if score < best_score:
                best_level, best_t, best_score = level, t_nom, score
        degradation = 'none'
        if best_t > 1.2 * t_req:
            degradation = 'slowed'
            warnings.append(f'segments[{i}] 要求 {t_req:.1f} s，Commander 最快档（fast）也要约 {best_t:.1f} s：'
                            '按 fast 执行并标 degradation=slowed' if best_level == 'fast' else
                            f'segments[{i}] 要求 {t_req:.1f} s，量化到 {best_level} 档后约 {best_t:.1f} s（偏慢）')
        elif best_t < t_req / 1.2:
            warnings.append(f'segments[{i}] 要求 {t_req:.1f} s，Commander 最慢档（slow）只需约 {best_t:.1f} s：'
                            '会比要求的快，节奏对不上' if best_level == 'slow' else
                            f'segments[{i}] 要求 {t_req:.1f} s，量化到 {best_level} 档后约 {best_t:.1f} s（偏快）')
        return best_level, best_t, degradation

    # ── 可达性 ──
    def _check_waypoints(self, spec: ss.ShotSpec, start_joints: Optional[Sequence[float]],
                         report: ss.ValidationReport) -> List[Optional[np.ndarray]]:
        """@brief 规则 25：每个 waypoint 的法兰位姿过 check_pose。返回各点关节解（不可达为 None）。"""
        seed = np.asarray(start_joints, dtype=float) if start_joints is not None else None
        sols: List[Optional[np.ndarray]] = []
        for k, wp in enumerate(spec.waypoints):
            res = check_pose(self.model, self._waypoint_flange(wp), self.margins, seed=seed)
            if not res.ok:
                suggestion = None
                if res.suggestion is not None:
                    _, p_sug = self.frames.optical_from_flange(res.suggestion)
                    suggestion = {'position': self.world.to_world_point(p_sug)}
                report.errors.append(ss.SpecError(25, f'waypoints[{k}]', f'相机位姿不可达：{res.reason}', suggestion))
                sols.append(None)
            else:
                sols.append(res.joints)
                seed = res.joints
        return sols

    def _check_path(self, i: int, sampler, seed: Optional[np.ndarray], length: float, ang: float,
                    report: ss.ValidationReport) -> Optional[np.ndarray]:
        """@brief 规则 26：沿一条原语自身的法兰路径连续采样 IK（不换几何分支）。返回末点关节解。"""
        n = int(max(6, math.ceil(length / self.reach_step_m), math.ceil(ang / self.reach_step_rad)))
        n = min(n, 400)
        q = seed
        for k in range(n + 1):
            u = k / n
            pose = self.frames.pose_from_flange_matrix(sampler(u))
            if q is None:
                res = check_pose(self.model, pose, self.margins)
            else:
                res = check_pose(self.model, pose, self.margins, seed=q, continuous=True)
                if not res.ok:   # 连续解不成立时再放开分支确认一次，避免精修不收敛的误报
                    res = check_pose(self.model, pose, self.margins, seed=q)
            if not res.ok:
                report.errors.append(ss.SpecError(26, f'segments[{i}]',
                                                  f'路径进度 {u:.2f} 处不可达：{res.reason or "IK 无解"}'))
                return None
            q = res.joints
        return q

    # ── 主入口 ──
    def compile(self, spec: ss.ShotSpec, start_joints: Optional[Sequence[float]] = None) -> CompiledShot:
        """@brief 编译 A 型 ShotSpec。
        @param spec         A 型（B 型先 tracking_to_static）
        @param start_joints 当前 6 轴关节角（rad），作 IK 种子；None 取限位中点
        @return CompiledShot
        @throws SpecValidationError 规则 25 / 26 不可达，或某段无法在容差内用 Commander 原语复现
        @throws ValueError 传入 B 型
        """
        if spec.is_tracking:
            raise ValueError('compile 只接受 A 型；B 型先用 shot_spec.tracking_to_static 按目标快照展开')
        report = ss.ValidationReport()
        warnings: List[str] = [str(w) for w in spec.warnings]
        degradation = 'none'
        sols = self._check_waypoints(spec, start_joints, report)
        if report.errors:
            raise ss.SpecValidationError(report, '运镜不可达')

        holds = [float(w.hold) for w in spec.waypoints]
        primitives: List[Primitive] = []
        if holds[0] > 0 or not spec.segments:
            pose0 = self._waypoint_flange(spec.waypoints[0])
            primitives.append(Primitive('ptp', -1, 0.0, 1.0, 'fast', 0.0, pose=pose0))

        q = sols[0]
        s_values = np.linspace(0.0, 1.0, 41)
        for i, seg in enumerate(spec.segments):
            tol = spec.tolerance
            length = ss.segment_length(spec, i)
            chosen: Optional[List[Dict[str, Any]]] = None
            # ① 单条 LINEAR
            spans, mats = self._chords(spec, i, 1)
            sampler = self._linear_sampler(mats[0], mats[1])
            if self._within(self._deviation(spec, i, sampler, s_values), tol):
                chosen = [self._linear_desc(spans[0], mats[0], mats[1], sampler)]
            # ② ORBIT
            if chosen is None:
                orbit = self._try_orbit(spec, i, s_values)
                if orbit is not None:
                    sampler, (center, sph0, sph1) = orbit
                    d_ang = math.hypot(sph1[0] - sph0[0], sph1[1] - sph0[1])
                    r_avg = 0.5 * (sph0[2] + sph1[2])
                    arc = math.hypot(r_avg * d_ang, sph1[2] - sph0[2])
                    chosen = [{'kind': 'orbit', 'span': (0.0, 1.0), 'sampler': sampler, 'center': center,
                               'sph0': sph0, 'sph1': sph1, 'dist': arc, 'ang': d_ang}]
            # ③ 细分成 N 条 LINEAR 弦
            if chosen is None:
                for n in range(2, self.max_chords + 1):
                    spans, mats = self._chords(spec, i, n)
                    sampler = self._piecewise_sampler(spans, mats)
                    if self._within(self._deviation(spec, i, sampler, s_values), tol):
                        chosen = [self._linear_desc(spans[k], mats[k], mats[k + 1],
                                                    self._linear_sampler(mats[k], mats[k + 1]))
                                  for k in range(n)]
                        warnings.append(f'segments[{i}]（{seg.path}）无法用单条 Commander 原语在容差内复现，'
                                        f'细分为 {n} 条直线弦；弦间各有约 {self.dwell_sec:.0f} s 的起点停顿')
                        break
            if chosen is None:
                dev = self._deviation(spec, i, self._piecewise_sampler(*self._chords(spec, i, self.max_chords)),
                                      s_values)
                report.errors.append(ss.SpecError(
                    'L1', f'segments[{i}]',
                    f'细分到 {self.max_chords} 条弦仍超出容差预算（偏差 位置 {dev[0]:.3f} m / 指向 {dev[1]:.3f} rad / '
                    f'roll {dev[2]:.3f} rad，预算 = tolerance × {self.tolerance_budget}）：放宽 tolerance 或拆段'))
                continue
            # 速度档位（整段统一）
            level, seg_nominal, seg_deg = self._pick_speed(spec, i, chosen, warnings)
            if seg_deg == 'slowed':
                degradation = 'slowed'
            if seg.law not in _NATIVE_LAWS:
                warnings.append(f'segments[{i}].law="{seg.law}" 不受支持：Commander 每条原语都是 Ruckig 静止→静止的 S 曲线'
                                '（等价 s_curve）')
            if i == len(spec.segments) - 1 and seg.law in _END_VELOCITY_LAWS:
                degradation = 'slowed'
                warnings.append(f'末段 law="{seg.law}" 末端带速度，执行层会减速停住（规范 4.1 → degradation=slowed）')
            # 可达性（沿每条原语自己的法兰路径）
            for desc in chosen:
                q = self._check_path(i, desc['sampler'], q, desc['dist'], desc['ang'], report)
                if q is None:
                    break
            if report.errors:
                continue
            # 生成原语
            for desc in chosen:
                lim = self.speed_profiles[level]
                nominal = max(desc['dist'] / lim.v_pos, desc['ang'] / lim.v_ori)
                s0, s1 = desc['span']
                if desc['kind'] == 'linear':
                    primitives.append(Primitive('linear', i, s0, s1, level, nominal,
                                                start=self.frames.pose_from_flange_matrix(desc['t0']),
                                                end=self.frames.pose_from_flange_matrix(desc['t1'])))
                else:
                    (c, sph0, sph1) = desc['center'], desc['sph0'], desc['sph1']
                    primitives.append(Primitive('orbit', i, s0, s1, level, nominal, center=np.asarray(c),
                                                az0=math.degrees(sph0[0]), el0=math.degrees(sph0[1]), r0=sph0[2],
                                                az1=math.degrees(sph1[0]), el1=math.degrees(sph1[1]), r1=sph1[2]))
        if report.errors:
            raise ss.SpecValidationError(report, '运镜无法编译到 Commander 原语')
        total = sum(p.nominal_sec for p in primitives) + len(primitives) * self.dwell_sec + sum(holds)
        return CompiledShot(spec, primitives, holds, warnings, degradation, total, self.dwell_sec)

    @staticmethod
    def _linear_desc(span: Tuple[float, float], t0: np.ndarray, t1: np.ndarray, sampler) -> Dict[str, Any]:
        """@brief 一条 LINEAR 弦的描述（位移与转角用于算时长）。"""
        dist = float(np.linalg.norm(t1[:3, 3] - t0[:3, 3]))
        ang = float(np.linalg.norm(ss.log_so3(t0[:3, :3].T @ t1[:3, :3])))
        return {'kind': 'linear', 'span': span, 'sampler': sampler, 't0': t0, 't1': t1, 'dist': dist, 'ang': ang}
