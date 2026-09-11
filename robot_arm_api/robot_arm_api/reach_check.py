# -*- coding: utf-8 -*-
"""
@file      reach_check.py
@brief     机械臂（J1-3）+ 云台 V2（J4-6）的可达性判定 / 余量计算 / 能力卡生成——给大模型运镜决策用。
           纯 numpy，不依赖 ROS、pinocchio；运动学常量一律从 URDF 现算（换云台、改限位自动跟着变）。
@version   0.1
@date      2026-09-09
@copyright Copyright (c) 2026 eMeet

定位：大模型是概率生成器，不是约束求解器。本模块提供两层保障：
  ① 余量盒子（headroom）：从当前关节角算"各方向还能动多少"的一维区间，让大模型只在区间里选数；
  ② 事后校验（check_pose / check_plan）：把大模型输出的位姿 / JSON 步骤表在执行前推演、判可达、给人话原因。
两层共用同一个 ArmModel：位置走闭式二连杆解（J1 偏航 + J2/J3 平行轴），朝向由云台 3 轴数值精解。

坐标约定（与 Commander 一致）：位姿在机械臂 arm_base_link 系（x 前 / y 左 / z 上），末端 = 云台法兰
gimbal_tool0（x 前 / z 上），rpy 按 motion::rpy_to_quat 的 ZYX 约定 R = Rz(yaw)·Ry(pitch)·Rx(roll)。

═══════════════════════════════ 本文件函数 / 类汇总 ═══════════════════════════════
  _rpy_to_mat(rpy)                    URDF rpy → 3×3 旋转
  _axis_angle_mat(axis, q)            绕单位轴转 q → 3×3 旋转
  _make_tf(xyz, rpy)                  URDF origin → 4×4 齐次变换
  Joint                               URDF 关节（含 origin / axis / limit）
  ArmModel.from_urdf_string(xml)      从完整 URDF 字符串建模（沿 parent→child 走链）
  ArmModel.from_share_files()         用 xacro 展开 robot_arm_description 的 arm.urdf.xacro 后建模
                                      （需 ROS 环境）
  ArmModel.fk / wrist_tf / ik         正解 / 云台段正解 / 全 6 轴逆解（闭式 + 数值精修）
  pose_to_matrix / matrix_to_pose     ArmPose 语义（米 / 度）↔ 4×4
  pose_from_look_at(pos, look_at)     相机位置 + 目标点 → ArmPose 字典（与 Commander aim_quat 同约定）
  Margins                             判定余量：距离（臂长边界）+ 各关节（rad）
  ReachResult                         check_pose 结果：ok / reason / joints / margins / suggestion
  check_pose(model, pose, margins)    单点可达判定，给人话原因与最近可行位姿
  theta_ref / sphere_to_cart / cart_to_sphere / arc_pose   球面环绕几何（与 motion/geometry.hpp 一致）
  headroom(model, joints, subject)    余量盒子：各方向连续可动的一维区间（dolly/truck/crane/dyaw/dpitch/arc_az）
  check_plan(model, steps, joints)    JSON 步骤表预检（reject / clip / project 三种模式），
                                      PlanReport.summary() 给人话汇总
  capability_data / capability_card   能力卡：结构化数据 / 喂给大模型的文本（静态表按高度分段 + 当前状态与余量）
  current_state_text(model, joints)   能力卡的"当前状态"段（每步重发的那部分）
  headroom_schema(h)                  余量区间 → 单步 JSON schema（oneOf 每 op 一项，min/max 硬约束）
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Union

import numpy as np

# ─────────────────────────────── 基础几何 ───────────────────────────────


def _rpy_to_mat(rpy: Sequence[float]) -> np.ndarray:
    """@brief URDF 的 rpy（固定轴 X-Y-Z，即 R = Rz(yaw)·Ry(pitch)·Rx(roll)）→ 3×3 旋转矩阵。
    @param rpy (roll, pitch, yaw)，rad
    @return 3×3 旋转矩阵
    """
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _axis_angle_mat(axis: Sequence[float], q: float) -> np.ndarray:
    """@brief 绕单位轴 axis 旋转 q（Rodrigues 公式）。
    @param axis 旋转轴（会归一化）
    @param q    角度，rad
    @return 3×3 旋转矩阵
    """
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(q) * k + (1.0 - math.cos(q)) * (k @ k)


def _make_tf(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """@brief URDF origin（xyz + rpy）→ 4×4 齐次变换。
    @param xyz 平移，m
    @param rpy 旋转，rad
    @return 4×4 矩阵
    """
    tf = np.eye(4)
    tf[:3, :3] = _rpy_to_mat(rpy)
    tf[:3, 3] = np.asarray(xyz, dtype=float)
    return tf


def _inv_tf(tf: np.ndarray) -> np.ndarray:
    """@brief 齐次变换求逆（利用旋转正交性，不做通用矩阵求逆）。
    @param tf 4×4
    @return 4×4 逆变换
    """
    inv = np.eye(4)
    rot_t = tf[:3, :3].T
    inv[:3, :3] = rot_t
    inv[:3, 3] = -rot_t @ tf[:3, 3]
    return inv


def _rot_log(rot: np.ndarray) -> np.ndarray:
    """@brief 旋转矩阵 → 旋转向量（轴 × 角，rad），小角与 π 附近都稳定。
    @param rot 3×3 旋转矩阵
    @return 3 维旋转向量
    """
    cos_a = max(-1.0, min(1.0, (np.trace(rot) - 1.0) * 0.5))
    angle = math.acos(cos_a)
    if angle < 1e-12:
        return np.zeros(3)
    if angle > math.pi - 1e-6:
        # 接近 π：用对称部分取轴
        diag = np.clip((np.diag(rot) + 1.0) * 0.5, 0.0, 1.0)
        axis = np.sqrt(diag)
        # 由非对角元定符号（以最大分量为正）
        i = int(np.argmax(axis))
        for j in range(3):
            if j != i and (rot[i, j] + rot[j, i]) < 0:
                axis[j] = -axis[j]
        return axis / np.linalg.norm(axis) * angle
    skew = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return skew / (2.0 * math.sin(angle)) * angle


def _wrap_pi(angle: float) -> float:
    """@brief 角度归一化到 (-π, π]。
    @param angle rad
    @return 归一化后的角度
    """
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi else wrapped


def _wrap_into(angle: float, lower: float, upper: float) -> float:
    """@brief 把角度按 2π 周期挪到 [lower, upper] 内；挪不进去就取最接近区间中点的那个代表值。
    @param angle rad
    @param lower 下限
    @param upper 上限
    @return 调整后的角度
    """
    mid = 0.5 * (lower + upper)
    best = angle + 2.0 * math.pi * round((mid - angle) / (2.0 * math.pi))
    return best


# ─────────────────────────────── URDF 链 ───────────────────────────────


@dataclass
class Joint:
    """@brief URDF 关节：类型、父子 link、origin、轴、限位（fixed 关节 axis/limit 为 None）。"""
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray                 # 4×4，父 link 系 → 关节系
    axis: Optional[np.ndarray] = None  # 关节系内的转轴（revolute）
    lower: float = 0.0
    upper: float = 0.0


def _parse_vec(text: Optional[str], default: Sequence[float]) -> List[float]:
    """@brief 解析 URDF 里 "a b c" 形式的向量属性。
    @param text    属性字符串，None 取默认
    @param default 默认值
    @return 3 个 float
    """
    if not text:
        return [float(v) for v in default]
    return [float(v) for v in text.split()]


def _parse_joint(elem: ET.Element) -> Joint:
    """@brief 把 <joint> 元素解析成 Joint。
    @param elem <robot> 直属的 <joint> 元素
    @return Joint
    """
    origin = elem.find('origin')
    xyz = _parse_vec(origin.get('xyz') if origin is not None else None, (0.0, 0.0, 0.0))
    rpy = _parse_vec(origin.get('rpy') if origin is not None else None, (0.0, 0.0, 0.0))
    joint = Joint(name=elem.get('name', ''), type=elem.get('type', 'fixed'),
                  parent=elem.find('parent').get('link'), child=elem.find('child').get('link'),
                  origin=_make_tf(xyz, rpy))
    if joint.type in ('revolute', 'continuous'):
        axis = elem.find('axis')
        joint.axis = np.asarray(_parse_vec(axis.get('xyz') if axis is not None else None,
                                           (1.0, 0.0, 0.0)), dtype=float)
        limit = elem.find('limit')
        if limit is not None:
            joint.lower = float(limit.get('lower', 0.0))
            joint.upper = float(limit.get('upper', 0.0))
    return joint


def _chain_tf(joints: Sequence[Joint], q: Sequence[float]) -> np.ndarray:
    """@brief 一段关节链的正解（首关节父 link 系 → 末关节子 link 系）。
    @param joints 关节序列（fixed + revolute）
    @param q      其中转动关节的角度，按出现顺序
    @return 4×4
    """
    tf = np.eye(4)
    k = 0
    for joint in joints:
        tf = tf @ joint.origin
        if joint.axis is not None:
            rot = np.eye(4)
            rot[:3, :3] = _axis_angle_mat(joint.axis, q[k])
            tf = tf @ rot
            k += 1
    return tf


class _PlanarArm:
    """@brief 「J1 偏航 + J2/J3 平行轴平面二连杆」的闭式结构，从关节链现算常量。

    在 J1 关节系 F1 内工作：J1 绕 z 转；n = J2 转轴（不要求严格水平，URDF 里 1.5708 这类
    四位小数会带来 ~1e-6 rad 的倾斜，这里按精确几何处理）；臂平面 ⊥ n、固定在 n 坐标 = H 处。
    平面内正交基：w = z 去掉 n 分量后归一（"竖直"方向），u = w × n（绕 +n 转正角把 u 转向 w）。
    J2 转 q2 使上臂在 (u, w) 内的角 ψ2 = c2 + q2；J3 转 q3 使小臂角 ψ3 = ψ2 + s3·q3 + c3。
    """

    def __init__(self, arm_chain: List[Joint], lower: np.ndarray, upper: np.ndarray):
        """@brief 由 base→臂末端（第 4 个转动关节之前）的关节链提取常量，并校验结构假设。
        @param arm_chain 含 3 个转动关节的链
        @param lower     3 个臂关节下限
        @param upper     3 个臂关节上限
        @throws ValueError 不满足平面结构（J1 轴不沿关节系 z、J2/J3 轴不平行、J2 轴平行 J1 轴、小臂出平面）
        """
        self.lower, self.upper = lower, upper
        idx = [i for i, j in enumerate(arm_chain) if j.axis is not None]
        if len(idx) != 3:
            raise ValueError(f'臂段需要 3 个转动关节，实际 {len(idx)}')
        i1, i2, i3 = idx
        # base → F1（J1 关节系，未转动）
        self.tf_base_f1 = _chain_tf(arm_chain[:i1], []) @ arm_chain[i1].origin
        axis1 = arm_chain[i1].axis / np.linalg.norm(arm_chain[i1].axis)
        if abs(abs(axis1[2]) - 1.0) > 1e-9:
            raise ValueError('J1 转轴必须沿其关节系 z 轴')
        self.s1 = 1.0 if axis1[2] > 0 else -1.0
        # F1'(q1=0 时 = F1) 内：J2 关节系、J3 关节系、臂末端
        tf_12 = _chain_tf(arm_chain[i1 + 1:i2], []) @ arm_chain[i2].origin
        tf_23 = _chain_tf(arm_chain[i2 + 1:i3], []) @ arm_chain[i3].origin
        tf_3t = _chain_tf(arm_chain[i3 + 1:], [])
        axis2 = tf_12[:3, :3] @ arm_chain[i2].axis
        axis3 = (tf_12 @ tf_23)[:3, :3] @ arm_chain[i3].axis
        n = axis2 / np.linalg.norm(axis2)
        axis3 = axis3 / np.linalg.norm(axis3)
        if abs(abs(n @ axis3) - 1.0) > 1e-9:
            raise ValueError('J2/J3 转轴必须平行（平面二连杆假设不成立）')
        self.n_xy_norm = float(math.hypot(n[0], n[1]))
        if self.n_xy_norm < 1e-6:
            raise ValueError('J2 转轴不能与 J1 转轴平行')
        self.phi_n = math.atan2(n[1], n[0])
        self.s3 = 1.0 if n @ axis3 > 0 else -1.0
        self.n = n
        z_axis = np.array([0.0, 0.0, 1.0])
        w = z_axis - n[2] * n
        self.w = w / np.linalg.norm(w)
        self.u = np.cross(self.w, n)
        shoulder = tf_12[:3, 3]
        elbow0 = (tf_12 @ tf_23)[:3, 3]
        tip0 = (tf_12 @ tf_23 @ tf_3t)[:3, 3]
        self.height_n = float(tip0 @ n)                       # 臂平面的 n 坐标 H
        if abs((elbow0 - shoulder) @ n - (tip0 - shoulder) @ n) > 1e-9:
            raise ValueError('小臂末端与肘不在同一平面内')
        self.shoulder = shoulder
        v2 = elbow0 - shoulder
        v3 = tip0 - elbow0
        self.len2 = float(math.hypot(v2 @ self.u, v2 @ self.w))
        self.len3 = float(math.hypot(v3 @ self.u, v3 @ self.w))
        self.c2 = math.atan2(v2 @ self.w, v2 @ self.u)
        self.c3 = math.atan2(v3 @ self.w, v3 @ self.u) - self.c2
        self.dist_min = math.sqrt(max(0.0, self.len2 ** 2 + self.len3 ** 2
                                      + 2 * self.len2 * self.len3 * math.cos(self._beta(upper[2]))))
        self.dist_max = math.sqrt(self.len2 ** 2 + self.len3 ** 2
                                  + 2 * self.len2 * self.len3 * math.cos(self._beta(lower[2])))
        if self.dist_min > self.dist_max:
            self.dist_min, self.dist_max = self.dist_max, self.dist_min

    def _beta(self, q3: float) -> float:
        """@brief 肘部相对角 β = ψ3 − ψ2。
        @param q3 J3 角
        @return β，rad
        """
        return self.s3 * q3 + self.c3

    def tip_position(self, q123: Sequence[float]) -> np.ndarray:
        """@brief 平面模型正解：臂末端在 base 系的位置（用于与链式正解对拍）。
        @param q123 (q1, q2, q3)
        @return 3 维位置
        """
        psi2 = self.c2 + q123[1]
        psi3 = psi2 + self._beta(q123[2])
        p_f1 = (self.shoulder
                + self.len2 * (math.cos(psi2) * self.u + math.sin(psi2) * self.w)
                + self.len3 * (math.cos(psi3) * self.u + math.sin(psi3) * self.w)
                + (self.height_n - self.shoulder @ self.n) * self.n)
        rot1 = _axis_angle_mat((0.0, 0.0, 1.0), self.s1 * q123[0])
        return (self.tf_base_f1 @ np.append(rot1 @ p_f1, 1.0))[:3]

    def solve(self, p_base: Sequence[float]) -> List[np.ndarray]:
        """@brief 闭式逆解：给臂末端位置，返回所有几何解 (q1, q2, q3)（J1 两分支 × 肘两分支），
               角度已按 2π 周期挪进各自限位所在圈；不做限位过滤，由调用方筛。

        J1 分支：绕 z 转 −q1 后的点必须落在臂平面 n·p' = H 上，
        即 ρ·|n_xy|·cos(φp − q1 − φn) + pz·nz = H，两解 q1 = φp − φn ∓ γ。
        @param p_base 臂末端位置，base 系
        @return 解列表（可能为空：离 J1 轴太近或超出臂长）
        """
        p_f1 = (_inv_tf(self.tf_base_f1) @ np.append(np.asarray(p_base, dtype=float), 1.0))[:3]
        rho = math.hypot(p_f1[0], p_f1[1])
        if rho < 1e-12:
            return []
        rhs = (self.height_n - p_f1[2] * self.n[2]) / (rho * self.n_xy_norm)
        # 点落在 J1 轴周围半径 |H| 的"盲柱"里时几何上无解；但作为数值精修的初值，夹到切点继续给
        # 候选（末端靠近轴时云台偏移估计误差常把点推进盲柱，真解仍可能存在）。
        gamma = math.acos(max(-1.0, min(1.0, rhs)))
        phi_p = math.atan2(p_f1[1], p_f1[0])
        sols: List[np.ndarray] = []
        for q1_raw in (phi_p - self.phi_n - gamma, phi_p - self.phi_n + gamma):
            q1 = _wrap_into(self.s1 * q1_raw, self.lower[0], self.upper[0])
            for q2, q3 in self._solve_2r(p_f1, q1_raw):
                sols.append(np.array([q1, q2, q3]))
        return sols

    def solve_with_q1(self, p_base: Sequence[float], q1: float) -> List[np.ndarray]:
        """@brief 固定 J1 的近似闭式解：把目标点投影到该 J1 角对应的臂平面上再解二连杆
               （出平面分量被忽略，只作数值精修的初值——用于末端靠近 J1 轴、方位角病态的情形）。
        @param p_base 臂末端位置，base 系
        @param q1     指定的 J1 角
        @return (q1, q2, q3) 候选列表（肘两分支）
        """
        p_f1 = (_inv_tf(self.tf_base_f1) @ np.append(np.asarray(p_base, dtype=float), 1.0))[:3]
        return [np.array([q1, q2, q3]) for q2, q3 in self._solve_2r(p_f1, self.s1 * q1)]

    def _solve_2r(self, p_f1: np.ndarray, q1_raw: float) -> List[tuple]:
        """@brief 给定绕 z 的转角 q1_raw，把 F1 系点转回臂平面后解平面二连杆。
        @param p_f1   目标点，F1 系
        @param q1_raw 绕 z 的几何转角（= s1·q1）
        @return [(q2, q3), ...]，0~2 个（已挪进限位所在圈）
        """
        p_plane = _axis_angle_mat((0.0, 0.0, 1.0), -q1_raw) @ p_f1 - self.shoulder
        xr = float(p_plane @ self.u)
        zr = float(p_plane @ self.w)
        d2 = xr * xr + zr * zr
        cos_b = (d2 - self.len2 ** 2 - self.len3 ** 2) / (2.0 * self.len2 * self.len3)
        # 这里的解只当数值精修的初值：目标超臂长 / 过折叠（因云台偏移估计不准）时夹到
        # 伸直 / 折叠位形继续算，而不是直接放弃；只有离谱到两倍臂长之外才判无解。
        if d2 > (2.0 * (self.len2 + self.len3)) ** 2:
            return []
        cos_b = max(-1.0, min(1.0, cos_b))
        out: List[tuple] = []
        for sign in (1.0, -1.0):
            beta = sign * math.acos(cos_b)
            q3 = _wrap_into((beta - self.c3) / self.s3, self.lower[2], self.upper[2])
            psi2 = math.atan2(zr, xr) - math.atan2(self.len3 * math.sin(beta),
                                                  self.len2 + self.len3 * math.cos(beta))
            q2 = _wrap_into(psi2 - self.c2, self.lower[1], self.upper[1])
            out.append((q2, q3))
            if cos_b >= 1.0 - 1e-15:
                break  # 伸直：两分支重合
        return out


class ArmModel:
    """@brief 从 URDF 建出的 base→tip 串链运动学模型（fixed + revolute 混排，转动关节按链序编号）。

    结构假设：前 3 个转动关节 = 偏航 + 平行双轴平面臂（闭式位置解），后 3 个 = 云台（数值姿态解）。
    """

    def __init__(self, chain: List[Joint], base_link: str, tip_link: str):
        """@brief 由已排好序的关节链构造，并从链中提取平面臂常量、划分臂段 / 云台段。
        @param chain     base_link → tip_link 路径上的关节（含 fixed）
        @param base_link 基座 link 名
        @param tip_link  末端 link 名
        @throws ValueError 转动关节不是 6 个，或臂段不满足平面结构
        """
        self.chain = chain
        self.base_link = base_link
        self.tip_link = tip_link
        self.revolute = [j for j in chain if j.type in ('revolute', 'continuous')]
        self.joint_names = [j.name for j in self.revolute]
        self.lower = np.array([j.lower for j in self.revolute], dtype=float)
        self.upper = np.array([j.upper for j in self.revolute], dtype=float)
        if len(self.revolute) != 6:
            raise ValueError(f'需要 3 臂 + 3 云台共 6 个转动关节，链上有 {len(self.revolute)} 个')
        idx4 = chain.index(self.revolute[3])
        self._arm_chain = chain[:idx4]
        self._wrist_chain = chain[idx4:]
        self.arm_tip_link = self._arm_chain[-1].child
        self.planar = _PlanarArm(self._arm_chain, self.lower[:3], self.upper[:3])
        self._self_check()

    def _self_check(self) -> None:
        """@brief 建模自检：平面闭式正解必须与链式正解一致，否则 URDF 不是本模块假设的结构。
        @throws ValueError 不一致
        """
        rng = np.random.default_rng(0)
        for _ in range(5):
            q = self.lower + rng.random(6) * (self.upper - self.lower)
            p_chain = self.fk(q, self.arm_tip_link)[:3, 3]
            p_plane = self.planar.tip_position(q[:3])
            if np.max(np.abs(p_chain - p_plane)) > 1e-9:
                raise ValueError('平面臂闭式模型与 URDF 链式正解不一致，结构假设不成立')

    # ── 建模入口 ──
    @classmethod
    def from_urdf_string(cls, xml: str, base_link: str = 'arm_base_link',
                         tip_link: str = 'gimbal_tool0') -> 'ArmModel':
        """@brief 从完整 URDF 字符串建模：只看 <robot> 直属 <joint>（跳过 ros2_control 里的同名元素），
               从 tip_link 沿 child→parent 回溯到 base_link。
        @param xml       URDF XML（如 /robot_description）
        @param base_link 基座 link 名
        @param tip_link  末端 link 名
        @return ArmModel
        @throws ValueError 链不通（tip 到不了 base）
        """
        root = ET.fromstring(xml)
        by_child: Dict[str, Joint] = {}
        for elem in root.findall('joint'):
            joint = _parse_joint(elem)
            by_child[joint.child] = joint
        chain: List[Joint] = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f'URDF 里从 {tip_link} 回溯不到 {base_link}（断在 link "{link}"）')
            joint = by_child[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return cls(chain, base_link, tip_link)

    @classmethod
    def from_share_files(cls, base_link: str = 'arm_base_link', tip_link: str = 'gimbal_tool0',
                         with_gimbal: bool = True) -> 'ArmModel':
        """@brief 离线建模：用 xacro 现场展开已安装的 robot_arm_description/urdf/arm.urdf.xacro
               （含 robot_gimbal_description_v2 云台）再建模。需要 source 了 ROS 与工作空间。
        @param base_link   基座 link 名
        @param tip_link    末端 link 名
        @param with_gimbal 是否带云台（False 时 tip 只能是 tool0 且模型退化为 3 轴，本模块不支持）
        @return ArmModel
        @throws RuntimeError xacro / 描述包不可用
        """
        try:
            import xacro  # noqa: WPS433  ROS 包
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory('robot_arm_description')
            if with_gimbal:
                get_package_share_directory('robot_gimbal_description_v2')
        except Exception as exc:  # pylint: disable=broad-except
            raise RuntimeError('from_share_files 需要 source ROS 与工作空间（xacro / 描述包）: '
                               f'{exc!r}') from exc
        import os
        import tempfile
        wrapper = ('<robot name="eMeetArm" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
                   '  <xacro:include filename="$(find robot_arm_description)/urdf/'
                   'arm.urdf.xacro"/>\n'
                   '  <link name="world"/>\n'
                   '  <xacro:emeet_arm parent="world" xyz="0 0 0" rpy="0 0 0" sim_mode="false" '
                   f'backend="real" arm_sim_mode="true" with_gimbal="{str(with_gimbal).lower()}" '
                   'controllers_yaml=""/>\n'
                   '</robot>\n')
        with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False) as fh:
            fh.write(wrapper)
            path = fh.name
        try:
            xml = xacro.process_file(path).toxml()
        finally:
            os.unlink(path)
        return cls.from_urdf_string(xml, base_link, tip_link)

    # ── 正解 ──
    def fk(self, q: Sequence[float], link: Optional[str] = None) -> np.ndarray:
        """@brief 正解：base_link 系下某个 link 的 4×4 位姿。
        @param q    转动关节角（按 joint_names 顺序，rad），多给的忽略
        @param link 要求的 link 名，None = tip_link；必须在链上
        @return 4×4 齐次变换
        @throws ValueError link 不在链上
        """
        target = link or self.tip_link
        q = np.asarray(q, dtype=float)
        tf = np.eye(4)
        k = 0
        for joint in self.chain:
            tf = tf @ joint.origin
            if joint.axis is not None:
                rot = np.eye(4)
                rot[:3, :3] = _axis_angle_mat(joint.axis, q[k])
                tf = tf @ rot
                k += 1
            if joint.child == target:
                return tf
        raise ValueError(f'link "{target}" 不在 {self.base_link}→{self.tip_link} 链上')

    def wrist_tf(self, q456: Sequence[float]) -> np.ndarray:
        """@brief 云台段正解：臂末端 link 系 → tip 系。
        @param q456 云台 3 个关节角
        @return 4×4
        """
        return _chain_tf(self._wrist_chain, q456)

    # ── 逆解 ──
    def _solve_wrist(self, rot_des: np.ndarray, q0: np.ndarray,
                     max_iter: int = 30) -> Optional[np.ndarray]:
        """@brief 云台 3 轴姿态数值解（阻尼最小二乘 + 数值雅可比）：wrist_tf(q)[:3,:3] → rot_des。
        @param rot_des  期望姿态（臂末端 link 系下）
        @param q0       初值
        @param max_iter 最大迭代
        @return 收敛则返回角度（已挪进限位所在圈），否则 None
        """
        q = np.array(q0, dtype=float)
        eps = 1e-6
        for _ in range(max_iter):
            rot = self.wrist_tf(q)[:3, :3]
            err = _rot_log(rot_des @ rot.T)
            if np.linalg.norm(err) < 1e-11:
                break
            jac = np.zeros((3, 3))
            for i in range(3):
                dq = np.zeros(3)
                dq[i] = eps
                rot_i = self.wrist_tf(q + dq)[:3, :3]
                jac[:, i] = _rot_log(rot_i @ rot.T) / eps
            step = np.linalg.solve(jac.T @ jac + 1e-8 * np.eye(3), jac.T @ err)
            q = q + step
        rot = self.wrist_tf(q)[:3, :3]
        if np.linalg.norm(_rot_log(rot_des @ rot.T)) > 1e-9:
            return None
        return np.array([_wrap_into(q[i], self.lower[3 + i], self.upper[3 + i]) for i in range(3)])

    def _solve_wrist_in_limits(self, rot_des: np.ndarray, q0: np.ndarray) -> Optional[np.ndarray]:
        """@brief 云台姿态解，优先返回限位内的那支：pan-tilt-roll 型云台每个姿态有两支欧拉解
               （第二支 tilt 翻过 90°，通常出限位），数值法从不同 pan 初值出发才能覆盖到正确那支。
        @param rot_des 期望姿态（臂末端 link 系下）
        @param q0      首选初值（上一轮的解 / 调用方种子）
        @return 限位内的解；没有限位内的解时返回任一收敛解；都不收敛返回 None
        """
        fallback: Optional[np.ndarray] = None
        seeds = [q0] + [np.array([a, 0.0, 0.0]) for a in (0.0, math.pi / 2, -math.pi / 2, math.pi)]
        for seed in seeds:
            sol = self._solve_wrist(rot_des, seed)
            if sol is None:
                continue
            if np.all(sol >= self.lower[3:] - 1e-9) and np.all(sol <= self.upper[3:] + 1e-9):
                return sol
            if fallback is None:
                fallback = sol
        return fallback

    def _pose_error(self, q: np.ndarray, target: np.ndarray) -> np.ndarray:
        """@brief 6 维位姿残差 [Δp; 旋转向量]（目标相对当前正解）。
        @param q      关节角
        @param target 目标 4×4
        @return 6 维残差
        """
        pose = self.fk(q)
        return np.concatenate([target[:3, 3] - pose[:3, 3],
                               _rot_log(target[:3, :3] @ pose[:3, :3].T)])

    def _polish(self, q0: np.ndarray, target: np.ndarray, tol_pos: float, tol_rot: float,
                max_iter: int = 12) -> Optional[np.ndarray]:
        """@brief 全 6 轴阻尼最小二乘精修（数值雅可比）：把定点迭代给出的近似解收敛到容差内。
        @param q0      近似解
        @param target  目标 4×4
        @param tol_pos 位置容差，m
        @param tol_rot 姿态容差，rad
        @param max_iter 最大迭代
        @return 收敛的关节角（已挪进限位所在圈），不收敛返回 None
        """
        q = np.array(q0, dtype=float)
        eps = 1e-6
        lam = 1e-6  # Levenberg-Marquardt 阻尼：步长让残差变小就放松，变大就加大重试
        err = self._pose_error(q, target)
        for _ in range(max_iter):
            if np.linalg.norm(err[:3]) < tol_pos and np.linalg.norm(err[3:]) < tol_rot:
                return np.array([_wrap_into(q[i], self.lower[i], self.upper[i]) for i in range(6)])
            jac = np.zeros((6, 6))
            for i in range(6):
                dq = np.zeros(6)
                dq[i] = eps
                jac[:, i] = (err - self._pose_error(q + dq, target)) / eps
            hess = jac.T @ jac
            grad = jac.T @ err
            while True:
                step = np.linalg.solve(hess + lam * np.eye(6), grad)
                q_new = q + step
                err_new = self._pose_error(q_new, target)
                if np.linalg.norm(err_new) < np.linalg.norm(err):
                    q, err = q_new, err_new
                    lam = max(lam * 0.1, 1e-12)
                    break
                lam *= 10.0
                if lam > 1e6:
                    return None
        return None

    def ik(self, target: np.ndarray, seed: Optional[Sequence[float]] = None,
           tol_pos: float = 1e-7, tol_rot: float = 1e-7) -> Optional[np.ndarray]:
        """@brief 全 6 轴逆解：tip 位姿 → 关节角。位置走平面臂闭式解，姿态走云台数值解，
               两者以"云台段带来的末端偏移"做几轮定点迭代得到近似解，再用 6 轴阻尼最小二乘精修
               到容差。所有几何分支（J1 两支 × 肘两支）都试，只返回在限位内且残差达标的解，
               多解时取离 seed 最近的。
        @param target  4×4 目标位姿（base 系下的 tip）
        @param seed    参考关节角（6），None 取限位中点
        @param tol_pos 位置残差容差，m
        @param tol_rot 姿态残差容差，rad
        @return 6 个关节角，或 None（不可达 / 超限位）
        """
        target = np.asarray(target, dtype=float)
        seed_q = (np.asarray(seed, dtype=float) if seed is not None
                  else 0.5 * (self.lower + self.upper))
        best: Optional[np.ndarray] = None
        best_dist = math.inf
        tried: List[np.ndarray] = []
        slack = 0.05  # 定点迭代阶段的限位判据松量（rad），近似解允许略出界，精修后再严格判
        for branch in range(4):
            q_w = seed_q[3:6].copy()
            q123_prev: Optional[np.ndarray] = None
            approx: Optional[np.ndarray] = None
            for _ in range(4):
                arm_tip_target = target @ _inv_tf(self.wrist_tf(q_w))
                cands = self.planar.solve(arm_tip_target[:3, 3])
                if q123_prev is None:
                    if branch >= len(cands):
                        break
                    q123 = cands[branch]
                else:
                    if not cands:
                        break
                    q123 = min(cands, key=lambda c: float(np.linalg.norm(c - q123_prev)))
                q123_prev = q123
                if np.any(q123 < self.lower[:3] - slack) or np.any(q123 > self.upper[:3] + slack):
                    approx = None
                    break  # 臂解明显出限位：这一支不用再算
                rot_arm = self.fk(np.concatenate([q123, q_w]), self.arm_tip_link)[:3, :3]
                q_w = self._solve_wrist_in_limits(rot_arm.T @ target[:3, :3], q_w)
                if q_w is None:
                    approx = None
                    break
                approx = np.concatenate([q123, q_w])
            if approx is None:
                continue
            sol = self._polish(approx, target, tol_pos, tol_rot)
            if sol is None:
                continue
            if np.any(sol < self.lower - 1e-9) or np.any(sol > self.upper + 1e-9):
                continue
            if any(np.allclose(sol, t, atol=1e-9) for t in tried):
                continue
            tried.append(sol)
            dist = float(np.linalg.norm(sol - seed_q))
            if dist < best_dist:
                best, best_dist = sol, dist
        if best is None:
            best = self._ik_multistart(target, seed_q, tol_pos, tol_rot)
        return best

    def _ik_multistart(self, target: np.ndarray, seed_q: np.ndarray,
                       tol_pos: float, tol_rot: float) -> Optional[np.ndarray]:
        """@brief 兜底逆解：末端靠近 J1 轴时方位角对云台偏移极敏感，定点迭代会漂走。
               这里在 J1 全程上铺一组初值，每个初值把 J1 钉住做几轮（q2,q3,云台）定点迭代得近似解，
               再交给 6 轴精修（精修阶段 J1 放开）。收集所有限位内的收敛解，取离 seed 最近的。
        @param target  4×4 目标位姿
        @param seed_q  参考关节角
        @param tol_pos 位置容差
        @param tol_rot 姿态容差
        @return 解或 None
        """
        best: Optional[np.ndarray] = None
        best_dist = math.inf
        for q1 in np.linspace(self.lower[0], self.upper[0], 9):
            q_w = seed_q[3:6].copy()
            approx_list: List[np.ndarray] = []
            for branch in range(2):
                q_w_b = q_w.copy()
                approx: Optional[np.ndarray] = None
                for _ in range(4):
                    arm_tip_target = target @ _inv_tf(self.wrist_tf(q_w_b))
                    cands = self.planar.solve_with_q1(arm_tip_target[:3, 3], float(q1))
                    if branch >= len(cands):
                        approx = None
                        break
                    q123 = cands[branch]
                    rot_arm = self.fk(np.concatenate([q123, q_w_b]), self.arm_tip_link)[:3, :3]
                    q_w_b = self._solve_wrist_in_limits(rot_arm.T @ target[:3, :3], q_w_b)
                    if q_w_b is None:
                        approx = None
                        break
                    approx = np.concatenate([q123, q_w_b])
                if approx is not None:
                    approx_list.append(approx)
            for approx in approx_list:
                sol = self._polish(approx, target, tol_pos, tol_rot, max_iter=20)
                if sol is None:
                    continue
                if np.any(sol < self.lower - 1e-9) or np.any(sol > self.upper + 1e-9):
                    continue
                dist = float(np.linalg.norm(sol - seed_q))
                if dist < best_dist:
                    best, best_dist = sol, dist
        return best


# ─────────────────────────────── 位姿表示（ArmPose 语义） ───────────────────────────────

PoseLike = Union[Dict[str, float], Sequence[float], Any]
_POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
_DELTA_KEYS = ('dx', 'dy', 'dz', 'droll', 'dpitch', 'dyaw')


def _pose_fields(pose: PoseLike) -> Dict[str, float]:
    """@brief 把 ArmPose 对象 / 字典 / 6 元序列统一成 {x,y,z,roll,pitch,yaw}（米 / 度）。
    @param pose 任意一种表示
    @return 字典
    @throws ValueError 无法识别
    """
    if isinstance(pose, dict):
        return {k: float(pose.get(k, 0.0)) for k in _POSE_KEYS}
    if hasattr(pose, 'x') and hasattr(pose, 'yaw'):
        return {k: float(getattr(pose, k, 0.0)) for k in _POSE_KEYS}
    seq = list(pose)
    if len(seq) != 6:
        raise ValueError(f'位姿序列需要 6 个数 (x,y,z,roll,pitch,yaw)，收到 {len(seq)} 个')
    return dict(zip(_POSE_KEYS, (float(v) for v in seq)))


def pose_to_matrix(pose: PoseLike) -> np.ndarray:
    """@brief ArmPose（米 / 度，R = Rz(yaw)·Ry(pitch)·Rx(roll)，与 motion::rpy_to_quat 一致）→ 4×4。
    @param pose ArmPose 对象 / 字典 / 6 元序列
    @return 4×4 齐次变换
    """
    p = _pose_fields(pose)
    return _make_tf((p['x'], p['y'], p['z']),
                    (math.radians(p['roll']), math.radians(p['pitch']), math.radians(p['yaw'])))


def matrix_to_pose(tf: np.ndarray) -> Dict[str, float]:
    """@brief 4×4 → ArmPose 字典（米 / 度），欧拉角提取与 motion::quat_to_rpy 同约定。
    @param tf 4×4 齐次变换
    @return {x,y,z,roll,pitch,yaw}
    """
    rot = tf[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    roll = math.atan2(rot[2, 1], rot[2, 2])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return {'x': float(tf[0, 3]), 'y': float(tf[1, 3]), 'z': float(tf[2, 3]),
            'roll': math.degrees(roll), 'pitch': math.degrees(pitch), 'yaw': math.degrees(yaw)}


def pose_from_look_at(pos: Sequence[float], look_at: Sequence[float],
                      roll_deg: float = 0.0) -> Dict[str, float]:
    """@brief 相机位置 + 目标点 → ArmPose 字典：末端 +X（光轴）指向目标，画面水平。
           与 Commander 的 aim_quat 同约定：yaw = atan2(dy, dx)，pitch = asin(−dz/n)（俯视为正）。
    @param pos      相机（gimbal_tool0）位置，m
    @param look_at  目标点，m
    @param roll_deg 横滚，默认 0（画面水平）
    @return {x,y,z,roll,pitch,yaw}
    """
    d = np.asarray(look_at, dtype=float) - np.asarray(pos, dtype=float)
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        pitch = yaw = 0.0
    else:
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -d[2] / n))))
        yaw = math.degrees(math.atan2(d[1], d[0]))
    return {'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2]),
            'roll': float(roll_deg), 'pitch': pitch, 'yaw': yaw}


# ─────────────────────────────── 可达判定 ───────────────────────────────


@dataclass
class Margins:
    """@brief 判定余量：dist_m 作用在肩→臂末端距离的两端（臂长 / 折叠边界），joint_rad 作用在各关节限位。
           joint_rad 给标量则 6 轴同值；默认 J1 留 10°（实机 J1 软限位与回绕编码器的历史问题），其余 0.1 rad。
    """
    dist_m: float = 0.05
    joint_rad: Union[float, Sequence[float]] = (0.1745, 0.10, 0.10, 0.10, 0.10, 0.10)

    def joint_array(self) -> np.ndarray:
        """@brief 6 轴余量数组。
        @return (6,) ndarray
        """
        if isinstance(self.joint_rad, (int, float)):
            return np.full(6, float(self.joint_rad))
        arr = np.asarray(self.joint_rad, dtype=float)
        if arr.shape != (6,):
            raise ValueError('joint_rad 需要 6 个值或一个标量')
        return arr


@dataclass
class ReachResult:
    """@brief check_pose 的结果。"""
    ok: bool
    reason: str = ''
    joints: Optional[np.ndarray] = None                  # 6 轴解（可达时）
    margins: Dict[str, Any] = field(default_factory=dict)  # reach_m / joints_rad（正 = 有余量）
    suggestion: Optional[Dict[str, float]] = None        # 不可达时最近的可行位姿（ArmPose 字典）


def _joint_label(i: int) -> str:
    """@brief 关节序号 → 人话名字。
    @param i 0..5
    @return 'J1'..'J6'
    """
    return f'J{i + 1}'


def _arm_tip_estimate(model: ArmModel, target: np.ndarray) -> np.ndarray:
    """@brief 逆解失败时估计臂末端位置（云台角按零位取偏移；只用于给原因文字，误差几 cm 无妨）。
    @param model  模型
    @param target 目标 4×4
    @return 臂末端位置估计（base 系）
    """
    return (target @ _inv_tf(model.wrist_tf(np.zeros(3))))[:3, 3]


def _shoulder_distance(model: ArmModel, arm_tip: np.ndarray, q1: float) -> float:
    """@brief 肩（J2 轴）到臂末端在臂平面内的距离。
    @param model   模型
    @param arm_tip 臂末端位置，base 系
    @param q1      该解的 J1 角（决定臂平面）
    @return 距离，m
    """
    pl = model.planar
    p_f1 = (_inv_tf(pl.tf_base_f1) @ np.append(arm_tip, 1.0))[:3]
    p_plane = _axis_angle_mat((0.0, 0.0, 1.0), -pl.s1 * q1) @ p_f1 - pl.shoulder
    return float(math.hypot(p_plane @ pl.u, p_plane @ pl.w))


def _diagnose(model: ArmModel, target: np.ndarray, margins: Margins) -> str:
    """@brief 逆解失败时给一句人话原因：按"离轴太近 → 方位超 J1 → 超臂长 / 太折叠 → J2/J3 超限 → 朝向"
           的顺序，在所有几何分支里挑违反最少的那支来描述。
    @param model   模型
    @param target  目标 4×4
    @param margins 余量
    @return 原因字符串
    """
    pl = model.planar
    jm = margins.joint_array()
    tip = _arm_tip_estimate(model, target)
    p_f1 = (_inv_tf(pl.tf_base_f1) @ np.append(tip, 1.0))[:3]
    rho = math.hypot(p_f1[0], p_f1[1])
    if rho * pl.n_xy_norm < abs(pl.height_n - p_f1[2] * pl.n[2]) - 1e-9:
        return f'末端离臂基座竖轴太近（{rho * 100:.0f} cm），J1 无法把臂平面对准该点'
    az_deg = math.degrees(math.atan2(tip[1], tip[0]))
    cands = pl.solve(tip)
    d_lo, d_hi = pl.dist_min + margins.dist_m, pl.dist_max - margins.dist_m
    best: Optional[List[str]] = None
    for c in cands:
        why: List[str] = []
        if not (model.lower[0] + jm[0] <= c[0] <= model.upper[0] - jm[0]):
            why.append(f'方位角 {az_deg:.0f}° 超出 J1 可用范围 '
                       f'±{math.degrees(model.upper[0] - jm[0]):.0f}°'
                       f'（限位 ±{math.degrees(model.upper[0]):.0f}° 留 {math.degrees(jm[0]):.0f}° 余量）')
        d = _shoulder_distance(model, tip, float(c[0]))
        if d > d_hi:
            why.append(f'超出臂长：肩到末端 {d:.2f} m，最大 {d_hi:.2f} m'
                       f'（含 {margins.dist_m * 100:.0f} cm 余量；J3 会到下限 {model.lower[2]:.2f}）')
        elif d < d_lo:
            why.append(f'太靠近肩部：肩到末端 {d:.2f} m，最小 {d_lo:.2f} m（臂折不进去，J3 会到上限）')
        for i in (1, 2):
            lo, hi = model.lower[i] + jm[i], model.upper[i] - jm[i]
            if not (lo <= c[i] <= hi):
                why.append(f'{_joint_label(i)} 需要 {c[i]:.2f} rad，超出可用范围 [{lo:.2f}, {hi:.2f}]')
        if best is None or len(why) < len(best):
            best = why
    if best is None:
        return f'超出臂长：肩到末端距离超过最大 {d_hi:.2f} m'
    if best:
        return best[0]
    return '位置可达，但云台在此位形做不出该朝向（J4/J5/J6 会超限）'


def _suggest(model: ArmModel, target: np.ndarray, margins: Margins,
             seed: Optional[Sequence[float]]) -> Optional[Dict[str, float]]:
    """@brief 不可达时给最近的可行位姿：保持朝向，把臂末端位置的方位角夹进 J1 范围、
           肩距夹进 [最小+余量, 最大−余量]，再按关节余量夹 J2/J3，重算位置后复核一次。
    @param model   模型
    @param target  原目标 4×4
    @param margins 余量
    @param seed    逆解种子
    @return 可行的 ArmPose 字典，找不到返回 None
    """
    pl = model.planar
    jm = margins.joint_array()
    tip = _arm_tip_estimate(model, target)
    cands = pl.solve(tip)
    if not cands:
        return None
    lo = model.lower[:3] + jm[:3]
    hi = model.upper[:3] - jm[:3]
    best_pose: Optional[Dict[str, float]] = None
    best_shift = math.inf
    for c in cands:
        q123 = np.clip(c, lo, hi)
        d = _shoulder_distance(model, tip, float(q123[0]))
        d_target = min(max(d, pl.dist_min + margins.dist_m + 0.01),
                       pl.dist_max - margins.dist_m - 0.01)
        if abs(d_target - d) > 1e-9:
            # 沿肩→末端方向缩放到目标肩距，再解一次拿 q2/q3
            p_f1 = (_inv_tf(pl.tf_base_f1) @ np.append(tip, 1.0))[:3]
            rot = _axis_angle_mat((0.0, 0.0, 1.0), -pl.s1 * q123[0])
            p_plane = rot @ p_f1 - pl.shoulder
            p_plane = p_plane * (d_target / max(d, 1e-9))
            p_f1_new = rot.T @ (p_plane + pl.shoulder)
            tip_new = (pl.tf_base_f1 @ np.append(p_f1_new, 1.0))[:3]
            sub = pl.solve_with_q1(tip_new, float(q123[0]))
            if not sub:
                continue
            q123 = np.clip(min(sub, key=lambda s: float(np.linalg.norm(s - c))), lo, hi)
        new_tip = pl.tip_position(q123)
        shifted = target.copy()
        shifted[:3, 3] += new_tip - tip
        res = check_pose(model, matrix_to_pose(shifted), margins, seed, _suggest_depth=1)
        if res.ok:
            shift = float(np.linalg.norm(new_tip - tip))
            if shift < best_shift:
                best_pose, best_shift = matrix_to_pose(shifted), shift
    return best_pose


def check_pose(model: ArmModel, pose: PoseLike, margins: Optional[Margins] = None,
               seed: Optional[Sequence[float]] = None, continuous: bool = False,
               _suggest_depth: int = 0) -> ReachResult:
    """@brief 单点可达判定：位姿（ArmPose 语义，机械臂 base_link 系，末端 gimbal_tool0）能否在余量内到达。
           可达 → joints 给 6 轴解、margins 给余量；不可达 → reason 给人话原因、suggestion 给最近可行位姿。
    @param model      ArmModel
    @param pose       ArmPose 对象 / 字典 / 6 元序列（米 / 度）
    @param margins    余量，None 取默认 Margins()
    @param seed       逆解种子（当前关节角），影响多解取舍
    @param continuous True = 只承认从 seed 连续运动（不换几何分支）能到的解——运动过程中的路点校验用这个；
                      False = 任何限位内的解都算（"这个位姿臂能不能摆出来"）
    @param _suggest_depth 内部递归深度（求 suggestion 时不再套娃）
    @return ReachResult
    """
    margins = margins or Margins()
    jm = margins.joint_array()
    target = pose_to_matrix(pose)
    if continuous:
        if seed is None:
            raise ValueError('continuous=True 需要给 seed（当前关节角）')
        sol = model._polish(np.asarray(seed, dtype=float), target, 1e-7, 1e-7, max_iter=15)
        if sol is not None and (np.any(sol < model.lower - 1e-9)
                                or np.any(sol > model.upper + 1e-9)):
            sol = None
    else:
        sol = model.ik(target, seed)
    if sol is None:
        reason = _diagnose(model, target, margins)
        suggestion = _suggest(model, target, margins, seed) if _suggest_depth == 0 else None
        return ReachResult(False, reason, None, {}, suggestion)
    joints_rad = np.minimum(sol - model.lower, model.upper - sol) - jm
    d = _shoulder_distance(model, model.fk(sol, model.arm_tip_link)[:3, 3], float(sol[0]))
    reach_m = min(d - model.planar.dist_min, model.planar.dist_max - d) - margins.dist_m
    result_margins = {'reach_m': float(reach_m), 'joints_rad': joints_rad, 'shoulder_dist_m': d}
    problems: List[str] = []
    if reach_m < 0:
        edge = '臂长上限' if model.planar.dist_max - d < d - model.planar.dist_min else '折叠下限'
        problems.append(f'肩到末端 {d:.2f} m 距{edge}只剩 {reach_m + margins.dist_m:.2f} m'
                        f'（臂长余量要求 {margins.dist_m:.2f} m）')
    for i in np.argsort(joints_rad):
        if joints_rad[i] < 0:
            problems.append(f'{_joint_label(int(i))} = {sol[i]:.2f} rad 距限位只剩 '
                            f'{joints_rad[i] + jm[i]:.2f} rad（余量要求 {jm[i]:.2f}）')
    if problems:
        suggestion = _suggest(model, target, margins, seed) if _suggest_depth == 0 else None
        return ReachResult(False, '；'.join(problems), sol, result_margins, suggestion)
    return ReachResult(True, '', sol, result_margins, None)


# ─────────────────────────────── 球面环绕几何（与 motion/geometry.hpp 一致） ───────────────────────────────


def theta_ref(ox: float, oy: float) -> float:
    """@brief 球坐标 θ=0 的参考方位：球心指向 base 原点的方向（"近侧"），与 Commander 的 theta_ref 一致。
    @param ox 球心 x
    @param oy 球心 y
    @return 参考方位角，rad
    """
    return math.atan2(-oy, -ox)


def sphere_to_cart(theta_rad: float, phi_rad: float, r: float,
                   center: Sequence[float]) -> np.ndarray:
    """@brief Z-up 球坐标 → base 系位置（θ 相对近侧参考方向，φ 为仰角）。
    @param theta_rad 方位角，rad
    @param phi_rad   仰角，rad
    @param r         半径，m
    @param center    球心
    @return (3,) 位置
    """
    tw = theta_ref(center[0], center[1]) + theta_rad
    cp = math.cos(phi_rad)
    return np.array([center[0] + r * cp * math.cos(tw), center[1] + r * cp * math.sin(tw),
                     center[2] + r * math.sin(phi_rad)])


def cart_to_sphere(pos: Sequence[float], center: Sequence[float]) -> tuple:
    """@brief base 系位置 → (θ, φ, r)，sphere_to_cart 的逆；θ 归一化到 (−π, π]。
    @param pos    点
    @param center 球心
    @return (theta_rad, phi_rad, r_m)
    """
    d = np.asarray(pos, dtype=float) - np.asarray(center, dtype=float)
    r = float(np.linalg.norm(d))
    if r < 1e-9:
        return 0.0, 0.0, 0.0
    phi = math.asin(max(-1.0, min(1.0, d[2] / r)))
    theta = _wrap_pi(math.atan2(d[1], d[0]) - theta_ref(center[0], center[1]))
    return theta, phi, r


def arc_pose(pose0: PoseLike, center: Sequence[float], daz_deg: float) -> Dict[str, float]:
    """@brief 从当前位姿出发、绕球心等半径等仰角转 daz 度后的位姿（光轴指向球心、画面水平），
           即 arc_around 一步的目标位姿。daz=0 表示"原地对准球心"。
    @param pose0   当前位姿（ArmPose 语义）
    @param center  球心
    @param daz_deg 方位增量，度（按 Commander 球坐标方向）
    @return ArmPose 字典
    """
    p0 = _pose_fields(pose0)
    theta, phi, r = cart_to_sphere((p0['x'], p0['y'], p0['z']), center)
    pos = sphere_to_cart(theta + math.radians(daz_deg), phi, r, center)
    return pose_from_look_at(pos, center)


# ─────────────────────────────── 余量盒子（headroom） ───────────────────────────────


def _feasible_from_seed(model: ArmModel, target: np.ndarray, seed: np.ndarray,
                        margins: Margins) -> tuple:
    """@brief 连续可达判定：只从 seed 出发做数值精修（不换几何分支），再查关节余量与臂长余量。
    @param model   模型
    @param target  目标 4×4
    @param seed    起点关节角
    @param margins 余量
    @return (ok, joints)；精修不收敛时 joints 为 None
    """
    sol = model._polish(np.asarray(seed, dtype=float), target, 1e-7, 1e-7, max_iter=15)
    if sol is None:
        return False, None
    jm = margins.joint_array()
    if np.any(sol < model.lower + jm - 1e-12) or np.any(sol > model.upper + 1e-12 - jm):
        return False, sol
    d = _shoulder_distance(model, model.fk(sol, model.arm_tip_link)[:3, 3], float(sol[0]))
    if d < model.planar.dist_min + margins.dist_m or d > model.planar.dist_max - margins.dist_m:
        return False, sol
    return True, sol


def _march(model: ArmModel, pose_fn, seed: np.ndarray, margins: Margins,
           step: float, max_t: float, refine: int) -> float:
    """@brief 沿参数 t 射线步进求最远可达 t：粗步进到第一个不可达点，再二分 refine 次。
    @param model   模型
    @param pose_fn t → ArmPose 字典
    @param seed    起点关节角
    @param margins 余量
    @param step    粗步长
    @param max_t   最大 t
    @param refine  二分次数
    @return 最远可达 t（≥ 0；起点本身不可达返回 0）
    """
    ok, sol = _feasible_from_seed(model, pose_to_matrix(pose_fn(0.0)), seed, margins)
    if not ok:
        return 0.0
    t_ok, t_bad = 0.0, None
    while t_ok + step <= max_t + 1e-9:
        ok, s = _feasible_from_seed(model, pose_to_matrix(pose_fn(t_ok + step)), sol, margins)
        if not ok:
            t_bad = t_ok + step
            break
        t_ok, sol = t_ok + step, s
    if t_bad is None:
        if max_t - t_ok < 1e-9:
            return t_ok
        # 整步走不到 max_t：补试 max_t 本身，不行再在 (t_ok, max_t) 里二分
        ok, s = _feasible_from_seed(model, pose_to_matrix(pose_fn(max_t)), sol, margins)
        if ok:
            return max_t
        t_bad = max_t
    for _ in range(refine):
        mid = 0.5 * (t_ok + t_bad)
        ok, s = _feasible_from_seed(model, pose_to_matrix(pose_fn(mid)), sol, margins)
        if ok:
            t_ok, sol = mid, s
        else:
            t_bad = mid
    return t_ok


def _shifted(pose0: Dict[str, float], key: str, delta: float) -> Dict[str, float]:
    """@brief 复制位姿并给某一字段加增量。
    @param pose0 位姿字典
    @param key   字段名
    @param delta 增量
    @return 新字典
    """
    p = dict(pose0)
    p[key] = p[key] + delta
    return p


def headroom(model: ArmModel, joints: Sequence[float], subject: Optional[Sequence[float]] = None,
             margins: Optional[Margins] = None, step_m: float = 0.02, step_deg: float = 5.0,
             refine: int = 4) -> Dict[str, Any]:
    """@brief 余量盒子：从当前关节角出发，各方向"连续运动还能走多远"的一维区间（给大模型只在区间里选数）。
           平移沿 base 系 x/y/z（对应 dolly / truck / crane，姿态不变）；转动为 ArmPose 的 yaw / pitch 增量
           （位置不变）；给了 subject 再加绕主体等半径等仰角环绕的方位增量 arc_az（光轴始终指向主体）。
           全部按"不换几何分支"的连续语义计算，且含余量（默认 Margins）。
    @param model    模型
    @param joints   当前 6 轴关节角
    @param subject  环绕主体位置（base 系），None 不算环绕
    @param margins  余量，None 取默认
    @param step_m   平移粗步长，m
    @param step_deg 转动粗步长，度
    @param refine   二分细化次数（2 cm / 2^4 ≈ 1 mm；5° / 2^4 ≈ 0.3°）
    @return 字典：dolly/truck/crane (lo, hi) m；dyaw/dpitch (lo, hi) 度；yaw_abs/pitch_abs 绝对区间；
            arc_az (lo, hi) 度与 arc {center, radius_m, elevation_deg, az0_deg}（有 subject 时）；
            pose 当前位姿；joints 当前关节角
    """
    margins = margins or Margins()
    q0 = np.asarray(joints, dtype=float)
    pose0 = matrix_to_pose(model.fk(q0))
    out: Dict[str, Any] = {'pose': pose0, 'joints': [float(v) for v in q0]}
    for key, axis in (('dolly', 'x'), ('truck', 'y'), ('crane', 'z')):
        neg = _march(model, lambda t, a=axis: _shifted(pose0, a, -t), q0, margins, step_m, 1.5,
                     refine)
        pos = _march(model, lambda t, a=axis: _shifted(pose0, a, t), q0, margins, step_m, 1.5,
                     refine)
        out[key] = (-neg, pos)
    neg = _march(model, lambda t: _shifted(pose0, 'yaw', -t), q0, margins, step_deg, 180.0, refine)
    pos = _march(model, lambda t: _shifted(pose0, 'yaw', t), q0, margins, step_deg, 180.0, refine)
    out['dyaw'] = (-neg, pos)
    # pitch 绝对值限制在 ±89.9°：欧拉角在 ±90° 退化（万向锁），不是臂的限制
    neg = _march(model, lambda t: _shifted(pose0, 'pitch', -t), q0, margins, step_deg,
                 max(0.0, pose0['pitch'] + 89.9), refine)
    pos = _march(model, lambda t: _shifted(pose0, 'pitch', t), q0, margins, step_deg,
                 max(0.0, 89.9 - pose0['pitch']), refine)
    out['dpitch'] = (-neg, pos)
    out['yaw_abs'] = (pose0['yaw'] + out['dyaw'][0], pose0['yaw'] + out['dyaw'][1])
    out['pitch_abs'] = (pose0['pitch'] + out['dpitch'][0], pose0['pitch'] + out['dpitch'][1])
    if subject is not None:
        center = [float(v) for v in subject]
        theta, phi, r = cart_to_sphere((pose0['x'], pose0['y'], pose0['z']), center)
        neg = _march(model, lambda t: arc_pose(pose0, center, -t), q0, margins, step_deg, 180.0,
                     refine)
        pos = _march(model, lambda t: arc_pose(pose0, center, t), q0, margins, step_deg, 180.0,
                     refine)
        out['arc_az'] = (-neg, pos)
        out['arc'] = {'center': center, 'radius_m': r, 'elevation_deg': math.degrees(phi),
                      'az0_deg': math.degrees(theta)}
    return out


# ─────────────────────────────── 步骤表预检（check_plan） ───────────────────────────────

# 不改变臂位姿、或本模块无法推演（预设位形 / 云台直连 / 速度流）的 op：标 checked=False 直接放过
_UNCHECKED_OPS = {'enable', 'disable', 'homing', 'reset_error', 'stop', 'mode', 'wait',
                  'wait_camera_ready', 'track_start', 'track_stop', 'jog', 'jog_joint',
                  'stow', 'observe'}
_UNCHECKED_NOTE = {'stow': '未推演：收纳位由 Commander 常量决定，后续步骤按当前位姿继续推演',
                   'observe': '未推演：观察位由 Commander 常量决定，后续步骤按当前位姿继续推演',
                   'jog': '未推演：速度流的终点取决于实际执行时长',
                   'jog_joint': '未推演：速度流的终点取决于实际执行时长'}


def _jlist(q: Sequence[float]) -> List[float]:
    """@brief 关节角 → 普通 float 列表（进 dataclass / JSON）。
    @param q 关节角
    @return list
    """
    return [float(v) for v in q]


def _rel_target(pose0: Dict[str, float], p: Dict[str, Any]) -> Dict[str, float]:
    """@brief 当前位姿 + (dx, dy, dz, droll, dpitch, dyaw) 增量 → 目标位姿字典。
    @param pose0 当前位姿
    @param p     步骤参数（缺省增量为 0）
    @return 目标位姿
    """
    return {k: pose0[k] + float(p.get(dk, 0.0)) for k, dk in zip(_POSE_KEYS, _DELTA_KEYS)}


@dataclass
class StepReport:
    """@brief 一条步骤的推演结果。fraction = 失败 / 夹取时已走完的比例（0~1），checked=False 表示未推演。"""
    index: int
    op: str
    checked: bool
    ok: bool
    reason: str = ''
    fraction: float = 1.0
    waypoint: Optional[Dict[str, float]] = None      # 失败 / 夹取 / 投影处的位姿
    clipped: Optional[Dict[str, Any]] = None         # 被改写的参数（clip 夹取 / project 投影）
    adjust_kind: Optional[str] = None                # 'clip'（夹到边界）/ 'project'（投影到最近可行）
    suggestion: Optional[Dict[str, float]] = None    # 整点类 op 失败时的最近可行位姿
    joints_end: Optional[List[float]] = None         # 步骤结束时的关节角

    def line(self) -> str:
        """@brief 一行人话（给大模型回喂 / 打日志）。
        @return 字符串
        """
        head = f'步骤 {self.index} {self.op}: '
        if not self.checked:
            return head + '未推演' + (f'（{self.reason}）' if self.reason else '')
        if self.ok and self.clipped:
            return head + '⚠ ' + self.reason
        if self.ok:
            return head + '✓'
        if 0.0 < self.fraction < 1.0:
            where = f'走到 {self.fraction * 100:.0f}% 处'
        else:
            where = '起点' if self.fraction == 0.0 else '终点'
        return head + f'✗ {where}不可达：{self.reason}'


@dataclass
class PlanReport:
    """@brief 整张步骤表的推演结果。"""
    ok: bool
    steps: List[StepReport]
    pose_start: Dict[str, float]
    pose_end: Dict[str, float]
    joints_end: np.ndarray

    def summary(self) -> str:
        """@brief 多行人话汇总（每步一行 + 结论），可直接拼进大模型的回喂提示。
        @return 字符串
        """
        lines = [s.line() for s in self.steps]
        lines.append('结论：' + ('整表可执行' if self.ok else '存在不可达步骤，需要重规划'))
        return '\n'.join(lines)


def _lerp_pose(a: Dict[str, float], b: Dict[str, float], f: float) -> Dict[str, float]:
    """@brief 位姿字典按比例线性插值（角度按最短弧）。
    @param a 起点
    @param b 终点
    @param f 0~1
    @return 插值位姿
    """
    out = {}
    for k in ('x', 'y', 'z'):
        out[k] = a[k] + (b[k] - a[k]) * f
    for k in ('roll', 'pitch', 'yaw'):
        d = math.degrees(_wrap_pi(math.radians(b[k] - a[k])))
        out[k] = a[k] + d * f
    return out


def _pose_gap(a: Dict[str, float], b: Dict[str, float]) -> tuple:
    """@brief 两位姿的位置距离（m）与最大角度变化（度）。
    @param a 起点
    @param b 终点
    @return (dist_m, max_deg)
    """
    dist = math.sqrt(sum((a[k] - b[k]) ** 2 for k in ('x', 'y', 'z')))
    ang = max(abs(math.degrees(_wrap_pi(math.radians(b[k] - a[k]))))
              for k in ('roll', 'pitch', 'yaw'))
    return dist, ang


def _sphere_pose(center: Sequence[float], az_deg: float, el_deg: float,
                 r: float) -> Dict[str, float]:
    """@brief Commander 球坐标 (θ, φ, r) 上的一点、光轴指向球心的位姿。
    @param center 球心
    @param az_deg 方位，度
    @param el_deg 仰角，度
    @param r      半径，m
    @return ArmPose 字典
    """
    pos = sphere_to_cart(math.radians(az_deg), math.radians(el_deg), r, center)
    return pose_from_look_at(pos, center)


def _walk_path(model: ArmModel, path_fn, q0: np.ndarray, margins: Margins, n: int) -> tuple:
    """@brief 沿路径 f∈(0,1] 采样 n 段做连续可达推演。
    @param model   模型
    @param path_fn f → ArmPose 字典
    @param q0      起点关节角（已在 path_fn(0) 处）
    @param margins 余量
    @param n       采样段数
    @return (f_ok, q_ok, f_bad)：f_bad=None 表示全程可达；否则 (f_ok, f_bad) 之间有边界（已二分 3 次）
    """
    f_ok, q_ok, f_bad = 0.0, np.asarray(q0, dtype=float), None
    for k in range(1, n + 1):
        f = k / n
        ok, s = _feasible_from_seed(model, pose_to_matrix(path_fn(f)), q_ok, margins)
        if not ok:
            f_bad = f
            break
        f_ok, q_ok = f, s
    if f_bad is not None:
        for _ in range(3):
            mid = 0.5 * (f_ok + f_bad)
            ok, s = _feasible_from_seed(model, pose_to_matrix(path_fn(mid)), q_ok, margins)
            if ok:
                f_ok, q_ok = mid, s
            else:
                f_bad = mid
    return f_ok, q_ok, f_bad


def _fail_reason(model: ArmModel, pose: Dict[str, float], q_seed: np.ndarray,
                 margins: Margins) -> str:
    """@brief 某个路点连续不可达时的人话原因（复用 check_pose 的诊断）。
    @param model   模型
    @param pose    路点
    @param q_seed  最近一个可达点的关节角
    @param margins 余量
    @return 原因
    """
    res = check_pose(model, pose, margins, seed=q_seed, continuous=True, _suggest_depth=1)
    return res.reason or '连续运动到该点无解'


def _simulate_step(model: ArmModel, index: int, step: Dict[str, Any], q: np.ndarray,
                   pose: Dict[str, float], margins: Margins, mode: str,
                   sample_m: float, sample_deg: float) -> tuple:
    """@brief 推演一条步骤。
    @param model      模型
    @param index      步骤序号（1 起）
    @param step       步骤字典（含 op）
    @param q          当前关节角
    @param pose       当前位姿
    @param margins    余量
    @param mode       'reject' / 'clip' / 'project'
    @param sample_m   笛卡尔路径位置采样步长，m
    @param sample_deg 角度采样步长，度
    @return (StepReport, q_new, pose_new)
    """
    p = dict(step)
    op = str(p.pop('op', '')).strip().lower()
    rts = bool(p.get('return_to_start', False))
    q0, pose0 = np.asarray(q, dtype=float), dict(pose)

    if op in _UNCHECKED_OPS or op.startswith('gimbal_'):
        note = _UNCHECKED_NOTE.get(op, '未推演：云台直连不经臂 IK，臂位姿不变' if op.startswith('gimbal_') else '')
        return StepReport(index, op, False, True, note, joints_end=_jlist(q0)), q0, pose0

    # ── 关节空间点到点：直接查关节余量 ──
    if op == 'joint':
        target = np.array([float(p['j1']), float(p['j2']), float(p['j3'])])
        q_new = q0.copy()
        q_new[:3] = q0[:3] + target if bool(p.get('relative', False)) else target
        jm = margins.joint_array()
        lo3, hi3 = model.lower[:3] + jm[:3], model.upper[:3] - jm[:3]
        bad = [f'{_joint_label(i)} = {q_new[i]:.2f} rad 超出可用范围 [{lo3[i]:.2f}, {hi3[i]:.2f}]'
               for i in range(3) if not (lo3[i] <= q_new[i] <= hi3[i])]
        if bad and mode == 'project':
            q_new[:3] = np.clip(q_new[:3], lo3, hi3)
            clipped = {f'j{i + 1}': round(float(q_new[i]), 4) for i in range(3)}
            if bool(p.get('relative', False)):
                clipped = {f'j{i + 1}': round(float(q_new[i] - q0[i]), 4) for i in range(3)}
            wp = matrix_to_pose(model.fk(q_new))
            return StepReport(index, op, True, True, '夹到关节可用范围：' + '；'.join(bad), 1.0, wp,
                              clipped=clipped, adjust_kind='clip',
                              joints_end=_jlist(q_new)), q_new, wp
        wp = matrix_to_pose(model.fk(q_new))
        if bad:
            return StepReport(index, op, True, False, '；'.join(bad), 1.0, wp,
                              joints_end=_jlist(q0)), q0, pose0
        return StepReport(index, op, True, True, '', 1.0, wp, joints_end=_jlist(q_new)), q_new, wp

    # ── 整点类：pose / move_rel（Commander 走 IK 点到点，允许换分支） ──
    if op in ('pose', 'move_rel'):
        if op == 'pose':
            target = {k: float(p.get(k, pose0[k])) for k in _POSE_KEYS}
        else:
            target = _rel_target(pose0, p)
        res = check_pose(model, target, margins, seed=q0)
        if not res.ok and mode == 'project' and res.suggestion is not None:
            moved = math.sqrt(sum((res.suggestion[k] - target[k]) ** 2 for k in ('x', 'y', 'z')))
            sub = check_pose(model, res.suggestion, margins, seed=q0)
            if sub.ok:
                if op == 'pose':
                    clipped = {k: round(res.suggestion[k], 4) for k in _POSE_KEYS}
                else:
                    clipped = {dk: round(res.suggestion[k] - pose0[k], 4)
                               for k, dk in zip(_POSE_KEYS, _DELTA_KEYS) if dk in p}
                reason = f'原目标不可达（{res.reason}），已投影到最近可行位姿（移动了 {moved:.2f} m）'
                rep = StepReport(index, op, True, True, reason, 1.0, res.suggestion,
                                 clipped=clipped, adjust_kind='project',
                                 joints_end=_jlist(q0 if rts else sub.joints))
                return (rep, q0, pose0) if rts else (rep, sub.joints, res.suggestion)
        if not res.ok:
            return StepReport(index, op, True, False, res.reason, 1.0, target,
                              suggestion=res.suggestion, joints_end=_jlist(q0)), q0, pose0
        q_new = res.joints
        if rts:
            return (StepReport(index, op, True, True, '', 1.0, target, joints_end=_jlist(q0)),
                    q0, pose0)
        return (StepReport(index, op, True, True, '', 1.0, target, joints_end=_jlist(q_new)),
                q_new, target)

    # ── 笛卡尔路径类：先（可选）到起点，再沿路径连续推演 ──
    path_fn = None
    clip_fn = None
    q_start, pose_start = q0, pose0
    n = 1
    if op in ('dolly', 'truck', 'crane'):
        axis = {'dolly': 'x', 'truck': 'y', 'crane': 'z'}[op]
        dist = float(p['distance_m'])
        path_fn = lambda f, a=axis, d=dist: _shifted(pose0, a, d * f)
        clip_fn = lambda f, d=dist: {'distance_m': round(d * f, 4)}
        n = max(1, int(math.ceil(abs(dist) / sample_m)))
    elif op == 'linear':
        if 'start' in p:
            start = {k: float(p['start'].get(k, pose0[k])) for k in _POSE_KEYS}
            end = {k: float(p['end'].get(k, start[k])) for k in _POSE_KEYS}
            res = check_pose(model, start, margins, seed=q0)
            if not res.ok:
                return (StepReport(index, op, True, False, '起点 ' + res.reason, 0.0, start,
                                   suggestion=res.suggestion, joints_end=_jlist(q0)), q0, pose0)
            q_start, pose_start = res.joints, start
            clip_fn = lambda f, s=start, e=end: {  # noqa: E731
                'end': {k: round(v, 4) for k, v in _lerp_pose(s, e, f).items()}}
        else:
            end = _rel_target(pose0, p)
            clip_fn = lambda f, pp=p: {k: round(float(pp[k]) * f, 4)  # noqa: E731
                                      for k in _DELTA_KEYS if k in pp}
        path_fn = lambda f, s=pose_start, e=end: _lerp_pose(s, e, f)
        dist, ang = _pose_gap(pose_start, end)
        n = max(1, int(math.ceil(dist / sample_m)), int(math.ceil(ang / sample_deg)))
    elif op in ('arc', 'orbit'):
        center = [float(v) for v in p['center']]
        az0, az1 = float(p['az_start_deg']), float(p['az_end_deg'])
        if op == 'arc' or 'radius_m' in p:
            el0 = el1 = float(p.get('elevation_deg', 0.0))
            r0 = r1 = float(p['radius_m'])
        else:
            el0, el1 = float(p.get('el_start_deg', 0.0)), float(p.get('el_end_deg', 0.0))
            r0 = float(p['r_start_m'])
            r1 = float(p.get('r_end_m', r0))
        start = _sphere_pose(center, az0, el0, r0)
        res = check_pose(model, start, margins, seed=q0)
        if not res.ok:
            return (StepReport(index, op, True, False, '起拍点 ' + res.reason, 0.0, start,
                               suggestion=res.suggestion, joints_end=_jlist(q0)), q0, pose0)
        q_start, pose_start = res.joints, start
        path_fn = lambda f, c=center: _sphere_pose(  # noqa: E731
            c, az0 + (az1 - az0) * f, el0 + (el1 - el0) * f, r0 + (r1 - r0) * f)
        if op == 'arc' or 'radius_m' in p:
            clip_fn = lambda f: {'az_end_deg': round(az0 + (az1 - az0) * f, 2)}
        else:
            clip_fn = lambda f: {'az_end_deg': round(az0 + (az1 - az0) * f, 2),
                                 'el_end_deg': round(el0 + (el1 - el0) * f, 2),
                                 'r_end_m': round(r0 + (r1 - r0) * f, 4)}
        n = max(1, int(math.ceil(max(abs(az1 - az0), abs(el1 - el0)) / sample_deg)),
                int(math.ceil(abs(r1 - r0) / sample_m)))
    else:
        return (StepReport(index, op, False, False, f'未知 op: {op}', joints_end=_jlist(q0)),
                q0, pose0)

    f_ok, q_ok, f_bad = _walk_path(model, path_fn, q_start, margins, n)
    end_pose = path_fn(f_ok)
    if f_bad is None:
        rep = StepReport(index, op, True, True, '', 1.0, end_pose, joints_end=_jlist(q_ok))
    else:
        bad_pose = path_fn(f_bad)
        reason = _fail_reason(model, bad_pose, q_ok, margins)
        if mode in ('clip', 'project') and f_ok > 0.0:
            clipped = clip_fn(f_ok)
            rep = StepReport(index, op, True, True,
                             f'夹取到 {f_ok * 100:.0f}%（{clipped}）：再往前 {reason}', f_ok, end_pose,
                             clipped=clipped, adjust_kind='clip', joints_end=_jlist(q_ok))
        else:
            return StepReport(index, op, True, False, reason, f_ok, bad_pose,
                              joints_end=_jlist(q0)), q0, pose0
    if rts:
        rep.joints_end = _jlist(q0)
        return rep, q0, pose0
    return rep, q_ok, end_pose


def check_plan(model: ArmModel, steps: Sequence[Dict[str, Any]], start_joints: Sequence[float],
               margins: Optional[Margins] = None, mode: str = 'reject',
               sample_m: float = 0.02, sample_deg: float = 5.0) -> PlanReport:
    """@brief 步骤表预检：把大模型输出的 JSON 步骤表从起始关节角逐条推演。
           笛卡尔路径类（dolly / truck / crane / linear / arc / orbit）按步长采样、从当前位形连续推演；
           整点类（pose / move_rel）按 Commander 的 IK 点到点语义判定；joint 直接查关节余量；
           enable / wait / gimbal_* / stow / observe 等不推演。
    @param model        模型
    @param steps        [{'op': ..., ...}, ...]
    @param start_joints 起始 6 轴关节角
    @param margins      余量，None 取默认
    @param mode         'reject'：第一条不可达就停；'clip'：路径类夹到可达边界继续、整点类仍拒绝；
                        'project'：在 clip 基础上，整点类（pose / move_rel）投影到最近可行位姿、
                        joint 夹进关节可用范围——即"能做多少做多少"，几乎不拒绝
    @param sample_m     位置采样步长，m
    @param sample_deg   角度采样步长，度
    @return PlanReport（ok / 每步 StepReport / 起止位姿 / 终止关节角；summary() 给人话汇总）
    """
    if mode not in ('reject', 'clip', 'project'):
        raise ValueError("mode 只能是 'reject' / 'clip' / 'project'")
    margins = margins or Margins()
    q = np.asarray(start_joints, dtype=float)
    pose = matrix_to_pose(model.fk(q))
    pose_start = dict(pose)
    reports: List[StepReport] = []
    ok_all = True
    for index, step in enumerate(steps, start=1):
        try:
            rep, q, pose = _simulate_step(model, index, step, q, pose, margins, mode,
                                          sample_m, sample_deg)
        except (KeyError, ValueError, TypeError) as exc:
            rep = StepReport(index, str(step.get('op', '')), True, False, f'参数错误: {exc!r}',
                             joints_end=_jlist(q))
        reports.append(rep)
        if not rep.ok:
            ok_all = False
            break
    return PlanReport(ok_all, reports, pose_start, pose, q)


# ─────────────────────────────── 能力卡（capability card） ───────────────────────────────


def _tip_feasible(model: ArmModel, p_base: Sequence[float], margins: Margins) -> bool:
    """@brief 臂末端（tool0 / 云台法兰基座）位置在余量内是否可达（只看 J1-3 与臂长，不看朝向）。
    @param model   模型
    @param p_base  位置，base 系
    @param margins 余量
    @return bool
    """
    jm = margins.joint_array()
    pl = model.planar
    p_f1 = (_inv_tf(pl.tf_base_f1) @ np.append(np.asarray(p_base, dtype=float), 1.0))[:3]
    rho = math.hypot(p_f1[0], p_f1[1])
    # 离 J1 轴太近（落进半径 |H| 的盲柱）：几何上无解；solve() 为了给数值初值会把它夹到切点，这里得自己拦
    if rho * pl.n_xy_norm < abs(pl.height_n - p_f1[2] * pl.n[2]) - 1e-9:
        return False
    for c in pl.solve(p_base):
        if np.any(c < model.lower[:3] + jm[:3]) or np.any(c > model.upper[:3] - jm[:3]):
            continue
        d = _shoulder_distance(model, np.asarray(p_base, dtype=float), float(c[0]))
        if pl.dist_min + margins.dist_m <= d <= pl.dist_max - margins.dist_m:
            return True
    return False


def _h_bands_at(model: ArmModel, r: float, margins: Margins, base_height_m: float,
                z_step: float = 0.005) -> List[tuple]:
    """@brief 前方水平距离 r 处（正前方、臂平面内）臂末端可达的离地高**分段**。
           r 小时（≲0.20 m）可达高度不是一段：肩部周围 dist_min+余量 的球内够不到，被分成上下两段。
    @param model         模型
    @param r             到臂基座竖轴的水平距离，m
    @param margins       余量
    @param base_height_m 臂基座离地高
    @param z_step        z 扫描步长
    @return [(h_lo, h_hi), ...] 按高度升序，可能为空
    """
    bands: List[tuple] = []
    start = prev = None
    for z in np.arange(-0.6, 1.2, z_step):
        ok = _tip_feasible(model, (r, 0.0, float(z)), margins)
        if ok and start is None:
            start = float(z)
        if not ok and start is not None:
            bands.append((start + base_height_m, prev + base_height_m))
            start = None
        if ok:
            prev = float(z)
    if start is not None:
        bands.append((start + base_height_m, prev + base_height_m))
    return bands


def _h_range_at(model: ArmModel, r: float, margins: Margins, base_height_m: float,
                z_step: float = 0.005) -> Optional[tuple]:
    """@brief 前方水平距离 r 处臂末端可达的离地高**主段**（分段里最长的一段；只报 min/max 会把中间的洞掩掉）。
    @param model         模型
    @param r             到臂基座竖轴的水平距离，m
    @param margins       余量
    @param base_height_m 臂基座离地高
    @param z_step        z 扫描步长
    @return (h_min, h_max) 或 None（该 r 全高不可达）
    """
    bands = _h_bands_at(model, r, margins, base_height_m, z_step)
    if not bands:
        return None
    return max(bands, key=lambda b: b[1] - b[0])


def capability_data(model: ArmModel, margins: Optional[Margins] = None,
                    base_height_m: float = 0.31,
                    r_grid: Sequence[float] = (0.15, 0.25, 0.30, 0.40, 0.50, 0.55)
                    ) -> Dict[str, Any]:
    """@brief 能力卡的结构化数据（全部从模型现算）：r→离地高表、最大前伸、J1 可用范围、舒适区、朝向规则。
    @param model         模型
    @param margins       余量，None 取默认
    @param base_height_m 臂基座（arm_base_link）离地高，默认 0.31（底盘顶面 0.236 + 轮半径 0.07）
    @param r_grid        表格的 r 网格
    @return 字典：table [{r, h_min, h_max}]、r_max、j1_deg、comfort {r:(lo,hi), h:(lo,hi)}、
            orientation {yaw_rel_deg, pitch_deg:(lo,hi), at_pose}、links、margins、base_height_m
    """
    margins = margins or Margins()
    jm = margins.joint_array()
    pl = model.planar
    table = []
    for r in r_grid:
        bands = _h_bands_at(model, float(r), margins, base_height_m)
        main = max(bands, key=lambda b: b[1] - b[0]) if bands else None
        table.append({'r': float(r), 'h_min': main[0] if main else None,
                      'h_max': main[1] if main else None, 'reachable': main is not None,
                      'bands': [(round(b[0], 3), round(b[1], 3)) for b in bands]})
    r_max = 0.0
    for r in np.arange(0.05, 1.0, 0.01):
        if _h_range_at(model, float(r), margins, base_height_m, z_step=0.02) is not None:
            r_max = float(r)
    # 舒适区：余量放大 2 倍（离限位 / 臂长边界都远）后，r ∈ [0.25, 0.45] 各处高度范围的交集
    wide = Margins(dist_m=margins.dist_m * 2.0, joint_rad=jm * 2.0)
    comfort_ranges = [rng for rng in (_h_range_at(model, r, wide, base_height_m, z_step=0.01)
                                      for r in (0.25, 0.30, 0.35, 0.40, 0.45)) if rng]
    if len(comfort_ranges) == 5:
        h_lo = max(rng[0] for rng in comfort_ranges)
        h_hi = min(rng[1] for rng in comfort_ranges)
        comfort = {'r': (0.25, 0.45), 'h': (round(h_lo, 2), round(h_hi, 2))}
    else:
        comfort = {'r': (0.25, 0.45), 'h': (None, None)}
    # 朝向规则：在舒适区中心、相机水平朝前的位形上算余量
    orientation: Dict[str, Any] = {'yaw_rel_deg': None, 'pitch_deg': (None, None), 'at_pose': None}
    if comfort['h'][0] is not None:
        h_mid = 0.5 * (comfort['h'][0] + comfort['h'][1])
        at = {'x': 0.35, 'y': 0.0, 'z': h_mid - base_height_m,
              'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        res = check_pose(model, at, margins)
        if res.ok:
            h = headroom(model, res.joints, margins=margins)
            orientation = {'yaw_rel_deg': float(math.floor(min(-h['dyaw'][0], h['dyaw'][1]))),
                           'pitch_deg': (float(math.ceil(h['pitch_abs'][0])),
                                         float(math.floor(h['pitch_abs'][1]))),
                           'at_pose': at}
    return {'table': table, 'r_max': r_max, 'j1_deg': math.degrees(model.upper[0] - jm[0]),
            'comfort': comfort, 'orientation': orientation,
            'links': {'upper_arm_m': pl.len2, 'forearm_m': pl.len3,
                      'shoulder_height_m': float(pl.tf_base_f1[2, 3] + pl.shoulder[2]
                                                 + base_height_m),
                      'reach_min_m': pl.dist_min, 'reach_max_m': pl.dist_max},
            'margins': {'dist_m': margins.dist_m, 'joint_rad': [float(v) for v in jm]},
            'base_height_m': base_height_m}


def _fmt_interval(iv: Sequence[float], unit: str, digits: int) -> str:
    """@brief 区间 → "[lo, hi] unit" 文本。
    @param iv     (lo, hi)
    @param unit   单位
    @param digits 小数位
    @return 字符串
    """
    return f'[{iv[0]:+.{digits}f}, {iv[1]:+.{digits}f}] {unit}'


def capability_card(model: ArmModel, joints: Optional[Sequence[float]] = None,
                    subject: Optional[Sequence[float]] = None, margins: Optional[Margins] = None,
                    base_height_m: float = 0.31) -> str:
    """@brief 生成喂给大模型的能力卡文本：静态约束（表格、J1、朝向规则）+（给了关节角时）当前状态与余量区间。
    @param model         模型
    @param joints        当前 6 轴关节角，None 只出静态部分
    @param subject       环绕主体位置（base 系），给了就附环绕余量
    @param margins       余量，None 取默认
    @param base_height_m 臂基座离地高
    @return 多行文本
    """
    margins = margins or Margins()
    data = capability_data(model, margins, base_height_m)
    jm = margins.joint_array()
    # 从臂 / 云台都在中位出发，光轴方位单向最多能转 J1 半程 + 云台 pan 半程（各减余量）
    pan_half = math.degrees(model.upper[0] + model.upper[3]) - math.degrees(jm[0] + jm[3])
    lines = [
        '[相机位姿约束]  坐标系：机械臂 base_link（x 前 / y 左 / z 上），末端 = 云台法兰 gimbal_tool0；'
        '位置用 (θ 方位角, r 前伸, h 离地高) 描述，θ 相对车头，r 是到臂基座竖轴的水平距离。',
        f'1. θ ∈ ±{data["j1_deg"]:.0f}°（J1 限位 ±{math.degrees(model.upper[0]):.0f}° 留 '
        f'{math.degrees(jm[0]):.0f}° 余量）；光轴方位从中位出发单向最多转 ±{pan_half:.0f}°'
        '（J1 + 云台 pan），整段镜头累计变化超过这个数必须底盘转身。',
        f'2. (r, h) 必须在下表内（臂末端位置，已含 {margins.dist_m * 100:.0f} cm 距离余量与关节余量）：',
    ]
    cells = []
    for row in data['table']:
        if not row['reachable']:
            cells.append(f'r={row["r"]:.2f}: 不可达')
            continue
        cell = f'r={row["r"]:.2f}: h {row["h_min"]:.2f}~{row["h_max"]:.2f}'
        others = [b for b in row['bands'] if abs(b[0] - row['h_min']) > 1e-6]
        if others:
            cell += '（另一段 ' + '、'.join(f'{b[0]:.2f}~{b[1]:.2f}' for b in others) + '）'
        cells.append(cell)
    cells.append(f'r ≥ {data["r_max"] + 0.01:.2f}: 不可达')
    for i in range(0, len(cells), 3):
        lines.append('     ' + ' | '.join(cells[i:i + 3]))
    c = data['comfort']
    if c['h'][0] is not None:
        lines.append(f'   余量最宽裕（离限位与臂长边界都远）的区域：r {c["r"][0]:.2f}~{c["r"][1]:.2f}、'
                     f'h {c["h"][0]:.2f}~{c["h"][1]:.2f}，没有构图理由时优先待在这里。')
    ori = data['orientation']
    if ori['yaw_rel_deg'] is not None:
        lines.append(f'3. 朝向 (pan, tilt)，画面始终水平：pan 相对臂方位 θ 在 '
                     f'±{ori["yaw_rel_deg"]:.0f}° 内任意；俯仰（ArmPose.pitch，俯视为正）∈ '
                     f'[{ori["pitch_deg"][0]:.0f}°, {ori["pitch_deg"][1]:.0f}°]'
                     '（在舒适区中心、相机水平时算得；相机很高时俯视余量变小、很低时仰视余量变小）。')
    lines.append(f'4. 相机比臂末端再偏 5~11 cm（随云台角变）；位置 ±0.1 m、角度 ±5° 的差别不影响可行性，'
                 '不必精算，执行前由 IK 精确确认。')
    lines.append('5. 底盘可平移 / 转身：相机位移 > 0.15 m 优先底盘走，臂只做 ≤ 0.15 m 的精细运动。')
    if joints is not None:
        lines.append('6. 当前状态：')
        lines.append(current_state_text(model, joints, subject, margins, base_height_m))
    return '\n'.join(lines)


def current_state_text(model: ArmModel, joints: Sequence[float],
                       subject: Optional[Sequence[float]] = None,
                       margins: Optional[Margins] = None, base_height_m: float = 0.31,
                       headroom_data: Optional[Dict[str, Any]] = None) -> str:
    """@brief 能力卡的"当前状态"段：相机 (θ, r, h) 与朝向、限位余量最小的关节、各方向余量区间。
           单独暴露是为了每一步都重发这一段、而静态部分放 system 提示词里只发一次。
    @param model         模型
    @param joints        当前 6 轴关节角
    @param subject       环绕主体位置，None 不算环绕
    @param margins       余量，None 取默认
    @param base_height_m 臂基座离地高
    @param headroom_data 已算好的 headroom() 结果（省一次重算），None 现算
    @return 多行文本
    """
    margins = margins or Margins()
    h = headroom_data or headroom(model, joints, subject=subject, margins=margins)
    p = h['pose']
    r_now = math.hypot(p['x'], p['y'])
    theta_now = math.degrees(math.atan2(p['y'], p['x']))
    q = np.asarray(joints, dtype=float)
    rem = np.minimum(q - model.lower, model.upper - q)
    i_min = int(np.argmin(rem))
    h_now = p['z'] + base_height_m
    fm = _fmt_interval
    lines = [
        f'     相机 θ={theta_now:+.0f}°, r={r_now:.2f} m, h={h_now:.2f} m；'
        f'朝向 yaw={p["yaw"]:+.0f}°, pitch={p["pitch"]:+.0f}°；'
        f'限位余量最小的是 {_joint_label(i_min)}（还剩 {rem[i_min]:.2f} rad）。',
        '     从当前位姿出发、姿态不变，各方向还能连续移动（下一步的数值必须落在区间内）：',
        f'       dolly {fm(h["dolly"], "m", 3)}   truck {fm(h["truck"], "m", 3)}'
        f'   crane {fm(h["crane"], "m", 3)}',
        f'       dyaw {fm(h["dyaw"], "°", 0)}   dpitch {fm(h["dpitch"], "°", 0)}'
        f'   （绝对 pitch 可到 {fm(h["pitch_abs"], "°", 0)}）',
    ]
    if 'arc_az' in h:
        a = h['arc']
        center_txt = tuple(round(v, 2) for v in a['center'])
        lines.append(f'       绕主体 {center_txt}（半径 {a["radius_m"]:.2f} m、'
                     f'仰角 {a["elevation_deg"]:+.0f}°）环绕：方位增量 {fm(h["arc_az"], "°", 0)}'
                     f'（当前方位 {a["az0_deg"]:+.0f}°）')
    return '\n'.join(lines)


def headroom_schema(h: Dict[str, Any]) -> Dict[str, Any]:
    """@brief 把 headroom 区间写成单步 JSON schema（oneOf 每个 op 一项，数值字段带 minimum/maximum），
           给支持 structured output 的大模型 API 当硬约束。
    @param h headroom() 的返回值
    @return JSON schema 字典
    """
    def num(lo: float, hi: float) -> Dict[str, Any]:
        """@brief 数值字段 schema。"""
        return {'type': 'number', 'minimum': float(lo), 'maximum': float(hi)}

    items: List[Dict[str, Any]] = []
    for op in ('dolly', 'truck', 'crane'):
        items.append({'type': 'object', 'required': ['op', 'distance_m'],
                      'properties': {'op': {'const': op}, 'distance_m': num(*h[op]),
                                     'speed': {'enum': ['slow', 'normal', 'fast']}}})
    items.append({'type': 'object', 'required': ['op'],
                  'properties': {'op': {'const': 'move_rel'},
                                 'dx': num(*h['dolly']), 'dy': num(*h['truck']),
                                 'dz': num(*h['crane']),
                                 'dyaw': num(*h['dyaw']), 'dpitch': num(*h['dpitch']),
                                 'speed': {'enum': ['slow', 'normal', 'fast']}},
                  'description': '多轴同时动时各区间的笛卡尔积略超可达域，执行侧会夹取'})
    if 'arc_az' in h:
        a = h['arc']
        items.append({'type': 'object',
                      'required': ['op', 'center', 'radius_m', 'az_start_deg', 'az_end_deg'],
                      'properties': {'op': {'const': 'arc'},
                                     'center': {'const': [round(v, 4) for v in a['center']]},
                                     'radius_m': {'const': round(a['radius_m'], 4)},
                                     'elevation_deg': {'const': round(a['elevation_deg'], 2)},
                                     'az_start_deg': {'const': round(a['az0_deg'], 2)},
                                     'az_end_deg': num(a['az0_deg'] + h['arc_az'][0],
                                                       a['az0_deg'] + h['arc_az'][1]),
                                     'speed': {'enum': ['slow', 'normal', 'fast']}}})
    return {'$schema': 'https://json-schema.org/draft/2020-12/schema', 'title': 'next_camera_step',
            'oneOf': items}
