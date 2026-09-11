# -*- coding: utf-8 -*-
"""
@file      reach_fit.py
@brief     可达区的多项式拟合（"把拟合函数喂给大模型"方案）：把 reach_check 闭式模型算出的
           臂末端可达区边界 h_min(r) / h_max(r) 拟成 r 的多项式，并**向内收缩到保守**
           （拟合区内任一点都真可达），生成大模型能直接代数的公式文本。
@version   0.1
@date      2026-09-11
@copyright Copyright (c) 2026 eMeet

与 capability_card 的静态表相比：表是离散采样，公式是连续函数——大模型输出任意 (x, y, z) 时可以自己
代入 r = sqrt(x²+y²)、h = z + 基座高 验算。两个必须知道的边界条件：

  ① **公式只覆盖"一段式"的 r 区间**。r 小到一定程度（本臂 ≲0.22 m）时，肩部周围 dist_min+余量 的球
     够不到，可达高度被切成上下两段（r=0.15：主段 0.72~1.15，另有 0.27~0.42）——带洞的区域没法用
     "h_min(r) ≤ h ≤ h_max(r)" 表达，硬拟就会把洞判成可达。所以拟合区间从洞合上的地方起步，
     更近的位置交给 capability_card 的分段表 / headroom。
  ② **保守优先**：拟合后按最大残差 + 安全量整体内移，再网格复核，发现假可达继续内移。
     coverage 告诉你为此损失了多少真实可达区。

═══════════════════════════════ 本文件函数 / 类汇总 ═══════════════════════════════
  RegionFit                        拟合结果：系数、r 范围、收缩量、覆盖率；h_bounds / contains / formula_text
  fit_reach_region(model, …)       从 ArmModel 现算边界 → 选单段区间 → 拟合 → 保守收缩 → 网格复核 → RegionFit
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .reach_check import ArmModel, Margins, _h_bands_at, _tip_feasible


def _poly_text(coeffs: Sequence[float]) -> str:
    """@brief 多项式 → "a·r^3 + b·r^2 + c·r + d" 文本。
    @param coeffs 最高次在前
    @return 字符串
    """
    n = len(coeffs) - 1
    parts: List[str] = []
    for i, c in enumerate(coeffs):
        p = n - i
        term = f'{abs(c):.3f}' + ('' if p == 0 else ('·r' if p == 1 else f'·r^{p}'))
        parts.append(('- ' if c < 0 else ('+ ' if parts else '')) + term)
    return ' '.join(parts)


@dataclass
class RegionFit:
    """@brief 可达区拟合结果。h_max / h_min 为 r 的多项式（系数最高次在前，numpy.polyval 约定），
           h 为离地高（m）。只在 r ∈ [r_min, r_max]（可达高度单段的区间）内有效。"""
    degree: int
    r_min: float
    r_max: float
    hmax_coeffs: List[float]
    hmin_coeffs: List[float]
    base_height_m: float
    shrink_hi_m: float = 0.0      # h_max 最大向下内移量（保守）
    shrink_lo_m: float = 0.0      # h_min 最大向上内移量
    coverage: float = 1.0         # 拟合区覆盖真实可达区的比例
    margins: Dict[str, Any] = field(default_factory=dict)

    def h_bounds(self, r: float) -> Optional[tuple]:
        """@brief 水平距离 r 处允许的离地高范围。
        @param r 到臂基座竖轴的水平距离，m
        @return (h_min, h_max)；r 不在可用范围或上下界交叉时 None
        """
        if r < self.r_min - 1e-12 or r > self.r_max + 1e-12:
            return None
        h_hi = float(np.polyval(self.hmax_coeffs, r))
        h_lo = float(np.polyval(self.hmin_coeffs, r))
        if h_lo >= h_hi:
            return None
        return h_lo, h_hi

    def contains(self, r: float, h: float) -> bool:
        """@brief (r, h) 是否在拟合可达区内。
        @param r 水平距离，m
        @param h 离地高，m
        @return bool
        """
        bounds = self.h_bounds(r)
        return bounds is not None and bounds[0] <= h <= bounds[1]

    def contains_xyz(self, x: float, y: float, z: float) -> bool:
        """@brief base_link 系坐标是否在拟合可达区内（r = hypot(x, y)，h = z + 基座离地高）。
        @param x,y,z 臂末端位置，机械臂 base_link 系，m
        @return bool
        """
        return self.contains(math.hypot(x, y), z + self.base_height_m)

    def formula_text(self) -> str:
        """@brief 给大模型的公式段：两条多项式、r 范围、判定式、坐标换算、几个代入样例。
        @return 多行文本
        """
        samples = []
        for r in np.linspace(self.r_min, self.r_max, 6):
            b = self.h_bounds(float(r))
            if b:
                samples.append(f'r={r:.2f} → h ∈ [{b[0]:.2f}, {b[1]:.2f}]')
        lines = [
            f'臂末端可达区用 r 的 {self.degree} 次多项式描述（r = 到臂基座竖轴的水平距离，'
            f'h = 离地高，单位 m）：',
            f'  r ∈ [{self.r_min:.2f}, {self.r_max:.2f}]',
            f'  h_max(r) = {_poly_text(self.hmax_coeffs)}',
            f'  h_min(r) = {_poly_text(self.hmin_coeffs)}',
            '  点 (r, h) 可达 ⇔ r 在范围内 且 h_min(r) ≤ h ≤ h_max(r)。',
            f'  r < {self.r_min:.2f} 时相机太贴近臂基座竖轴，可达高度被肩部盲区切成两段、公式不适用，'
            '这种位置请改用能力卡的分段表。',
            f'  换算：r = sqrt(x² + y²)，h = z + {self.base_height_m:.2f}（x, y, z 为机械臂 base_link 系）。',
            f'  公式已向内收缩（h_max 下移 ≤ {self.shrink_hi_m * 100:.0f} cm、h_min 上移 ≤ '
            f'{self.shrink_lo_m * 100:.0f} cm）保证保守：公式内的点全部真可达，覆盖真实可达区 '
            f'{self.coverage * 100:.0f}%。',
            '  样例：' + '；'.join(samples),
        ]
        return '\n'.join(lines)

    def to_dict(self) -> Dict[str, Any]:
        """@brief 可 JSON 化的字典。
        @return dict
        """
        return {'degree': self.degree, 'r_min': self.r_min, 'r_max': self.r_max,
                'hmax_coeffs': [float(c) for c in self.hmax_coeffs],
                'hmin_coeffs': [float(c) for c in self.hmin_coeffs],
                'base_height_m': self.base_height_m, 'shrink_hi_m': self.shrink_hi_m,
                'shrink_lo_m': self.shrink_lo_m, 'coverage': self.coverage,
                'margins': dict(self.margins)}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'RegionFit':
        """@brief to_dict 的逆。
        @param d 字典
        @return RegionFit
        """
        return cls(**d)


def _single_band_boundary(model: ArmModel, margins: Margins, base_height_m: float,
                          r_step: float, z_step: float) -> tuple:
    """@brief 扫描闭式模型，取**可达高度只有一段**（无洞）的最长连续 r 区间及其上下边界。
    @param model         模型
    @param margins       余量
    @param base_height_m 基座离地高
    @param r_step        r 采样步长
    @param z_step        z 扫描步长
    @return (rs, h_lo, h_hi) 三个等长 ndarray
    @throws ValueError 单段区间太短，拟合不了
    """
    rows = []
    for r in np.arange(0.03, 1.0 + 1e-9, r_step):
        bands = _h_bands_at(model, float(r), margins, base_height_m, z_step=z_step)
        rows.append((float(r), bands[0] if len(bands) == 1 else None))
    best: List[tuple] = []
    cur: List[tuple] = []
    for r, band in rows:
        if band is None:
            if len(cur) > len(best):
                best = cur
            cur = []
        else:
            cur.append((r, band[0], band[1]))
    if len(cur) > len(best):
        best = cur
    if len(best) < 6:
        raise ValueError('单段可达区太短，拟合不了')
    arr = np.array(best)
    return arr[:, 0], arr[:, 1], arr[:, 2]


def _coverage(fit: RegionFit, rs: np.ndarray, h_lo: np.ndarray, h_hi: np.ndarray,
              z_step: float) -> float:
    """@brief 拟合区覆盖真实可达区（拟合 r 区间内）的比例。
    @param fit    拟合
    @param rs     真实边界的 r 采样
    @param h_lo   对应 h_min
    @param h_hi   对应 h_max
    @param z_step 高度采样步长
    @return 0~1
    """
    inside = total = 0
    for r, lo, hi in zip(rs, h_lo, h_hi):
        if r < fit.r_min - 1e-12 or r > fit.r_max + 1e-12:
            continue
        for h in np.arange(lo, hi + 1e-9, z_step):
            total += 1
            inside += int(fit.contains(float(r), float(h)))
    return inside / total if total else 0.0


def fit_reach_region(model: ArmModel, margins: Optional[Margins] = None,
                     base_height_m: float = 0.31, degree: int = 3, r_step: float = 0.01,
                     z_step: float = 0.005, safety_m: float = 0.01) -> RegionFit:
    """@brief 拟合可达区：扫出"一段式"的最长 r 区间 → 在几种两端截断里选覆盖率最高的多项式
           （按最大残差 + safety 向内收缩）→ 网格复核保守性（发现假可达继续内移）→ 算最终覆盖率。
    @param model         ArmModel
    @param margins       余量，None 取默认 Margins()
    @param base_height_m 臂基座离地高
    @param degree        多项式次数（3 已足够贴圆弧段）
    @param r_step        边界采样步长，m
    @param z_step        高度扫描步长，m
    @param safety_m      在最大残差之外再多收的安全量，m
    @return RegionFit
    """
    margins = margins or Margins()
    rs, h_lo, h_hi = _single_band_boundary(model, margins, base_height_m, r_step, z_step)
    margin_rec = {'dist_m': margins.dist_m, 'joint_rad': [float(v) for v in margins.joint_array()]}

    def build(head: int, tail: int) -> RegionFit:
        """@brief 用 rs[head:len-tail] 拟一版（含保守内移）。
        @param head 前端截断点数
        @param tail 末端截断点数
        @return RegionFit
        """
        seg_r = rs[head:len(rs) - tail] if tail else rs[head:]
        seg_lo = h_lo[head:len(rs) - tail] if tail else h_lo[head:]
        seg_hi = h_hi[head:len(rs) - tail] if tail else h_hi[head:]
        hi_c = np.polyfit(seg_r, seg_hi, degree)
        lo_c = np.polyfit(seg_r, seg_lo, degree)
        s_hi = max(0.0, float(np.max(np.polyval(hi_c, seg_r) - seg_hi))) + safety_m
        s_lo = max(0.0, float(np.max(seg_lo - np.polyval(lo_c, seg_r)))) + safety_m
        hi_c[-1] -= s_hi
        lo_c[-1] += s_lo
        return RegionFit(degree, float(seg_r[0]), float(seg_r[-1]), list(hi_c), list(lo_c),
                         base_height_m, s_hi, s_lo, 1.0, margin_rec)

    best: Optional[RegionFit] = None
    best_cov = -1.0
    for head in range(0, 4):
        for tail in range(0, 4):
            if len(rs) - head - tail < degree + 3:
                continue
            cand = build(head, tail)
            cov = _coverage(cand, rs, h_lo, h_hi, z_step * 4)
            if cov > best_cov:
                best, best_cov = cand, cov
    fit = best

    # 网格复核：拟合区内出现不可达点就把对应边再往里收，直到干净
    for _ in range(30):
        worst_hi = worst_lo = 0.0
        for r in np.arange(fit.r_min, fit.r_max + 1e-9, r_step / 2):
            bounds = fit.h_bounds(float(r))
            if bounds is None:
                continue
            mid = 0.5 * (bounds[0] + bounds[1])
            for h in np.arange(bounds[0], bounds[1] + 1e-9, z_step * 2):
                if _tip_feasible(model, (float(r), 0.0, float(h) - base_height_m), margins):
                    continue
                if h > mid:
                    worst_hi = max(worst_hi, bounds[1] - h + z_step * 2)
                else:
                    worst_lo = max(worst_lo, h - bounds[0] + z_step * 2)
        if worst_hi == 0.0 and worst_lo == 0.0:
            break
        fit.hmax_coeffs[-1] -= worst_hi
        fit.hmin_coeffs[-1] += worst_lo
        fit.shrink_hi_m += worst_hi
        fit.shrink_lo_m += worst_lo

    fit.coverage = _coverage(fit, rs, h_lo, h_hi, z_step * 2)
    return fit
