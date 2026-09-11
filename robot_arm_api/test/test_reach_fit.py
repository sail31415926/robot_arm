# -*- coding: utf-8 -*-
"""
@file  test_reach_fit.py
@brief reach_fit（可达区多项式拟合，"拟合函数喂大模型"方案）测试：拟合必须保守（拟合区内每个点都真可达）、
       覆盖率够高、公式文本可读、序列化往返。
"""

import numpy as np
import pytest

from robot_arm_api import reach_check as rc
from robot_arm_api import reach_fit as rf

ARM_NOMINAL = np.array([0.0, 1.0, -1.5, 0.0, 0.3, 0.0])


@pytest.fixture(scope='session')
def model(urdf_xml):
    """@brief 全测试共用的 ArmModel。"""
    return rc.ArmModel.from_urdf_string(urdf_xml)


@pytest.fixture(scope='session')
def fit(model):
    """@brief 默认参数（三次多项式、默认余量、基座离地 0.31）的拟合结果。"""
    return rf.fit_reach_region(model)


def test_fit_is_conservative_everywhere(model, fit):
    """@brief 拟合区内按 5 mm×1 cm 网格取点，每个点都必须被闭式模型判为可达（不允许假可达）。"""
    margins = rc.Margins()
    bad = []
    for r in np.arange(fit.r_min, fit.r_max + 1e-9, 0.005):
        bounds = fit.h_bounds(float(r))
        assert bounds is not None, f'r={r:.3f} 处拟合区塌缩（上下界交叉）'
        h_lo, h_hi = bounds
        for h in np.arange(h_lo, h_hi + 1e-9, 0.01):
            if not rc._tip_feasible(model, (float(r), 0.0, float(h) - fit.base_height_m), margins):
                bad.append((round(float(r), 3), round(float(h), 3)))
    assert not bad, f'拟合区内有 {len(bad)} 个不可达点，例如 {bad[:5]}'


def test_fit_range_starts_where_the_hole_closes(model, fit):
    """@brief 拟合区间只取"可达高度单段"的 r：本臂肩部盲区让 r ≲ 0.22 分成两段，所以 r_min 落在 0.22~0.28；
    r_max 至少到 0.55。范围外（含洞区里的点）一律不判可达。"""
    assert 0.22 <= fit.r_min <= 0.28, fit.r_min
    assert fit.r_max >= 0.55, fit.r_max
    assert fit.h_bounds(fit.r_max + 0.05) is None
    assert fit.h_bounds(fit.r_min - 0.05) is None
    margins = rc.Margins()
    r_in_hole = fit.r_min - 0.06
    bands = rc._h_bands_at(model, r_in_hole, margins, fit.base_height_m)
    assert len(bands) >= 2, (r_in_hole, bands)          # 确认那里真有洞
    assert not fit.contains(r_in_hole, 0.9)             # 公式不覆盖 → 一律不判可达


def test_fit_covers_most_of_true_region(fit):
    """@brief 保守收缩后仍要覆盖拟合 r 区间内真实可达区的 85% 以上，且收缩量不超过 5 cm。"""
    assert fit.coverage >= 0.85, fit.coverage
    assert fit.shrink_hi_m <= 0.05 and fit.shrink_lo_m <= 0.05, (fit.shrink_hi_m, fit.shrink_lo_m)


def test_fit_matches_static_table_at_sample_radii(fit):
    """@brief r=0.30 处的高度范围与静态表（0.20~1.09）一致到 5 cm，且只会向内收（不许比表更宽）。"""
    h_lo, h_hi = fit.h_bounds(0.30)
    assert 0.20 <= h_lo <= 0.20 + 0.05, h_lo
    assert 1.09 - 0.05 <= h_hi <= 1.09, h_hi


def test_fit_contains_xyz_uses_base_link_frame(model, fit):
    """@brief contains_xyz 吃 base_link 系坐标（r = hypot(x,y)、h = z + 基座高）：舒适区中心可达，
    前方 0.9 m、同 r 上方 1.30 m 都不可达。"""
    r_mid = 0.5 * (fit.r_min + fit.r_max)
    h_mid = 0.5 * sum(fit.h_bounds(r_mid))
    assert fit.contains_xyz(r_mid, 0.0, h_mid - fit.base_height_m)
    assert fit.contains_xyz(0.0, -r_mid, h_mid - fit.base_height_m)   # 只看 hypot，与方位无关
    assert not fit.contains_xyz(0.9, 0.0, 0.5)
    assert not fit.contains(0.30, 1.30)


def test_fit_formula_text_and_dict_round_trip(fit):
    """@brief 公式文本含两条多项式、r 范围、换算说明、洞区提示；to_dict/from_dict 往返后判定一致。"""
    text = fit.formula_text()
    assert 'h_max(r)' in text and 'h_min(r)' in text and 'r ∈' in text and 'sqrt(x' in text
    assert f'{fit.base_height_m:.2f}' in text and '分段表' in text
    back = rf.RegionFit.from_dict(fit.to_dict())
    for r in (0.25, 0.30, 0.50):
        assert back.h_bounds(r) == pytest.approx(fit.h_bounds(r))
    assert back.contains(0.30, 0.60) == fit.contains(0.30, 0.60)
