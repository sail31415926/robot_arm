# -*- coding: utf-8 -*-
"""
@file  test_shot_spec.py
@brief robot_arm_api.shot_spec 单元测试：《摄影机器人·拍摄接口规范 v9》JSON 的解析 / 第 8 章校验规则 /
       A 型参考轨迹几何（line / arc / spline / 纯摇镜）/ 相机姿态构造 / B 型按目标快照展开成 A 型。
       纯 Python，不需要 ROS。
"""

import glob
import json
import math
import os

import numpy as np
import pytest

from robot_arm_api import shot_spec as ss

_EXAMPLES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'examples', 'spec_v9')


def _load(name):
    """@brief 读 examples/spec_v9 下的一个示例。"""
    with open(os.path.join(_EXAMPLES, name), encoding='utf-8') as fh:
        return json.load(fh)


def _rules(report):
    """@brief 报告里的错误规则号集合。"""
    return {e.rule for e in report.errors}


def _warn_rules(report):
    """@brief 报告里的告警规则号集合。"""
    return {w.rule for w in report.warnings}


def _orbit():
    """@brief 规范例 5.5.2 的深拷贝。"""
    return _load('a_5_5_2_orbit90.json')


# ═══════════════════════════════ 示例全部合法 ═══════════════════════════════

@pytest.mark.parametrize('name', sorted(os.path.basename(p)
                                        for p in glob.glob(os.path.join(_EXAMPLES, '*.json'))))
def test_spec_examples_have_no_errors(name):
    """@brief 规范 5.5 / 6.6 的十个例子都应通过校验（只允许告警）。"""
    report = ss.validate_shot(_load(name))
    assert report.errors == [], [e.to_dict() for e in report.errors]


def test_parse_shot_returns_typed_spec():
    """@brief parse_shot 把 5.5.2 解析成 ShotSpec：类型 / 路点数 / 段数 / 圆弧字段 / 容差。"""
    spec = ss.parse_shot(_orbit())
    assert spec.type == 'static'
    assert len(spec.waypoints) == 2 and len(spec.segments) == 1
    np.testing.assert_allclose(spec.waypoints[1].look_at, [1.2, 0.0, 0.9])
    assert spec.waypoints[1].hold == 1.5 and spec.waypoints[0].hold == 0.0
    assert spec.segments[0].path == 'arc'
    np.testing.assert_allclose(spec.segments[0].axis, [0, 0, -1])
    assert spec.tolerance.aim == 0.02


def test_parse_shot_raises_structured_errors():
    """@brief 非法 JSON：parse_shot 抛 SpecValidationError，errors 里每条都有 rule / field / message。"""
    bad = _orbit()
    bad['segments'] = []
    with pytest.raises(ss.SpecValidationError) as exc:
        ss.parse_shot(bad)
    errs = exc.value.report.errors
    assert errs and all(e.rule and e.field and e.message for e in errs)
    assert 2 in _rules(exc.value.report)


# ═══════════════════════════════ 8.1 两型通用 ═══════════════════════════════

def test_rule_1_type_and_foreign_fields():
    """@brief 规则 1：type 必须是 static/tracking；A 型里出现 B 型字段报错。"""
    bad = _orbit()
    bad['type'] = 'B'
    assert 1 in _rules(ss.validate_shot(bad))
    bad = _orbit()
    bad['target'] = '/subject_pose'
    assert 1 in _rules(ss.validate_shot(bad))


def test_rule_2_segments_length_mismatch():
    """@brief 规则 2：segments 长度必须等于 waypoints−1。"""
    bad = _orbit()
    bad['segments'].append(dict(bad['segments'][0]))
    assert 2 in _rules(ss.validate_shot(bad))


def test_rule_3_waypoints_non_empty():
    """@brief 规则 3：waypoints 至少一个。"""
    bad = _orbit()
    bad['waypoints'] = []
    bad['segments'] = []
    assert 3 in _rules(ss.validate_shot(bad))


def test_rule_4_duration_speed_exactly_one():
    """@brief 规则 4：duration 与 speed 恰好一个。"""
    both = _orbit()
    both['segments'][0]['speed'] = 0.1
    assert 4 in _rules(ss.validate_shot(both))
    neither = _orbit()
    del neither['segments'][0]['duration']
    assert 4 in _rules(ss.validate_shot(neither))


def test_rule_5_positive_numbers():
    """@brief 规则 5：duration/speed > 0，hold ≥ 0。"""
    bad = _orbit()
    bad['segments'][0]['duration'] = 0.0
    assert 5 in _rules(ss.validate_shot(bad))
    bad = _orbit()
    bad['waypoints'][1]['hold'] = -1.0
    assert 5 in _rules(ss.validate_shot(bad))


def test_rule_6_speed_forbidden_on_zero_length_segment():
    """@brief 规则 6：纯摇镜（两端 position 相同）不能用 speed。"""
    pan = {
        'type': 'static',
        'waypoints': [{'position': [0.3, 0, 0.6], 'look_at': [1.0, 0.0, 0.6]},
                      {'position': [0.3, 0, 0.6], 'look_at': [0.3, 1.0, 0.6]}],
        'segments': [{'speed': 0.05}],
        'tolerance': {'position': 0.03, 'aim': 0.02, 'roll': 0.02},
    }
    assert 6 in _rules(ss.validate_shot(pan))
    pan['segments'] = [{'duration': 2.0}]
    assert ss.validate_shot(pan).errors == []


# ═══════════════════════════════ 8.2 A 型 ═══════════════════════════════

def test_rule_7_position_look_at_required():
    """@brief 规则 7：A 型 waypoint 必须有 position 与 look_at。"""
    bad = _orbit()
    del bad['waypoints'][0]['look_at']
    assert 7 in _rules(ss.validate_shot(bad))


def test_rule_8_look_at_must_differ_from_position():
    """@brief 规则 8：look_at 不能与 position 重合。"""
    bad = _orbit()
    bad['waypoints'][0]['look_at'] = list(bad['waypoints'][0]['position'])
    assert 8 in _rules(ss.validate_shot(bad))


def test_rule_9_center_axis_presence():
    """@brief 规则 9：arc 必须有 center/axis 且 axis 非零；非 arc 禁止出现 center/axis。"""
    no_axis = _orbit()
    del no_axis['segments'][0]['axis']
    assert 9 in _rules(ss.validate_shot(no_axis))
    zero_axis = _orbit()
    zero_axis['segments'][0]['axis'] = [0, 0, 0]
    assert 9 in _rules(ss.validate_shot(zero_axis))
    line_with_center = _load('a_5_5_4_topdown_truck.json')
    line_with_center['segments'][0]['center'] = [0.5, 0, 1.5]
    assert 9 in _rules(ss.validate_shot(line_with_center))


def test_rule_10_arc_center_at_subject_is_rejected_with_suggestion():
    """@brief 规则 10：把 center 误填成被摄物位置（不垂直于 axis）→ 报错并建议正确的圆心。"""
    bad = _orbit()
    bad['segments'][0]['center'] = [1.20, 0.00, 0.90]
    report = ss.validate_shot(bad)
    assert 10 in _rules(report)
    err = next(e for e in report.errors if e.rule == 10)
    np.testing.assert_allclose(err.suggestion['center'], [1.20, 0.00, 1.21], atol=1e-6)


def test_rule_10_arc_unequal_radius():
    """@brief 规则 10：两端到圆心距离不等也报错。"""
    bad = _orbit()
    bad['waypoints'][1]['position'] = [0.6, 0.6, 1.21]
    assert 10 in _rules(ss.validate_shot(bad))


def test_rule_11_arc_endpoints_coincide():
    """@brief 规则 11：arc 起止点重合报错。"""
    bad = _orbit()
    bad['waypoints'][1]['position'] = list(bad['waypoints'][0]['position'])
    assert 11 in _rules(ss.validate_shot(bad))


def test_rule_12_spline_with_two_waypoints_warns():
    """@brief 规则 12：只有 2 个 waypoint 却用 spline → 告警而非错误。"""
    two = _load('a_5_5_4_topdown_truck.json')
    two['segments'][0]['path'] = 'spline'
    report = ss.validate_shot(two)
    assert report.errors == [] and 12 in _warn_rules(report)


def test_rule_13_pan_of_180_degrees_is_rejected():
    """@brief 规则 13：纯摇镜扫角达到 180° 报错（往左还是往右不可表达）。"""
    pan = {
        'type': 'static',
        'waypoints': [{'position': [0.3, 0, 0.6], 'look_at': [1.3, 0.0, 0.6]},
                      {'position': [0.3, 0, 0.6], 'look_at': [-0.7, 0.0, 0.6]}],
        'segments': [{'duration': 2.0}],
        'tolerance': {'position': 0.03, 'aim': 0.02, 'roll': 0.02},
    }
    assert 13 in _rules(ss.validate_shot(pan))
    pan['waypoints'][1]['look_at'] = [0.3, 1.0, 0.6]   # 90°：合法
    assert 13 not in _rules(ss.validate_shot(pan))


def test_rule_14_tolerance_complete_and_positive():
    """@brief 规则 14：A 型 tolerance 三项齐全且为正。"""
    bad = _orbit()
    del bad['tolerance']['aim']
    assert 14 in _rules(ss.validate_shot(bad))
    bad = _orbit()
    bad['tolerance']['roll'] = 0.0
    assert 14 in _rules(ss.validate_shot(bad))


# ═══════════════════════════════ 8.3 B 型 ═══════════════════════════════

def test_rule_15_16_target_and_position_ref():
    """@brief 规则 15/16：target 非空字符串；position_ref 三选一。"""
    bad = _load('b_6_6_4_follow.json')
    bad['target'] = ''
    assert 15 in _rules(ss.validate_shot(bad))
    bad = _load('b_6_6_4_follow.json')
    bad['position_ref'] = 'subject'
    assert 16 in _rules(ss.validate_shot(bad))


def test_rule_17_target_ref_requires_d_az_el_and_forbids_position():
    """@brief 规则 17：position_ref=target 时 waypoint/tolerance 用 d/az/el、不能有 position。"""
    bad = _load('b_6_6_4_follow.json')
    bad['waypoints'][0]['position'] = [1, 2, 3]
    assert 17 in _rules(ss.validate_shot(bad))
    bad = _load('b_6_6_4_follow.json')
    del bad['waypoints'][0]['el']
    assert 17 in _rules(ss.validate_shot(bad))
    bad = _load('b_6_6_4_follow.json')
    bad['tolerance']['position'] = 0.03
    assert 17 in _rules(ss.validate_shot(bad))


def test_rule_18_world_ref_requires_position_and_forbids_d_az_el():
    """@brief 规则 18：position_ref=world 时用 position、不能有 d/az/el。"""
    bad = _load('b_6_6_1_world_push.json')
    bad['waypoints'][0]['d'] = 1.0
    assert 18 in _rules(ss.validate_shot(bad))
    bad = _load('b_6_6_1_world_push.json')
    bad['tolerance']['az'] = 0.1
    assert 18 in _rules(ss.validate_shot(bad))


def test_rule_19_u_v_roll_tolerance_required():
    """@brief 规则 19：u / v / roll 容差三种 position_ref 下都必填且为正。"""
    bad = _load('b_6_6_1_world_push.json')
    del bad['tolerance']['u']
    assert 19 in _rules(ss.validate_shot(bad))


def test_rule_20_d_positive():
    """@brief 规则 20：d > 0。"""
    bad = _load('b_6_6_4_follow.json')
    bad['waypoints'][0]['d'] = 0.0
    assert 20 in _rules(ss.validate_shot(bad))


def test_rule_21_uv_bounds_depend_on_lens_and_aspect():
    """@brief 规则 21：uv 出画边界 = 镜头视场 × 交付画幅；同一个 uv 在 4:3 合法、在 9:16 出画。"""
    shot = _load('b_6_6_4_follow.json')
    shot['waypoints'][0]['uv'] = [0.30, 0.0]
    assert 21 not in _rules(ss.validate_shot(shot, lens='WIDE', aspect='4:3'))
    assert 21 in _rules(ss.validate_shot(shot, lens='WIDE', aspect='9:16'))
    shot['waypoints'][0]['uv'] = [0.0, 0.35]
    assert 21 in _rules(ss.validate_shot(shot, lens='WIDE', aspect='16:9'))


def test_rule_22_large_delta_az_warns():
    """@brief 规则 22：相邻 waypoint |Δaz| > 2π 只告警。"""
    shot = _load('b_6_6_5_static_orbit.json')
    shot['waypoints'][1]['az'] = 7.0
    report = ss.validate_shot(shot)
    assert 22 in _warn_rules(report) and 22 not in _rules(report)


def test_rule_23_target_static_with_loose_tolerance():
    """@brief 规则 23：声明 target_static 却把位置容差写得比"紧档"松 → 报错。"""
    bad = _load('b_6_6_5_static_orbit.json')
    bad['tolerance']['d'] = 0.30
    assert 23 in _rules(ss.validate_shot(bad))


def test_rule_28_d_tolerance_below_perception_floor_warns():
    """@brief 规则 28：d 容差 < 0.08·d 只告警（规范自己的例 6.6.3 就踩线）。"""
    shot = _load('b_6_6_5_static_orbit.json')
    shot['tolerance']['d'] = 0.05
    report = ss.validate_shot(shot)
    assert 28 in _warn_rules(report) and 28 not in _rules(report)


# ═══════════════════════════════ 8.5 av ═══════════════════════════════

def _with_av(shot, av):
    """@brief 给分镜挂一组 av。"""
    shot = dict(shot)
    shot['av'] = av
    return shot


def test_av_rule_29_31_33_enums():
    """@brief 规则 29/31/33：lens.camera、recording.mode、slowmo 枚举。"""
    base = _orbit()
    assert 29 in _rules(ss.validate_shot(_with_av(base, {'lens': {'camera': '35mm'}})))
    assert 31 in _rules(ss.validate_shot(_with_av(base, {'recording': {'mode': 'STILL', 'fps': 30}})))
    assert 33 in _rules(ss.validate_shot(_with_av(
        base, {'recording': {'mode': 'VIDEO', 'fps': 120, 'slowmo': 4, 'duration_ms': 1000}})))


def test_av_rule_32_duration_ms_by_mode():
    """@brief 规则 32：PHOTO 的 duration_ms 必须 0；VIDEO 在 5–30000 或 0。"""
    base = _orbit()
    assert 32 in _rules(ss.validate_shot(_with_av(
        base, {'recording': {'mode': 'PHOTO', 'fps': 30, 'duration_ms': 100},
               'audio': {'enabled': False}})))
    assert 32 in _rules(ss.validate_shot(_with_av(
        base, {'recording': {'mode': 'VIDEO', 'fps': 30, 'duration_ms': 3}})))
    ok = ss.validate_shot(_with_av(base, {'recording': {'mode': 'VIDEO', 'fps': 30,
                                                          'duration_ms': 0}}))
    assert ok.errors == []


def test_av_rule_34_38_manual_modes():
    """@brief 规则 34：focus MANUAL 要 distance_m；规则 38：exposure MANUAL 不能四项全 auto。"""
    base = _orbit()
    assert 34 in _rules(ss.validate_shot(_with_av(base, {'focus': {'target': 'MANUAL'}})))
    assert 38 in _rules(ss.validate_shot(_with_av(base, {'exposure': {'mode': 'MANUAL'}})))
    ok = ss.validate_shot(_with_av(base, {'exposure': {'mode': 'MANUAL', 'shutter': '1/120'}}))
    assert 38 not in _rules(ok)


def test_av_rule_35_subject_anchor_and_target_beamform_need_tracking():
    """@brief 规则 35：A 型不能用 focus SUBJECT_ANCHOR / audio TARGET。"""
    base = _orbit()
    report = ss.validate_shot(_with_av(base, {'focus': {'target': 'SUBJECT_ANCHOR'},
                                              'audio': {'beamform': 'TARGET'}}))
    assert [e.rule for e in report.errors].count(35) == 2
    tracking = _load('b_6_6_4_follow.json')
    assert 35 not in _rules(ss.validate_shot(_with_av(tracking, {'focus': {'target': 'SUBJECT_ANCHOR'}})))


def test_av_rule_37_39_audio_constraints():
    """@brief 规则 37：PHOTO 不许收音；规则 39：slowmo 2 不许收音。"""
    base = _orbit()
    assert 37 in _rules(ss.validate_shot(_with_av(
        base, {'recording': {'mode': 'PHOTO', 'fps': 30, 'duration_ms': 0},
               'audio': {'enabled': True}})))
    assert 39 in _rules(ss.validate_shot(_with_av(
        base, {'recording': {'mode': 'VIDEO', 'fps': 60, 'slowmo': 2, 'duration_ms': 2500},
               'audio': {'enabled': True}})))


def test_av_rule_40_fps_equals_base_times_slowmo():
    """@brief 规则 40：给了基准帧率才查 fps = 基准 × slowmo。"""
    base = _orbit()
    av = {'recording': {'mode': 'VIDEO', 'fps': 30, 'slowmo': 2, 'duration_ms': 2500},
          'audio': {'enabled': False}}
    assert 40 in _rules(ss.validate_shot(_with_av(base, av), base_fps=30))
    assert 40 not in _rules(ss.validate_shot(_with_av(base, av)))


# ═══════════════════════════════ 相机姿态构造 ═══════════════════════════════

def test_camera_rotation_axes_for_level_camera():
    """@brief 光学系列向量 = (右, 下, 前)：看向 +X 且 roll=0 时 右=−Y、下=−Z、前=+X。"""
    rot = ss.camera_rotation([0, 0, 1], [1, 0, 1], 0.0)
    np.testing.assert_allclose(rot[:, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(rot[:, 1], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(rot[:, 2], [1, 0, 0], atol=1e-12)


def test_camera_rotation_roll_sign_is_clockwise_from_behind():
    """@brief roll 正 = 从相机背后看顺时针：右向量往下沉（z 分量变负），前向量不变。"""
    rot = ss.camera_rotation([0, 0, 1], [1, 0, 1], 0.2)
    assert rot[2, 0] < 0
    np.testing.assert_allclose(rot[:, 2], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(rot @ rot.T, np.eye(3), atol=1e-12)


def test_camera_rotation_straight_down_is_well_defined():
    """@brief 垂直俯视时地平线不存在：取世界 +X 为画面上方，右向量仍水平。"""
    rot = ss.camera_rotation([0, 0, 1.5], [0, 0, 0], 0.0)
    assert not np.isnan(rot).any()
    np.testing.assert_allclose(rot[:, 2], [0, 0, -1], atol=1e-12)
    assert abs(rot[2, 0]) < 1e-12
    np.testing.assert_allclose(-rot[:, 1], [1, 0, 0], atol=1e-12)


def test_uv_of_projects_target_into_normalized_image_coords():
    """@brief uv_of：目标在光轴上是 (0,0)；目标偏右 → u>0；目标偏下 → v>0。"""
    rot = ss.camera_rotation([0, 0, 1], [1, 0, 1], 0.0)
    assert ss.uv_of(rot, [0, 0, 1], [2, 0, 1]) == pytest.approx((0.0, 0.0))
    u, v = ss.uv_of(rot, [0, 0, 1], [2, -0.4, 1])
    assert u == pytest.approx(0.2) and v == pytest.approx(0.0)
    u, v = ss.uv_of(rot, [0, 0, 1], [2, 0, 0.8])
    assert v == pytest.approx(0.1)


# ═══════════════════════════════ A 型参考轨迹 ═══════════════════════════════

def test_line_reference_interpolates_position_and_look_at_point():
    """@brief 例 5.5.4：直线段中点位置在两点之间，look_at 点随之插值 → 全程垂直俯视。"""
    spec = ss.parse_shot(_load('a_5_5_4_topdown_truck.json'))
    ref = ss.reference_pose(spec, 0, 0.5)
    np.testing.assert_allclose(ref.position, [0.5, 0, 1.5], atol=1e-12)
    np.testing.assert_allclose(ref.look_at, [0.5, 0, 0.0], atol=1e-12)
    np.testing.assert_allclose(ref.rotation[:, 2], [0, 0, -1], atol=1e-12)
    assert ss.segment_length(spec, 0) == pytest.approx(1.0)


def test_arc_reference_follows_axis_direction():
    """@brief 例 5.5.2：axis=[0,0,-1] 走 90° 短弧，中点在圆心 −X 侧；翻转 axis 则走 270° 长弧。"""
    spec = ss.parse_shot(_orbit())
    mid = ss.reference_pose(spec, 0, 0.5)
    np.testing.assert_allclose(mid.position, [1.2 - 1.2007, 0.0, 1.21], atol=2e-3)
    np.testing.assert_allclose(mid.look_at, [1.2, 0.0, 0.9])
    assert ss.segment_length(spec, 0) == pytest.approx(1.2007 * math.pi / 2, rel=1e-3)
    flipped = _orbit()
    flipped['segments'][0]['axis'] = [0, 0, 1]
    spec2 = ss.parse_shot(flipped)
    mid2 = ss.reference_pose(spec2, 0, 0.5)
    np.testing.assert_allclose(mid2.position, [1.2 + 1.2007, 0.0, 1.21], atol=2e-3)
    assert ss.segment_length(spec2, 0) == pytest.approx(1.2007 * 3 * math.pi / 2, rel=1e-3)


def test_arc_reference_endpoints_match_waypoints():
    """@brief 圆弧 s=0 / s=1 精确落在两端 waypoint。"""
    spec = ss.parse_shot(_orbit())
    np.testing.assert_allclose(ss.reference_pose(spec, 0, 0.0).position, spec.waypoints[0].position,
                               atol=1e-9)
    np.testing.assert_allclose(ss.reference_pose(spec, 0, 1.0).position, spec.waypoints[1].position,
                               atol=1e-9)


def test_spline_reference_passes_through_waypoints_and_is_smooth():
    """@brief 例 5.5.1：样条段两端精确过点，中点偏离折线（确实是曲线），且 s 按弧长归一化。"""
    spec = ss.parse_shot(_load('a_5_5_1_reveal.json'))
    np.testing.assert_allclose(ss.reference_pose(spec, 0, 1.0).position, spec.waypoints[1].position,
                               atol=1e-6)
    np.testing.assert_allclose(ss.reference_pose(spec, 1, 0.0).position, spec.waypoints[1].position,
                               atol=1e-6)
    mid = ss.reference_pose(spec, 0, 0.5).position
    chord_mid = 0.5 * (spec.waypoints[0].position + spec.waypoints[1].position)
    assert np.linalg.norm(mid - chord_mid) > 1e-3
    # 弧长归一化：前半段与后半段长度相等
    pts = [ss.reference_pose(spec, 0, s).position for s in np.linspace(0, 1, 201)]
    seg_len = [np.linalg.norm(b - a) for a, b in zip(pts[:-1], pts[1:])]
    assert sum(seg_len[:100]) == pytest.approx(sum(seg_len[100:]), rel=1e-2)


def test_spline_with_two_waypoints_degenerates_to_line():
    """@brief 只有 2 个 waypoint 的 spline 就是直线（5.3.4）。"""
    two = _load('a_5_5_4_topdown_truck.json')
    two['segments'][0]['path'] = 'spline'
    spec = ss.parse_shot(two)
    np.testing.assert_allclose(ss.reference_pose(spec, 0, 0.5).position, [0.5, 0, 1.5], atol=1e-9)


def test_pure_pan_with_tilt_follows_great_circle_of_directions():
    """@brief 5.3.7 字面定义：纯摇镜按光轴**方向**的大圆插值。两端同为俯视 26.6°、yaw 相差 90° 时，
           中点方向 = 归一化(f0 + f1)（大圆中点，俯角比两端更深），而不是绕竖轴的等俯角小圆。"""
    pan = ss.parse_shot({
        'type': 'static',
        'waypoints': [{'position': [0.3, 0, 0.6], 'look_at': [1.3, 0.0, 0.1]},
                      {'position': [0.3, 0, 0.6], 'look_at': [0.3, 1.0, 0.1]}],
        'segments': [{'duration': 2.0}],
        'tolerance': {'position': 0.03, 'aim': 0.02, 'roll': 0.02},
    })
    f0 = np.array([1.0, 0.0, -0.5]) / np.linalg.norm([1.0, 0.0, -0.5])
    f1 = np.array([0.0, 1.0, -0.5]) / np.linalg.norm([0.0, 1.0, -0.5])
    mid = ss.reference_pose(pan, 0, 0.5).rotation[:, 2]
    np.testing.assert_allclose(mid, (f0 + f1) / np.linalg.norm(f0 + f1), atol=1e-9)
    # 大圆插值对"方向夹角 < 180°"总是唯一：前下方 → 右后下方（夹角约 120°）的中点是两方向的角平分线
    pan2 = ss.parse_shot({
        'type': 'static',
        'waypoints': [{'position': [0.3, 0, 0.6], 'look_at': [1.3, 0.0, 0.1]},
                      {'position': [0.3, 0, 0.6], 'look_at': [-0.4, 0.6, 0.1]}],
        'segments': [{'duration': 2.0}],
        'tolerance': {'position': 0.03, 'aim': 0.02, 'roll': 0.02},
    })
    g0 = np.array([1.0, 0.0, -0.5]) / np.linalg.norm([1.0, 0.0, -0.5])
    g1 = np.array([-0.7, 0.6, -0.5]) / np.linalg.norm([-0.7, 0.6, -0.5])
    np.testing.assert_allclose(ss.reference_pose(pan2, 0, 0.5).rotation[:, 2], (g0 + g1) / np.linalg.norm(g0 + g1),
                               atol=1e-9)


def test_pure_pan_interpolates_direction_by_angle():
    """@brief 纯摇镜：位置不变，光轴方向按角度插值（90° 扫角中点正好 45°），roll 线性。"""
    pan = ss.parse_shot({
        'type': 'static',
        'waypoints': [{'position': [0.3, 0, 0.6], 'look_at': [1.3, 0.0, 0.6], 'roll': 0.0},
                      {'position': [0.3, 0, 0.6], 'look_at': [0.3, 1.0, 0.6], 'roll': 0.2}],
        'segments': [{'duration': 2.0}],
        'tolerance': {'position': 0.03, 'aim': 0.02, 'roll': 0.02},
    })
    assert ss.pure_pan(pan, 0)
    ref = ss.reference_pose(pan, 0, 0.5)
    np.testing.assert_allclose(ref.position, [0.3, 0, 0.6], atol=1e-12)
    np.testing.assert_allclose(ref.rotation[:, 2], [math.sqrt(0.5), math.sqrt(0.5), 0], atol=1e-9)
    assert ref.roll == pytest.approx(0.1)
    assert ss.segment_length(pan, 0) == 0.0


def test_reference_roll_interpolates_linearly():
    """@brief 段内 roll 线性插值（5.3.7）。"""
    shot = _load('a_5_5_4_topdown_truck.json')
    shot['waypoints'][1]['roll'] = 0.4
    spec = ss.parse_shot(shot)
    assert ss.reference_pose(spec, 0, 0.25).roll == pytest.approx(0.1)


# ═══════════════════════════════ B 型 → A 型快照展开 ═══════════════════════════════

def _target(x=1.0, y=2.0, z=0.5, yaw=None, ident='trk_1', stamp=100.0):
    """@brief 构造一条 6.2.4 口径的目标数据。"""
    data = {'stamp': stamp, 'id': ident, 'position': [x, y, z], 'confidence': 0.9}
    if yaw is not None:
        data['yaw'] = yaw
    return data


def test_target_data_required_fields():
    """@brief 运行时数据：stamp / id / position 必须有；target_front 还要 yaw（规则 24）。"""
    assert ss.validate_target_data({'position': [0, 0, 0]}, 'target')
    assert ss.validate_target_data(_target(), 'target') == []
    errs = ss.validate_target_data(_target(), 'target_front')
    assert errs and errs[0].rule == 24
    assert ss.validate_target_data(_target(yaw=0.3), 'target_front') == []


def test_tracking_to_static_target_ref_places_camera_on_sphere():
    """@brief position_ref=target：d/az/el → 相机位置 = 目标 + d·(cos el cos az, cos el sin az, sin el)。"""
    spec = ss.parse_shot(_load('b_6_6_4_follow.json'))
    static = ss.tracking_to_static(spec, ss.TargetData.from_dict(_target()))
    assert static.type == 'static'
    wp = static.waypoints[0]
    np.testing.assert_allclose(wp.position, [1.0 + 2.0 * math.cos(0.10), 2.0, 0.5 + 2.0 * math.sin(0.10)])
    assert wp.hold == 0.0


def test_tracking_to_static_az_is_counterclockwise_from_world_x():
    """@brief az=+90° 时相机在目标的世界 +Y 方向（俯视逆时针为正）。"""
    shot = _load('b_6_6_4_follow.json')
    shot['waypoints'][0].update({'az': math.pi / 2, 'el': 0.0, 'uv': [0, 0]})
    static = ss.tracking_to_static(ss.parse_shot(shot), ss.TargetData.from_dict(_target()))
    np.testing.assert_allclose(static.waypoints[0].position, [1.0, 4.0, 0.5], atol=1e-12)


def test_tracking_to_static_target_front_uses_target_yaw():
    """@brief position_ref=target_front：az 零点是目标正面（yaw），az=0 相机就站在目标面朝的方向。"""
    shot = _load('b_6_6_3_target_front.json')
    shot['waypoints'][1].update({'d': 1.5, 'az': 0.0, 'el': 0.0, 'uv': [0, 0]})
    static = ss.tracking_to_static(ss.parse_shot(shot),
                                   ss.TargetData.from_dict(_target(yaw=math.pi / 2)))
    np.testing.assert_allclose(static.waypoints[1].position, [1.0, 3.5, 0.5], atol=1e-12)


def test_tracking_to_static_world_ref_keeps_position_and_aims_at_target():
    """@brief position_ref=world：position 原样保留，uv=[0,0] 时 look_at 就是目标位置。"""
    shot = _load('b_6_6_1_world_push.json')
    for wp in shot['waypoints']:
        wp['uv'] = [0.0, 0.0]
    static = ss.tracking_to_static(ss.parse_shot(shot), ss.TargetData.from_dict(_target()))
    np.testing.assert_allclose(static.waypoints[0].position, [0.40, 0.60, 1.05])
    np.testing.assert_allclose(static.waypoints[0].look_at, [1.0, 2.0, 0.5], atol=1e-9)
    assert static.waypoints[2].hold == 3.0


def test_tracking_to_static_uv_offsets_optical_axis():
    """@brief 非零 uv：展开后的姿态把目标投到画面上正好 (u, v) 的位置。"""
    spec = ss.parse_shot(_load('b_6_6_5_static_orbit.json'))
    static = ss.tracking_to_static(spec, ss.TargetData.from_dict(_target()))
    wp = static.waypoints[1]
    rot = ss.camera_rotation(wp.position, wp.look_at, wp.roll)
    u, v = ss.uv_of(rot, wp.position, [1.0, 2.0, 0.5])
    assert (u, v) == pytest.approx((-0.10, 0.0), abs=1e-6)
    # 视距不变
    assert np.linalg.norm(wp.position - np.array([1.0, 2.0, 0.5])) == pytest.approx(1.10)


def test_tracking_to_static_tolerance_mapping():
    """@brief 容差映射：position = min(d, d·az, d·el)，aim = min(u, v)，roll 原样。"""
    spec = ss.parse_shot(_load('b_6_6_5_static_orbit.json'))
    static = ss.tracking_to_static(spec, ss.TargetData.from_dict(_target()))
    assert static.tolerance.position == pytest.approx(min(0.10, 1.10 * 0.05, 1.10 * 0.05))
    assert static.tolerance.aim == pytest.approx(0.02)
    assert static.tolerance.roll == pytest.approx(0.02)
    world = ss.tracking_to_static(ss.parse_shot(_load('b_6_6_1_world_push.json')),
                                  ss.TargetData.from_dict(_target()))
    assert world.tolerance.position == pytest.approx(0.03)


def test_tracking_to_static_segments_interpolate_in_spherical_coords():
    """@brief 6.4.3：position_ref=target 的段在 (d, az, el) 里插值——例 6.6.5 展开后段中点视距仍是 1.10、
           az / el 各取中值（走的是绕目标的弧，不是弦），弧长大于弦长。"""
    spec = ss.parse_shot(_load('b_6_6_5_static_orbit.json'))
    t = np.array([1.0, 2.0, 0.5])
    static = ss.tracking_to_static(spec, ss.TargetData.from_dict(_target()))
    mid = ss.reference_pose(static, 0, 0.5)
    rel = mid.position - t
    assert np.linalg.norm(rel) == pytest.approx(1.10, abs=1e-9)
    assert math.asin(rel[2] / 1.10) == pytest.approx(0.5 * (0.17 + 0.35), abs=1e-9)
    assert math.atan2(rel[1], rel[0]) == pytest.approx(0.5 * 1.571, abs=1e-9)
    chord = np.linalg.norm(static.waypoints[1].position - static.waypoints[0].position)
    assert ss.segment_length(static, 0) > chord * 1.05
    assert not ss.pure_pan(static, 0)


def test_tracking_to_static_uv_interpolates_linearly_along_segment():
    """@brief 6.4.3：uv 段内线性插值——例 6.6.5 的 u 从 0 到 −0.10，中点处目标落在画面 u=−0.05。"""
    spec = ss.parse_shot(_load('b_6_6_5_static_orbit.json'))
    static = ss.tracking_to_static(spec, ss.TargetData.from_dict(_target()))
    mid = ss.reference_pose(static, 0, 0.5)
    u, v = ss.uv_of(mid.rotation, mid.position, [1.0, 2.0, 0.5])
    assert (u, v) == pytest.approx((-0.05, 0.0), abs=1e-6)


def test_tracking_to_static_world_ref_keeps_cartesian_path_but_tracks_target():
    """@brief position_ref=world：位置仍走世界直线，但每一点的光轴都指向目标（uv 插值）。"""
    shot = _load('b_6_6_1_world_push.json')
    static = ss.tracking_to_static(ss.parse_shot(shot), ss.TargetData.from_dict(_target()))
    mid = ss.reference_pose(static, 0, 0.5)
    np.testing.assert_allclose(mid.position, [0.40, 0.725, 1.05], atol=1e-9)
    u, v = ss.uv_of(mid.rotation, mid.position, [1.0, 2.0, 0.5])
    assert (u, v) == pytest.approx((0.0, -0.10), abs=1e-6)


def test_tracking_to_static_multi_turn_orbit_is_not_zero_length():
    """@brief Δaz = 2π 的整圈环绕：展开后两端位置相同，但段不是零长度纯摇镜（球坐标路径长 2πd·cos el）。"""
    shot = _load('b_6_6_5_static_orbit.json')
    shot['waypoints'][1].update({'az': 2 * math.pi, 'el': 0.17})
    static = ss.tracking_to_static(ss.parse_shot(shot), ss.TargetData.from_dict(_target()))
    assert not ss.pure_pan(static, 0)
    assert ss.segment_length(static, 0) == pytest.approx(2 * math.pi * 1.10 * math.cos(0.17), rel=1e-3)


def test_spherical_spline_is_arc_length_normalized():
    """@brief B 型三点球坐标 spline：s 按弧长归一化，s=0.5 处走过的弧长 ≈ 段长一半。"""
    shot = _load('b_6_6_2_orbit120.json')
    for seg in shot['segments']:
        seg['path'] = 'spline'
    static = ss.tracking_to_static(ss.parse_shot(shot), ss.TargetData.from_dict(_target()))
    pts = [ss.reference_pose(static, 0, s).position for s in np.linspace(0, 1, 401)]
    seg_len = [np.linalg.norm(b - a) for a, b in zip(pts[:-1], pts[1:])]
    assert sum(seg_len[:200]) == pytest.approx(sum(seg_len[200:]), rel=1e-2)


def test_pan_through_zenith_is_rejected():
    """@brief 纯摇镜的大圆经过正上方 / 正下方时画面"上方"无定义（roll 参考翻转）→ 规则 13 报错并说明。"""
    pan = {
        'type': 'static',
        'waypoints': [{'position': [0.3, 0, 0.6], 'look_at': [1.3, 0.0, 0.1]},
                      {'position': [0.3, 0, 0.6], 'look_at': [-0.7, 0.0, 0.1]}],
        'segments': [{'duration': 2.0}],
        'tolerance': {'position': 0.03, 'aim': 0.02, 'roll': 0.02},
    }
    report = ss.validate_shot(pan)
    assert 13 in _rules(report) and any('竖直' in e.message for e in report.errors)
    pan['waypoints'][1]['look_at'] = [-0.7, 0.3, 0.1]      # 稍偏一点就不经过天底
    assert 13 not in _rules(ss.validate_shot(pan))


def test_rule_10_reports_both_plane_and_radius_problems():
    """@brief 规则 10 的两项检查独立：center 既不在端点平面上、两端半径又不等时，两条都报。"""
    bad = _orbit()
    bad['segments'][0]['center'] = [1.20, 0.00, 0.90]
    bad['waypoints'][1]['position'] = [0.6, 0.6, 1.21]
    report = ss.validate_shot(bad)
    assert [e.rule for e in report.errors].count(10) == 2


def test_av_rule_35_only_flags_explicit_values():
    """@brief 规则 35：A 型只对**显式**写出的 SUBJECT_ANCHOR / TARGET 报错，只写 focus.mode 或 audio.enabled 不报。"""
    base = _orbit()
    assert 35 not in _rules(ss.validate_shot(_with_av(base, {'focus': {'mode': 'ONCE'}})))
    assert 35 not in _rules(ss.validate_shot(_with_av(base, {'audio': {'enabled': True}})))
    assert 33 in _rules(ss.validate_shot(_with_av(
        base, {'recording': {'mode': 'VIDEO', 'fps': 60, 'slowmo': True, 'duration_ms': 1000}})))


def test_tracking_to_static_keeps_segments_and_metadata():
    """@brief 展开后 segments 原样、shot_id 等附加字段透传、记录来源目标 id。"""
    shot = _load('b_6_6_2_orbit120.json')
    shot['shot_id'] = 7
    static = ss.tracking_to_static(ss.parse_shot(shot), ss.TargetData.from_dict(_target()))
    assert len(static.segments) == 2 and static.segments[0].duration == 8.0
    assert static.extra['shot_id'] == 7
    assert static.source_target_id == 'trk_1'
