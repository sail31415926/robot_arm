# -*- coding: utf-8 -*-
"""
@file  test_shot_compiler.py
@brief robot_arm_api.shot_compiler 单元测试：坐标系（odom→arm_base、光心↔法兰）、规范 A 型段 → Commander
       原语（LINEAR / ORBIT / PTP）的选择与细分、速度档位量化与降级标记、可达性校验（规则 25/26）、
       输出步骤表与 run_plan_step 的 op 兼容。需要 conftest 的 urdf_xml（source 了 ROS 才有，否则 skip）。
"""

import json
import math
import os

import numpy as np
import pytest

from robot_arm_api import reach_check as rc
from robot_arm_api import shot_compiler as sc
from robot_arm_api import shot_spec as ss

_LLM_EXAMPLE = '/home/qwe/eMeetWork_sail/摄影机器人规范/大模型输出示例/output/child.json'


@pytest.fixture(scope='session')
def model(urdf_xml):
    """@brief 全测试共用的 ArmModel（末端 gimbal_tool0）。"""
    return rc.ArmModel.from_urdf_string(urdf_xml)


@pytest.fixture(scope='session')
def frames(urdf_xml):
    """@brief 从同一份 URDF 取 gimbal_tool0 → camera_optical_frame 的固定变换。"""
    return sc.CameraFrames.from_urdf(urdf_xml)


@pytest.fixture
def compiler(model, frames):
    """@brief 世界系 = 机械臂基座系的编译器。"""
    return sc.ShotCompiler(model, frames=frames)


def _static(waypoints, segments, tolerance=None):
    """@brief 组一条 A 型 spec。"""
    return ss.parse_shot({'type': 'static', 'waypoints': waypoints, 'segments': segments,
                          'tolerance': tolerance or {'position': 0.03, 'aim': 0.03, 'roll': 0.02}})


def _wp(position, look_at, **kw):
    """@brief 组一个 A 型 waypoint。"""
    out = {'position': list(position), 'look_at': list(look_at)}
    out.update(kw)
    return out


# ═══════════════════════════════ 坐标系 ═══════════════════════════════

def test_camera_frames_from_urdf_matches_default_constant(frames):
    """@brief URDF 里 gimbal_tool0→camera_optical_frame：光轴 +Z = 法兰 +X，偏移约 4.4cm，与内置常量一致。"""
    tf = frames.tool0_to_optical
    np.testing.assert_allclose(tf, sc.DEFAULT_TOOL0_TO_OPTICAL, atol=1e-6)
    np.testing.assert_allclose(tf[:3, 2], [1, 0, 0], atol=1e-9)       # 光学 +Z（前）= 法兰 +X
    np.testing.assert_allclose(tf[:3, 0], [0, -1, 0], atol=1e-9)      # 光学 +X（右）= 法兰 −Y
    assert np.linalg.norm(tf[:3, 3]) == pytest.approx(0.0437, abs=5e-4)


def test_flange_pose_roundtrip_and_offset(frames):
    """@brief 光心位姿 → 法兰 ArmPose → 光心位姿 往返一致；法兰与光心相距 4.4cm。"""
    p_cam = np.array([0.40, 0.05, 0.60])
    rot = ss.camera_rotation(p_cam, [0.9, 0.0, 0.5], 0.1)
    pose = frames.flange_pose(rot, p_cam)
    assert set(pose) == {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}
    rot2, p2 = frames.optical_from_flange(pose)
    np.testing.assert_allclose(p2, p_cam, atol=1e-9)
    np.testing.assert_allclose(rot2, rot, atol=1e-9)
    flange = np.array([pose['x'], pose['y'], pose['z']])
    assert np.linalg.norm(flange - p_cam) == pytest.approx(0.0437, abs=5e-4)


def test_flange_pose_roll_sign_and_level_aim_match_commander_convention(frames):
    """@brief roll=0 时法兰姿态与 Commander 的 aim_quat（pose_from_look_at）同一个旋转；spec roll +0.1 rad →
           ArmPose.roll ≈ +5.73°（同号）。"""
    p_cam = np.array([0.40, 0.0, 0.60])
    look = np.array([0.9, 0.1, 0.5])
    pose0 = frames.flange_pose(ss.camera_rotation(p_cam, look, 0.0), p_cam)
    ref = rc.pose_to_matrix(rc.pose_from_look_at(p_cam, look))
    np.testing.assert_allclose(rc.pose_to_matrix(pose0)[:3, :3], ref[:3, :3], atol=1e-9)
    assert pose0['roll'] == pytest.approx(0.0, abs=1e-9)
    pose1 = frames.flange_pose(ss.camera_rotation(p_cam, look, 0.1), p_cam)
    assert pose1['roll'] == pytest.approx(math.degrees(0.1), abs=1e-6)


def test_flange_pose_straight_down_survives_gimbal_lock(frames):
    """@brief 垂直俯视（pitch=±90°）时 rpy 退化，法兰位姿仍要能还原出同一个旋转。"""
    p_cam = np.array([0.35, 0.0, 0.7])
    rot = ss.camera_rotation(p_cam, [0.35, 0.0, 0.2], 0.3)
    pose = frames.flange_pose(rot, p_cam)
    rot2, _ = frames.optical_from_flange(pose)
    np.testing.assert_allclose(rot2, rot, atol=1e-6)


def test_world_frame_from_chassis():
    """@brief odom 系 → 机械臂基座系：底盘在 (1, 2, ψ=90°)，臂基座离地 0.31。"""
    world = sc.WorldFrame.from_chassis(1.0, 2.0, math.pi / 2, arm_base_height=0.31)
    np.testing.assert_allclose(world.to_arm_point([1.0, 2.5, 0.31]), [0.5, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(world.to_world_point([0.5, 0.0, 0.0]), [1.0, 2.5, 0.31], atol=1e-12)
    rot_world = ss.camera_rotation([0, 0, 0], [0, 1, 0], 0.0)         # 世界系里看向 +Y
    rot_arm = world.to_arm_rotation(rot_world)
    np.testing.assert_allclose(rot_arm[:, 2], [1, 0, 0], atol=1e-12)  # 臂系里就是 +X
    assert sc.WorldFrame.identity().to_arm_point([0.1, 0.2, 0.3]).tolist() == [0.1, 0.2, 0.3]


# ═══════════════════════════════ 原语选择 ═══════════════════════════════

def test_single_waypoint_compiles_to_ptp_then_hold(compiler):
    """@brief 1 点 0 段（例 5.5.5 形态）：一个 PTP 原语 + 该点的 hold。"""
    spec = _static([_wp((0.40, 0.0, 0.55), (0.9, 0.0, 0.45), hold=2.0)], [])
    out = compiler.compile(spec)
    assert [p.kind for p in out.primitives] == ['ptp']
    assert out.primitives[0].segment_index == -1
    assert out.holds == [2.0]
    steps = out.plan_steps()
    assert steps[0]['op'] == 'pose' and steps[-1] == {'op': 'wait', 'seconds': 2.0}
    assert out.degradation == 'none'


def _arc_spec(axis=(0, 0, -1), tolerance=None, center=(0.40, 0.0, 0.55), radius=0.12):
    """@brief 平视绕被摄物的水平圆弧：圆心 = 被摄物（同高）。端点在圆心近侧（−X 方向）±50°：
           axis=[0,0,-1] 走近侧 100° 短弧（俯视顺时针），axis=[0,0,1] 走远侧 260° 长弧。"""
    c = np.array(center)
    p0 = c + radius * np.array([-math.cos(math.radians(50)), -math.sin(math.radians(50)), 0.0])
    p1 = c + radius * np.array([-math.cos(math.radians(50)), math.sin(math.radians(50)), 0.0])
    return _static([_wp(p0, c), _wp(p1, c, hold=1.0)],
                   [{'path': 'arc', 'center': c.tolist(), 'axis': list(axis), 'duration': 8.0}],
                   tolerance)


def _orbit_optical(prim, frames, s):
    """@brief 按 Commander ORBIT 的几何（球坐标线性插值、法兰 +X 对准球心、roll 0）还原进度 s 处的光心位姿。"""
    az = math.radians(prim.az0 + s * (prim.az1 - prim.az0))
    el = math.radians(prim.el0 + s * (prim.el1 - prim.el0))
    r = prim.r0 + s * (prim.r1 - prim.r0)
    pos_f = rc.sphere_to_cart(az, el, r, prim.center)
    return frames.optical_from_flange(rc.pose_from_look_at(pos_f, prim.center))


def test_arc_with_constant_look_at_compiles_to_one_orbit(compiler, frames):
    """@brief look_at 恒定的平视圆弧 = Commander 球面环绕：法兰球心 = look_at 抬高一个光心偏置（2.44 cm），
           法兰半径 = 光心半径 + 3.62 cm，一条 ORBIT 原语精确复现光心轨迹。"""
    spec = _arc_spec()
    out = compiler.compile(spec)
    assert [p.kind for p in out.primitives] == ['orbit']
    prim = out.primitives[0]
    dz = -frames.tool0_to_optical[2, 3]
    np.testing.assert_allclose(prim.center, [0.40, 0.0, 0.55 + dz], atol=1e-6)
    assert prim.r0 == pytest.approx(0.12 + frames.tool0_to_optical[0, 3], abs=1e-6)
    assert prim.r1 == pytest.approx(prim.r0, abs=1e-6)
    assert prim.el0 == pytest.approx(0.0, abs=1e-6) and prim.el1 == pytest.approx(0.0, abs=1e-6)
    assert abs(prim.az1 - prim.az0) == pytest.approx(100.0, abs=1e-6)
    for s in (0.0, 0.25, 0.5, 0.75, 1.0):
        rot, pos = _orbit_optical(prim, frames, s)
        ref = ss.reference_pose(spec, 0, s)
        np.testing.assert_allclose(pos, ref.position, atol=1e-6)
        np.testing.assert_allclose(rot, ref.rotation, atol=1e-6)
    assert prim.progress_offset == 60.0        # ORBIT：0→20 PTP、20→60 规划、60→100 运镜
    assert out.holds == [0.0, 1.0]
    assert not any('细分' in w for w in out.warnings)


def test_long_arc_toward_base_is_rejected_by_gimbal_pan_limit(compiler):
    """@brief 翻转 axis 走 260° 长弧要经过被摄物远侧、镜头回望机械臂基座：云台 pan（J4）顶限位 → 规则 26。"""
    spec = _arc_spec(axis=(0, 0, 1), center=(0.38, 0.0, 0.45), radius=0.08)
    with pytest.raises(ss.SpecValidationError) as exc:
        compiler.compile(spec)
    errs = exc.value.report.errors
    assert errs[0].rule == 26 and 'J4' in errs[0].message


def test_orbit_sweep_direction_follows_axis(compiler, frames, monkeypatch):
    """@brief 只看几何（跳过 IK）：同样两个端点，axis=[0,0,1] 编成 260° 的 ORBIT，中点在圆心远侧，
           且整条 ORBIT 路径与规范圆弧一致（验证 θ1 ± 2π 的转向选择）。"""
    monkeypatch.setattr(compiler, '_check_path', lambda i, sampler, seed, length, ang, report: seed)
    spec = _arc_spec(axis=(0, 0, 1), center=(0.38, 0.0, 0.45), radius=0.08)
    out = compiler.compile(spec)
    assert [p.kind for p in out.primitives] == ['orbit']
    prim = out.primitives[0]
    assert abs(prim.az1 - prim.az0) == pytest.approx(260.0, abs=1e-6)
    for s in (0.25, 0.5, 0.75):
        _, pos = _orbit_optical(prim, frames, s)
        np.testing.assert_allclose(pos, ss.reference_pose(spec, 0, s).position, atol=1e-6)
    _, mid = _orbit_optical(prim, frames, 0.5)
    assert mid[0] > 0.38 + 0.07


def test_truck_with_parallel_look_at_is_one_linear(compiler):
    """@brief 例 5.5.4 形态：look_at 随机位平移、光轴方向不变 → slerp 精确，一条 LINEAR 原语。"""
    spec = _static([_wp((0.30, -0.10, 0.60), (0.70, -0.10, 0.45)),
                    _wp((0.30, 0.10, 0.60), (0.70, 0.10, 0.45))],
                   [{'path': 'line', 'duration': 4.0}])
    out = compiler.compile(spec)
    assert [p.kind for p in out.primitives] == ['linear']
    prim = out.primitives[0]
    assert prim.progress_offset == 50.0 and (prim.s0, prim.s1) == (0.0, 1.0)
    start_rot, start_p = compiler.frames.optical_from_flange(prim.start)
    np.testing.assert_allclose(start_p, [0.30, -0.10, 0.60], atol=1e-9)
    end_rot, end_p = compiler.frames.optical_from_flange(prim.end)
    np.testing.assert_allclose(end_p, [0.30, 0.10, 0.60], atol=1e-9)
    np.testing.assert_allclose(start_rot, end_rot, atol=1e-9)


def test_line_past_close_subject_is_subdivided_by_aim_tolerance(model, frames):
    """@brief 直线掠过近处主体、look_at 恒定：姿态 slerp 与"锁定目标点"偏差大 → 按 aim 容差细分成多段 LINEAR；
           容差放宽则一段就够。细分后的 s 区间连续覆盖 [0, 1]，每段端点都在参考路径上。"""
    waypoints = [_wp((0.30, -0.12, 0.60), (0.40, 0.0, 0.55)), _wp((0.30, 0.12, 0.60), (0.40, 0.0, 0.55))]
    tight = _static(waypoints, [{'duration': 4.0}], {'position': 0.03, 'aim': 0.02, 'roll': 0.02})
    out = sc.ShotCompiler(model, frames=frames).compile(tight)
    kinds = [p.kind for p in out.primitives]
    assert len(kinds) >= 2 and set(kinds) == {'linear'}
    assert out.primitives[0].s0 == 0.0 and out.primitives[-1].s1 == 1.0
    for a, b in zip(out.primitives[:-1], out.primitives[1:]):
        assert a.s1 == pytest.approx(b.s0)
        assert a.end == b.start
    for prim in out.primitives:
        _, p = frames.optical_from_flange(prim.end)
        np.testing.assert_allclose(p, ss.reference_pose(tight, 0, prim.s1).position, atol=1e-9)
    assert any('细分' in w for w in out.warnings)
    loose = _static(waypoints, [{'duration': 4.0}], {'position': 0.03, 'aim': 1.0, 'roll': 0.5})
    assert len(sc.ShotCompiler(model, frames=frames).compile(loose).primitives) == 1


def test_pure_pan_compiles_to_linear_with_fixed_position(compiler):
    """@brief 水平纯摇镜：光心位置不变、只转朝向 → 一条 LINEAR（Commander 对姿态做 slerp；两端都平视时
           slerp 就是绕竖轴匀速摇，与规范的方向大圆一致）；法兰因 4.4 cm 偏置会画一小段弦，光心不动。"""
    spec = _static([_wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.60)), _wp((0.35, 0.0, 0.60), (0.35, 0.5, 0.60))],
                   [{'duration': 8.0}])
    out = compiler.compile(spec)
    assert [p.kind for p in out.primitives] == ['linear']
    prim = out.primitives[0]
    _, p_start = compiler.frames.optical_from_flange(prim.start)
    _, p_end = compiler.frames.optical_from_flange(prim.end)
    np.testing.assert_allclose(p_start, p_end, atol=1e-9)
    assert prim.speed == 'fast'                         # 90° / 0.20 rad/s ≈ 7.85 s → 最接近 8 s 的档
    assert prim.nominal_sec == pytest.approx(math.pi / 2 / 0.20, rel=1e-6)


def test_tilted_pan_is_subdivided_to_follow_direction_great_circle(compiler):
    """@brief 带俯角的 90° 摇：规范按光轴方向大圆插值（中点俯角更深），Commander 的 slerp 走等俯角小圆，
           偏差超 aim 容差预算 → 细分成多条 LINEAR，并告警。"""
    spec = _static([_wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50)), _wp((0.35, 0.0, 0.60), (0.35, 0.5, 0.50))],
                   [{'duration': 8.0}], {'position': 0.03, 'aim': 0.02, 'roll': 0.02})
    out = compiler.compile(spec)
    assert len(out.primitives) >= 2 and {p.kind for p in out.primitives} == {'linear'}
    assert any('细分' in w for w in out.warnings)


# ═══════════════════════════════ 速度档位 ═══════════════════════════════

def _dolly_spec(seg):
    """@brief 沿 +X 推 10cm、光轴方向不变的直线段。"""
    return _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)), _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50))],
                   [seg])


def test_duration_picks_nearest_speed_level(compiler):
    """@brief 0.10 m 走 2 s = 0.05 m/s 正好是 normal 档，无降级。"""
    out = compiler.compile(_dolly_spec({'duration': 2.0}))
    assert out.primitives[0].speed == 'normal'
    assert out.primitives[0].nominal_sec == pytest.approx(2.0)
    assert out.degradation == 'none'
    assert sc.SPEED_PROFILES['normal'].v_pos == 0.05


def test_too_fast_request_uses_fast_and_marks_slowed(compiler):
    """@brief 要求 0.5 s 走 10cm（0.2 m/s）超过 fast 档上限 → 用 fast，degradation=slowed 并告警。"""
    out = compiler.compile(_dolly_spec({'duration': 0.5}))
    assert out.primitives[0].speed == 'fast'
    assert out.degradation == 'slowed'
    assert any('slow' in w or '慢' in w for w in out.warnings)


def test_too_slow_request_uses_slow_and_warns(compiler):
    """@brief 要求 20 s 走 10cm（0.005 m/s）比 slow 档还慢 → 用 slow，只告警（会比要求的快），不算 slowed。"""
    out = compiler.compile(_dolly_spec({'duration': 20.0}))
    assert out.primitives[0].speed == 'slow'
    assert out.degradation == 'none'
    assert any('快' in w for w in out.warnings)


def test_speed_field_maps_to_level(compiler):
    """@brief segment 用 speed（m/s）而不是 duration：0.02 m/s → slow。"""
    out = compiler.compile(_dolly_spec({'speed': 0.02}))
    assert out.primitives[0].speed == 'slow'


def test_last_segment_law_with_end_velocity_marks_slowed(compiler):
    """@brief 规范 4.1：末段 law 为 constant / ease_in（末端有速度）时执行层会减速停住并标 slowed。"""
    out = compiler.compile(_dolly_spec({'duration': 2.0, 'law': 'constant'}))
    assert out.degradation == 'slowed'
    assert any('law' in w for w in out.warnings)


# ═══════════════════════════════ 可达性（规则 25 / 26） ═══════════════════════════════

def test_unreachable_waypoint_raises_rule_25(compiler):
    """@brief 大模型示例 child.json 第 1 镜（相机离主体 2 m）对单臂不可达 → 规则 25，带 check_pose 的原因。"""
    if not os.path.exists(_LLM_EXAMPLE):
        pytest.skip('大模型输出示例不在本机')
    with open(_LLM_EXAMPLE, encoding='utf-8') as fh:
        shot = json.load(fh)['agent_outputs']['cinematographer']['shot_plans'][0]
    with pytest.raises(ss.SpecValidationError) as exc:
        compiler.compile(ss.parse_shot(shot))
    errs = exc.value.report.errors
    assert errs[0].rule == 25 and errs[0].field == 'waypoints[0]'
    assert '臂长' in errs[0].message


def test_path_through_base_column_raises_rule_26(model, frames):
    """@brief 两端可达、中间穿过机械臂立柱的半圆 → 沿路径采样校验报规则 26。"""
    compiler = sc.ShotCompiler(model, frames=frames)
    c = [0.30, 0.0, 0.35]
    p0, p1 = (0.30, 0.28, 0.35), (0.30, -0.28, 0.35)
    for p in (p0, p1):   # 前置：端点本身可达，否则报的会是规则 25
        rot = ss.camera_rotation(p, c, 0.0)
        assert rc.check_pose(model, frames.flange_pose(rot, np.asarray(p))).ok
    spec = _static([_wp(p0, c), _wp(p1, c)],
                   [{'path': 'arc', 'center': c, 'axis': [0, 0, 1], 'duration': 10.0}])
    with pytest.raises(ss.SpecValidationError) as exc:
        compiler.compile(spec)
    rules = {e.rule for e in exc.value.report.errors}
    assert 26 in rules and 25 not in rules


# ═══════════════════════════════ 世界系与输出 ═══════════════════════════════

def test_world_frame_shifts_flange_poses(model, frames):
    """@brief 同一条镜头：odom 系写法 + 底盘位姿 与 直接写在臂系里 编译出同样的法兰位姿。"""
    world = sc.WorldFrame.from_chassis(0.2, 0.0, 0.0, arm_base_height=0.31)
    spec_world = _static([_wp((0.50, 0.0, 0.91), (1.00, 0.0, 0.81)), _wp((0.60, 0.0, 0.91), (1.10, 0.0, 0.81))],
                         [{'duration': 2.0}])
    spec_arm = _dolly_spec({'duration': 2.0})
    out_w = sc.ShotCompiler(model, world=world, frames=frames).compile(spec_world)
    out_a = sc.ShotCompiler(model, frames=frames).compile(spec_arm)
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        assert out_w.primitives[0].start[key] == pytest.approx(out_a.primitives[0].start[key], abs=1e-9)
        assert out_w.primitives[0].end[key] == pytest.approx(out_a.primitives[0].end[key], abs=1e-9)


def test_plan_steps_are_run_plan_step_compatible(compiler):
    """@brief 输出步骤表只用 run_plan_step 认识的 op 与参数名。"""
    out = compiler.compile(_arc_spec())
    steps = out.plan_steps()
    assert {s['op'] for s in steps} <= {'pose', 'linear', 'orbit', 'wait'}
    orbit = next(s for s in steps if s['op'] == 'orbit')
    assert set(orbit) >= {'center', 'az_start_deg', 'az_end_deg', 'el_start_deg', 'el_end_deg',
                          'r_start_m', 'r_end_m', 'speed'}
    out2 = compiler.compile(_dolly_spec({'duration': 2.0}))
    linear = out2.plan_steps()[0]
    assert linear['op'] == 'linear' and set(linear['start']) == {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}
    assert linear['speed'] in ('slow', 'normal', 'fast')


def test_plan_steps_shorten_intermediate_holds_by_dwell(compiler):
    """@brief 步骤表里的 wait 与执行器同一口径：后面还有 goal 的 hold 扣掉 Commander 1 s 起点停顿，末点不扣。"""
    spec = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50), hold=1.0),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50), hold=3.0),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50), hold=1.5)],
                   [{'duration': 1.0}, {'duration': 1.0}])
    steps = compiler.compile(spec).plan_steps()
    waits = [s['seconds'] for s in steps if s['op'] == 'wait']
    assert waits == pytest.approx([3.0 - compiler.dwell_sec, 1.5])   # 首点 hold 1 s 被 1 s 停顿吃满，不再单独 wait
    assert steps[0]['op'] == 'pose'


def test_primitive_progress_from_pose(compiler, frames):
    """@brief primitive_progress：由当前法兰位姿在原语几何上投影得到进度（LINEAR 投到弦上、ORBIT 取方位角占比、
           纯摇镜取转角占比），不依赖 Commander 的 progress_percent。"""
    lin = compiler.compile(_dolly_spec({'duration': 2.0})).primitives[0]
    mid = {k: 0.5 * (lin.start[k] + lin.end[k]) for k in lin.start}
    assert sc.primitive_progress(lin, mid) == pytest.approx(0.5, abs=1e-6)
    assert sc.primitive_progress(lin, lin.start) == pytest.approx(0.0, abs=1e-9)
    assert sc.primitive_progress(lin, lin.end) == pytest.approx(1.0, abs=1e-9)
    orb = compiler.compile(_arc_spec()).primitives[0]
    for s in (0.0, 0.3, 1.0):
        pose = rc.pose_from_look_at(rc.sphere_to_cart(math.radians(orb.az0 + s * (orb.az1 - orb.az0)),
                                                       math.radians(orb.el0), orb.r0, orb.center), orb.center)
        assert sc.primitive_progress(orb, pose) == pytest.approx(s, abs=1e-6)
    pan = compiler.compile(_static([_wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.60)),
                                    _wp((0.35, 0.0, 0.60), (0.35, 0.5, 0.60))],
                                   [{'duration': 8.0}])).primitives[0]
    t0, t1 = rc.pose_to_matrix(pan.start), rc.pose_to_matrix(pan.end)
    half = frames.pose_from_flange_matrix(sc.ShotCompiler._linear_sampler(t0, t1)(0.5))
    assert sc.primitive_progress(pan, half) == pytest.approx(0.5, abs=1e-6)


def test_full_circle_tracking_orbit_compiles_to_one_orbit(compiler, frames, monkeypatch):
    """@brief B 型整圈环绕（Δaz = 2π，两端位置相同）：跳过 IK 只看几何，应编成一条 |Δaz|=360° 的 ORBIT，
           且 ORBIT 路径中点在目标远侧（与球坐标参考一致）。"""
    monkeypatch.setattr(compiler, '_check_path', lambda i, sampler, seed, length, ang, report: seed)
    shot = {'type': 'tracking', 'target': '/subject_pose', 'position_ref': 'target',
            'waypoints': [{'d': 0.12, 'az': math.pi, 'el': 0.0}, {'d': 0.12, 'az': 3 * math.pi, 'el': 0.0, 'hold': 1.0}],
            'segments': [{'duration': 12.0}],
            'tolerance': {'d': 0.05, 'az': 0.05, 'el': 0.05, 'u': 0.02, 'v': 0.02, 'roll': 0.02}}
    static = ss.tracking_to_static(ss.parse_shot(shot),
                                   ss.TargetData.from_dict({'stamp': 0.0, 'id': 't', 'position': [0.40, 0.0, 0.55]}))
    out = compiler.compile(static)
    assert [p.kind for p in out.primitives] == ['orbit']
    prim = out.primitives[0]
    assert abs(prim.az1 - prim.az0) == pytest.approx(360.0, abs=1e-6)
    for s in (0.25, 0.5, 0.75):
        _, pos = _orbit_optical(prim, frames, s)
        np.testing.assert_allclose(pos, ss.reference_pose(static, 0, s).position, atol=1e-6)


def test_primitive_progress_on_pan_chord_is_robust_to_position_noise(compiler, frames):
    """@brief 纯摇镜的法兰弦只有几厘米：进度改按转角占比，2 mm 位置噪声下进度误差 < 0.05。"""
    pan = compiler.compile(_static([_wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.60)),
                                    _wp((0.35, 0.0, 0.60), (0.35, 0.15, 0.60))],
                                   [{'duration': 4.0}])).primitives[0]
    t0, t1 = rc.pose_to_matrix(pan.start), rc.pose_to_matrix(pan.end)
    rng = np.random.default_rng(1)
    for u in (0.2, 0.5, 0.8):
        t_mid = sc.ShotCompiler._linear_sampler(t0, t1)(u)
        noisy = t_mid.copy()
        noisy[:3, 3] += rng.normal(0.0, 0.002, 3)
        assert sc.primitive_progress(pan, frames.pose_from_flange_matrix(noisy)) == pytest.approx(u, abs=0.05)


def test_nominal_total_includes_holds_and_dwell(compiler):
    """@brief 名义总时长 = 各原语名义时长 + 每条原语前 Commander 的 1 s 起点停顿 + 各点 hold。"""
    out = compiler.compile(_arc_spec())
    prim = out.primitives[0]
    assert out.nominal_total_sec == pytest.approx(prim.nominal_sec + compiler.dwell_sec + 1.0)


def test_pose_error_against_reference(compiler):
    """@brief pose_error：法兰位姿正好是参考位姿时三项误差为 0；绕相机"下"轴转 0.1 rad → aim 误差 0.1、roll 0；
           绕光轴转 0.05 → roll 误差 0.05、aim 0；平移 2 cm → 位置误差 0.02。"""
    pos = np.array([0.35, 0.0, 0.6])
    rot = ss.camera_rotation(pos, [0.85, 0.0, 0.5], 0.0)
    ref = ss.CamRef(pos, rot, np.array([0.85, 0.0, 0.5]), 0.0)
    exact = compiler.frames.flange_pose(rot, pos)
    err = compiler.pose_error(ref, exact)
    assert err == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    yawed = compiler.frames.flange_pose(ss._rodrigues(rot[:, 1], 0.1) @ rot, pos)
    e_pos, e_aim, e_roll = compiler.pose_error(ref, yawed)
    assert e_pos == pytest.approx(0.0, abs=1e-9) and e_aim == pytest.approx(0.1, abs=1e-6)
    assert e_roll == pytest.approx(0.0, abs=1e-6)
    rolled = compiler.frames.flange_pose(rot @ ss._rodrigues(np.array([0, 0, 1.0]), 0.05), pos)
    _, e_aim2, e_roll2 = compiler.pose_error(ref, rolled)
    assert e_aim2 == pytest.approx(0.0, abs=1e-9) and e_roll2 == pytest.approx(0.05, abs=1e-6)
    shifted = compiler.frames.flange_pose(rot, pos + [0.0, 0.02, 0.0])
    assert compiler.pose_error(ref, shifted)[0] == pytest.approx(0.02, abs=1e-9)
