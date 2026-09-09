# -*- coding: utf-8 -*-
"""
@file  test_reach_check.py
@brief robot_arm_api.reach_check 单元测试：URDF 解析、正解、闭式/精解逆解、可达判定、余量、步骤表预检、能力卡。
       参考正解用 pinocchio（若可用），否则只做自洽性检验。
"""

import numpy as np
import pytest

from robot_arm_api import reach_check as rc


# ═══════════════════════════════ ArmModel：建模 ═══════════════════════════════

def test_from_urdf_string_builds_six_joint_chain(urdf_xml):
    """@brief 从完整 URDF 建模：链 arm_base_link → gimbal_tool0 上恰有 6 个转动关节，顺序 Joint1..6，限位取自 URDF。"""
    model = rc.ArmModel.from_urdf_string(urdf_xml)

    assert model.joint_names == ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
    np.testing.assert_allclose(model.lower, [-2.618, -0.981, -2.5, -3.0, -1.5, -1.5])
    np.testing.assert_allclose(model.upper, [2.618, 2.959, 0.02, 3.0, 1.5, 1.5])
    assert model.base_link == 'arm_base_link'
    assert model.tip_link == 'gimbal_tool0'


# ═══════════════════════════════ 正解 ═══════════════════════════════

@pytest.fixture(scope='session')
def model(urdf_xml):
    """@brief 全测试共用的 ArmModel。"""
    return rc.ArmModel.from_urdf_string(urdf_xml)


@pytest.fixture(scope='session')
def pin_fk(urdf_xml):
    """@brief pinocchio 参考正解：pin_fk(q, frame) → 4×4；pinocchio 缺失时 skip。"""
    pin = pytest.importorskip('pinocchio')
    pm = pin.buildModelFromXML(urdf_xml)
    pd = pm.createData()

    def _fk(q, frame):
        pin.framesForwardKinematics(pm, pd, np.asarray(q, dtype=float))
        return np.asarray(pd.oMf[pm.getFrameId(frame)].homogeneous)
    return _fk


def _random_q(model, rng, n):
    """@brief 限位内均匀随机关节角 n×6。"""
    return model.lower + rng.random((n, 6)) * (model.upper - model.lower)


def test_fk_matches_pinocchio(model, pin_fk):
    """@brief 末端 gimbal_tool0 与中间 link tool0 的正解都与 pinocchio 一致（随机 200 组位形）。"""
    rng = np.random.default_rng(0)
    for q in _random_q(model, rng, 200):
        np.testing.assert_allclose(model.fk(q), pin_fk(q, 'gimbal_tool0'), atol=1e-9)
        np.testing.assert_allclose(model.fk(q, 'tool0'), pin_fk(q, 'tool0'), atol=1e-9)


# ═══════════════════════════════ 逆解 ═══════════════════════════════

def test_ik_round_trip_recovers_pose_within_limits(model):
    """@brief 限位内随机位形 → 正解 → 逆解：解必须在限位内，且正解回去与原位姿重合（位置 1e-6 m、姿态 1e-6 rad）。"""
    rng = np.random.default_rng(1)
    for q in _random_q(model, rng, 300):
        target = model.fk(q)
        sol = model.ik(target)
        assert sol is not None, f'逆解失败 q={q}'
        assert np.all(sol >= model.lower - 1e-9) and np.all(sol <= model.upper + 1e-9)
        back = model.fk(sol)
        np.testing.assert_allclose(back[:3, 3], target[:3, 3], atol=1e-6)
        np.testing.assert_allclose(back[:3, :3], target[:3, :3], atol=1e-6)


def test_ik_prefers_solution_near_seed(model):
    """@brief 给种子时，逆解在多解（J1 正向 / 反折）里选离种子最近的那个——种子就是真值时必须原样解回。"""
    rng = np.random.default_rng(2)
    for q in _random_q(model, rng, 100):
        sol = model.ik(model.fk(q), seed=q)
        np.testing.assert_allclose(sol, q, atol=1e-6)


def test_ik_returns_none_when_out_of_reach(model):
    """@brief 明显超出臂长（前方 1.5 m）的位姿逆解返回 None。"""
    target = np.eye(4)
    target[:3, 3] = [1.5, 0.0, 0.5]
    assert model.ik(target) is None


# ═══════════════════════════════ 位姿表示 ═══════════════════════════════

ARM_NOMINAL = np.array([0.0, 1.0, -1.5, 0.0, 0.3, 0.0])


def test_pose_matrix_round_trip():
    """@brief ArmPose 语义（米 / 度，R = Rz(yaw)·Ry(pitch)·Rx(roll)）与 4×4 矩阵互转一致。"""
    pose = {'x': 0.3, 'y': -0.1, 'z': 0.7, 'roll': 5.0, 'pitch': -20.0, 'yaw': 35.0}
    back = rc.matrix_to_pose(rc.pose_to_matrix(pose))
    for key, val in pose.items():
        assert abs(back[key] - val) < 1e-9, key
    # 也接受带属性的对象与 6 元序列
    class P:  # noqa: D401
        x, y, z, roll, pitch, yaw = 0.3, -0.1, 0.7, 5.0, -20.0, 35.0
    np.testing.assert_allclose(rc.pose_to_matrix(P()), rc.pose_to_matrix(pose))
    np.testing.assert_allclose(rc.pose_to_matrix([0.3, -0.1, 0.7, 5.0, -20.0, 35.0]),
                               rc.pose_to_matrix(pose))


def test_pose_from_look_at_matches_commander_aim_convention():
    """@brief look_at → rpy 与 Commander 的 aim_quat 一致：
    yaw = atan2(dy, dx)，pitch = asin(−dz/n)，roll = 0。"""
    level = rc.pose_from_look_at([0.3, 0.0, 0.7], [1.3, 0.0, 0.7])
    assert abs(level['yaw']) < 1e-9 and abs(level['pitch']) < 1e-9 and abs(level['roll']) < 1e-9
    down = rc.pose_from_look_at([0.3, 0.0, 0.7], [1.3, 0.0, 0.2])
    assert abs(down['pitch'] - np.degrees(np.arctan2(0.5, 1.0))) < 1e-9      # 俯视 → pitch 正
    left = rc.pose_from_look_at([0.3, 0.0, 0.7], [0.3, 1.0, 0.7])
    assert abs(left['yaw'] - 90.0) < 1e-9                                      # 看向 +y → yaw +90


# ═══════════════════════════════ check_pose ═══════════════════════════════

def test_check_pose_accepts_comfortable_pose_and_returns_joints(model):
    """@brief arm_nominal 正解出的位姿必须判可达，reason 为空，解回的关节角就是 arm_nominal。"""
    res = rc.check_pose(model, rc.matrix_to_pose(model.fk(ARM_NOMINAL)))
    assert res.ok and res.reason == ''
    np.testing.assert_allclose(res.joints, ARM_NOMINAL, atol=1e-6)
    assert res.margins['reach_m'] > 0 and np.all(res.margins['joints_rad'] > 0)


def test_check_pose_rejects_beyond_reach_with_reason_and_suggestion(model):
    """@brief 前方 0.9 m 超臂长：不可达，reason 提到臂长，suggestion 是一个可达的最近位姿。"""
    res = rc.check_pose(model, {'x': 0.9, 'y': 0.0, 'z': 0.6, 'roll': 0, 'pitch': 0, 'yaw': 0})
    assert not res.ok and '臂长' in res.reason
    assert res.suggestion is not None
    assert rc.check_pose(model, res.suggestion).ok


def test_check_pose_rejects_azimuth_beyond_j1(model):
    """@brief 方位角被 J1 卡住时 reason 提到 J1。
    这台臂 J1 ±150° 的正向 / 反折两支几乎覆盖全部方位，要让 J1 成为瓶颈得把 J1 余量放大：
    余量 2.0 rad → J1 只能用 ±35°，此时正侧方（方位 90°）两支都到不了。"""
    margins = rc.Margins(joint_rad=(2.0, 0.1, 0.1, 0.1, 0.1, 0.1))
    res = rc.check_pose(model, {'x': 0.0, 'y': 0.35, 'z': 0.6, 'roll': 0, 'pitch': 0, 'yaw': 90.0},
                        margins=margins)
    assert not res.ok and 'J1' in res.reason


def test_check_pose_rejects_low_forward_point_with_j2_reason(model):
    """@brief 前下方 (0.45, 0, −0.15)：臂长够，但要 J2 越过上限，reason 提到 J2。"""
    res = rc.check_pose(model, {'x': 0.45, 'y': 0.0, 'z': -0.15, 'roll': 0, 'pitch': 0, 'yaw': 0})
    assert not res.ok and 'J2' in res.reason


def test_check_pose_flags_joint_inside_limit_but_within_margin(model):
    """@brief J5 停在 1.45（距上限 0.05 < 余量 0.10）：限位内但余量不足，不可达且 reason 提到 J5 与余量。"""
    q = ARM_NOMINAL.copy()
    q[4] = 1.45
    res = rc.check_pose(model, rc.matrix_to_pose(model.fk(q)))
    assert not res.ok and 'J5' in res.reason and '余量' in res.reason
    # 余量放宽到 0 就可达
    relaxed = rc.Margins(joint_rad=0.0)
    assert rc.check_pose(model, rc.matrix_to_pose(model.fk(q)), margins=relaxed).ok


# ═══════════════════════════════ headroom（余量盒子） ═══════════════════════════════

def test_headroom_at_arm_nominal_within_sanity_bands(model):
    """@brief arm_nominal 处平移余量落在独立数值分析给出的量级带内：前推 / 上升只剩 5~12 cm（J3 下限卡的），
    横移两侧各 0.2~0.3 m。独立分析按 tool0 纯位置算、忽略云台段 6~9 cm 偏移，故只比量级；精确性由
    下面的端点属性测试保证。两处与纯位置分析的差别都是**姿态固定**带来的，属预期：
    ① dolly 负向：末端沿 y=0.0265 后退正好留在 J1=0 的臂平面内，可越过基座顶部继续向后
       （J2 下限 −0.981 允许上臂后仰），所以只要求 ≤ −0.265；
    ② crane 负向：纯位置能降 0.84 m，但 arm_nominal 的相机仰视 45°，臂下降时小臂俯角变大、
       J5 得补更多仰角，到 −0.32 m 就顶到 J5 余量（实测 J5=1.432 > 1.4），所以带取 [−0.6, −0.2]。"""
    h = rc.headroom(model, ARM_NOMINAL)
    assert 0.05 <= h['dolly'][1] <= 0.12, h['dolly']
    assert h['dolly'][0] <= -0.265, h['dolly']
    assert -0.32 <= h['truck'][0] <= -0.18 and 0.18 <= h['truck'][1] <= 0.32, h['truck']
    assert 0.02 <= h['crane'][1] <= 0.12 and -0.60 <= h['crane'][0] <= -0.20, h['crane']


def test_headroom_translation_endpoints_feasible_and_beyond_not(model):
    """@brief 平移余量区间端点处可达，再多走 2 cm 不可达（continuous=True：从当前位形连续运动、不换分支，
    与余量的语义一致——余量描述的是"从这里连续动能到哪"，不是"换个臂姿能不能到"）。"""
    h = rc.headroom(model, ARM_NOMINAL)
    pose0 = rc.matrix_to_pose(model.fk(ARM_NOMINAL))
    for key, axis in (('dolly', 'x'), ('truck', 'y'), ('crane', 'z')):
        for end in h[key]:
            at_end = dict(pose0)
            at_end[axis] += end
            res_end = rc.check_pose(model, at_end, seed=ARM_NOMINAL, continuous=True)
            assert res_end.ok, (key, end, res_end.reason)
            beyond = dict(pose0)
            beyond[axis] += end + (0.02 if end > 0 else -0.02)
            res_beyond = rc.check_pose(model, beyond, seed=res_end.joints, continuous=True)
            assert not res_beyond.ok, (key, end)


def test_headroom_orientation_intervals(model):
    """@brief 云台 3 轴让朝向几乎自由：arm_nominal 处 yaw 可动 ≥ ±150°；绝对 pitch 至少覆盖 [−70°, +50°]
    （小臂上扬 28°，俯视被 J5 ±86° 减余量卡在 ~59°，仰视宽裕）；区间端点可达、再转 3° 不可达。"""
    h = rc.headroom(model, ARM_NOMINAL)
    assert h['dyaw'][0] <= -150.0 and h['dyaw'][1] >= 150.0, h['dyaw']
    assert h['pitch_abs'][0] <= -70.0 and h['pitch_abs'][1] >= 50.0, h['pitch_abs']
    pose0 = rc.matrix_to_pose(model.fk(ARM_NOMINAL))
    for key, axis in (('dyaw', 'yaw'), ('dpitch', 'pitch')):
        for end in h[key]:
            at_end = dict(pose0)
            at_end[axis] += end
            res_end = rc.check_pose(model, at_end, seed=ARM_NOMINAL, continuous=True)
            assert res_end.ok, (key, end, res_end.reason)
            if abs(at_end[axis] + (3.0 if end > 0 else -3.0)) < 89.0 or axis == 'yaw':
                beyond = dict(pose0)
                beyond[axis] += end + (3.0 if end > 0 else -3.0)
                res_beyond = rc.check_pose(model, beyond, seed=res_end.joints, continuous=True)
                assert not res_beyond.ok, (key, end)


def test_headroom_arc_around_subject(model):
    """@brief 给主体（前方 0.6 m、同高）时多出 arc_az 区间：含 0、两端 ≤ ±45°、端点可达、再多 3° 不可达。"""
    pose0 = rc.matrix_to_pose(model.fk(ARM_NOMINAL))
    center = [pose0['x'] + 0.6, pose0['y'], pose0['z']]
    h = rc.headroom(model, ARM_NOMINAL, subject=center)
    lo, hi = h['arc_az']
    assert lo <= 0.0 <= hi and lo > -45.0 and hi < 45.0, h['arc_az']
    for end, extra in ((lo, -3.0), (hi, 3.0)):
        res_end = rc.check_pose(model, rc.arc_pose(pose0, center, end), seed=ARM_NOMINAL,
                                continuous=True)
        assert res_end.ok, res_end.reason
        res_beyond = rc.check_pose(model, rc.arc_pose(pose0, center, end + extra),
                                   seed=res_end.joints, continuous=True)
        assert not res_beyond.ok


# ═══════════════════════════════ check_plan（步骤表预检） ═══════════════════════════════

def test_check_plan_reports_every_step_and_marks_unchecked_ops(model):
    """@brief 每条步骤都有一份 StepReport；enable/wait/gimbal_* 这类不改臂位置或本模块无法推演的 op
    标 checked=False、ok=True，不影响后续步骤推演。"""
    steps = [{'op': 'enable'}, {'op': 'dolly', 'distance_m': 0.03}, {'op': 'wait', 'seconds': 1.0},
             {'op': 'gimbal_rotate', 'pan': 10.0}]
    rep = rc.check_plan(model, steps, ARM_NOMINAL)
    assert rep.ok and len(rep.steps) == 4
    assert [s.checked for s in rep.steps] == [False, True, False, False]
    assert all(s.ok for s in rep.steps)
    assert rep.steps[1].op == 'dolly' and rep.steps[1].index == 2
    assert abs(rep.pose_end['x'] - (rep.pose_start['x'] + 0.03)) < 1e-6


def test_check_plan_rejects_oversized_dolly_at_right_step(model):
    """@brief [dolly 0.05, dolly 0.30]：第 2 步超出前推余量（arm_nominal 处约 0.10），reject 模式在此停下，
    fraction < 1 且 reason 非空；步骤 1 正常。"""
    steps = [{'op': 'dolly', 'distance_m': 0.05}, {'op': 'dolly', 'distance_m': 0.30}]
    rep = rc.check_plan(model, steps, ARM_NOMINAL)
    assert not rep.ok
    assert rep.steps[0].ok and rep.steps[1].ok is False
    assert rep.steps[1].index == 2 and 0.0 <= rep.steps[1].fraction < 1.0
    assert rep.steps[1].reason and rep.steps[1].waypoint is not None
    assert len(rep.steps) == 2  # reject 模式：后面没有步骤了（这里正好是最后一步）


def test_check_plan_clip_mode_clips_and_continues(model):
    """@brief clip 模式：超余量的 dolly 被夹到边界（剩余余量 ≈ 0.10 − 0.05），记录 clipped，整表 ok，
    后续步骤从夹取后的位姿继续推演。"""
    h = rc.headroom(model, ARM_NOMINAL)
    steps = [{'op': 'dolly', 'distance_m': 0.05}, {'op': 'dolly', 'distance_m': 0.30},
             {'op': 'truck', 'distance_m': 0.05}]
    rep = rc.check_plan(model, steps, ARM_NOMINAL, mode='clip')
    assert rep.ok and len(rep.steps) == 3
    clipped = rep.steps[1].clipped
    assert clipped is not None and abs(clipped['distance_m'] - (h['dolly'][1] - 0.05)) < 0.02
    assert '夹' in rep.steps[1].reason or 'clip' in rep.steps[1].reason.lower()
    assert abs(rep.pose_end['x'] - (rep.pose_start['x'] + h['dolly'][1])) < 0.02
    assert abs(rep.pose_end['y'] - (rep.pose_start['y'] + 0.05)) < 1e-6


def test_check_plan_arc_is_sampled_and_start_point_checked(model):
    """@brief arc：先查起拍点再按方位采样。余量内的小弧 ok；起拍点本身不可达的大弧在 fraction=0 失败。"""
    pose0 = rc.matrix_to_pose(model.fk(ARM_NOMINAL))
    center = [pose0['x'] + 0.6, pose0['y'], pose0['z']]
    theta0, phi0, r0 = rc.cart_to_sphere((pose0['x'], pose0['y'], pose0['z']), center)
    az0 = np.degrees(theta0)
    ok_rep = rc.check_plan(model, [{'op': 'arc', 'center': center, 'radius_m': r0,
                                    'az_start_deg': az0 - 10, 'az_end_deg': az0 + 10,
                                    'elevation_deg': np.degrees(phi0)}], ARM_NOMINAL)
    assert ok_rep.ok, ok_rep.steps[0].reason
    bad_rep = rc.check_plan(model, [{'op': 'arc', 'center': center, 'radius_m': r0,
                                     'az_start_deg': az0 - 80, 'az_end_deg': az0 + 80,
                                     'elevation_deg': np.degrees(phi0)}], ARM_NOMINAL)
    assert not bad_rep.ok and bad_rep.steps[0].fraction == 0.0


def test_check_plan_pose_step_fails_with_suggestion(model):
    """@brief pose 到 0.9 m 外：整点判定（允许换分支），失败时带 suggestion。"""
    rep = rc.check_plan(model, [{'op': 'pose', 'x': 0.9, 'y': 0.0, 'z': 0.6}], ARM_NOMINAL)
    assert not rep.ok and rep.steps[0].suggestion is not None
    assert rc.check_pose(model, rep.steps[0].suggestion).ok


def test_check_plan_linear_and_move_rel_and_joint(model):
    """@brief linear（从当前出发 dx）、move_rel、joint 三种 op 都能推演；joint 直接查关节余量。"""
    steps = [{'op': 'linear', 'dx': 0.03}, {'op': 'move_rel', 'dz': -0.05, 'dyaw': 10.0},
             {'op': 'joint', 'j1': 0.2, 'j2': 1.0, 'j3': -1.5}]
    rep = rc.check_plan(model, steps, ARM_NOMINAL)
    assert rep.ok, [s.reason for s in rep.steps]
    np.testing.assert_allclose(rep.joints_end[:3], [0.2, 1.0, -1.5], atol=1e-9)
    bad = rc.check_plan(model, [{'op': 'joint', 'j1': 0.0, 'j2': 1.0, 'j3': -2.45}], ARM_NOMINAL)
    assert not bad.ok and 'J3' in bad.steps[0].reason


# ═══════════════════════════════ 能力卡 ═══════════════════════════════

def test_capability_data_table_matches_independent_reference(model):
    """@brief 静态表按 r 网格给出臂末端（tool0）离地高范围，与独立数值分析一致到 2 cm：
    r=0.30 → h 0.20~1.09、r=0.55 → 0.34~0.80；r_max 在 0.55~0.60；J1 可用 ±140°。"""
    data = rc.capability_data(model, base_height_m=0.31)
    rows = {round(r['r'], 2): r for r in data['table']}
    assert list(rows) == [0.15, 0.25, 0.30, 0.40, 0.50, 0.55]
    for row in rows.values():
        assert row['h_min'] < row['h_max']
    assert abs(rows[0.30]['h_min'] - 0.20) < 0.02 and abs(rows[0.30]['h_max'] - 1.09) < 0.02
    assert abs(rows[0.55]['h_min'] - 0.34) < 0.02 and abs(rows[0.55]['h_max'] - 0.80) < 0.02
    assert 0.55 < data['r_max'] < 0.60
    assert abs(data['j1_deg'] - 140.0) < 0.5
    assert data['comfort']['r'][0] < data['comfort']['r'][1]
    assert data['comfort']['h'][0] < data['comfort']['h'][1]
    ori = data['orientation']
    # 舒适区中心（r=0.35、h≈0.9）相机水平时小臂是俯着的：仰视余量被 J5 卡在 ~40°，俯视宽裕
    assert ori['yaw_rel_deg'] >= 150.0
    assert ori['pitch_deg'][0] <= -30.0 and ori['pitch_deg'][1] >= 45.0


def test_capability_card_text_and_current_state(model):
    """@brief 能力卡文本含静态表、J1 范围、朝向规则；给关节角时多出"当前状态"段与余量区间。"""
    text = rc.capability_card(model)
    assert 'r=0.30' in text and '不可达' in text and '±140°' in text and '当前状态' not in text
    text2 = rc.capability_card(model, joints=ARM_NOMINAL)
    assert '当前状态' in text2 and 'dolly' in text2 and 'crane' in text2 and 'pitch' in text2


def test_headroom_schema_encodes_intervals_per_op(model):
    """@brief headroom → JSON schema：oneOf 里每个 op 一项，distance_m 的 minimum/maximum 就是余量区间；
    有主体时含 arc 项。"""
    pose0 = rc.matrix_to_pose(model.fk(ARM_NOMINAL))
    h = rc.headroom(model, ARM_NOMINAL, subject=[pose0['x'] + 0.6, pose0['y'], pose0['z']])
    schema = rc.headroom_schema(h)
    by_op = {item['properties']['op']['const']: item for item in schema['oneOf']}
    for op in ('dolly', 'truck', 'crane'):
        assert by_op[op]['properties']['distance_m']['minimum'] == pytest.approx(h[op][0])
        assert by_op[op]['properties']['distance_m']['maximum'] == pytest.approx(h[op][1])
    az_min = by_op['arc']['properties']['az_end_deg']['minimum']
    assert az_min == pytest.approx(h['arc']['az0_deg'] + h['arc_az'][0])
    assert 'move_rel' in by_op
    assert by_op['move_rel']['properties']['dyaw']['maximum'] == pytest.approx(h['dyaw'][1])


# ═══════════════════════════════ from_share_files ═══════════════════════════════

def test_from_share_files_matches_urdf_string_model(model):
    """@brief 从安装的描述包现场 xacro 展开建模，与直接喂 URDF 字符串得到同一套模型。"""
    try:
        other = rc.ArmModel.from_share_files()
    except RuntimeError as exc:
        pytest.skip(str(exc))
    assert other.joint_names == model.joint_names
    np.testing.assert_allclose(other.lower, model.lower)
    np.testing.assert_allclose(other.upper, model.upper)
    np.testing.assert_allclose(other.fk(ARM_NOMINAL), model.fk(ARM_NOMINAL), atol=1e-12)
