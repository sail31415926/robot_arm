# -*- coding: utf-8 -*-
"""
@file  test_shot_executor.py
@brief robot_arm_api.shot_executor 单元测试：用假 ArmApi（记录调用、按进度回放反馈）验证一段规范 JSON 从解析、
       编译到逐条下发 Commander goal 的流程，以及第 9 章反馈契约（phase / segment_index / progress / error /
       in_tolerance / deviation_cause / degradation / target_status）与 B 型目标快照的 id / moved / stale 处理。
       需要 conftest 的 urdf_xml（可达性判定要真模型）。
"""

import json
import math
import os

import numpy as np
import pytest

from robot_arm_api import reach_check as rc
from robot_arm_api import shot_compiler as sc
from robot_arm_api import shot_executor as se
from robot_arm_api import shot_spec as ss
from robot_arm_api.arm_commander_client import CallResult

_LLM_EXAMPLE = '/home/qwe/eMeetWork_sail/摄影机器人规范/大模型输出示例/output/child.json'


# ═══════════════════════════════ 假 ArmApi ═══════════════════════════════

class _Feedback:
    """@brief 假的 action 反馈消息（只带 progress_percent / current_pose）。"""

    def __init__(self, progress_percent, current_pose):
        self.progress_percent = progress_percent
        self.current_pose = current_pose


class _Pose:
    """@brief 假 ArmPose msg（属性 x y z roll pitch yaw）。"""

    def __init__(self, d):
        for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            setattr(self, k, float(d[k]))


class FakeArm:
    """@brief 假 ArmCommanderClient：记录每次调用；运镜类调用按 Commander 真实的 progress_percent 模式回放
           （搬到起点 → 停顿在起点 → 运镜段内百分比按**时间**归一化、与路径进度无关）并把"当前位姿"推进到终点。"""

    #: 运镜段内各帧的 (路径进度 s, progress_percent)：百分比故意与 s 不成比例（Commander 是 elapsed/6s，按时间归一化）。
    #: 末帧 s 停在 0.96 而不是 1.0——真实 Commander 的反馈频率（10Hz）与 goal 结束之间有间隙，
    #: 最后一帧反馈时臂还没走完（2026-09-17 mock 臂实测 0.9566）。
    MOTION_FRAMES = {
        'shot_linear': ((0.25, 58.0), (0.5, 66.0), (0.75, 75.0), (0.96, 83.0)),
        'shot_orbit': ((0.25, 70.0), (0.5, 80.0), (0.75, 90.0), (0.96, 99.9)),
    }
    #: 运镜前的帧：(pct, 位姿=起点)。LINEAR：PTP 25 → 停顿 50；ORBIT：PTP 10 → 停顿 20 → 规划期 60
    PRE_FRAMES = {'shot_linear': (25.0, 50.0), 'shot_orbit': (10.0, 20.0, 60.0)}

    def __init__(self):
        self.calls = []
        self.pose = None                 # 当前法兰位姿（dict）
        self.joints = {}
        self.fail_on = None              # ('shot_linear', CallResult) → 该类调用直接失败
        self.on_progress = None          # 测试钩子：每次回放反馈后调用 (name, s)
        self.cancelled = 0
        self.last_feedback_cb = None

    def get_joints(self):
        return dict(self.joints)

    def get_pose(self):
        return _Pose(self.pose) if self.pose is not None else None

    def _emit(self, feedback_cb, pct, pose):
        self.pose = pose
        if feedback_cb is not None:
            feedback_cb(_Feedback(pct, _Pose(pose)))

    def _approach(self, name, start, feedback_cb):
        """@brief 回放"搬到起点"：从臂当前所在位姿插值到本原语起点（真实 Commander 就是先 PTP 过去）。"""
        frm = self.pose if self.pose is not None else start
        pres = self.PRE_FRAMES[name]
        for k, pct in enumerate(pres):
            u = (k + 1) / len(pres)
            self._emit(feedback_cb, pct, {key: frm[key] + u * (start[key] - frm[key]) for key in start})

    def _play(self, name, start, end, feedback_cb, on_camera_ready):
        self.calls.append(name)
        self.last_feedback_cb = feedback_cb
        if self.fail_on and self.fail_on[0] == name:
            return self.fail_on[1]
        if name == 'move_to_pose':
            self._emit(feedback_cb, 50.0, end)
            self._emit(feedback_cb, 100.0, end)
            return CallResult(True, 'reached')
        self._approach(name, start, feedback_cb)
        if on_camera_ready is not None:
            on_camera_ready()
        for s, pct in self.MOTION_FRAMES[name]:
            pose = {k: start[k] + s * (end[k] - start[k]) for k in start}
            self._emit(feedback_cb, pct, pose)
            if self.on_progress is not None:
                self.on_progress(name, s)
            if self.cancelled:
                return CallResult(False, 'cancelled')
        self.pose = dict(end)          # goal 返回 reached 时臂已到终点（最后一段没有反馈覆盖）
        return CallResult(True, 'reached')

    def move_to_pose(self, pose, speed='normal', return_to_start=False, timeout_sec=None, feedback_cb=None):
        return self._play('move_to_pose', None, dict(pose), feedback_cb, None)

    def shot_linear(self, start, end, speed='normal', return_to_start=False, timeout_sec=None,
                    on_camera_ready=None, feedback_cb=None):
        return self._play('shot_linear', dict(start), dict(end), feedback_cb, on_camera_ready)

    def shot_orbit(self, center, az_start_deg, az_end_deg, el_start_deg, el_end_deg, r_start_m, r_end_m,
                   speed='normal', return_to_start=False, timeout_sec=None, on_camera_ready=None,
                   feedback_cb=None):
        def _pose(az, el, r):
            return rc.pose_from_look_at(rc.sphere_to_cart(math.radians(az), math.radians(el), r, center), center)
        # 球面上按球坐标插值回放（与 Commander plan_orbit 一致），位姿字典逐帧现算
        self.calls.append('shot_orbit')
        self.last_feedback_cb = feedback_cb
        if self.fail_on and self.fail_on[0] == 'shot_orbit':
            return self.fail_on[1]
        start = _pose(az_start_deg, el_start_deg, r_start_m)
        self._approach('shot_orbit', start, feedback_cb)
        if on_camera_ready is not None:
            on_camera_ready()
        for s, pct in self.MOTION_FRAMES['shot_orbit']:
            pose = _pose(az_start_deg + s * (az_end_deg - az_start_deg), el_start_deg + s * (el_end_deg - el_start_deg),
                         r_start_m + s * (r_end_m - r_start_m))
            self._emit(feedback_cb, pct, pose)
            if self.on_progress is not None:
                self.on_progress('shot_orbit', s)
            if self.cancelled:
                return CallResult(False, 'cancelled')
        self.pose = _pose(az_end_deg, el_end_deg, r_end_m)
        return CallResult(True, 'reached')

    def cancel(self):
        self.cancelled += 1
        return True

    def call_after(self, name, hook):
        """@brief 测试钩子：某类调用**返回之后**调 hook（模拟"上一条已回、下一条未发"的窗口）。"""
        orig = getattr(self, name)

        def _wrapped(*a, **kw):
            result = orig(*a, **kw)
            hook()
            return result
        setattr(self, name, _wrapped)


class _Logger:
    """@brief 假 logger：记录 warning / error（执行器把回调里的异常吞成 warning，测试要能看见）。"""

    def __init__(self):
        self.warnings = []

    def info(self, *_):
        pass

    def warning(self, msg, *_):
        self.warnings.append(str(msg))

    def error(self, msg, *_):
        self.warnings.append(str(msg))


class FakeApi:
    """@brief 假 ArmApi：arm + node.get_logger()。"""

    def __init__(self):
        self.arm = FakeArm()
        self.logger = _Logger()
        logger = self.logger
        self.node = type('Node', (), {'get_logger': lambda self_: logger})()


class FakeClock:
    """@brief 假单调时钟：sleep 推进时间。"""

    def __init__(self):
        self.t = 1000.0
        self.sleeps = []

    def now(self):
        return self.t

    def sleep(self, sec):
        self.sleeps.append(sec)
        self.t += sec


@pytest.fixture(scope='session')
def model(urdf_xml):
    """@brief ArmModel。"""
    return rc.ArmModel.from_urdf_string(urdf_xml)


@pytest.fixture
def rig(model):
    """@brief (executor, fake_api, clock, feedbacks)：假臂 + 真编译器 + 收集所有反馈字典。"""
    api = FakeApi()
    clock = FakeClock()
    feedbacks = []
    published = []
    executor = se.ShotExecutor(api, sc.ShotCompiler(model), on_feedback=feedbacks.append,
                               feedback_publisher=published.append, pose_factory=dict,
                               sleep=clock.sleep, clock=clock.now, stamp_clock=clock.now)
    executor.published = published
    return executor, api, clock, feedbacks


def _static(waypoints, segments, tolerance=None, **extra):
    """@brief A 型 JSON。"""
    out = {'type': 'static', 'waypoints': waypoints, 'segments': segments,
           'tolerance': tolerance or {'position': 0.03, 'aim': 0.03, 'roll': 0.02}}
    out.update(extra)
    return out


def _wp(position, look_at, **kw):
    """@brief A 型 waypoint。"""
    out = {'position': list(position), 'look_at': list(look_at)}
    out.update(kw)
    return out


def _dolly(hold_end=1.0, hold_start=0.0, law='s_curve'):
    """@brief 推 10cm 的单段镜头。"""
    return _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50), hold=hold_start),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50), hold=hold_end)],
                   [{'duration': 2.0, 'law': law}])


def _tracking(az0, az1, target_static=False, position_ref='target', hold=1.0, segments=True):
    """@brief 绕近侧的 B 型镜头（视距 0.12，平视）。"""
    wps = [{'d': 0.12, 'az': az0, 'el': 0.0}]
    if segments:
        wps.append({'d': 0.12, 'az': az1, 'el': 0.0, 'hold': hold})
    return {'type': 'tracking', 'target': '/subject_pose', 'target_static': target_static,
            'position_ref': position_ref, 'waypoints': wps,
            'segments': [{'duration': 6.0}] if segments else [],
            'tolerance': {'d': 0.05, 'az': 0.05, 'el': 0.05, 'u': 0.02, 'v': 0.02, 'roll': 0.02}}


def _target(clock, ident='trk_1', pos=(0.40, 0.0, 0.55), age=0.0, yaw=None):
    """@brief 一帧目标数据。"""
    d = {'stamp': clock.now() - age, 'id': ident, 'position': list(pos), 'confidence': 0.9}
    if yaw is not None:
        d['yaw'] = yaw
    return d


# ═══════════════════════════════ A 型流程与反馈 ═══════════════════════════════

def test_static_shot_runs_segment_then_hold_then_done(rig):
    """@brief 单段 + 末点 hold：只发一条 shot_linear；反馈 phase 依次 segment → hold → done，进度到 1.0。"""
    executor, api, clock, fbs = rig
    report = executor.run(_dolly(hold_end=1.0))
    assert report.ok and report.exit_reason == 'done'
    assert api.arm.calls == ['shot_linear']
    phases = [f['phase'] for f in fbs]
    assert phases[0] == 'segment' and phases[-1] == 'done'
    assert 'hold' in phases
    first_hold = next(f for f in fbs if f['phase'] == 'hold')
    assert first_hold['segment_index'] == 0 and first_hold['progress'] == 1.0
    assert fbs[-1]['segment_index'] == 0 and fbs[-1]['progress'] == 1.0
    assert fbs[-1]['in_tolerance'] == [True, True, True]
    assert all(f['in_tolerance'] == [True, True, True] for f in fbs if f['phase'] == 'hold')
    assert fbs[-1]['deviation_cause'] == 'none' and fbs[-1]['degradation'] == 'none'
    assert 'time_scale' not in fbs[-1] and 'target_status' not in fbs[-1]
    assert not [w for w in api.logger.warnings if '异常' in w or '失败' in w], api.logger.warnings


def test_progress_comes_from_pose_not_commander_percent(rig):
    """@brief 运镜段内 progress 由 current_pose 在原语几何上投影得到：假臂回放的 progress_percent 与路径进度
           故意不成比例（Commander 按时间归一化），反馈里的 progress 仍是 0 → 0.25 → 0.5 → 0.75 → 1.0。"""
    executor, api, clock, fbs = rig
    executor.run(_dolly(hold_end=0.0))
    seg = [f['progress'] for f in fbs if f['phase'] == 'segment']
    assert seg[0] == pytest.approx(0.0) and seg[-1] == pytest.approx(1.0, abs=1e-6)
    assert seg == sorted(seg)
    assert 0.5 in [round(v, 6) for v in seg]


def test_error_and_in_tolerance_computed_from_current_pose(rig):
    """@brief 运镜段内 error 三项来自反馈里的 current_pose 与参考位姿之差；假臂沿法兰直线回放，位置误差应很小、都在容差内。"""
    executor, api, clock, fbs = rig
    executor.run(_dolly(hold_end=0.0))
    moving = [f for f in fbs if f['error']['position'] is not None]
    assert moving
    for f in moving:
        assert set(f['error']) == {'position', 'aim', 'roll'}
        assert f['error']['position'] < 0.03 and f['in_tolerance'] == [True, True, True]


def test_approach_to_start_reports_no_error(rig):
    """@brief 规范 2.3：把臂搬到分镜起点属于走位、不归本格式，容差只约束运镜段。
           所以 Commander 还在搬到起点（progress_percent 未越过运镜段起点）时，反馈的 error / in_tolerance
           必须留空，不能拿"离起点还差多远"去判超差——否则一镜完美的运镜会被上层判成「完成但有降级」。"""
    executor, api, clock, fbs = rig
    api.arm.pose = {'x': 0.0, 'y': 0.0, 'z': 0.10, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}   # 臂在别处
    executor.run(_dolly(hold_end=0.0))
    approach = [f for f in fbs if f['phase'] == 'segment' and f['progress'] == 0.0]
    assert approach, '应该有搬到起点阶段的反馈'
    for f in approach:
        assert f['error'] == {'position': None, 'aim': None, 'roll': None}
        assert f['in_tolerance'] is None
    assert any(f['error']['position'] is not None for f in fbs), '运镜段仍要报误差'


def test_segment_end_emits_progress_one_before_hold(rig):
    """@brief 段走完那一刻要发一帧 phase=segment / progress=1.0，再进 hold；否则上层看到的最后一帧 segment
           停在 0.9x（反馈频率与 goal 结束之间的间隙），无法判断这一段是否真的走完。"""
    executor, api, clock, fbs = rig
    executor.run(_dolly(hold_end=1.0))
    idx_end = [i for i, f in enumerate(fbs) if f['phase'] == 'segment' and f['progress'] == 1.0]
    idx_hold = [i for i, f in enumerate(fbs) if f['phase'] == 'hold']
    assert idx_end and idx_hold and idx_end[0] < idx_hold[0]
    assert fbs[idx_end[0]]['segment_index'] == 0
    assert fbs[idx_end[0]]['in_tolerance'] == [True, True, True]


def test_hold_at_first_waypoint_uses_ptp_and_reports_segment_zero(rig):
    """@brief 首点有 hold 2 s：先 move_to_pose 到首点再停留；停留期 segment_index=0、progress=1.0（9.1）；
           后面还有 goal，本地只等 2 − 1（Commander 起点停顿）= 1 s。"""
    executor, api, clock, fbs = rig
    t0 = clock.now()
    report = executor.run(_dolly(hold_end=0.0, hold_start=2.0))
    assert report.ok
    assert api.arm.calls == ['move_to_pose', 'shot_linear']
    hold = [f for f in fbs if f['phase'] == 'hold']
    assert hold and hold[0]['segment_index'] == 0 and hold[0]['progress'] == 1.0
    assert clock.now() - t0 == pytest.approx(1.0, abs=0.15)
    assert hold[-1]['hold_elapsed'] >= 0.85
    # 首点停留期臂就在首点：误差相对首点算，应全在容差内（9.1 的快门判据靠这个）；
    # 之后 Commander 搬去下一段起点那几帧仍报 hold，但已离开首点，误差留空不判超差
    settled = [f for f in hold if f['error']['position'] is not None]
    assert settled
    assert all(f['in_tolerance'] == [True, True, True] and f['error']['position'] < 0.005 for f in settled)
    assert all(f['in_tolerance'] is None for f in hold if f['error']['position'] is None)
    # 下一条 goal 的搬到起点 / 停顿帧也算 hold（segment_index 仍是 0），之后才进 segment
    phases = [f['phase'] for f in fbs]
    assert phases.index('segment') < phases.index('hold') < len(phases) - 1
    assert phases[phases.index('hold'):].count('segment') >= 4


def test_intermediate_hold_reports_previous_segment_and_stays_in_tolerance(rig):
    """@brief 中间点 hold：反馈 segment_index = 刚走完的段（0），下一段 goal 的起点停顿帧继续算 hold，
           误差相对该中间点算，全在容差内。"""
    executor, api, clock, fbs = rig
    shot = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50), hold=3.0),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50), hold=0.0)],
                   [{'duration': 1.0}, {'duration': 1.0}])
    executor.run(shot)
    hold = [f for f in fbs if f['phase'] == 'hold']
    assert hold and all(f['segment_index'] == 0 and f['progress'] == 1.0 for f in hold)
    settled = [f for f in hold if f['error']['position'] is not None]
    assert settled and all(f['in_tolerance'] == [True, True, True] for f in settled)
    seg1 = [f for f in fbs if f['phase'] == 'segment' and f['segment_index'] == 1]
    assert seg1 and seg1[-1]['progress'] == pytest.approx(1.0, abs=1e-6)


def test_intermediate_hold_is_shortened_by_commander_dwell(rig):
    """@brief 中间点 hold 3 s：下一条 goal 自带 1 s 起点停顿，所以本地只等 2 s；末点 hold 全等。"""
    executor, api, clock, fbs = rig
    shot = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50), hold=3.0),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50), hold=1.5)],
                   [{'duration': 1.0}, {'duration': 1.0}])
    t0 = clock.now()
    executor.run(shot)
    assert api.arm.calls == ['shot_linear', 'shot_linear']
    assert clock.now() - t0 == pytest.approx(3.0 - 1.0 + 1.5, abs=0.25)


def test_single_waypoint_without_hold_is_done_immediately(rig):
    """@brief 1 点 0 段无 hold（3.2）：到位后立刻 done，无 hold 阶段。"""
    executor, api, clock, fbs = rig
    report = executor.run(_static([_wp((0.40, 0.0, 0.55), (0.9, 0.0, 0.45))], []))
    assert report.ok and api.arm.calls == ['move_to_pose']
    assert [f['phase'] for f in fbs][-1] == 'done'
    assert 'hold' not in [f['phase'] for f in fbs]
    assert fbs[-1]['segment_index'] == 0 and fbs[-1]['progress'] == 1.0


def test_invalid_json_is_rejected_without_moving(rig):
    """@brief 校验失败：不动臂，report.exit_reason='rejected'，errors 带规则号。"""
    executor, api, clock, fbs = rig
    bad = _dolly()
    bad['segments'] = []
    report = executor.run(bad)
    assert not report.ok and report.exit_reason == 'rejected'
    assert api.arm.calls == []
    assert any(e['rule'] == 2 for e in report.errors)


def test_unreachable_shot_is_refused_before_running(rig):
    """@brief 8.0：开跑前验可达性，不可达就拒绝执行、上报，不下发任何 goal。"""
    executor, api, clock, fbs = rig
    if not os.path.exists(_LLM_EXAMPLE):
        pytest.skip('大模型输出示例不在本机')
    with open(_LLM_EXAMPLE, encoding='utf-8') as fh:
        shot = json.load(fh)['agent_outputs']['cinematographer']['shot_plans'][0]
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'unreachable'
    assert api.arm.calls == []
    assert report.errors[0]['rule'] == 25


def test_goal_failure_stops_and_reports_limit(rig):
    """@brief Commander 拒绝 / 失败（unreachable）：中止后续，exit_reason='failed'，deviation_cause='limit'。"""
    executor, api, clock, fbs = rig
    api.arm.fail_on = ('shot_linear', CallResult(False, 'unreachable'))
    shot = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50)),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50), hold=1.0)],
                   [{'duration': 1.0}, {'duration': 1.0}])
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'failed'
    assert api.arm.calls == ['shot_linear']
    assert report.feedback['deviation_cause'] == 'limit'
    assert report.feedback['phase'] != 'done'


def test_slowed_degradation_propagates_to_feedback(rig):
    """@brief 编译期标的 degradation=slowed 出现在每一条反馈里。"""
    executor, api, clock, fbs = rig
    executor.run(_dolly(law='constant'))
    assert all(f['degradation'] == 'slowed' for f in fbs)


def test_feedback_is_published_as_json_lines(rig):
    """@brief feedback_publisher 收到的是可解析的 JSON，字段齐全。"""
    executor, api, clock, fbs = rig
    executor.run(_dolly())
    assert executor.published
    obj = json.loads(executor.published[-1])
    assert {'phase', 'hold_elapsed', 'segment_index', 'progress', 'error', 'in_tolerance',
            'deviation_cause', 'degradation'} <= set(obj)


def test_camera_ready_callback_gets_av_once(rig):
    """@brief 第一条运镜 goal 的 camera_ready 上升沿调一次 on_camera_ready(av)。"""
    executor, api, clock, fbs = rig
    got = []
    executor.on_camera_ready = got.append
    av = {'recording': {'mode': 'VIDEO', 'fps': 30, 'duration_ms': 5000}}
    shot = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50)),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50))],
                   [{'duration': 1.0}, {'duration': 1.0}], av=av)
    report = executor.run(shot)
    assert report.ok and got == [av]
    assert report.av == av


def test_camera_ready_fires_before_first_waypoint_hold(rig):
    """@brief 首点带 hold：到达首点即触发 on_camera_ready（首点停留要进素材），早于第一条运镜 goal。"""
    executor, api, clock, fbs = rig
    executor.on_camera_ready = lambda av: api.arm.calls.append('camera_ready')
    report = executor.run(_dolly(hold_end=0.0, hold_start=2.0))
    assert report.ok
    assert api.arm.calls == ['move_to_pose', 'camera_ready', 'shot_linear']


def test_late_feedback_after_goal_result_is_ignored(rig):
    """@brief goal 结果已回、执行器已收尾后到达的迟到反馈不得改动状态（phase 仍是 done）。"""
    executor, api, clock, fbs = rig
    report = executor.run(_dolly(hold_end=0.0))
    assert report.feedback['phase'] == 'done'
    n = len(fbs)
    api.arm.last_feedback_cb(_Feedback(70.0, _Pose(api.arm.pose)))
    assert len(fbs) == n and json.loads(executor.published[-1])['phase'] == 'done'


def test_cancel_stops_after_current_primitive(rig):
    """@brief 执行中 cancel()：Commander 把当前 goal 以 cancelled 收尾（CallResult False），执行器要报 cancelled
           而不是 failed，且不再下发后续。"""
    executor, api, clock, fbs = rig
    api.arm.on_progress = lambda name, s: executor.cancel() if s >= 0.5 else None
    shot = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50)),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50))],
                   [{'duration': 1.0}, {'duration': 1.0}])
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'cancelled'
    assert api.arm.calls == ['shot_linear']


def test_cancel_between_primitives_skips_the_next_goal(rig):
    """@brief 取消落在"上一条 goal 已回、下一条未发"的窗口：下一条不再下发，exit_reason='cancelled'。"""
    executor, api, clock, fbs = rig
    api.arm.call_after('shot_linear', executor.cancel)
    shot = _static([_wp((0.30, 0.0, 0.60), (0.80, 0.0, 0.50)),
                    _wp((0.35, 0.0, 0.60), (0.85, 0.0, 0.50)),
                    _wp((0.40, 0.0, 0.60), (0.90, 0.0, 0.50))],
                   [{'duration': 1.0}, {'duration': 1.0}])
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'cancelled'
    assert api.arm.calls == ['shot_linear']


def test_report_user_line_is_plain_language(rig):
    """@brief report.user_line() 给最终用户看：不含 JSON 字段名。"""
    executor, api, clock, fbs = rig
    report = executor.run(_dolly())
    line = report.user_line()
    assert line and 'segment_index' not in line and 'progress' not in line


# ═══════════════════════════════ B 型：目标快照 ═══════════════════════════════

def test_tracking_shot_snapshots_target_and_reports_target_status(rig):
    """@brief B 型：开始时读一帧目标、展开成 A 型执行；反馈多出 time_scale=1.0 与 target_status='ok'；
           报告里说明是按快照开环执行。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock)
    report = executor.run(_tracking(math.pi - 0.8, math.pi + 0.8))
    assert report.ok, report.errors
    assert api.arm.calls == ['shot_orbit']          # 平视绕目标、uv=0 → 精确一条 ORBIT
    seg = [f for f in fbs if f['phase'] == 'segment']
    assert seg[-1]['progress'] == pytest.approx(1.0, abs=1e-6) and seg[-1]['in_tolerance'] == [True] * 6
    assert fbs[-1]['target_status'] == 'ok' and fbs[-1]['time_scale'] == 1.0
    assert any('快照' in w for w in report.warnings)


def test_tracking_feedback_error_has_six_components_for_spherical_ref(rig):
    """@brief 第 9 章：B 型 position_ref=target 的 error / in_tolerance 是 d / az / el / u / v / roll 六项，
           假臂精确跟随时全在容差内。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock)
    report = executor.run(_tracking(math.pi - 0.8, math.pi + 0.8))
    assert report.ok
    last = fbs[-1]
    assert not [w for w in api.logger.warnings if '异常' in w or '失败' in w], api.logger.warnings
    assert set(last['error']) == {'d', 'az', 'el', 'u', 'v', 'roll'}
    assert all(v is not None for v in last['error'].values())
    assert last['in_tolerance'] == [True] * 6
    seg = [f for f in fbs if f['phase'] == 'segment' and f['error']['d'] is not None]
    assert seg and all(f['in_tolerance'] == [True] * 6 for f in seg)


def test_tracking_feedback_error_has_four_components_for_world_ref(rig):
    """@brief B 型 position_ref=world：error / in_tolerance 是 position / u / v / roll 四项。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock)
    shot = {'type': 'tracking', 'target': '/subject_pose', 'position_ref': 'world',
            'waypoints': [{'position': [0.28, -0.05, 0.55], 'uv': [0.0, -0.05]},
                          {'position': [0.28, 0.05, 0.55], 'uv': [0.0, -0.05], 'hold': 1.0}],
            'segments': [{'duration': 4.0}],
            'tolerance': {'position': 0.03, 'u': 0.03, 'v': 0.03, 'roll': 0.02}}
    report = executor.run(shot)
    assert report.ok, report.errors
    assert set(fbs[-1]['error']) == {'position', 'u', 'v', 'roll'}
    assert fbs[-1]['in_tolerance'] == [True] * 4


def test_tracking_target_jitter_below_noise_floor_does_not_stop(rig):
    """@brief target_static 的段间复查要高于感知噪声：1.5 cm 横向抖动（低于 2 cm 地板）不触发 moved，
           哪怕映射后的位置容差只有 6 mm。"""
    executor, api, clock, fbs = rig
    positions = iter([(0.40, 0.0, 0.55), (0.40, 0.015, 0.55), (0.40, 0.015, 0.55)])
    executor.target_provider = lambda: _target(clock, pos=next(positions))
    shot = _tracking(math.pi - 0.8, math.pi, target_static=True, hold=0.0)
    shot['waypoints'].append({'d': 0.12, 'az': math.pi + 0.8, 'el': 0.0, 'hold': 1.0})
    shot['segments'].append({'duration': 6.0})
    report = executor.run(shot)
    assert report.ok and len(api.arm.calls) == 2


def test_tracking_camera_on_target_is_rejected_not_raised(rig):
    """@brief position_ref=world 且机位与目标重合：展开失败要回 rejected 报告，不能抛异常。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock, pos=(0.30, 0.0, 0.60))
    shot = {'type': 'tracking', 'target': '/subject_pose', 'position_ref': 'world',
            'waypoints': [{'position': [0.30, 0.0, 0.60]}], 'segments': [],
            'tolerance': {'position': 0.03, 'u': 0.03, 'v': 0.03, 'roll': 0.02}}
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'rejected' and api.arm.calls == []


def test_tracking_without_provider_is_rejected(rig):
    """@brief B 型但没有目标数据源 → rejected。"""
    executor, api, clock, fbs = rig
    report = executor.run(_tracking(math.pi - 0.8, math.pi + 0.8))
    assert not report.ok and report.exit_reason == 'rejected' and api.arm.calls == []


def test_tracking_stale_target_is_refused(rig):
    """@brief 目标数据过期（stamp 太旧）→ 不开跑，exit_reason='target_stale'。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock, age=5.0)
    report = executor.run(_tracking(math.pi - 0.8, math.pi + 0.8))
    assert not report.ok and report.exit_reason == 'target_stale' and api.arm.calls == []


def test_tracking_target_front_without_yaw_is_rejected(rig):
    """@brief 规则 24：position_ref=target_front 而数据源没有 yaw → rejected，errors 里有 24。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock)
    report = executor.run(_tracking(-0.8, 0.8, position_ref='target_front'))
    assert report.exit_reason == 'rejected' and any(e['rule'] == 24 for e in report.errors)


def test_tracking_id_change_between_segments_stops(rig):
    """@brief 6.2.4：段与段之间目标 id 变了 → 停止、target_status='id_changed'，不再下发后一段。"""
    executor, api, clock, fbs = rig
    ids = iter(['trk_1', 'trk_2', 'trk_2'])
    executor.target_provider = lambda: _target(clock, ident=next(ids))
    shot = _tracking(math.pi - 0.8, math.pi, hold=0.0)
    shot['waypoints'].append({'d': 0.12, 'az': math.pi + 0.8, 'el': 0.0, 'hold': 1.0})
    shot['segments'].append({'duration': 6.0})
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'target_id_changed'
    assert len(api.arm.calls) == 1
    assert report.feedback['target_status'] == 'id_changed'


def test_tracking_static_target_moved_stops(rig):
    """@brief 6.2.2：target_static=true 但目标位置偏离锁定值超过位置容差 → 停止并报 moved。"""
    executor, api, clock, fbs = rig
    positions = iter([(0.40, 0.0, 0.55), (0.40, 0.03, 0.55), (0.40, 0.03, 0.55)])
    executor.target_provider = lambda: _target(clock, pos=next(positions))
    shot = _tracking(math.pi - 0.8, math.pi, target_static=True, hold=0.0)
    shot['waypoints'].append({'d': 0.12, 'az': math.pi + 0.8, 'el': 0.0, 'hold': 1.0})
    shot['segments'].append({'duration': 6.0})
    report = executor.run(shot)
    assert not report.ok and report.exit_reason == 'target_moved'
    assert report.feedback['target_status'] == 'moved'
    assert len(api.arm.calls) == 1


def test_tracking_single_waypoint_follow_is_done_after_ptp(rig):
    """@brief 例 6.6.4 形态（1 点 0 段）：按快照 PTP 到相对机位后 done。"""
    executor, api, clock, fbs = rig
    executor.target_provider = lambda: _target(clock)
    report = executor.run(_tracking(math.pi, None, segments=False))
    assert report.ok and api.arm.calls == ['move_to_pose']
    assert fbs[-1]['phase'] == 'done' and fbs[-1]['target_status'] == 'ok'
