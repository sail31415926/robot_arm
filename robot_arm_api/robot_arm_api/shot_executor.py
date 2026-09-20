# -*- coding: utf-8 -*-
"""
@file      shot_executor.py
@brief     规范 JSON 的执行器：解析（shot_spec）→ B 型按目标快照展开 → 编译成 Commander 原语（shot_compiler）
           → 经 ArmApi 逐条下发 → 按规范第 9 章回报 phase / progress / error / in_tolerance / degradation …。
           核心流程只依赖鸭子类型的 api（api.arm.move_to_pose / shot_linear / shot_orbit / get_pose / get_joints /
           cancel），可用假臂单测；ROS 胶水（反馈话题、目标话题订阅、tf 查世界系）放在文件末尾，运行时才 import。
@version   0.1
@date      2026-09-16
@copyright Copyright (c) 2026 eMeet

执行链的能力边界（诚实地写在报告 / 反馈里）：
  · 机械臂 6 轴，底盘不动：世界系 → 臂基座系的变换由 WorldFrame 给定（tf 查 odom → arm_base_link 或手填底盘位姿）。
  · B 型没有闭环：开始时刻读一帧目标，展开成 A 型开环执行；段与段之间复查目标 id（变了 → id_changed 停）、
    target_static 时复查位置（动了 → moved 停）、数据是否过期（stale 只告警）。time_scale 恒 1.0。
  · deviation_cause 只能给 none / limit（Commander 报 unreachable / limit 即 limit）；没有避障，不会给 obstacle。
  · progress 由 current_pose 在原语几何上投影得到（Commander 的 progress_percent 在运镜段内是按时间归一化的，
    只用来判断"还没开始动"）。
  · Commander 每条 goal 自带"先 PTP 到起点"，那一段属于走位（规范 2.3），期间 error / in_tolerance 留空不判超差；
    段走完补发一帧 progress=1.0。
  · av 只透传：第一条运镜 goal 的 camera_ready 上升沿调 on_camera_ready(av)，由相机 / 录音模块消费。

═══════════════════════════════ 本文件函数 / 类汇总 ═══════════════════════════════
  ShotFeedback                        第 9 章反馈（to_dict 去掉 A 型没有的字段）
  ShotReport                          一次 run 的结果：ok / exit_reason / errors / warnings / degradation / 最后一帧反馈
  ShotExecutor.run(shot_json)         主入口（阻塞到 done / 失败 / 取消）
  ShotExecutor.cancel()               取消：中止当前 goal 并不再下发
  make_shot_feedback_publisher(node)  反馈 → std_msgs/String（JSON）话题
  make_target_provider(node, topic)   订阅目标话题（std_msgs/String JSON 或 geometry_msgs/PointStamped）→ 最新一帧
  lookup_world_frame(node, ...)       tf2 查 odom → arm_base_link → WorldFrame
"""

import json
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from . import shot_spec as ss
from .shot_compiler import CompiledShot, Primitive, ShotCompiler, WorldFrame, primitive_progress

SHOT_FEEDBACK_TOPIC = '/robot_arm/shot_feedback'
JOINT_ORDER = ('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6')
_LIMIT_WORDS = ('unreachable', 'limit', '限位', '不可达', '超出')
#: B 型段间复查"目标动了"的位移地板（m）：低于感知噪声（附录 C.4 横向 ±1.4 cm）的位移不算动
TARGET_MOVED_FLOOR_M = 0.02


class _CancelledResult:
    """@brief _send 在下发前就发现已取消时返回的占位结果（与 CallResult 同样按真值 / reason 使用）。"""
    success = False
    reason = 'cancelled'

    def __bool__(self) -> bool:
        return False


# ─────────────────────────────── 反馈与报告 ───────────────────────────────

@dataclass
class ShotFeedback:
    """@brief 规范第 9 章的运镜反馈。time_scale / target_status 只有 B 型才有（None = 不输出）。"""
    phase: str = 'segment'
    hold_elapsed: float = 0.0
    segment_index: int = 0
    progress: float = 0.0
    error: Dict[str, Optional[float]] = field(default_factory=lambda: {'position': None, 'aim': None, 'roll': None})
    in_tolerance: Optional[List[bool]] = None
    deviation_cause: str = 'none'
    degradation: str = 'none'
    time_scale: Optional[float] = None
    target_status: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """@brief 转字典（可 JSON 化）。
        @return dict
        """
        out: Dict[str, Any] = {
            'phase': self.phase, 'hold_elapsed': round(self.hold_elapsed, 3),
            'segment_index': self.segment_index, 'progress': round(self.progress, 4),
            'error': {k: (None if v is None else round(v, 5)) for k, v in self.error.items()},
            'in_tolerance': self.in_tolerance, 'deviation_cause': self.deviation_cause,
            'degradation': self.degradation,
        }
        if self.time_scale is not None:
            out['time_scale'] = self.time_scale
        if self.target_status is not None:
            out['target_status'] = self.target_status
        return out


@dataclass
class ShotReport:
    """@brief 一次 run 的结果。exit_reason：done / rejected / unreachable / failed / cancelled /
           target_stale / target_id_changed / target_moved。"""
    ok: bool = False
    exit_reason: str = 'rejected'
    errors: List[Dict[str, Any]] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    degradation: str = 'none'
    av: Optional[Dict[str, Any]] = None
    compiled: Optional[CompiledShot] = None
    feedback: Optional[Dict[str, Any]] = None
    plan_steps: List[Dict[str, Any]] = field(default_factory=list)
    detail: str = ''

    def to_dict(self) -> Dict[str, Any]:
        """@brief 转字典（不含 compiled 对象）。
        @return dict
        """
        return {'ok': self.ok, 'exit_reason': self.exit_reason, 'errors': self.errors,
                'warnings': self.warnings, 'degradation': self.degradation, 'av': self.av,
                'feedback': self.feedback, 'plan_steps': self.plan_steps, 'detail': self.detail}

    def user_line(self) -> str:
        """@brief 给最终用户 / 拍摄记录的一句人话。
        @return 文本
        """
        if self.ok:
            base = '这一镜拍完了'
            if self.degradation == 'slowed':
                base += '，但机械臂跟不上要求的节奏，实际比剧本慢'
            if self.feedback and self.feedback.get('in_tolerance') and not all(self.feedback['in_tolerance']):
                base += '，收尾时画面没完全回到容差内'
            return base + '。'
        reasons = {
            'rejected': '这一镜的写法不合规范，没有执行',
            'unreachable': '机械臂够不着这一镜要求的机位，没有执行',
            'failed': '执行中机械臂报错，中途停了',
            'cancelled': '这一镜被取消了',
            'target_stale': '拿不到新鲜的目标位置，没有开拍',
            'target_id_changed': '拍摄对象中途换人了，已停下等新的分镜',
            'target_moved': '说好不动的拍摄对象动了，已停下等重新决策',
        }
        text = reasons.get(self.exit_reason, '这一镜没有完成')
        if self.detail:
            text += f'：{self.detail}'
        return text + '。'


# ─────────────────────────────── 执行器 ───────────────────────────────

class ShotExecutor:
    """@brief 规范 JSON → Commander 的执行器（阻塞式）。"""

    def __init__(self, api: Any, compiler: ShotCompiler, *,
                 on_feedback: Optional[Callable[[Dict[str, Any]], None]] = None,
                 feedback_publisher: Optional[Callable[[str], None]] = None,
                 on_camera_ready: Optional[Callable[[Optional[Dict[str, Any]]], None]] = None,
                 target_provider: Optional[Callable[[], Optional[Dict[str, Any]]]] = None,
                 target_stale_sec: float = 0.5,
                 pose_factory: Optional[Callable[[Dict[str, float]], Any]] = None,
                 sleep: Callable[[float], None] = time.sleep,
                 clock: Callable[[], float] = time.monotonic,
                 stamp_clock: Callable[[], float] = time.time,
                 lens: str = 'WIDE', aspect: str = '4:3', base_fps: Optional[float] = None,
                 hold_tick_sec: float = 0.1, target_moved_floor_m: float = TARGET_MOVED_FLOOR_M):
        """@brief 构造。
        @param api                ArmApi（或同形状的假对象）：api.arm 提供 move_to_pose / shot_linear / shot_orbit /
                                  get_pose / get_joints / cancel，api.node.get_logger()
        @param compiler           ShotCompiler（带 WorldFrame / CameraFrames）
        @param on_feedback        每帧反馈回调 f(dict)
        @param feedback_publisher 每帧反馈的 JSON 文本出口（如 make_shot_feedback_publisher）
        @param on_camera_ready    第一条运镜 goal 到达起拍点时调一次 f(av)，用来开录像
        @param target_provider    B 型目标数据源：无参可调用，返回 6.2.4 口径的 dict 或 None
        @param target_stale_sec   目标数据多久没更新算 stale（与 stamp 同一时钟）
        @param pose_factory       法兰位姿 dict → 传给 api 的对象；None = robot_arm_api.make_pose（ArmPose msg）
        @param sleep,clock        hold 用的睡眠与单调时钟（测试注入）
        @param stamp_clock        与目标 stamp 同基准的时钟（默认 time.time）
        @param lens,aspect        校验规则 21 用的镜头 / 交付画幅
        @param base_fps           校验规则 40 用的基准帧率
        @param hold_tick_sec      hold 期间反馈的节拍
        @param target_moved_floor_m B 型 target_static 段间复查的位移地板（低于感知噪声的位移不算动）
        """
        self.api = api
        self.compiler = compiler
        self.on_feedback = on_feedback
        self.feedback_publisher = feedback_publisher
        self.on_camera_ready = on_camera_ready
        self.target_provider = target_provider
        self.target_stale_sec = target_stale_sec
        self._pose_factory = pose_factory
        self._sleep = sleep
        self._clock = clock
        self._stamp_clock = stamp_clock
        self.lens, self.aspect, self.base_fps = lens, aspect, base_fps
        self.hold_tick_sec = hold_tick_sec
        self.target_moved_floor_m = target_moved_floor_m
        self._cancel = False
        self._fb = ShotFeedback()
        self._static: Optional[ss.ShotSpec] = None
        self._tracking: Optional[ss.ShotSpec] = None     # 原 B 型（算六项 / 四项误差用）
        self._locked: Optional[ss.TargetData] = None
        self._compiled: Optional[CompiledShot] = None
        self._hold_t0: Optional[float] = None
        self._camera_ready_fired = False
        self._gen = 0            # 当前 goal 的代号：迟到的 feedback（goal 结果已回）按代号丢弃
        self._active = False
        self._lock = threading.RLock()   # 反馈回调跑在 rclpy 执行器线程，与主线程共用 _fb

    # ── 小工具 ──
    @property
    def _log(self) -> Any:
        """@brief 日志器。"""
        return self.api.node.get_logger()

    def _make_pose(self, pose: Dict[str, float]) -> Any:
        """@brief 法兰位姿 dict → api 需要的对象。"""
        if self._pose_factory is None:
            from .arm_commander_client import make_pose   # noqa: WPS433  运行时才要 ROS
            self._pose_factory = lambda d: make_pose(d['x'], d['y'], d['z'], d['roll'], d['pitch'], d['yaw'])
        return self._pose_factory(pose)

    def _start_joints(self) -> Optional[List[float]]:
        """@brief 当前 6 轴关节角（IK 种子）；读不全返回 None。"""
        try:
            joints = self.api.arm.get_joints() or {}
        except Exception:  # pylint: disable=broad-except
            return None
        if all(name in joints for name in JOINT_ORDER):
            return [float(joints[name]) for name in JOINT_ORDER]
        return None

    def cancel(self) -> None:
        """@brief 取消：置标志并让当前 goal 中止；run 会在当前原语返回后以 cancelled 收尾。"""
        self._cancel = True
        try:
            self.api.arm.cancel()
        except Exception:  # pylint: disable=broad-except
            pass

    def _publish(self) -> None:
        """@brief 把当前反馈发给回调 / 话题。"""
        data = self._fb.to_dict()
        if self.on_feedback is not None:
            self.on_feedback(data)
        if self.feedback_publisher is not None:
            self.feedback_publisher(json.dumps(data, ensure_ascii=False))

    def _update_error(self, flange_pose: Any, seg_index: int, s: float) -> None:
        """@brief 用一帧法兰位姿更新 error / in_tolerance，参考取第 seg_index 段进度 s 处（(0, 0.0) 即首点）。
               A 型：position / aim / roll；B 型（第 9 章）：d / az / el / u / v / roll 或 position / u / v / roll。"""
        if flange_pose is None or self._static is None:
            return
        try:
            ref = self._reference(seg_index, s)
            rot_a, p_a = self.compiler.frames.optical_from_flange(flange_pose)
            rot_w = self.compiler.world.to_world_rotation(rot_a)
            p_w = self.compiler.world.to_world_point(p_a)
            roll_err = abs(float(ss.log_so3(ref.rotation.T @ rot_w)[2]))
            if self._tracking is None:
                pos = float(np.linalg.norm(p_w - ref.position))
                aim = ss._angle_between(rot_w[:, 2], ref.rotation[:, 2])
                tol = self._static.tolerance
                self._fb.error = {'position': pos, 'aim': aim, 'roll': roll_err}
                self._fb.in_tolerance = [pos <= tol.position, aim <= tol.aim, roll_err <= tol.roll]
                return
            self._fb.error, self._fb.in_tolerance = self._tracking_error(rot_w, p_w, roll_err, seg_index, s)
        except Exception as exc:  # pylint: disable=broad-except
            self._log.warning(f'误差计算失败：{exc!r}')

    def _tracking_error(self, rot_w: np.ndarray, p_w: np.ndarray, roll_err: float,
                        seg_index: int, s: float):
        """@brief B 型误差：相对锁定目标算实际 (d, az, el) / (u, v)，与该进度的参考值比较。"""
        spec, static, t = self._tracking, self._static, self._locked.position
        az0 = float(self._locked.yaw) if spec.position_ref == 'target_front' else 0.0
        if static.segments:
            track = static.segments[seg_index].track
            uv_ref = (track.uv0[0] + s * (track.uv1[0] - track.uv0[0]), track.uv0[1] + s * (track.uv1[1] - track.uv0[1]))
            sph_ref = ss.spherical_param(static, seg_index, s) if track.sph is not None else None
        else:
            wp = spec.waypoints[0]
            uv_ref = wp.uv
            sph_ref = (wp.d, az0 + wp.az, wp.el) if spec.position_ref != 'world' else None
        u, v = ss.uv_of(rot_w, p_w, t)
        u_err = abs(u - uv_ref[0]) if math.isfinite(u) else math.inf
        v_err = abs(v - uv_ref[1]) if math.isfinite(v) else math.inf
        tol = spec.tolerance
        if sph_ref is not None:
            rel = p_w - t
            d = float(np.linalg.norm(rel))
            az = math.atan2(rel[1], rel[0])
            el = math.asin(max(-1.0, min(1.0, rel[2] / max(d, 1e-9))))
            d_err = abs(d - float(sph_ref[0]))
            az_err = abs((az - float(sph_ref[1]) + math.pi) % (2 * math.pi) - math.pi)
            el_err = abs(el - float(sph_ref[2]))
            error = {'d': d_err, 'az': az_err, 'el': el_err, 'u': u_err, 'v': v_err, 'roll': roll_err}
            ok = [d_err <= tol.d, az_err <= tol.az, el_err <= tol.el, u_err <= tol.u, v_err <= tol.v,
                  roll_err <= tol.roll]
        else:
            pos_err = float(np.linalg.norm(p_w - self._reference(seg_index, s).position))
            error = {'position': pos_err, 'u': u_err, 'v': v_err, 'roll': roll_err}
            ok = [pos_err <= tol.position, u_err <= tol.u, v_err <= tol.v, roll_err <= tol.roll]
        return error, ok

    def _error_keys(self) -> tuple:
        """@brief 本分镜反馈里 error 的字段集，与 tolerance 一致（第 9 章）：A 型 3 项，B 型 6 项或 4 项。
        @return 字段名元组
        """
        if self._tracking is None:
            return ('position', 'aim', 'roll')
        if self._tracking.position_ref == 'world':
            return ('position', 'u', 'v', 'roll')
        return ('d', 'az', 'el', 'u', 'v', 'roll')

    def _clear_error(self) -> None:
        """@brief 把 error / in_tolerance 清空（尚未进入受控的运镜段，超差无从谈起），字段集仍按型给全。"""
        self._fb.error = dict.fromkeys(self._error_keys(), None)
        self._fb.in_tolerance = None

    def _reference(self, seg_index: int, s: float) -> ss.CamRef:
        """@brief 参考位姿：有段就按段内进度，0 段的分镜就是唯一的 waypoint。"""
        spec = self._static
        if spec.segments:
            return ss.reference_pose(spec, seg_index, s)
        wp = spec.waypoints[0]
        return ss.CamRef(wp.position, ss.camera_rotation(wp.position, wp.look_at, wp.roll), wp.look_at, wp.roll)

    # ── 目标数据（B 型） ──
    def _read_target(self, spec: ss.ShotSpec, report: ShotReport) -> Optional[ss.TargetData]:
        """@brief 读一帧目标数据并校验；失败时把原因写进 report 并返回 None。"""
        if self.target_provider is None:
            report.exit_reason = 'rejected'
            report.detail = 'B 型需要目标数据源（target_provider），当前没有'
            report.errors.append(ss.SpecError(15, 'target', report.detail).to_dict())
            return None
        data = self.target_provider()
        if data is None:
            report.exit_reason = 'target_stale'
            report.detail = f'目标话题 {spec.target} 还没有数据'
            return None
        errs = ss.validate_target_data(data, spec.position_ref)
        if errs:
            report.exit_reason = 'rejected'
            report.errors += [e.to_dict() for e in errs]
            report.detail = '；'.join(e.message for e in errs)
            return None
        target = ss.TargetData.from_dict(data)
        age = self._stamp_clock() - target.stamp
        if age > self.target_stale_sec:
            report.exit_reason = 'target_stale'
            report.detail = f'目标数据已 {age:.2f} s 未更新（阈值 {self.target_stale_sec} s）'
            return None
        return target

    def _boundary_check(self, spec: ss.ShotSpec, locked: ss.TargetData, report: ShotReport) -> bool:
        """@brief 段间复查目标：id 变了 → id_changed 停；target_static 且动了 → moved 停；过期只告警。
        @return True 可以继续
        """
        data = self.target_provider() if self.target_provider is not None else None
        if data is None or ss.validate_target_data(data, spec.position_ref):
            self._fb.target_status = 'stale'
            report.warnings.append('段间复查：目标数据缺失或不合口径，按开始时刻的快照继续')
            return True
        now = ss.TargetData.from_dict(data)
        if str(now.id) != str(locked.id):
            self._fb.target_status = 'id_changed'
            report.exit_reason = 'target_id_changed'
            report.detail = f'目标 id 由 {locked.id} 变为 {now.id}'
            return False
        if self._stamp_clock() - now.stamp > self.target_stale_sec:
            self._fb.target_status = 'stale'
            report.warnings.append('段间复查：目标数据过期，按开始时刻的快照继续')
            return True
        if spec.target_static:
            reason = self._target_moved(spec, locked, now)
            if reason:
                self._fb.target_status = 'moved'
                report.exit_reason = 'target_moved'
                report.detail = reason
                return False
        self._fb.target_status = 'ok'
        return True

    def _target_moved(self, spec: ss.ShotSpec, locked: ss.TargetData, now: ss.TargetData) -> str:
        """@brief target_static 的位移判据（6.2.4：阈值要高于感知噪声、按轴分开看）：
               target 系按纵深（相机 → 目标方向）对 tolerance.d、水平横向对 d·tolerance.az、竖向对 d·tolerance.el；
               world 系对 tolerance.position；各阈值不低于 target_moved_floor_m。
        @return 空串 = 没动；否则人话原因
        """
        delta = np.asarray(now.position, dtype=float) - np.asarray(locked.position, dtype=float)
        tol = spec.tolerance
        floor = self.target_moved_floor_m
        if spec.position_ref == 'world':
            dist = float(np.linalg.norm(delta))
            thr = max(float(tol.position), floor)
            return f'目标移动了 {dist:.3f} m（阈值 {thr:.3f} m）' if dist > thr else ''
        d_min = min(wp.d for wp in spec.waypoints)
        pose = self.api.arm.get_pose()
        if pose is not None:
            _, p_a = self.compiler.frames.optical_from_flange(pose)
            cam = self.compiler.world.to_world_point(p_a)
        else:
            cam = self._static.waypoints[0].position
        depth_axis = np.asarray(locked.position, dtype=float) - cam
        depth_axis[2] = 0.0
        if np.linalg.norm(depth_axis) < 1e-6:
            depth_axis = np.array([1.0, 0.0, 0.0])
        depth_axis /= np.linalg.norm(depth_axis)
        lateral_axis = np.array([-depth_axis[1], depth_axis[0], 0.0])
        parts = (
            ('纵深', abs(float(np.dot(delta, depth_axis))), max(float(tol.d), floor)),
            ('横向', abs(float(np.dot(delta, lateral_axis))), max(d_min * float(tol.az), floor)),
            ('竖向', abs(float(delta[2])), max(d_min * float(tol.el), floor)),
        )
        over = [f'{name} {val:.3f} m > {thr:.3f} m' for name, val, thr in parts if val > thr]
        return ('声明 target_static 的目标动了：' + '、'.join(over)) if over else ''

    # ── 下发一条原语 ──
    def _send(self, prim: Primitive, av: Optional[Dict[str, Any]]) -> Any:
        """@brief 下发一条原语并阻塞到结果；期间用 Commander 反馈更新 progress / error。

        progress_percent 只用来判断"还没开始动"（≤ 搬到起点 + 停顿的份额），运镜段内的路径进度由
        current_pose 在原语几何上投影（primitive_progress）得到。goal 结果返回后到达的迟到反馈按代号丢弃。
        """
        if self._cancel:
            return _CancelledResult()
        offset = prim.progress_offset
        seg_index = max(prim.segment_index, 0)
        hold_t0 = self._hold_t0
        # 原语起点（搬到起点 / 停顿期间臂就停在这里）在参考轨迹上的位置：PTP 原语 = 首点
        start_at = (0, 0.0) if prim.kind == 'ptp' else (seg_index, prim.s0)
        with self._lock:
            self._gen += 1
            gen = self._gen

        def _cb(fb_msg: Any) -> None:
            """@brief Commander 反馈 → 规范反馈（跑在 rclpy 执行器线程，整段持锁，并在锁内复核代号）。"""
            with self._lock:
                if gen != self._gen or not self._active:
                    return
                try:
                    pct = float(getattr(fb_msg, 'progress_percent', 0.0))
                    pose = getattr(fb_msg, 'current_pose', None)
                    if prim.kind == 'ptp':
                        self._fb.phase, self._fb.segment_index, self._fb.progress = 'segment', 0, 0.0
                        self._fb.hold_elapsed = 0.0
                        self._update_error(pose, *start_at)
                    elif pct <= offset + 1e-6:
                        # 运镜段还没开始：Commander 正把臂搬到本原语起点（ORBIT 还含规划期）。
                        # 这一段属于走位（规范 2.3），容差只约束运镜段，所以**不判超差**——
                        # 拿"离起点还差多远"去填 in_tolerance，会让上层把一镜完美的运镜判成「完成但有降级」。
                        if hold_t0 is not None:
                            self._fb.phase = 'hold'
                            self._fb.hold_elapsed = max(0.0, self._clock() - hold_t0)
                            # 停在上一个 waypoint：报"刚走完的那一段"（首点时为 0）与 progress 1.0（9.1）
                            self._fb.segment_index = seg_index - 1 if seg_index > 0 else 0
                            self._fb.progress = 1.0
                        else:
                            self._fb.phase, self._fb.segment_index, self._fb.progress = 'segment', seg_index, prim.s0
                            self._fb.hold_elapsed = 0.0
                        self._clear_error()
                    else:
                        self._fb.phase, self._fb.segment_index, self._fb.hold_elapsed = 'segment', seg_index, 0.0
                        local = primitive_progress(prim, pose) if pose is not None else 0.0
                        self._fb.progress = prim.s0 + (prim.s1 - prim.s0) * local
                        self._update_error(pose, seg_index, self._fb.progress)
                    self._publish()
                except Exception as exc:  # pylint: disable=broad-except  别让用户回调的异常杀掉 spin 线程
                    self._log.warning(f'运镜反馈处理异常：{exc!r}')

        def _camera_ready() -> None:
            """@brief 第一条运镜 goal 的 camera_ready 上升沿。"""
            self._fire_camera_ready(av)

        timeout = max(60.0, prim.nominal_sec * 2.0 + 30.0)
        arm = self.api.arm
        self._active = True
        try:
            if prim.kind == 'ptp':
                return arm.move_to_pose(self._make_pose(prim.pose), prim.speed, return_to_start=False,
                                        timeout_sec=timeout, feedback_cb=_cb)
            if prim.kind == 'linear':
                return arm.shot_linear(self._make_pose(prim.start), self._make_pose(prim.end), prim.speed,
                                       return_to_start=False, timeout_sec=timeout,
                                       on_camera_ready=_camera_ready, feedback_cb=_cb)
            return arm.shot_orbit([float(v) for v in prim.center], prim.az0, prim.az1, prim.el0, prim.el1,
                                  prim.r0, prim.r1, prim.speed, return_to_start=False, timeout_sec=timeout,
                                  on_camera_ready=_camera_ready, feedback_cb=_cb)
        finally:
            with self._lock:
                self._active = False
                self._gen += 1

    def _fire_camera_ready(self, av: Optional[Dict[str, Any]]) -> None:
        """@brief 只触发一次 on_camera_ready(av)。"""
        if not self._camera_ready_fired:
            self._camera_ready_fired = True
            if self.on_camera_ready is not None:
                try:
                    self.on_camera_ready(av)
                except Exception as exc:  # pylint: disable=broad-except
                    self._log.warning(f'on_camera_ready 回调异常：{exc!r}')

    def _hold(self, seg_index: int, seconds: float, shorten: bool, at: tuple) -> None:
        """@brief 在某个 waypoint 停留：本地睡 seconds（后面还有 goal 时扣掉 Commander 自带的起点停顿），
               期间按节拍回报 phase=hold / hold_elapsed，误差相对臂所停的那个 waypoint。
        @param seg_index 反馈里报的段号（9.1：刚走完的那一段，首点为 0）
        @param seconds   规范里的 hold
        @param shorten   后面还有 goal → 扣掉 dwell_sec
        @param at        臂所停 waypoint 在参考轨迹上的 (段号, 进度)：首点 (0, 0.0)，其余 (i, 1.0)
        """
        wait = seconds - (self._compiled.dwell_sec if shorten else 0.0)
        t0 = self._clock()
        with self._lock:
            self._hold_t0 = t0
            self._fb.phase = 'hold'
            self._fb.segment_index = seg_index
            self._fb.progress = 1.0
            self._fb.hold_elapsed = 0.0
            self._update_error(self.api.arm.get_pose(), *at)
            self._publish()
        while not self._cancel:
            elapsed = self._clock() - t0
            if elapsed >= wait - 1e-9:
                break
            self._sleep(min(self.hold_tick_sec, wait - elapsed))
            with self._lock:
                self._fb.hold_elapsed = self._clock() - t0
                self._update_error(self.api.arm.get_pose(), *at)
                self._publish()
        if not shorten:
            self._hold_t0 = None

    # ── 主入口 ──
    def run(self, shot: Dict[str, Any]) -> ShotReport:
        """@brief 执行一段规范 JSON（一个分镜的运镜段），阻塞到 done / 失败 / 取消。
        @param shot 规范 JSON（dict）
        @return ShotReport
        """
        self._cancel = False
        self._camera_ready_fired = False
        self._hold_t0 = None
        report = ShotReport()
        try:
            spec = ss.parse_shot(shot, self.lens, self.aspect, self.base_fps)
        except ss.SpecValidationError as exc:
            report.errors = [e.to_dict() for e in exc.report.errors]
            report.detail = '；'.join(str(e) for e in exc.report.errors[:3])
            self._log.error(str(exc))
            return report
        report.warnings = [str(w) for w in spec.warnings]
        report.av = spec.av
        static, locked = spec, None
        self._tracking, self._locked = None, None
        if spec.is_tracking:
            locked = self._read_target(spec, report)
            if locked is None:
                self._log.error(f'B 型分镜无法开始：{report.detail}')
                return report
            try:
                static = ss.tracking_to_static(spec, locked)
            except ValueError as exc:
                report.exit_reason = 'rejected'
                report.detail = str(exc)
                report.errors.append(ss.SpecError('6.3', 'waypoints', str(exc)).to_dict())
                self._log.error(f'B 型分镜无法展开：{exc}')
                return report
            self._tracking, self._locked = spec, locked
            report.warnings.append(
                f'B 型按开始时刻的目标快照（id={locked.id}）展开为 A 型开环执行：uv 不闭环、time_scale 恒 1.0；'
                + ('目标声明静止，段间复查位置' if spec.target_static else '目标可能移动，段间只复查 id'))
        self._static = static
        try:
            compiled = self.compiler.compile(static, self._start_joints())
        except ss.SpecValidationError as exc:
            report.errors = [e.to_dict() for e in exc.report.errors]
            rules = {e.rule for e in exc.report.errors}
            report.exit_reason = 'unreachable' if rules <= {25, 26} else 'rejected'
            report.detail = '；'.join(e.message for e in exc.report.errors[:3])
            self._log.error(str(exc))
            return report
        self._compiled = compiled
        report.compiled = compiled
        report.warnings += compiled.warnings
        report.degradation = compiled.degradation
        report.plan_steps = compiled.plan_steps()
        self._log.info('运镜编译完成：\n' + compiled.summary())

        self._fb = ShotFeedback(degradation=compiled.degradation,
                                time_scale=1.0 if spec.is_tracking else None,
                                target_status='ok' if spec.is_tracking else None)
        self._clear_error()
        holds = compiled.holds
        n_seg = len(static.segments)

        def _finish(reason: str, detail: str = '') -> ShotReport:
            """@brief 收尾：写报告、发最后一帧（先作废 goal 代号，迟到的反馈不再改动状态）。"""
            with self._lock:
                self._active = False
                self._gen += 1
                report.ok = reason == 'done'
                report.exit_reason = reason
                if detail:
                    report.detail = detail
                self._publish()
                report.feedback = self._fb.to_dict()
            (self._log.info if report.ok else self._log.warning)(report.user_line())
            return report

        def _fail(result: Any) -> ShotReport:
            """@brief 某条 goal 失败。"""
            reason = str(getattr(result, 'reason', result))
            if any(w in reason.lower() for w in _LIMIT_WORDS):
                self._fb.deviation_cause = 'limit'
            return _finish('failed', f'Commander 返回 {reason}')

        # 起始 PTP（首点有 hold，或 0 段分镜）：到位即算"到达起拍点"，首点的停留也要进素材
        for prim in compiled.primitives_of_segment(-1):
            self._fb.phase, self._fb.segment_index, self._fb.progress = 'segment', 0, 0.0
            result = self._send(prim, spec.av)
            if self._cancel:
                return _finish('cancelled')
            if not result:
                return _fail(result)
            self._fire_camera_ready(spec.av)
        with self._lock:
            self._fb.progress = 1.0
            self._update_error(self.api.arm.get_pose(), 0, 0.0)
        if holds[0] > 0:
            self._hold(0, holds[0], shorten=n_seg > 0, at=(0, 0.0))
            if self._cancel:
                return _finish('cancelled')

        for i in range(n_seg):
            if spec.is_tracking and i > 0 and not self._boundary_check(spec, locked, report):
                return _finish(report.exit_reason, report.detail)
            for prim in compiled.primitives_of_segment(i):
                if self._hold_t0 is None:
                    with self._lock:
                        self._fb.phase, self._fb.segment_index, self._fb.progress = 'segment', i, prim.s0
                        self._fb.hold_elapsed = 0.0
                result = self._send(prim, spec.av)
                self._hold_t0 = None
                if self._cancel:
                    return _finish('cancelled')
                if not result:
                    self._fb.segment_index = i
                    return _fail(result)
            with self._lock:
                # 段走完补发一帧 progress=1.0：反馈频率与 goal 结束之间有间隙，最后一帧 Commander 反馈时
                # 臂往往还差几个百分点（实测 0.957），不补这一帧上层就判不出这一段真的走完了
                self._fb.phase, self._fb.segment_index, self._fb.progress = 'segment', i, 1.0
                self._fb.hold_elapsed = 0.0
                self._update_error(self.api.arm.get_pose(), i, 1.0)
                self._publish()
            if holds[i + 1] > 0:
                self._hold(i, holds[i + 1], shorten=i < n_seg - 1, at=(i, 1.0))
                if self._cancel:
                    return _finish('cancelled')
        with self._lock:
            self._fb.phase = 'done'
            self._fb.hold_elapsed = 0.0
            self._fb.segment_index = max(n_seg - 1, 0)
            self._fb.progress = 1.0
            self._hold_t0 = None
        return _finish('done')


# ─────────────────────────────── ROS 胶水 ───────────────────────────────

def make_shot_feedback_publisher(node: Any, topic: Optional[str] = SHOT_FEEDBACK_TOPIC,
                                 depth: int = 10) -> Optional[Callable[[str], None]]:
    """@brief 反馈 → std_msgs/String（JSON）话题。
    @param node  rclpy 节点
    @param topic 话题名；None 不发
    @param depth QoS 深度
    @return 文本出口回调或 None
    """
    from .llm_shot_loop import make_feedback_publisher   # noqa: WPS433
    return make_feedback_publisher(node, topic, depth)


def make_target_provider(node: Any, topic: str, msg_type: str = 'json') -> Callable[[], Optional[Dict[str, Any]]]:
    """@brief 订阅目标话题，返回"取最新一帧"的可调用（给 ShotExecutor.target_provider）。
    @param node     rclpy 节点
    @param topic    话题名（B 型 JSON 的 target 字段）
    @param msg_type 'json'：std_msgs/String，内容是 6.2.4 口径的 JSON；
                    'point'：geometry_msgs/PointStamped（WBC 的 ~/subject 口径），id 固定 'point'、
                    stamp 取 header（ROS 时间；实机 = 墙钟，与 time.time 同基准）
    @return 无参可调用 → dict 或 None
    """
    import threading
    latest: Dict[str, Any] = {}
    lock = threading.Lock()

    def _on_json(msg: Any) -> None:
        """@brief String → dict。"""
        try:
            data = json.loads(msg.data)
        except ValueError:
            node.get_logger().warning(f'{topic} 收到非 JSON 数据，忽略')
            return
        with lock:
            latest['data'] = data

    def _on_point(msg: Any) -> None:
        """@brief PointStamped → 6.2.4 口径。"""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with lock:
            latest['data'] = {'stamp': stamp, 'id': 'point',
                              'position': [msg.point.x, msg.point.y, msg.point.z], 'confidence': 1.0}

    if msg_type == 'json':
        from std_msgs.msg import String   # noqa: WPS433
        node.create_subscription(String, topic, _on_json, 10)
    elif msg_type == 'point':
        from geometry_msgs.msg import PointStamped   # noqa: WPS433
        node.create_subscription(PointStamped, topic, _on_point, 10)
    else:
        raise ValueError(f'msg_type 只能是 json / point，收到 {msg_type!r}')

    def _get() -> Optional[Dict[str, Any]]:
        """@brief 最新一帧。"""
        with lock:
            return dict(latest['data']) if 'data' in latest else None
    return _get


def lookup_world_frame(node: Any, world_frame: str = 'odom', arm_frame: str = 'arm_base_link',
                       timeout_sec: float = 2.0) -> WorldFrame:
    """@brief 用 tf2 查 world_frame → arm_frame，得到 WorldFrame。需要节点正在 spin。
    @param node        rclpy 节点
    @param world_frame 规范的世界系（odom）
    @param arm_frame   Commander 的基座系
    @param timeout_sec 等 TF 的时长
    @return WorldFrame
    @throws RuntimeError 查不到
    """
    from rclpy.duration import Duration   # noqa: WPS433
    from tf2_ros import Buffer, TransformListener   # noqa: WPS433
    buf = Buffer()
    _listener = TransformListener(buf, node)   # noqa: F841  保持引用
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if buf.can_transform(world_frame, arm_frame, node.get_clock().now(), Duration(seconds=0.2)):
            tf_msg = buf.lookup_transform(world_frame, arm_frame, node.get_clock().now())
            q = tf_msg.transform.rotation
            t = tf_msg.transform.translation
            # 四元数 → 旋转矩阵
            x, y, z, w = q.x, q.y, q.z, q.w
            rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            tf = np.eye(4)
            tf[:3, :3] = rot
            tf[:3, 3] = [t.x, t.y, t.z]
            return WorldFrame.from_matrix(tf)
        time.sleep(0.05)
    raise RuntimeError(f'{timeout_sec} s 内查不到 TF {world_frame} → {arm_frame}')
