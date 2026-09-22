# -*- coding: utf-8 -*-
"""
@file      shot_spec.py
@brief     《摄影机器人·拍摄接口规范 v9》（层级 2 运镜 JSON）的解析、第 8 章校验与参考轨迹几何。
           纯 numpy，不依赖 ROS；是 shot_compiler（规范 JSON → Commander goal）的前置层。
@version   0.1
@date      2026-09-16
@copyright Copyright (c) 2026 eMeet

覆盖的规范内容：
  · A 型 static：waypoints(position/look_at/roll/hold) + segments(path/law/duration|speed, arc 的 center/axis)
    + tolerance(position/aim/roll)，参考轨迹按 5.3.7 插值（look_at 点插值 / 纯摇镜按角度插值 / roll 线性）。
  · B 型 tracking：target/target_static/position_ref + d/az/el|position + uv/roll/hold + 六项容差；
    tracking_to_static() 用一帧目标数据（6.2.4 口径）把它展开成等价的 A 型（"编译期冻结取样"，3.4）。
  · 校验规则 1–24、28（告警）、29–35、37–40；36 需要目标尺寸、不在本层。每条返回结构化 SpecError。
  · av 五组只校验、不执行（相机 / 录音模块另行消费）。

坐标约定（4.1）：世界系 Z 向上、米、弧度；受控体是相机光心；光学系 +X 右 / +Y 下 / +Z 前；
roll 正 = 从相机背后沿光轴看顺时针；uv = (X/Z, Y/Z)。

═══════════════════════════════ 本文件函数 / 类汇总 ═══════════════════════════════
  SpecError / ValidationReport / SpecValidationError   结构化错误：rule / field / message / suggestion
  Waypoint / Segment / Tolerance                       A 型三件
  TrackingWaypoint / TrackingTolerance                 B 型三件（segments 与 A 型共用）
  ShotSpec                                             一段运镜（两型共用容器）
  validate_shot(obj, lens, aspect, base_fps)           第 8 章校验 → ValidationReport（不抛）
  parse_shot(obj, ...)                                 校验 + 解析 → ShotSpec；有错抛 SpecValidationError
  camera_rotation(position, look_at, roll)             光心位置 + 对准点 + roll → 光学系 3×3（列 = 右/下/前）
  rotation_from_forward(forward, roll)                 光轴方向 + roll → 3×3
  log_so3 / exp_so3 / slerp                            SO(3) 对数 / 指数 / 最短弧插值
  uv_of(rotation, position, target)                    目标点 → 归一化像坐标 (u, v)
  pure_pan(spec, i) / segment_length(spec, i)          零长度段判定 / 段路径弧长
  sph_to_cart(target, d, az, el)                       目标球坐标 → 世界点
  spherical_param(spec, i, s)                          B 型球坐标段进度 s 处的 (d, az, el)
  TrackInfo                                            B 型展开后附在段上的目标 / uv / 球坐标信息
  path_position(spec, i, s)                            段内位置（s 按弧长归一化）
  reference_pose(spec, i, s)                           段内完整参考位姿 CamRef（position / rotation / look_at / roll）
  TargetData / validate_target_data(d, position_ref)   B 型运行时数据（6.2.4）
  tracking_to_static(spec, target)                     B 型 → 等价 A 型（按目标快照展开，含 uv → look_at 求解）
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

# ─────────────────────────────── 枚举与常量（附录 A.4） ───────────────────────────────

TYPES = ('static', 'tracking')
PATHS_A = ('line', 'arc', 'spline')
PATHS_B = ('line', 'spline')
LAWS = ('constant', 'trapezoid', 's_curve', 'ease_in', 'ease_out', 'ease_in_out')
POSITION_REFS = ('target', 'target_front', 'world')
LENSES = ('WIDE', 'TELE')
RECORDING_MODES = ('VIDEO', 'PHOTO')
FOCUS_MODES = ('CONTINUOUS', 'ONCE', 'LOCKED')
FOCUS_TARGETS = ('SUBJECT_ANCHOR', 'CENTER', 'MANUAL')
EXPOSURE_MODES = ('AUTO', 'MANUAL')
BEAMFORMS = ('TARGET', 'OMNI', 'OFF')

#: uv 出画边界 (|u|max, |v|max)，按 (镜头, 交付画幅)（6.3.6 ①②）。TELE 视场待实测 → 不在表里。
FOV_LIMITS: Dict[Tuple[str, str], Tuple[float, float]] = {
    ('WIDE', '4:3'): (0.539, 0.405),
    ('WIDE', '16:9'): (0.539, 0.303),
    ('WIDE', '1:1'): (0.405, 0.405),
    ('WIDE', '9:16'): (0.228, 0.405),
}

#: 附录 C.5 容差档位表的"紧档"（规则 23 用）
TIGHT_TIER_A = {'position': 0.03, 'aim': 0.02, 'roll': 0.02}
TIGHT_TIER_B = {'d': 0.10, 'az': 0.05, 'el': 0.05, 'u': 0.02, 'v': 0.02, 'roll': 0.02}
#: 规则 28：d 容差下限 = D_TOL_FLOOR_RATIO × d
D_TOL_FLOOR_RATIO = 0.08

_A_TOP = {'type', 'waypoints', 'segments', 'tolerance', 'av'}
_B_TOP = _A_TOP | {'target', 'target_static', 'position_ref'}
_A_WP = {'position', 'look_at', 'roll', 'hold'}
_B_WP = {'d', 'az', 'el', 'position', 'uv', 'roll', 'hold'}
_ZERO_LEN_EPS = 1e-6


# ─────────────────────────────── 结构化错误 ───────────────────────────────

@dataclass
class SpecError:
    """@brief 一条校验结果：违反了第 8 章哪条规则、在哪个字段、为什么、建议值（可选）。"""
    rule: Union[int, str]
    field: str
    message: str
    suggestion: Any = None

    def to_dict(self) -> Dict[str, Any]:
        """@brief 转成可 JSON 化的字典（回给大模型改一版用）。
        @return {'rule', 'field', 'message', 'suggestion'}
        """
        out = {'rule': self.rule, 'field': self.field, 'message': self.message}
        if self.suggestion is not None:
            out['suggestion'] = _jsonable(self.suggestion)
        return out

    def __str__(self) -> str:
        return f'[规则 {self.rule}] {self.field}: {self.message}'


@dataclass
class ValidationReport:
    """@brief validate_shot 的输出：errors 非空则不得执行；warnings 只提示。"""
    errors: List[SpecError] = field(default_factory=list)
    warnings: List[SpecError] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """@brief 没有错误。
        @return bool
        """
        return not self.errors

    def to_dict(self) -> Dict[str, Any]:
        """@brief 转成字典。
        @return {'ok', 'errors', 'warnings'}
        """
        return {'ok': self.ok, 'errors': [e.to_dict() for e in self.errors],
                'warnings': [w.to_dict() for w in self.warnings]}


class SpecValidationError(ValueError):
    """@brief parse_shot / 编译阶段的校验失败：report 里是全部结构化错误。"""

    def __init__(self, report: ValidationReport, prefix: str = '运镜 JSON 不合法'):
        """@brief 构造。
        @param report 校验报告
        @param prefix 消息前缀
        """
        self.report = report
        lines = [prefix] + [f'  {e}' for e in report.errors]
        super().__init__('\n'.join(lines))


# ─────────────────────────────── 数据结构 ───────────────────────────────

@dataclass
class Waypoint:
    """@brief A 型路点：光心世界坐标 + 光轴对准的世界点 + 画面倾斜 + 到达后停留。"""
    position: np.ndarray
    look_at: np.ndarray
    roll: float = 0.0
    hold: float = 0.0


@dataclass
class TrackInfo:
    """@brief B 型展开成 A 型后附在段上的信息（tracking_to_static 生成，用户 JSON 里没有）：
           目标位置、两端 uv（段内线性插值，6.4.3）、position_ref=target/target_front 时全部路点的 (d, az_world, el)
           （位置在目标球坐标里插值，走出来才是绕目标的弧；world 时为 None，位置走世界直角坐标）。"""
    target: np.ndarray
    uv0: Tuple[float, float]
    uv1: Tuple[float, float]
    sph: Optional[List[Tuple[float, float, float]]] = None


@dataclass
class Segment:
    """@brief 相邻两个路点之间怎么走：路径形状 / 速度曲线 / 时长或线速度；arc 专属 center/axis；
           track 只在 B 型展开后存在。"""
    path: str = 'line'
    law: str = 's_curve'
    duration: Optional[float] = None
    speed: Optional[float] = None
    center: Optional[np.ndarray] = None
    axis: Optional[np.ndarray] = None
    track: Optional[TrackInfo] = None


@dataclass
class Tolerance:
    """@brief A 型容差：光心位置半径（m）/ 光轴指向角（rad）/ 画面倾斜（rad）。"""
    position: float
    aim: float
    roll: float


@dataclass
class TrackingWaypoint:
    """@brief B 型路点：d/az/el（target / target_front）或 position（world）+ uv + roll + hold。"""
    d: Optional[float] = None
    az: Optional[float] = None
    el: Optional[float] = None
    position: Optional[np.ndarray] = None
    uv: Tuple[float, float] = (0.0, 0.0)
    roll: float = 0.0
    hold: float = 0.0


@dataclass
class TrackingTolerance:
    """@brief B 型容差：位置部分随 position_ref 变，u / v / roll 通用。"""
    u: float
    v: float
    roll: float
    d: Optional[float] = None
    az: Optional[float] = None
    el: Optional[float] = None
    position: Optional[float] = None


@dataclass
class ShotSpec:
    """@brief 一段运镜（一个分镜的运镜段）。type 决定 waypoints / tolerance 的具体类型。"""
    type: str
    waypoints: List[Any]
    segments: List[Segment]
    tolerance: Any
    target: Optional[str] = None
    target_static: bool = False
    position_ref: Optional[str] = None
    av: Optional[Dict[str, Any]] = None
    extra: Dict[str, Any] = field(default_factory=dict)
    warnings: List[SpecError] = field(default_factory=list)
    source_target_id: Optional[Union[str, int]] = None   # B 型展开成 A 型时记录目标 id

    @property
    def is_tracking(self) -> bool:
        """@brief 是否 B 型。
        @return bool
        """
        return self.type == 'tracking'

    def to_dict(self) -> Dict[str, Any]:
        """@brief 还原成规范 JSON 结构（数值转普通 float），日志 / 回传用。
        @return dict
        """
        out: Dict[str, Any] = {'type': self.type}
        if self.is_tracking:
            out.update({'target': self.target, 'target_static': self.target_static,
                        'position_ref': self.position_ref})
        out['waypoints'] = [_wp_dict(w) for w in self.waypoints]
        out['segments'] = [_seg_dict(s) for s in self.segments]
        out['tolerance'] = {k: v for k, v in vars(self.tolerance).items() if v is not None}
        if self.av is not None:
            out['av'] = self.av
        out.update(self.extra)
        return out


# ─────────────────────────────── 小工具 ───────────────────────────────

def _jsonable(value: Any) -> Any:
    """@brief numpy → 普通 Python 类型。
    @param value 任意值
    @return 可 json.dumps 的值
    """
    if isinstance(value, np.ndarray):
        return [float(v) for v in value]
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


def _wp_dict(wp: Any) -> Dict[str, Any]:
    """@brief 路点 → 字典（去掉 None）。"""
    out = {}
    for key, val in vars(wp).items():
        if val is None:
            continue
        out[key] = _jsonable(val)
    return out


def _seg_dict(seg: Segment) -> Dict[str, Any]:
    """@brief 段 → 字典（去掉 None；B 型展开附带的 track 不进 JSON）。"""
    return {k: _jsonable(v) for k, v in vars(seg).items() if v is not None and k != 'track'}


def _is_num(value: Any) -> bool:
    """@brief 是否有限实数（bool 不算）。"""
    return isinstance(value, (int, float, np.floating, np.integer)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def _vec3(value: Any) -> Optional[np.ndarray]:
    """@brief 3 个有限实数的序列 → ndarray，否则 None。"""
    if isinstance(value, (list, tuple, np.ndarray)) and len(value) == 3 and all(_is_num(v) for v in value):
        return np.asarray([float(v) for v in value], dtype=float)
    return None


def _unit(vec: np.ndarray) -> np.ndarray:
    """@brief 归一化。"""
    return vec / np.linalg.norm(vec)


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    """@brief 绕单位轴转 angle 的 3×3 旋转。"""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """@brief 两向量夹角 [0, π]。"""
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return math.acos(max(-1.0, min(1.0, float(np.dot(a, b)) / (na * nb))))


def log_so3(rot: np.ndarray) -> np.ndarray:
    """@brief 旋转矩阵 → 旋转向量（单位轴 × 转角）。
    @param rot 3×3
    @return (3,)
    """
    cos_a = max(-1.0, min(1.0, (float(np.trace(rot)) - 1.0) * 0.5))
    ang = math.acos(cos_a)
    if ang < 1e-12:
        return np.zeros(3)
    if math.pi - ang < 1e-6:
        sym = 0.5 * (rot + np.eye(3))          # 接近 π：从对称部分取轴
        diag = np.sqrt(np.maximum(np.diag(sym), 0.0))
        idx = int(np.argmax(diag))
        axis = sym[:, idx] / max(diag[idx], 1e-12)
        return ang * axis / np.linalg.norm(axis)
    vec = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return vec * (ang / (2.0 * math.sin(ang)))


def exp_so3(vec: np.ndarray) -> np.ndarray:
    """@brief 旋转向量 → 旋转矩阵。
    @param vec (3,)
    @return 3×3
    """
    ang = float(np.linalg.norm(vec))
    if ang < 1e-12:
        return np.eye(3)
    return _rodrigues(vec / ang, ang)


def slerp(rot0: np.ndarray, rot1: np.ndarray, u: float) -> np.ndarray:
    """@brief 两个旋转之间沿 SO(3) 最短弧插值（等价于四元数 slerp 取短弧，与 Commander 的 quat_slerp 一致）。
    @param rot0,rot1 3×3
    @param u         ∈ [0, 1]
    @return 3×3
    """
    return rot0 @ exp_so3(u * log_so3(rot0.T @ rot1))


# ─────────────────────────────── 校验（第 8 章） ───────────────────────────────

class _Ctx:
    """@brief 校验过程的累加器。"""

    def __init__(self) -> None:
        self.report = ValidationReport()

    def err(self, rule: Union[int, str], field_: str, message: str, suggestion: Any = None) -> None:
        """@brief 记一条错误。"""
        self.report.errors.append(SpecError(rule, field_, message, suggestion))

    def warn(self, rule: Union[int, str], field_: str, message: str, suggestion: Any = None) -> None:
        """@brief 记一条告警。"""
        self.report.warnings.append(SpecError(rule, field_, message, suggestion))


def validate_shot(obj: Dict[str, Any], lens: str = 'WIDE', aspect: str = '4:3',
                  base_fps: Optional[float] = None) -> ValidationReport:
    """@brief 第 8 章校验（1–24、28 告警、29–35、37–40），每条独立检查、全部返回。
    @param obj      规范 JSON（dict）
    @param lens     这条分镜用哪个镜头（'WIDE' 25mm / 'TELE' 50mm），决定规则 21 的出画边界（8.6）
    @param aspect   整片交付画幅 '4:3' / '16:9' / '1:1' / '9:16'，同上
    @param base_fps 基准帧率，给了才查规则 40
    @return ValidationReport（errors 为空即可解析 / 编译）
    """
    ctx = _Ctx()
    if not isinstance(obj, dict):
        ctx.err(1, '$', '顶层必须是 JSON 对象')
        return ctx.report
    shot_type = obj.get('type')
    if shot_type not in TYPES:
        ctx.err(1, 'type', f'type 必须是 "static" 或 "tracking"，收到 {shot_type!r}')
        return ctx.report
    waypoints = obj.get('waypoints')
    segments = obj.get('segments')
    if not isinstance(waypoints, list) or not waypoints:
        ctx.err(3, 'waypoints', 'waypoints 必须是长度 ≥ 1 的数组')
        waypoints = []
    if not isinstance(segments, list):
        ctx.err(2, 'segments', 'segments 必须是数组（可以是 []）')
        segments = []
    elif waypoints and len(segments) != len(waypoints) - 1:
        ctx.err(2, 'segments', f'segments 长度必须等于 waypoints.length − 1 = {len(waypoints) - 1}，'
                f'收到 {len(segments)}')
    tol = obj.get('tolerance')
    if not isinstance(tol, dict):
        ctx.err(14 if shot_type == 'static' else 19, 'tolerance', 'tolerance 必填且为对象')
        tol = {}

    if shot_type == 'static':
        for key in ('target', 'target_static', 'position_ref'):
            if key in obj:
                ctx.err(1, key, f'A 型（static）不允许出现 B 型字段 {key}')
        _validate_static(ctx, waypoints, segments, tol)
    else:
        _validate_tracking(ctx, obj, waypoints, segments, tol, lens, aspect)
    _validate_segments_common(ctx, segments)
    if 'av' in obj and obj['av'] is not None:
        _validate_av(ctx, obj['av'], shot_type, base_fps)
    return ctx.report


def _check_hold(ctx: _Ctx, wp: Dict[str, Any], i: int) -> None:
    """@brief 规则 5 的 hold ≥ 0。"""
    if 'hold' in wp and (not _is_num(wp['hold']) or wp['hold'] < 0):
        ctx.err(5, f'waypoints[{i}].hold', 'hold 必须是 ≥ 0 的数')


def _check_roll(ctx: _Ctx, wp: Dict[str, Any], i: int) -> None:
    """@brief roll 类型。"""
    if 'roll' in wp and not _is_num(wp['roll']):
        ctx.err(1, f'waypoints[{i}].roll', 'roll 必须是数（rad）')


def _validate_segments_common(ctx: _Ctx, segments: List[Any]) -> None:
    """@brief 规则 4 / 5（段的时长与速度）与枚举（path / law）。零长度段的规则 6 在各型里查。"""
    for i, seg in enumerate(segments):
        pre = f'segments[{i}]'
        if not isinstance(seg, dict):
            ctx.err(4, pre, '每个 segment 必须是对象')
            continue
        has_d, has_s = 'duration' in seg, 'speed' in seg
        if has_d == has_s:
            ctx.err(4, pre, 'duration 与 speed 必须恰好写一个')
        if has_d and (not _is_num(seg['duration']) or seg['duration'] <= 0):
            ctx.err(5, f'{pre}.duration', 'duration 必须 > 0（秒）')
        if has_s and (not _is_num(seg['speed']) or seg['speed'] <= 0):
            ctx.err(5, f'{pre}.speed', 'speed 必须 > 0（m/s）')
        law = seg.get('law', 's_curve')
        if law not in LAWS:
            ctx.err('A.4', f'{pre}.law', f'law 必须是 {LAWS} 之一，收到 {law!r}')


def _validate_static(ctx: _Ctx, waypoints: List[Any], segments: List[Any], tol: Dict[str, Any]) -> None:
    """@brief 8.2 A 型规则 7–14 与 A 型的规则 1 / 6。"""
    positions: List[Optional[np.ndarray]] = []
    look_ats: List[Optional[np.ndarray]] = []
    for i, wp in enumerate(waypoints):
        pre = f'waypoints[{i}]'
        if not isinstance(wp, dict):
            ctx.err(7, pre, '每个 waypoint 必须是对象')
            positions.append(None)
            look_ats.append(None)
            continue
        for key in wp:
            if key in _B_WP - _A_WP:
                ctx.err(1, f'{pre}.{key}', f'A 型 waypoint 不允许出现 B 型字段 {key}')
        pos = _vec3(wp.get('position'))
        look = _vec3(wp.get('look_at'))
        if pos is None:
            ctx.err(7, f'{pre}.position', 'position 必填，[x, y, z] 米')
        if look is None:
            ctx.err(7, f'{pre}.look_at', 'look_at 必填，[x, y, z] 米')
        if pos is not None and look is not None and np.linalg.norm(look - pos) < 1e-6:
            ctx.err(8, f'{pre}.look_at', 'look_at 不能与 position 重合（光轴方向无定义）')
        _check_roll(ctx, wp, i)
        _check_hold(ctx, wp, i)
        positions.append(pos)
        look_ats.append(look)

    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            continue
        pre = f'segments[{i}]'
        path = seg.get('path', 'line')
        if path not in PATHS_A:
            ctx.err('A.4', f'{pre}.path', f'A 型 path 必须是 {PATHS_A} 之一，收到 {path!r}')
            continue
        p0 = positions[i] if i < len(positions) else None
        p1 = positions[i + 1] if i + 1 < len(positions) else None
        if path == 'arc':
            _validate_arc(ctx, seg, i, p0, p1)
        else:
            for key in ('center', 'axis'):
                if key in seg:
                    ctx.err(9, f'{pre}.{key}', f'{key} 只在 path="arc" 时允许出现')
        if p0 is not None and p1 is not None and np.linalg.norm(p1 - p0) < _ZERO_LEN_EPS:
            if 'speed' in seg:
                ctx.err(6, f'{pre}.speed', '零长度段（纯摇镜 / 纯俯仰）不能用 speed，必须用 duration')
            l0, l1 = look_ats[i], look_ats[i + 1]
            if l0 is not None and l1 is not None:
                sweep = _angle_between(l0 - p0, l1 - p1)
                if sweep >= math.pi - 1e-6:
                    ctx.err(13, f'waypoints[{i + 1}].look_at',
                            f'纯摇镜扫角 {math.degrees(sweep):.1f}° ≥ 180°：两端方向对顶，往左还是往右摇无法表达；'
                            '请在中间插一个 waypoint 说明走法')
                elif sweep > 1e-9 and _pan_passes_vertical(l0 - p0, l1 - p1):
                    ctx.err(13, f'waypoints[{i + 1}].look_at',
                            '纯摇镜的光轴大圆经过竖直方向（正上 / 正下），画面"上方"在该点无定义、roll 参考会翻转；'
                            '请让路径偏离竖直或拆成两段')
    if len(waypoints) == 2 and any(isinstance(s, dict) and s.get('path') == 'spline' for s in segments):
        ctx.warn(12, 'segments[0].path', '整条运镜只有 2 个 waypoint 却用 spline：样条会退化成直线，与 line 无区别')
    for key in ('position', 'aim', 'roll'):
        if key not in tol:
            ctx.err(14, f'tolerance.{key}', f'A 型 tolerance 必须有 {key}')
        elif not _is_num(tol[key]) or tol[key] <= 0:
            ctx.err(14, f'tolerance.{key}', f'tolerance.{key} 必须 > 0')
    for key in tol:
        if key in ('d', 'az', 'el', 'u', 'v'):
            ctx.err(1, f'tolerance.{key}', f'A 型 tolerance 不允许出现 B 型字段 {key}')


def _pan_passes_vertical(d0: np.ndarray, d1: np.ndarray, margin_deg: float = 1.0) -> bool:
    """@brief 两个方向之间的大圆短弧是否经过距竖直（±Z）不到 margin_deg 的地方。"""
    f0, f1 = _unit(d0), _unit(d1)
    n = np.cross(f0, f1)
    if np.linalg.norm(n) < 1e-9:
        return False
    n = _unit(n)
    total = _angle_between(f0, f1)
    z = np.array([0.0, 0.0, 1.0])
    for pole in (z, -z):
        proj = pole - np.dot(pole, n) * n          # 大圆上离该极最近的点
        if np.linalg.norm(proj) < 1e-9:
            continue                              # 大圆是水平的（法向就是竖直轴）：离两极最远，不可能经过
        q = _unit(proj)
        if _angle_between(q, pole) > math.radians(margin_deg):
            continue
        if _angle_between(f0, q) + _angle_between(q, f1) <= total + 1e-9:
            return True
    return False


def _validate_arc(ctx: _Ctx, seg: Dict[str, Any], i: int, p0: Optional[np.ndarray],
                  p1: Optional[np.ndarray]) -> None:
    """@brief 规则 9 / 10 / 11：arc 的 center / axis 齐全、端点垂直于轴且等距、起止不重合。"""
    pre = f'segments[{i}]'
    center = _vec3(seg.get('center'))
    axis = _vec3(seg.get('axis'))
    if center is None:
        ctx.err(9, f'{pre}.center', 'path="arc" 必须给 center [x, y, z]（相机绕行的圆心，不是被摄物位置）')
    if axis is None:
        ctx.err(9, f'{pre}.axis', 'path="arc" 必须给 axis [x, y, z]（转轴方向，右手定则）')
    elif np.linalg.norm(axis) < 1e-9:
        ctx.err(9, f'{pre}.axis', 'axis 必须是非零向量')
        axis = None
    if center is None or axis is None or p0 is None or p1 is None:
        return
    if np.linalg.norm(p1 - p0) < _ZERO_LEN_EPS:
        ctx.err(11, f'{pre}', 'arc 段的起点与终点重合：转 0 还是 2π 无法区分，绕满一圈请拆成两段')
        return
    a = _unit(axis)
    v0, v1 = p0 - center, p1 - center
    h0, h1 = float(np.dot(v0, a)), float(np.dot(v1, a))
    r0 = float(np.linalg.norm(v0 - h0 * a))
    r1 = float(np.linalg.norm(v1 - h1 * a))
    tol_m = 2e-3
    if abs(h0) > tol_m or abs(h1) > tol_m:
        suggested = center + a * (0.5 * (h0 + h1))
        ctx.err(10, f'{pre}.center',
                f'两个端点到 center 沿 axis 的分量为 {h0:.3f} / {h1:.3f} m，不在垂直于 axis 的平面上；'
                '最常见的原因是把 center 误填成了被摄物的位置（center 应与相机同高，见规范 5.5.2）',
                suggestion={'center': suggested})
    if abs(r0 - r1) > tol_m:
        ctx.err(10, f'{pre}.center', f'两个端点到 center 的距离不等（{r0:.3f} 与 {r1:.3f} m），不在同一圆上')


def _validate_tracking(ctx: _Ctx, obj: Dict[str, Any], waypoints: List[Any], segments: List[Any],
                       tol: Dict[str, Any], lens: str, aspect: str) -> None:
    """@brief 8.3 B 型规则 15–23、28（告警）与 B 型的规则 1 / 6。"""
    target = obj.get('target')
    if not isinstance(target, str) or not target.strip():
        ctx.err(15, 'target', 'target 必须是非空字符串（目标位姿数据源的话题名）')
    ref = obj.get('position_ref')
    if ref not in POSITION_REFS:
        ctx.err(16, 'position_ref', f'position_ref 必须是 {POSITION_REFS} 之一，收到 {ref!r}')
        ref = None
    static = obj.get('target_static', False)
    if not isinstance(static, bool):
        ctx.err('6.2.2', 'target_static', 'target_static 必须是布尔')
        static = False
    spherical = ref in ('target', 'target_front')
    limits = FOV_LIMITS.get((lens, aspect))
    if limits is None:
        ctx.warn(21, 'uv', f'镜头 {lens} × 画幅 {aspect} 的视场未知（50mm 待实测），未校验 uv 出画边界')

    azs: List[Optional[float]] = []
    ds: List[float] = []
    keys: List[Tuple[Any, ...]] = []
    for i, wp in enumerate(waypoints):
        pre = f'waypoints[{i}]'
        if not isinstance(wp, dict):
            ctx.err(17, pre, '每个 waypoint 必须是对象')
            azs.append(None)
            keys.append(())
            continue
        if 'look_at' in wp:
            ctx.err(1, f'{pre}.look_at', 'B 型 waypoint 没有 look_at（朝向由目标 + uv 决定）')
        if ref is not None:
            if spherical:
                for key in ('d', 'az', 'el'):
                    if key not in wp or not _is_num(wp[key]):
                        ctx.err(17, f'{pre}.{key}', f'position_ref="{ref}" 时 waypoint 必须有 {key}')
                if 'position' in wp:
                    ctx.err(17, f'{pre}.position', f'position_ref="{ref}" 时 waypoint 不能有 position')
                if _is_num(wp.get('d')) and wp['d'] <= 0:
                    ctx.err(20, f'{pre}.d', 'd（视距）必须 > 0')
            else:
                if _vec3(wp.get('position')) is None:
                    ctx.err(18, f'{pre}.position', 'position_ref="world" 时 waypoint 必须有 position [x, y, z]')
                for key in ('d', 'az', 'el'):
                    if key in wp:
                        ctx.err(18, f'{pre}.{key}', f'position_ref="world" 时 waypoint 不能有 {key}')
        uv = wp.get('uv', [0.0, 0.0])
        if not (isinstance(uv, (list, tuple)) and len(uv) == 2 and all(_is_num(v) for v in uv)):
            ctx.err(21, f'{pre}.uv', 'uv 必须是 [u, v] 两个数')
        elif limits is not None and (abs(uv[0]) > limits[0] or abs(uv[1]) > limits[1]):
            ctx.err(21, f'{pre}.uv', f'uv={list(uv)} 超出 {lens} × {aspect} 的有效画幅 '
                    f'|u| ≤ {limits[0]}、|v| ≤ {limits[1]}：目标已出画')
        _check_roll(ctx, wp, i)
        _check_hold(ctx, wp, i)
        azs.append(float(wp['az']) if _is_num(wp.get('az')) else None)
        if _is_num(wp.get('d')):
            ds.append(float(wp['d']))
        if spherical:
            keys.append(tuple(float(wp[k]) for k in ('d', 'az', 'el') if _is_num(wp.get(k))))
        else:
            pos = _vec3(wp.get('position'))
            keys.append(tuple(pos) if pos is not None else ())

    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            continue
        pre = f'segments[{i}]'
        path = seg.get('path', 'line')
        if path not in PATHS_B:
            ctx.err('A.4', f'{pre}.path', f'B 型 path 必须是 {PATHS_B} 之一（环绕靠 Δaz，没有 arc），收到 {path!r}')
        for key in ('center', 'axis'):
            if key in seg:
                ctx.err(1, f'{pre}.{key}', f'B 型 segment 没有 {key}')
        if i + 1 < len(keys) and keys[i] and keys[i] == keys[i + 1] and 'speed' in seg:
            ctx.err(6, f'{pre}.speed', '零长度段（只改 uv / roll）不能用 speed，必须用 duration')
        if i + 1 < len(azs) and azs[i] is not None and azs[i + 1] is not None \
                and abs(azs[i + 1] - azs[i]) > 2 * math.pi + 1e-9:
            ctx.warn(22, f'waypoints[{i + 1}].az', f'相邻 waypoint |Δaz| = {abs(azs[i + 1] - azs[i]):.3f} > 2π：'
                     '多半是把"顺时针 90°"写成了"逆时针 270°"（绕多圈是合法的，故只告警）')

    for key in ('u', 'v', 'roll'):
        if key not in tol or not _is_num(tol[key]) or tol[key] <= 0:
            ctx.err(19, f'tolerance.{key}', f'B 型 tolerance 必须有 {key} 且 > 0（三种 position_ref 都要）')
    if 'aim' in tol:
        ctx.err(1, 'tolerance.aim', 'B 型 tolerance 没有 aim（用 u / v）')
    if ref is not None:
        if spherical:
            for key in ('d', 'az', 'el'):
                if key not in tol or not _is_num(tol[key]) or tol[key] <= 0:
                    ctx.err(17, f'tolerance.{key}', f'position_ref="{ref}" 时 tolerance 必须有 {key} 且 > 0')
            if 'position' in tol:
                ctx.err(17, 'tolerance.position', f'position_ref="{ref}" 时 tolerance 不能有 position')
        else:
            if 'position' not in tol or not _is_num(tol['position']) or tol['position'] <= 0:
                ctx.err(18, 'tolerance.position', 'position_ref="world" 时 tolerance 必须有 position 且 > 0')
            for key in ('d', 'az', 'el'):
                if key in tol:
                    ctx.err(18, f'tolerance.{key}', f'position_ref="world" 时 tolerance 不能有 {key}')
    if static and ref is not None:
        loose = []
        checks = (('d', 'az', 'el') if spherical else ('position',))
        tiers = TIGHT_TIER_B if spherical else {'position': TIGHT_TIER_A['position']}
        for key in checks:
            if _is_num(tol.get(key)) and tol[key] > tiers[key] + 1e-9:
                loose.append(f'{key}={tol[key]} > {tiers[key]}')
        if loose:
            ctx.err(23, 'tolerance', 'target_static: true 却把位置容差写得比"静止目标"档更松：' + '、'.join(loose)
                    + '。既声明目标不动又允许大幅偏离，意图矛盾（附录 C.5）')
    if spherical and ds and _is_num(tol.get('d')):
        floor = D_TOL_FLOOR_RATIO * max(ds)
        if tol['d'] < floor - 1e-9:
            ctx.warn(28, 'tolerance.d', f'd 容差 {tol["d"]} 小于感知精度下限 {D_TOL_FLOOR_RATIO} × d = {floor:.3f} m'
                     '（ToF 融合后相对误差约 5%），现场大概率保不住')


def _validate_av(ctx: _Ctx, av: Any, shot_type: str, base_fps: Optional[float]) -> None:
    """@brief 8.5 规则 29–35、37–40（36 需要目标尺寸，不在本层）。"""
    if not isinstance(av, dict):
        ctx.err('7', 'av', 'av 必须是对象')
        return
    lens = av.get('lens') or {}
    if lens:
        if lens.get('camera') not in LENSES:
            ctx.err(29, 'av.lens.camera', f'lens.camera 必须是 {LENSES} 之一，收到 {lens.get("camera")!r}')
        crop = lens.get('crop', 1.0)
        if not _is_num(crop) or crop < 1.0:
            ctx.err(30, 'av.lens.crop', 'lens.crop 必须 ≥ 1.0')
    rec = av.get('recording') or {}
    mode = rec.get('mode') if rec else None
    slowmo = rec.get('slowmo', 1) if rec else 1
    if rec:
        if mode not in RECORDING_MODES:
            ctx.err(31, 'av.recording.mode', f'recording.mode 必须是 {RECORDING_MODES} 之一，收到 {mode!r}')
        dur = rec.get('duration_ms', 0)
        if not _is_num(dur):
            ctx.err(32, 'av.recording.duration_ms', 'duration_ms 必须是整数毫秒')
        elif mode == 'PHOTO' and dur != 0:
            ctx.err(32, 'av.recording.duration_ms', 'PHOTO 的 duration_ms 必须为 0')
        elif mode == 'VIDEO' and not (dur == 0 or 5 <= dur <= 30000):
            ctx.err(32, 'av.recording.duration_ms', 'VIDEO 的 duration_ms 必须在 5–30000 或为 0（无限）')
        if isinstance(slowmo, bool) or slowmo not in (1, 2):
            ctx.err(33, 'av.recording.slowmo', 'slowmo 只能是 1 或 2（4 倍以上暂不支持）')
        fps = rec.get('fps')
        if not _is_num(fps) or fps <= 0:
            ctx.err(33, 'av.recording.fps', 'recording.fps 必填且为正')
        elif base_fps is not None and slowmo in (1, 2) and abs(fps - base_fps * slowmo) > 1e-6:
            ctx.err(40, 'av.recording.fps', f'fps 必须等于基准帧率 × slowmo = {base_fps} × {slowmo} = '
                    f'{base_fps * slowmo}，收到 {fps}')
    focus = av.get('focus') or {}
    if focus:
        if focus.get('mode', 'CONTINUOUS') not in FOCUS_MODES:
            ctx.err('7.3', 'av.focus.mode', f'focus.mode 必须是 {FOCUS_MODES} 之一')
        ftarget = focus.get('target', 'SUBJECT_ANCHOR')
        if ftarget not in FOCUS_TARGETS:
            ctx.err('7.3', 'av.focus.target', f'focus.target 必须是 {FOCUS_TARGETS} 之一')
        if ftarget == 'MANUAL' and (not _is_num(focus.get('distance_m')) or focus['distance_m'] <= 0):
            ctx.err(34, 'av.focus.distance_m', 'focus.target="MANUAL" 时 distance_m 必填且为正')
        if ftarget == 'SUBJECT_ANCHOR' and shot_type == 'static' and 'target' in focus:
            # 与 audio.beamform 同口径：只对显式写出的值报错，缺省值由规则引擎按型填（7.5 的表）
            ctx.err(35, 'av.focus.target', 'A 型没有可跟踪目标，focus.target 不能是 SUBJECT_ANCHOR，请退回 CENTER',
                    suggestion={'target': 'CENTER'})
    exposure = av.get('exposure') or {}
    if exposure:
        emode = exposure.get('mode', 'AUTO')
        if emode not in EXPOSURE_MODES:
            ctx.err('7.2', 'av.exposure.mode', f'exposure.mode 必须是 {EXPOSURE_MODES} 之一')
        if exposure.get('aperture', 'auto') != 'auto':
            ctx.err('7.2.1', 'av.exposure.aperture', '这一版光圈恒为 "auto"，不接受具体 f 值')
        if emode == 'MANUAL' and all(exposure.get(k, 'auto') == 'auto'
                                     for k in ('aperture', 'shutter', 'iso', 'white_balance')):
            ctx.err(38, 'av.exposure', 'exposure.mode="MANUAL" 却四项全 "auto"，等于 AUTO，写 MANUAL 没有意义')
    audio = av.get('audio') or {}
    if audio:
        beam = audio.get('beamform', 'TARGET')
        if beam not in BEAMFORMS:
            ctx.err('7.5', 'av.audio.beamform', f'audio.beamform 必须是 {BEAMFORMS} 之一')
        if beam == 'TARGET' and shot_type == 'static' and 'beamform' in audio:
            ctx.err(35, 'av.audio.beamform', 'A 型没有可跟踪目标，audio.beamform 不能是 TARGET，请退回 OMNI',
                    suggestion={'beamform': 'OMNI'})
        enabled = audio.get('enabled')
        if enabled is True and mode == 'PHOTO':
            ctx.err(37, 'av.audio.enabled', 'recording.mode="PHOTO" 时 audio.enabled 必须为 false')
        if enabled is True and slowmo == 2:
            ctx.err(39, 'av.audio.enabled', 'slowmo=2 时 audio.enabled 必须为 false（音频没法慢放）')


# ─────────────────────────────── 解析 ───────────────────────────────

def parse_shot(obj: Dict[str, Any], lens: str = 'WIDE', aspect: str = '4:3',
               base_fps: Optional[float] = None) -> ShotSpec:
    """@brief 校验并解析成 ShotSpec；有任何错误抛 SpecValidationError（report 带全部错误）。
    @param obj      规范 JSON
    @param lens     镜头（规则 21）
    @param aspect   交付画幅（规则 21）
    @param base_fps 基准帧率（规则 40）
    @return ShotSpec（warnings 字段带告警）
    @throws SpecValidationError
    """
    report = validate_shot(obj, lens, aspect, base_fps)
    if not report.ok:
        raise SpecValidationError(report)
    segments = [Segment(path=s.get('path', 'line'), law=s.get('law', 's_curve'),
                        duration=float(s['duration']) if 'duration' in s else None,
                        speed=float(s['speed']) if 'speed' in s else None,
                        center=_vec3(s.get('center')), axis=_vec3(s.get('axis')))
                for s in obj['segments']]
    known = _A_TOP if obj['type'] == 'static' else _B_TOP
    extra = {k: v for k, v in obj.items() if k not in known}
    if obj['type'] == 'static':
        waypoints = [Waypoint(_vec3(w['position']), _vec3(w['look_at']), float(w.get('roll', 0.0)),
                              float(w.get('hold', 0.0))) for w in obj['waypoints']]
        tol = obj['tolerance']
        return ShotSpec('static', waypoints, segments,
                        Tolerance(float(tol['position']), float(tol['aim']), float(tol['roll'])),
                        av=obj.get('av'), extra=extra, warnings=report.warnings)
    ref = obj['position_ref']
    tw = []
    for w in obj['waypoints']:
        uv = w.get('uv', [0.0, 0.0])
        tw.append(TrackingWaypoint(
            d=float(w['d']) if 'd' in w else None, az=float(w['az']) if 'az' in w else None,
            el=float(w['el']) if 'el' in w else None, position=_vec3(w.get('position')),
            uv=(float(uv[0]), float(uv[1])), roll=float(w.get('roll', 0.0)), hold=float(w.get('hold', 0.0))))
    tol = obj['tolerance']
    ttol = TrackingTolerance(u=float(tol['u']), v=float(tol['v']), roll=float(tol['roll']),
                             d=float(tol['d']) if 'd' in tol else None,
                             az=float(tol['az']) if 'az' in tol else None,
                             el=float(tol['el']) if 'el' in tol else None,
                             position=float(tol['position']) if 'position' in tol else None)
    return ShotSpec('tracking', tw, segments, ttol, target=obj['target'],
                    target_static=bool(obj.get('target_static', False)), position_ref=ref,
                    av=obj.get('av'), extra=extra, warnings=report.warnings)


# ─────────────────────────────── 相机姿态 ───────────────────────────────

def rotation_from_forward(forward: Sequence[float], roll: float = 0.0) -> np.ndarray:
    """@brief 光轴方向 + roll → 光学系旋转（列 = 右 / 下 / 前，世界系）。roll=0 时右向量水平（画面与地平线平齐）；
           光轴竖直（俯视 / 仰视）时地平线不存在，取世界 +X 为画面上方。
    @param forward 光轴方向（不必归一）
    @param roll    绕光轴的右手转角，rad；正 = 从相机背后看顺时针
    @return 3×3
    """
    f = _unit(np.asarray(forward, dtype=float))
    up_ref = np.array([0.0, 0.0, 1.0])
    if np.linalg.norm(np.cross(f, up_ref)) < 1e-6:
        up_ref = np.array([1.0, 0.0, 0.0])
    right0 = _unit(np.cross(f, up_ref))
    down0 = np.cross(f, right0)
    c, s = math.cos(roll), math.sin(roll)
    right = c * right0 + s * down0
    down = -s * right0 + c * down0
    return np.column_stack([right, down, f])


def camera_rotation(position: Sequence[float], look_at: Sequence[float], roll: float = 0.0) -> np.ndarray:
    """@brief 光心位置 + 光轴对准的世界点 + roll → 光学系旋转（列 = 右 / 下 / 前）。
    @param position 光心世界坐标
    @param look_at  对准点世界坐标
    @param roll     画面倾斜，rad
    @return 3×3
    """
    return rotation_from_forward(np.asarray(look_at, dtype=float) - np.asarray(position, dtype=float), roll)


def uv_of(rotation: np.ndarray, position: Sequence[float], target: Sequence[float]) -> Tuple[float, float]:
    """@brief 目标点在该相机位姿下的归一化像坐标 (u, v) = (X/Z, Y/Z)（6.3.6）。
    @param rotation 光学系 3×3（列 = 右 / 下 / 前）
    @param position 光心位置
    @param target   目标点
    @return (u, v)；目标在相机后方（Z ≤ 0）时返回 (inf, inf)
    """
    rel = np.asarray(rotation).T @ (np.asarray(target, dtype=float) - np.asarray(position, dtype=float))
    if rel[2] <= 1e-9:
        return math.inf, math.inf
    return float(rel[0] / rel[2]), float(rel[1] / rel[2])


# ─────────────────────────────── A 型参考轨迹（5.3.7） ───────────────────────────────

@dataclass
class CamRef:
    """@brief 某一时刻的参考位姿：光心位置 / 光学系旋转 / 等效对准点 / roll。"""
    position: np.ndarray
    rotation: np.ndarray
    look_at: np.ndarray
    roll: float


def _require_static(spec: ShotSpec) -> None:
    """@brief 参考轨迹只对 A 型（或已展开的 B 型）有定义。"""
    if spec.type != 'static':
        raise ValueError('参考轨迹只对 A 型定义；B 型先用 tracking_to_static 展开')


def pure_pan(spec: ShotSpec, i: int) -> bool:
    """@brief 第 i 段是否零长度（两端 position 相同：纯摇镜 / 纯俯仰 / 原地改构图）。B 型展开的球坐标段按
           (d, az, el) 三个都不变判定（az 差整圈时位置相同但不是零长度）。
    @param spec A 型
    @param i    段号
    @return bool
    """
    _require_static(spec)
    track = spec.segments[i].track
    if track is not None and track.sph is not None:
        a, b = track.sph[i], track.sph[i + 1]
        return all(abs(x - y) < _ZERO_LEN_EPS for x, y in zip(a, b))
    return bool(np.linalg.norm(spec.waypoints[i + 1].position - spec.waypoints[i].position) < _ZERO_LEN_EPS)


def sph_to_cart(target: Sequence[float], d: float, az: float, el: float) -> np.ndarray:
    """@brief 目标球坐标（6.3.2–6.3.4：视距、方位（世界 +X 起、俯视逆时针为正）、仰角向上为正）→ 世界点。
    @param target 目标位置
    @param d,az,el 球坐标
    @return (3,)
    """
    return np.asarray(target, dtype=float) + d * np.array([math.cos(el) * math.cos(az),
                                                           math.cos(el) * math.sin(az), math.sin(el)])


def _spherical_control(spec: ShotSpec, i: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """@brief 球坐标 spline 的四个控制点（端点外镜像虚点）。"""
    sph = [np.asarray(v, dtype=float) for v in spec.segments[i].track.sph]
    p1, p2 = sph[i], sph[i + 1]
    p0 = sph[i - 1] if i - 1 >= 0 else 2 * p1 - p2
    p3 = sph[i + 2] if i + 2 < len(sph) else 2 * p2 - p1
    return p0, p1, p2, p3


def spherical_param(spec: ShotSpec, i: int, s: float) -> np.ndarray:
    """@brief B 型球坐标段：进度 s 处的 (d, az_world, el)（6.4.3）。
           line：在 (d, az, el) 里线性——与 Commander ORBIT 的球坐标线性插值同一几何，环绕 / 螺旋都能一条原语复现；
           spline：对全部路点的 (d, az, el) 做向心 Catmull-Rom，s 按走出来的弧长归一化（Commander 无对应原语，只能细分）。
    @param spec 展开后的 A 型（段带 track.sph）
    @param i    段号
    @param s    进度
    @return (3,)
    """
    if spec.segments[i].path == 'spline' and len(spec.segments[i].track.sph) > 2:
        ctrl = _spherical_control(spec, i)
        target = spec.segments[i].track.target
        ts = np.linspace(0.0, 1.0, _SPLINE_SAMPLES + 1)
        pts = np.array([sph_to_cart(target, *_catmull_rom(*ctrl, t)) for t in ts])
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        t = float(np.interp(s, cum / cum[-1], ts)) if cum[-1] > 1e-12 else s
        return _catmull_rom(*ctrl, t)
    sph = spec.segments[i].track.sph
    a, b = np.asarray(sph[i], dtype=float), np.asarray(sph[i + 1], dtype=float)
    return a + s * (b - a)


def _arc_params(spec: ShotSpec, i: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """@brief arc 段的 (圆心, 单位轴, 起点相对向量, 有符号扫角∈(0, 2π])。"""
    seg = spec.segments[i]
    a = _unit(seg.axis)
    v0 = spec.waypoints[i].position - seg.center
    v1 = spec.waypoints[i + 1].position - seg.center
    # 投到垂直于轴的平面里量转角（端点已由规则 10 保证近似共面）
    v0p = v0 - np.dot(v0, a) * a
    v1p = v1 - np.dot(v1, a) * a
    ang = math.atan2(float(np.dot(np.cross(v0p, v1p), a)), float(np.dot(v0p, v1p)))
    if ang <= 1e-12:
        ang += 2 * math.pi
    return seg.center, a, v0, ang


def _spline_control(spec: ShotSpec, i: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """@brief 第 i 段的 Catmull-Rom 四个控制点（端点外用镜像虚点补）。"""
    pts = [w.position for w in spec.waypoints]
    p1, p2 = pts[i], pts[i + 1]
    p0 = pts[i - 1] if i - 1 >= 0 else 2 * p1 - p2
    p3 = pts[i + 2] if i + 2 < len(pts) else 2 * p2 - p1
    return p0, p1, p2, p3


def _catmull_rom(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float,
                 alpha: float = 0.5) -> np.ndarray:
    """@brief 向心 Catmull-Rom（alpha=0.5）在 p1→p2 段上参数 t∈[0,1] 的点。"""
    def _knot(a: np.ndarray, b: np.ndarray) -> float:
        return max(float(np.linalg.norm(b - a)) ** alpha, 1e-9)
    t0 = 0.0
    t1 = t0 + _knot(p0, p1)
    t2 = t1 + _knot(p1, p2)
    t3 = t2 + _knot(p2, p3)
    tt = t1 + t * (t2 - t1)

    def _lerp(a: np.ndarray, b: np.ndarray, ta: float, tb: float) -> np.ndarray:
        return (tb - tt) / (tb - ta) * a + (tt - ta) / (tb - ta) * b
    a1 = _lerp(p0, p1, t0, t1)
    a2 = _lerp(p1, p2, t1, t2)
    a3 = _lerp(p2, p3, t2, t3)
    b1 = _lerp(a1, a2, t0, t2)
    b2 = _lerp(a2, a3, t1, t3)
    return _lerp(b1, b2, t1, t2)


_SPLINE_SAMPLES = 128


def _spline_table(spec: ShotSpec, i: int) -> Tuple[np.ndarray, np.ndarray]:
    """@brief 样条段的 (参数 t 表, 归一化累计弧长表)，用于 s → t 反查。"""
    p0, p1, p2, p3 = _spline_control(spec, i)
    ts = np.linspace(0.0, 1.0, _SPLINE_SAMPLES + 1)
    pts = np.array([_catmull_rom(p0, p1, p2, p3, t) for t in ts])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    if total < 1e-12:
        return ts, ts.copy()
    return ts, cum / total


def segment_length(spec: ShotSpec, i: int) -> float:
    """@brief 第 i 段光心路径的弧长（m）；零长度段返回 0。
    @param spec A 型
    @param i    段号
    @return 米
    """
    _require_static(spec)
    if pure_pan(spec, i):
        return 0.0
    path = spec.segments[i].path
    track = spec.segments[i].track
    if track is not None and track.sph is not None:
        pts = np.array([path_position(spec, i, s) for s in np.linspace(0.0, 1.0, _SPLINE_SAMPLES + 1)])
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    p0, p1 = spec.waypoints[i].position, spec.waypoints[i + 1].position
    if path == 'line' or (path == 'spline' and len(spec.waypoints) == 2):
        return float(np.linalg.norm(p1 - p0))
    if path == 'arc':
        _, a, v0, ang = _arc_params(spec, i)
        r = float(np.linalg.norm(v0 - np.dot(v0, a) * a))
        return r * ang
    p0c, p1c, p2c, p3c = _spline_control(spec, i)
    ts = np.linspace(0.0, 1.0, _SPLINE_SAMPLES + 1)
    pts = np.array([_catmull_rom(p0c, p1c, p2c, p3c, t) for t in ts])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def path_position(spec: ShotSpec, i: int, s: float) -> np.ndarray:
    """@brief 第 i 段进度 s∈[0,1]（按弧长归一化）处的光心位置。
    @param spec A 型
    @param i    段号
    @param s    进度
    @return (3,)
    """
    _require_static(spec)
    s = min(1.0, max(0.0, float(s)))
    track = spec.segments[i].track
    if track is not None and track.sph is not None:
        d, az, el = spherical_param(spec, i, s)
        return sph_to_cart(track.target, float(d), float(az), float(el))
    p0, p1 = spec.waypoints[i].position, spec.waypoints[i + 1].position
    path = spec.segments[i].path
    if pure_pan(spec, i) or path == 'line' or (path == 'spline' and len(spec.waypoints) == 2):
        return p0 + s * (p1 - p0)
    if path == 'arc':
        center, a, v0, ang = _arc_params(spec, i)
        return center + _rodrigues(a, s * ang) @ v0
    ts, cum = _spline_table(spec, i)
    t = float(np.interp(s, cum, ts))
    return _catmull_rom(*_spline_control(spec, i), t)


def reference_pose(spec: ShotSpec, i: int, s: float) -> CamRef:
    """@brief 第 i 段进度 s 处的完整参考位姿（5.3.7）：位置按 path；相机移动时 look_at 点线性插值；
           纯摇镜时光轴**方向**在两端方向的大圆上按角度插值（与规则 13 的"扫角 < 180° 才唯一"一致）；
           roll 线性插值。B 型展开的段（segment.track）按 6.4.3：位置在目标球坐标里插值、uv 线性插值，
           光轴每一点都由目标 + uv(s) 决定。
    @param spec A 型
    @param i    段号
    @param s    进度 ∈ [0, 1]
    @return CamRef
    """
    _require_static(spec)
    s = min(1.0, max(0.0, float(s)))
    w0, w1 = spec.waypoints[i], spec.waypoints[i + 1]
    position = path_position(spec, i, s)
    roll = w0.roll + s * (w1.roll - w0.roll)
    track = spec.segments[i].track
    if track is not None:
        uv = (track.uv0[0] + s * (track.uv1[0] - track.uv0[0]), track.uv0[1] + s * (track.uv1[1] - track.uv0[1]))
        dist = float(np.linalg.norm(track.target - position))
        forward = _solve_forward_for_uv(position, track.target, uv, roll)
        return CamRef(position, rotation_from_forward(forward, roll), position + forward * dist, roll)
    if pure_pan(spec, i):
        d0, d1 = w0.look_at - w0.position, w1.look_at - w1.position
        f0, f1 = _unit(d0), _unit(d1)
        ang = _angle_between(f0, f1)
        if ang < 1e-9:
            forward = f0
        else:
            axis = np.cross(f0, f1)
            if np.linalg.norm(axis) < 1e-9:       # 对顶：规则 13 已拒，这里兜底任选一个平面
                axis = np.cross(f0, [0.0, 0.0, 1.0])
                if np.linalg.norm(axis) < 1e-9:
                    axis = np.cross(f0, [1.0, 0.0, 0.0])
            forward = _rodrigues(_unit(axis), s * ang) @ f0
        dist = float(np.linalg.norm(d0) + s * (np.linalg.norm(d1) - np.linalg.norm(d0)))
        look_at = position + forward * dist
        return CamRef(position, rotation_from_forward(forward, roll), look_at, roll)
    look_at = w0.look_at + s * (w1.look_at - w0.look_at)
    return CamRef(position, camera_rotation(position, look_at, roll), look_at, roll)


# ─────────────────────────────── B 型：运行时数据与快照展开 ───────────────────────────────

@dataclass
class TargetData:
    """@brief 6.2.4 运行时数据：一帧目标位姿。"""
    stamp: float
    id: Union[str, int]
    position: np.ndarray
    yaw: Optional[float] = None
    velocity: Optional[np.ndarray] = None
    confidence: Optional[float] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TargetData':
        """@brief 从字典构造（不校验，校验用 validate_target_data）。
        @param data 字典
        @return TargetData
        """
        return cls(stamp=float(data['stamp']), id=data['id'], position=_vec3(data['position']),
                   yaw=float(data['yaw']) if _is_num(data.get('yaw')) else None,
                   velocity=_vec3(data.get('velocity')),
                   confidence=float(data['confidence']) if _is_num(data.get('confidence')) else None)


def validate_target_data(data: Any, position_ref: Optional[str]) -> List[SpecError]:
    """@brief 校验一帧运行时数据是否符合 6.2.4 / 附录 A.3：stamp / id / position 必须；
           position_ref="target_front" 还必须有 yaw（规则 24）。
    @param data         字典
    @param position_ref 分镜的 position_ref
    @return 错误列表（空 = 合法）
    """
    errs: List[SpecError] = []
    if not isinstance(data, dict):
        return [SpecError('A.3', 'target', '运行时数据必须是对象')]
    if not _is_num(data.get('stamp')):
        errs.append(SpecError('A.3', 'target.stamp', '缺 stamp（秒）'))
    if data.get('id') in (None, ''):
        errs.append(SpecError('A.3', 'target.id', '缺 id（目标标识，换目标必须换 id）'))
    if _vec3(data.get('position')) is None:
        errs.append(SpecError('A.3', 'target.position', '缺 position [x, y, z]（世界系，米）'))
    if position_ref == 'target_front' and not _is_num(data.get('yaw')):
        errs.append(SpecError(24, 'target.yaw', 'position_ref="target_front" 需要数据源提供 yaw（目标朝向，rad）；'
                              '拿不到时按附录 C.6 降级成 position_ref="target" 并保持当前方位'))
    return errs


def _solve_forward_for_uv(position: np.ndarray, target: np.ndarray, uv: Tuple[float, float],
                          roll: float) -> np.ndarray:
    """@brief 求光轴方向 f，使目标在该相机位姿下的归一化像坐标正好是 uv：
           (t−p)/|t−p| = (u·右 + v·下 + 前)/√(1+u²+v²)，对 f 做定点迭代（u、v 小，收敛快）。
    @param position 光心
    @param target   目标点
    @param uv       (u, v)
    @param roll     roll
    @return 单位光轴方向
    """
    g = _unit(target - position)
    u, v = uv
    scale = math.sqrt(1.0 + u * u + v * v)
    f = g.copy()
    for _ in range(100):
        rot = rotation_from_forward(f, roll)
        f_new = _unit(g * scale - u * rot[:, 0] - v * rot[:, 1])
        if np.linalg.norm(f_new - f) < 1e-14:
            f = f_new
            break
        f = f_new
    return f


def tracking_to_static(spec: ShotSpec, target: TargetData) -> ShotSpec:
    """@brief 用一帧目标数据把 B 型展开成等价的 A 型（编译期冻结取样，3.4）：
           d/az/el 或 position → 光心位置；目标位置 + uv → look_at；六项容差 → 三项容差。
           段上附 TrackInfo，让 reference_pose 按 6.4.3 插值：target/target_front 的位置在目标球坐标 (d, az, el)
           里走（环绕是绕目标的弧，不是弦），uv 段内线性，每一点的光轴都由目标 + uv(s) 定。
           展开后是开环执行，`uv` 的闭环修正不在其中——这是 Commander 执行链的能力边界。
    @param spec   B 型 ShotSpec
    @param target 目标数据（position_ref="target_front" 时必须有 yaw）
    @return A 型 ShotSpec（extra 透传、source_target_id 记录目标 id）
    @throws ValueError 不是 B 型，或 target_front 缺 yaw
    """
    if not spec.is_tracking:
        raise ValueError('tracking_to_static 只接受 B 型')
    if spec.position_ref == 'target_front' and target.yaw is None:
        raise ValueError('position_ref="target_front" 需要目标 yaw')
    t = np.asarray(target.position, dtype=float)
    az0 = float(target.yaw) if spec.position_ref == 'target_front' else 0.0
    spherical = spec.position_ref != 'world'
    sph: Optional[List[Tuple[float, float, float]]] = [] if spherical else None
    waypoints: List[Waypoint] = []
    for wp in spec.waypoints:
        if spherical:
            sph.append((float(wp.d), az0 + float(wp.az), float(wp.el)))
            pos = sph_to_cart(t, *sph[-1])
        else:
            pos = np.asarray(wp.position, dtype=float)
        dist = float(np.linalg.norm(t - pos))
        if dist < 1e-6:
            raise ValueError('相机位置与目标重合，无法定义光轴')
        forward = _solve_forward_for_uv(pos, t, wp.uv, wp.roll)
        waypoints.append(Waypoint(pos, pos + forward * dist, wp.roll, wp.hold))
    segments = [Segment(path=seg.path, law=seg.law, duration=seg.duration, speed=seg.speed,
                        track=TrackInfo(t, spec.waypoints[i].uv, spec.waypoints[i + 1].uv, sph))
                for i, seg in enumerate(spec.segments)]
    tol = spec.tolerance
    if spec.position_ref == 'world':
        pos_tol = float(tol.position)
    else:
        d_min = min(wp.d for wp in spec.waypoints)
        pos_tol = min(float(tol.d), d_min * float(tol.az), d_min * float(tol.el))
    static = ShotSpec('static', waypoints, segments,
                      Tolerance(pos_tol, min(float(tol.u), float(tol.v)), float(tol.roll)),
                      av=spec.av, extra=dict(spec.extra), warnings=list(spec.warnings),
                      source_target_id=target.id)
    return static
