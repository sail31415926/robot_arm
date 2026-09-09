# -*- coding: utf-8 -*-
"""
@file      llm_shot_loop.py
@brief     大模型单步运镜闭环（"余量盒子"方案的调用层）：
           读关节角 → headroom 余量 → 组提示词（能力卡 + 当前状态 + JSON schema）→ 大模型出**一步** JSON
           → check_plan 校验 / 夹取 → 执行 → 下一步。核心循环不依赖 ROS（可用假臂 / 脚本大模型测试），
           ROS 胶水（ArmApi 读关节角、run_plan_step 执行、可选相机画面）在文件末尾。
@version   0.1
@date      2026-09-09
@copyright Copyright (c) 2026 eMeet

大模型接入方式（都实现 LLMClient.complete(messages, schema) -> str）：
  · OpenAICompatClient  任何 OpenAI 兼容的 /chat/completions（DashScope 兼容模式 / DeepSeek / vLLM / Ollama…），
                        支持 response_format=json_schema 的服务会把余量区间当硬约束；不支持的自动退到 json_object
  · ManualClient        没有模型时由人在终端里当大模型（实机联调用：看提示词、手敲一步 JSON）
  · ScriptedClient      预置回复序列（单元测试）

═══════════════════════════════ 本文件函数 / 类汇总 ═══════════════════════════════
  extract_json(text)                 从大模型输出里抽出 JSON 对象（容忍 ```json 围栏与前后闲话）
  StepRecord                         一步的记录：大模型提议 / 实际执行（可能被夹）/ 是否成功 / 说明
  PromptBuilder                      组 system / user 提示词与单步 JSON schema
  ScriptedClient / ManualClient / OpenAICompatClient   三种 LLMClient
  run_step_loop(model, llm, task, get_joints, execute, …)   核心闭环（无 ROS）
  ros_joint_reader(api) / ros_executor(api, dry_run)       ArmApi → get_joints / execute 回调
  make_image_grabber(node, topic)    订阅相机话题，给大模型附最新一帧（JPEG base64；需 cv2）
  run_llm_shot_loop(api, model, llm, task, …)              ROS 版入口：把上面串起来
"""

import base64
import json
import os
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from .reach_check import (ArmModel, Margins, capability_card, check_plan, current_state_text,
                          headroom, headroom_schema)

# 大模型允许输出的 op（其余一律拒绝并回喂）；done = 任务完成
ALLOWED_OPS = ('dolly', 'truck', 'crane', 'move_rel', 'arc', 'linear', 'pose', 'wait', 'done')


def extract_json(text: str) -> Dict[str, Any]:
    """@brief 从大模型输出里抽出第一个 JSON 对象：去掉 ```json 围栏，取首个 '{' 到与之配对的 '}'。
    @param text 原始输出
    @return 解析出的字典
    @throws ValueError 找不到 / 解析失败 / 不是对象
    """
    body = text.strip()
    if body.startswith('```'):
        body = body.split('\n', 1)[1] if '\n' in body else ''
        body = body.rsplit('```', 1)[0]
    start = body.find('{')
    if start < 0:
        raise ValueError(f'输出里没有 JSON 对象: {text[:80]!r}')
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(body)):
        ch = body[i]
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                obj = json.loads(body[start:i + 1])
                if not isinstance(obj, dict):
                    raise ValueError('JSON 顶层不是对象')
                return obj
    raise ValueError(f'JSON 括号不配对: {text[:80]!r}')


@dataclass
class StepRecord:
    """@brief 闭环里一步的记录。executed=None 表示没有执行（被拒 / 输出非法 / done）。"""
    index: int
    proposed: Dict[str, Any]
    executed: Optional[Dict[str, Any]]
    ok: bool
    clipped: bool = False
    note: str = ''
    result: Any = None
    attempts: int = 1

    def line(self) -> str:
        """@brief 一行人话（历史回喂 / 日志）。
        @return 字符串
        """
        op = self.proposed.get('op', '?')
        why = self.proposed.get('reason', '')
        head = f'第 {self.index} 步 {op}'
        if self.executed is None:
            return f'{head}：未执行（{self.note}）' + (f'，模型理由：{why}' if why else '')
        params = {k: v for k, v in self.executed.items() if k not in ('op', 'reason', 'speed')}
        status = '成功' if self.ok else f'执行失败（{self.note}）'
        clip = '，被夹取到 ' + json.dumps(params, ensure_ascii=False) if self.clipped else ''
        return f'{head} {json.dumps(params, ensure_ascii=False)}：{status}{clip}' + \
            (f'，模型理由：{why}' if why else '')


class PromptBuilder:
    """@brief 组提示词：system = 角色 + 能力卡静态部分 + 输出规则（只发一次）；
           user = 任务 + 当前状态与余量（每步现算）+ 历史 + 上一轮被拒原因；schema = 余量区间 + done。
    """

    def __init__(self, model: ArmModel, subject: Optional[Sequence[float]] = None,
                 margins: Optional[Margins] = None, base_height_m: float = 0.31,
                 history_lines: int = 6):
        """@brief 构造。
        @param model         运动学模型
        @param subject       环绕主体位置（base_link 系），None 不给环绕选项
        @param margins       余量
        @param base_height_m 臂基座离地高
        @param history_lines 回喂最近多少步历史
        """
        self.model = model
        self.subject = [float(v) for v in subject] if subject is not None else None
        self.margins = margins or Margins()
        self.base_height_m = base_height_m
        self.history_lines = history_lines
        self._system: Optional[str] = None

    def system_prompt(self) -> str:
        """@brief system 提示词（缓存：能力卡静态部分只算一次）。
        @return 文本
        """
        if self._system is None:
            card = capability_card(self.model, None, None, self.margins, self.base_height_m)
            self._system = '\n'.join([
                '你是一台机械臂相机机器人的运镜决策器。机械臂末端装着相机，你每次只决定**下一步**动作。',
                '',
                card,
                '',
                '输出规则：',
                '1. 每次只输出一个 JSON 对象，不要任何解释文字；字段 op 取 dolly / truck / crane'
                ' / move_rel / arc / done，可加 reason（一句话理由）。',
                '2. dolly/truck/crane 的 distance_m、move_rel 的 dx/dy/dz/dyaw/dpitch、'
                'arc 的 az_end_deg，**必须落在用户消息给出的当前余量区间内**——'
                '区间外的数会被拒绝并要求重来。',
                '3. 优先小步（≤ 0.10 m / ≤ 20°）、每步看效果；构图达到任务要求就输出 {"op": "done"}。',
                '4. 多轴同时动（move_rel 里几个字段一起给）时各区间的组合可能略超可达域，执行侧会夹取。',
                '5. dolly 正 = 向前推，truck 正 = 向左，crane 正 = 向上；dyaw 正 = 向左转，'
                'dpitch 正 = 低头（俯视）。',
            ])
        return self._system

    def user_prompt(self, task: str, joints: Sequence[float], history: Sequence[StepRecord],
                    feedback: Optional[str], h: Dict[str, Any]) -> str:
        """@brief user 提示词。
        @param task     任务描述（自然语言）
        @param joints   当前关节角
        @param history  已发生的步骤记录
        @param feedback 上一轮被拒 / 非法输出的原因，None 没有
        @param h        本步的 headroom 结果
        @return 文本
        """
        lines = [f'任务：{task}', '', '当前状态：',
                 current_state_text(self.model, joints, self.subject, self.margins,
                                    self.base_height_m, headroom_data=h)]
        recent = list(history)[-self.history_lines:]
        if recent:
            lines += ['', '已执行：'] + [rec.line() for rec in recent]
        if feedback:
            lines += ['', f'上一次输出被拒绝：{feedback}', '请修正后重新只输出一个 JSON。']
        lines += ['', '请输出下一步（一个 JSON 对象）。']
        return '\n'.join(lines)

    def schema(self, h: Dict[str, Any]) -> Dict[str, Any]:
        """@brief 单步 JSON schema：余量区间（headroom_schema）+ done + wait。
        @param h headroom 结果
        @return schema 字典
        """
        sch = headroom_schema(h)
        sch['oneOf'].append({'type': 'object', 'required': ['op'],
                             'properties': {'op': {'const': 'done'}, 'reason': {'type': 'string'}}})
        for item in sch['oneOf']:
            item['properties'].setdefault('reason', {'type': 'string'})
        return sch

    def build(self, task: str, joints: Sequence[float], history: Sequence[StepRecord],
              feedback: Optional[str] = None, image_b64: Optional[str] = None) -> tuple:
        """@brief 组一轮请求。
        @param task      任务
        @param joints    当前关节角
        @param history   历史
        @param feedback  被拒原因
        @param image_b64 相机画面 JPEG base64（None 不附图）
        @return (messages, schema, headroom)
        """
        h = headroom(self.model, joints, subject=self.subject, margins=self.margins)
        text = self.user_prompt(task, joints, history, feedback, h)
        content: Any = text
        if image_b64:
            content = [{'type': 'text', 'text': text},
                       {'type': 'image_url',
                        'image_url': {'url': f'data:image/jpeg;base64,{image_b64}',
                                      'detail': 'low'}}]
        messages = [{'role': 'system', 'content': self.system_prompt()},
                    {'role': 'user', 'content': content}]
        return messages, self.schema(h), h


# ─────────────────────────────── LLM 客户端 ───────────────────────────────


class ScriptedClient:
    """@brief 预置回复的假大模型（测试用）；记录每次收到的 (messages, schema)。"""

    def __init__(self, replies: Sequence[Any]):
        """@brief 构造。
        @param replies 回复序列：dict 会被 json.dumps，str 原样返回
        """
        self._replies = list(replies)
        self.calls: List[tuple] = []

    def complete(self, messages: List[Dict[str, Any]], schema: Dict[str, Any]) -> str:
        """@brief 返回下一条预置回复。
        @param messages 提示词
        @param schema   schema
        @return 文本
        @throws RuntimeError 回复用完
        """
        self.calls.append((messages, schema))
        if not self._replies:
            raise RuntimeError('ScriptedClient 的回复已用完')
        reply = self._replies.pop(0)
        return reply if isinstance(reply, str) else json.dumps(reply, ensure_ascii=False)


class ManualClient:
    """@brief 由人在终端里当大模型：打印 user 提示词，读一行 JSON（回车 / done 视为 {"op":"done"}）。"""

    def __init__(self, show_system: bool = False, stream=None):
        """@brief 构造。
        @param show_system 第一次是否把 system 提示词也打出来
        @param stream      输出流，None 用 stdout
        """
        self.show_system = show_system
        self.stream = stream or sys.stdout
        self._shown = False

    def complete(self, messages: List[Dict[str, Any]], schema: Dict[str, Any]) -> str:
        """@brief 打印提示、读终端输入。
        @param messages 提示词
        @param schema   schema（不打印）
        @return 输入的 JSON 文本
        """
        if self.show_system and not self._shown:
            print('═══ system ═══\n' + messages[0]['content'], file=self.stream)
            self._shown = True
        content = messages[-1]['content']
        if isinstance(content, list):
            content = next(part['text'] for part in content if part.get('type') == 'text')
        print('═══ user ═══\n' + content, file=self.stream)
        print('请输入下一步 JSON（回车或 done 结束）> ', end='', file=self.stream, flush=True)
        line = sys.stdin.readline().strip()
        if not line or line.lower() == 'done':
            return '{"op": "done", "reason": "人工结束"}'
        return line


class OpenAICompatClient:
    """@brief OpenAI 兼容 /chat/completions 客户端（urllib，无额外依赖）。"""

    def __init__(self, base_url: str, api_key: str, model: str, timeout_sec: float = 60.0,
                 temperature: float = 0.2, use_json_schema: bool = True,
                 extra_body: Optional[Dict[str, Any]] = None):
        """@brief 构造。
        @param base_url        形如 https://host/v1（不带 /chat/completions）
        @param api_key         Bearer token（本地服务可随便填）
        @param model           模型名
        @param timeout_sec     HTTP 超时
        @param temperature     采样温度（决策任务用低温）
        @param use_json_schema 优先用 response_format=json_schema；服务不支持时自动退到 json_object
        @param extra_body      原样并进请求体的额外字段，如千问 3.x 关思考 {"enable_thinking": False}
                               （思考模式下非流式调用会被 DashScope 拒绝）
        """
        self.base_url = base_url.rstrip('/')
        self.api_key = api_key
        self.model = model
        self.timeout_sec = timeout_sec
        self.temperature = temperature
        self.use_json_schema = use_json_schema
        self.extra_body = dict(extra_body or {})

    @classmethod
    def from_env(cls, **kwargs) -> 'OpenAICompatClient':
        """@brief 从环境变量构造：LLM_BASE_URL / LLM_MODEL 必填，LLM_API_KEY、LLM_TIMEOUT_SEC、
               LLM_EXTRA_JSON（额外请求字段的 JSON，如 '{"enable_thinking": false}'）可选。
        @param kwargs 透传给构造函数（优先级高于环境变量）
        @return 客户端
        @throws RuntimeError 缺少 LLM_BASE_URL 或 LLM_MODEL
        """
        base_url = os.environ.get('LLM_BASE_URL', '')
        model = os.environ.get('LLM_MODEL', '')
        if not base_url or not model:
            raise RuntimeError('请设置环境变量 LLM_BASE_URL（如 https://host/v1）与 LLM_MODEL，'
                               '可选 LLM_API_KEY / LLM_TIMEOUT_SEC / LLM_EXTRA_JSON')
        if 'timeout_sec' not in kwargs and os.environ.get('LLM_TIMEOUT_SEC'):
            kwargs['timeout_sec'] = float(os.environ['LLM_TIMEOUT_SEC'])
        if 'extra_body' not in kwargs and os.environ.get('LLM_EXTRA_JSON'):
            kwargs['extra_body'] = json.loads(os.environ['LLM_EXTRA_JSON'])
        return cls(base_url, os.environ.get('LLM_API_KEY', 'none'), model, **kwargs)

    def _post(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """@brief POST JSON 并解析响应。
        @param payload 请求体
        @return 响应字典
        """
        req = urllib.request.Request(
            self.base_url + '/chat/completions', data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json', 'Authorization': f'Bearer {self.api_key}'},
            method='POST')
        with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
            return json.loads(resp.read().decode('utf-8'))

    def complete(self, messages: List[Dict[str, Any]], schema: Dict[str, Any]) -> str:
        """@brief 一次对话补全。
        @param messages OpenAI 格式消息
        @param schema   单步 JSON schema
        @return 模型输出文本
        """
        payload: Dict[str, Any] = {'model': self.model, 'messages': messages,
                                   'temperature': self.temperature, **self.extra_body}
        if self.use_json_schema:
            payload['response_format'] = {
                'type': 'json_schema',
                'json_schema': {'name': schema.get('title', 'next_camera_step'), 'schema': schema}}
        try:
            data = self._post(payload)
        except urllib.error.HTTPError as exc:
            if exc.code != 400 or not self.use_json_schema:
                raise
            # 服务不支持 json_schema：退到 json_object，区间约束只靠提示词 + 执行侧校验
            self.use_json_schema = False
            payload['response_format'] = {'type': 'json_object'}
            data = self._post(payload)
        return data['choices'][0]['message']['content']


# ─────────────────────────────── 核心闭环（无 ROS） ───────────────────────────────


def _apply_clip(step: Dict[str, Any], clipped: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """@brief 把 check_plan 夹取后的参数写回步骤。
    @param step    原步骤
    @param clipped StepReport.clipped
    @return 新步骤
    """
    out = dict(step)
    if clipped:
        out.update(clipped)
    return out


def run_step_loop(model: ArmModel, llm: Any, task: str,
                  get_joints: Callable[[], Sequence[float]],
                  execute: Callable[[Dict[str, Any]], Any], *,
                  subject: Optional[Sequence[float]] = None, margins: Optional[Margins] = None,
                  mode: str = 'clip', max_steps: int = 10, max_retries: int = 2,
                  get_image_b64: Optional[Callable[[], Optional[str]]] = None,
                  on_step: Optional[Callable[[StepRecord], None]] = None,
                  log: Callable[[str], None] = print) -> List[StepRecord]:
    """@brief 核心闭环：每步 读关节角 → 余量 → 提示词 → 大模型 → 校验 / 夹取 → 执行，直到 done 或 max_steps。
           大模型输出非法 / 被拒时把原因回喂再要一次，最多 max_retries 次；用尽则结束闭环。
    @param model        运动学模型
    @param llm          LLMClient（complete(messages, schema) -> str）
    @param task         任务描述
    @param get_joints   读当前 6 轴关节角的回调
    @param execute      执行一步的回调（返回值 bool() 为成败）
    @param subject      环绕主体（可选）
    @param margins      余量
    @param mode         check_plan 模式：'clip' 夹取 / 'reject' 拒绝
    @param max_steps    最多执行多少步
    @param max_retries  同一步最多回喂重试次数
    @param get_image_b64 取相机画面 JPEG base64 的回调（可选）
    @param on_step      每条记录产生时的回调（可选）
    @param log          日志函数
    @return 所有 StepRecord（含 done / 失败记录）
    """
    builder = PromptBuilder(model, subject=subject, margins=margins)
    history: List[StepRecord] = []
    for index in range(1, max_steps + 1):
        joints = list(get_joints())
        feedback: Optional[str] = None
        record: Optional[StepRecord] = None
        for attempt in range(1, max_retries + 2):
            image = get_image_b64() if get_image_b64 else None
            messages, schema, _ = builder.build(task, joints, history, feedback, image)
            raw = llm.complete(messages, schema)
            try:
                step = extract_json(raw)
            except ValueError as exc:
                feedback = f'不是合法 JSON（{exc}）'
                log(f'[{index}.{attempt}] 输出非法：{feedback}')
                continue
            op = str(step.get('op', '')).lower()
            if op == 'done':
                record = StepRecord(index, step, None, True, note='任务完成', attempts=attempt)
                break
            if op not in ALLOWED_OPS:
                feedback = f'op "{op}" 不允许，只能是 {", ".join(ALLOWED_OPS)}'
                log(f'[{index}.{attempt}] {feedback}')
                continue
            report = check_plan(model, [step], joints, margins, mode)
            if not report.ok:
                feedback = report.summary()
                log(f'[{index}.{attempt}] 校验不通过：\n{feedback}')
                continue
            srep = report.steps[0]
            to_run = _apply_clip(step, srep.clipped)
            log(f'[{index}] 执行 {json.dumps(to_run, ensure_ascii=False)}'
                + (f'（{srep.reason}）' if srep.clipped else ''))
            result = execute(to_run)
            ok = bool(result)
            note = srep.reason if srep.clipped else ('' if ok else str(result))
            record = StepRecord(index, step, to_run, ok, clipped=bool(srep.clipped), note=note,
                                result=result, attempts=attempt)
            break
        if record is None:
            record = StepRecord(index, {'op': '?'}, None, False,
                                note=f'重试 {max_retries} 次仍不可用：{feedback}', attempts=max_retries + 1)
            history.append(record)
            if on_step:
                on_step(record)
            log(f'[{index}] {record.note}，闭环结束')
            break
        history.append(record)
        if on_step:
            on_step(record)
        if record.executed is None:
            log(f'[{index}] 大模型判定任务完成：{record.proposed.get("reason", "")}')
            break
        if not record.ok:
            log(f'[{index}] 执行失败：{record.note}，闭环结束')
            break
    return history


# ─────────────────────────────── ROS 胶水 ───────────────────────────────


def ros_joint_reader(api: Any, timeout_sec: float = 2.0) -> Callable[[], List[float]]:
    """@brief 用 ArmApi 读 /joint_states 的回调工厂（Joint1..6 顺序）。
    @param api         ArmApi
    @param timeout_sec 等首帧的最长时间
    @return get_joints 回调
    @throws RuntimeError 超时没收到 6 轴
    """
    from .arm_commander_client import ARM_JOINT_NAMES

    def _get() -> List[float]:
        """@brief 读一次关节角。"""
        deadline = time.monotonic() + timeout_sec
        while True:
            joints = api.arm.get_joints()
            if all(name in joints for name in ARM_JOINT_NAMES):
                return [float(joints[name]) for name in ARM_JOINT_NAMES]
            if time.monotonic() > deadline:
                raise RuntimeError('/joint_states 里没有 Joint1..6，机械臂栈没起来？')
            time.sleep(0.05)
    return _get


def ros_executor(api: Any, dry_run: bool = False,
                 settle_sec: float = 0.3) -> Callable[[Dict[str, Any]], Any]:
    """@brief 用 run_plan_step 执行一步的回调工厂。
    @param api        ArmApi
    @param dry_run    True 只打印不下发（ArmApi 自身的 dry_run 也会生效）
    @param settle_sec 到位后再等多久让 /joint_states 更新
    @return execute 回调（返回 CallResult）
    """
    from .arm_commander_client import CallResult, run_plan_step

    def _exec(step: Dict[str, Any]) -> Any:
        """@brief 执行一步。"""
        if dry_run:
            api.node.get_logger().info(f'[dry-run] {json.dumps(step, ensure_ascii=False)}')
            return CallResult(True, 'dry-run')
        result = run_plan_step(api, step)
        time.sleep(settle_sec)
        return result
    return _exec


def make_image_grabber(node: Any, topic: str,
                       jpeg_quality: int = 70) -> Callable[[], Optional[str]]:
    """@brief 订阅 sensor_msgs/Image，返回"取最新一帧 JPEG base64"的回调（需要 cv2；没有则恒返回 None）。
    @param node         rclpy 节点（ArmApi.node，后台已在 spin）
    @param topic        图像话题
    @param jpeg_quality JPEG 质量
    @return 回调
    """
    try:
        import cv2  # noqa: WPS433
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:  # pragma: no cover - 依赖缺失时退化
        node.get_logger().warning(f'附图不可用（{exc!r}），只发文字')
        return lambda: None
    latest: Dict[str, Any] = {}

    def _cb(msg: Any) -> None:
        """@brief 缓存最新帧。"""
        latest['msg'] = msg
    node.create_subscription(Image, topic, _cb, qos_profile_sensor_data)

    def _grab() -> Optional[str]:
        """@brief 最新帧 → JPEG base64。"""
        msg = latest.get('msg')
        if msg is None:
            return None
        channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
        if channels is None:
            return None
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, :msg.width * channels]
        if channels > 1:
            arr = arr.reshape(msg.height, msg.width, channels)
        if msg.encoding == 'rgb8':
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', arr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        return base64.b64encode(buf.tobytes()).decode('ascii') if ok else None
    return _grab


def run_llm_shot_loop(api: Any, model: ArmModel, llm: Any, task: str, *,
                      subject: Optional[Sequence[float]] = None, image_topic: Optional[str] = None,
                      dry_run: bool = False, joints_override: Optional[Sequence[float]] = None,
                      **kwargs) -> List[StepRecord]:
    """@brief ROS 版入口：ArmApi 提供关节角与执行，其余交给 run_step_loop。
    @param api             ArmApi
    @param model           运动学模型
    @param llm             LLMClient
    @param task            任务描述
    @param subject         环绕主体（可选）
    @param image_topic     相机话题（可选，给支持图像的模型附图）
    @param dry_run         只打印不下发
    @param joints_override 没有 /joint_states 时（离线演示）用这组关节角起步，且每步按推演结果更新
    @param kwargs          透传 run_step_loop（max_steps / max_retries / mode / on_step …）
    @return StepRecord 列表
    """
    log = api.node.get_logger().info
    if joints_override is not None:
        state = {'q': [float(v) for v in joints_override]}

        def get_joints() -> List[float]:
            """@brief 离线：用推演结果代替实测。"""
            return state['q']

        real_exec = ros_executor(api, dry_run)

        def execute(step: Dict[str, Any]) -> Any:
            """@brief 离线：执行（或 dry-run）后把关节角推演到终点。"""
            result = real_exec(step)
            rep = check_plan(model, [step], state['q'], mode='reject')
            if rep.ok:
                state['q'] = [float(v) for v in rep.joints_end]
            return result
    else:
        get_joints = ros_joint_reader(api)
        execute = ros_executor(api, dry_run)
    grabber = make_image_grabber(api.node, image_topic) if image_topic else None
    return run_step_loop(model, llm, task, get_joints, execute, subject=subject,
                         get_image_b64=grabber, log=log, **kwargs)
