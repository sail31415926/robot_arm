# -*- coding: utf-8 -*-
"""
@file  test_llm_shot_loop.py
@brief 大模型单步运镜闭环（llm_shot_loop）测试：提示词 / schema 组装、JSON 抽取、
       "读关节角 → 余量 → 大模型出单步 → 校验夹取 → 执行" 的闭环行为（用脚本化的假大模型与假机械臂，不依赖 ROS）。
"""

import json

import numpy as np
import pytest

from robot_arm_api import llm_shot_loop as loop
from robot_arm_api import reach_check as rc

ARM_NOMINAL = np.array([0.0, 1.0, -1.5, 0.0, 0.3, 0.0])


@pytest.fixture(scope='session')
def model(urdf_xml):
    """@brief 全测试共用的 ArmModel。"""
    return rc.ArmModel.from_urdf_string(urdf_xml)


class FakeArm:
    """@brief 假机械臂：执行一步就把关节角更新到该步推演出的终点（模拟臂真的到位了）。"""

    def __init__(self, model, q0):
        self.model = model
        self.q = np.asarray(q0, dtype=float)
        self.executed = []

    def get_joints(self):
        """@brief 当前关节角。"""
        return list(self.q)

    def execute(self, step):
        """@brief 记录并"执行"一步。"""
        self.executed.append(dict(step))
        rep = rc.check_plan(self.model, [step], self.q, mode='reject')
        if rep.ok:
            self.q = np.asarray(rep.joints_end, dtype=float)
        return rep.ok


# ═══════════════════════════════ 工具函数 ═══════════════════════════════

def test_extract_json_strips_fences_and_prose():
    """@brief 大模型常把 JSON 包在 ```json 里、前后带一句话：都要能抽出来。"""
    raw = '好的，下一步：\n```json\n{"op": "dolly", "distance_m": 0.05, "reason": "推近"}\n```\n'
    assert loop.extract_json(raw) == {'op': 'dolly', 'distance_m': 0.05, 'reason': '推近'}
    assert loop.extract_json('{"op":"done"}') == {'op': 'done'}
    with pytest.raises(ValueError):
        loop.extract_json('这不是 JSON')


def test_prompt_builder_messages_and_schema(model):
    """@brief system 里有能力卡静态部分；user 里有任务、当前状态与余量；schema 的 oneOf 含 done 与带 min/max 的 dolly。"""
    builder = loop.PromptBuilder(model)
    messages, schema, h = builder.build('拍摄前方的花瓶，慢慢推近', ARM_NOMINAL, history=[])
    assert messages[0]['role'] == 'system' and '相机位姿约束' in messages[0]['content']
    user = messages[1]['content']
    assert '拍摄前方的花瓶' in user and '当前状态' in user and 'dolly' in user
    ops = {item['properties']['op']['const'] for item in schema['oneOf']}
    assert {'done', 'dolly', 'truck', 'crane', 'move_rel'} <= ops
    dolly = next(i for i in schema['oneOf'] if i['properties']['op']['const'] == 'dolly')
    assert dolly['properties']['distance_m']['maximum'] == pytest.approx(h['dolly'][1])


def test_prompt_builder_includes_feedback_and_history(model):
    """@brief 上一轮被拒的原因、已执行的历史都要出现在 user 提示里，大模型才知道改什么。"""
    builder = loop.PromptBuilder(model)
    history = [loop.StepRecord(index=1, proposed={'op': 'dolly', 'distance_m': 0.05},
                               executed={'op': 'dolly', 'distance_m': 0.05}, ok=True,
                               clipped=False, note='✓')]
    messages, _, _ = builder.build('任务', ARM_NOMINAL, history=history, feedback='pose 超出臂长')
    user = messages[1]['content']
    assert 'pose 超出臂长' in user and 'dolly' in user and '0.05' in user


# ═══════════════════════════════ 闭环 ═══════════════════════════════

def test_step_loop_executes_clips_and_stops_on_done(model):
    """@brief 脚本大模型：dolly 0.05 → truck 0.6（超余量，clip 模式夹取）→ done。
    执行 2 步、第二步被夹（执行距离 < 0.6）、done 后结束。"""
    arm = FakeArm(model, ARM_NOMINAL)
    llm = loop.ScriptedClient([{'op': 'dolly', 'distance_m': 0.05, 'reason': '推近'},
                               {'op': 'truck', 'distance_m': 0.6, 'reason': '大幅左移'},
                               {'op': 'done', 'reason': '构图完成'}])
    records = loop.run_step_loop(model, llm, '任务', arm.get_joints, arm.execute, max_steps=5)
    assert [s['op'] for s in arm.executed] == ['dolly', 'truck']
    assert arm.executed[1]['distance_m'] < 0.6
    assert records[-1].proposed['op'] == 'done'
    assert records[1].clipped and records[0].ok and not records[0].clipped
    assert len(llm.calls) == 3


def test_step_loop_feeds_back_rejection_and_retries(model):
    """@brief 整点类 pose 到 0.9 m 外被拒（不可夹取）：不执行，把原因回喂，大模型第二次给 dolly 0.02 才执行。"""
    arm = FakeArm(model, ARM_NOMINAL)
    llm = loop.ScriptedClient([{'op': 'pose', 'x': 0.9, 'y': 0.0, 'z': 0.6, 'reason': '冲过去'},
                               {'op': 'dolly', 'distance_m': 0.02, 'reason': '小步'},
                               {'op': 'done'}])
    records = loop.run_step_loop(model, llm, '任务', arm.get_joints, arm.execute, max_steps=5)
    assert [s['op'] for s in arm.executed] == ['dolly']
    second_user = llm.calls[1][0][-1]['content']
    assert '臂长' in second_user or '不可达' in second_user
    assert records[0].ok and records[0].executed['op'] == 'dolly'


def test_step_loop_gives_up_after_repeated_bad_output(model):
    """@brief 连续 3 次输出非法 JSON（max_retries=2）：不执行任何动作，闭环结束并留下失败记录。"""
    arm = FakeArm(model, ARM_NOMINAL)
    llm = loop.ScriptedClient(['不是 JSON', '还不是', '依旧不是'])
    records = loop.run_step_loop(model, llm, '任务', arm.get_joints, arm.execute,
                                 max_steps=3, max_retries=2)
    assert arm.executed == []
    assert len(records) == 1 and not records[0].ok and records[0].executed is None


def test_step_loop_respects_max_steps(model):
    """@brief 大模型一直不说 done：到 max_steps 就停。"""
    arm = FakeArm(model, ARM_NOMINAL)
    llm = loop.ScriptedClient([{'op': 'crane', 'distance_m': -0.01}] * 10)
    records = loop.run_step_loop(model, llm, '任务', arm.get_joints, arm.execute, max_steps=3)
    assert len(arm.executed) == 3 and len(records) == 3


# ═══════════════════════════════ OpenAI 兼容客户端 ═══════════════════════════════

def test_openai_compat_client_payload_and_parse(monkeypatch):
    """@brief 请求体：model / messages / response_format(json_schema)；响应取 choices[0].message.content。"""
    captured = {}

    class FakeResp:
        """@brief 假 HTTP 响应。"""

        def __init__(self, body):
            self._body = body

        def read(self):
            """@brief 响应体。"""
            return self._body

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

    def fake_urlopen(req, timeout=None):
        """@brief 记录请求并返回固定响应。"""
        captured['url'] = req.full_url
        captured['headers'] = dict(req.header_items())
        captured['body'] = json.loads(req.data.decode('utf-8'))
        return FakeResp(json.dumps({'choices': [{'message': {'content': '{"op": "done"}'}}]}).encode())

    monkeypatch.setattr(loop.urllib.request, 'urlopen', fake_urlopen)
    client = loop.OpenAICompatClient(base_url='http://llm.local/v1', api_key='k', model='m')
    schema = {'oneOf': [{'type': 'object', 'properties': {'op': {'const': 'done'}}}]}
    text = client.complete([{'role': 'user', 'content': 'hi'}], schema)
    assert text == '{"op": "done"}'
    assert captured['url'] == 'http://llm.local/v1/chat/completions'
    assert captured['headers'].get('Authorization') == 'Bearer k'
    assert captured['body']['model'] == 'm'
    assert captured['body']['response_format']['type'] == 'json_schema'
    assert captured['body']['messages'][0]['content'] == 'hi'


def test_openai_compat_client_extra_body_and_env(monkeypatch):
    """@brief 千问 3.x 这类模型要关思考模式（enable_thinking=false）等额外字段：extra_body 原样并进请求体；
    from_env 读 LLM_BASE_URL / LLM_MODEL / LLM_API_KEY / LLM_EXTRA_JSON。"""
    captured = {}

    class FakeResp:
        """@brief 假 HTTP 响应。"""

        def read(self):
            """@brief 响应体。"""
            return json.dumps({'choices': [{'message': {'content': '{"op":"done"}'}}]}).encode()

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

    def fake_urlopen(req, timeout=None):
        """@brief 记录请求体。"""
        captured['body'] = json.loads(req.data.decode('utf-8'))
        captured['timeout'] = timeout
        return FakeResp()

    monkeypatch.setattr(loop.urllib.request, 'urlopen', fake_urlopen)
    monkeypatch.setenv('LLM_BASE_URL', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
    monkeypatch.setenv('LLM_MODEL', 'qwen3.6-flash')
    monkeypatch.setenv('LLM_API_KEY', 'sk-test')
    monkeypatch.setenv('LLM_EXTRA_JSON', '{"enable_thinking": false}')
    monkeypatch.setenv('LLM_TIMEOUT_SEC', '45')
    client = loop.OpenAICompatClient.from_env()
    assert client.model == 'qwen3.6-flash' and client.extra_body == {'enable_thinking': False}
    client.complete([{'role': 'user', 'content': 'hi'}], {'oneOf': []})
    assert captured['body']['enable_thinking'] is False
    assert captured['body']['model'] == 'qwen3.6-flash'
    assert captured['timeout'] == 45.0
