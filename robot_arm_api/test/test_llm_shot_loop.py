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


# ═══════════════════════════════ 面向用户的文字反馈 ═══════════════════════════════

def _rec(proposed, executed, ok=True, clipped=False, adjust_kind=None, note=''):
    """@brief 造一条 StepRecord。"""
    return loop.StepRecord(index=1, proposed=proposed, executed=executed, ok=ok, clipped=clipped,
                           adjust_kind=adjust_kind, note=note)


def test_user_line_uses_plain_words_not_jargon():
    """@brief 面向用户的一行：说"向前推近 5 厘米"，不出现 op 名、关节名、弧度这些术语。"""
    text = _rec({'op': 'dolly', 'distance_m': 0.05}, {'op': 'dolly', 'distance_m': 0.05}).user_line()
    assert '向前' in text and '5' in text and '厘米' in text
    for jargon in ('dolly', 'J3', 'rad', 'op'):
        assert jargon not in text, text
    back = _rec({'op': 'dolly', 'distance_m': -0.08}, {'op': 'dolly', 'distance_m': -0.08}).user_line()
    assert '后' in back and '8' in back
    up = _rec({'op': 'crane', 'distance_m': 0.05}, {'op': 'crane', 'distance_m': 0.05}).user_line()
    assert '升' in up or '抬' in up


def test_user_line_reports_requested_and_actual_when_clipped():
    """@brief 降级执行要把"你要多少"和"实际做了多少"都说出来，并说明是行程到头了。"""
    text = _rec({'op': 'truck', 'distance_m': 0.60}, {'op': 'truck', 'distance_m': 0.225},
                clipped=True, adjust_kind='clip', note='夹取到 38%').user_line()
    assert '60' in text and '22' in text and ('行程' in text or '最多' in text)
    assert '向左' in text


def test_user_line_reports_projection():
    """@brief 投影降级要说明"改到最近的可行位置"。"""
    text = _rec({'op': 'pose', 'x': 0.9, 'y': 0.0, 'z': 0.6},
                {'op': 'pose', 'x': 0.55, 'y': 0.0, 'z': 0.6},
                clipped=True, adjust_kind='project', note='已投影到最近可行位姿').user_line()
    assert '最近' in text and ('可行' in text or '能到' in text)


def test_user_line_reports_failure_and_done():
    """@brief 失败说明没执行；done 说任务完成。"""
    bad = _rec({'op': 'dolly', 'distance_m': 0.3}, None, ok=False, note='机械臂伸不了那么远').user_line()
    assert '没' in bad or '未' in bad
    assert '机械臂伸不了那么远' in bad
    done = _rec({'op': 'done', 'reason': '构图完成'}, None, note='任务完成').user_line()
    assert '完成' in done


def test_format_user_report_lists_steps_and_summary(model):
    """@brief 整段报告：每步一行 + 末尾总结做了几步、其中几步被降级。"""
    records = [_rec({'op': 'dolly', 'distance_m': 0.05}, {'op': 'dolly', 'distance_m': 0.05}),
               _rec({'op': 'truck', 'distance_m': 0.6}, {'op': 'truck', 'distance_m': 0.22},
                    clipped=True, adjust_kind='clip'),
               _rec({'op': 'done'}, None, note='任务完成')]
    text = loop.format_user_report(records)
    assert text.count('\n') >= 2
    assert '2' in text and '降级' in text


def test_step_loop_project_mode_never_asks_llm_again(model):
    """@brief 方案 2 的核心：project 模式下超范围的动作直接降级执行，不回喂大模型重来。
    大模型给了一个远超行程的 truck 和一个超出可达域的 pose，两步都执行了，且大模型只被调用 3 次
    （两步 + done），没有任何重试。"""
    arm = FakeArm(model, ARM_NOMINAL)
    llm = loop.ScriptedClient([{'op': 'truck', 'distance_m': 1.0, 'reason': '大幅左移'},
                               {'op': 'pose', 'x': 0.9, 'y': 0.0, 'z': 0.6, 'reason': '冲过去'},
                               {'op': 'done'}])
    seen = []
    records = loop.run_step_loop(model, llm, '任务', arm.get_joints, arm.execute,
                                 mode='project', max_steps=5, user_log=seen.append)
    assert [s['op'] for s in arm.executed] == ['truck', 'pose']
    assert len(llm.calls) == 3
    assert all(r.attempts == 1 for r in records)
    assert records[0].adjust_kind == 'clip' and records[1].adjust_kind == 'project'
    assert len(seen) == 3 and '22' in seen[0] or '厘米' in seen[0]


def test_prompt_builder_can_embed_region_formula(model):
    """@brief 方案 1 的接入点：给 PromptBuilder 一个 RegionFit，system 提示词里就带上可达区公式与自检要求，
    大模型输出绝对位置前可以自己代入验算。不给则不带。"""
    from robot_arm_api import reach_fit as rf
    fit = rf.fit_reach_region(model)
    plain = loop.PromptBuilder(model).system_prompt()
    assert 'h_max(r)' not in plain
    with_fit = loop.PromptBuilder(model, region_fit=fit).system_prompt()
    assert 'h_max(r)' in with_fit and 'h_min(r)' in with_fit
    assert '自检' in with_fit or '代入' in with_fit


# ═══════════════════════════════ 反馈话题 ═══════════════════════════════

class FakeNode:
    """@brief 假 rclpy 节点：记录 create_publisher 的参数，publisher 记录发出的消息。"""

    class Pub:
        """@brief 假 publisher。"""

        def __init__(self):
            self.sent = []

        def publish(self, msg):
            """@brief 记录一条消息。"""
            self.sent.append(msg)

    def __init__(self):
        self.created = []
        self.pub = FakeNode.Pub()
        self.warnings = []

    def create_publisher(self, msg_type, topic, qos):
        """@brief 记录并返回假 publisher。"""
        self.created.append((msg_type, topic, qos))
        return self.pub

    def get_logger(self):
        """@brief 假 logger。"""
        node = self

        class _Log:
            def info(self, text):
                """@brief 忽略。"""

            def warning(self, text):
                """@brief 记录警告。"""
                node.warnings.append(text)
        return _Log()


def test_make_feedback_publisher_publishes_user_text():
    """@brief 反馈话题：默认发 std_msgs/String 到 /robot_arm/llm_feedback，内容就是给用户的那句话。"""
    pytest.importorskip('std_msgs')
    node = FakeNode()
    emit = loop.make_feedback_publisher(node)
    assert node.created and node.created[0][1] == loop.FEEDBACK_TOPIC
    assert node.created[0][0].__name__ == 'String'
    emit('向左平移 22 厘米')
    assert [m.data for m in node.pub.sent] == ['向左平移 22 厘米']


def test_make_feedback_publisher_accepts_custom_topic_and_none():
    """@brief 可换话题名；topic=None 表示不发话题（返回 None，不建 publisher）。"""
    pytest.importorskip('std_msgs')
    node = FakeNode()
    loop.make_feedback_publisher(node, '/my/topic')
    assert node.created[0][1] == '/my/topic'
    node2 = FakeNode()
    assert loop.make_feedback_publisher(node2, None) is None
    assert node2.created == []


def test_user_log_fan_out_calls_every_sink():
    """@brief 多个反馈出口（日志 + 话题 + 上层回调）用 fan_out 串起来，一次调用全部收到。"""
    got_a, got_b = [], []
    sink = loop.fan_out(got_a.append, None, got_b.append)
    sink('测试')
    assert got_a == ['测试'] and got_b == ['测试']
