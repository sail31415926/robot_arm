#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file      arm_commander_demo.py
@brief     机械臂对外接口调用 Demo（命令行）：把 arm_commander_client 的每个接口映射成子命令，
           并提供 JSON 运镜步骤表执行、小幅度全流程演示与 --dry-run 只打印等效 ros2 指令。
@version   1.1
@date      2026-09-08
@copyright Copyright (c) 2026 eMeet

库在同包 arm_commander_client.py（ArmCommanderClient / GimbalV2Client / ArmApi / execute_plan），
本文件只做命令行壳：解析参数 → 组装一条步骤字典 → run_plan_step，所以 CLI 子命令与
JSON 步骤表的 op / 参数名完全一致。

运行前置（实机，臂侧 Jetson）：
  ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false   # 臂 + Commander
  云台板：ros2 launch robot_gimbal_bringup_v2 gimbal_only.launch.py                 # 云台直连接口才需要
  两台机器同 ROS_DOMAIN_ID，且都不能设 ROS_LOCALHOST_ONLY=1。
开发机无硬件（mock 臂）：
  ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false arm_sim_mode:=true

命令行示例（任何子命令加 --dry-run 只打印等效 ros2 指令、不真正下发）：
  ros2 run robot_arm_api arm_commander_demo.py status
  ros2 run robot_arm_api arm_commander_demo.py enable
  ros2 run robot_arm_api arm_commander_demo.py observe
  ros2 run robot_arm_api arm_commander_demo.py pose 0.25 0.0 0.65 0 0 0 --speed normal
  ros2 run robot_arm_api arm_commander_demo.py dolly 0.10            # 推镜 10cm（base_link +x）
  ros2 run robot_arm_api arm_commander_demo.py orbit 0.6 0.0 0.7 -30 30 --radius 0.4
  ros2 run robot_arm_api arm_commander_demo.py gimbal-rotate 20 0 -10      # 云台 pan/roll/tilt（度）
  ros2 run robot_arm_api arm_commander_demo.py plan shot_plan.json         # 执行 JSON 运镜步骤表
  ros2 run robot_arm_api arm_commander_demo.py demo                        # 小幅度全流程演示
离线工具（不下发、不需要 Commander 在跑；关节角缺省从 /joint_states 读，读不到可 --joints 给）：
  ros2 run robot_arm_api arm_commander_demo.py check shot_plan.json [--clip]   # 步骤表可达性预检
  ros2 run robot_arm_api arm_commander_demo.py headroom [--subject 0.8 0 0.7] [--schema]  # 余量盒子
  ros2 run robot_arm_api arm_commander_demo.py card [--joints j1 … j6]         # 大模型能力卡
  ros2 run robot_arm_api arm_commander_demo.py fit [--json]                    # 可达区拟合公式
大模型单步闭环（每步：读关节角 → 余量 → 大模型出一步 JSON → 校验/夹取 → 执行；
--llm manual 由人在终端当模型）：
  ros2 run robot_arm_api arm_commander_demo.py llm-step "把花瓶推成特写" --llm manual --max-steps 5
  ros2 run robot_arm_api arm_commander_demo.py llm-step "任务" --mode project --formula
      # project = 超范围的动作直接降级执行（夹到边界 / 投影到最近可行位姿），不回喂大模型重来
      # 每步给用户的口语反馈同时发到 /robot_arm/llm_feedback（std_msgs/String），--feedback-topic none 可关
  LLM_BASE_URL=https://host/v1 LLM_MODEL=qwen-vl LLM_API_KEY=… \
  ros2 run robot_arm_api arm_commander_demo.py llm-step "任务" --llm openai \
      --image-topic /camera/image_raw

═══════════════════════════════ 本文件函数汇总 ═══════════════════════════════
  print_status(api)                打印机械臂 / 云台当前状态（等 1s 收状态）
  run_demo_sequence(api)           小幅度全流程演示：使能 → 观察位 → 推/横移/升 5cm 并返回 → 云台点头 → 收纳
  build_arg_parser()               构造 argparse：每个子命令的参数名与 run_plan_step 的 op 参数一致
  live_joints(timeout_sec)         起一个临时节点从 /joint_states 读 6 轴关节角（离线工具用）
  load_model(args)                 按 --urdf 或安装的描述包建 ArmModel
  run_offline(args)                check / headroom / card / fit 四个离线子命令（不下发）
  run_llm_step(api, args)          llm-step：大模型单步运镜闭环（llm_shot_loop）
  main(argv)                       入口：解析参数 → ArmApi → 执行子命令 → 收尾（Ctrl-C 取消 goal 并急停）
"""

import argparse
import json
import math
import sys
import time
from typing import Any, Dict, List, Optional, Sequence

import rclpy

from robot_arm_api.arm_commander_client import (ARM_JOINT_NAMES, DEFAULT_HOMING_TIMEOUT_SEC,
                                                GIMBAL_TOPIC_STATUS, ArmApi, execute_plan,
                                                run_plan_step)
from robot_arm_api.llm_shot_loop import (FEEDBACK_TOPIC, ManualClient, OpenAICompatClient,
                                         format_user_report, run_llm_shot_loop)
from robot_arm_api.reach_check import (ArmModel, capability_card, check_plan, headroom,
                                       headroom_schema)
from robot_arm_api.reach_fit import fit_reach_region

OFFLINE_CMDS = ('check', 'headroom', 'card', 'fit')   # 不建 ArmApi、不下发的子命令


def print_status(api: ArmApi) -> None:
    """@brief 打印机械臂 / 云台的当前状态（等 1s 收状态）。
    @param api ArmApi
    """
    log = api.node.get_logger()
    time.sleep(1.0)
    log.info('[机械臂] ' + api.arm.status_summary())
    if api.gimbal is not None:
        status = api.gimbal.get_status()
        if status is None:
            log.info(f'[云台] 未收到 {GIMBAL_TOPIC_STATUS}（板端节点未启动或跨机 DDS 不通）')
        else:
            pan, roll, tilt = api.gimbal.get_angles_deg()
            log.info(f'[云台] pan={pan:.1f} roll={roll:.1f} tilt={tilt:.1f} deg '
                     f'at_target={status.at_target} gbc_stat={status.gbc_stat} '
                     f'hw_fault={status.has_hw_fault} tca_ready={status.tca_ready}')


def run_demo_sequence(api: ArmApi) -> bool:
    """@brief 小幅度全流程演示：状态 → 使能 → 观察位 → 推/横移/升各 5cm 并返回 → 云台点头 → 收纳。
    @param api ArmApi
    @return 全部成功为 True
    """
    log = api.node.get_logger()
    if not api.arm.wait_ready():
        return False
    print_status(api)
    if api.arm.has_error():
        log.warning('有未清除的错误，先 reset_error')
        if not api.arm.reset_error():
            return False
    steps: List[Dict[str, Any]] = [
        {'op': 'enable'},
        {'op': 'observe', 'speed': 'normal'},
        {'op': 'wait', 'seconds': 1.0},
        {'op': 'dolly', 'distance_m': 0.05, 'speed': 'slow', 'return_to_start': True},
        {'op': 'truck', 'distance_m': 0.05, 'speed': 'slow', 'return_to_start': True},
        {'op': 'crane', 'distance_m': 0.05, 'speed': 'slow', 'return_to_start': True},
    ]
    if api.gimbal is not None and api.gimbal.wait_ready(timeout_sec=2.0):
        steps += [{'op': 'gimbal_rotate', 'tilt': -10.0}, {'op': 'gimbal_rotate', 'tilt': 0.0}]
    steps.append({'op': 'stow', 'speed': 'normal'})
    log.info('3 秒后开始演示，机械臂将小幅运动，请确认周围无人无障碍……')
    if not api.arm.dry_run:
        time.sleep(3.0)
    results = execute_plan(api, steps)
    return len(results) == len(steps) and all(results)


def build_arg_parser() -> argparse.ArgumentParser:
    """@brief 构造命令行解析器：每个子命令的参数名与 run_plan_step 的 op 参数一致。
    @return ArgumentParser
    """
    parser = argparse.ArgumentParser(
        prog='arm_commander_demo', description=__doc__.split('═')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dry-run', action='store_true', help='只打印等效 ros2 指令，不下发')
    parser.add_argument('--no-cli', action='store_true', help='不打印等效 ros2 指令')
    parser.add_argument('--no-gimbal', action='store_true', help='不创建云台直连客户端')
    sub = parser.add_subparsers(dest='cmd', required=True)

    def add(name: str, help_text: str, op: Optional[str] = None) -> argparse.ArgumentParser:
        """@brief 注册一个子命令并记住对应 op。
        @param name      子命令名
        @param help_text 帮助
        @param op        run_plan_step 的 op（None = 与 name 相同，'-' 换 '_'）
        @return 子解析器
        """
        sp = sub.add_parser(name, help=help_text)
        sp.set_defaults(op=op or name.replace('-', '_'))
        return sp

    def add_speed(sp: argparse.ArgumentParser, rts: bool = True) -> None:
        """@brief 给子命令加 --speed / --return-to-start。
        @param sp  子解析器
        @param rts 是否加 --return-to-start
        """
        sp.add_argument('--speed', choices=['slow', 'normal', 'fast'], default='normal')
        if rts:
            sp.add_argument('--return-to-start', dest='return_to_start', action='store_true')

    add('status', '打印机械臂 / 云台状态', op='status')
    add('demo', '小幅度全流程演示', op='demo')
    sp = add('plan', '执行 JSON 运镜步骤表', op='plan')
    sp.add_argument('file', help='JSON 文件：[{"op": ..., ...}, ...]')
    sp.add_argument('--continue-on-error', action='store_true')

    add('enable', '伺服上电')
    add('disable', '伺服下电')
    add('homing', '回零')
    add('reset-error', '清错并复位 Commander')
    add('stop', '软件急停')
    sp = add('mode', '切控制模式')
    sp.add_argument('mode', choices=['trajectory', 'velocity'])
    add_speed(add('stow', '收纳位'))
    add_speed(add('observe', '观察位'))
    sp = add('pose', '绝对末端位姿（米 / 度）')
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        sp.add_argument(key, type=float)
    add_speed(sp)
    sp = add('move-rel', '相对当前末端位姿移动（米 / 度）')
    for key in ('dx', 'dy', 'dz'):
        sp.add_argument(key, type=float)
    for key in ('droll', 'dpitch', 'dyaw'):
        sp.add_argument(f'--{key}', type=float, default=0.0)
    add_speed(sp, rts=False)
    sp = add('joint', '关节空间点到点 J1 J2 J3（rad）')
    for key in ('j1', 'j2', 'j3'):
        sp.add_argument(key, type=float)
    sp.add_argument('--relative', action='store_true')
    sp.add_argument('--duration-sec', dest='duration_sec', type=float, default=0.0)
    add_speed(sp, rts=False)
    for name, text in (('dolly', '推拉：+x 米'), ('truck', '横移：+y（左）米'),
                       ('crane', '升降：+z 米')):
        sp = add(name, text)
        sp.add_argument('distance_m', type=float)
        add_speed(sp)
    sp = add('orbit', '球面环绕：cx cy cz az_start az_end [--radius] [--elevation]')
    for key in ('cx', 'cy', 'cz', 'az_start_deg', 'az_end_deg'):
        sp.add_argument(key, type=float)
    sp.add_argument('--radius', dest='radius_m', type=float, default=0.4)
    sp.add_argument('--elevation', dest='elevation_deg', type=float, default=0.0)
    add_speed(sp)
    sp = add('jog', '末端速度点动 vx vy vz（m/s）[--wroll --wpitch --wyaw deg/s] [--duration]')
    for key in ('vx', 'vy', 'vz'):
        sp.add_argument(key, type=float)
    for key in ('wroll', 'wpitch', 'wyaw'):
        sp.add_argument(f'--{key}', type=float, default=0.0)
    sp.add_argument('--duration', dest='duration_sec', type=float, default=1.0)
    sp = add('jog-joint', '关节速度点动 v1 v2 v3（rad/s）[--duration]')
    for key in ('v1', 'v2', 'v3'):
        sp.add_argument(key, type=float)
    sp.add_argument('--duration', dest='duration_sec', type=float, default=1.0)
    sp = add('track-start', '启动视觉跟随')
    sp.add_argument('--depth', dest='desired_depth_m', type=float, default=0.0)
    sp.add_argument('--timeout', dest='total_timeout_sec', type=float, default=0.0)
    add('track-stop', '停止视觉跟随')

    sp = add('gimbal-rotate', '云台转到 pan roll tilt（度；nan = 该轴不动）')
    for key in ('pan', 'roll', 'tilt'):
        sp.add_argument(key, type=float)
    sp.add_argument('--timeout', dest='timeout_sec', type=float, default=5.0)
    sp = add('gimbal-jog', '云台速度点动 pan_vel roll_vel tilt_vel（rad/s）[--duration]')
    for key in ('pan_vel', 'roll_vel', 'tilt_vel'):
        sp.add_argument(key, type=float)
    sp.add_argument('--duration', dest='duration_sec', type=float, default=1.0)
    for name in ('gimbal-freeze', 'gimbal-go-zero', 'gimbal-start', 'gimbal-stop',
                 'gimbal-gyro-calib'):
        add(name, f'云台 {name[7:].replace("-", "_")}')
    sp = add('gimbal-forward-enable', '开/关机械臂转发流对云台的控制权')
    sp.add_argument('enable', choices=['on', 'off'])

    def add_offline(sp: argparse.ArgumentParser) -> None:
        """@brief 给离线子命令加公共参数：--joints / --urdf / --subject。
        @param sp 子解析器
        """
        sp.add_argument('--joints', type=float, nargs=6, metavar='RAD',
                        help='当前 6 轴关节角（rad）；缺省从 /joint_states 读 2s')
        sp.add_argument('--urdf', help='URDF 文件；缺省用 xacro 现场展开安装的描述包')
        sp.add_argument('--subject', type=float, nargs=3, metavar='M',
                        help='环绕主体位置 x y z（base_link 系），给了就算环绕余量')

    sp = add('check', '离线预检 JSON 步骤表的可达性（不下发）', op='check')
    sp.add_argument('file', help='JSON 文件：[{"op": ..., ...}, ...]')
    sp.add_argument('--clip', action='store_true', help='路径类步骤超余量时夹到边界继续，而不是拒绝')
    sp.add_argument('--json', action='store_true', help='额外输出 JSON 格式报告')
    add_offline(sp)
    sp = add('headroom', '当前位姿各方向还能连续移动多少（余量盒子）', op='headroom')
    sp.add_argument('--schema', action='store_true', help='输出单步 JSON schema（min/max 硬约束）')
    add_offline(sp)
    add_offline(add('card', '生成喂给大模型的能力卡文本', op='card'))
    sp = add('fit', '拟合可达区并输出公式（喂大模型让它自检绝对坐标）', op='fit')
    sp.add_argument('--json', action='store_true', help='输出 JSON（系数、r 范围、覆盖率）')
    sp.add_argument('--degree', type=int, default=3, help='多项式次数，默认 3')
    add_offline(sp)
    sp = add('llm-step', '大模型单步运镜闭环（余量盒子方案；会动臂，先 --dry-run）', op='llm_step')
    sp.add_argument('task', help='任务描述（自然语言）')
    sp.add_argument('--llm', choices=['manual', 'openai'], default='manual',
                    help='manual = 人在终端当大模型；openai = 环境变量 LLM_BASE_URL/LLM_MODEL/LLM_API_KEY')
    sp.add_argument('--max-steps', dest='max_steps', type=int, default=8)
    sp.add_argument('--max-retries', dest='max_retries', type=int, default=2)
    sp.add_argument('--mode', choices=['clip', 'reject', 'project'], default='clip',
                    help='clip=路径类夹到边界（默认）；reject=不可行就回喂大模型重来；'
                         'project=再加上绝对位姿投影到最近可行点，超范围一律降级执行、不回喂')
    sp.add_argument('--formula', action='store_true',
                    help='把可达区拟合公式写进提示词，让大模型自检绝对坐标')
    sp.add_argument('--feedback-topic', dest='feedback_topic', default=FEEDBACK_TOPIC,
                    help=f'把给用户的反馈发到该话题（std_msgs/String），默认 {FEEDBACK_TOPIC}；'
                         'none = 不发')
    sp.add_argument('--image-topic', dest='image_topic', default=None,
                    help='给支持图像的模型附最新一帧，如 /camera/image_raw')
    sp.add_argument('--show-system', dest='show_system', action='store_true',
                    help='manual 模式下也打印 system 提示词')
    add_offline(sp)
    return parser


def load_model(args: argparse.Namespace) -> ArmModel:
    """@brief 按 --urdf（文件）或安装的描述包（xacro 现场展开）建 ArmModel。
    @param args 解析后的参数
    @return ArmModel
    """
    if getattr(args, 'urdf', None):
        with open(args.urdf, 'r', encoding='utf-8') as fp:
            return ArmModel.from_urdf_string(fp.read())
    return ArmModel.from_share_files()


def run_llm_step(api: ArmApi, args: argparse.Namespace) -> int:
    """@brief llm-step：大模型单步运镜闭环。--dry-run 只打印不下发；没有 /joint_states 时用 --joints 起步。
    @param api  ArmApi
    @param args 解析后的参数
    @return 退出码（0 = 大模型宣布完成或步数用完且全部成功；1 = 中途失败）
    """
    model = load_model(args)
    if args.llm == 'openai':
        llm = OpenAICompatClient.from_env()
    else:
        llm = ManualClient(show_system=args.show_system)
    if not api.arm.dry_run and args.joints is None and not api.arm.wait_ready():
        return 2
    region = fit_reach_region(model) if args.formula else None
    log = api.node.get_logger()
    topic = None if str(args.feedback_topic).lower() == 'none' else args.feedback_topic
    records = run_llm_shot_loop(api, model, llm, args.task, subject=args.subject,
                                image_topic=args.image_topic, dry_run=api.arm.dry_run,
                                joints_override=args.joints, max_steps=args.max_steps,
                                max_retries=args.max_retries, mode=args.mode, region_fit=region,
                                feedback_topic=topic)
    for rec in records:
        log.info(rec.line())
    print('\n──────── 给用户的反馈 ────────')
    print(format_user_report(records))
    return 0 if records and all(r.ok for r in records) else 1


def live_joints(timeout_sec: float = 2.0) -> Optional[List[float]]:
    """@brief 起一个临时节点从 /joint_states 读 6 轴关节角（Joint1..6 顺序，rad）。
    @param timeout_sec 最长等待
    @return 6 个关节角；超时或 ROS 不可用返回 None
    """
    if not rclpy.ok():
        rclpy.init()
    api = ArmApi(node_name='arm_reach_probe', print_cli=False, with_gimbal=False)
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            joints = api.arm.get_joints()
            if all(name in joints for name in ARM_JOINT_NAMES):
                return [float(joints[name]) for name in ARM_JOINT_NAMES]
            time.sleep(0.05)
        return None
    finally:
        api.shutdown()


def run_offline(args: argparse.Namespace) -> int:
    """@brief check / headroom / card：reach_check 离线工具，不建 Commander 客户端、不下发。
    @param args 解析后的参数
    @return 退出码（0 成功 / 1 步骤表不可达 / 2 拿不到关节角）
    """
    model = load_model(args)
    joints = args.joints
    if joints is None and args.cmd != 'fit':
        joints = live_joints()
        if joints is None and args.cmd != 'card':
            print('拿不到关节角：/joint_states 没数据，请用 --joints j1 j2 j3 j4 j5 j6 指定', file=sys.stderr)
            return 2
    subject = args.subject
    if args.cmd == 'fit':
        region = fit_reach_region(model, degree=args.degree)
        print(json.dumps(region.to_dict(), ensure_ascii=False, indent=2) if args.json
              else region.formula_text())
        return 0
    if args.cmd == 'card':
        print(capability_card(model, joints, subject=subject))
        return 0
    if args.cmd == 'headroom':
        h = headroom(model, joints, subject=subject)
        if args.schema:
            print(json.dumps(headroom_schema(h), ensure_ascii=False, indent=2))
        else:
            print(json.dumps(h, ensure_ascii=False, indent=2, default=lambda v: round(float(v), 4)))
        return 0
    with open(args.file, 'r', encoding='utf-8') as fp:
        steps = json.load(fp)
    if not isinstance(steps, list):
        raise ValueError('JSON 顶层必须是数组')
    report = check_plan(model, steps, joints, mode='clip' if args.clip else 'reject')
    print(report.summary())
    if args.json:
        print(json.dumps([vars(s) for s in report.steps], ensure_ascii=False, indent=2,
                         default=lambda v: round(float(v), 4)))
    return 0 if report.ok else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    """@brief 命令行入口：解析参数 → 建 ArmApi → 执行子命令 → 收尾。
    @param argv 参数列表（None = sys.argv[1:]）
    @return 进程退出码（0 成功，1 失败，2 机械臂未就绪，130 Ctrl-C）
    """
    args = build_arg_parser().parse_args(argv)
    if args.cmd in OFFLINE_CMDS:
        try:
            return run_offline(args)
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    rclpy.init()
    api = ArmApi(node_name='arm_commander_demo', dry_run=args.dry_run,
                 print_cli=not args.no_cli, with_gimbal=not args.no_gimbal)
    log = api.node.get_logger()
    exit_code = 0
    try:
        if args.cmd == 'status':
            if not api.arm.dry_run:
                api.arm.wait_ready(timeout_sec=3.0)
            print_status(api)
        elif args.cmd == 'demo':
            exit_code = 0 if run_demo_sequence(api) else 1
        elif args.cmd == 'llm-step':
            exit_code = run_llm_step(api, args)
        elif args.cmd == 'plan':
            with open(args.file, 'r', encoding='utf-8') as fp:
                steps = json.load(fp)
            if not isinstance(steps, list):
                raise ValueError('JSON 顶层必须是数组')
            if not api.arm.dry_run:
                api.arm.wait_ready()
            results = execute_plan(api, steps, stop_on_error=not args.continue_on_error)
            exit_code = 0 if (len(results) == len(steps) and all(results)) else 1
        else:
            step = {k: v for k, v in vars(args).items()
                    if k not in ('cmd', 'dry_run', 'no_cli', 'no_gimbal') and v is not None}
            if args.cmd == 'orbit':
                step['center'] = [step.pop('cx'), step.pop('cy'), step.pop('cz')]
                step['op'] = 'arc'
            if args.cmd == 'gimbal-rotate':
                for key in ('pan', 'roll', 'tilt'):
                    if math.isnan(step[key]):
                        step[key] = None
            if args.cmd == 'gimbal-forward-enable':
                step['enable'] = step['enable'] == 'on'
            if args.cmd == 'homing':
                step.setdefault('timeout_sec', DEFAULT_HOMING_TIMEOUT_SEC)
            needs_arm = not step['op'].startswith('gimbal_')
            if needs_arm and not api.arm.wait_ready():   # dry-run 下只是顺带等 1.5s 状态，恒为 True
                return 2
            result = run_plan_step(api, step)
            if result:
                log.info(f'{args.cmd}: {result}')
            else:
                log.error(f'{args.cmd}: {result}')
            exit_code = 0 if result else 1
    except KeyboardInterrupt:
        log.warning('Ctrl-C：取消当前 goal 并急停机械臂')
        api.arm.cancel()
        if not api.arm.dry_run:
            api.arm.stop()
        exit_code = 130
    finally:
        api.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
