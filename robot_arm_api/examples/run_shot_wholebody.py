#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file      run_shot_wholebody.py
@brief     《拍摄接口规范 v9》A 型分镜 → 全身控制（底盘 + 臂 + 云台）执行。**演示脚本**，不是适配层的一部分。
@version   0.1
@date      2026-09-18
@copyright Copyright (c) 2026 eMeet

和 robot_arm_api 那套单臂适配层的区别：单臂只有 0.6 m 工作半径，大模型分镜 0.8~4.5 m 的机位必须缩放；
底盘参与后**不用缩放**，直接用原始坐标。规范附录 D 指明的对应关系在这里逐条兑现：
    A 型 waypoint 的 position + look_at + roll
        → 规划器 ~target 的 {pos, look_at, roll}（走位 + 到位，~plan_to + ~execute）
        → 或控制器 ~/task 的 η 口径 {subject, nom:{d, alpha, beta, u, v, roll}}
           其中 d=视距、alpha=方位（世界 +X 起、逆时针为正，= 规范的 az）、beta=仰角（= 规范的 el）、
           u/v=构图（A 型光轴对准 look_at ⇒ 0）、roll=画面倾斜。
    arc 段 → 控制器的绕圆口径 {look_at, orbit:{radius, cam_z, sweep_deg, start_deg}}

⚠ 避障：规划器的 ~plan_to 在没有地图时会**降级成直线直冲**（消息里会写明），撞上就停。
   所以本脚本支持 --via 分段走位：把无遮挡的中间点串起来，绕开桌子/背景墙这类静态障碍。
   这符合规范 2.3 的分工——走位归导航栈，运镜段不改相机参考轨迹。

用法:
  ./run_shot_wholebody.py 分镜.json --subject X Y Z [--via X Y ...] [--shot-index N] [--dry-run]
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class WholeBodyShot:
    """@brief 规划器 / 控制器的薄封装：发目标 → 规划 → 回放 → 等完成。"""

    def __init__(self, node, planner='/wholebody_planner'):
        """@brief 构造。
        @param node    rclpy 节点
        @param planner 规划器命名空间
        """
        self.node = node
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_target = node.create_publisher(String, f'{planner}/target', q)
        self.cli = {n: node.create_client(Trigger, f'{planner}/{n}')
                    for n in ('plan_to', 'plan', 'execute', 'stop', 'reset')}
        self.log = node.get_logger()

    def call(self, name, timeout=300.0):
        """@brief 调一个 Trigger 服务。
        @param name    服务名
        @param timeout 超时秒
        @return (success, message)
        """
        cli = self.cli[name]
        if not cli.wait_for_service(timeout_sec=15.0):
            return False, f'{name} 服务不可用'
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not fut.done():
            return False, f'{name} 超时 {timeout}s'
        r = fut.result()
        return r.success, r.message

    def goto(self, pos, look_at, roll=0.0, label='', settle=3.0):
        """@brief 让相机去一个 6D 位姿（走位或到达运镜起点）：发 target → plan_to → execute → 等回放完。
        @param pos     相机光心世界坐标
        @param look_at 光轴对准点
        @param roll    画面倾斜，rad
        @param label   日志标签
        @param settle  回放结束后的静定等待
        @return True 成功
        """
        task = {'pos': [float(v) for v in pos], 'look_at': [float(v) for v in look_at],
                'roll': float(roll)}
        self.pub_target.publish(String(data=json.dumps(task)))
        time.sleep(0.8)
        ok, msg = self.call('plan_to')
        self.log.info(f'{label} plan_to: {msg[:110]}')
        if not ok:
            return False
        if '降级直线' in msg:
            self.log.warning(f'{label} ⚠ 没有地图 → 直线直冲，不绕障；路径上必须无静态障碍')
        dur = 0.0
        m = [t for t in msg.split() if t.startswith('时长')]
        try:
            dur = float(msg.split('时长')[1].split('s')[0])
        except Exception:
            dur = 60.0
        ok, msg = self.call('execute', timeout=30.0)
        self.log.info(f'{label} execute: {msg[:90]}')
        if not ok:
            return False
        # execute 是异步的，按规划时长等它跑完
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < dur + settle:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return True


def load_shots(path, index=None):
    """@brief 读分镜（单条 / 数组 / 大模型输出文件），只留 A 型。
    @param path  文件
    @param index 取第几镜
    @return 分镜列表
    """
    with open(path, encoding='utf-8') as fh:
        d = json.load(fh)
    if isinstance(d, dict) and 'type' in d:
        shots = [d]
    elif isinstance(d, dict):
        shots = d.get('shot_plans') or (
            d.get('agent_outputs', {}).get('cinematographer', {}) or {}).get('shot_plans') or []
    else:
        shots = d
    shots = [s for s in shots if s.get('type') == 'static']
    return [shots[index]] if index is not None else shots


def main(argv=None):
    """@brief 入口。
    @param argv 参数
    @return 退出码
    """
    ap = argparse.ArgumentParser(description=__doc__.split('用法')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file', help='规范分镜 JSON / 大模型输出文件')
    ap.add_argument('--subject', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'),
                    help='被摄物在 odom 系的位置（分镜原点平移到这里），**不缩放**')
    ap.add_argument('--via', type=float, nargs='+', default=[], metavar='X Y',
                    help='走位中间点（odom 系，成对给 x y）：绕开桌子/背景墙用，'
                         '因为没有地图时 plan_to 是直线直冲')
    ap.add_argument('--via-z', type=float, default=0.65, help='走位时的相机高度，默认 0.65')
    ap.add_argument('--shot-index', type=int, default=None, help='只跑第几镜')
    ap.add_argument('--dry-run', action='store_true', help='只打印，不下发')
    ap.add_argument('--status-file', default=None, help='把当前目标写进这个 JSON，供录像 HUD 叠加')
    a = ap.parse_args(argv)

    shots = load_shots(a.file, a.shot_index)
    off = np.array(a.subject, float)
    vias = [(a.via[i], a.via[i + 1]) for i in range(0, len(a.via) - 1, 2)]

    if a.dry_run:
        for i, (vx, vy) in enumerate(vias):
            print(f'走位{i}: 相机 [{vx}, {vy}, {a.via_z}]')
        for si, s in enumerate(shots):
            for wi, w in enumerate(s['waypoints']):
                p = np.array(w['position'], float) + off
                la = np.array(w['look_at'], float) + off
                rel = p - la
                d = np.linalg.norm(rel)
                print(f"shot{s.get('shot_id', si+1)} 点{wi}: pos={np.round(p,3).tolist()} "
                      f"look_at={np.round(la,3).tolist()}  d={d:.2f}m "
                      f"az={math.degrees(math.atan2(rel[1], rel[0])):.0f}° "
                      f"el={math.degrees(math.asin(rel[2]/d)):.0f}°  hold={w.get('hold',0)}s")
        return 0

    rclpy.init()
    node = rclpy.create_node('run_shot_wholebody')
    wb = WholeBodyShot(node)
    time.sleep(1.5)
    try:
        for i, (vx, vy) in enumerate(vias):
            nxt = vias[i + 1] if i + 1 < len(vias) else (off[0], off[1])
            look = [nxt[0], nxt[1], a.via_z]
            if a.status_file:      # 走位不属于分镜，HUD 只标注去哪、不评容差
                json.dump({'label': f'WALK {i+1}/{len(vias)} (approach, not in shot)',
                           'target': [vx, vy, a.via_z], 'd': 0.0, 'az': 0.0, 'el': 0.0},
                          open(a.status_file, 'w'))
            if not wb.goto([vx, vy, a.via_z], look, 0.0, f'[走位{i+1}/{len(vias)}]'):
                node.get_logger().error('走位失败，中止')
                return 1
        for si, shot in enumerate(shots):
            sid = shot.get('shot_id', si + 1)
            node.get_logger().info(f'════ shot {sid} ════')
            for wi, w in enumerate(shot['waypoints']):
                p = (np.array(w['position'], float) + off).tolist()
                la = (np.array(w['look_at'], float) + off).tolist()
                if a.status_file:
                    rel = np.array(p) - np.array(la)
                    dd = float(np.linalg.norm(rel))
                    json.dump({'label': f'SHOT {sid}  WAYPOINT {wi+1}/{len(shot["waypoints"])}'
                                        f'  (spec v9 type=static, arc segment)',
                               'target': p, 'd': dd,
                               'az': math.degrees(math.atan2(rel[1], rel[0])),
                               'el': math.degrees(math.asin(rel[2]/dd))},
                              open(a.status_file, 'w'))
                if not wb.goto(p, la, float(w.get('roll', 0.0)), f'[shot{sid} 点{wi}]'):
                    node.get_logger().error(f'shot {sid} 点{wi} 失败')
                    break
                hold = float(w.get('hold', 0.0))
                if hold > 0:
                    node.get_logger().info(f'  hold {hold}s')
                    if a.status_file:
                        d0 = json.load(open(a.status_file))
                        d0['label'] = d0['label'] + f'  [HOLD {hold:.0f}s]'
                        json.dump(d0, open(a.status_file, 'w'))
                    t0 = time.time()
                    while rclpy.ok() and time.time() - t0 < hold:
                        rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    finally:
        wb.call('stop', timeout=20)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
