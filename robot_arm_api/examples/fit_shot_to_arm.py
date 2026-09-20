#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file      fit_shot_to_arm.py
@brief     把大模型输出的分镜（规范 v9 A 型）等比缩放 / 平移到单臂工作空间——**演示用**，不是规范的一部分。
@version   0.1
@date      2026-09-17
@copyright Copyright (c) 2026 eMeet

为什么要这一步：大模型输出的分镜是给整机（底盘走位 + 臂运镜）设计的，机位在被摄物 0.5~4.5 m 外；
单臂的肩到末端上限只有 0.60 m，直接执行会被规范规则 25（IK 不可达）全部拒掉。
把整组分镜**以被摄物为中心等比缩放**再平移到臂前方，角度关系、路径形状、时序全部保持不变，
只是尺度变小——相当于用模型代替实物拍，用来在仿真里看镜头语言。

变换（对每个 waypoint 的 position / look_at 和 arc 段的 center）：
    新点 = 目标中心 + k × (原点 − 原中心)
原中心取整组分镜所有 look_at 的重心；k 由 --scale 给定，或由 --target-d 反推
（k = 目标视距 / 原分镜的中位视距）。速度类字段（duration / speed）不动：
缩放后路径变短、时长不变，等于放慢，符合"看清楚运镜形态"的目的。

用法:
  ./fit_shot_to_arm.py 输入.json 输出目录 [--center X Y Z] [--target-d 0.15] [--shot-index N]
  ./fit_shot_to_arm.py ~/eMeetWork_sail/摄影机器人规范/大模型输出示例/output/model_car.json out/ \\
      --center 0.45 0 0.38 --target-d 0.16
"""

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_shots(path: str) -> List[Dict[str, Any]]:
    """@brief 从大模型输出文件里取出 shot_plans（也接受单条分镜或分镜数组）。
    @param path 文件路径
    @return 分镜列表
    """
    with open(path, encoding='utf-8') as fh:
        data = json.load(fh)
    if isinstance(data, dict) and 'type' in data:
        return [data]
    if isinstance(data, dict):
        plans = data.get('shot_plans')
        if plans is None:
            plans = (data.get('agent_outputs', {}).get('cinematographer', {}) or {}).get('shot_plans')
        if plans is None:
            raise ValueError('文件里没有 shot_plans')
        return plans
    return data


def source_center(shots: List[Dict[str, Any]], anchor: str = 'bottom') -> List[float]:
    """@brief 原分镜的被摄物参考点。look_at 散布在被摄物的不同高度（拍车顶 / 车身 / 车底），
           所以 x、y 取重心，z 按 anchor 选：
             bottom（默认）—— 取最低的 look_at，即被摄物**底面**。放在桌上的东西用这个，
                              缩放后把底面对齐桌面，物体不会浮空或陷进桌子。
             center        —— 取 z 的重心。悬空的被摄物用这个。
    @param shots  分镜列表
    @param anchor 'bottom' | 'center'
    @return [x, y, z]
    """
    pts = [w['look_at'] for s in shots for w in s.get('waypoints', []) if 'look_at' in w]
    if not pts:
        raise ValueError('这组分镜里没有 look_at（可能全是 B 型），无法定参考点')
    x = sum(p[0] for p in pts) / len(pts)
    y = sum(p[1] for p in pts) / len(pts)
    z = min(p[2] for p in pts) if anchor == 'bottom' else sum(p[2] for p in pts) / len(pts)
    return [x, y, z]


def subject_height(shots: List[Dict[str, Any]]) -> float:
    """@brief 从 look_at 的 z 跨度估计被摄物高度（大模型把 look_at 打在被摄物的不同高度上）。
    @param shots 分镜列表
    @return 米
    """
    zs = [w['look_at'][2] for s in shots for w in s.get('waypoints', []) if 'look_at' in w]
    return max(zs) - min(zs)


def median_distance(shots: List[Dict[str, Any]]) -> float:
    """@brief 原分镜的中位视距（position 到 look_at 的距离）。
    @param shots 分镜列表
    @return 米
    """
    ds = [math.dist(w['position'], w['look_at'])
          for s in shots for w in s.get('waypoints', []) if 'position' in w and 'look_at' in w]
    if not ds:
        raise ValueError('这组分镜里没有可用的 position / look_at')
    ds.sort()
    return ds[len(ds) // 2]


def fit_shot(shot: Dict[str, Any], src_c: List[float], dst_c: List[float], k: float) -> Dict[str, Any]:
    """@brief 对一条分镜做缩放 + 平移。
    @param shot  原分镜
    @param src_c 原中心
    @param dst_c 目标中心（臂基座系）
    @param k     缩放系数
    @return 新分镜
    """
    def _map(p):
        return [round(dst_c[i] + k * (p[i] - src_c[i]), 5) for i in range(3)]
    out = json.loads(json.dumps(shot))
    for w in out.get('waypoints', []):
        for key in ('position', 'look_at'):
            if key in w:
                w[key] = _map(w[key])
    for seg in out.get('segments', []):
        if 'center' in seg:
            seg['center'] = _map(seg['center'])
        if 'speed' in seg:                      # 线速度随尺度一起缩，保持"看起来一样快"
            seg['speed'] = round(seg['speed'] * k, 5)
    out.pop('shot_id', None) if False else None
    return out


def main(argv=None) -> int:
    """@brief 入口。
    @param argv 参数
    @return 退出码
    """
    ap = argparse.ArgumentParser(description=__doc__.split('用法')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='大模型输出 JSON')
    ap.add_argument('outdir', help='输出目录')
    ap.add_argument('--center', type=float, nargs=3, default=[0.42, 0.0, 0.35], metavar=('X', 'Y', 'Z'),
                    help='被摄物参考点在 arm_base_link 系的位置；anchor=bottom 时就是**底面中心**'
                         '（放桌上就填桌面高度），默认 0.42 0 0.35（桌面、臂正前方）')
    ap.add_argument('--anchor', choices=['bottom', 'center'], default='bottom',
                    help='原分镜的哪个部位对齐到 --center：bottom=被摄物底面（放桌上，默认）/ center=重心')
    ap.add_argument('--target-d', type=float, default=0.16, help='缩放后的中位视距（m），默认 0.16')
    ap.add_argument('--scale', type=float, default=None, help='直接给缩放系数，覆盖 --target-d')
    ap.add_argument('--shot-index', type=int, default=None, help='只转第几镜（默认全部）')
    args = ap.parse_args(argv)

    shots = load_shots(args.input)
    if args.shot_index is not None:
        shots = [shots[args.shot_index]]
    static = [s for s in shots if s.get('type') == 'static']
    if len(static) < len(shots):
        print(f'⚠ 跳过 {len(shots) - len(static)} 条 B 型分镜（需要目标话题，另行处理）', file=sys.stderr)
    src_c = source_center(static, args.anchor)
    med = median_distance(static)
    h0 = subject_height(static)
    k = args.scale if args.scale is not None else args.target_d / med
    os.makedirs(args.outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.input))[0]
    print(f'原参考点({args.anchor}) {[round(v,3) for v in src_c]}  中位视距 {med:.2f} m  '
          f'被摄物高 {h0:.2f} m  →  目标 {args.center}  缩放 k={k:.4f}')
    print(f'★ 缩放后被摄物应有的尺寸：高 {h0*k*1000:.0f} mm'
          f'（Gazebo 里的模型必须做成这个大小，否则构图对不上）')
    for i, shot in enumerate(static):
        sid = shot.get('shot_id', i + 1)
        out = fit_shot(shot, src_c, args.center, k)
        path = os.path.join(args.outdir, f'{base}_shot{sid}.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
        ds = [round(math.dist(w['position'], w['look_at']), 3) for w in out['waypoints']]
        print(f'  shot {sid}: 视距 {ds}  hold {[w.get("hold",0) for w in out["waypoints"]]}  → {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
