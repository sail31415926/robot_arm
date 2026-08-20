#!/usr/bin/env python3
"""
@file   eval_orbit.py
@brief  环绕运镜策略评估/可视化：加载 PPO 模型在 MuJoCo 中回放

用法：
  cd ~/E7009_ws/src/E7009/robot_arm/robot_arm_rl/scripts
  python3 eval_orbit.py                          # 默认弹 viewer，10 轮
  python3 eval_orbit.py --no-render --episodes 50
  python3 eval_orbit.py --arc 90 --rho 0.45      # 固定口径（跨训练可比）
  python3 eval_orbit.py --model models/orbit/ppo_orbit_200000_steps

评估口径：默认 stage=3、弧度目标固定（--arc，默认 90°）、主体静止，
保证跨 checkpoint 数字可比；--stage 4 可看域随机化下的鲁棒性。

@copyright Copyright (c) 2026 eMeet
"""

import argparse
import math
import os
import sys
import time

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPTS)

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.orbit_env import ArmOrbitEnv, CTRL_HZ

MODEL_DIR = os.path.join(os.path.dirname(_SCRIPTS), 'models', 'orbit')


def main():
    parser = argparse.ArgumentParser(description='环绕运镜策略评估')
    parser.add_argument('--model', type=str,
                        default=os.path.join(MODEL_DIR, 'ppo_orbit_final'))
    parser.add_argument('--vecnorm', type=str, default=None,
                        help='VecNormalize 统计量（默认取模型同目录 final）')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--stage', type=int, default=3, choices=[1, 2, 3, 4])
    parser.add_argument('--arc', type=float, default=90.0,
                        help='固定弧度目标（deg）；0 = 按阶段随机采样')
    parser.add_argument('--rho', type=float, default=0.40)
    parser.add_argument('--no-render', action='store_true')
    parser.add_argument('--stochastic', action='store_true',
                        help='采样动作（默认确定性均值动作）')
    args = parser.parse_args()

    render_mode = None if args.no_render else 'human'
    goal_arc = math.radians(args.arc) if args.arc > 0 else None

    env = DummyVecEnv([lambda: ArmOrbitEnv(
        stage=args.stage, rho_d=args.rho, goal_arc=goal_arc,
        render_mode=render_mode)])

    vn_path = args.vecnorm or os.path.join(
        os.path.dirname(args.model.replace('.zip', '')), 'vec_normalize_final.pkl')
    if os.path.exists(vn_path):
        env = VecNormalize.load(vn_path, env)
        env.training = False          # 冻结统计量
        env.norm_reward = False       # 打印原始奖励
        print(f'VecNormalize 已加载: {vn_path}')
    else:
        print(f'⚠️ 未找到 VecNormalize（{vn_path}），策略将看到错误的观测分布')

    model = PPO.load(args.model.replace('.zip', ''), device='cpu')
    print(f'模型: {args.model}  stage={args.stage}  '
          f'arc={args.arc:.0f}°  rho_d={args.rho}\n')

    results = []
    for ep in range(args.episodes):
        obs = env.reset()
        done, ep_rew, info = False, 0.0, {}
        while not done:
            act, _ = model.predict(obs, deterministic=not args.stochastic)
            obs, rew, dones, infos = env.step(act)
            done, info = bool(dones[0]), infos[0]
            ep_rew += float(rew[0])
            if render_mode:
                time.sleep(1.0 / CTRL_HZ)
        results.append(info)
        print(f'  ep{ep:02d}  {"✓成功" if info["success"] else "✗" + info["reason"]:<12}'
              f'  进度 {info["progress_deg"]:6.1f}° / {info["goal_deg"]:5.1f}°'
              f'  中心误差 {info["center_err"]:.3f}  回报 {ep_rew:8.1f}')

    n = len(results)
    succ = sum(r['success'] for r in results)
    print(f'\n== 汇总 ({n} 轮) ==')
    print(f'  成功率        {succ / n * 100:5.1f}%  ({int(succ)}/{n})')
    print(f'  平均中心误差  {np.mean([r["center_err"] for r in results]):.3f}')
    print(f'  平均环绕进度  {np.mean([r["progress_deg"] for r in results]):.1f}°'
          f'  (目标 {np.mean([r["goal_deg"] for r in results]):.1f}°)')
    env.close()


if __name__ == '__main__':
    main()
