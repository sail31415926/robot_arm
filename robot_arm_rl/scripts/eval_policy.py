#!/usr/bin/env python3
"""
@file   eval_policy.py
@brief  评估已训练的 SAC 策略（构图跟拍任务）

用法：
  # 可视化评估（弹出 MuJoCo 窗口，跑 5 轮，静止主体 + 默认三分构图）
  python3 eval_policy.py

  # 纯数字评估（无窗口，跑更多轮）
  python3 eval_policy.py --no-render --n-episodes 20

  # 运动主体评估
  python3 eval_policy.py --no-render --subject-motion

  # 指定构图目标 / 模型路径
  python3 eval_policy.py --goal 0.5 0.33 0.06 --model ../models/best/best_model
"""

import argparse
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from envs.emeet_arm_env import eMeetArmEnv, DEFAULT_GOAL


def evaluate(model_path: str, vecnorm_path: str, n_episodes: int,
             render: bool, subject_motion: bool, goal):
    render_mode = 'human' if render else None

    env = DummyVecEnv([lambda: eMeetArmEnv(
        subject_motion=subject_motion, goal=goal, render_mode=render_mode)])
    if os.path.isfile(vecnorm_path):
        env = VecNormalize.load(vecnorm_path, env)
        env.training = False
        env.norm_reward = False
        print(f'VecNormalize 已加载: {vecnorm_path}')
    else:
        print('未找到 VecNormalize，直接加载模型')

    model = SAC.load(model_path, env=env)
    print(f'模型已加载: {model_path}')
    print(f'构图目标: u*={goal[0]:.2f} v*={goal[1]:.2f} s*={goal[2]:.3f}  '
          f'主体: {"运动" if subject_motion else "静止"}\n')

    raw_env = env.envs[0] if hasattr(env, 'envs') else env.venv.envs[0]

    ep_rewards, ep_framings, ep_visible, ep_lengths = [], [], [], []

    for ep in range(n_episodes):
        obs = env.reset()
        u_tgt, v_tgt, _ = raw_env.goal
        ep_reward, framing_errs, visible_steps, step = 0.0, [], 0, 0

        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)

            u, v, s = raw_env._project_subject()
            framing_errs.append(np.sqrt((u - u_tgt)**2 + (v - v_tgt)**2))
            if 0.05 < u < 0.95 and 0.05 < v < 0.95:
                visible_steps += 1

            ep_reward += reward[0]
            step += 1
            if done[0]:
                break

        ep_rewards.append(ep_reward)
        ep_framings.append(np.mean(framing_errs))
        ep_visible.append(visible_steps / step)
        ep_lengths.append(step)

        print(f'  第{ep+1:2d}轮  奖励={ep_reward:8.1f}  '
              f'构图误差={np.mean(framing_errs):.3f}  '
              f'可见率={visible_steps/step*100:.1f}%  '
              f'步数={step}')

    print('\n' + '='*60)
    print(f'评估轮数       : {n_episodes}')
    print(f'平均总奖励     : {np.mean(ep_rewards):8.1f}  ± {np.std(ep_rewards):.1f}')
    print(f'平均构图误差   : {np.mean(ep_framings):.4f}  ± {np.std(ep_framings):.4f}')
    print(f'  （误差含义：0=构图点完美命中，0.5=偏半屏，1.0=偏全屏）')
    print(f'平均主体可见率 : {np.mean(ep_visible)*100:.1f}%')
    print(f'平均轮长       : {np.mean(ep_lengths):.0f} 步（500=跑满，短=丢主体提前终止）')
    print('='*60)

    env.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',      default='../models/best/best_model',
                        help='模型路径（不含 .zip）')
    parser.add_argument('--vecnorm',    default='../models/vec_normalize_best.pkl',
                        help='VecNormalize 路径')
    parser.add_argument('--n-episodes', type=int, default=5)
    parser.add_argument('--no-render',  action='store_true', help='关闭 MuJoCo 窗口')
    parser.add_argument('--subject-motion', action='store_true',
                        help='评估时主体运动（默认静止）')
    parser.add_argument('--goal', type=float, nargs=3, default=list(DEFAULT_GOAL),
                        metavar=('U', 'V', 'S'),
                        help=f'构图目标 u* v* s*（默认 {DEFAULT_GOAL}）')
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    evaluate(os.path.join(script_dir, args.model),
             os.path.join(script_dir, args.vecnorm),
             args.n_episodes, not args.no_render,
             args.subject_motion, tuple(args.goal))


if __name__ == '__main__':
    main()
