#!/usr/bin/env python3
"""
@file   sim_to_sim_test.py
@brief  Sim-to-Sim 泛化测试（构图跟拍任务）

对比策略在「训练分布内」与「分布外条件」下的性能差距，
评估策略是否真正泛化，而非只是记住训练场景。

用法：
  python3 sim_to_sim_test.py

测试条件（Sim B vs Sim A 基准）：
  B1. 主体位置超出训练范围（更远，部分目标不可达 → 考验边界行为）
  B2. 构图目标超出训练采样范围（更极端的偏心构图）
  B3. 主体速度超出训练范围（2 倍速游走）

实现说明：条件通过 monkeypatch 环境模块常量注入（reset 在调用时读取
模块全局量），不再扒 env 私有状态——环境内部实现变了测试也不用跟着改。
"""

import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import envs.emeet_arm_env as E
from envs.emeet_arm_env import eMeetArmEnv, DEFAULT_GOAL

MODEL_PATH   = '../models/best/best_model'
VECNORM_PATH = '../models/vec_normalize_best.pkl'
N_EPISODES   = 10

# 训练分布默认值（用于每个条件跑完后还原）
_DEFAULTS = dict(
    SUBJ_X_RANGE     = E.SUBJ_X_RANGE,
    SUBJ_SPEED_MAX   = E.SUBJ_SPEED_MAX,
    SUBJ_STATIC_PROB = E.SUBJ_STATIC_PROB,
)


def _restore_defaults():
    for k, v in _DEFAULTS.items():
        setattr(E, k, v)


def run_condition(model_path, vecnorm_path, name,
                  patches=None, goal=None, subject_motion=True,
                  n_episodes=N_EPISODES):
    """在指定条件下评估。patches = 环境模块常量覆盖 dict。"""
    _restore_defaults()
    for k, v in (patches or {}).items():
        setattr(E, k, v)

    env = DummyVecEnv([lambda: eMeetArmEnv(
        subject_motion=subject_motion, goal=goal)])
    if os.path.isfile(vecnorm_path):
        env = VecNormalize.load(vecnorm_path, env)
        env.training = False
        env.norm_reward = False
    model = SAC.load(model_path, env=env)
    raw_env = env.envs[0] if hasattr(env, 'envs') else env.venv.envs[0]

    rewards, framing_errs, visible_rates = [], [], []
    for _ in range(n_episodes):
        obs = env.reset()
        u_tgt, v_tgt, _ = raw_env.goal
        ep_reward, framing_list, visible_steps, steps = 0.0, [], 0, 0
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            u, v, s = raw_env._project_subject()
            framing_list.append(np.sqrt((u - u_tgt)**2 + (v - v_tgt)**2))
            if 0.05 < u < 0.95 and 0.05 < v < 0.95:
                visible_steps += 1
            ep_reward += reward[0]
            steps += 1
            if done[0]:
                break
        rewards.append(ep_reward)
        framing_errs.append(np.mean(framing_list))
        visible_rates.append(visible_steps / steps)

    env.close()
    _restore_defaults()
    return {
        'name':    name,
        'reward':  (np.mean(rewards),       np.std(rewards)),
        'framing': (np.mean(framing_errs),  np.std(framing_errs)),
        'visible': (np.mean(visible_rates), np.std(visible_rates)),
    }


def print_result(r: dict):
    rew_m, rew_s = r['reward']
    fra_m, fra_s = r['framing']
    vis_m, vis_s = r['visible']
    bar = '█' * int(vis_m * 20)
    print(f"\n  【{r['name']}】")
    print(f"    平均奖励   : {rew_m:8.1f} ± {rew_s:.1f}")
    print(f"    构图误差   : {fra_m:.4f} ± {fra_s:.4f}  （0=完美，1=偏全屏）")
    print(f"    可见率     : {vis_m*100:.1f}% ± {vis_s*100:.1f}%  |{bar:<20}|")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path   = os.path.join(script_dir, MODEL_PATH)
    vecnorm_path = os.path.join(script_dir, VECNORM_PATH)

    print('=' * 60)
    print('Sim-to-Sim 泛化测试')
    print('=' * 60)

    results = [
        run_condition(model_path, vecnorm_path,
                      'Sim A  训练分布内（基准，默认构图 + 运动主体）',
                      goal=DEFAULT_GOAL),
        run_condition(model_path, vecnorm_path,
                      'Sim B1 主体超出范围（更远 x:1.1~1.4，部分不可达）',
                      patches={'SUBJ_X_RANGE': (1.10, 1.40)},
                      goal=DEFAULT_GOAL),
        run_condition(model_path, vecnorm_path,
                      'Sim B2 构图目标超出训练采样范围（u*=0.25 v*=0.80）',
                      goal=(0.25, 0.80, 0.06)),
        run_condition(model_path, vecnorm_path,
                      'Sim B3 主体 2 倍速游走（恒运动）',
                      patches={'SUBJ_SPEED_MAX': 2 * E.SUBJ_SPEED_MAX,
                               'SUBJ_STATIC_PROB': 0.0},
                      goal=DEFAULT_GOAL),
    ]

    print('\n' + '=' * 60)
    print('结果汇总')
    print('=' * 60)
    for r in results:
        print_result(r)

    print('\n' + '=' * 60)
    print('泛化差距（相对 Sim A 基准）')
    print('=' * 60)
    base_vis = results[0]['visible'][0]
    base_fra = results[0]['framing'][0]
    for r in results[1:]:
        vis_drop = (base_vis - r['visible'][0]) * 100
        fra_rise = r['framing'][0] - base_fra
        flag = '⚠️  泛化差' if vis_drop > 20 or fra_rise > 0.2 else '✓  泛化OK'
        print(f"  {r['name']}")
        print(f"    可见率下降 {vis_drop:+.1f}%   构图误差上升 {fra_rise:+.4f}   {flag}")


if __name__ == '__main__':
    main()
