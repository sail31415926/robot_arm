#!/usr/bin/env python3
"""
@file   train_sac.py
@brief  eMeetArm 摄影构图 RL 训练脚本（SAC，Stable-Baselines3）

前置：构建并 source 工作区
  cd ~/eMeet_ws
  colcon build --packages-select robot_arm_rl robot_arm_description --symlink-install
  source install/setup.bash

用法（ros2 run）：
  # 从头训练
  ros2 run robot_arm_rl train_sac

  # 继续训练已有模型
  ros2 run robot_arm_rl train_sac --resume models/sac_emeet_arm_latest

  # 被摄主体随机游走（增加训练难度）
  ros2 run robot_arm_rl train_sac --subject-motion

  # 开启 MuJoCo 渲染（单环境，调试用）
  ros2 run robot_arm_rl train_sac --render

  # 调整并行环境数与训练步数
  ros2 run robot_arm_rl train_sac --n-envs 4 --timesteps 1000000

用法（直接 Python，无需 ROS2）：
  cd ~/eMeet_ws/src/E7009/robot_arm/robot_arm_rl/scripts
  python3 train_sac.py [同上选项]

依赖：
  pip install stable-baselines3 gymnasium mujoco
  pip install tensorboard   # 可选，用于可视化训练曲线

@copyright Copyright (c) 2026 eMeet
"""

import argparse
import os
import sys

# 把 envs/ 加入路径
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPTS)

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, SubprocVecEnv
from stable_baselines3.common.callbacks import (
    EvalCallback,
    CheckpointCallback,
    BaseCallback,
)
from stable_baselines3.common.monitor import Monitor

from envs.emeet_arm_env import eMeetArmEnv

# ── GPU 检测 ─────────────────────────────────────────────────────────────────
import torch as _torch
_DEVICE = 'cuda' if _torch.cuda.is_available() else 'cpu'
if _DEVICE == 'cuda':
    _gpu = _torch.cuda.get_device_properties(0)
    print(f'[GPU] {_gpu.name}  VRAM={_gpu.total_memory/1024**3:.1f} GB')
else:
    print('[GPU] CUDA 不可用，使用 CPU 训练')

# ── 超参数 ────────────────────────────────────────────────────────────────────
TOTAL_TIMESTEPS = 2_000_000
# GPU 训练：更多并行环境供给经验，更大 batch 充分利用显卡
N_ENVS          = 8      # SubprocVecEnv 并行数（CPU 核心）
EVAL_FREQ       = 20_000
N_EVAL_EPISODES = 5
SAVE_FREQ       = 50_000


def _tb_log():
    """tensorboard 未安装时返回 None，避免启动失败。"""
    try:
        import tensorboard  # noqa: F401
        os.makedirs('./logs/tb', exist_ok=True)
        return './logs/tb'
    except ImportError:
        return None


SAC_KWARGS = dict(
    device           = _DEVICE,
    learning_rate    = 3e-4,
    # GPU 可容纳更大经验回放缓冲（RTX 5070 Ti 11.5 GB VRAM）
    buffer_size      = 1_000_000,
    learning_starts  = 10_000,   # N_ENVS=8，约 1250 个 episode 步后开始更新
    # 大 batch + 多梯度步 → GPU 利用率显著提升
    batch_size       = 1024,
    gradient_steps   = 4,        # 每收集一步做 4 次梯度更新
    tau              = 0.005,
    gamma            = 0.99,
    train_freq       = 1,
    ent_coef         = 'auto',
    # 更宽网络适配 GPU 并行计算能力
    policy_kwargs    = dict(
        net_arch       = [512, 512],
        optimizer_kwargs = dict(eps=1e-5),
    ),
    verbose          = 1,
    tensorboard_log  = _tb_log(),
)

# ── 路径 ────────────────────────────────────────────────────────────────────
MODEL_DIR = os.path.join(os.path.dirname(_SCRIPTS), 'models')
os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs('./logs/tb', exist_ok=True)
os.makedirs('./logs/eval', exist_ok=True)


# ── 回调：保存时同步 VecNormalize ─────────────────────────────────────────────
class SaveVecNormCallback(BaseCallback):
    def __init__(self, eval_cb: EvalCallback, save_dir: str, verbose=0):
        super().__init__(verbose)
        self._eval_cb  = eval_cb
        self._save_dir = save_dir
        self._best     = -np.inf

    def _on_step(self) -> bool:
        if self._eval_cb.best_mean_reward > self._best:
            self._best = self._eval_cb.best_mean_reward
            path = os.path.join(self._save_dir, 'vec_normalize_best.pkl')
            if hasattr(self.training_env, 'save'):
                self.training_env.save(path)
                if self.verbose:
                    print(f'[SaveVecNorm] saved → {path}')
        return True


def make_env(subject_motion: bool, render_mode=None):
    def _init():
        env = eMeetArmEnv(subject_motion=subject_motion, render_mode=render_mode)
        return Monitor(env)
    return _init


def main():
    parser = argparse.ArgumentParser(description='eMeetArm SAC 训练')
    parser.add_argument('--resume',         type=str,  default=None,
                        help='继续训练的模型路径（不含 .zip）')
    parser.add_argument('--subject-motion', action='store_true',
                        help='允许被摄主体随机游走（增加训练难度）')
    parser.add_argument('--render',         action='store_true',
                        help='开启 MuJoCo viewer（仅单环境，调试用）')
    parser.add_argument('--timesteps',      type=int,  default=TOTAL_TIMESTEPS)
    parser.add_argument('--n-envs',         type=int,  default=N_ENVS,
                        help=f'并行环境数，默认 {N_ENVS}')
    args = parser.parse_args()

    render_mode = 'human' if args.render else None
    n_envs      = 1 if args.render else args.n_envs

    # ── 训练环境 ──────────────────────────────────────────────────────────────
    vec_cls = SubprocVecEnv if n_envs > 1 else make_vec_env
    if n_envs > 1:
        train_env = SubprocVecEnv(
            [make_env(args.subject_motion) for _ in range(n_envs)])
    else:
        train_env = make_vec_env(
            lambda: eMeetArmEnv(subject_motion=args.subject_motion,
                                render_mode=render_mode),
            n_envs=1)

    train_env = VecNormalize(train_env, norm_obs=True, norm_reward=True,
                              clip_obs=10.0, clip_reward=10.0)

    # ── 评估环境（不归一化奖励，方便观察原始得分）────────────────────────────
    eval_env = VecNormalize(
        make_vec_env(lambda: eMeetArmEnv(subject_motion=False), n_envs=1),
        norm_obs=True, norm_reward=False, training=False)

    # ── 回调 ──────────────────────────────────────────────────────────────────
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path = os.path.join(MODEL_DIR, 'best'),
        log_path             = './logs/eval',
        eval_freq            = max(EVAL_FREQ // n_envs, 1),
        n_eval_episodes      = N_EVAL_EPISODES,
        deterministic        = True,
        verbose              = 1,
    )
    checkpoint_cb = CheckpointCallback(
        save_freq   = max(SAVE_FREQ // n_envs, 1),
        save_path   = MODEL_DIR,
        name_prefix = 'sac_emeet_arm',
    )
    vec_norm_cb = SaveVecNormCallback(eval_cb, MODEL_DIR, verbose=1)

    callbacks = [eval_cb, checkpoint_cb, vec_norm_cb]

    # ── 模型 ──────────────────────────────────────────────────────────────────
    if args.resume:
        print(f'从 {args.resume} 继续训练...')
        model = SAC.load(args.resume, env=train_env, **{
            k: v for k, v in SAC_KWARGS.items() if k != 'verbose'})
        vec_norm_path = args.resume.replace('.zip', '') + '_vec_normalize.pkl'
        if os.path.exists(vec_norm_path):
            train_env = VecNormalize.load(vec_norm_path, train_env.venv)
            print(f'  VecNormalize 已加载: {vec_norm_path}')
    else:
        model = SAC('MlpPolicy', train_env, **SAC_KWARGS)

    # ── 训练 ──────────────────────────────────────────────────────────────────
    print(f'开始训练  timesteps={args.timesteps}  n_envs={n_envs}')
    try:
        model.learn(
            total_timesteps  = args.timesteps,
            callback         = callbacks,
            reset_num_timesteps = args.resume is None,
            progress_bar     = True,
        )
    except KeyboardInterrupt:
        print('\n训练中断，保存当前模型...')

    # ── 保存最终模型 ──────────────────────────────────────────────────────────
    final_path = os.path.join(MODEL_DIR, 'sac_emeet_arm_final')
    model.save(final_path)
    train_env.save(os.path.join(MODEL_DIR, 'vec_normalize_final.pkl'))
    print(f'模型已保存 → {final_path}.zip')

    train_env.close()
    eval_env.close()


if __name__ == '__main__':
    main()
