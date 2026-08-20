#!/usr/bin/env python3
"""
@file   train_ppo_orbit.py
@brief  环绕运镜 RL 训练脚本（PPO，Stable-Baselines3，四阶段课程学习）

前置：构建并 source 工作区（或直接 Python 跑，见下）
  cd ~/E7009_ws && source /opt/ros/humble/setup.bash && source .venv/bin/activate

用法（直接 Python）：
  cd ~/E7009_ws/src/E7009/robot_arm/robot_arm_rl/scripts
  python3 train_ppo_orbit.py                       # 从阶段1开始自动课程
  python3 train_ppo_orbit.py --stage 3             # 固定阶段（关自动推进）
  python3 train_ppo_orbit.py --resume models/orbit/ppo_orbit_final --stage 4
  python3 train_ppo_orbit.py --n-envs 16 --timesteps 3000000

课程（详见 orbit_env.ArmOrbitEnv 头注释）：
  1 静态对准 → 2 小弧(10~30°) → 3 长弧(30°→arc_max, 60°→120°渐进) → 4 域随机化
  推进条件（近 100 轮成功率）：1→2: ≥0.9；2→3: ≥0.8；
  阶段3内 arc_max 每次 +10°（成功率 ≥0.7），到 120° 后 ≥0.7 → 阶段4。

并行环境数：规格建议 64~256 是 GPU 大规模并行仿真（Isaac Lab）的量级；
本环境是 CPU MuJoCo + SubprocVecEnv，进程数超过物理核（本机 24）只会
上下文切换拖慢采样，默认 16、留核给系统（可 --n-envs 覆盖）。

@copyright Copyright (c) 2026 eMeet
"""

import argparse
import math
import os
import sys

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPTS)

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.monitor import Monitor

from envs.orbit_env import ArmOrbitEnv, ARC_MAX_INIT, ARC_MAX_FULL

import torch as _torch
_DEVICE = 'cuda' if _torch.cuda.is_available() else 'cpu'
print(f'[GPU] device={_DEVICE}')

# ── 超参数 ────────────────────────────────────────────────────────────────────
TOTAL_TIMESTEPS = 3_000_000
N_ENVS          = 16
SAVE_FREQ       = 100_000      # 全局步（Checkpoint 回调内部会除以 n_envs）

PPO_KWARGS = dict(
    device        = _DEVICE,
    learning_rate = 3e-4,
    n_steps       = 256,        # 每环境每 rollout 步数 → 16 env = 4096 样本
    batch_size    = 1024,
    n_epochs      = 10,
    gamma         = 0.99,
    gae_lambda    = 0.95,
    clip_range    = 0.2,
    ent_coef      = 0.001,
    vf_coef       = 0.5,
    max_grad_norm = 0.5,
    policy_kwargs = dict(net_arch=[256, 256]),
    verbose       = 1,
)

MODEL_DIR = os.path.join(os.path.dirname(_SCRIPTS), 'models', 'orbit')
INFO_KEYS = ('success', 'progress_deg', 'goal_deg', 'center_err')


def _tb_log():
    try:
        import tensorboard  # noqa: F401
        return './logs/tb_orbit'
    except ImportError:
        return None


class CurriculumCallback(BaseCallback):
    """课程推进 + 任务指标记录（成功率/中心误差/环绕进度/弧度上限）。

    指标从 Monitor 的 episode info（INFO_KEYS）取，每个 rollout 结束时统计；
    推进经 VecEnv.env_method 广播到所有子进程环境（含 VecNormalize 透传）。
    """

    ADVANCE = {1: 0.9, 2: 0.8}      # stage → 推进所需成功率
    STAGE3_OK = 0.7                 # 阶段3：加弧度 / 进阶段4 的门槛
    # 推进冷却：课程切换时会清空 ep_info_buffer（滑窗 100），所以
    # len(buffer) 就是"当前课程下的新轮数"，攒够再判推进
    MIN_EPISODES = 80

    def __init__(self, start_stage: int, auto: bool, verbose=1):
        super().__init__(verbose)
        self.stage = start_stage
        self.auto = auto
        self.arc_max = ARC_MAX_FULL if start_stage >= 3 and not auto \
            else ARC_MAX_INIT

    def _on_training_start(self):
        self.training_env.env_method('set_stage', self.stage)
        self.training_env.env_method('set_arc_max', self.arc_max)

    def _on_step(self) -> bool:
        return True

    def _on_rollout_end(self):
        buf = list(self.model.ep_info_buffer)
        if not buf:
            return
        succ = float(np.mean([e.get('success', 0.0) for e in buf]))
        self.logger.record('orbit/success_rate', succ)
        self.logger.record('orbit/center_err',
                           float(np.mean([e.get('center_err', 0.) for e in buf])))
        self.logger.record('orbit/progress_deg',
                           float(np.mean([e.get('progress_deg', 0.) for e in buf])))
        self.logger.record('orbit/stage', self.stage)
        self.logger.record('orbit/arc_max_deg', math.degrees(self.arc_max))

        if not self.auto:
            return
        if len(buf) < self.MIN_EPISODES:
            return

        if self.stage in self.ADVANCE and succ >= self.ADVANCE[self.stage]:
            self.stage += 1
            self._change(f'成功率 {succ:.2f} → 进入阶段 {self.stage}')
        elif self.stage == 3 and succ >= self.STAGE3_OK:
            if self.arc_max < ARC_MAX_FULL - 1e-6:
                self.arc_max = min(self.arc_max + math.radians(10), ARC_MAX_FULL)
                self._change(f'成功率 {succ:.2f} → arc_max '
                             f'{math.degrees(self.arc_max):.0f}°')
            else:
                self.stage = 4
                self._change(f'成功率 {succ:.2f} @120° → 进入阶段 4（域随机化）')

    def _change(self, msg: str):
        print(f'[Curriculum] {msg}')
        self.training_env.env_method('set_stage', self.stage)
        self.training_env.env_method('set_arc_max', self.arc_max)
        # 课程切换改变了奖励构成与初始分布，清窗口避免旧数据误判下一次推进
        self.model.ep_info_buffer.clear()


def make_env(stage: int):
    def _init():
        return Monitor(ArmOrbitEnv(stage=stage), info_keywords=INFO_KEYS)
    return _init


def main():
    parser = argparse.ArgumentParser(description='环绕运镜 PPO 训练')
    parser.add_argument('--resume',    type=str, default=None,
                        help='继续训练的模型路径（不含 .zip）')
    parser.add_argument('--stage',     type=int, default=None, choices=[1, 2, 3, 4],
                        help='固定课程阶段（给出即关闭自动推进）')
    parser.add_argument('--timesteps', type=int, default=TOTAL_TIMESTEPS)
    parser.add_argument('--n-envs',    type=int, default=N_ENVS)
    parser.add_argument('--render',    action='store_true',
                        help='MuJoCo viewer（强制单环境，调试用）')
    args = parser.parse_args()

    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs('./logs/tb_orbit', exist_ok=True)

    auto = args.stage is None
    stage = args.stage if args.stage is not None else 1
    n_envs = 1 if args.render else args.n_envs

    if n_envs > 1:
        train_env = SubprocVecEnv([make_env(stage) for _ in range(n_envs)])
    else:
        train_env = make_vec_env(
            lambda: ArmOrbitEnv(stage=stage,
                                render_mode='human' if args.render else None),
            n_envs=1)
    train_env = VecNormalize(train_env, norm_obs=True, norm_reward=True,
                             clip_obs=10.0, clip_reward=10.0)

    curriculum_cb = CurriculumCallback(start_stage=stage, auto=auto)
    checkpoint_cb = CheckpointCallback(
        save_freq=max(SAVE_FREQ // n_envs, 1),
        save_path=MODEL_DIR, name_prefix='ppo_orbit')

    if args.resume:
        print(f'从 {args.resume} 继续训练...')
        base = args.resume.replace('.zip', '')
        vn = os.path.join(os.path.dirname(base), 'vec_normalize_final.pkl')
        if os.path.exists(vn):
            train_env = VecNormalize.load(vn, train_env.venv)
            print(f'  VecNormalize 已加载: {vn}')
        else:
            print('  ⚠️ 未找到 VecNormalize 统计量，观测分布将断裂')
        model = PPO.load(base, env=train_env, device=_DEVICE,
                         tensorboard_log=_tb_log())
    else:
        model = PPO('MlpPolicy', train_env, tensorboard_log=_tb_log(),
                    **PPO_KWARGS)

    print(f'开始训练  timesteps={args.timesteps}  n_envs={n_envs}  '
          f'stage={stage}{"(auto)" if auto else "(fixed)"}')
    try:
        model.learn(total_timesteps=args.timesteps,
                    callback=[curriculum_cb, checkpoint_cb],
                    reset_num_timesteps=args.resume is None,
                    progress_bar=True)
    except KeyboardInterrupt:
        print('\n训练中断，保存当前模型...')

    final = os.path.join(MODEL_DIR, 'ppo_orbit_final')
    model.save(final)
    train_env.save(os.path.join(MODEL_DIR, 'vec_normalize_final.pkl'))
    print(f'模型已保存 → {final}.zip（含 vec_normalize_final.pkl）')
    print(f'最终课程状态：stage={curriculum_cb.stage}  '
          f'arc_max={math.degrees(curriculum_cb.arc_max):.0f}°')
    train_env.close()


if __name__ == '__main__':
    main()
