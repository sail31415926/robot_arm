#!/usr/bin/env python3
"""离线绘图工具函数，供脚本和 Jupyter Notebook 调用。"""
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd


def load_joint_states(path: str) -> pd.DataFrame:
    """加载 joint_states_*.csv，添加相对时间列 'time'（秒）。"""
    df = pd.read_csv(path)
    if 'timestamp' in df.columns and len(df) > 0:
        df['time'] = df['timestamp'] - df['timestamp'].iloc[0]
    return df


def load_arm_status(path: str) -> pd.DataFrame:
    """加载 arm_status_*.csv，添加相对时间列 'time'（秒）。"""
    df = pd.read_csv(path)
    if 'timestamp' in df.columns and len(df) > 0:
        df['time'] = df['timestamp'] - df['timestamp'].iloc[0]
    return df


def plot_ee_velocity(df: pd.DataFrame,
                     ax: Optional[plt.Axes] = None,
                     title: str = '末端速度') -> plt.Figure:
    """绘制末端线速度与角速度曲线（来自 arm_status CSV）。"""
    standalone = ax is None
    if standalone:
        fig, (ax_lin, ax_ang) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        fig.suptitle(title)
    else:
        ax_lin = ax
        ax_ang = None
        fig = ax.figure

    t = df['time']
    for col, label in [('vx', 'vx'), ('vy', 'vy'), ('vz', 'vz')]:
        if col in df.columns:
            ax_lin.plot(t, df[col], label=f'{label} (m/s)')
    ax_lin.set_ylabel('linear velocity (m/s)')
    ax_lin.legend(fontsize=8)
    ax_lin.grid(True, linestyle='--', alpha=0.5)

    if ax_ang is not None:
        for col, label in [('wroll', 'ωroll'), ('wpitch', 'ωpitch'), ('wyaw', 'ωyaw')]:
            if col in df.columns:
                ax_ang.plot(t, df[col], label=f'{label} (°/s)')
        ax_ang.set_ylabel('angular velocity (°/s)')
        ax_ang.set_xlabel('time (s)')
        ax_ang.legend(fontsize=8)
        ax_ang.grid(True, linestyle='--', alpha=0.5)

    if standalone:
        fig.tight_layout()
    return fig


def plot_joint_torques(df: pd.DataFrame,
                       ax: Optional[plt.Axes] = None,
                       title: str = '关节力矩') -> plt.Figure:
    """绘制各关节力矩曲线（来自 joint_states CSV 的 *_eff 列）。"""
    eff_cols = [c for c in df.columns if c.endswith('_eff')]
    if not eff_cols:
        raise ValueError('未找到力矩列（期望列名以 _eff 结尾）')

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.set_title(title)
    else:
        fig = ax.figure

    for col in eff_cols:
        ax.plot(df['time'], df[col], label=col.replace('_eff', ''))
    ax.set_xlabel('time (s)')
    ax.set_ylabel('torque (N·m)')
    ax.legend(fontsize=8)
    ax.grid(True, linestyle='--', alpha=0.5)

    if standalone:
        fig.tight_layout()
    return fig


def plot_joint_velocities(df: pd.DataFrame,
                          ax: Optional[plt.Axes] = None,
                          title: str = '关节速度') -> plt.Figure:
    """绘制各关节速度曲线（来自 joint_states CSV 的 *_vel 列）。"""
    vel_cols = [c for c in df.columns if c.endswith('_vel')]
    if not vel_cols:
        raise ValueError('未找到速度列（期望列名以 _vel 结尾）')

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.set_title(title)
    else:
        fig = ax.figure

    for col in vel_cols:
        ax.plot(df['time'], df[col], label=col.replace('_vel', ''))
    ax.set_xlabel('time (s)')
    ax.set_ylabel('velocity (rad/s)')
    ax.legend(fontsize=8)
    ax.grid(True, linestyle='--', alpha=0.5)

    if standalone:
        fig.tight_layout()
    return fig


def plot_joint_positions(df: pd.DataFrame,
                         ax: Optional[plt.Axes] = None,
                         title: str = '关节位置') -> plt.Figure:
    """绘制各关节位置曲线（来自 joint_states CSV 的 *_pos 列）。"""
    pos_cols = [c for c in df.columns if c.endswith('_pos')]
    if not pos_cols:
        raise ValueError('未找到位置列（期望列名以 _pos 结尾）')

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.set_title(title)
    else:
        fig = ax.figure

    for col in pos_cols:
        ax.plot(df['time'], df[col], label=col.replace('_pos', ''))
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position (rad)')
    ax.legend(fontsize=8)
    ax.grid(True, linestyle='--', alpha=0.5)

    if standalone:
        fig.tight_layout()
    return fig


def plot_all_joint_data(df: pd.DataFrame, title: str = '关节综合数据') -> plt.Figure:
    """一图绘制关节力矩、速度、位置三组曲线（3 行子图）。"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title)
    plot_joint_torques(df, ax=axes[0], title='关节力矩')
    plot_joint_velocities(df, ax=axes[1], title='关节速度')
    plot_joint_positions(df, ax=axes[2], title='关节位置')
    axes[0].set_title('关节力矩')
    axes[1].set_title('关节速度')
    axes[2].set_title('关节位置')
    fig.tight_layout()
    return fig
