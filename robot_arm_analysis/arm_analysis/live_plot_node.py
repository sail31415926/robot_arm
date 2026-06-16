#!/usr/bin/env python3
"""ROS 2 node: 实时绘制末端速度与关节力矩/速度曲线（滑动窗口）。"""
import collections
import threading
from typing import Optional

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robot_arm_interfaces.msg import ArmStatus

_DEFAULT_WINDOW_SEC = 30.0
_UPDATE_INTERVAL_MS = 200


class LivePlotNode(Node):
    def __init__(self):
        super().__init__('arm_live_plot')
        self.declare_parameter('window_sec', _DEFAULT_WINDOW_SEC)
        window = self.get_parameter('window_sec').value

        # Estimate max samples: joint_states up to 200 Hz, arm_status 10 Hz
        maxlen = int(window * 200)
        self._t_js: collections.deque = collections.deque(maxlen=maxlen)
        self._js: dict = {}  # joint_name → {'pos': deque, 'vel': deque, 'eff': deque}

        self._t_st: collections.deque = collections.deque(maxlen=maxlen)
        self._vx: collections.deque = collections.deque(maxlen=maxlen)
        self._vy: collections.deque = collections.deque(maxlen=maxlen)
        self._vz: collections.deque = collections.deque(maxlen=maxlen)

        self._lock = threading.Lock()

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 100)
        self.create_subscription(ArmStatus, '/robot_arm/arm_status', self._on_arm_status, 10)

    def _on_joint_state(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._t_js.append(t)
            maxlen = self._t_js.maxlen
            for i, name in enumerate(msg.name):
                if name not in self._js:
                    self._js[name] = {
                        'pos': collections.deque(maxlen=maxlen),
                        'vel': collections.deque(maxlen=maxlen),
                        'eff': collections.deque(maxlen=maxlen),
                    }
                d = self._js[name]
                d['pos'].append(msg.position[i] if i < len(msg.position) else 0.0)
                d['vel'].append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                d['eff'].append(msg.effort[i] if i < len(msg.effort) else 0.0)

    def _on_arm_status(self, msg: ArmStatus):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._t_st.append(t)
            self._vx.append(msg.arm_twist.vx)
            self._vy.append(msg.arm_twist.vy)
            self._vz.append(msg.arm_twist.vz)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                't_st': list(self._t_st),
                'vx': list(self._vx),
                'vy': list(self._vy),
                'vz': list(self._vz),
                't_js': list(self._t_js),
                'js': {n: {k: list(v) for k, v in d.items()} for n, d in self._js.items()},
            }


def _make_figure():
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=False)
    fig.suptitle('机械臂实时数据 (Live)')
    return fig, axes


def main():
    rclpy.init()
    node = LivePlotNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = _make_figure()

    def _update(_frame):
        data = node.snapshot()
        for ax in axes:
            ax.cla()
            ax.grid(True, linestyle='--', alpha=0.5)

        # ── 末端线速度 ──
        ax_vel, ax_eff, ax_jvel = axes
        ax_vel.set_title('末端线速度 (m/s)')
        ax_vel.set_ylabel('velocity (m/s)')
        if data['t_st']:
            t0 = data['t_st'][0]
            t = [s - t0 for s in data['t_st']]
            ax_vel.plot(t, data['vx'], label='vx')
            ax_vel.plot(t, data['vy'], label='vy')
            ax_vel.plot(t, data['vz'], label='vz')
            ax_vel.legend(loc='upper left', fontsize=8)

        # ── 关节力矩 & 速度 ──
        ax_eff.set_title('关节力矩 (N·m)')
        ax_eff.set_ylabel('torque (N·m)')
        ax_jvel.set_title('关节速度 (rad/s)')
        ax_jvel.set_ylabel('vel (rad/s)')
        ax_jvel.set_xlabel('time (s)')
        if data['t_js'] and data['js']:
            t0 = data['t_js'][0]
            t = [s - t0 for s in data['t_js']]
            for name, d in data['js'].items():
                ax_eff.plot(t, d['eff'], label=name)
                ax_jvel.plot(t, d['vel'], label=name)
            ax_eff.legend(loc='upper left', fontsize=8)
            ax_jvel.legend(loc='upper left', fontsize=8)

        fig.tight_layout(pad=2.5)

    ani = animation.FuncAnimation(fig, _update, interval=_UPDATE_INTERVAL_MS,
                                   cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()
