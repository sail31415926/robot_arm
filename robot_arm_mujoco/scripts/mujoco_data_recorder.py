#!/usr/bin/env python3
"""
@file   mujoco_data_recorder.py
@brief  MuJoCo 仿真数据采集节点 — 输出 IRIS 训练格式
@version 1.0
@date   2026-06-04

输出目录结构（与 IRIS iris_rosbag_reader.py 输出一致）：
  <output_dir>/<prefix>_episode_XXXX/
      rgb/
          {timestamp}.png   (224×224 RGB)
      robot/
          joint_states.csv  (pos_0 ~ pos_5)

之后直接用 IRIS 的 rgb_goal_absolute.py 预处理成 clips 供训练。


用法：
  ros2 run robot_arm_node mujoco_data_recorder [--output ~/dataset --prefix orbit]
  ros2 launch robot_arm_mujoco mujoco.launch.py controller:=sphere_orbit  # 先启动仿真

@copyright Copyright (c) 2026 eMeet
"""

import argparse
import csv
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber


class DataRecorder(Node):
    def __init__(self, output_dir: str, prefix: str):
        super().__init__('mujoco_data_recorder')

        self._output_dir = Path(output_dir)
        self._prefix     = prefix
        self._bridge     = CvBridge()

        self._recording  = False
        self._episode_id = self._find_next_episode_id()
        self._buf_rgb    = []   # (timestamp_str, np.ndarray HxWx3)
        self._buf_joints = []   # (timestamp_str, [j0..j5])
        self._lock       = threading.Lock()

        # 订阅相机和关节状态（近似时间同步，50ms 容差）
        img_sub   = Subscriber(self, Image,      '/camera/camera_sensor/image_raw')
        joint_sub = Subscriber(self, JointState, '/joint_states')
        self._sync = ApproximateTimeSynchronizer(
            [img_sub, joint_sub], queue_size=10, slop=0.05)
        self._sync.registerCallback(self._sync_cb)

        self.get_logger().info(
            f'DataRecorder ready — output: {self._output_dir}, prefix: {self._prefix}')
        self.get_logger().info(
            f'Next episode id: {self._episode_id:04d}')

    def _find_next_episode_id(self) -> int:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        existing = sorted(self._output_dir.glob(f'{self._prefix}_episode_*'))
        if not existing:
            return 0
        last = existing[-1].name
        try:
            return int(last.split('_episode_')[-1]) + 1
        except ValueError:
            return len(existing)

    def _sync_cb(self, img_msg: Image, joint_msg: JointState):
        if not self._recording:
            return

        ts = f'{img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec:09d}'

        # 转 BGR → RGB → resize 224×224
        bgr = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb_224 = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_AREA)

        # 关节角顺序：Joint1..Joint6
        joint_map = dict(zip(joint_msg.name, joint_msg.position))
        joints = [joint_map.get(f'Joint{i+1}', 0.0) for i in range(6)]

        with self._lock:
            self._buf_rgb.append((ts, rgb_224))
            self._buf_joints.append((ts, joints))

    def start_episode(self):
        with self._lock:
            self._buf_rgb.clear()
            self._buf_joints.clear()
        self._recording = True
        self.get_logger().info(
            f'▶ Recording episode {self._episode_id:04d} ...')

    def stop_episode(self) -> bool:
        self._recording = False
        with self._lock:
            rgb_buf    = list(self._buf_rgb)
            joint_buf  = list(self._buf_joints)

        if len(rgb_buf) < 30:
            self.get_logger().warn(
                f'Episode too short ({len(rgb_buf)} frames), discarded.')
            return False

        ep_dir = self._output_dir / f'{self._prefix}_episode_{self._episode_id:04d}'
        rgb_dir   = ep_dir / 'rgb'
        robot_dir = ep_dir / 'robot'
        rgb_dir.mkdir(parents=True, exist_ok=True)
        robot_dir.mkdir(parents=True, exist_ok=True)

        # 保存 RGB 帧
        for ts, img in rgb_buf:
            cv2.imwrite(
                str(rgb_dir / f'{ts}.png'),
                cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        # 保存关节角 CSV（IRIS 格式）
        with open(robot_dir / 'joint_states.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['pos_0', 'pos_1', 'pos_2', 'pos_3', 'pos_4', 'pos_5'])
            for _, joints in joint_buf:
                writer.writerow([f'{v:.6f}' for v in joints])

        self.get_logger().info(
            f'✔ Saved episode {self._episode_id:04d} — '
            f'{len(rgb_buf)} frames → {ep_dir}')
        self._episode_id += 1
        return True


def keyboard_thread(recorder: DataRecorder):
    """主线程：Enter 开始/结束 episode，q+Enter 退出。"""
    print('\n=== MuJoCo Data Recorder ===')
    print('  Enter  → 开始录制 / 结束并保存本段 episode')
    print('  q      → 退出\n')

    in_episode = False
    while True:
        try:
            key = input()
        except (EOFError, KeyboardInterrupt):
            break

        if key.strip().lower() == 'q':
            break

        if not in_episode:
            recorder.start_episode()
            in_episode = True
            print('  [录制中... 再按 Enter 结束]')
        else:
            recorder.stop_episode()
            in_episode = False
            print('  [已保存. 再按 Enter 开始下一段]')

    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='MuJoCo IRIS 数据采集器')
    parser.add_argument('--output', default=os.path.expanduser('~/iris_dataset'),
                        help='输出根目录 (default: ~/iris_dataset)')
    parser.add_argument('--prefix', default='mujoco',
                        help='episode 名称前缀 (default: mujoco)')
    args = parser.parse_args()

    rclpy.init()
    node = DataRecorder(args.output, args.prefix)

    kb_thread = threading.Thread(target=keyboard_thread, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
