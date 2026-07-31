#!/usr/bin/env python3
"""
@file   cartesian_realtime_ik_controller_node.py
@brief  eMeetArm 末端笛卡尔实时 IK 控制 node（无 GUI，可被 GUI import 或 headless 运行）
@version 1.1
@date   2026-06-09

实时 IK 直发关节指令，不依赖任何 GUI 框架：
           目标位姿 → pick_ik（/compute_ik，local 模式）→ /arm_controller/joint_trajectory
         无碰撞检测、无路径规划，末端走关节插值曲线，响应延迟 ~50ms。
         状态/位姿通过 gui_q 队列上报（headless 运行时传空队列即可）。

被 cartesian_realtime_ik_gui.py（tkinter 调试 GUI）import 使用；也可独立运行：
  ros2 run robot_arm_node cartesian_realtime_ik_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer

from arm_utils import rpy_to_quat, quat_to_rpy

# ── 常量 ──────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'gimbal_tool0'   # 2026-07-28 云台换 V2：MoveIt 规划组 tip
BASE_FRAME     = 'arm_base_link'

TRAJ_DURATION  = 0.3    # s，控制器平滑插值窗口，越长运动越平滑
DEBOUNCE_MS    = 10     # ms，滑块防抖延迟，越短响应越及时

# 2026-07-28 云台换 V2 后重算：末端 = gimbal_tool0（SRDF 规划组 tip）。
# 取值来自位形 [0, 1.2, -1.2, 0, 0, 0] 的 FK —— J2=-J3 时前臂保持水平，
# 末端姿态恰为中性 (0, 0.29°, 0)，故姿态角取 0；老值 (roll=90°, pitch=10°)
# 是 V1 云台时代的约定，在新末端坐标系下已无意义。
READY_POSE = dict(x=0.315, y=-0.036, z=0.548, roll=0.0, pitch=0.0, yaw=0.0)

PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianRealtimeIkControllerNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_realtime_ik_controller_node')
        # use_sim_time 由 launch 按后端传入（gazebo/mujoco=true，real=false）。
        # 切勿在此硬编码覆盖：实物无 /clock 时 use_sim_time=true 会让所有
        # ROS 定时器永不触发（位姿面板卡 '--' 的教训）。
        self._q   = gui_q
        self._seq = 0  # 请求序号，过时响应自动丢弃

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        # 保存当前关节状态，作为 IK 初始解参考（提高连续性）
        self._joint_positions = [0.0] * 6
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

    # ── 关节状态回调 ──────────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(JOINT_NAMES):
            idx = name_to_idx.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_positions[i] = msg.position[idx]

    # ── TF → GUI 末端位姿 ─────────────────────────────────────────────────────
    def _pub_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
            self._q.put(('pose', tr.x, tr.y, tr.z,
                         math.degrees(r), math.degrees(p), math.degrees(y)))
        except Exception:
            pass

    def get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
            return (tr.x, tr.y, tr.z,
                    math.degrees(r), math.degrees(p), math.degrees(y))
        except Exception:
            return None

    # ── IK 请求（异步）───────────────────────────────────────────────────────
    def send_ik(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        if not self._ik_cli.service_is_ready():
            self._q.put(('status', '✗ /compute_ik 服务未就绪，请确认 MoveIt 已启动'))
            return

        self._seq += 1
        seq = self._seq

        qx, qy, qz, qw = rpy_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_FRAME
        pose_stamped.header.stamp    = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x    = x
        pose_stamped.pose.position.y    = y
        pose_stamped.pose.position.z    = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        # 以当前关节状态为初始解，提高 IK 成功率和解的连续性
        rs = RobotState()
        rs.joint_state.name     = JOINT_NAMES
        rs.joint_state.position = list(self._joint_positions)

        req = GetPositionIK.Request()
        req.ik_request.group_name      = PLANNING_GROUP
        req.ik_request.ik_link_name    = EEF_LINK
        req.ik_request.pose_stamped    = pose_stamped
        req.ik_request.robot_state      = rs
        req.ik_request.avoid_collisions = False       # 实时控制不做碰撞检测，提升速度
        req.ik_request.timeout.sec      = 0
        req.ik_request.timeout.nanosec  = 30_000_000  # local 模式 30ms 足够

        self._ik_cli.call_async(req).add_done_callback(
            lambda f, s=seq: self._on_ik_result(f, s))

    # ── IK 结果回调 → 发布关节轨迹 ───────────────────────────────────────────
    def _on_ik_result(self, future, seq):
        if seq != self._seq:
            return  # 过时响应，丢弃

        try:
            resp = future.result()
        except Exception as e:
            self._q.put(('status', f'✗ IK 调用异常: {e}'))
            return

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self._q.put(('status', f'✗ IK 无解（目标超出工作空间或奇异点附近）'))
            return

        name_to_pos = dict(zip(
            resp.solution.joint_state.name,
            resp.solution.joint_state.position))
        positions = [name_to_pos.get(n, 0.0) for n in JOINT_NAMES]

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=int(TRAJ_DURATION * 1e9))
        msg.points = [pt]
        self._traj_pub.publish(msg)

        self._q.put(('status', '● 实时运行中'))

    def go_ready(self):
        p = READY_POSE
        self.send_ik(p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw'])

    # ── 回零（直发关节零位）──────────────────────────────────────────────────
    def go_home(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * 6
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self._q.put(('status', '回零中...'))


def main():
    rclpy.init()
    node     = CartesianRealtimeIkControllerNode(queue.Queue())
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
