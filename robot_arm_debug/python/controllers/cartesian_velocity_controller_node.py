#!/usr/bin/env python3
"""
@file   cartesian_velocity_controller_node.py
@brief  eMeetArm 笛卡尔速度控制 node（IBVS 接口 / 点动，无 GUI，可被 GUI import 或 headless 运行）
@version 1.0
@date   2026-06-09

100Hz 速度流节点，不依赖任何 GUI 框架。两种速度来源，统一输出：
           A. 外部 IBVS：/arm_vel_cmd (TwistStamped) + 看门狗 + 限幅
           B. set_jog_vel()：点动接口（GUI 按钮调用）
                ↓
           /servo_node/delta_twist_cmds (100Hz)
         下游：MuJoCo 直驱（内部 DLS）或 MoveIt Servo。
         状态/位姿通过 gui_q 队列上报（headless 运行时传空队列即可）。

被 cartesian_velocity_gui.py（tkinter 调试 GUI）import 使用；也可独立运行（IBVS 桥）：
  ros2 run robot_arm_node cartesian_velocity_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer

from arm_utils import rpy_to_quat, quat_to_rpy

# ── 常量 ──────────────────────────────────────────────────────────────────────
EEF_LINK   = 'gimbal_tool0'   # 2026-07-28 云台换 V2：MoveIt 规划组 tip
BASE_FRAME = 'arm_base_link'

JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
# 2026-07-28 云台换 V2 后重算：末端 = gimbal_tool0（SRDF 规划组 tip）。
# 取值来自位形 [0, 1.2, -1.2, 0, 0, 0] 的 FK —— J2=-J3 时前臂保持水平，
# 末端姿态恰为中性 (0, 0.29°, 0)，故姿态角取 0；老值 (roll=90°, pitch=10°)
# 是 V1 云台时代的约定，在新末端坐标系下已无意义。
READY_POSE     = dict(x=0.315, y=-0.036, z=0.548, roll=0.0, pitch=0.0, yaw=0.0)
# READY_POSE 对应的关节角（IK 不可用时的回退）——与上面 READY_POSE 同一位形，
# 2026-07-28 云台换 V2 后一并重取：J2=-J3 保持前臂水平、末端姿态中性。
READY_JOINTS   = [0.0, 1.2, -1.2, 0.0, 0.0, 0.0]

TWIST_TOPIC         = '/servo_node/delta_twist_cmds'
TRAJ_TOPIC          = '/arm_controller/joint_trajectory'
VEL_CMD_TOPIC       = '/arm_vel_cmd'
START_SERVO_SERVICE = '/servo_node/start_servo'

STREAM_DT  = 0.01   # s，100Hz 发布周期
WD_TIMEOUT = 1.0    # s，IBVS 指令看门狗超时（ibvs_controller 200ms 心跳，留足余量）

MAX_V_LIN = 0.30    # m/s   线速度上限（接收外部指令时裁剪）
MAX_V_ANG = 1.00    # rad/s 角速度上限

DEFAULT_JOG_LIN = 0.05   # m/s   GUI 点动默认线速度
DEFAULT_JOG_ANG = 0.20   # rad/s GUI 点动默认角速度


def clamp(v, limit):
    return max(-limit, min(limit, v))


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianVelocityControllerNode(Node):
    """
    100Hz 速度流节点。

    接受两种速度来源：
      - set_jog_vel()：GUI 按钮点动调用
      - /arm_vel_cmd 订阅：外部 IBVS 控制器输入（带看门狗）

    统一输出到 /servo_node/delta_twist_cmds（MoveIt Servo）。
    """

    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_velocity_controller_node')
        # use_sim_time 由 launch 按后端传入（gazebo/mujoco=true，real=false）。
        # 切勿在此硬编码覆盖：实物无 /clock 时 use_sim_time=true 会让所有
        # ROS 定时器永不触发（位姿面板卡 '--' 的教训）。
        self._q = gui_q

        self._lock            = threading.Lock()
        self._vel             = [0.0] * 6   # [vx, vy, vz, wx, wy, wz]
        self._cmd_time        = 0.0
        self._source          = 'idle'      # 'jog' | 'ibvs' | 'idle'
        self._suppress_stream = False       # 发轨迹时暂停速度流，避免 servo 冲突

        self._twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self._traj_pub  = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        self.create_subscription(
            TwistStamped, VEL_CMD_TOPIC, self._on_vel_cmd, 10)

        self._start_cli     = self.create_client(Trigger, START_SERVO_SERVICE)
        self._stop_cli      = self.create_client(Trigger, '/servo_node/stop_servo')
        self._servo_started = False
        self.create_timer(0.5, self._try_start_servo)

        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self._joint_positions  = [0.0] * 6
        self._joint_velocities = [0.0] * 6
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        self.create_timer(STREAM_DT, self._stream_step)

        self.get_logger().info(
            f'CartesianVelocityControllerNode 已启动 | '
            f'IBVS输入={VEL_CMD_TOPIC} | Servo输出={TWIST_TOPIC} | '
            f'看门狗={WD_TIMEOUT}s')

    # ── MoveIt Servo 启动（Gazebo/真机模式；MuJoCo 模式下服务不存在，跳过即可）──
    def _try_start_servo(self):
        if self._servo_started:
            return
        if not self._start_cli.service_is_ready():
            # MuJoCo 模式下 servo_node 不存在，首次检测后停止轮询并标记就绪
            self._no_servo_count = getattr(self, '_no_servo_count', 0) + 1
            if self._no_servo_count == 4:   # ~2s 后确认服务不存在
                self._servo_started = True  # 停止轮询
                self._q.put(('status', '就绪（MuJoCo 直驱模式，无需 servo_node）'))
            return
        self._start_cli.call_async(Trigger.Request()).add_done_callback(
            self._on_start_servo)
        self._servo_started = True

    def _on_start_servo(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('✓ MoveIt Servo 已启动')
                self._q.put(('status', '✓ MoveIt Servo 就绪'))
            else:
                self.get_logger().warn(f'⚠ start_servo 失败: {resp.message}')
                self._servo_started = False
        except Exception as e:
            self.get_logger().warn(f'⚠ start_servo 异常: {e}')
            self._servo_started = False

    # ── 外部 IBVS 速度输入 ────────────────────────────────────────────────────
    def _on_vel_cmd(self, msg: TwistStamped):
        vx = clamp(msg.twist.linear.x,  MAX_V_LIN)
        vy = clamp(msg.twist.linear.y,  MAX_V_LIN)
        vz = clamp(msg.twist.linear.z,  MAX_V_LIN)
        wx = clamp(msg.twist.angular.x, MAX_V_ANG)
        wy = clamp(msg.twist.angular.y, MAX_V_ANG)
        wz = clamp(msg.twist.angular.z, MAX_V_ANG)
        with self._lock:
            self._vel      = [vx, vy, vz, wx, wy, wz]
            self._cmd_time = time.monotonic()
            self._source   = 'ibvs'

    # ── GUI 点动接口（由 GUI 线程调用）──────────────────────────────────────
    def set_jog_vel(self, vx=0., vy=0., vz=0., wx=0., wy=0., wz=0.):
        with self._lock:
            self._vel      = [vx, vy, vz, wx, wy, wz]
            self._cmd_time = time.monotonic()
            self._source   = 'jog'

    def stop(self):
        with self._lock:
            self._vel    = [0.0] * 6
            self._source = 'idle'

    def _on_joint_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        vels = []
        for i, name in enumerate(JOINT_NAMES):
            idx = name_to_idx.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_positions[i] = msg.position[idx]
            vel = 0.0
            if idx is not None and idx < len(msg.velocity):
                vel = msg.velocity[idx]
                self._joint_velocities[i] = vel
            vels.append(vel)
        self._q.put(('jvel', *vels))

    def go_ready(self):
        self.stop()
        self._start_suppress()
        if not self._ik_cli.service_is_ready():
            # MuJoCo 直驱模式：无 MoveIt，回退到预计算关节角
            threading.Timer(1.2, lambda: self._send_joint_traj(
                READY_JOINTS, duration_sec=3)).start()
            threading.Timer(5.5, self._end_suppress).start()
            self._q.put(('status', '移动至预备位置（关节空间，无 IK）...'))
            return
        p = READY_POSE
        qx, qy, qz, qw = rpy_to_quat(
            math.radians(p['roll']), math.radians(p['pitch']), math.radians(p['yaw']))

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id        = BASE_FRAME
        pose_stamped.header.stamp           = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x        = p['x']
        pose_stamped.pose.position.y        = p['y']
        pose_stamped.pose.position.z        = p['z']
        pose_stamped.pose.orientation.x     = qx
        pose_stamped.pose.orientation.y     = qy
        pose_stamped.pose.orientation.z     = qz
        pose_stamped.pose.orientation.w     = qw

        rs = RobotState()
        rs.joint_state.name     = JOINT_NAMES
        rs.joint_state.position = list(self._joint_positions)

        req = GetPositionIK.Request()
        req.ik_request.group_name       = PLANNING_GROUP
        req.ik_request.ik_link_name     = EEF_LINK
        req.ik_request.pose_stamped     = pose_stamped
        req.ik_request.robot_state      = rs
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec      = 0
        req.ik_request.timeout.nanosec  = 50_000_000

        self._ik_cli.call_async(req).add_done_callback(self._on_ready_ik)
        self._q.put(('status', '求解预备位置 IK...（速度流已暂停）'))

    def _on_ready_ik(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self._q.put(('status', f'✗ IK 调用异常: {e}'))
            return
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self._q.put(('status', '✗ IK 无解（目标超出工作空间？）'))
            return
        name_to_pos = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
        positions = [name_to_pos.get(n, 0.0) for n in JOINT_NAMES]
        threading.Timer(1.2, lambda: self._send_joint_traj(
            positions, duration_sec=3)).start()
        threading.Timer(5.5, self._end_suppress).start()
        self._q.put(('status', '移动至预备位置...'))

    def go_home(self):
        self.stop()
        self._start_suppress()
        threading.Timer(1.2, lambda: self._send_joint_traj(
            [0.0] * 6, duration_sec=3)).start()
        threading.Timer(5.5, self._end_suppress).start()
        self._q.put(('status', '回零中...（速度流已暂停）'))

    def _send_joint_traj(self, positions, duration_sec: int):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    # ── 速度流暂停（发轨迹时使用）────────────────────────────────────────────
    def _start_suppress(self):
        with self._lock:
            self._suppress_stream = True
        # 显式停止 servo，防止 servo 最后一帧轨迹覆盖我们的目标轨迹
        if self._stop_cli.service_is_ready():
            self._stop_cli.call_async(Trigger.Request())

    def _end_suppress(self):
        with self._lock:
            self._suppress_stream = False
        # 重新启动 servo
        if self._start_cli.service_is_ready():
            self._servo_started = False   # 允许 _try_start_servo 重新调用
            self._start_cli.call_async(Trigger.Request())

    # ── 100Hz 发布步骤 ────────────────────────────────────────────────────────
    def _stream_step(self):
        with self._lock:
            suppress = self._suppress_stream
            vel      = list(self._vel)
            source   = self._source
            t        = self._cmd_time

        # 暂停中：不向 servo 发命令，让 servo 的命令超时机制自然停止输出
        if suppress:
            return

        if source == 'ibvs' and (time.monotonic() - t) > WD_TIMEOUT:
            vel = [0.0] * 6
            with self._lock:
                self._vel    = vel
                self._source = 'idle'
            self._q.put(('status', '⚠ IBVS 指令超时，已停止'))

        self._publish_twist(*vel)

    def _publish_twist(self, vx, vy, vz, wx, wy, wz):
        msg = TwistStamped()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self._twist_pub.publish(msg)

    # ── TF → GUI 位姿 ────────────────────────────────────────────────────────
    def _pub_pose(self):
        pose = self.get_ee_pose()
        if pose is None:
            return
        x, y, z, r, p, yw = pose
        self._q.put(('pose', x, y, z,
                     math.degrees(r), math.degrees(p), math.degrees(yw)))

    def get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, yw = quat_to_rpy(q.x, q.y, q.z, q.w)
            return tr.x, tr.y, tr.z, r, p, yw
        except Exception:
            return None


def main():
    rclpy.init()
    node     = CartesianVelocityControllerNode(queue.Queue())
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
