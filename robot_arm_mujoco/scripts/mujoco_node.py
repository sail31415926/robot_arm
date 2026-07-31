#!/usr/bin/env python3
"""
@file   mujoco_node.py
@brief  MuJoCo ↔ ROS2 bridge — 仿真物理引擎与 ROS2 控制接口桥接
@version 1.0
@date   2026-06-04

MuJoCo ↔ ROS2 bridge for eMeetArm.

Subscribes : /arm_controller/joint_trajectory          (trajectory_msgs/JointTrajectory)
             /servo_node/delta_twist_cmds              (geometry_msgs/TwistStamped)
Action Srv : /arm_controller/follow_joint_trajectory   (control_msgs/FollowJointTrajectory)
Publishes  : /joint_states                             (sensor_msgs/JointState)
Opens      : MuJoCo passive viewer window

FollowJointTrajectory action server 让 MoveIt 的 execute_trajectory
可以直接驱动 MuJoCo，不需要 ros2_control。

TwistStamped 速度控制（对接 MoveIt Servo / IBVS）：
  /servo_node/delta_twist_cmds → resolved-rate（DLS 雅可比反解）→ 关节位置积分
  优先级高于轨迹跟踪；看门狗 TWIST_WD 秒无指令自动切回轨迹模式。

用法：
  ros2 run robot_arm_node mujoco_node
  ros2 launch robot_arm_mujoco mujoco.launch.py

@copyright Copyright (c) 2026 eMeet
"""

import os
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory

import mujoco
import mujoco.viewer

from ament_index_python.packages import get_package_share_directory

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
MJCF_PATH   = os.path.join(
    get_package_share_directory('robot_arm_description'),
    'mujoco', 'eMeetArm.xml')

# ── Actuator PD gains (from eMeetArm.xml) for velocity feedforward ──────────
#     ctrl = target_pos + (kv/kp) * target_vel
#     → force = kp*(pos_err) - kv*(qvel - target_vel)  — D on vel error
_KP = np.array([500.0, 800.0, 300.0, 80.0, 50.0, 50.0])
_KV = np.array([60.0,  100.0, 40.0,  12.0, 8.0,  8.0])
_KV_OVER_KP = _KV / _KP

TWIST_WD  = 0.5    # s，TwistStamped 看门狗超时，超时切回轨迹模式
TWIST_DLS = 0.05   # DLS 阻尼系数，抑制奇异点附近关节速度爆炸


class MuJoCoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        self.model = mujoco.MjModel.from_xml_path(MJCF_PATH)
        self.data  = mujoco.MjData(self.model)

        self._jnt_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in JOINT_NAMES
        ]
        self._act_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'act_{n}')
            for n in JOINT_NAMES
        ]

        # --- shared state (sim thread writes, ROS timer reads) ---
        self._state_lock = threading.Lock()
        self._joint_pos  = np.zeros(6)
        self._joint_vel  = np.zeros(6)

        # --- shared control (callbacks write, sim thread reads) ---
        self._ctrl_lock        = threading.Lock()
        self._target_pos       = np.zeros(6)
        self._traj_points      = []
        self._traj_times       = []
        self._traj_joint_names = []
        self._traj_wall_start  = None

        # --- Cartesian velocity (TwistStamped, resolved-rate) ---
        self._twist_lock = threading.Lock()
        self._twist_vel  = np.zeros(6)   # [vx, vy, vz, wx, wy, wz]，世界系
        self._twist_time = 0.0           # time.monotonic() 时间戳
        self._ee_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'gimbal_tool0')
        self._dof_addrs  = [self.model.jnt_dofadr[jid] for jid in self._jnt_ids]

        # --- active FJT goal handle (for cancellation) ---
        self._fjtraj_lock        = threading.Lock()
        self._fjtraj_goal_handle = None

        # ROS interfaces
        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._traj_cb,
            10,
        )
        self.create_subscription(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            self._twist_cb,
            10,
        )
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self._publish_js)        # 50 Hz

        # Camera image publisher (matches Gazebo /camera namespace)
        self._cam_pub = self.create_publisher(ImageMsg, '/camera/camera_sensor/image_raw', 10)
        self.create_timer(0.033, self._publish_cam)      # ~30 Hz
        self._cam_lock = threading.Lock()
        self._cam_image = None  # latest rendered frame (numpy RGB)

        # FollowJointTrajectory action server (for MoveIt execute_trajectory)
        self._fjtraj_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._fjtraj_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()

        self.get_logger().info(f'MuJoCo node ready — model: {MJCF_PATH}')

    # ------------------------------------------------------------------ #
    #  Shared trajectory setter                                            #
    # ------------------------------------------------------------------ #

    def _set_trajectory(self, joint_names, points):
        with self._ctrl_lock:
            self._traj_joint_names = list(joint_names)
            self._traj_points      = list(points)
            self._traj_times       = [
                p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                for p in points
            ]
            self._traj_wall_start  = time.monotonic()

    # ------------------------------------------------------------------ #
    #  ROS topic callback                                                  #
    # ------------------------------------------------------------------ #

    def _traj_cb(self, msg: JointTrajectory):
        self._set_trajectory(msg.joint_names, msg.points)

    def _twist_cb(self, msg: TwistStamped):
        v = msg.twist
        vel = np.array([v.linear.x,  v.linear.y,  v.linear.z,
                        v.angular.x, v.angular.y, v.angular.z])
        with self._twist_lock:
            self._twist_vel  = vel
            self._twist_time = time.monotonic()

    # ------------------------------------------------------------------ #
    #  FollowJointTrajectory action server (MoveIt → MuJoCo)              #
    # ------------------------------------------------------------------ #

    def _fjtraj_execute(self, goal_handle):
        traj = goal_handle.request.trajectory

        with self._fjtraj_lock:
            self._fjtraj_goal_handle = goal_handle

        self._set_trajectory(traj.joint_names, traj.joint_trajectory.points)

        # Estimate execution duration from last trajectory point
        duration = 0.0
        if traj.joint_trajectory.points:
            last = traj.joint_trajectory.points[-1]
            duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9

        deadline = time.monotonic() + duration + 0.5
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self._fjtraj_lock:
                    self._fjtraj_goal_handle = None
                return FollowJointTrajectory.Result()
            time.sleep(0.05)

        goal_handle.succeed()
        with self._fjtraj_lock:
            self._fjtraj_goal_handle = None

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ------------------------------------------------------------------ #
    #  Joint state publisher                                               #
    # ------------------------------------------------------------------ #

    def _publish_js(self):
        with self._state_lock:
            pos = self._joint_pos.copy()
            vel = self._joint_vel.copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = JOINT_NAMES
        msg.position     = pos.tolist()
        msg.velocity     = vel.tolist()
        self._js_pub.publish(msg)

    def _publish_cam(self):
        """Publish latest rendered camera image as compressed JPEG."""
        with self._cam_lock:
            if self._cam_image is None:
                return
            img = self._cam_image.copy()

        msg = ImageMsg()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        msg.height          = img.shape[0]
        msg.width           = img.shape[1]
        msg.encoding        = 'rgb8'
        msg.is_bigendian    = False
        msg.step            = msg.width * 3
        msg.data            = img.tobytes()
        self._cam_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Simulation thread                                                   #
    # ------------------------------------------------------------------ #

    def _interpolate_target(self):
        """Return (target_pos[6], target_vel[6]) by linear interpolation."""
        with self._ctrl_lock:
            if not self._traj_points:
                return self._target_pos.copy(), np.zeros(6)

            elapsed = time.monotonic() - self._traj_wall_start
            times   = self._traj_times
            jnames  = self._traj_joint_names
            points  = self._traj_points

            n_pts = len(points)
            if n_pts == 1:
                # Hold single point, no velocity
                raw = np.array(points[0].positions)
                raw_vel = np.zeros(len(raw))
            elif elapsed >= times[-1]:
                raw = np.array(points[-1].positions)
                raw_vel = np.zeros(len(raw))
            elif elapsed <= times[0]:
                raw = np.array(points[0].positions)
                raw_vel = np.zeros(len(raw))
            else:
                # Linear interpolation between segment [i, i+1]
                raw = np.array(points[0].positions)
                raw_vel = np.zeros(len(raw))
                for i in range(n_pts - 1):
                    if times[i] <= elapsed < times[i + 1]:
                        seg_dt = times[i + 1] - times[i]
                        alpha = (elapsed - times[i]) / seg_dt
                        p_i = np.array(points[i].positions)
                        p_next = np.array(points[i + 1].positions)
                        raw = p_i + alpha * (p_next - p_i)
                        raw_vel = (p_next - p_i) / seg_dt  # constant per segment
                        break

            target_pos = self._target_pos.copy()
            target_vel = np.zeros(6)
            for ji, jname in enumerate(jnames):
                if jname in JOINT_NAMES:
                    idx = JOINT_NAMES.index(jname)
                    target_pos[idx] = raw[ji]
                    target_vel[idx] = raw_vel[ji]
            return target_pos, target_vel

    def _sim_loop(self):
        # Offscreen renderer — matches URDF camera 640×480 @ 30Hz
        cam_width, cam_height = 640, 480
        renderer = mujoco.Renderer(self.model, height=cam_height, width=cam_width)

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Matches Gazebo initial pose: -1.8 -2.2 1.6 (side-rear of arm)
            # MuJoCo azimuth is CW from +X → camera in -X/-Y quadrant = ~130°
            viewer.cam.azimuth   = 40
            viewer.cam.elevation = -25
            viewer.cam.distance  = 2.8
            viewer.cam.lookat[:] = [0.3, 0.0, 0.3]

            dt = self.model.opt.timestep
            cam_interval = max(1, int(0.5 + 1.0 / (30.0 * dt)))  # ~30 Hz

            step = 0
            while self._running and viewer.is_running():
                t0 = time.monotonic()

                # Twist 速度模式优先于轨迹模式
                with self._twist_lock:
                    twist_vel  = self._twist_vel.copy()
                    twist_time = self._twist_time

                if (time.monotonic() - twist_time < TWIST_WD
                        and np.linalg.norm(twist_vel) > 1e-6):
                    # Resolved-rate: Cartesian vel → joint vel → 积分到位置
                    jacp = np.zeros((3, self.model.nv))
                    jacr = np.zeros((3, self.model.nv))
                    mujoco.mj_jacBody(
                        self.model, self.data, jacp, jacr, self._ee_body_id)
                    J = np.vstack([jacp, jacr])[:, self._dof_addrs]

                    # DLS 伪逆（阻尼最小二乘，奇异点安全）
                    JJT = J @ J.T
                    J_dls = J.T @ np.linalg.inv(
                        JJT + TWIST_DLS * TWIST_DLS * np.eye(6))
                    q_dot = J_dls @ twist_vel

                    with self._state_lock:
                        cur_pos = self._joint_pos.copy()
                    target_pos = cur_pos + q_dot * dt
                    target_vel = q_dot

                    # 软关节限位
                    for i, jid in enumerate(self._jnt_ids):
                        if self.model.jnt_limited[jid]:
                            lo = self.model.jnt_range[jid, 0]
                            hi = self.model.jnt_range[jid, 1]
                            target_pos[i] = float(np.clip(target_pos[i], lo, hi))
                else:
                    target_pos, target_vel = self._interpolate_target()

                with self._ctrl_lock:
                    self._target_pos = target_pos

                # Velocity feedforward: ctrl = pos + (kv/kp)*vel
                # → force = kp*(pos_err) - kv*(qvel - target_vel)
                # D term acts on velocity error, not absolute velocity.
                for i, aid in enumerate(self._act_ids):
                    self.data.ctrl[aid] = (target_pos[i]
                                           + _KV_OVER_KP[i] * target_vel[i])

                mujoco.mj_step(self.model, self.data)

                with self._state_lock:
                    for i, jid in enumerate(self._jnt_ids):
                        qadr = self.model.jnt_qposadr[jid]
                        vadr = self.model.jnt_dofadr[jid]
                        self._joint_pos[i] = self.data.qpos[qadr]
                        self._joint_vel[i] = self.data.qvel[vadr]

                # Render camera at ~30 Hz — 使用末端执行器相机 ee_cam
                if step % cam_interval == 0:
                    renderer.update_scene(self.data, camera='ee_cam')
                    pixels = renderer.render()
                    with self._cam_lock:
                        self._cam_image = pixels.copy()

                viewer.sync()

                step += 1
                elapsed = time.monotonic() - t0
                if elapsed < dt:
                    time.sleep(dt - elapsed)

        self._running = False

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = MuJoCoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
