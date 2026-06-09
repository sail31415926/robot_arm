#!/usr/bin/env python3
"""
@file   pose_command_debug.py
@brief  ArmMoveToPose action 执行器（无 GUI）
@version 2.0
@date   2026-06-09

提供 action /robot_arm/move_to_pose 的服务端，收到目标后执行对应预设运动：
  STOWED   → 所有关节归零（关节空间直接下发，不经过 IK）
  OBSERVE  → POSE_OBSERVE（下方修改）
  SHOOTING → 目标中的 target_pose_x/y/z + target_pose_roll/pitch/yaw

执行过程中通过 feedback 上报进度，完成后返回 result（success / exit_reason），
支持 cancel。

用法：
  ros2 run robot_arm_node pose_command_debug
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=pose_command_debug

@copyright Copyright (c) 2026 eMeet
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer

try:
    from robot_arm_interfaces.action import ArmMoveToPose
except ImportError as e:
    raise SystemExit('✗ 未找到 robot_arm_interfaces.action，请先构建：colcon build --packages-select robot_arm_interfaces') from e

try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
except ImportError as e:
    raise SystemExit('✗ 未找到 ruckig 库：pip install ruckig') from e


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║              ★  可编辑配置区  ★  标定后在此处修改预设位置               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── STOWED（收纳位） ──────────────────────────────────────────────────────────
# 直接下发关节空间全零轨迹，不经过 IK，不存在无解风险。
# 按速度档指定运动时长（秒）：时间越短冲击越大，建议 FAST ≥ 1.5s。
STOWED_DURATION = {
    0: 5.0,   # SPEED_SLOW   — 缓慢归零，适合从任意姿态安全收纳
    1: 3.0,   # SPEED_NORMAL — 正常归零
    2: 1.5,   # SPEED_FAST   — 快速归零
}

# ── OBSERVE（观察位） ─────────────────────────────────────────────────────────
# 笛卡尔目标位姿，坐标系：base_link，朝向单位：度。
# 通过 Ruckig + IK 执行，需确保该位置在工作空间内有解。
POSE_OBSERVE = dict(
    x        =  0.2,   # 前向距离（m）
    y        =  0.00,   # 侧向距离（m），正值向左
    z        =  0.80,   # 高度（m）
    roll_deg =  90.0,   # 末端横滚角（°）
    pitch_deg=   0.0,   # 末端俯仰角（°）
    yaw_deg  =   0.0,   # 末端偏航角（°）
)

# 注：SHOOTING 的目标 XYZ 与朝向 RPY 全部由 action goal 携带，无需在此配置。

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                        以下参数一般无需修改                              ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── ROS / MoveIt 固定常量 ─────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'          # MoveIt 规划组名
EEF_LINK       = 'tool0'        # 末端执行器 link
BASE_FRAME     = 'base_link'    # 参考坐标系
ACTION_NAME    = '/robot_arm/move_to_pose'  # action 名称

# ── 错误码（对应 ArmStatus.ERR_*）─────────────────────────────────────────────
ERR_NONE    = 0
ERR_DRIVER  = 2
ERR_TIMEOUT = 3

# ── Ruckig 轨迹规划参数 ───────────────────────────────────────────────────────
STREAM_DT    = 0.01    # 规划步长（s），即轨迹点间隔，100 Hz
IK_TIMEOUT_S = 0.05   # 单次 IK 超时（s）

# 速度档 → Ruckig 限制参数（v=速度, a=加速度, j=加加速度）
# pos 对应平移轴（m/s, m/s², m/s³），ori 对应姿态轴（rad/s 等）
SPEED_PARAMS = {
    0: dict(v_pos=0.03, a_pos=0.06, j_pos=0.50,    # SPEED_SLOW   — 缓慢平稳
            v_ori=0.06, a_ori=0.12, j_ori=1.00),
    1: dict(v_pos=0.08, a_pos=0.15, j_pos=1.50,    # SPEED_NORMAL — 正常
            v_ori=0.15, a_ori=0.30, j_ori=3.00),
    2: dict(v_pos=0.15, a_pos=0.30, j_pos=3.00,    # SPEED_FAST   — 快速
            v_ori=0.30, a_ori=0.60, j_ori=6.00),
}

# ── 日志显示用名称映射 ────────────────────────────────────────────────────────
STATE_NAMES = {0: 'STOWED', 1: 'OBSERVE', 2: 'SHOOTING'}
SPEED_NAMES = {0: 'SLOW', 1: 'NORMAL', 2: 'FAST'}


# ── 数学工具 ──────────────────────────────────────────────────────────────────
def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cy = math.cos(yaw*0.5);  sy = math.sin(yaw*0.5)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def quat_to_rpy(x, y, z, w):
    roll  = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
    yaw   = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n) if n > 1e-12 else (0., 0., 0., 1.)


def quat_dot(q1, q2):
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]


def quat_slerp(q0, q1, t):
    q0, q1 = quat_normalize(q0), quat_normalize(q1)
    d = quat_dot(q0, q1)
    if d < 0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3]); d = -d
    if d > 0.9995:
        return quat_normalize((q0[0]+t*(q1[0]-q0[0]), q0[1]+t*(q1[1]-q0[1]),
                               q0[2]+t*(q1[2]-q0[2]), q0[3]+t*(q1[3]-q0[3])))
    th0  = math.acos(max(-1., min(1., d)))
    sin0 = math.sin(th0)
    th   = th0 * t
    s0   = math.cos(th) - d * math.sin(th) / sin0
    s1   = math.sin(th) / sin0
    return (s0*q0[0]+s1*q1[0], s0*q0[1]+s1*q1[1],
            s0*q0[2]+s1*q1[2], s0*q0[3]+s1*q1[3])


# ── 节点 ──────────────────────────────────────────────────────────────────────
class PoseCommandExecutor(Node):

    def __init__(self):
        super().__init__('pose_command_executor',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])

        cb = ReentrantCallbackGroup()

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._ik_cli = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=cb)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._joint_pos = [0.0] * 6
        self.create_subscription(
            JointState, '/joint_states', self._on_js, 10, callback_group=cb)

        self._stop_req = False
        self._active   = False

        self._action_server = ActionServer(
            self, ArmMoveToPose, ACTION_NAME,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb)

        self.get_logger().info(f'PoseCommandExecutor 就绪，action: {ACTION_NAME}')
        self.get_logger().info(
            '  STOWED  → 所有关节归零（关节空间直接下发，不经过 IK）')
        self.get_logger().info(
            f'  OBSERVE → ({POSE_OBSERVE["x"]:.2f}, {POSE_OBSERVE["y"]:.2f}, '
            f'{POSE_OBSERVE["z"]:.2f})')
        self.get_logger().info('  SHOOTING → XYZ + RPY 由 goal 携带')

    # ── action 回调 ────────────────────────────────────────────────────────────
    def _on_goal(self, goal_request):
        if self._active:
            self.get_logger().warn('执行中，拒绝新目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._active   = True
        self._stop_req = False
        try:
            goal  = goal_handle.request
            state = goal.target_pose_state
            speed = goal.transition_speed
            self.get_logger().info(
                f'收到目标: {STATE_NAMES.get(state, state)}  '
                f'速度={SPEED_NAMES.get(speed, speed)}')

            # ── 规划 + 下发 ──────────────────────────────────────────────────
            if state == ArmMoveToPose.Goal.POSE_STATE_STOWED:
                t_traj = self._go_stowed(speed)
                ok, exit_reason, err = True, 'reached', ERR_NONE
            elif state in (ArmMoveToPose.Goal.POSE_STATE_OBSERVE,
                           ArmMoveToPose.Goal.POSE_STATE_SHOOTING):
                if state == ArmMoveToPose.Goal.POSE_STATE_OBSERVE:
                    p = POSE_OBSERVE
                    target = (p['x'], p['y'], p['z'],
                              p['roll_deg'], p['pitch_deg'], p['yaw_deg'])
                else:
                    tp = goal.target_pose
                    target = (tp.x, tp.y, tp.z, tp.roll, tp.pitch, tp.yaw)
                ok, t_traj, exit_reason, err = self._plan_ptp(target, speed)
            else:
                self.get_logger().error(f'未知 target_pose_state={state}')
                goal_handle.abort()
                return self._make_result(False, 'error', ERR_DRIVER)

            if not ok:
                goal_handle.abort()
                return self._make_result(False, exit_reason, err)

            # ── 等待执行完成（发 feedback / 支持取消）───────────────────────
            exit_reason = self._wait_with_feedback(goal_handle, t_traj)
            if exit_reason == 'cancelled':
                goal_handle.canceled()
                return self._make_result(False, 'cancelled', ERR_NONE)
            if exit_reason == 'timeout':
                goal_handle.abort()
                return self._make_result(False, 'timeout', ERR_TIMEOUT)

            goal_handle.succeed()
            return self._make_result(True, 'reached', ERR_NONE)
        finally:
            self._active = False

    # ── 执行等待 + 反馈 ────────────────────────────────────────────────────────
    def _wait_with_feedback(self, goal_handle, t_traj):
        fb = ArmMoveToPose.Feedback()
        t0_sim  = self.get_clock().now()
        wall_deadline = time.monotonic() + t_traj * 4.0 + 10.0
        while True:
            elapsed = (self.get_clock().now() - t0_sim).nanoseconds / 1e9
            if goal_handle.is_cancel_requested:
                self._stop_req = True
                self._hold_current()
                return 'cancelled'
            if elapsed >= t_traj:
                break
            if time.monotonic() > wall_deadline:
                self.get_logger().error('✗ 执行超时（墙钟）')
                return 'timeout'
            fb.progress_percent = max(0., min(100., elapsed / t_traj * 100.)) \
                if t_traj > 1e-6 else 100.0
            self._fill_pose(fb.current_pose, self._get_ee_pose())
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        fb.progress_percent = 100.0
        self._fill_pose(fb.current_pose, self._get_ee_pose())
        goal_handle.publish_feedback(fb)
        return 'reached'

    def _make_result(self, success, exit_reason, err):
        res = ArmMoveToPose.Result()
        res.success     = success
        res.exit_reason = exit_reason
        res.error_code  = err
        self._fill_pose(res.actual_pose, self._get_ee_pose())
        return res

    @staticmethod
    def _fill_pose(arm_pose, ee_pose):
        """把 _get_ee_pose() 的 (x,y,z,roll,pitch,yaw[rad]) 写入 ArmPose（角度转度）。"""
        if ee_pose is None:
            return
        x, y, z, r, p, yw = ee_pose
        arm_pose.x     = x
        arm_pose.y     = y
        arm_pose.z     = z
        arm_pose.roll  = math.degrees(r)
        arm_pose.pitch = math.degrees(p)
        arm_pose.yaw   = math.degrees(yw)

    def _hold_current(self):
        """取消时下发一条保持当前关节位置的短轨迹，使运动停止。"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions       = list(self._joint_pos)
        pt.velocities      = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self.get_logger().info('■ 已取消，保持当前位置')

    # ── STOWED：关节空间归零，不经过 IK ──────────────────────────────────────
    def _go_stowed(self, speed: int) -> float:
        duration_s = STOWED_DURATION.get(speed, STOWED_DURATION[1])
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions       = [0.0] * 6
        pt.velocities      = [0.0] * 6
        ns = int(duration_s * 1e9)
        pt.time_from_start = Duration(sec=ns // 1_000_000_000,
                                      nanosec=ns % 1_000_000_000)
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self.get_logger().info(f'● STOWED 归零  时长={duration_s:.1f}s')
        return duration_s

    # ── 工具方法 ──────────────────────────────────────────────────────────────
    def _on_js(self, msg: JointState):
        n2i = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(JOINT_NAMES):
            idx = n2i.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_pos[i] = msg.position[idx]

    def _get_ee_pose(self):
        try:
            t  = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, yw = quat_to_rpy(q.x, q.y, q.z, q.w)
            return tr.x, tr.y, tr.z, r, p, yw
        except Exception:
            return None

    def _get_ee_quat(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_LINK, rclpy.time.Time())
            q = t.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except Exception:
            return None

    # ── IK 同步 ───────────────────────────────────────────────────────────────
    _IK_ERR = {1: 'OK', -1: 'PLANNING_FAILED', -6: 'TIMED_OUT',
               -31: 'NO_IK_SOLUTION', 99999: 'TIMEOUT_WAIT'}

    def _ik_sync(self, x, y, z, qx, qy, qz, qw, seed):
        if not self._ik_cli.service_is_ready():
            return None, -1
        ps = PoseStamped()
        ps.header.frame_id    = BASE_FRAME
        ps.header.stamp       = self.get_clock().now().to_msg()
        ps.pose.position.x    = x;  ps.pose.position.y    = y
        ps.pose.position.z    = z;  ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy; ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        rs = RobotState()
        rs.joint_state.name     = JOINT_NAMES
        rs.joint_state.position = list(seed)
        req = GetPositionIK.Request()
        req.ik_request.group_name       = PLANNING_GROUP
        req.ik_request.ik_link_name     = EEF_LINK
        req.ik_request.pose_stamped     = ps
        req.ik_request.robot_state      = rs
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.nanosec  = int(IK_TIMEOUT_S * 1e9)
        ev = threading.Event(); box = [None]
        def _cb(f): box[0] = f; ev.set()
        self._ik_cli.call_async(req).add_done_callback(_cb)
        ev.wait(timeout=IK_TIMEOUT_S + 0.05)
        if box[0] is None:
            return None, 99999
        try:
            resp = box[0].result()
        except Exception:
            return None, -1
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, resp.error_code.val
        n2p = dict(zip(resp.solution.joint_state.name,
                       resp.solution.joint_state.position))
        return [n2p.get(n, 0.0) for n in JOINT_NAMES], MoveItErrorCodes.SUCCESS

    # ── 批量 IK + 轨迹下发 ────────────────────────────────────────────────────
    def _solve_and_send(self, all_pts, desc):
        """返回 (ok, t_traj)。ok=False 表示首点 IK 失败或被中止。"""
        n_ik   = len(all_pts)
        t_traj = all_pts[-1][0]
        self.get_logger().info(f'⚙ 规划中 {n_ik} 点  {desc}  时长={t_traj:.2f}s')

        seed      = list(self._joint_pos)
        joint_pos = []
        joint_t   = []

        for idx, (t_pt, wx, wy, wz, qx, qy, qz, qw) in enumerate(all_pts):
            if self._stop_req:
                self.get_logger().info('■ 规划中止'); return False, 0.0
            sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, seed)
            if sol is None and idx == 0:
                sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, [0.0]*6)
            if sol is None:
                err_name = self._IK_ERR.get(err, str(err))
                if joint_pos:
                    sol = joint_pos[-1]
                    self.get_logger().warn(
                        f'IK 失败 step={idx} err={err_name} '
                        f'pos=({wx:.3f},{wy:.3f},{wz:.3f})')
                else:
                    self.get_logger().error(
                        f'✗ IK 失败(step=0) err={err_name} '
                        f'pos=({wx:.3f},{wy:.3f},{wz:.3f})')
                    return False, 0.0
            joint_pos.append(sol)
            joint_t.append(t_pt)
            seed = sol
            if (idx + 1) % 20 == 0:
                self.get_logger().info(f'⚙ 规划中 {idx+1}/{n_ik}...')

        if self._stop_req:
            return False, 0.0

        n    = len(joint_pos)
        jvel = [[0.0] * 6 for _ in range(n)]
        for i in range(1, n - 1):
            dt2 = joint_t[i+1] - joint_t[i-1]
            for j in range(6):
                jvel[i][j] = (joint_pos[i+1][j] - joint_pos[i-1][j]) / dt2

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        for i in range(n):
            pt            = JointTrajectoryPoint()
            pt.positions  = joint_pos[i]
            pt.velocities = jvel[i]
            ns = int(joint_t[i] * 1e9)
            pt.time_from_start = Duration(sec=ns // 1_000_000_000,
                                          nanosec=ns % 1_000_000_000)
            msg.points.append(pt)
        self._traj_pub.publish(msg)
        self.get_logger().info(f'● 执行中  {n} 路点  {t_traj:.2f}s  {desc}')
        return True, t_traj

    # ── Ruckig 点到点规划 ─────────────────────────────────────────────────────
    def _plan_ptp(self, target, speed):
        """同步规划并下发。返回 (ok, t_traj, exit_reason, error_code)。"""
        x, y, z, roll_deg, pitch_deg, yaw_deg = target
        sp = SPEED_PARAMS.get(speed, SPEED_PARAMS[1])
        v_pos, a_pos, j_pos = sp['v_pos'], sp['a_pos'], sp['j_pos']
        v_ori, a_ori, j_ori = sp['v_ori'], sp['a_ori'], sp['j_ori']

        start = self._get_ee_pose(); q_cur = self._get_ee_quat()
        if start is None or q_cur is None:
            self.get_logger().error('✗ TF 未就绪')
            return False, 0.0, 'error', ERR_DRIVER
        x0, y0, z0 = start[0], start[1], start[2]

        q_end = rpy_to_quat(math.radians(roll_deg),
                            math.radians(pitch_deg),
                            math.radians(yaw_deg))
        if quat_dot(q_cur, q_end) < 0:
            q_end = (-q_end[0], -q_end[1], -q_end[2], -q_end[3])
        theta_total = 2.0 * math.acos(
            max(-1., min(1., abs(quat_dot(quat_normalize(q_cur),
                                          quat_normalize(q_end))))))

        otg = Ruckig(4, STREAM_DT)
        inp = InputParameter(4); out = OutputParameter(4)
        inp.current_position     = [x0, y0, z0, 0.]
        inp.current_velocity     = [0., 0., 0., 0.]
        inp.current_acceleration = [0., 0., 0., 0.]
        inp.target_position      = [x,  y,  z,  theta_total]
        inp.target_velocity      = [0., 0., 0., 0.]
        inp.target_acceleration  = [0., 0., 0., 0.]
        s_v = v_ori if theta_total > 1e-6 else max(v_ori, 1e-3)
        s_a = a_ori if theta_total > 1e-6 else max(a_ori, 1e-3)
        s_j = j_ori if theta_total > 1e-6 else max(j_ori, 1e-3)
        inp.max_velocity     = [v_pos, v_pos, v_pos, s_v]
        inp.max_acceleration = [a_pos, a_pos, a_pos, s_a]
        inp.max_jerk         = [j_pos, j_pos, j_pos, s_j]

        all_pts = []; t_acc = 0.
        while True:
            res = otg.update(inp, out); t_acc += STREAM_DT
            rx, ry, rz, rs = out.new_position
            if theta_total > 1e-6:
                t_s = max(0., min(1., rs / theta_total))
                q   = quat_slerp(q_cur, q_end, t_s)
            else:
                q   = q_end
            all_pts.append((t_acc, rx, ry, rz, *q))
            out.pass_to_input(inp)
            if res == Result.Finished: break
            if res == Result.Error:
                self.get_logger().error('✗ Ruckig 求解失败')
                return False, 0.0, 'error', ERR_DRIVER

        dist = math.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)
        ok, t_traj = self._solve_and_send(
            all_pts,
            f'Δs={dist*1000:.1f}mm  Δθ={math.degrees(theta_total):.1f}°')
        if not ok:
            reason = 'cancelled' if self._stop_req else 'error'
            return False, 0.0, reason, ERR_DRIVER
        return True, t_traj, 'reached', ERR_NONE


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node     = PoseCommandExecutor()
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
