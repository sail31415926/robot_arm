#!/usr/bin/env python3
"""
@file   spherical_orbit_controller_node.py
@brief  eMeetArm 球面坐标环绕运镜 node（无 GUI，可被 GUI import 或 headless 运行）
@version 1.1
@date   2026-06-09

球面坐标系环绕运镜，相机始终对准被摄主体，不依赖任何 GUI 框架：
  1-DOF Ruckig 规划归一化参数 s∈[0,1]，θ/φ/r 随 s 线性插值，
  pipeline：Ruckig(s) → (θ,φ,r) → 球坐标→笛卡尔 → aim_quat
           → /compute_ik × N（种子延续）→ JointTrajectory → /arm_controller
  状态/位姿通过 gui_q 队列上报（headless 运行时传空队列即可）。

被 spherical_orbit_gui.py（tkinter 调试 GUI）import 使用；也可独立运行：
  ros2 run robot_arm_node spherical_orbit_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading

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

try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
except ImportError as e:
    raise SystemExit('✗ 未找到 ruckig 库，请先安装：pip install ruckig') from e

from arm_utils import (rpy_to_quat, quat_to_rpy, quat_normalize, quat_dot, quat_slerp,
                        aim_quat, look_at_quat, sphere_to_cart, cart_to_sphere)

# ── 常量 ──────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'gimbal_tool0'   # 2026-07-28 云台换 V2：MoveIt 规划组 tip
BASE_FRAME     = 'arm_base_link'

STREAM_DT    = 0.01    # Ruckig 步长，100 Hz
IK_TIMEOUT_S = 0.05    # 单次 IK 超时

# 球面轨道 Ruckig 默认限制（作用于归一化参数 s，单位 1/s, 1/s², 1/s³）
DEFAULT_S_VEL  = 0.10
DEFAULT_S_ACC  = 0.10
DEFAULT_S_JERK = 1.50

# 移到起始位置 PTP 默认限制
DEFAULT_V_POS = 0.05;  DEFAULT_A_POS = 0.10;  DEFAULT_J_POS = 1.00
DEFAULT_V_ORI = 0.10;  DEFAULT_A_ORI = 0.20;  DEFAULT_J_ORI = 2.00

# 2026-07-28 云台换 V2 后重算：末端 = gimbal_tool0（SRDF 规划组 tip）。
# 取值来自位形 [0, 1.2, -1.2, 0, 0, 0] 的 FK —— J2=-J3 时前臂保持水平，
# 末端姿态恰为中性 (0, 0.29°, 0)，故姿态角取 0；老值 (roll=90°, pitch=10°)
# 是 V1 云台时代的约定，在新末端坐标系下已无意义。
READY_POSE = dict(x=0.315, y=-0.036, z=0.548, roll=0.0, pitch=0.0, yaw=0.0)


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class SphericalOrbitControllerNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('spherical_orbit_controller_node')
        # use_sim_time 由 launch 按后端传入（gazebo/mujoco=true，real=false）。
        # 切勿在此硬编码覆盖：实物无 /clock 时 use_sim_time=true 会让所有
        # ROS 定时器永不触发（位姿面板卡 '--' 的教训）。
        self._q = gui_q

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        self._joint_pos = [0.0] * 6
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)

        self._planning = False
        self._stop_req = False

    # ── TF / 关节状态 ──────────────────────────────────────────────────────────
    def _on_js(self, msg: JointState):
        n2i = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(JOINT_NAMES):
            idx = n2i.get(name)
            if idx is not None and idx < len(msg.position):
                self._joint_pos[i] = msg.position[idx]

    def _pub_pose(self):
        pose = self.get_ee_pose()
        if pose is None:
            return
        x, y, z, r, p, yw = pose
        self._q.put(('pose', x, y, z,
                     math.degrees(r), math.degrees(p), math.degrees(yw)))

    def get_ee_pose(self):
        try:
            t  = self.tf_buffer.lookup_transform(BASE_FRAME, EEF_LINK, rclpy.time.Time())
            tr = t.transform.translation
            q  = t.transform.rotation
            r, p, yw = quat_to_rpy(q.x, q.y, q.z, q.w)
            return tr.x, tr.y, tr.z, r, p, yw
        except Exception:
            return None

    def get_ee_quat(self):
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, EEF_LINK, rclpy.time.Time())
            q = t.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except Exception:
            return None

    # ── 同步 IK ────────────────────────────────────────────────────────────────
    _IK_ERR = {1: 'OK', -1: 'PLANNING_FAILED', -6: 'TIMED_OUT',
               -15: 'INVALID_GROUP_NAME', -31: 'NO_IK_SOLUTION',
               99999: 'TIMEOUT_WAIT'}

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
        req.ik_request.timeout.sec      = 0
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

    # ── 停止 ───────────────────────────────────────────────────────────────────
    def stop_motion(self):
        self._stop_req = True
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions       = list(self._joint_pos)
        pt.velocities      = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=int(0.1e9))
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self._q.put(('status', '■ 已停止'))

    # ── 共用：IK 批量求解 + JointTrajectory 下发 ─────────────────────────────
    def _solve_and_send(self, all_pts, desc):
        """all_pts: list of (t, x, y, z, qx, qy, qz, qw)"""
        n_ik   = len(all_pts)
        t_traj = all_pts[-1][0]
        self._q.put(('status',
                     f'⚙ 规划中... {n_ik} 个 IK 点  {desc}  时长={t_traj:.2f}s'))

        seed      = list(self._joint_pos)
        joint_pos = []
        joint_t   = []

        for idx, (t_pt, wx, wy, wz, qx, qy, qz, qw) in enumerate(all_pts):
            if self._stop_req:
                self._q.put(('status', '■ 规划中止'))
                return
            sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, seed)
            if sol is None and idx == 0:
                sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw, [0.0]*6)
            if sol is None:
                err_name = self._IK_ERR.get(err, str(err))
                if joint_pos:
                    sol = joint_pos[-1]
                    self.get_logger().warn(
                        f'IK 失败 step={idx} err={err_name}  '
                        f'pos=({wx:.3f},{wy:.3f},{wz:.3f})')
                else:
                    self._q.put(('status',
                                 f'✗ IK 失败（step=0 err={err_name}）'
                                 f'  pos=({wx:.3f},{wy:.3f},{wz:.3f})'
                                 f'  quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})'))
                    return
            joint_pos.append(sol)
            joint_t.append(t_pt)
            seed = sol
            if (idx + 1) % 20 == 0:
                self._q.put(('status', f'⚙ 规划中 {idx+1}/{n_ik}...'))

        if self._stop_req:
            return

        # 中央差分计算关节速度，端点为零
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
        self._q.put(('status',
                     f'● 执行中  {n} 个路点  时长 {t_traj:.2f}s  {desc}'))

    # ── 移到起始位置（4-DOF PTP Ruckig）──────────────────────────────────────
    def goto_start(self, ox, oy, oz, theta0_rad, phi0_rad, r0,
                   v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        px, py, pz = sphere_to_cart(theta0_rad, phi0_rad, r0, ox, oy, oz)
        q_end = aim_quat(px, py, pz, ox, oy, oz)
        threading.Thread(
            target=self._plan_ptp,
            args=(px, py, pz, q_end,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
            daemon=True).start()

    def _build_ptp_plan(self, tx, ty, tz, q_end_in,
                        v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        """构建 PTP 路径点列表，不发送。失败返回 None。"""
        start = self.get_ee_pose();  q_cur = self.get_ee_quat()
        if start is None or q_cur is None:
            self._q.put(('status', '✗ TF 未就绪')); return None
        x0, y0, z0 = start[0], start[1], start[2]
        q_end = q_end_in
        if quat_dot(q_cur, q_end) < 0:
            q_end = (-q_end[0], -q_end[1], -q_end[2], -q_end[3])
        theta_total = 2.0 * math.acos(
            max(-1., min(1., abs(quat_dot(quat_normalize(q_cur),
                                          quat_normalize(q_end))))))
        otg = Ruckig(4, STREAM_DT)
        inp = InputParameter(4);  out = OutputParameter(4)
        inp.current_position     = [x0, y0, z0, 0.]
        inp.current_velocity     = [0., 0., 0., 0.]
        inp.current_acceleration = [0., 0., 0., 0.]
        inp.target_position      = [tx, ty, tz, theta_total]
        inp.target_velocity      = [0., 0., 0., 0.]
        inp.target_acceleration  = [0., 0., 0., 0.]
        sv = v_ori if theta_total > 1e-6 else max(v_ori, 1e-3)
        sa = a_ori if theta_total > 1e-6 else max(a_ori, 1e-3)
        sj = j_ori if theta_total > 1e-6 else max(j_ori, 1e-3)
        inp.max_velocity     = [v_pos, v_pos, v_pos, sv]
        inp.max_acceleration = [a_pos, a_pos, a_pos, sa]
        inp.max_jerk         = [j_pos, j_pos, j_pos, sj]

        all_pts = [];  t_acc = 0.
        while True:
            res = otg.update(inp, out);  t_acc += STREAM_DT
            rx, ry, rz, rs = out.new_position
            if theta_total > 1e-6:
                t_s = max(0., min(1., rs / theta_total))
                qx, qy, qz, qw = quat_slerp(q_cur, q_end, t_s)
            else:
                qx, qy, qz, qw = q_end
            all_pts.append((t_acc, rx, ry, rz, qx, qy, qz, qw))
            out.pass_to_input(inp)
            if res == Result.Finished: break
            if res == Result.Error:
                self._q.put(('status', '✗ Ruckig 求解失败')); return None
        return all_pts

    def _plan_ptp(self, tx, ty, tz, q_end_in,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        self._planning = True
        try:
            all_pts = self._build_ptp_plan(tx, ty, tz, q_end_in,
                                           v_pos, a_pos, j_pos,
                                           v_ori, a_ori, j_ori)
            if all_pts is None:
                return
            dist = math.sqrt((tx - all_pts[0][1])**2 +
                             (ty - all_pts[0][2])**2 +
                             (tz - all_pts[0][3])**2)
            # 计算 Δθ 用于日志
            q_cur = self.get_ee_quat()
            d = abs(quat_dot(quat_normalize(q_cur) if q_cur else (0.,0.,0.,1.),
                             quat_normalize(q_end_in)))
            ang = 2.0 * math.acos(max(-1., min(1., d)))
            self._solve_and_send(
                all_pts,
                f'移到起始位  Δs={dist*1000:.1f}mm  Δθ={math.degrees(ang):.1f}°')
        finally:
            self._planning = False

    def go_ready(self, v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        p = READY_POSE
        q = rpy_to_quat(math.radians(p['roll']),
                        math.radians(p['pitch']),
                        math.radians(p['yaw']))
        threading.Thread(
            target=self._plan_ptp,
            args=(p['x'], p['y'], p['z'], q,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
            daemon=True).start()

    # ── 球面轨道规划（1-DOF Ruckig on s）────────────────────────────────────
    def send_orbit(self, ox, oy, oz,
                   theta0_rad, phi0_rad, r0,
                   theta1_rad, phi1_rad, r1,
                   s_vel, s_acc, s_jerk):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(
            target=self._plan_orbit,
            args=(ox, oy, oz,
                  theta0_rad, phi0_rad, r0,
                  theta1_rad, phi1_rad, r1,
                  s_vel, s_acc, s_jerk),
            daemon=True).start()

    def send_orbit_chained(self, ox, oy, oz,
                           theta0_rad, phi0_rad, r0,
                           theta1_rad, phi1_rad, r1,
                           s_vel, s_acc, s_jerk,
                           v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        """先 PTP 移到球面轨道起点，再执行轨道规划。"""
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(
            target=self._plan_chained,
            args=(ox, oy, oz,
                  theta0_rad, phi0_rad, r0,
                  theta1_rad, phi1_rad, r1,
                  s_vel, s_acc, s_jerk,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
            daemon=True).start()

    def _plan_chained(self, ox, oy, oz,
                      theta0, phi0, r0,
                      theta1, phi1, r1,
                      s_vel, s_acc, s_jerk,
                      v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        self._planning = True
        try:
            # Step 1: PTP 到起点
            px, py, pz = sphere_to_cart(theta0, phi0, r0, ox, oy, oz)
            q_end = aim_quat(px, py, pz, ox, oy, oz)
            self._q.put(('status',
                         f'● 先移到起始位置 …  '
                         f'({px:.3f},{py:.3f},{pz:.3f})'))
            ptp_pts = self._build_ptp_plan(px, py, pz, q_end,
                                           v_pos, a_pos, j_pos,
                                           v_ori, a_ori, j_ori)
            if ptp_pts is None or self._stop_req:
                if not self._stop_req:
                    self._q.put(('status', '✗ 移到起始位置失败，放弃轨道执行'))
                return
            self._solve_and_send(ptp_pts, 'PTP → 起点')
            if self._stop_req:
                return

            # Step 2: 执行球面轨道
            self._q.put(('status', '● 已到达起点，开始执行球面轨道 …'))
            self._plan_orbit_inner(ox, oy, oz,
                                   theta0, phi0, r0,
                                   theta1, phi1, r1,
                                   s_vel, s_acc, s_jerk)
        finally:
            self._planning = False

    def _plan_orbit(self, ox, oy, oz,
                    theta0, phi0, r0,
                    theta1, phi1, r1,
                    s_vel, s_acc, s_jerk):
        self._planning = True
        try:
            self._plan_orbit_inner(ox, oy, oz,
                                   theta0, phi0, r0,
                                   theta1, phi1, r1,
                                   s_vel, s_acc, s_jerk)
        finally:
            self._planning = False

    def _plan_orbit_inner(self, ox, oy, oz,
                          theta0, phi0, r0,
                          theta1, phi1, r1,
                          s_vel, s_acc, s_jerk):
        d_theta = theta1 - theta0
        d_phi   = phi1   - phi0
        d_r     = r1     - r0

        otg = Ruckig(1, STREAM_DT)
        inp = InputParameter(1);  out = OutputParameter(1)
        inp.current_position     = [0.]
        inp.current_velocity     = [0.]
        inp.current_acceleration = [0.]
        inp.target_position      = [1.]
        inp.target_velocity      = [0.]
        inp.target_acceleration  = [0.]
        inp.max_velocity         = [s_vel]
        inp.max_acceleration     = [s_acc]
        inp.max_jerk             = [s_jerk]

        all_pts = [];  t_acc = 0.
        while True:
            res = otg.update(inp, out);  t_acc += STREAM_DT
            s = max(0., min(1., out.new_position[0]))

            theta = theta0 + s * d_theta
            phi   = phi0   + s * d_phi
            r     = r0     + s * d_r

            px, py, pz         = sphere_to_cart(theta, phi, r, ox, oy, oz)
            qx, qy, qz, qw     = aim_quat(px, py, pz, ox, oy, oz)
            all_pts.append((t_acc, px, py, pz, qx, qy, qz, qw))
            out.pass_to_input(inp)
            if res == Result.Finished: break
            if res == Result.Error:
                self._q.put(('status', '✗ Ruckig 求解失败')); return

        self._solve_and_send(
            all_pts,
            f'球面轨道  Δθ={abs(math.degrees(d_theta)):.1f}°  '
            f'Δφ={abs(math.degrees(d_phi)):.1f}°  '
            f'Δr={abs(d_r)*1000:.0f}mm')


def main():
    rclpy.init()
    node     = SphericalOrbitControllerNode(queue.Queue())
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
