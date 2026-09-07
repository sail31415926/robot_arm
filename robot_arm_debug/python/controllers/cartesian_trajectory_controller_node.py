#!/usr/bin/env python3
"""
@file   cartesian_trajectory_controller_node.py
@brief  eMeetArm Ruckig 笛卡尔轨迹 + 批量 IK 执行 node（无 GUI，可被 GUI import 或 headless 运行）
@version 1.2
@date   2026-06-09

两种运镜模式，共用 IK 批量求解 + JointTrajectory 下发流程，不依赖任何 GUI 框架：
         【点到点】Ruckig 4-DOF(x,y,z,s) → 100Hz 笛卡尔位姿序列
         【环  绕】Ruckig 1-DOF(θ) → 圆弧位置 + look-at 朝向
                         ↓ 共用
                  /compute_ik × N（pick_ik，种子延续）→ JointTrajectory → /arm_controller
         状态/位姿通过 gui_q 队列上报（headless 运行时传空队列即可）。

被 cartesian_trajectory_gui.py（tkinter 调试 GUI）import 使用；也可独立运行：
  ros2 run robot_arm_node cartesian_trajectory_controller_node.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import queue
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer

try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
except ImportError as e:
    raise SystemExit('✗ 未找到 ruckig 库，请先安装：pip install ruckig') from e

from arm_utils import (rpy_to_quat, quat_to_rpy, quat_normalize, quat_dot, quat_slerp,
                        quat_mul, look_at_quat)

# ── 常量 ──────────────────────────────────────────────────────────────────────
JOINT_NAMES    = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
PLANNING_GROUP = 'arm'
EEF_LINK       = 'gimbal_tool0'   # 2026-07-28 云台换 V2：MoveIt 规划组 tip
BASE_FRAME     = 'arm_base_link'

STREAM_DT    = 0.01    # s，Ruckig 内部步长（100Hz）
IK_TIMEOUT_S = 0.05    # s，单次 IK 超时（50ms）

# ── 轨迹保真度（2026-08-31）──────────────────────────────────────────────────
# 需求：下发的每个路点，其 FK 末端都必须落在规划笛卡尔轨迹的邻域内。
#
# ★ 因此 IK 求不出解时**剔除该点**，不再「沿用上一帧」——
#   沿用上帧是把末端钉在上一点、再跳到下一个有解点，位置序列明确偏离规划曲线；
#   剔除后前后两个保留点本身都在规划轨迹上，JTC 只在这两点间抄一段近路，偏差是
#   弦弧差 L²/(8R)（L=v·Δt）：50ms 间隔、0.3m/s、R=0.3m 时 L=15mm，偏差约 0.09mm。
#
#   老的 MAX_CONSEC_IK_FAIL（连续 0.3s 才放弃）已被 MAX_GAP_SEC 取代：那个阈值
#   是为「沿用上帧」配的容忍度，剔点方案下保留点间隔才是真正决定偏差的量。
MAX_GAP_SEC  = 0.05    # s，保留点之间允许的最大间隔
MAX_DROP_RUN = max(1, int(round(MAX_GAP_SEC / STREAM_DT)) - 1)   # → 允许连续剔 4 点

# IK 是数值解，返回 SUCCESS 不等于精确命中目标位姿 —— 每个解回代 FK 校验，
# 超差点按「无解」处理（走换种子重试 → 剔除 → 放弃的同一条流程）。
FK_VERIFY      = True   # 关掉可省约一半规划耗时，但失去邻域保证
FK_POS_TOL     = 0.002  # m，末端位置容差
FK_ORI_TOL_DEG = 1.0    # °，末端姿态容差

# ── 云台摄像机拍摄专项（2026-08-31）──────────────────────────────────────────
# 末端是相机（gimbal_tool0 挂在云台 J4-6 之后），拍摄场景对**画面连续性**的要求
# 比几何精度更苛刻：关节速度突变直接变成画面抖动，IK 解族跳变就是画面天旋地转。
#
# ① 跳变判据 per-joint、以 URDF 的速度上限为准（不再用一个全局常量）：
#    相邻保留点的关节增量不得超过「该关节速度上限 × Δt × JUMP_MARGIN」。
#    这样云台 J4-6（限速 1.0 rad/s，远低于臂 J1-3 的 3.14）自动拿到更严的朝向
#    连续性约束 —— 正是相机需要的；而超过速度上限的增量本来也执行不了。
JUMP_MARGIN = 1.0       # 1.0 = 严格按 URDF 速度上限，放宽可调大

# ② 开拍瞬间不许甩：轨迹首点解与机械臂当前位形的关节距离超过 RAMP_IN_TOL_RAD 时，
#    先用 Ruckig 在关节空间生成一段平顺过渡（ramp-in）接到首点，再接拍摄段。
#    过渡段是**额外**时长，拍摄段本身时长不变；状态栏把两段分开报，成片只取拍摄段。
#    （orbit 这类首点不等于当前位姿的路径，不做这一步就会在开拍瞬间高速甩过去：
#      Gazebo 里表现为瞬移，实机上是一次没人预期的大幅运动。）
RAMP_IN_ENABLE    = True
RAMP_IN_TOL_RAD   = 0.02    # rad，首点与当前位形的容许差（≈1.1°）
RAMP_IN_VEL_SCALE = 0.30    # 过渡段速度取 URDF 上限的这个比例（保守、平顺）

# 点到点默认限制
DEFAULT_V_POS = 0.05;  DEFAULT_A_POS = 0.10;  DEFAULT_J_POS = 1.00
DEFAULT_V_ORI = 0.10;  DEFAULT_A_ORI = 0.20;  DEFAULT_J_ORI = 2.00

# 环绕默认限制（角度空间）
DEFAULT_W_ORB = 0.30   # rad/s   轨道角速度上限
DEFAULT_A_ORB = 0.30   # rad/s²
DEFAULT_J_ORB = 1.00   # rad/s³

# 2026-07-28 云台换 V2 后重算：末端 = gimbal_tool0（SRDF 规划组 tip）。
# 取值来自位形 [0, 1.2, -1.2, 0, 0, 0] 的 FK —— J2=-J3 时前臂保持水平，
# 末端姿态恰为中性 (0, 0.29°, 0)，故姿态角取 0；老值 (roll=90°, pitch=10°)
# 是 V1 云台时代的约定，在新末端坐标系下已无意义。
READY_POSE = dict(x=0.315, y=-0.036, z=0.548, roll=0.0, pitch=0.0, yaw=0.0)

PTP_PARAMS = [
    ('X',     'm',   -0.8,   0.8),
    ('Y',     'm',   -0.8,   0.8),
    ('Z',     'm',   -0.1,   0.8),
    ('Roll',  '°', -180.0, 180.0),
    ('Pitch', '°',  -90.0,  90.0),
    ('Yaw',   '°', -180.0, 180.0),
]


# ── ROS 节点 ──────────────────────────────────────────────────────────────────
class CartesianTrajectoryControllerNode(Node):
    def __init__(self, gui_q: queue.Queue):
        super().__init__('cartesian_trajectory_controller_node')
        # use_sim_time 由 launch 按后端传入（gazebo/mujoco=true，real=false）。
        # 切勿在此硬编码覆盖：实物无 /clock 时 use_sim_time=true 会让所有
        # ROS 定时器永不触发（位姿面板卡 '--' 的教训）。
        self._q = gui_q

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self._fk_cli = self.create_client(GetPositionFK, '/compute_fk')
        self._fk_warned = False   # /compute_fk 不可用只告警一次，见 _solve_point

        # 关节限位（位置 + 速度上限），从 /robot_description 解析，见 _on_robot_description
        self._jlim = None
        self.create_subscription(
            String, '/robot_description', self._on_robot_description,
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._pub_pose)

        self._joint_pos = [0.0] * 6
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)

        self._planning = False
        self._stop_req = False

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
        self._q.put(('pose', x, y, z, math.degrees(r),
                     math.degrees(p), math.degrees(yw)))

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

    # ── 同步 IK（后台线程专用）────────────────────────────────────────────────
    # 返回 (joint_list, error_code_val)；成功时 error_code=1，失败时 joint_list=None
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

    def _on_robot_description(self, msg):
        """解析 /robot_description，取各关节的位置与速度上限。

        两个用途：
          · 把 IK 种子夹进合法范围 —— pick_ik 拿到超限的种子会直接丢弃它、改用
            **随机**位形重新搜索（move_group 日志 'Initial guess exceeds joint
            limits. Regenerating a random valid configuration.'），解族就完全不
            可控了。而机械臂停在限位上时，实测关节角本身就可能因浮点误差略微越界，
            这条路径很容易踩到。
          · per-joint 的跳变判据（见 JUMP_MARGIN）。
        """
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().error(f'/robot_description 解析失败：{e}')
            return
        lim = {}
        for j in root.findall('joint'):
            name = j.get('name')
            if name not in JOINT_NAMES:
                continue
            node = j.find('limit')
            if node is None:
                continue
            lim[name] = dict(lower=float(node.get('lower', '-3.14')),
                             upper=float(node.get('upper', '3.14')),
                             vel=float(node.get('velocity', '1.0')))
        if len(lim) != len(JOINT_NAMES):
            self.get_logger().warn(
                f'/robot_description 只解析到 {len(lim)}/{len(JOINT_NAMES)} 个关节限位，'
                f'种子 clamp 与 per-joint 跳变判据退化为保守默认值')
            return
        self._jlim = [lim[n] for n in JOINT_NAMES]
        self.get_logger().info(
            '关节限位已载入  ' + '  '.join(
                f'{n}[{lim[n]["lower"]:+.2f},{lim[n]["upper"]:+.2f}]'
                f'v≤{lim[n]["vel"]:.2f}' for n in JOINT_NAMES))

    def _clamp_seed(self, seed):
        """把 IK 种子夹进关节限位内（留 1e-3 余量，避开边界浮点越界）。

        限位未就绪时原样返回 —— 宁可不夹，也不要用猜的限位把种子改错。
        """
        if not self._jlim:
            return list(seed)
        out = []
        for v, lm in zip(seed, self._jlim):
            lo, hi = lm['lower'] + 1e-3, lm['upper'] - 1e-3
            out.append(lo if v < lo else (hi if v > hi else v))
        return out

    def _jump_limit(self, dt):
        """相邻保留点各关节允许的最大增量（rad），per-joint。"""
        if not self._jlim:
            return [3.14 * dt * JUMP_MARGIN] * len(JOINT_NAMES)
        return [lm['vel'] * dt * JUMP_MARGIN for lm in self._jlim]

    def _plan_ramp_in(self, q_from, q_to):
        """关节空间平顺过渡（Ruckig 6-DOF），返回 [(t, q6), ...]，失败返回 None。

        速度取 URDF 上限的 RAMP_IN_VEL_SCALE —— 这一段也会被相机拍到，
        宁可慢一点也不要甩。
        """
        n = len(JOINT_NAMES)
        vmax = ([lm['vel'] * RAMP_IN_VEL_SCALE for lm in self._jlim]
                if self._jlim else [1.0 * RAMP_IN_VEL_SCALE] * n)
        otg = Ruckig(n, STREAM_DT)
        inp = InputParameter(n);  out = OutputParameter(n)
        inp.current_position     = list(q_from)
        inp.current_velocity     = [0.0] * n
        inp.current_acceleration = [0.0] * n
        inp.target_position      = list(q_to)
        inp.target_velocity      = [0.0] * n
        inp.target_acceleration  = [0.0] * n
        inp.max_velocity     = vmax
        inp.max_acceleration = [v * 2.0  for v in vmax]
        inp.max_jerk         = [v * 10.0 for v in vmax]
        pts = [];  t = 0.0
        while True:
            res = otg.update(inp, out)
            t += STREAM_DT
            pts.append((t, list(out.new_position)))
            out.pass_to_input(inp)
            if res == Result.Finished:
                return pts
            if res == Result.Error or t > 30.0:
                return None

    def _fk_sync(self, joints):
        """关节解回代 FK，返回末端 (x,y,z,qx,qy,qz,qw)；服务不可用或失败返回 None。

        用途是校验 IK 数值解有没有真的命中目标位姿 —— 求解器返回 SUCCESS 只说明
        它自己收敛了，不保证落在我们要求的容差内。
        """
        if not self._fk_cli.service_is_ready():
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names   = [EEF_LINK]
        req.robot_state.joint_state.name     = JOINT_NAMES
        req.robot_state.joint_state.position = list(joints)
        ev = threading.Event(); box = [None]
        def _cb(f): box[0] = f; ev.set()
        self._fk_cli.call_async(req).add_done_callback(_cb)
        ev.wait(timeout=IK_TIMEOUT_S + 0.05)
        if box[0] is None:
            return None
        try:
            resp = box[0].result()
        except Exception:
            return None
        if resp.error_code.val != MoveItErrorCodes.SUCCESS or not resp.pose_stamped:
            return None
        p = resp.pose_stamped[0].pose
        return (p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    @staticmethod
    def _pose_err(fk, wx, wy, wz, qx, qy, qz, qw):
        """FK 结果与规划位姿的偏差，返回 (位置 m, 姿态 rad)。

        姿态取四元数夹角，abs(dot) 消去 q 与 -q 表示同一旋转的歧义。
        """
        dpos = math.sqrt((fk[0] - wx) ** 2 + (fk[1] - wy) ** 2 + (fk[2] - wz) ** 2)
        d    = abs(quat_dot((fk[3], fk[4], fk[5], fk[6]), (qx, qy, qz, qw)))
        return dpos, 2.0 * math.acos(min(1.0, d))

    def _solve_point(self, wx, wy, wz, qx, qy, qz, qw, seeds,
                     prev_sol=None, dt=None):
        """求单点 IK：多种子重试 + 解族跳变校验 + FK 回代校验。

        返回 (sol, why, dpos, dori)。成功时 why=None；失败时 sol=None、why 是最后
        一次失败的原因（供日志和状态栏定位）。dpos/dori 是该解的 FK 偏差，
        FK_VERIFY 关闭或 /compute_fk 不可用时为 None。
        """
        why = 'NO_IK_SOLUTION'
        for sd in seeds:
            # 种子必须先夹进限位：超限种子会让 pick_ik 丢弃它并随机重启（见 _clamp_seed）
            sol, err = self._ik_sync(wx, wy, wz, qx, qy, qz, qw,
                                     self._clamp_seed(sd))
            if sol is None:
                why = self._IK_ERR.get(err, str(err))
                continue

            # ① 解族跳变校验。放在 FK 之前：跳变解的 FK 往往是"正确"的
            #    （末端位姿确实对得上），只有关节空间才看得出它换了解族。
            if prev_sol is not None and dt:
                lims = self._jump_limit(dt)
                over = [(i, abs(a - b) - lims[i], abs(a - b), lims[i])
                        for i, (a, b) in enumerate(zip(sol, prev_sol))
                        if abs(a - b) > lims[i]]
                if over:
                    i, _, dq, lm = max(over, key=lambda t: t[1])
                    why = (f'{JOINT_NAMES[i]} 解族跳变 Δq={dq:.3f}rad > {lm:.3f}rad'
                           f'（限速 {lm/dt/JUMP_MARGIN:.2f}rad/s × {dt*1000:.0f}ms）')
                    continue

            # ② FK 回代校验：确认这个解真把末端放在了规划点的邻域内
            if not FK_VERIFY:
                return sol, None, None, None
            fk = self._fk_sync(sol)
            if fk is None:
                # FK 服务不可用时不阻断规划，退化为"不校验"，但留下痕迹。
                # 只警告一次：本节点 100Hz 全点求解，按点打会把日志淹掉。
                if not self._fk_warned:
                    self._fk_warned = True
                    self.get_logger().warn(
                        'FK 校验不可用（/compute_fk 无响应），本段退化为不校验邻域'
                        '（后续同类点不再重复告警，状态栏会标注"FK 未校验"）')
                return sol, None, None, None
            dpos, dori = self._pose_err(fk, wx, wy, wz, qx, qy, qz, qw)
            if dpos <= FK_POS_TOL and dori <= math.radians(FK_ORI_TOL_DEG):
                return sol, None, dpos, dori
            why = (f'FK 超差 {dpos*1000:.1f}mm/{math.degrees(dori):.2f}°'
                   f'（容差 {FK_POS_TOL*1000:.0f}mm/{FK_ORI_TOL_DEG:.1f}°）')
        return None, why, None, None

    # ── 停止 ──────────────────────────────────────────────────────────────────
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
        """
        all_pts: list of (t, x, y, z, qx, qy, qz, qw)
        对每个点求 IK，计算关节速度，下发完整 JointTrajectory。

        ★ 核心约定：下发的每个路点，其 FK 末端必须落在规划笛卡尔轨迹的邻域内
          （FK_POS_TOL / FK_ORI_TOL_DEG）。围绕这条，求不出解时的处理是
          「剔除该点」而不是「沿用上一帧」：

          · 首点求解失败                  → 起点不可达，放弃，不下发
          · 末点求解失败                  → 终点不可达，放弃，不下发
          · 中间点求解失败                → 换种子重试；仍失败则把该点从点列里剔除，
                                            由 JTC 在前后两个保留点之间插值走过去
          · 连续剔除 > MAX_DROP_RUN 点      → 成片不可达，放弃，不下发
          · 解族跳变 / FK 超差             → 同样算「求解失败」，走上面同一条流程

          首末两点不允许剔除：末点是唯一必须精确到达的点，首点决定整段起姿。

        为什么剔除优于沿用上一帧：沿用上帧把末端钉在上一点、再跳到下一个有解点，
        位置序列明确偏离规划曲线；剔除后前后两个保留点本身都在规划轨迹上，JTC 的
        插值只在两点之间抄一段近路，偏差是弦弧差量级（见 MAX_GAP_SEC 处的估算）。
        """
        n_ik   = len(all_pts)
        t_traj = all_pts[-1][0]
        self._q.put(('status', f'⚙ 规划中... {n_ik} 个 IK 点  {desc}  时长={t_traj:.2f}s'))

        seed      = list(self._joint_pos)
        joint_pos = []
        joint_t   = []

        # 轨迹保真度统计：成功下发时在状态栏报出，作为「末端确实贴着规划路径」的凭据
        dropped     = 0     # 累计剔除的点数
        consec_drop = 0     # 当前连续剔除计数（限制保留点之间的间隔）
        first_drop  = 0     # 当前连续剔除段的起始 step（报错定位用）
        max_gap     = 0.0   # s，保留点之间的最大间隔
        max_pos_err = 0.0   # m，FK 回代的最大位置偏差
        max_ori_err = 0.0   # rad，FK 回代的最大姿态偏差
        fk_checked  = 0     # 真正做过 FK 回代的点数（区分"校验通过"与"没校验"）

        for idx, (t_pt, wx, wy, wz, qx, qy, qz, qw) in enumerate(all_pts):
            if self._stop_req:
                self._q.put(('status', '■ 规划中止'))
                return

            is_first = (idx == 0)
            is_last  = (idx == n_ik - 1)

            # 种子顺序 = 连续性优先：上一个保留点的解 → 当前实测位形 → 零位。
            # 后两个是「种子离解太远」时的兜底，它们容易跳到别的解族，
            # 所以 _solve_point 内部对每个候选解都做跳变校验。
            seeds    = [seed, list(self._joint_pos), [0.0] * 6]
            prev_sol = joint_pos[-1] if joint_pos else None
            dt_prev  = (t_pt - joint_t[-1]) if joint_t else STREAM_DT

            sol, why, dp, do = self._solve_point(
                wx, wy, wz, qx, qy, qz, qw, seeds,
                prev_sol=prev_sol, dt=dt_prev)

            if sol is None:
                # 首点与末点必须精确命中，不允许剔除
                if is_first:
                    self.get_logger().error(
                        f'首点求解失败（{why}）pos=({wx:.3f},{wy:.3f},{wz:.3f})，'
                        f'起点不可达，放弃本段')
                    self._q.put(('status',
                                 f'✗ 首点求解失败（{why}）'
                                 f'  pos=({wx:.3f},{wy:.3f},{wz:.3f})'
                                 f'  quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})'
                                 f'  → 起点不可达，已放弃（未下发）'))
                    return
                if is_last:
                    self.get_logger().error(
                        f'末点求解失败（{why}）pos=({wx:.3f},{wy:.3f},{wz:.3f})，'
                        f'终点不可达，放弃本段')
                    self._q.put(('status',
                                 f'✗ 末点求解失败（{why}）'
                                 f'  pos=({wx:.3f},{wy:.3f},{wz:.3f})'
                                 f'  → 终点不可达，已放弃（未下发）'))
                    return

                # 中间点：剔除，交给 JTC 在前后两个保留点之间插值
                if consec_drop == 0:
                    first_drop = idx
                consec_drop += 1
                dropped     += 1
                gap = t_pt - joint_t[-1] + STREAM_DT
                self.get_logger().warn(
                    f'step={idx} 求解失败（{why}）pos=({wx:.3f},{wy:.3f},{wz:.3f})'
                    f' → 剔除该点（连续第 {consec_drop} 个）')
                if consec_drop > MAX_DROP_RUN:
                    fx, fy, fz = all_pts[first_drop][1:4]
                    self.get_logger().error(
                        f'连续 {consec_drop} 点求不出解（保留点间隔已达 '
                        f'{gap*1000:.0f}ms > {MAX_GAP_SEC*1000:.0f}ms，'
                        f'自 step={first_drop} pos=({fx:.3f},{fy:.3f},{fz:.3f}) 起），'
                        f'判定成片不可达，放弃本段')
                    self._q.put(('status',
                                 f'✗ 连续 {consec_drop} 点求不出解'
                                 f'（间隔 {gap*1000:.0f}ms > {MAX_GAP_SEC*1000:.0f}ms）'
                                 f'  自 step={first_drop}'
                                 f'  pos=({fx:.3f},{fy:.3f},{fz:.3f})'
                                 f'  → 路径超出可达域，已放弃（未下发）'))
                    return
                continue

            # 该点保留
            if joint_t:
                max_gap = max(max_gap, t_pt - joint_t[-1])
            if dp is not None:
                fk_checked += 1
                max_pos_err = max(max_pos_err, dp)
                max_ori_err = max(max_ori_err, do)
            consec_drop = 0
            joint_pos.append(sol)
            joint_t.append(t_pt)
            seed = sol

            if (idx + 1) % 20 == 0:
                self._q.put(('status', f'⚙ 规划中 {idx+1}/{n_ik}...'))

        if self._stop_req:
            return

        # ── 开拍不许甩：首点解离当前位形太远时，前面接一段关节空间平顺过渡 ──────
        # 相机挂在末端，开拍瞬间的高速甩动会直接毁掉这一段素材，实机上还可能撞。
        # 过渡段是额外时长，拍摄段本身时长不变（状态栏分开报，成片只取拍摄段）。
        ramp_t = 0.0
        if RAMP_IN_ENABLE and joint_pos:
            q_now = self._clamp_seed(self._joint_pos)
            d0    = max(abs(a - b) for a, b in zip(joint_pos[0], q_now))
            if d0 > RAMP_IN_TOL_RAD:
                ramp = self._plan_ramp_in(q_now, joint_pos[0])
                if ramp is None:
                    self.get_logger().error(
                        f'开拍过渡段规划失败（首点离当前位形 {d0:.3f}rad），放弃本段')
                    self._q.put(('status',
                                 f'✗ 开拍过渡段规划失败'
                                 f'（首点离当前位形 {d0:.3f}rad）  → 已放弃（未下发）'))
                    return
                ramp_t = ramp[-1][0]
                ramp   = ramp[:-1]      # 过渡末点与拍摄段首点重合，去掉避免零间隔
                joint_t   = [t + ramp_t for t in joint_t]
                joint_pos = [p for _, p in ramp] + joint_pos
                joint_t   = [t for t, _ in ramp] + joint_t
                self.get_logger().info(
                    f'首点离当前位形 {d0:.3f}rad > {RAMP_IN_TOL_RAD}rad，'
                    f'插入 {ramp_t:.2f}s 过渡段（{len(ramp)} 点）后再接拍摄段')

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
            pt           = JointTrajectoryPoint()
            pt.positions = joint_pos[i]
            pt.velocities = jvel[i]
            ns = int(joint_t[i] * 1e9)
            pt.time_from_start = Duration(sec=ns // 1_000_000_000,
                                          nanosec=ns % 1_000_000_000)
            msg.points.append(pt)
        self._traj_pub.publish(msg)
        # 轨迹保真度必须写在界面上：剔了几点、保留点最大间隔、FK 回代最大偏差。
        # 只打终端 WARN 的话，状态栏一句"执行中"会让人以为轨迹是干净的。
        note = ''
        if dropped:
            note += (f'  ⚠ 剔除 {dropped} 点（最大间隔 {max_gap*1000:.0f}ms，'
                     f'JTC 在保留点间插值）')
        if FK_VERIFY and fk_checked:
            note += (f'  FK 校验 {fk_checked} 点 ≤{max_pos_err*1000:.2f}mm/'
                     f'{math.degrees(max_ori_err):.2f}°')
        elif FK_VERIFY:
            # 一个点都没校验成功：不能印 ≤0.00mm/0.00°，那会被读成"校验完美通过"
            note += '  ⚠ FK 未校验（/compute_fk 无响应，末端邻域无凭据）'
        seg = (f'（过渡 {ramp_t:.2f}s + 拍摄 {t_traj:.2f}s）' if ramp_t > 0 else '')
        self._q.put(('status',
                     f'● 执行中  {n} 个路点  时长 {joint_t[-1]:.2f}s{seg}'
                     f'  {desc}{note}'))

    # ── 点到点 ────────────────────────────────────────────────────────────────
    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(target=self._plan_ptp,
                         args=(x, y, z, roll_deg, pitch_deg, yaw_deg,
                               v_pos, a_pos, j_pos, v_ori, a_ori, j_ori),
                         daemon=True).start()

    def go_ready(self, v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        p = READY_POSE
        self.send_goal(p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw'],
                       v_pos, a_pos, j_pos, v_ori, a_ori, j_ori)

    def _plan_ptp(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                  v_pos, a_pos, j_pos, v_ori, a_ori, j_ori):
        self._planning = True
        try:
            start = self.get_ee_pose();  q_cur = self.get_ee_quat()
            if start is None or q_cur is None:
                self._q.put(('status', '✗ TF 未就绪')); return
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
            inp = InputParameter(4);  out = OutputParameter(4)
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
                    self._q.put(('status', '✗ Ruckig 求解失败')); return

            dist = math.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)
            self._solve_and_send(
                all_pts, f'Δs={dist*1000:.1f}mm  Δθ={math.degrees(theta_total):.1f}°')
        finally:
            self._planning = False

    # ── 环绕 ──────────────────────────────────────────────────────────────────
    def send_orbit(self, cx, cy, z_sub, radius, h_cam,
                   theta0_deg, theta1_deg, w_max, a_max, j_max,
                   roll_deg=0.):
        """
        cx, cy    : 被摄主体 XY 坐标
        z_sub     : 被摄主体高度（base_link z）
        radius    : 轨道半径
        h_cam     : 相机高于主体的距离（h_cam > 0 = 相机在主体上方）
        theta0/1  : 轨道起止角（度）
        """
        if self._planning:
            self._q.put(('status', '⚠ 规划中，请稍候或先停止')); return
        self._stop_req = False
        threading.Thread(target=self._plan_orbit,
                         args=(cx, cy, z_sub, radius, h_cam,
                               theta0_deg, theta1_deg, w_max, a_max, j_max,
                               roll_deg),
                         daemon=True).start()

    def _plan_orbit(self, cx, cy, z_sub, radius, h_cam,
                    theta0_deg, theta1_deg, w_max, a_max, j_max,
                    roll_deg):
        self._planning = True
        try:
            theta0    = math.radians(theta0_deg)
            theta1    = math.radians(theta1_deg)
            roll_rad  = math.radians(roll_deg)
            pitch_rad = math.atan2(h_cam, radius)
            z_cam     = z_sub + h_cam

            otg = Ruckig(1, STREAM_DT)
            inp = InputParameter(1);  out = OutputParameter(1)
            inp.current_position     = [theta0]
            inp.current_velocity     = [0.]
            inp.current_acceleration = [0.]
            inp.target_position      = [theta1]
            inp.target_velocity      = [0.]
            inp.target_acceleration  = [0.]
            inp.max_velocity         = [w_max]
            inp.max_acceleration     = [a_max]
            inp.max_jerk             = [j_max]

            all_pts = [];  t_acc = 0.
            while True:
                res = otg.update(inp, out);  t_acc += STREAM_DT
                theta = out.new_position[0]
                px = cx + radius * math.cos(theta + math.pi)
                py = cy + radius * math.sin(theta + math.pi)
                # yaw=θ 跟踪轨道角，roll/pitch 由用户固定
                qx, qy, qz, qw = rpy_to_quat(roll_rad, pitch_rad, theta)
                all_pts.append((t_acc, px, py, z_cam, qx, qy, qz, qw))
                out.pass_to_input(inp)
                if res == Result.Finished: break
                if res == Result.Error:
                    self._q.put(('status', '✗ Ruckig 求解失败')); return

            sweep = abs(theta1_deg - theta0_deg)
            self._solve_and_send(
                all_pts,
                f'r={radius:.2f}m  h={h_cam:+.2f}m  '
                f'θ:{theta0_deg:.0f}°→{theta1_deg:.0f}°({sweep:.0f}°)')
        finally:
            self._planning = False


def main():
    rclpy.init()
    node     = CartesianTrajectoryControllerNode(queue.Queue())
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
