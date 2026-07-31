#!/usr/bin/env python3
"""
@file   arm_utils.py
@brief  eMeetArm 通用四元数数学工具
@date   2026-06-04

提供 rpy_to_quat / quat_to_rpy / quat_normalize / quat_dot / quat_slerp，
安装到 lib/robot_arm_node/arm_utils.py，供各控制脚本通过 `from arm_utils import ...` 引入。

若系统已安装 ros-humble-tf-transformations，可将 rpy_to_quat / quat_to_rpy
替换为：
    from tf_transformations import quaternion_from_euler, euler_from_quaternion
"""

import math


def rpy_to_quat(roll, pitch, yaw):
    """RPY (rad) → (qx, qy, qz, qw)，ZYX 内旋 / XYZ 外旋约定。"""
    cr = math.cos(roll  * 0.5); sr = math.sin(roll  * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw   * 0.5); sy = math.sin(yaw   * 0.5)
    return (sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy)


def quat_to_rpy(x, y, z, w):
    """(qx, qy, qz, qw) → (roll, pitch, yaw) rad。"""
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def quat_normalize(q):
    """归一化四元数，返回 (qx, qy, qz, qw) tuple。"""
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n) if n > 1e-12 else (0., 0., 0., 1.)


def quat_dot(q1, q2):
    """四元数点积（标量）。"""
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]


def quat_mul(q1, q2):
    """四元数乘法 q1 * q2（body-frame 叠加旋转）。"""
    x1, y1, z1, w1 = q1;  x2, y2, z2, w2 = q2
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def quat_slerp(q0, q1, t):
    """球面线性插值，t ∈ [0, 1]，自动选最短路径。"""
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


# ── 球坐标 / 相机朝向工具（供 spherical_orbit_controller / trajectory_shot_server 等使用）──

# 画面水平所需的 EEF roll（rad）——由「EEF 系 → 相机光学系」的固定安装旋转决定。
#
# 推导：设 R_c = R(EEF→光学系)。画面水平 ⇔ 图像右向量（光学 +X）水平
#       ⇔ (R_eef · R_c)[2,0] == 0，其中 R_eef = Rz(yaw)Ry(pitch)Rx(roll)。
#       该式对任意 pitch/yaw 成立的 roll 就是本常量。
#
#   V1（tool0 → camera_optical_frame，rpy = 0, -π/2, π）    → roll = π/2  ✔ 老代码
#   V2（gimbal_tool0 → Cam0，       rpy = -π/2, 0, -π/2）   → roll = 0
#
# 2026-07-29：云台换 V2 后仍沿用 π/2，导致环绕运镜画面歪斜 90°，且逼云台 roll 轴
# （Joint5，±1.5 rad）去凑该姿态 → IK 大面积无解。故改为 0。
# 两代的光轴都是 EEF 的 +X 轴，所以下面 pitch/yaw 的算法不变。
# 若以后改相机安装姿态，重新按上式解一次 roll 即可（0 和 π 都能让画面水平，
# 差别是画面上下翻转；取 0）。
EEF_LEVEL_ROLL = 0.0


def aim_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z):
    """EEF X 轴朝向目标（= 相机光轴指向目标），roll 取 EEF_LEVEL_ROLL 保证画面水平。"""
    dx, dy, dz = tgt_x - cam_x, tgt_y - cam_y, tgt_z - cam_z
    n = math.sqrt(dx*dx + dy*dy + dz*dz)
    if n < 1e-9:
        return rpy_to_quat(EEF_LEVEL_ROLL, 0., 0.)
    dx, dy, dz = dx/n, dy/n, dz/n
    pitch = math.asin(max(-1., min(1., -dz)))
    yaw   = math.atan2(dy, dx)
    return rpy_to_quat(EEF_LEVEL_ROLL, pitch, yaw)


def look_at_quat(cam_x, cam_y, cam_z, tgt_x, tgt_y, tgt_z):
    """EEF Z 轴从相机位置指向目标（look-at），world Z-up hint。返回 (qx,qy,qz,qw)。

    ⚠ 本机器人的相机光轴是 EEF 的 **+X** 轴，不是 +Z —— 直接拿本函数的结果当
    「相机对准目标」会差 90°。当前无调用点（仅被 import）。要做对准请用 aim_quat。
    """
    dx, dy, dz = tgt_x - cam_x, tgt_y - cam_y, tgt_z - cam_z
    n = math.sqrt(dx*dx + dy*dy + dz*dz)
    if n < 1e-9:
        return (0., 0., 0., 1.)
    zx, zy, zz = dx/n, dy/n, dz/n
    ux, uy, uz = (0., 0., 1.) if abs(zz) < 0.999 else (1., 0., 0.)
    xx = zy*uz - zz*uy;  xy = zz*ux - zx*uz;  xz = zx*uy - zy*ux
    xn = math.sqrt(xx*xx + xy*xy + xz*xz)
    xx, xy, xz = xx/xn, xy/xn, xz/xn
    yx = xy*zz - xz*zy;  yy = xz*zx - xx*zz;  yz = xx*zy - xy*zx
    R = [[xx, yx, zx], [xy, yy, zy], [xz, yz, zz]]
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2][1] - R[1][2]) * s;  qy = (R[0][2] - R[2][0]) * s
        qz = (R[1][0] - R[0][1]) * s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        qw = (R[2][1] - R[1][2]) / s;  qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s;  qz = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        qw = (R[0][2] - R[2][0]) / s;  qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s;  qz = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        qw = (R[1][0] - R[0][1]) / s;  qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s;  qz = 0.25 * s
    return quat_normalize((qx, qy, qz, qw))


def _theta_ref(ox, oy):
    """主体→世界原点在 XY 平面的方位角，作为 θ=0 的参考方向（近侧）。"""
    return math.atan2(-oy, -ox)


def sphere_to_cart(theta_rad, phi_rad, r, ox, oy, oz):
    """Z-up 球坐标 → 世界系笛卡尔位置。

    theta=0  : 相机在近侧（主体朝向世界原点方向）
    theta=±π : 相机在远侧
    phi=+π/2 : 正上方，phi=-π/2 : 正下方
    """
    theta_world = _theta_ref(ox, oy) + theta_rad
    cp = math.cos(phi_rad)
    return (
        ox + r * cp * math.cos(theta_world),
        oy + r * cp * math.sin(theta_world),
        oz + r * math.sin(phi_rad),
    )


def cart_to_sphere(px, py, pz, ox, oy, oz):
    """笛卡尔位置 → Z-up 球坐标（θ 以近侧为 0）。返回 (theta_rad, phi_rad, r)。"""
    dx, dy, dz = px - ox, py - oy, pz - oz
    r = math.sqrt(dx*dx + dy*dy + dz*dz)
    if r < 1e-9:
        return 0., 0., 0.
    phi   = math.asin(max(-1., min(1., dz / r)))
    theta = math.atan2(dy, dx) - _theta_ref(ox, oy)
    theta = (theta + math.pi) % (2 * math.pi) - math.pi
    return theta, phi, r
