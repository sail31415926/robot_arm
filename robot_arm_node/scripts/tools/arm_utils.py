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
