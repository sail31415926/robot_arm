/**
 * @file motion_policy.hpp
 * @brief commander 运动策略（C++, header-only）—— 到位容差/判定 + 速度档位限制（单一来源）
 *
 * 对应 Python commander/motion_policy.py。到位判据（is_at_pose / is_at_joints / angular_diff）
 * 与速度档位「值」（SLOW / NORMAL / FAST）的唯一来源，被 move_to_pose / trajectory_shot
 * server 共用；配合 execution_monitor（统一等待循环）消除各 server 的重复到位逻辑。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <robot_arm_interfaces/msg/arm_pose.hpp>

namespace robot_arm_node::commander
{

using ArmPose = robot_arm_interfaces::msg::ArmPose;

// ── 到位容差 ────────────────────────────────────────────────────────────────
constexpr double POSITION_TOLERANCE_M      = 0.01;   // 位置容差（米）
constexpr double ORIENTATION_TOLERANCE_DEG = 2.0;    // 姿态容差（度）
constexpr double JOINT_TOLERANCE_RAD       = 0.02;   // 关节容差（rad）

// ── 速度档位限制（值单一来源）────────────────────────────────────────────────
struct Speed
{
  double v_pos, a_pos, j_pos;   // 位置：速度/加速/加加速
  double v_ori, a_ori, j_ori;   // 姿态：速度/加速/加加速
};
inline constexpr Speed SPEED_SLOW_LIMITS  {0.02, 0.05, 0.50, 0.05, 0.10, 1.00};
inline constexpr Speed SPEED_NORMAL_LIMITS{0.05, 0.10, 1.00, 0.10, 0.20, 2.00};
inline constexpr Speed SPEED_FAST_LIMITS  {0.10, 0.20, 2.00, 0.20, 0.40, 4.00};

// ArmMoveToPose / ArmTrajectoryShot 的 SPEED_SLOW/NORMAL/FAST 取值一致（0/1/2），
// 故此处只按数值映射（对应 Python 各 server 的 SPEED_PROFILES）。
inline const Speed & speed_profile(uint8_t k)
{
  switch (k) {
    case 0:  return SPEED_SLOW_LIMITS;
    case 2:  return SPEED_FAST_LIMITS;
    default: return SPEED_NORMAL_LIMITS;   // 含 SPEED_NORMAL(1) 与非法值
  }
}

// ── 到位判定 ────────────────────────────────────────────────────────────────
// 两角度之差（度），考虑 ±180° 环绕，返回 [0,180]
inline double angular_diff(double a, double b)
{
  const double d = std::fmod(std::fabs(a - b), 360.0);
  return d <= 180.0 ? d : 360.0 - d;
}

// 末端是否到达目标（位置 + 姿态双容差）
inline bool is_at_pose(const ArmPose & c, const ArmPose & t,
                       double pos_tol = POSITION_TOLERANCE_M,
                       double ori_tol = ORIENTATION_TOLERANCE_DEG)
{
  const bool pos_ok = std::fabs(c.x - t.x) < pos_tol &&
                      std::fabs(c.y - t.y) < pos_tol &&
                      std::fabs(c.z - t.z) < pos_tol;
  const bool ori_ok = angular_diff(c.roll,  t.roll)  < ori_tol &&
                      angular_diff(c.pitch, t.pitch) < ori_tol &&
                      angular_diff(c.yaw,   t.yaw)   < ori_tol;
  return pos_ok && ori_ok;
}

// 所有关节是否到位（逐轴容差）
inline bool is_at_joints(const std::vector<double> & c, const std::vector<double> & t,
                         double tol = JOINT_TOLERANCE_RAD)
{
  if (c.size() != t.size()) return false;
  for (size_t i = 0; i < c.size(); ++i) {
    if (std::fabs(c[i] - t[i]) > tol) return false;
  }
  return true;
}

}  // namespace robot_arm_node::commander
