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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <robot_arm_interfaces/msg/arm_pose.hpp>

#include "robot_arm_node/tuning.hpp"

namespace robot_arm_node::commander
{

using ArmPose = robot_arm_interfaces::msg::ArmPose;

// ── 到位容差 / 速度档位「值」──────────────────────────────────────────────────
// 2026-08-12：这些量从本文件的 constexpr 挪进了 tuning::params()，由
// robot_arm_bringup/config/arm_params.yaml 配置（实机标定要动的就是它们，
// 原来改一个数得重编）。本文件保留「判据与档位映射的唯一来源」这个职责，
// 只是数值改为运行期读取 —— 下面所有函数的名字与签名都没变，调用点无需改。
using Speed = tuning::Speed;

// ArmMoveToPose / ArmTrajectoryShot 的 SPEED_SLOW/NORMAL/FAST 取值一致（0/1/2），
// 故只按数值映射（非法值落 NORMAL）。
inline const Speed & speed_profile(uint8_t k)
{
  return tuning::params().speed(k);
}

// ── 关节空间速度档位（rad/s，**平均**角速度）─────────────────────────────────
// 用途：ArmMoveToJoint 由「最大关节位移 / 档位角速度」反算两点轨迹的时长。
// 上面的 Speed 全是笛卡尔量（m/s、rad/s 的末端速度），关节空间点到点用不上，故单列。
// 峰值角速度：两点 JointTrajectory 由 JTC 做五次多项式插值，峰值 ≈ 1.875 × 平均值，
//   故 FAST 的峰值 ≈ 2.25 rad/s，仍低于 URDF 里 J1-3 的 velocity=3.14 rad/s 机械限。
// 量级校准：FAST 走满行程（J1 的 ±2.618）约 2.2 s，与既有 STOWED 回零的固定 2.0 s 相当。
// 档位角速度与时长上下限同样移入 tuning（joint_speed.* / posture.*）。
inline double joint_speed_profile(uint8_t k)
{
  return tuning::params().joint_speed_rps(k);
}

// 按档位计算关节空间点到点时长：max|Δq| / v_档位，夹在 [MIN, MAX] 内。
// cur / tgt 长度不一致时按较短者比较（调用方应已保证长度一致）。
inline double joint_move_duration(const std::vector<double> & cur,
                                 const std::vector<double> & tgt, uint8_t speed_key)
{
  double max_delta = 0.0;
  const size_t n = std::min(cur.size(), tgt.size());
  for (size_t i = 0; i < n; ++i) {
    max_delta = std::max(max_delta, std::fabs(tgt[i] - cur[i]));
  }
  const double dur = max_delta / joint_speed_profile(speed_key);
  const auto & tp = tuning::params();
  return std::min(std::max(dur, tp.joint_min_duration_sec), tp.joint_max_duration_sec);
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
                       double pos_tol = tuning::params().position_tolerance_m,
                       double ori_tol = tuning::params().orientation_tolerance_deg)
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
                         double tol = tuning::params().joint_tolerance_rad)
{
  if (c.size() != t.size()) return false;
  for (size_t i = 0; i < c.size(); ++i) {
    if (std::fabs(c[i] - t[i]) > tol) return false;
  }
  return true;
}

// 只判前 n 个关节是否到位（n = motion::ARM_JOINT_COUNT 时即「只判臂 J1-3」）。
//
// ★ 为什么所有笛卡尔动作的到位判据都必须走这条、而不是 is_at_pose ★
// 2026-07-28 云台换 V2 后规划末端是 gimbal_tool0，它在**云台 J4-6 之后**：
// 末端位姿 = f(J1..J6)。云台的回读一旦不收敛或有静差（板端未上电、跨机 DDS 不通、
// GCU 自稳环让 IMU 角与关节指令不完全一致），is_at_pose(status_.pose(), target)
// 就永远不满足 → 动作全部走到 timeout 报错，哪怕机械臂本身早就到位了。
// 规划/下发仍是 6 轴（云台跟着一起动），只是**判据只认臂**，这样云台不拖累成败。
inline bool is_at_joints_prefix(const std::vector<double> & c, const std::vector<double> & t,
                               size_t n, double tol = tuning::params().joint_tolerance_rad)
{
  if (c.size() < n || t.size() < n) return false;
  for (size_t i = 0; i < n; ++i) {
    if (std::fabs(c[i] - t[i]) > tol) return false;
  }
  return true;
}

}  // namespace robot_arm_node::commander
