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
/**
 * @brief 按档位键取笛卡尔速度档位（位置与姿态两组限制）。
 *
 * @param k 档位键：0=SLOW、1=NORMAL、2=FAST，非法值落 NORMAL。
 * @return 对应档位的速度限制（引用，指向 tuning 里的单一来源）。
 */
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
/**
 * @brief 按档位键取关节空间**平均**角速度。
 *
 * 与笛卡尔档位分开：Speed 里全是末端量（m/s、rad/s），关节点到点用不上。
 *
 * @param k 档位键：0=SLOW、1=NORMAL、2=FAST。
 * @return 平均角速度（rad/s）。
 */
inline double joint_speed_profile(uint8_t k)
{
  return tuning::params().joint_speed_rps(k);
}

// 按档位计算关节空间点到点时长：max|Δq| / v_档位，夹在 [MIN, MAX] 内。
// cur / tgt 长度不一致时按较短者比较（调用方应已保证长度一致）。
/**
 * @brief 按档位计算关节空间点到点的轨迹时长。
 *
 * 时长 = max|Δq| / 档位角速度，再夹到 [min, max] 区间内。取最大位移轴而非
 * 各轴分别算，保证所有轴同步到达（JTC 对每个轴用同一时长做插值）。
 *
 * @param cur 当前关节角（rad）。
 * @param tgt 目标关节角（rad）。
 * @param speed_key 档位键。
 * @return 轨迹时长（s），已夹在配置的上下限内。
 */
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
/**
 * @brief 两角度之差（度），考虑 ±180° 环绕。
 *
 * @param a 角度 a（度）。
 * @param b 角度 b（度）。
 * @return 最小夹角，范围 [0, 180]。
 */
inline double angular_diff(double a, double b)
{
  const double d = std::fmod(std::fabs(a - b), 360.0);
  return d <= 180.0 ? d : 360.0 - d;
}

// 末端是否到达目标（位置 + 姿态双容差）
/**
 * @brief 判断末端是否到达目标位姿（位置与姿态双容差）。
 *
 * @warning 笛卡尔动作**不要**用它做到位判据，改用 is_at_joints_prefix ——
 *          原因见该函数上方注释。本函数只留作拿不到关节解时的退化兜底。
 *
 * @param c 当前位姿。
 * @param t 目标位姿。
 * @param pos_tol 位置容差（m），默认取 tuning 配置值。
 * @param ori_tol 姿态容差（度），默认取 tuning 配置值。
 * @return 位置与姿态都在容差内返回 true。
 */
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
/**
 * @brief 判断全部关节是否到位（逐轴容差）。
 *
 * 长度不等直接返回 false —— 长度不一致说明调用方传错了，不该猜。
 *
 * @param c 当前关节角（rad）。
 * @param t 目标关节角（rad）。
 * @param tol 逐轴容差（rad），默认取 tuning 配置值。
 * @return 每一轴都在容差内返回 true。
 */
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
/**
 * @brief 只判前 n 个关节是否到位（n = ARM_JOINT_COUNT 即「只判臂 J1-3」）。
 *
 * 这是所有笛卡尔动作到位判据的**标准做法**，理由见函数上方的长注释：
 * 末端 gimbal_tool0 在云台 J4-6 之后，云台回读不收敛会让 is_at_pose 永不满足。
 *
 * @param c 当前关节角（rad），长度须 ≥ n。
 * @param t 目标关节角（rad），长度须 ≥ n。
 * @param n 参与判定的关节个数（前缀长度）。
 * @param tol 逐轴容差（rad），默认取 tuning 配置值。
 * @return 前 n 轴都在容差内返回 true；任一输入长度不足返回 false。
 */
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
