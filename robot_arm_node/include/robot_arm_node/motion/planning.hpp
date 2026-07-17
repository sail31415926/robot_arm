/**
 * @file planning.hpp
 * @brief motion 层轨迹规划（C++）—— 纯 Ruckig 路点生成，无 ROS 依赖
 *
 * 对应 Python arm_motion/planning.py。plan_orbit_waypoints / plan_line_waypoints 用
 * Ruckig 1-DOF 对归一化路径参数 s∈[0,1] 做 jerk-limited 规划，再插值（球坐标+朝向球心 /
 * 直线+姿态 slerp），返回统一路点 (t, x,y,z, 四元数)，交给 trajectory 求 IK 下发。
 * stop_check 可中途放弃。
 *
 * @version 1.1
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "robot_arm_node/motion/geometry.hpp"   // Quat

namespace robot_arm_node::motion
{

// 统一路点：(t 秒, x,y,z 米, 四元数 qx,qy,qz,qw)
struct Waypoint
{
  double t{0.0};
  double x{0.0}, y{0.0}, z{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

// 球面环绕运镜路点：Ruckig 1-DOF(s∈[0,1]) → 球坐标插值 → Cartesian + 朝向球心。
//   stop_check: 可选，返回 true 时中途放弃（急停/取消）。
//   返回 std::nullopt 表示 Ruckig 求解失败或被中止。
std::optional<std::vector<Waypoint>> plan_orbit_waypoints(
    double ox, double oy, double oz,
    double theta0, double phi0, double r0,
    double theta1, double phi1, double r1,
    double s_vel, double s_acc, double s_jerk,
    const std::function<bool()> & stop_check = nullptr);

// 笛卡尔直线运镜路点：Ruckig 1-DOF(s∈[0,1]) → 位置沿线插值 + 姿态 slerp（末端严格走直线）。
// 归一化限制取位置/姿态两组约束的更严者（limit/extent 最小值），位移与转角同时≈0 时返回单路点。
//   stop_check: 可选，返回 true 时中途放弃（急停/取消）。
//   返回 std::nullopt 表示 Ruckig 求解失败或被中止。
std::optional<std::vector<Waypoint>> plan_line_waypoints(
    double x0, double y0, double z0, const Quat & q0,
    double x1, double y1, double z1, const Quat & q1,
    double v_pos, double a_pos, double j_pos,
    double v_ori, double a_ori, double j_ori,
    const std::function<bool()> & stop_check = nullptr);

}  // namespace robot_arm_node::motion
