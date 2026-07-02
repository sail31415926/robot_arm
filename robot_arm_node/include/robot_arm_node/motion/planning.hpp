/**
 * @file planning.hpp
 * @brief motion 层轨迹规划（C++）—— 纯 Ruckig 路点生成，无 ROS 依赖
 *
 * 对应 Python arm_motion/planning.py。plan_orbit_waypoints 用 Ruckig 1-DOF 对归一化
 * 路径参数 s∈[0,1] 做 jerk-limited 规划，再插值球坐标 → Cartesian + 朝向球心，返回统一
 * 路点 (t, x,y,z, 四元数)，交给 trajectory 求 IK 下发。stop_check 可中途放弃。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <optional>
#include <vector>

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

}  // namespace robot_arm_node::motion
