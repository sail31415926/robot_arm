/**
 * @file planning.cpp
 * @brief plan_orbit_waypoints 实现 —— Ruckig 1-DOF 球面轨道路点生成
 *
 * 对归一化路径参数 s∈[0,1] 做 Ruckig jerk-limited 规划，逐步插值 (theta,phi,r) →
 * sphere_to_cart 求相机位置 + aim_quat 求朝向球心姿态，输出统一路点。stop_check 中途放弃。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/motion/planning.hpp"

#include <algorithm>

#include <ruckig/ruckig.hpp>

#include "robot_arm_node/motion/constants.hpp"
#include "robot_arm_node/motion/geometry.hpp"

namespace robot_arm_node::motion
{

std::optional<std::vector<Waypoint>> plan_orbit_waypoints(
    double ox, double oy, double oz,
    double theta0, double phi0, double r0,
    double theta1, double phi1, double r1,
    double s_vel, double s_acc, double s_jerk,
    const std::function<bool()> & stop_check)
{
  const double d_theta = theta1 - theta0;
  const double d_phi   = phi1 - phi0;
  const double d_r     = r1 - r0;

  ruckig::Ruckig<1> otg{STREAM_DT};
  ruckig::InputParameter<1> inp;
  ruckig::OutputParameter<1> out;
  inp.current_position     = {0.0};
  inp.current_velocity     = {0.0};
  inp.current_acceleration = {0.0};
  inp.target_position      = {1.0};
  inp.target_velocity      = {0.0};
  inp.target_acceleration  = {0.0};
  inp.max_velocity         = {s_vel};
  inp.max_acceleration     = {s_acc};
  inp.max_jerk             = {s_jerk};

  std::vector<Waypoint> pts;
  double t_acc = 0.0;
  while (true) {
    if (stop_check && stop_check()) return std::nullopt;

    const ruckig::Result res = otg.update(inp, out);
    t_acc += STREAM_DT;
    const double s = std::clamp(out.new_position[0], 0.0, 1.0);

    const double theta = theta0 + s * d_theta;
    const double phi   = phi0 + s * d_phi;
    const double r     = r0 + s * d_r;

    const Vec3 p = sphere_to_cart(theta, phi, r, ox, oy, oz);
    const Quat q = aim_quat(p[0], p[1], p[2], ox, oy, oz);
    pts.push_back(Waypoint{t_acc, p[0], p[1], p[2], q[0], q[1], q[2], q[3]});

    out.pass_to_input(inp);
    if (res == ruckig::Result::Finished) break;
    if (res == ruckig::Result::Error) return std::nullopt;
  }
  return pts;
}

}  // namespace robot_arm_node::motion
