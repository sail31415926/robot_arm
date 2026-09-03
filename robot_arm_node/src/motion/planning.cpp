/**
 * @file planning.cpp
 * @brief plan_orbit_waypoints / plan_line_waypoints 实现 —— Ruckig 1-DOF 路点生成
 *
 * 对归一化路径参数 s∈[0,1] 做 Ruckig jerk-limited 规划：orbit 逐步插值 (theta,phi,r) →
 * sphere_to_cart + aim_quat 朝向球心；line 位置沿线插值 + 姿态 slerp（末端严格直线）。
 * 输出统一路点。stop_check 中途放弃。
 *
 * @version 1.1
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

namespace
{
// 四元数球面插值（取最短路径），u∈[0,1]
Quat quat_slerp(const Quat & a, Quat b, double u)
{
  double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0.0) {                 // 反号取短弧
    for (auto & c : b) c = -c;
    dot = -dot;
  }
  if (dot > 0.9995) {              // 夹角极小：线性插值 + 归一化，避免 sin(θ)≈0
    Quat r{a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]),
           a[2] + u * (b[2] - a[2]), a[3] + u * (b[3] - a[3])};
    const double n = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
    for (auto & c : r) c /= n;
    return r;
  }
  const double th = std::acos(std::clamp(dot, -1.0, 1.0));
  const double s0 = std::sin((1.0 - u) * th) / std::sin(th);
  const double s1 = std::sin(u * th) / std::sin(th);
  return {s0 * a[0] + s1 * b[0], s0 * a[1] + s1 * b[1],
          s0 * a[2] + s1 * b[2], s0 * a[3] + s1 * b[3]};
}
}  // namespace

std::optional<std::vector<Waypoint>> plan_orbit_waypoints(
    double ox, double oy, double oz,
    double theta0, double phi0, double r0,
    double theta1, double phi1, double r1,
    double v_pos, double a_pos, double j_pos,
    double v_ori, double a_ori, double j_ori,
    const std::function<bool()> & stop_check)
{
  const double d_theta = theta1 - theta0;
  const double d_phi   = phi1 - phi0;
  const double d_r     = r1 - r0;

  // 归一化 s∈[0,1] 的限制（与 plan_line_waypoints 同一套做法）：位置约束除以位置行程
  // (m)、姿态约束除以姿态角跨度 (rad)，各自得到无量纲的 ds/dt 上限后取更严者。
  // 两组约束必须分别用**同量纲**的行程归一化 —— 两次实机踩坑：
  //  1) 原实现直接透传物理量（execute_orbit 传的是 speed.v_ori/a_ori/j_ori，单位
  //     rad/s、rad/s²、rad/s³），Ruckig 把它们当无量纲 ds/dt 用 → 环绕时长恒为
  //     1/v_ori（与跨度无关），实际角速度与角加速度 ∝ 跨度，跨度越大启动越猛。
  //     2026-08-31 实测：SLOW 档跨度 30° 与 120° 的轨迹均为 points=689 /
  //     horizon=20.800s，而 J3 指令速度峰值差 4.94 倍。
  //  2) 只按 d_ang 归一化、仍只喂姿态一组：纯径向推拉（d_ang≈0）退化为拿
  //     v_ori(rad/s) 去除弧长(m)，量纲不一致；带径向分量的环绕也只受姿态约束，
  //     末端线速度可超 v_pos 数倍（SLOW 档纯径向推拉实为 0.05m/s，限值 0.02）。
  constexpr double EPS = 1e-6;
  const double d_ang = std::sqrt(d_theta * d_theta + d_phi * d_phi);   // 姿态角跨度(rad)
  const double r_avg = 0.5 * (r0 + r1);                                // 相机始终朝球心 ->
  const double arc   = std::sqrt(r_avg * d_ang * r_avg * d_ang + d_r * d_r);  // 位置行程(m)

  if (arc < EPS && d_ang < EPS) {   // 起止重合：单路点直接收尾（与 plan_line 一致）
    const Vec3 p = sphere_to_cart(theta1, phi1, r1, ox, oy, oz);
    const Quat q = aim_quat(p[0], p[1], p[2], ox, oy, oz);
    return std::vector<Waypoint>{
        Waypoint{STREAM_DT, p[0], p[1], p[2], q[0], q[1], q[2], q[3]}};
  }

  double n_vel = 1e9, n_acc = 1e9, n_jerk = 1e9;
  if (arc > EPS) {              // 位置：m/s ÷ m -> 1/s
    n_vel  = std::min(n_vel,  v_pos / arc);
    n_acc  = std::min(n_acc,  a_pos / arc);
    n_jerk = std::min(n_jerk, j_pos / arc);
  }
  if (d_ang > EPS) {            // 姿态：rad/s ÷ rad -> 1/s
    n_vel  = std::min(n_vel,  v_ori / d_ang);
    n_acc  = std::min(n_acc,  a_ori / d_ang);
    n_jerk = std::min(n_jerk, j_ori / d_ang);
  }

  ruckig::Ruckig<1> otg{STREAM_DT};
  ruckig::InputParameter<1> inp;
  ruckig::OutputParameter<1> out;
  inp.current_position     = {0.0};
  inp.current_velocity     = {0.0};
  inp.current_acceleration = {0.0};
  inp.target_position      = {1.0};
  inp.target_velocity      = {0.0};
  inp.target_acceleration  = {0.0};
  inp.max_velocity         = {n_vel};
  inp.max_acceleration     = {n_acc};
  inp.max_jerk             = {n_jerk};

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

std::optional<std::vector<Waypoint>> plan_line_waypoints(
    double x0, double y0, double z0, const Quat & q0,
    double x1, double y1, double z1, const Quat & q1,
    double v_pos, double a_pos, double j_pos,
    double v_ori, double a_ori, double j_ori,
    const std::function<bool()> & stop_check)
{
  const double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  // 起止姿态夹角（最短路径，|dot| 消除双倍覆盖号差）
  const double dot = std::fabs(q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]);
  const double ang = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));

  constexpr double EPS = 1e-6;
  if (dist < EPS && ang < EPS) {   // 起止重合：单路点直接收尾
    return std::vector<Waypoint>{Waypoint{STREAM_DT, x1, y1, z1, q1[0], q1[1], q1[2], q1[3]}};
  }

  // 归一化 s∈[0,1] 的限制 = 位置/姿态两组约束的更严者（limit / extent）
  double s_vel = 1e9, s_acc = 1e9, s_jerk = 1e9;
  if (dist > EPS) {
    s_vel  = std::min(s_vel,  v_pos / dist);
    s_acc  = std::min(s_acc,  a_pos / dist);
    s_jerk = std::min(s_jerk, j_pos / dist);
  }
  if (ang > EPS) {
    s_vel  = std::min(s_vel,  v_ori / ang);
    s_acc  = std::min(s_acc,  a_ori / ang);
    s_jerk = std::min(s_jerk, j_ori / ang);
  }

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

    const Quat q = quat_slerp(q0, q1, s);
    pts.push_back(Waypoint{t_acc, x0 + s * dx, y0 + s * dy, z0 + s * dz,
                           q[0], q[1], q[2], q[3]});

    out.pass_to_input(inp);
    if (res == ruckig::Result::Finished) break;
    if (res == ruckig::Result::Error) return std::nullopt;
  }
  return pts;
}

}  // namespace robot_arm_node::motion
