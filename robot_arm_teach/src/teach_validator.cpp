/**
 * @file teach_validator.cpp
 * @brief teach_validator.hpp 的实现
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/teach_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace robot_arm_teach
{

namespace
{
// 时间戳单调性的判据。相邻点时间完全相等会让 JTC 遇到零时长段（除零 / 无限加速度），
// 所以要求「严格」递增并留一个下限；1us 足够宽松，50/100Hz 采样差 4 个数量级。
constexpr double MIN_TIME_STEP = 1e-6;

bool finite6(const std::array<double, TEACH_JOINT_COUNT> & a)
{
  return std::all_of(a.begin(), a.end(), [](double v) { return std::isfinite(v); });
}

std::string fmt(double v)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(4);
  os << v;
  return os.str();
}
}  // namespace

ValidateResult validate_structure(const TeachTrajectoryMsg & traj,
                                 const std::vector<std::string> & expected_names)
{
  if (traj.format_version == 0 || traj.format_version > TeachTrajectoryMsg::FORMAT_VERSION) {
    return ValidateResult::failure(
        "invalid_trajectory",
        "轨迹格式版本 " + std::to_string(traj.format_version) + " 不受支持（本节点支持 1.." +
            std::to_string(TeachTrajectoryMsg::FORMAT_VERSION) + "）");
  }
  if (traj.points.empty()) {
    return ValidateResult::failure("invalid_trajectory", "轨迹为空（0 个采样点）");
  }
  if (traj.joint_names.size() != expected_names.size()) {
    return ValidateResult::failure(
        "invalid_trajectory",
        "关节数 " + std::to_string(traj.joint_names.size()) + " 不等于期望的 " +
            std::to_string(expected_names.size()));
  }
  for (size_t i = 0; i < expected_names.size(); ++i) {
    if (traj.joint_names[i] != expected_names[i]) {
      return ValidateResult::failure(
          "invalid_trajectory",
          "关节名不匹配：第 " + std::to_string(i) + " 个是「" + traj.joint_names[i] +
              "」，期望「" + expected_names[i] + "」（不做按名重排 —— 猜错就是走错轴）");
    }
  }

  double prev_t = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto & p = traj.points[i];
    if (!std::isfinite(p.time_from_start) || !finite6(p.positions) || !finite6(p.velocities)) {
      return ValidateResult::failure(
          "invalid_trajectory",
          "第 " + std::to_string(i) + " 个点含 NaN / inf（位置、速度或时间戳）");
    }
    if (p.time_from_start < 0.0) {
      return ValidateResult::failure(
          "invalid_trajectory",
          "第 " + std::to_string(i) + " 个点 time_from_start=" + fmt(p.time_from_start) +
              " 小于 0");
    }
    if (i > 0 && p.time_from_start <= prev_t + MIN_TIME_STEP) {
      return ValidateResult::failure(
          "invalid_trajectory",
          "时间戳非严格递增：第 " + std::to_string(i) + " 个点 " + fmt(p.time_from_start) +
              "s 不大于第 " + std::to_string(i - 1) + " 个点 " + fmt(prev_t) + "s");
    }
    prev_t = p.time_from_start;
  }
  return ValidateResult::success();
}

ValidateResult validate_limits(const TeachTrajectoryMsg & traj, const JointBoundMap & limits)
{
  // fail-open：URDF 未就绪（/robot_description 还没到）时放行，调用方负责告警。
  if (limits.empty()) return ValidateResult::success();

  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto & p = traj.points[i];
    for (size_t j = 0; j < traj.joint_names.size() && j < TEACH_JOINT_COUNT; ++j) {
      auto it = limits.find(traj.joint_names[j]);
      if (it == limits.end()) continue;   // 该轴 continuous / 无 limit 声明，无界不校验
      const double q = p.positions[j];
      if (q < it->second.lower || q > it->second.upper) {
        return ValidateResult::failure(
            "out_of_range",
            "第 " + std::to_string(i) + " 个点 " + traj.joint_names[j] + "=" + fmt(q) +
                " 超出 URDF 限位 [" + fmt(it->second.lower) + ", " + fmt(it->second.upper) + "]");
      }
    }
  }
  return ValidateResult::success();
}

ValidateResult validate_speed(const TeachTrajectoryMsg & traj, const MotionCaps & caps,
                              double speed_scale)
{
  const Envelope env = scale_envelope(compute_envelope(traj), speed_scale);
  const auto & names = traj.joint_names;

  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    const std::string jn = j < names.size() ? names[j] : ("Joint" + std::to_string(j + 1));
    if (caps.max_velocity > 0.0 && env.max_velocity[j] > caps.max_velocity) {
      return ValidateResult::failure(
          "over_speed",
          jn + " 速度峰值 " + fmt(env.max_velocity[j]) + " rad/s 超过上限 " +
              fmt(caps.max_velocity) + " rad/s（倍率 " + fmt(speed_scale) + "）");
    }
    if (caps.max_acceleration > 0.0 && env.max_acceleration[j] > caps.max_acceleration) {
      return ValidateResult::failure(
          "over_speed",
          jn + " 加速度峰值 " + fmt(env.max_acceleration[j]) + " rad/s2 超过上限 " +
              fmt(caps.max_acceleration) + " rad/s2（倍率 " + fmt(speed_scale) + "）");
    }
  }
  return ValidateResult::success();
}

bool is_at_start(const TeachTrajectoryMsg & traj, const std::vector<double> & current_positions,
                 double tolerance_rad, size_t arm_joint_count, double * max_deviation)
{
  if (max_deviation) *max_deviation = 0.0;
  if (traj.points.empty()) return false;

  const size_t n = std::min({arm_joint_count, current_positions.size(), TEACH_JOINT_COUNT});
  if (n == 0) return false;

  const auto & start = traj.points.front().positions;
  double worst = 0.0;
  for (size_t j = 0; j < n; ++j) {
    worst = std::max(worst, std::fabs(current_positions[j] - start[j]));
  }
  if (max_deviation) *max_deviation = worst;
  return worst <= tolerance_rad;
}

Envelope compute_envelope(const TeachTrajectoryMsg & traj)
{
  Envelope env{};
  env.max_velocity.fill(0.0);
  env.max_acceleration.fill(0.0);
  if (traj.points.empty()) return env;

  // 速度：录下来的 velocities 与相邻点位置差分，取较大者。
  // 只信 velocities 的话，/joint_states 不带 velocity 字段的后端（或云台开环回显）
  // 会统计出全 0 包络，闸4 就形同虚设 —— 那正是最需要它拦住的情况。
  for (const auto & p : traj.points) {
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      env.max_velocity[j] = std::max(env.max_velocity[j], std::fabs(p.velocities[j]));
    }
  }
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const double dt = traj.points[i].time_from_start - traj.points[i - 1].time_from_start;
    if (dt <= 0.0) continue;   // 非单调轨迹由 validate_structure 负责拒绝，这里只跳过
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      const double dq = traj.points[i].positions[j] - traj.points[i - 1].positions[j];
      env.max_velocity[j] = std::max(env.max_velocity[j], std::fabs(dq / dt));
    }
  }

  // 加速度：相邻点速度差分。用录下来的 velocities 而不是位置二阶差分 ——
  // 位置二阶差分在 50Hz 采样上噪声被放大 1/dt2 倍，统计出的峰值几乎全是噪声。
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const double dt = traj.points[i].time_from_start - traj.points[i - 1].time_from_start;
    if (dt <= 0.0) continue;
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      const double dv = traj.points[i].velocities[j] - traj.points[i - 1].velocities[j];
      env.max_acceleration[j] = std::max(env.max_acceleration[j], std::fabs(dv / dt));
    }
  }
  return env;
}

Envelope scale_envelope(const Envelope & env, double speed_scale)
{
  const double s = speed_scale > 0.0 ? speed_scale : 1.0;
  Envelope out{};
  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    out.max_velocity[j]     = env.max_velocity[j] * s;
    out.max_acceleration[j] = env.max_acceleration[j] * s * s;
  }
  return out;
}

bool normalize_speed_scale(double * scale, double min_scale, double max_scale)
{
  if (!scale) return false;
  // 0 / 负数 / NaN 一律当作「没指定」→ 原速。这是 PlayTrajectory.srv 里承诺的行为，
  // 也是 ros2 service call 不填该字段时的默认值，所以**不算夹紧**、不告警 ——
  // 否则最常见的调用方式每次都会刷一条无意义的 WARN。
  if (!std::isfinite(*scale) || *scale <= 0.0) { *scale = 1.0; return false; }
  const double before = *scale;
  *scale = std::clamp(*scale, min_scale, max_scale);
  return std::fabs(*scale - before) > 1e-12;
}

}  // namespace robot_arm_teach
