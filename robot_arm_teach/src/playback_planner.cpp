/**
 * @file playback_planner.cpp
 * @brief PlaybackPlanner 实现
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/playback_planner.hpp"

#include <algorithm>
#include <cmath>

namespace robot_arm_teach
{

namespace
{
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

// time_from_start（builtin_interfaces/Duration）的秒数写入
void set_duration(JointTrajectoryPoint & pt, double seconds)
{
  if (seconds < 0.0) seconds = 0.0;
  pt.time_from_start.sec     = static_cast<int32_t>(seconds);
  pt.time_from_start.nanosec =
      static_cast<uint32_t>(std::llround((seconds - static_cast<double>(pt.time_from_start.sec)) *
                                         1e9));
  // 进位兜底：四舍五入可能把 nanosec 抬到 1e9
  if (pt.time_from_start.nanosec >= 1000000000u) {
    pt.time_from_start.sec += 1;
    pt.time_from_start.nanosec -= 1000000000u;
  }
}
}  // namespace

PlaybackPlanner::PlaybackPlanner()
: PlaybackPlanner(Config{})
{
}

PlaybackPlanner::PlaybackPlanner(Config cfg)
: cfg_(cfg)
{
}

void PlaybackPlanner::set_config(const Config & cfg)
{
  cfg_ = cfg;
  if (cfg_.chunk_horizon_sec <= 0.0) cfg_.chunk_horizon_sec = 1.0;
  if (cfg_.min_point_dt <= 0.0) cfg_.min_point_dt = 0.005;
  if (cfg_.hold_duration_sec <= 0.0) cfg_.hold_duration_sec = 0.2;
  if (cfg_.approach_duration_sec <= 0.0) cfg_.approach_duration_sec = 3.0;
}

PlaybackPlanner::ChunkResult PlaybackPlanner::make_chunk(const TeachTrajectoryMsg & traj,
                                                        double phase, double speed_scale) const
{
  ChunkResult out;
  out.next_phase = phase;
  out.trajectory.joint_names = traj.joint_names;
  // header.stamp 留 0：JTC 约定「立即开始」。见头文件的时间约定。

  if (traj.points.empty()) {
    out.finished = true;
    return out;
  }

  const double s = speed_scale > 0.0 ? speed_scale : 1.0;
  const double horizon = cfg_.chunk_horizon_sec;   // 原始时间轴上的视野长度

  // 第一个严格晚于 phase 的采样点
  size_t first = traj.points.size();
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (traj.points[i].time_from_start > phase + 1e-9) { first = i; break; }
  }
  if (first >= traj.points.size()) {
    out.finished    = true;
    out.point_index = traj.points.size();
    out.next_phase  = traj.points.back().time_from_start;
    return out;
  }

  const double window_end = phase + horizon;
  size_t last = first;
  for (size_t i = first; i < traj.points.size(); ++i) {
    if (traj.points[i].time_from_start > window_end) break;
    last = i;
  }
  // 视野里一个点都没有（压缩后此处极稀疏）时，至少带上紧随其后的那一个 ——
  // 否则 phase 推不动，流式下发会在稀疏段原地卡住。
  if (last < first) last = first;

  for (size_t i = first; i <= last; ++i) {
    const auto & src = traj.points[i];
    JointTrajectoryPoint pt;
    pt.positions.assign(src.positions.begin(), src.positions.end());
    pt.velocities.resize(TEACH_JOINT_COUNT);
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      pt.velocities[j] = src.velocities[j] * s;   // 时间轴压缩 s 倍 → 速度 ×s
    }
    // 刻意不填 accelerations：robot_arm_node 的 build_joint_trajectory 也只给位置+速度
    // （注释里说明五次样条经验证劣于三次）。这里保持同样的做法，免得两条路径手感不一致。
    const double rel = std::max(cfg_.min_point_dt, (src.time_from_start - phase) / s);
    set_duration(pt, rel);
    out.trajectory.points.push_back(std::move(pt));
  }

  out.next_phase  = traj.points[last].time_from_start;
  out.point_index = last + 1;
  out.finished    = (last + 1 >= traj.points.size());
  return out;
}

JointTrajectoryMsg PlaybackPlanner::make_hold(const std::vector<std::string> & joint_names,
                                             const std::vector<double> & current_positions) const
{
  JointTrajectoryMsg traj;
  traj.joint_names = joint_names;
  if (current_positions.size() < joint_names.size()) return traj;   // 回读不全，不发（调用方告警）

  JointTrajectoryPoint pt;
  pt.positions.assign(current_positions.begin(),
                      current_positions.begin() + static_cast<long>(joint_names.size()));
  // 速度显式给 0：不给的话 JTC 会自己插值出一条带速度的样条，暂停时机械臂会再"溜"一小段
  pt.velocities.assign(joint_names.size(), 0.0);
  set_duration(pt, cfg_.hold_duration_sec);
  traj.points.push_back(std::move(pt));
  return traj;
}

JointTrajectoryMsg PlaybackPlanner::make_approach(const TeachTrajectoryMsg & traj,
                                                 const std::vector<std::string> & joint_names) const
{
  JointTrajectoryMsg out;
  out.joint_names = joint_names;
  if (traj.points.empty()) return out;

  const auto & start = traj.points.front().positions;
  JointTrajectoryPoint pt;
  pt.positions.assign(start.begin(), start.end());
  pt.velocities.assign(TEACH_JOINT_COUNT, 0.0);   // 停在起点，等下一段再起步
  set_duration(pt, cfg_.approach_duration_sec);
  out.points.push_back(std::move(pt));
  return out;
}

}  // namespace robot_arm_teach
