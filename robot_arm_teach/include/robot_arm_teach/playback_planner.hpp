/**
 * @file playback_planner.hpp
 * @brief 回放分段规划 —— 把示教轨迹切成滚动分段 / 保持轨迹 / 接近段（纯逻辑，可单元测试）
 *
 * ★ 为什么要分段流式下发，而不是一次把整条轨迹发给 JTC
 *   一次发完最省事，但那样**回放中途什么都做不了**：JTC 只认最后收到的那条轨迹，
 *   暂停/停止/改速度都无从下手（发空轨迹并不会让它停下，它会继续执行手上那条 ——
 *   这正是 requirement 9 点名要避免的做法）。
 *   分段之后：暂停 = 发一条「保持在当前实测位置」的轨迹（见 make_hold）；停止同理；
 *   改速度 = 下一段用新倍率重算时间轴。代价是每段只被执行一部分就被下一段替换。
 *
 * ★ 分段频率与视野的取舍（改这两个参数前先读）
 *   robot_arm_node 的速度流踩过这个坑：JTC 每收到新轨迹就丢弃旧的、从**当前实际状态**
 *   重新插值，所以「下发周期 / 计划时长」的比值直接决定跟踪质量。那里是**单点**轨迹，
 *   计划开头恰是五次样条最慢的一段，50Hz + 50ms 视野只跟到指令的 71%。
 *   本类的分段是**稠密多点**轨迹（直接用录下来的采样点，带各点速度），样条穿过的是
 *   实测路径本身，执行前 20% 也是正确的瞬时速度，所以不会退化成那种情况。
 *   但仍然别把 republish_hz 调高：默认 5Hz / 1.0s 视野（每段执行 20%）已经足够
 *   让暂停在 200ms 内生效，再高只是白抢占 JTC。
 *
 * 时间约定：分段里的 time_from_start 是**相对下发时刻**的，header.stamp 留 0
 * （JTC 约定 = 立即开始）。不用节点时钟盖戳是为了不引入本节点与控制器之间的时钟偏差，
 * 也顺带避开 use_sim_time 那类陷阱。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "robot_arm_teach/msg/teach_trajectory.hpp"
#include "robot_arm_teach/teach_types.hpp"

namespace robot_arm_teach
{

using JointTrajectoryMsg = trajectory_msgs::msg::JointTrajectory;

class PlaybackPlanner
{
public:
  struct Config
  {
    double chunk_horizon_sec{1.0};      // 每段覆盖的轨迹时长（原始时间轴上的秒数 × 倍率）
    double min_point_dt{0.005};         // 段内首点的最小 time_from_start（s）。
                                        // 0 时长的点会让 JTC 遇到除零 / 无限加速度。
    double approach_duration_sec{3.0};  // 接近段时长（从当前位置慢速走到轨迹起点）
    double hold_duration_sec{0.2};      // 保持轨迹的时长（暂停/停止时用）
  };

  // 默认构造与单参构造分开声明的原因见 teach_recorder.hpp 同名注释
  // （GCC 对嵌套类默认成员初始值用作外层类默认参数的已知 bug）。
  PlaybackPlanner();
  explicit PlaybackPlanner(Config cfg);
  void set_config(const Config & cfg);
  const Config & config() const { return cfg_; }

  struct ChunkResult
  {
    JointTrajectoryMsg trajectory;   // points 为空 = 没有可下发的内容
    double next_phase{0.0};          // 本段末点在原始时间轴上的位置（下次调用的 phase）
    size_t point_index{0};           // 已越过的采样点序号（进度反馈用）
    bool   finished{false};          // 本段已包含轨迹末点，播完
  };

  // 取 phase 之后的一段。
  //   phase        当前播放进度在**原始**时间轴上的位置（s）
  //   speed_scale  倍率 s：墙钟时间 = 原始时间 / s。段内 time_from_start 除以 s，
  //                各点速度乘以 s（时间轴压缩 s 倍 → 速度 ×s）
  // 若 phase 之后视野内没有采样点（压缩后点很稀疏），仍会带上紧随其后的那一个点 ——
  // 否则流式下发会在稀疏段卡住不动。
  ChunkResult make_chunk(const TeachTrajectoryMsg & traj, double phase, double speed_scale) const;

  // 保持轨迹：单点 = 当前实测位置，时长 hold_duration_sec，速度全 0。
  // 暂停/停止时下发这一条，机械臂明确停在当前位置。
  // ★ 绝不发空轨迹 —— 空 points 不会让 JTC 停下，它会把手上那条继续执行完。
  JointTrajectoryMsg make_hold(const std::vector<std::string> & joint_names,
                               const std::vector<double> & current_positions) const;

  // 接近段：从当前位置单点走到轨迹首点，时长 approach_duration_sec。
  // 当前位置离起点较远时（闸6 未过且允许自动接近）先走这一段，再进入正常分段流。
  JointTrajectoryMsg make_approach(const TeachTrajectoryMsg & traj,
                                   const std::vector<std::string> & joint_names) const;

private:
  Config cfg_{};
};

}  // namespace robot_arm_teach
