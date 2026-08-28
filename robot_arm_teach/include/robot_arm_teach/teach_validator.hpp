/**
 * @file teach_validator.hpp
 * @brief 轨迹静态校验 + 运动学包络统计 —— 纯函数，无 ROS 节点依赖（可单元测试）
 *
 * 回放前的九道闸里，①~④、⑥ 的判据全在这里；⑤（自碰撞）要调服务、⑦（急停）要读状态、
 * ⑧⑨（切模式/下发）要发话题，那三样留在节点里。这样切分的目的是：**判据可测**。
 * 校验逻辑写错的后果是让一条坏轨迹驱动实物机械臂，必须能在 CI 里跑到。
 *
 * 设计约定：
 *   · 所有函数不改入参、不打日志、不抛异常，失败一律经返回值上报（reason 是稳定的
 *     机器可读串，会直接填进 PlayTrajectory.Response.exit_reason；detail 给人看）。
 *   · 限位 map 为空 = URDF 未就绪 → **fail-open**（放行，由调用方告警）。与
 *     robot_arm_node 的 JointLimitsCache 同一策略：拿不到 URDF 就把机械臂卡死，
 *     在实机上比不校验更糟（整台机器谁都动不了，还看不出为什么）。
 *   · 速度/加速度闸不 fail-open —— 那两个上限来自本包参数，永远拿得到。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <string>
#include <vector>

#include "robot_arm_teach/msg/teach_trajectory.hpp"
#include "robot_arm_teach/teach_types.hpp"

namespace robot_arm_teach
{

// 校验结论。reason 取值与 PlayTrajectory.srv 注释里列的 exit_reason 一致。
struct ValidateResult
{
  bool        ok{true};
  std::string reason;    // 机器可读：""(ok) / "invalid_trajectory" / "out_of_range" /
                         //           "over_speed"
  std::string detail;    // 人类可读，含具体轴名与数值

  explicit operator bool() const { return ok; }

  static ValidateResult success() { return ValidateResult{}; }
  static ValidateResult failure(std::string r, std::string d)
  {
    return ValidateResult{false, std::move(r), std::move(d)};
  }
};

// 逐轴运动学包络。录制结束时写进 TeachTrajectory；回放前用来与 MotionCaps 比对。
struct Envelope
{
  JointArray max_velocity{};        // rad/s，|q̇| 峰值
  JointArray max_acceleration{};    // rad/s^2，|q̈| 峰值
};

// ── 闸①②：结构 + 数值有效性 + 时间单调性 ───────────────────────────────────
// 校验：format_version 受支持、points 非空、joint_names 恰为 expected_names（逐字、同序）、
//       每点位置/速度全为有限值、time_from_start 严格单调递增且首点 >= 0。
//
// 为什么关节名要求「逐字同序」而不是按名字重排：能重排就意味着能猜，猜错的代价是
// 机械臂按错的轴走。轨迹文件是本包自己写的，名字对不上说明文件来自别的机器人或被改坏了，
// 那就该拒绝，不该修补。
ValidateResult validate_structure(const TeachTrajectoryMsg & traj,
                                  const std::vector<std::string> & expected_names);

// ── 闸③：URDF 关节限位（limits 为空 → fail-open 返回 ok）───────────────────
ValidateResult validate_limits(const TeachTrajectoryMsg & traj, const JointBoundMap & limits);

// ── 闸④：速度 / 加速度上限 ──────────────────────────────────────────────────
// speed_scale 为回放倍率：速度线性缩放（×s），加速度按平方缩放（×s²）——
// 时间轴压缩 s 倍时 q̇ = dq/(dt/s) = s·dq/dt，q̈ = s²·d²q/dt²。
// 降速（s<1）必然更安全，所以真正会被这道闸挡住的是 s>1 的加速请求。
ValidateResult validate_speed(const TeachTrajectoryMsg & traj, const MotionCaps & caps,
                              double speed_scale);

// ── 闸⑥：当前位置是否落在轨迹起点附近 ──────────────────────────────────────
// 只比前 arm_joint_count 个轴（默认 3）。理由同 robot_arm_node/motion_policy.hpp 的
// is_at_joints_prefix：末端在云台之后，云台有静差就永远"到不了"，把判据交给云台
// 会让每次回放都以"没在起点"告终。max_deviation 出参给调用方拼错误信息用。
bool is_at_start(const TeachTrajectoryMsg & traj, const std::vector<double> & current_positions,
                 double tolerance_rad, size_t arm_joint_count = JOG_JOINT_COUNT,
                 double * max_deviation = nullptr);

// ── 包络统计 ────────────────────────────────────────────────────────────────
// 速度取录下来的 velocities 与相邻点位置差分两者的较大值 —— 只信 velocities 的话，
// /joint_states 不带速度字段的后端会统计出全 0 包络，闸④就形同虚设。
// 加速度由相邻点速度差分得到；点数 < 2 时全 0。
Envelope compute_envelope(const TeachTrajectoryMsg & traj);

// 按 speed_scale 缩放后的包络（不改原轨迹，供闸④与日志使用）
Envelope scale_envelope(const Envelope & env, double speed_scale);

// 把 speed_scale 规范化：<=0 视为 1.0（PlayTrajectory.srv 里承诺的行为），
// 并夹到 [min_scale, max_scale]。返回是否发生了夹紧（调用方据此决定是否告警）。
bool normalize_speed_scale(double * scale, double min_scale, double max_scale);

}  // namespace robot_arm_teach
