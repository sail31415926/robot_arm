/**
 * @file trajectory_shot_server.cpp
 * @brief TrajectoryShotServer 实现 —— MOTION_LINEAR / MOTION_ORBIT
 *
 * LINEAR：move_and_wait 分段（起点→终点[→返回]）。ORBIT：PTP 到起始球坐标→dwell→
 * plan_orbit_ruckig 球面轨道→wait_at_pose[→原路返回]。sphere_to_pose 复用 motion 几何。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/trajectory_shot_server.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <robot_arm_interfaces/msg/arm_status.hpp>

#include "robot_arm_node/commander/motion_policy.hpp"
#include "robot_arm_node/motion/geometry.hpp"

namespace robot_arm_node::commander
{

using ArmStatus = robot_arm_interfaces::msg::ArmStatus;

namespace
{
constexpr double DEFAULT_TIMEOUT_SEC = 60.0;
constexpr double FEEDBACK_RATE_HZ    = 10.0;
constexpr double DWELL_AT_START_SEC  = 1.0;
constexpr double DEG2RAD = M_PI / 180.0;

// WaitOutcome → exit_reason 字符串（success 情形外）
const char * outcome_reason(WaitOutcome o)
{
  switch (o) {
    case WaitOutcome::REACHED:   return "reached";
    case WaitOutcome::STOPPED:   return "stopped";
    case WaitOutcome::CANCELLED: return "cancelled";
    default:                     return "timeout";
  }
}
}  // namespace

TrajectoryShotServer::TrajectoryShotServer(rclcpp::Node & node, MotionExecutor & motion,
                                           state::StatusAggregator & status,
                                           ExecutionMonitor & monitor,
                                           std::function<bool()> is_stopped)
: node_(node), logger_(node.get_logger()), motion_(motion), status_(status),
  monitor_(monitor), is_stopped_(std::move(is_stopped))
{
}

TrajectoryShotServer::Action::Result TrajectoryShotServer::execute(
    const std::shared_ptr<GoalHandle> & gh)
{
  const auto goal  = gh->get_goal();
  const Speed & speed = speed_profile(goal->transition_speed);

  if (goal->motion_type == Action::Goal::MOTION_LINEAR) {
    return execute_linear(gh, *goal, speed);
  }
  if (goal->motion_type == Action::Goal::MOTION_ORBIT) {
    return execute_orbit(gh, *goal, speed);
  }
  RCLCPP_ERROR(logger_, "非法 motion_type: %d", goal->motion_type);
  Action::Result r;
  r.success = false; r.exit_reason = "error"; r.error_code = ArmStatus::ERR_LIMIT;
  return r;
}

// ── MOTION_LINEAR ────────────────────────────────────────────────────────────────
TrajectoryShotServer::Action::Result TrajectoryShotServer::execute_linear(
    const std::shared_ptr<GoalHandle> & gh, const Action::Goal & goal, const Speed & speed)
{
  const ArmPose & start = goal.linear_start_pose;
  const ArmPose & end   = goal.linear_end_pose;
  RCLCPP_INFO(logger_, "LINEAR 起始=(%.3f,%.3f,%.3f) 终止=(%.3f,%.3f,%.3f)",
              start.x, start.y, start.z, end.x, end.y, end.z);

  // 步骤 1：移到起始位姿
  auto r = move_and_wait(gh, start, speed, "LINEAR 起始位",
                         0.0, goal.return_to_start ? 33.0 : 50.0);
  if (!r.success) return r;
  if (!dwell_at_start(gh, "LINEAR")) {
    Action::Result res; res.success = false; res.exit_reason = "cancelled";
    return res;
  }

  // 步骤 2：移到终止位姿
  const double p2_end = goal.return_to_start ? 67.0 : 100.0;
  r = move_and_wait(gh, end, speed, "LINEAR 终止位",
                    goal.return_to_start ? 33.0 : 50.0, p2_end);
  if (!r.success || !goal.return_to_start) return r;

  // 步骤 3：返回起始位姿
  RCLCPP_INFO(logger_, "LINEAR return_to_start: 返回起始位姿");
  return move_and_wait(gh, start, speed, "LINEAR 返回起始", 67.0, 100.0);
}

// ── MOTION_ORBIT ─────────────────────────────────────────────────────────────────
TrajectoryShotServer::Action::Result TrajectoryShotServer::execute_orbit(
    const std::shared_ptr<GoalHandle> & gh, const Action::Goal & goal, const Speed & speed)
{
  const double ox = goal.orbit_center_x, oy = goal.orbit_center_y, oz = goal.orbit_center_z;
  const double az0 = goal.azimuth_start_deg * DEG2RAD, el0 = goal.elevation_start_deg * DEG2RAD;
  const double r0  = goal.radius_start_m;
  const double az1 = goal.azimuth_end_deg * DEG2RAD, el1 = goal.elevation_end_deg * DEG2RAD;
  const double r1  = goal.radius_end_m;

  RCLCPP_INFO(logger_, "ORBIT 球心=(%.3f,%.3f,%.3f)  起(%.1f°,%.1f°,%.3fm) → 终(%.1f°,%.1f°,%.3fm)",
              ox, oy, oz, goal.azimuth_start_deg, goal.elevation_start_deg, r0,
              goal.azimuth_end_deg, goal.elevation_end_deg, r1);

  // 球面轨道归一化 Ruckig 参数取自 speed 的姿态分量
  const double s_vel = speed.v_ori, s_acc = speed.a_ori, s_jerk = speed.j_ori;

  // 用户取消或急停均视为应中止
  auto cancelled = [this, gh]() { return gh->is_canceling() || (is_stopped_ && is_stopped_()); };

  Action::Result result;

  // 步骤 1：PTP 移到起始球坐标
  const ArmPose start_pose = sphere_to_pose(goal.azimuth_start_deg, goal.elevation_start_deg, r0, ox, oy, oz);
  auto r_ptp = move_and_wait(gh, start_pose, speed, "ORBIT PTP→起点",
                             0.0, goal.return_to_start ? 30.0 : 20.0,
                             goal.azimuth_start_deg, goal.elevation_start_deg, r0);
  if (!r_ptp.success) return r_ptp;
  if (!dwell_at_start(gh, "ORBIT")) { result.exit_reason = "cancelled"; return result; }

  // 步骤 2：Ruckig 1-DOF 球面轨道（起 → 终）
  if (cancelled()) { motion_.stop(); result.exit_reason = "cancelled"; return result; }
  RCLCPP_INFO(logger_, "ORBIT 球面轨道开始（Ruckig 1-DOF）");
  if (!motion_.plan_orbit_ruckig(ox, oy, oz, az0, el0, r0, az1, el1, r1,
                                 s_vel, s_acc, s_jerk, cancelled)) {
    result.success = false;
    result.exit_reason = cancelled() ? "cancelled" : "error";
    result.error_code = ArmStatus::ERR_DRIVER;
    return result;
  }

  if (!goal.return_to_start) {
    const ArmPose end_pose = sphere_to_pose(goal.azimuth_end_deg, goal.elevation_end_deg, r1, ox, oy, oz);
    return wait_at_pose(gh, end_pose, "ORBIT 终止到位", 60.0, 100.0,
                        goal.azimuth_end_deg, goal.elevation_end_deg, r1);
  }

  // 步骤 3：Ruckig 1-DOF 原路返回（终 → 起）
  if (cancelled()) { motion_.stop(); result.exit_reason = "cancelled"; return result; }
  RCLCPP_INFO(logger_, "ORBIT return_to_start: 原路返回");
  if (!motion_.plan_orbit_ruckig(ox, oy, oz, az1, el1, r1, az0, el0, r0,
                                 s_vel, s_acc, s_jerk, cancelled)) {
    result.success = false;
    result.exit_reason = cancelled() ? "cancelled" : "error";
    result.error_code = ArmStatus::ERR_DRIVER;
    return result;
  }
  return wait_at_pose(gh, start_pose, "ORBIT 返回到位", 90.0, 100.0,
                      goal.azimuth_start_deg, goal.elevation_start_deg, r0);
}

// ── 到达起始点后的停顿 ─────────────────────────────────────────────────────────────
bool TrajectoryShotServer::dwell_at_start(const std::shared_ptr<GoalHandle> & gh, const char * label)
{
  status_.set_at_pose_start(true);
  status_.set_camera_ready(true);
  RCLCPP_INFO(logger_, "%s 已到达起始点，停顿 %.1fs 后执行运镜", label, DWELL_AT_START_SEC);

  using clock = std::chrono::steady_clock;
  const auto t_end = clock::now() + std::chrono::duration<double>(DWELL_AT_START_SEC);
  bool cancelled = false;
  while (clock::now() < t_end) {
    if (is_stopped_ && is_stopped_()) {
      RCLCPP_INFO(logger_, "%s 起点停顿期间被急停", label);
      cancelled = true; break;
    }
    if (gh->is_canceling()) {
      motion_.stop();
      RCLCPP_INFO(logger_, "%s 起点停顿期间被取消", label);
      cancelled = true; break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(0.02));
  }
  status_.set_at_pose_start(false);
  return !cancelled;
}

// ── 共用：IK 移动 + 等待到位 ────────────────────────────────────────────────────────
TrajectoryShotServer::Action::Result TrajectoryShotServer::move_and_wait(
    const std::shared_ptr<GoalHandle> & gh, const ArmPose & target, const Speed & speed,
    const char * label, double p_lo, double p_hi, double azimuth, double elevation, double radius)
{
  Action::Result result;

  auto exec_r = motion_.plan_and_execute(target, speed);
  if (!exec_r.success) {
    result.success = false; result.exit_reason = exec_r.exit_reason;
    result.error_code = ArmStatus::ERR_DRIVER;
    return result;
  }

  const ArmPose cur = status_.pose();
  const double dist = std::sqrt(std::pow(target.x - cur.x, 2) +
                                std::pow(target.y - cur.y, 2) +
                                std::pow(target.z - cur.z, 2));
  const double total_dur = std::max(dist / std::max(speed.v_pos, 1e-6), 0.5);

  ExecutionMonitor::WaitParams p;
  p.arrived = [this, target]() { return is_at_pose(status_.pose(), target); };
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  p.on_feedback = [this, gh, p_lo, p_hi, total_dur, azimuth, elevation, radius](double elapsed) {
    const double ratio = std::min(elapsed / std::max(total_dur, 1e-6), 0.999);
    auto fb = std::make_shared<Action::Feedback>();
    fb->progress_percent      = static_cast<float>(p_lo + ratio * (p_hi - p_lo));
    fb->elapsed_sec           = static_cast<float>(elapsed);
    fb->current_pose          = status_.pose();
    fb->current_azimuth_deg   = static_cast<float>(azimuth);
    fb->current_elevation_deg = static_cast<float>(elevation);
    fb->current_radius_m      = static_cast<float>(radius);
    gh->publish_feedback(fb);
  };
  p.timeout_sec = DEFAULT_TIMEOUT_SEC;
  p.feedback_hz = FEEDBACK_RATE_HZ;
  p.label = std::string(label) + " ";
  const auto outcome = monitor_.wait_until(p);

  result.success     = is_reached(outcome);
  result.exit_reason = outcome_reason(outcome);
  result.error_code  = result.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  return result;
}

// ── 等待已下发轨迹执行完毕（只轮询到位）────────────────────────────────────────────
TrajectoryShotServer::Action::Result TrajectoryShotServer::wait_at_pose(
    const std::shared_ptr<GoalHandle> & gh, const ArmPose & target,
    const char * label, double p_lo, double p_hi, double azimuth, double elevation, double radius)
{
  ExecutionMonitor::WaitParams p;
  p.arrived = [this, target]() { return is_at_pose(status_.pose(), target); };
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  p.on_feedback = [this, gh, p_lo, p_hi, azimuth, elevation, radius](double elapsed) {
    // 轨迹已下发、无 dist 依据；沿用原实现的 0.1×timeout 归一化
    const double ratio = std::min(elapsed / std::max(DEFAULT_TIMEOUT_SEC * 0.1, 1e-6), 0.999);
    auto fb = std::make_shared<Action::Feedback>();
    fb->progress_percent      = static_cast<float>(p_lo + ratio * (p_hi - p_lo));
    fb->elapsed_sec           = static_cast<float>(elapsed);
    fb->current_pose          = status_.pose();
    fb->current_azimuth_deg   = static_cast<float>(azimuth);
    fb->current_elevation_deg = static_cast<float>(elevation);
    fb->current_radius_m      = static_cast<float>(radius);
    gh->publish_feedback(fb);
  };
  p.timeout_sec = DEFAULT_TIMEOUT_SEC;
  p.feedback_hz = FEEDBACK_RATE_HZ;
  p.label = std::string(label) + " ";
  const auto outcome = monitor_.wait_until(p);

  Action::Result result;
  result.success     = is_reached(outcome);
  result.exit_reason = outcome_reason(outcome);
  result.error_code  = result.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  return result;
}

// ── 球坐标 → Cartesian 位姿 ────────────────────────────────────────────────────────
ArmPose TrajectoryShotServer::sphere_to_pose(double azimuth_deg, double elevation_deg,
                                             double radius_m, double ox, double oy, double oz)
{
  const double az = azimuth_deg * DEG2RAD, el = elevation_deg * DEG2RAD;
  const auto c = motion::sphere_to_cart(az, el, radius_m, ox, oy, oz);
  const auto q = motion::aim_quat(c[0], c[1], c[2], ox, oy, oz);
  const auto rpy = motion::quat_to_rpy(q[0], q[1], q[2], q[3]);
  ArmPose pose;
  pose.x = static_cast<float>(c[0]); pose.y = static_cast<float>(c[1]); pose.z = static_cast<float>(c[2]);
  pose.roll  = static_cast<float>(rpy[0] / DEG2RAD);
  pose.pitch = static_cast<float>(rpy[1] / DEG2RAD);
  pose.yaw   = static_cast<float>(rpy[2] / DEG2RAD);
  return pose;
}

}  // namespace robot_arm_node::commander
