/**
 * @file motion_executor.cpp
 * @brief MotionExecutor 实现 —— 委托 motion 库 + 持有共享 ROS 资源
 *
 * 内部工具（匿名命名空间）：to_duration（秒→Duration）、single_point_traj（只给目标点的
 * 单点 JointTrajectory）。plan_and_execute/go_to_joints 复用 single_point_traj，
 * solve_and_send/plan_orbit_ruckig 转调 motion 库，急停判据 is_stopped 与传入的
 * cancel_check 合成后注入。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/motion_executor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robot_arm_node/motion/constants.hpp"
#include "robot_arm_node/motion/geometry.hpp"
#include "robot_arm_node/motion/kinematics.hpp"
#include "robot_arm_node/motion/trajectory.hpp"
#include "robot_arm_node/tuning.hpp"

namespace robot_arm_node::commander
{

namespace
{
// 秒 → builtin_interfaces::Duration
builtin_interfaces::msg::Duration to_duration(double sec)
{
  builtin_interfaces::msg::Duration d;
  d.sec     = static_cast<int32_t>(sec);
  d.nanosec = static_cast<uint32_t>((sec - static_cast<int32_t>(sec)) * 1e9);
  return d;
}

// 构建只含目标点的单点 JointTrajectory。
//
// ★ 为什么不把当前关节当首点发（2026-09-01 修，实机「起步抖一下」的根因）★
// 曾经的写法是两点轨迹 [当前实测关节@0s, 目标@duration]。首点 time_from_start=0 让
// JTC 的融合窗口宽度为 0（Trajectory::sample 只在 sample_time < 首点时刻时才用
// state_before_traj_msg_ 插值），轨迹一起跑，指令位置就**阶跃**到实测关节 —— 而 JTC
// 此刻保持的指令是上一段动作的终点，于是起步瞬间往回跳一个到位残差。
// 现在只发目标点，由 JTC 用自身的 state_before_traj_msg_（open_loop_control=false →
// 实测位置+实测速度）到目标点之间插三次样条，起点由它自己决定 ——
// 实测量不写进指令流。
trajectory_msgs::msg::JointTrajectory single_point_traj(
    const std::vector<double> & to, double duration,
    const builtin_interfaces::msg::Time & stamp)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = stamp;
  msg.joint_names  = motion::JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions       = to;
  pt.velocities.assign(to.size(), 0.0);
  pt.time_from_start = to_duration(duration);

  msg.points = {std::move(pt)};
  return msg;
}
}  // namespace

MotionExecutor::MotionExecutor(rclcpp::Node & node, state::StatusAggregator & status,
                               std::function<bool()> is_stopped)
: node_(node), logger_(node.get_logger()), status_(status),
  is_stopped_(std::move(is_stopped))
{
  traj_pub_  = node_.create_publisher<trajectory_msgs::msg::JointTrajectory>(
      motion::TRAJ_TOPIC, 10);
  ik_client_ = node_.create_client<GetPositionIK>(motion::IK_SERVICE);

  RCLCPP_INFO(logger_, "MotionExecutor 就绪  |  JointTrajectory → %s  |  IK → %s",
              motion::TRAJ_TOPIC.c_str(), motion::IK_SERVICE.c_str());
}

// ── 点对点运动 ──────────────────────────────────────────────────────────────────
ExecResult MotionExecutor::plan_and_execute(const ArmPose & target, const Speed & speed)
{
  const ArmPose start = get_ee_pose();
  RCLCPP_INFO(logger_, "plan_and_execute: 起点=(%.3f,%.3f,%.3f) → 目标=(%.3f,%.3f,%.3f)",
              start.x, start.y, start.z, target.x, target.y, target.z);

  // 1. 构建 PoseStamped
  const auto q = motion::rpy_to_quat(
      target.roll * M_PI / 180.0, target.pitch * M_PI / 180.0, target.yaw * M_PI / 180.0);
  const auto ps = motion::make_pose_stamped(
      target.x, target.y, target.z, q[0], q[1], q[2], q[3],
      motion::BASE_FRAME, node_.get_clock()->now());

  // 2. IK 求解
  auto [joints, err] = ik_sync(ps);
  if (!joints) {
    RCLCPP_WARN(logger_, "IK 无解，目标不可达 (error_code=%d)", err);
    return {false, "unreachable", err, {}};
  }

  // 3. 到达时间：按位移 / 速度估算（含 1.5x 加减速余量，下限 0.5s）
  const double dist = std::sqrt(
      std::pow(target.x - start.x, 2) + std::pow(target.y - start.y, 2) +
      std::pow(target.z - start.z, 2));
  const double duration = std::max(dist / std::max(speed.v_pos, 1e-6) * 1.5, 0.5);

  // 4. 下发单点 JointTrajectory（终点=IK 解；起点交给 JTC 从实测状态自己插值）
  auto msg = single_point_traj(*joints, duration, node_.get_clock()->now());

  // 急停在 IK 期间发生时，绝不再下发轨迹（否则会把已停住的机械臂重新开动）
  if (is_stopped_ && is_stopped_()) {
    RCLCPP_INFO(logger_, "plan_and_execute: 急停生效，放弃下发轨迹");
    return {false, "stopped", 0, {}};
  }
  traj_pub_->publish(msg);
  RCLCPP_INFO(logger_, "JointTrajectory 已下发 (duration=%.2fs)", duration);
  return {true, "reached", 0, *joints};
}

// ── 急停 ────────────────────────────────────────────────────────────────────────
void MotionExecutor::stop()
{
  const auto positions = status_.joint_position_list(motion::JOINT_NAMES);
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = node_.get_clock()->now();
  msg.joint_names  = motion::JOINT_NAMES;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions       = positions;
  pt.velocities.assign(motion::JOINT_NAMES.size(), 0.0);
  pt.time_from_start = to_duration(0.0);
  msg.points = {std::move(pt)};
  traj_pub_->publish(msg);
  RCLCPP_INFO(logger_, "急停指令已发送");
}

// ── 关节空间直驱 ────────────────────────────────────────────────────────────────
ExecResult MotionExecutor::go_to_joints(const std::vector<double> & target_joints,
                                        double duration_sec)
{
  auto msg = single_point_traj(target_joints, duration_sec,
                               node_.get_clock()->now());
  traj_pub_->publish(msg);
  return {true, "sent", 0, target_joints};
}

// ── 状态查询 ────────────────────────────────────────────────────────────────────
std::vector<double> MotionExecutor::get_current_joints() const
{
  return status_.joint_position_list(motion::JOINT_NAMES);
}

ArmPose MotionExecutor::get_ee_pose() const
{
  return status_.pose();
}

// ── IK ──────────────────────────────────────────────────────────────────────────
std::pair<std::optional<std::vector<double>>, int> MotionExecutor::ik_sync(
    const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  const auto seed = status_.joint_position_list(motion::JOINT_NAMES);
  // 单点 IK：不设 ik_link_name（eef_link=""），与 Python ik_sync 一致
  auto res = motion::solve_ik(ik_client_, pose_stamped, seed, motion::JOINT_NAMES,
                              motion::PLANNING_GROUP, "", tuning::params().ik_timeout_s,
                              /*wait_service=*/true, logger_);
  return {res.joints, res.error_code};
}

// ── 批量 IK 下发 ────────────────────────────────────────────────────────────────
motion::PlanResult MotionExecutor::solve_and_send(const std::vector<motion::Waypoint> & all_pts,
                                                  std::function<bool()> cancel_check,
                                                  std::vector<double> * final_joints)
{
  const auto seed = status_.joint_position_list(motion::JOINT_NAMES);
  auto stop_check = [this, cancel_check]() {
    return (is_stopped_ && is_stopped_()) || (cancel_check && cancel_check());
  };
  return motion::solve_and_send(node_, ik_client_, traj_pub_, all_pts, seed,
                                motion::JOINT_NAMES, motion::PLANNING_GROUP, motion::EEF_LINK,
                                motion::BASE_FRAME, tuning::params().ik_decimate,
                                tuning::params().ik_timeout_s,
                                stop_check, logger_, final_joints);
}

motion::PlanResult MotionExecutor::plan_orbit_ruckig(double ox, double oy, double oz,
                                                     double theta0, double phi0, double r0,
                                                     double theta1, double phi1, double r1,
                                                     const Speed & speed,
                                                     std::function<bool()> cancel_check,
                                                     std::vector<double> * final_joints)
{
  auto stop_check = [this, cancel_check]() {
    return (is_stopped_ && is_stopped_()) || (cancel_check && cancel_check());
  };
  auto pts = motion::plan_orbit_waypoints(ox, oy, oz, theta0, phi0, r0,
                                          theta1, phi1, r1,
                                          speed.v_pos, speed.a_pos, speed.j_pos,
                                          speed.v_ori, speed.a_ori, speed.j_ori, stop_check);
  // Ruckig 规划失败（含被 stop_check 中止）：上层按 cancelled/error 归类
  if (!pts) return motion::PlanResult::Error;
  return solve_and_send(*pts, cancel_check, final_joints);
}

motion::PlanResult MotionExecutor::plan_line_ruckig(const ArmPose & start, const ArmPose & end,
                                                    const Speed & speed, std::function<bool()> cancel_check,
                                                    std::vector<double> * final_joints)
{
  auto stop_check = [this, cancel_check]() {
    return (is_stopped_ && is_stopped_()) || (cancel_check && cancel_check());
  };
  constexpr double D2R = M_PI / 180.0;
  const auto q0 = motion::rpy_to_quat(start.roll * D2R, start.pitch * D2R, start.yaw * D2R);
  const auto q1 = motion::rpy_to_quat(end.roll * D2R, end.pitch * D2R, end.yaw * D2R);
  auto pts = motion::plan_line_waypoints(start.x, start.y, start.z, q0,
                                         end.x, end.y, end.z, q1,
                                         speed.v_pos, speed.a_pos, speed.j_pos,
                                         speed.v_ori, speed.a_ori, speed.j_ori, stop_check);
  // Ruckig 规划失败（含被 stop_check 中止）：上层按 cancelled/error 归类
  if (!pts) return motion::PlanResult::Error;
  return solve_and_send(*pts, cancel_check, final_joints);
}

bool MotionExecutor::can_plan_line(const ArmPose & start, const ArmPose & end,
                                   const Speed & speed)
{
  constexpr double D2R = M_PI / 180.0;
  const auto q0 = motion::rpy_to_quat(start.roll * D2R, start.pitch * D2R, start.yaw * D2R);
  const auto q1 = motion::rpy_to_quat(end.roll * D2R, end.pitch * D2R, end.yaw * D2R);

  // Ruckig 仅对归一化参数 s∈[0,1] 做时间最优规划，与实际距离无关，任意起止点都能
  // 生成合法的运动曲线——它不判断末端是否落在机械臂可达域内，因此不能单独作为可行性依据
  if (!motion::plan_line_waypoints(start.x, start.y, start.z, q0,
                                   end.x, end.y, end.z, q1,
                                   speed.v_pos, speed.a_pos, speed.j_pos,
                                   speed.v_ori, speed.a_ori, speed.j_ori).has_value()) {
    return false;
  }

  // 起点/终点必须先各自过一次 IK，才能确认直线两端都在可达域内
  // （中间路点仍可能因奇异位形失败，交由 solve_and_send 批量 IK 兜底处理）
  const auto now = node_.get_clock()->now();
  const auto ps0 = motion::make_pose_stamped(start.x, start.y, start.z,
                                             q0[0], q0[1], q0[2], q0[3],
                                             motion::BASE_FRAME, now);
  const auto ps1 = motion::make_pose_stamped(end.x, end.y, end.z,
                                             q1[0], q1[1], q1[2], q1[3],
                                             motion::BASE_FRAME, now);
  return ik_sync(ps0).first.has_value() && ik_sync(ps1).first.has_value();
}

}  // namespace robot_arm_node::commander
