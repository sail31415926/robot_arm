/**
 * @file trajectory_shot_server.hpp
 * @brief ArmTrajectoryShot Action 执行逻辑（C++）—— 直线运镜 / 球面环绕运镜
 *
 * 对应 Python commander/trajectory_shot_server.py。同 MoveToPose：只提供 execute→Result，
 * 终态由 commander 决定。MOTION_LINEAR 走相对直线（PTP 分段）；MOTION_ORBIT 先 PTP 到起始
 * 球坐标、再 Ruckig 1-DOF 球面轨道；均支持 return_to_start。is_stopped 注入（dwell / orbit
 * 取消判据用）。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robot_arm_interfaces/action/arm_trajectory_shot.hpp>
#include <robot_arm_interfaces/msg/arm_pose.hpp>

#include "robot_arm_node/commander/motion_executor.hpp"
#include "robot_arm_node/commander/execution_monitor.hpp"
#include "robot_arm_node/state/status_aggregator.hpp"

namespace robot_arm_node::commander
{

class TrajectoryShotServer
{
public:
  using Action     = robot_arm_interfaces::action::ArmTrajectoryShot;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  TrajectoryShotServer(rclcpp::Node & node, MotionExecutor & motion,
                       state::StatusAggregator & status, ExecutionMonitor & monitor,
                       std::function<bool()> is_stopped);

  Action::Result execute(const std::shared_ptr<GoalHandle> & gh);

private:
  Action::Result execute_linear(const std::shared_ptr<GoalHandle> & gh, const Action::Goal & goal,
                                const Speed & speed);
  Action::Result execute_orbit(const std::shared_ptr<GoalHandle> & gh, const Action::Goal & goal,
                               const Speed & speed);
  // 到达起始点后置位 → 停顿（可取消）→ 复位；返回 false 表示停顿期间被取消/急停
  bool dwell_at_start(const std::shared_ptr<GoalHandle> & gh, const char * label);
  // 下发一段 IK 轨迹并等待到位（发 Feedback）
  Action::Result move_and_wait(const std::shared_ptr<GoalHandle> & gh, const ArmPose & target,
                               const Speed & speed, const char * label, double p_lo, double p_hi,
                               double azimuth = 0.0, double elevation = 0.0, double radius = 0.0);
  // 轨迹已下发，仅轮询等到位（发 Feedback）
  Action::Result wait_at_pose(const std::shared_ptr<GoalHandle> & gh, const ArmPose & target,
                              const char * label, double p_lo, double p_hi,
                              double azimuth = 0.0, double elevation = 0.0, double radius = 0.0);
  static ArmPose sphere_to_pose(double azimuth_deg, double elevation_deg, double radius_m,
                                double ox, double oy, double oz);

  rclcpp::Node &            node_;
  rclcpp::Logger            logger_;
  MotionExecutor &          motion_;
  state::StatusAggregator & status_;
  ExecutionMonitor &        monitor_;
  std::function<bool()>     is_stopped_;
};

}  // namespace robot_arm_node::commander
