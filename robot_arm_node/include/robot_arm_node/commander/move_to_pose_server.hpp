/**
 * @file move_to_pose_server.hpp
 * @brief ArmMoveToPose Action 执行逻辑（C++）—— 姿态切换（STOWED / OBSERVE / SHOOTING）
 *
 * 对应 Python commander/move_to_pose_server.py。不持有 Action Server 本体（由 commander
 * 承载），只提供 execute(goal_handle)→Result；终态（succeed/abort/canceled）由 commander 决定。
 * STOWED 走关节空间回零；OBSERVE/SHOOTING 走 Cartesian IK；支持 return_to_start 原路返回。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robot_arm_interfaces/action/arm_move_to_pose.hpp>
#include <robot_arm_interfaces/msg/arm_pose.hpp>

#include "robot_arm_node/commander/motion_executor.hpp"
#include "robot_arm_node/commander/execution_monitor.hpp"
#include "robot_arm_node/state/status_aggregator.hpp"

namespace robot_arm_node::commander
{

class MoveToPoseServer
{
public:
  using Action     = robot_arm_interfaces::action::ArmMoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  // observe_pose：读取 OBSERVE 预定义位姿（由 commander 从 ROS param 提供）
  MoveToPoseServer(rclcpp::Node & node, MotionExecutor & motion,
                   state::StatusAggregator & status, ExecutionMonitor & monitor,
                   std::function<ArmPose()> observe_pose);

  // 在专用线程中阻塞执行，返回 Result（不调用 goal_handle 终态）
  Action::Result execute(const std::shared_ptr<GoalHandle> & gh);

private:
  Action::Result execute_stowed(const std::shared_ptr<GoalHandle> & gh);
  Action::Result wait_arrival(const std::shared_ptr<GoalHandle> & gh, const ArmPose & target,
                              const Speed & speed, double p_lo, double p_hi);
  std::optional<ArmPose> resolve_target(const Action::Goal & goal);

  rclcpp::Node &            node_;
  rclcpp::Logger            logger_;
  MotionExecutor &          motion_;
  state::StatusAggregator & status_;
  ExecutionMonitor &        monitor_;
  std::function<ArmPose()>  observe_pose_;
};

}  // namespace robot_arm_node::commander
