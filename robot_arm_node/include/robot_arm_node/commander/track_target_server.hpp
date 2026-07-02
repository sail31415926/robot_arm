/**
 * @file track_target_server.hpp
 * @brief ArmTrackTarget Action 执行逻辑（C++）—— 视觉伺服目标跟随
 *
 * 对应 Python commander/track_target_server.py。经 SetParameters 启停 visp_ibvs_node，
 * 订阅 /red_detector/feature 计算图像 / 深度误差，监测收敛 / 特征丢失 / 超时 / cancel。
 *
 * 注意：与另外两个 server 不同，本 server 的 execute 自行调用 goal_handle 终态
 *       （succeed/abort/canceled），commander 包装层只做状态转换（与 Python 一致）。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <robot_arm_interfaces/action/arm_track_target.hpp>

#include "robot_arm_node/state/status_aggregator.hpp"

namespace robot_arm_node::commander
{

class TrackTargetServer
{
public:
  using Action     = robot_arm_interfaces::action::ArmTrackTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using SetParameters = rcl_interfaces::srv::SetParameters;

  TrackTargetServer(rclcpp::Node & node, state::StatusAggregator & status,
                    std::function<bool()> is_stopped);

  // 阻塞执行至退出条件；内部调用 goal_handle 终态
  Action::Result execute(const std::shared_ptr<GoalHandle> & gh);

  // 供 commander cancel 回调调用：通知循环退出
  void cancel();

private:
  void on_feature(const geometry_msgs::msg::PointStamped & msg);
  // 向 visp_ibvs_node 写参数（bool + double）；服务不可用（仿真）返回 true
  bool set_params(const std::vector<rcl_interfaces::msg::Parameter> & params);
  bool set_paused(bool paused);

  rclcpp::Node &            node_;
  rclcpp::Logger            logger_;
  state::StatusAggregator & status_;
  std::function<bool()>     is_stopped_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr feat_sub_;
  rclcpp::Client<SetParameters>::SharedPtr                         param_cli_;

  // 特征缓存（线程安全）
  std::mutex feat_mtx_;
  double     feat_x_{0.0}, feat_y_{0.0}, feat_z_{0.0};
  double     last_feat_s_{0.0};
  bool       has_feat_{false};

  std::atomic<bool> cancel_flag_{false};
};

}  // namespace robot_arm_node::commander
