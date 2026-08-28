/**
 * @file joint_limits_guard.cpp
 * @brief JointLimitsGuard 实现 —— latched /robot_description + urdf::Model 解析
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/joint_limits_guard.hpp"

#include <urdf/model.h>

namespace robot_arm_teach
{

JointLimitsGuard::JointLimitsGuard(rclcpp::Node & node, const std::string & topic)
: logger_(node.get_logger())
{
  // transient_local + KeepLast(1)：匹配 robot_state_publisher 的 latched 发布，
  // 晚于 rsp 启动也能立刻收到最后一帧
  rclcpp::QoS qos(1);
  qos.transient_local().reliable();

  sub_ = node.create_subscription<std_msgs::msg::String>(
      topic, qos, [this](const std_msgs::msg::String & msg) { on_description(msg); });

  RCLCPP_INFO(logger_, "JointLimitsGuard: 等待 %s（latched）解析关节限位", topic.c_str());
}

void JointLimitsGuard::on_description(const std_msgs::msg::String & msg)
{
  urdf::Model model;
  if (!model.initString(msg.data)) {
    RCLCPP_ERROR(logger_, "JointLimitsGuard: URDF 解析失败，限位校验将 fail-open");
    return;
  }

  JointBoundMap parsed;
  for (const auto & [name, joint] : model.joints_) {
    if (!joint) continue;
    // continuous 轴无界、fixed 轴无 limit 字段 —— 都不产出条目（即不校验）
    if (joint->type == urdf::Joint::CONTINUOUS || joint->type == urdf::Joint::FIXED) continue;
    if (!joint->limits) continue;
    parsed[name] = JointBound{joint->limits->lower, joint->limits->upper};
  }

  std::string summary;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    bounds_ = std::move(parsed);
    ready_  = !bounds_.empty();
    for (const auto & [name, b] : bounds_) {
      summary += name + "[" + std::to_string(b.lower).substr(0, 6) + "," +
                 std::to_string(b.upper).substr(0, 6) + "] ";
    }
  }
  RCLCPP_INFO(logger_, "JointLimitsGuard: 已解析 %s", summary.c_str());
}

bool JointLimitsGuard::ready() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return ready_;
}

JointBoundMap JointLimitsGuard::bounds() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return bounds_;
}

}  // namespace robot_arm_teach
