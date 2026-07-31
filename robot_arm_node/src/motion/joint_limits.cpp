/**
 * @file joint_limits.cpp
 * @brief JointLimitsCache 实现 —— 订阅 /robot_description（latched）+ urdf::Model 解析
 *
 * @version 1.0
 * @date 2026-07-31
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/motion/joint_limits.hpp"

#include <urdf/model.h>

namespace robot_arm_node::motion
{

JointLimitsCache::JointLimitsCache(rclcpp::Node & node, const std::string & topic)
: logger_(node.get_logger())
{
  // transient_local + KeepLast(1)：匹配 robot_state_publisher 的 latched 发布，
  // 晚于 rsp 启动也能立刻收到最后一帧
  rclcpp::QoS qos(1);
  qos.transient_local().reliable();

  sub_ = node.create_subscription<std_msgs::msg::String>(
      topic, qos, [this](const std_msgs::msg::String & msg) { on_description(msg); });

  RCLCPP_INFO(logger_, "JointLimitsCache: 等待 '%s'（latched）解析关节限位", topic.c_str());
}

void JointLimitsCache::on_description(const std_msgs::msg::String & msg)
{
  urdf::Model model;
  if (!model.initString(msg.data)) {
    RCLCPP_ERROR(logger_, "JointLimitsCache: URDF 解析失败，限位校验将 fail-open");
    return;
  }

  std::map<std::string, JointLimit> parsed;
  for (const auto & [name, joint] : model.joints_) {
    if (!joint) continue;
    // continuous 轴无界、fixed 轴无 limit 字段 —— 都不产出条目（即不校验）
    if (joint->type == urdf::Joint::CONTINUOUS || joint->type == urdf::Joint::FIXED) continue;
    if (!joint->limits) continue;
    parsed[name] = JointLimit{joint->limits->lower, joint->limits->upper};
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    limits_ = std::move(parsed);
    ready_  = !limits_.empty();
  }

  std::string summary;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & [name, lim] : limits_) {
      summary += name + "[" + std::to_string(lim.lower).substr(0, 6) + "," +
                 std::to_string(lim.upper).substr(0, 6) + "] ";
    }
  }
  RCLCPP_INFO(logger_, "JointLimitsCache: 已解析 %s", summary.c_str());
}

bool JointLimitsCache::ready() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return ready_;
}

std::optional<JointLimit> JointLimitsCache::limit(const std::string & joint_name) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = limits_.find(joint_name);
  if (it == limits_.end()) return std::nullopt;
  return it->second;
}

bool JointLimitsCache::within_limits(const std::vector<std::string> & joint_names,
                                     const std::vector<double> & positions,
                                     std::string * offender,
                                     double * offending_value,
                                     JointLimit * offending_limit) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!ready_) return true;   // fail-open：未拿到 URDF 时不拦（调用方告警）

  const size_t n = std::min(joint_names.size(), positions.size());
  for (size_t i = 0; i < n; ++i) {
    auto it = limits_.find(joint_names[i]);
    if (it == limits_.end()) continue;   // 该轴无界/未声明限位
    if (positions[i] < it->second.lower || positions[i] > it->second.upper) {
      if (offender)        *offender        = joint_names[i];
      if (offending_value) *offending_value = positions[i];
      if (offending_limit) *offending_limit = it->second;
      return false;
    }
  }
  return true;
}

}  // namespace robot_arm_node::motion
