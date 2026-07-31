/**
 * @file joint_limits.hpp
 * @brief 关节限位缓存 —— 从 /robot_description 话题解析 URDF，取每轴 lower/upper
 *
 * 为什么不写死常量：限位会随实机标定变（2026-07-28 J2/J3 零点重标定就把 J2/J3 的
 * lower/upper 整体平移了）。写死等于制造第二份真相，改了 URDF 必然漏同步。
 *
 * 数据来源：`robot_state_publisher` 以 **transient_local**（latched）发布
 * `/robot_description`（std_msgs/String），任何后起的节点都能收到最后一帧，
 * 因此无需 launch 传参、无需向别的节点要参数。
 *
 * 就绪时序：构造后首帧未必立刻到（DDS 发现有延迟）。调用方须按 fail-open 处理
 * `ready()==false`（放行并告警），而不是把机械臂卡死 —— 与 visp_ibvs_node 的
 * 碰撞守护在 /check_state_validity 不可用时的策略一致。
 *
 * @version 1.0
 * @date 2026-07-31
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_arm_node::motion
{

// 单轴限位（rad）。连续轴（continuous）不产出条目 —— 无界即不校验。
struct JointLimit
{
  double lower{0.0};
  double upper{0.0};
};

class JointLimitsCache
{
public:
  // node：共享其 ROS 资源（订阅归属该节点）。topic 默认 /robot_description。
  explicit JointLimitsCache(rclcpp::Node & node,
                            const std::string & topic = "/robot_description");

  // URDF 是否已解析成功（收到首帧且解析通过）
  bool ready() const;

  // 取某轴限位；未就绪或该轴无界/不存在时返回 nullopt
  std::optional<JointLimit> limit(const std::string & joint_name) const;

  // 逐轴校验 positions 是否都在限位内（长度须与 joint_names 一致）。
  // 未就绪 → 返回 true（fail-open，由调用方告警）。
  // 越界时把第一个越界轴写入 offender / offending_value / offending_limit。
  bool within_limits(const std::vector<std::string> & joint_names,
                     const std::vector<double> & positions,
                     std::string * offender = nullptr,
                     double * offending_value = nullptr,
                     JointLimit * offending_limit = nullptr) const;

private:
  void on_description(const std_msgs::msg::String & msg);

  rclcpp::Logger logger_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  mutable std::mutex mtx_;
  std::map<std::string, JointLimit> limits_;
  bool ready_{false};
};

}  // namespace robot_arm_node::motion
