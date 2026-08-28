/**
 * @file joint_limits_guard.hpp
 * @brief 关节限位来源 —— 订阅 latched /robot_description 解析 URDF 取每轴 lower/upper
 *
 * ★ 为什么本包又写了一份而不是复用 robot_arm_node 的 JointLimitsCache
 *   robot_arm_node 的 CMakeLists 只做了 install(TARGETS ...)，没有 ament_export_targets /
 *   ament_export_libraries，下游包 find_package(robot_arm_node) 拿不到可链接的目标。
 *   要复用就得改它的 CMakeLists —— 而「尽量不动既有文件」是本次的硬要求。
 *   代价可接受：**限位数值的真相源仍然只有 URDF 这一份**，重复的只是几十行解析代码，
 *   不是把 lower/upper 抄成第二份常量（2026-07-28 J2/J3 零点重标定就把限位整体平移过，
 *   写死常量必然漏同步 —— 那才是真正要避免的重复）。
 *   哪天 robot_arm_node 导出了目标，把本文件删掉换成它即可，接口是刻意对齐的。
 *
 * 数据来源与就绪时序同 JointLimitsCache：robot_state_publisher 以 transient_local
 * （latched）发布 /robot_description，晚起的节点也能收到最后一帧；构造后首帧未必立刻到，
 * 调用方须按 **fail-open** 处理 ready()==false（放行并告警，不要把机械臂卡死）。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_arm_teach/teach_types.hpp"

namespace robot_arm_teach
{

class JointLimitsGuard
{
public:
  // node：共享其 ROS 资源（订阅归属该节点）。生命周期须长于本对象。
  explicit JointLimitsGuard(rclcpp::Node & node,
                            const std::string & topic = TOPIC_DESCRIPTION);

  bool ready() const;

  // 当前限位表的快照。未就绪时返回空 map —— teach_validator 的 validate_limits()
  // 收到空 map 即 fail-open，语义是一致的。
  JointBoundMap bounds() const;

private:
  void on_description(const std_msgs::msg::String & msg);

  rclcpp::Logger logger_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  mutable std::mutex mtx_;
  JointBoundMap bounds_;
  bool ready_{false};
};

}  // namespace robot_arm_teach
