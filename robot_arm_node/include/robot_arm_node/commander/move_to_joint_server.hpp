/**
 * @file move_to_joint_server.hpp
 * @brief ArmMoveToJoint 执行体 —— 关节空间点到点（只动臂 J1-3，云台原样保持）
 *
 * 与 MoveToPoseServer 平级：同样只负责「一个 action 的执行逻辑」，状态机/终态判定
 * 留在 ArmCommanderNode，等待循环复用 ExecutionMonitor，下发复用
 * MotionExecutor::go_to_joints（两点 JointTrajectory，不过 IK）。
 *
 * 三道安全闸（顺序即代价从小到大，任一不过都**不下发任何指令**）：
 *   ① 个数校验：target_joints 必须 ARM_JOINT_COUNT 个            → "invalid_goal"
 *   ② 限位校验：JointLimitsCache（/robot_description 解析 URDF）  → "out_of_range"
 *   ③ 自碰撞：  /check_state_validity（move_group 提供）          → "collision"
 * ②③ 在数据源不可用时 fail-open + 节流告警（与 visp_ibvs_node 的碰撞守护同策略），
 * 不因为 move_group 没起就让关节控制不可用。
 *
 * 云台处理：下发的 JointTrajectory 仍是 6 轴（JTC claim 全 6 轴），J4-6 用**当前回读**
 * 填充 = 保持不动；到位判据只看 J1-3（云台未上电时回读不收敛，见 ARM_JOINT_COUNT 注释）。
 *
 * @version 1.0
 * @date 2026-07-31
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>

#include <robot_arm_interfaces/action/arm_move_to_joint.hpp>

#include "robot_arm_node/commander/execution_monitor.hpp"
#include "robot_arm_node/commander/motion_executor.hpp"
#include "robot_arm_node/motion/joint_limits.hpp"
#include "robot_arm_node/state/status_aggregator.hpp"

namespace robot_arm_node::commander
{

class MoveToJointServer
{
public:
  using Action     = robot_arm_interfaces::action::ArmMoveToJoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  MoveToJointServer(rclcpp::Node & node, MotionExecutor & motion,
                    state::StatusAggregator & status, ExecutionMonitor & monitor);

  // 阻塞执行（在 commander 派生的执行线程里调用）
  Action::Result execute(const std::shared_ptr<GoalHandle> & gh);

private:
  // 解析 goal → 绝对目标关节角（J1-3）。个数不对返回 nullopt。
  std::optional<std::vector<double>> resolve_target(const Action::Goal & goal,
                                                    const std::vector<double> & current_arm) const;

  // 自碰撞检查：J1-3 用目标值、J4-6 用当前回读，整体送 /check_state_validity。
  // 返回 false = 确认碰撞；服务不可用/超时 → true（fail-open）+ 告警。
  bool collision_free(const std::vector<double> & full_target);

  rclcpp::Node &            node_;
  rclcpp::Logger            logger_;
  MotionExecutor &          motion_;
  state::StatusAggregator & status_;
  ExecutionMonitor &        monitor_;

  motion::JointLimitsCache  limits_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_cli_;
};

}  // namespace robot_arm_node::commander
