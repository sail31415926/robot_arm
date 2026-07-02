/**
 * @file kinematics.hpp
 * @brief motion 层逆解（C++）—— 统一的同步 IK 求解（/compute_ik）
 *
 * 对应 Python arm_motion/kinematics.py 的 solve_ik / make_pose_stamped。
 * IK 种子由调用方显式传入（commander 传 StatusAggregator，controller 传本地缓存）；
 * 结果按 joint_names 顺序重排，不依赖求解器返回顺序。
 *
 * 线程：solve_ik 阻塞等 future，须由 MultiThreadedExecutor + 独立/可重入回调组的线程
 *       调用（与 Python 版用独立 action 执行线程一致），否则单线程 executor 会死锁。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_arm_node/motion/constants.hpp"

namespace robot_arm_node::motion
{

using GetPositionIK = moveit_msgs::srv::GetPositionIK;

// IK 结果：joints 为 nullopt 表示失败；error_code == 1 (SUCCESS) 表示成功
struct IkResult
{
  std::optional<std::vector<double>> joints;
  int error_code{-1};
};

// 构建 PoseStamped（供批量/单点 IK 复用）
geometry_msgs::msg::PoseStamped make_pose_stamped(
    double x, double y, double z, double qx, double qy, double qz, double qw,
    const std::string & frame_id, const builtin_interfaces::msg::Time & stamp);

// 同步 IK 求解。
//   wait_service=true  → wait_for_service(0.5s)（单点/首次）
//   wait_service=false → service_is_ready()（批量逐点，廉价）
//   eef_link 为空串时不设 ik_link_name；结果按 joint_names 顺序重排。
IkResult solve_ik(
    const rclcpp::Client<GetPositionIK>::SharedPtr & client,
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    const std::vector<double> & seed,
    const std::vector<std::string> & joint_names = JOINT_NAMES,
    const std::string & group = PLANNING_GROUP,
    const std::string & eef_link = "",
    double timeout_s = IK_TIMEOUT_S,
    bool wait_service = true,
    rclcpp::Logger logger = rclcpp::get_logger("arm_motion"));

}  // namespace robot_arm_node::motion
