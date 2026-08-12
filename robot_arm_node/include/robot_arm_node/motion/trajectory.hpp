/**
 * @file trajectory.hpp
 * @brief motion 层轨迹下发（C++）—— 降采样 + 关节速度 + JointTrajectory 构建 + 批量 IK 下发
 *
 * 对应 Python arm_motion/trajectory.py 的 decimate / build_joint_trajectory / solve_and_send：
 *   decimate               100Hz 路点降采样（每 k 取 1，保留首末点），减少 IK 求解次数
 *                          （30ms 间距兼作 IK 数值噪声的低通，加密路点会放大速度差分噪声）
 *   build_joint_trajectory 关节序列 + 时间序列 → JointTrajectory（中央差分算速度，端点零；
 *                          刻意不补加速度，五次样条经验证劣于三次，见 .cpp 注释）
 *   solve_and_send         路点批量 IK（种子延续 / 首帧零种子重试 / 失败沿用上帧）
 *                          → 首点前插入当前关节作起步融合段 → 发布
 * 依赖注入（seed / stop_check），无 StatusAggregator / is_stopped 硬耦合。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "robot_arm_node/motion/constants.hpp"
#include "robot_arm_node/tuning.hpp"    // JOINT_NAMES 等默认值
#include "robot_arm_node/motion/kinematics.hpp"   // GetPositionIK / solve_ik / make_pose_stamped
#include "robot_arm_node/motion/planning.hpp"     // Waypoint

namespace robot_arm_node::motion
{

// 100Hz 路点降采样：每 k 个取 1，始终保留首/末点（不改起止位姿与总时长）
std::vector<Waypoint> decimate(const std::vector<Waypoint> & pts, int k);

// 由关节位置序列 + 时间序列构建 JointTrajectory（中央差分算速度，端点为零）
trajectory_msgs::msg::JointTrajectory build_joint_trajectory(
    const std::vector<std::vector<double>> & joint_pos,
    const std::vector<double> & joint_t,
    const std::vector<std::string> & joint_names,
    const builtin_interfaces::msg::Time & stamp);

// 规划期批量 IK 的结果：区分「成功 / 被取消 / 目标不可达 / 其它错误」，
// 供上层把「IK 无解」映射为 action 的 "unreachable"（秒回、不进 ERROR），
// 而不是硬发退化轨迹再靠执行期超时兜底。
enum class PlanResult
{
  Success,      // 全程 IK 有解，已下发 JointTrajectory
  Cancelled,    // 规划期间被急停 / 取消
  Unreachable,  // 首帧 / 末帧 / 成片连续 IK 无解 —— 路径驶出可达域，未下发
  Error,        // 服务不可用 / 空路点等其它失败
};

// 批量 IK + JointTrajectory 下发（对应 Python solve_and_send）。
//   路点降采样 → 逐点 IK（种子延续，首帧失败零种子重试，零星漏解沿用上帧）
//   → 中央差分算关节速度 → 构建并发布 JointTrajectory。
// 依赖注入、无状态：seed 由调用方提供，stop_check 由调用方注入（急停 / 取消合成一个判据）。
//   node    仅用于 get_clock()（时间戳），不读其它状态。
//   逐点 IK 阻塞等 future，须由 MultiThreadedExecutor 的执行线程调用（见 kinematics.hpp）。
// 可达性判定（规划期即知，不下发退化轨迹）：首帧无解 / 末帧无解 / 连续无解达阈值
//   → 返回 Unreachable；零星单点漏解仍沿用上帧容忍。见 PlanResult 各枚举语义。
PlanResult solve_and_send(
    rclcpp::Node & node,
    const rclcpp::Client<GetPositionIK>::SharedPtr & ik_client,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & traj_pub,
    const std::vector<Waypoint> & all_pts,
    const std::vector<double> & seed,
    const std::vector<std::string> & joint_names = JOINT_NAMES,
    const std::string & group = PLANNING_GROUP,
    const std::string & eef_link = EEF_LINK,
    const std::string & base_frame = BASE_FRAME,
    int decimate_k = tuning::params().ik_decimate,
    double ik_timeout_s = tuning::params().ik_timeout_s,
    const std::function<bool()> & stop_check = nullptr,
    rclcpp::Logger logger = rclcpp::get_logger("arm_motion"),
    // 出参：成功时写入**末点的关节解**。上层用它做「只判臂 J1-3」的到位判据
    // （末端 gimbal_tool0 在云台之后，笛卡尔判据会被云台回读拖累，见
    //  commander/motion_policy.hpp 的 is_at_joints_prefix 注释）。
    std::vector<double> * final_joints = nullptr);

}  // namespace robot_arm_node::motion
