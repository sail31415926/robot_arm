/**
 * @file motion_executor.hpp
 * @brief 共享运动执行引擎（C++）—— Cartesian → IK → JointTrajectory / 批量下发
 *
 * 对应 Python commander/motion_executor.py。运动算法核心下沉在 motion 库
 * （solve_ik / Ruckig plan_orbit_waypoints / solve_and_send），本类只做「委托 +
 * 持有共享 ROS 资源」：
 *   - 持有 JointTrajectory 发布器 + /compute_ik 客户端（归属传入的 Node）
 *   - 状态（IK 种子 / 当前关节 / 末端位姿）统一向 state::StatusAggregator 读，
 *     不自建 /joint_states 订阅与 TF，避免同一进程两份状态源
 *   - 急停判据 is_stopped 由 commander 注入（无状态耦合）
 *
 * 能力：
 *   plan_and_execute  点到点：Cartesian 目标 → 单点 IK → 两点 JointTrajectory
 *   go_to_joints      关节空间直驱（STOWED / 回零，跳过 IK）
 *   solve_and_send    批量 IK → 多点 JointTrajectory（委托
 * motion::solve_and_send） plan_orbit_ruckig 球面环绕运镜（相机始终朝向球心）
 *   plan_line_ruckig  笛卡尔直线运镜（位置沿线 + 姿态 slerp，末端严格直线）
 *   stop              急停：当前位置发零速度 JointTrajectory
 *
 * 线程：IK 阻塞等 future，须由 MultiThreadedExecutor 的执行线程调用
 *       （见 motion/kinematics.hpp），否则单线程 executor 会死锁。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <robot_arm_interfaces/msg/arm_pose.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <utility>
#include <vector>

#include "robot_arm_node/commander/motion_policy.hpp"  // Speed
#include "robot_arm_node/motion/planning.hpp"          // Waypoint
#include "robot_arm_node/motion/trajectory.hpp"        // PlanResult
#include "robot_arm_node/state/status_aggregator.hpp"

namespace robot_arm_node::commander {

using ArmPose = robot_arm_interfaces::msg::ArmPose;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;

// plan_and_execute / go_to_joints 的结果（对应 Python 返回 dict）
struct ExecResult {
    bool success{false};
    std::string exit_reason;  // "reached" | "unreachable" | "stopped" | "sent"
                              // | "error"
    int error_code{0};
    // 本次下发轨迹的**终点关节解**（6 轴）。上层拿它做「只判臂
    // J1-3」的到位判据， 不再用末端位姿比较——末端 gimbal_tool0
    // 在云台之后，云台回读不收敛会让笛卡尔 判据永不满足（见
    // commander/motion_policy.hpp 的 is_at_joints_prefix 注释）。
    std::vector<double> target_joints;
};

class MotionExecutor {
   public:
    // node：共享其 ROS
    // 资源；status：状态唯一真相源；is_stopped：急停判据（commander 注入）。
    MotionExecutor(rclcpp::Node& node, state::StatusAggregator& status,
                   std::function<bool()> is_stopped);

    // 点对点：Cartesian 目标 → 单点 IK → 两点 JointTrajectory（JTC 插值）
    ExecResult plan_and_execute(const ArmPose& target, const Speed& speed);

    // 急停：在当前位置发布零速度 JointTrajectory
    void stop();

    // 关节空间直驱（STOWED / 回零）：当前 → target_joints，耗时 duration_sec
    ExecResult go_to_joints(const std::vector<double>& target_joints,
                            double duration_sec = 2.0);

    // 当前关节 / 末端位姿 —— 向 StatusAggregator 读
    std::vector<double> get_current_joints() const;
    ArmPose get_ee_pose() const;

    // 同步 IK（种子取自 StatusAggregator）。返回 (joints|nullopt, error_code)。
    std::pair<std::optional<std::vector<double>>, int> ik_sync(
        const geometry_msgs::msg::PoseStamped& pose_stamped);

    // 批量 IK + JointTrajectory 下发（委托 motion::solve_and_send）。
    // cancel_check 与内部 is_stopped 合成一个急停判据。
    // 返回 PlanResult：Unreachable 表示 IK 无解（供上层映射为 "unreachable"）。
    // final_joints 出参：成功时写入末点关节解，供上层做只判臂的到位判据
    motion::PlanResult solve_and_send(
        const std::vector<motion::Waypoint>& all_pts,
        std::function<bool()> cancel_check = nullptr,
        std::vector<double>* final_joints = nullptr);

    // 球面轨道运镜：plan_orbit_waypoints + solve_and_send（相机始终朝向球心）
    // 限制取 speed 位置/姿态分量的更严者（同 plan_line_ruckig）：环绕段既有姿态
    // 转动也有位置行程，纯径向推拉更是只有位置行程，只喂姿态一组会超 v_pos 限制
    motion::PlanResult plan_orbit_ruckig(
        double ox, double oy, double oz, double theta0, double phi0, double r0,
        double theta1, double phi1, double r1, const Speed& speed,
        std::function<bool()> cancel_check = nullptr,
        std::vector<double>* final_joints = nullptr);

    // 笛卡尔直线运镜：plan_line_waypoints + solve_and_send
    // （位置沿线插值、姿态 slerp，末端严格走直线；限制取 speed
    // 位置/姿态分量的更严者）
    motion::PlanResult plan_line_ruckig(
        const ArmPose& start, const ArmPose& end, const Speed& speed,
        std::function<bool()> cancel_check = nullptr,
        std::vector<double>* final_joints = nullptr);

    // 可行性预判（Ruckig 几何规划 + 起点/终点 IK 可达性检查，不下发轨迹）：
    // 用于执行前判断 start→end
    // 直线能否规划成功，规划失败时避免先把机械臂搬到起点再落空
    bool can_plan_line(const ArmPose& start, const ArmPose& end,
                       const Speed& speed);

   private:
    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    state::StatusAggregator& status_;
    std::function<bool()> is_stopped_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        traj_pub_;
    rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
};

}  // namespace robot_arm_node::commander
