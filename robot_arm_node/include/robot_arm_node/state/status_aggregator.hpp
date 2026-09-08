/**
 * @file status_aggregator.hpp
 * @brief 状态聚合器（C++）—— 订阅 /joint_states + TF2 → 合成 ArmStatus
 *
 * 对应 Python commander/status_aggregator.py。机械臂当前状态的唯一权威来源：
 *   - 维护关节位置/速度缓存（线程安全）
 *   - TF2 查询末端位姿（ArmPose），位姿历史数值微分估算末端速度（ArmTwist）
 *   - 维护命令执行 / 运镜 / 跟随等 flag，构建完整 ArmStatus
 * 非节点：共享调用方（C++ ArmCommanderNode）的 ROS 资源。
 *
 * 线程：/joint_states 回调线程写缓存，commander 线程读；所有读写经互斥保护。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_arm_interfaces/msg/arm_pose.hpp>
#include <robot_arm_interfaces/msg/arm_twist.hpp>
#include <robot_arm_interfaces/msg/arm_status.hpp>
#include <robot_arm_interfaces/msg/control_mode.hpp>

#include "robot_arm_node/motion/constants.hpp"

namespace robot_arm_node::state
{

using ArmPose     = robot_arm_interfaces::msg::ArmPose;
using ArmTwist    = robot_arm_interfaces::msg::ArmTwist;
using ArmStatus   = robot_arm_interfaces::msg::ArmStatus;
using ControlMode = robot_arm_interfaces::msg::ControlMode;

// 末端速度估算：保留最近 N 个位姿样本做数值微分（100Hz 下 ≈0.1s 窗口）
constexpr size_t VELOCITY_WINDOW_SIZE = 10;

class StatusAggregator
{
public:
  // node：共享其 ROS 资源（订阅 / TF / 时钟）。生命周期须长于本对象。
  explicit StatusAggregator(rclcpp::Node & node);

  // ── 末端位姿 / 速度 ─────────────────────────────────────────────────────────
  // 每次调用做一次 TF 查询；失败返回零位姿（与 Python property pose 一致）。
  ArmPose  pose() const;
  // 数值微分估算；样本不足时返回全零。
  ArmTwist twist() const;

  // TF 缓冲（共享给需要自行做运动学查询的组件，如 VelocityStreamServer 的 Jacobian）。
  // 一个节点只养一个 TransformListener，别再各建各的。
  std::shared_ptr<tf2_ros::Buffer> tf_buffer() const { return tf_buffer_; }

  // 当前语义控制模式（取 ControlMode 常量），由 /robot_arm/control_mode（latched）更新
  uint8_t active_control_mode() const;

  // ── 关节状态查询 ────────────────────────────────────────────────────────────
  std::map<std::string, double> joint_positions() const;
  std::map<std::string, double> joint_velocities() const;
  // 按指定顺序返回关节位置；未知关节记 0（缺省用 motion::JOINT_NAMES）。
  // ⚠️ 未知关节返回 0 而不是报错 —— 拿它当「目标位置」下发前**必须**先用
  //    has_joint_positions() 确认回读齐全，否则缺回读时会命令关节摆到 0 位。
  std::vector<double> joint_position_list(
      const std::vector<std::string> & names = motion::JOINT_NAMES) const;
  // 按指定顺序返回关节速度（/joint_states 回读）；未知关节记 0。
  // 只用于「是否已静止」这类判据，0 在这里是安全侧（缺回读 = 当作已静止，
  // 兜底判定退化为只看残差）。
  std::vector<double> joint_velocity_list(
      const std::vector<std::string> & names = motion::JOINT_NAMES) const;

  /// 指定的关节是否**全部**有回读。用于区分 joint_position_list() 里的
  /// 「真实的 0」与「缺回读被记成 0」。
  bool has_joint_positions(
      const std::vector<std::string> & names = motion::JOINT_NAMES) const;

  // ── 运动 / 运镜 / 跟随 flag ──────────────────────────────────────────────────
  bool is_moving() const;
  void set_moving(bool moving);              // 由 commander 状态机维护，避免速度微分抖动
  bool at_pose_start() const;
  void set_at_pose_start(bool at_start);
  bool camera_ready() const;
  void set_camera_ready(bool ready);
  void set_tracking(bool tracking, double img_err, double depth_err_m);

  // ── 命令执行状态（由 commander / action server 调用）────────────────────────
  void set_command_state(uint32_t command_id, uint8_t result);
  void set_pose_state(uint8_t state);        // STOWED / OBSERVE / SHOOTING
  void set_error(uint8_t error_code);
  uint8_t get_error_code() const;
  void clear_error();

  // ── 构建完整 ArmStatus（由 commander 周期调用）──────────────────────────────
  ArmStatus build_status_message() const;

private:
  void on_joint_state(const sensor_msgs::msg::JointState & msg);
  std::optional<ArmPose> lookup_pose() const;
  static ArmPose transform_to_pose(const geometry_msgs::msg::Transform & tf);

  rclcpp::Node & node_;
  rclcpp::Logger logger_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr                  mode_sub_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 关节状态缓存
  mutable std::mutex            joint_mtx_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_velocities_;

  // 位姿历史（速度估算）：(t 秒, 位姿)
  mutable std::mutex                        pose_mtx_;
  std::deque<std::pair<double, ArmPose>>    pose_history_;

  // 命令 / flag 状态
  mutable std::mutex state_mtx_;
  uint32_t executing_command_id_{0};
  uint8_t  command_result_{ArmStatus::RESULT_NONE};
  uint8_t  current_pose_state_{ArmStatus::POSE_STATE_OBSERVE};
  uint8_t  error_code_{ArmStatus::ERR_NONE};
  uint8_t  active_control_mode_{ControlMode::TRAJECTORY};   // 由 /robot_arm/control_mode 更新
  bool     is_moving_{false};
  bool     at_pose_start_{false};
  bool     camera_ready_{false};
  bool     is_tracking_{false};
  double   tracking_img_err_{0.0};
  double   tracking_depth_err_m_{0.0};
};

}  // namespace robot_arm_node::state
