/**
 * @file velocity_stream_server.cpp
 * @brief VelocityStreamServer 实现 —— 两条速度总线 → q̇ → 积分成位置流 → JTC
 *
 * @version 3.0
 * @date 2026-08-04
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/velocity_stream_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "robot_arm_node/motion/constants.hpp"

namespace robot_arm_node::commander
{

using namespace std::chrono_literals;
using ControlMode = robot_arm_interfaces::msg::ControlMode;

namespace
{
const char * TOPIC_FOLLOW_COMMAND = "/robot_arm/follow_command";
const char * TOPIC_JOINT_VELOCITY = "/robot_arm/cmd/joint_velocity";
constexpr double ZERO_EPS = 1e-6;   // 小于此值视为「停」
constexpr double DEG2RAD  = M_PI / 180.0;

double clamp(double v, double lim) { return std::max(-lim, std::min(lim, v)); }

double steady_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

VelocityStreamServer::VelocityStreamServer(rclcpp::Node & node,
                                           state::StatusAggregator & status,
                                           std::function<bool()> is_stopped)
: node_(node), logger_(node.get_logger()), status_(status), is_stopped_(std::move(is_stopped)),
  jacobian_(node), limits_(node)
{
  rate_hz_           = node_.declare_parameter("velocity_stream.rate_hz", 50.0);
  command_timeout_   = node_.declare_parameter("velocity_stream.command_timeout", 0.3);
  lookahead_         = node_.declare_parameter("velocity_stream.lookahead", 0.05);
  max_linear_speed_  = node_.declare_parameter("velocity_stream.max_linear_speed",
                                               motion::MAX_V_LIN_FOLLOW);
  max_angular_speed_ = node_.declare_parameter("velocity_stream.max_angular_speed",
                                               motion::MAX_V_ANG);
  max_joint_speed_   = node_.declare_parameter("velocity_stream.max_joint_speed", 1.0);
  max_gimbal_speed_  = node_.declare_parameter("velocity_stream.max_gimbal_speed", 2.0);
  singularity_eps_   = node_.declare_parameter("velocity_stream.singularity_eps", 0.02);
  damping_max_       = node_.declare_parameter("velocity_stream.damping_max", 0.05);
  traj_topic_        = node_.declare_parameter<std::string>(
      "velocity_stream.trajectory_topic", motion::TRAJ_TOPIC);

  const auto & JN = motion::JOINT_NAMES;
  arm_joints_ = node_.declare_parameter<std::vector<std::string>>(
      "velocity_stream.arm_joint_names",
      std::vector<std::string>(JN.begin(), JN.begin() + motion::ARM_JOINT_COUNT));
  gimbal_joints_ = node_.declare_parameter<std::vector<std::string>>(
      "velocity_stream.gimbal_joint_names",
      std::vector<std::string>(JN.begin() + motion::ARM_JOINT_COUNT, JN.end()));

  all_joints_ = arm_joints_;
  all_joints_.insert(all_joints_.end(), gimbal_joints_.begin(), gimbal_joints_.end());
  target_.assign(all_joints_.size(), 0.0);

  if (rate_hz_ < 1.0) rate_hz_ = 1.0;

  follow_sub_ = node_.create_subscription<FollowCommand>(
      TOPIC_FOLLOW_COMMAND, 10,
      [this](const FollowCommand & msg) { this->on_follow_command(msg); });
  joint_sub_ = node_.create_subscription<JointVelCommand>(
      TOPIC_JOINT_VELOCITY, 10,
      [this](const JointVelCommand & msg) { this->on_joint_command(msg); });
  traj_pub_ = node_.create_publisher<JointTrajectory>(traj_topic_, rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  timer_ = node_.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { this->on_tick(); });

  RCLCPP_INFO(logger_,
              "速度流控制就绪  %.0fHz → %s（%zu 轴，位置流）\n"
              "  关节速度 %s（%zu 轴）   末端 6 维 twist %s\n"
              "  限幅 线%.2fm/s 角%.2frad/s 臂%.2frad/s 云台%.2frad/s  断流 %.0fms 停流"
              "  （需先切 JOINT_VELOCITY 模式）",
              rate_hz_, traj_topic_.c_str(), all_joints_.size(),
              TOPIC_JOINT_VELOCITY, arm_joints_.size(), TOPIC_FOLLOW_COMMAND,
              max_linear_speed_, max_angular_speed_, max_joint_speed_, max_gimbal_speed_,
              command_timeout_ * 1000.0);
}

bool VelocityStreamServer::is_streaming() const
{
  return streaming_.load();
}

// ── 指令订阅：只缓存，换算与下发都在定时器里做 ────────────────────────────────
void VelocityStreamServer::on_follow_command(const FollowCommand & msg)
{
  const auto & t = msg.twist;
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  cmd_[0] = clamp(t.vx, max_linear_speed_);
  cmd_[1] = clamp(t.vy, max_linear_speed_);
  cmd_[2] = clamp(t.vz, max_linear_speed_);
  // ArmTwist 的角速度单位是 **度/秒**（见 msg 注释），换成 rad/s 再进 Jacobian
  cmd_[3] = clamp(t.wroll  * DEG2RAD, max_angular_speed_);
  cmd_[4] = clamp(t.wpitch * DEG2RAD, max_angular_speed_);
  cmd_[5] = clamp(t.wyaw   * DEG2RAD, max_angular_speed_);
  cmd_src_     = Source::Cartesian;
  cmd_stamp_s_ = node_.get_clock()->now().seconds();
  has_cmd_     = true;
}

void VelocityStreamServer::on_joint_command(const JointVelCommand & msg)
{
  if (msg.velocities.size() != arm_joints_.size()) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 2000,
                         "关节速度指令长度 %zu ≠ %zu，已丢弃",
                         msg.velocities.size(), arm_joints_.size());
    return;
  }
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  std::fill(std::begin(cmd_), std::end(cmd_), 0.0);
  for (size_t i = 0; i < msg.velocities.size() && i < 6; ++i) {
    cmd_[i] = clamp(msg.velocities[i], max_joint_speed_);
  }
  cmd_src_     = Source::Joint;
  cmd_stamp_s_ = node_.get_clock()->now().seconds();
  has_cmd_     = true;
}

// ── 指令 → 6 关节速度 ────────────────────────────────────────────────────────
bool VelocityStreamServer::solve_joint_velocity(Source src, const double * cmd,
                                                std::vector<double> * qdot)
{
  qdot->assign(all_joints_.size(), 0.0);

  if (src == Source::Joint) {
    // 关节速度直给臂那几轴，云台保持（q̇=0，积分后角度不变）
    for (size_t i = 0; i < arm_joints_.size() && i < qdot->size(); ++i) (*qdot)[i] = cmd[i];
    return true;
  }

  // 笛卡尔：6×6 几何 Jacobian + DLS 伪逆。算不出必须停车，不能 fail-open。
  auto J = jacobian_.geometric(*status_.tf_buffer(), all_joints_,
                               motion::BASE_FRAME, motion::EEF_LINK);
  if (!J) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 2000,
                         "Jacobian 不可用（URDF 未就绪或 TF 查询失败），末端速度指令已停");
    return false;
  }

  Eigen::Matrix<double, 6, 1> twist;
  twist << cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5];

  double sigma_min = 0.0;
  const Eigen::VectorXd sol =
      motion::ArmJacobian::dls_solve(*J, twist, singularity_eps_, damping_max_, &sigma_min);
  if (sigma_min < singularity_eps_) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 2000,
                         "接近奇异位形（σ_min=%.4f < %.4f），已加 DLS 阻尼，末端跟踪有偏差",
                         sigma_min, singularity_eps_);
  }
  for (Eigen::Index i = 0; i < sol.size() && i < static_cast<Eigen::Index>(qdot->size()); ++i) {
    (*qdot)[static_cast<size_t>(i)] = sol[i];
  }
  return true;
}

// 整体等比缩放：臂与云台各有上限，取最紧的比例统一缩放 —— 分开缩放会扭曲运动方向
void VelocityStreamServer::scale_to_limits(std::vector<double> * qdot) const
{
  const size_t n_arm = arm_joints_.size();
  double scale = 1.0;
  for (size_t i = 0; i < qdot->size(); ++i) {
    const double lim = (i < n_arm) ? max_joint_speed_ : max_gimbal_speed_;
    const double a = std::fabs((*qdot)[i]);
    if (a > lim) scale = std::min(scale, lim / a);
  }
  if (scale < 1.0) {
    for (auto & v : *qdot) v *= scale;
    RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 2000,
                         "关节速度超限，已整体缩放到 %.0f%%", scale * 100.0);
  }
}

// ── 固定周期下发 ──────────────────────────────────────────────────────────────
void VelocityStreamServer::on_tick()
{
  double cmd[6];
  Source src;
  double stamp;
  bool has_cmd;
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    std::copy(std::begin(cmd_), std::end(cmd_), std::begin(cmd));
    src = cmd_src_; stamp = cmd_stamp_s_; has_cmd = has_cmd_;
  }
  if (!has_cmd) return;   // 从未收到过指令：完全不碰轨迹总线

  // ① 模式闸：不自动切模式（切模式只经 mode_manager）
  if (status_.active_control_mode() != ControlMode::JOINT_VELOCITY) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 2000,
                         "收到速度指令但当前非 JOINT_VELOCITY 模式，已忽略"
                         "（先调 /robot_arm/switch_control_mode）");
    halt("模式不匹配");
    return;
  }
  // ② 急停 / 故障闸
  if (is_stopped_ && is_stopped_()) {
    halt("急停/故障");
    return;
  }
  // ③ 断流看门狗：停发布 = 停流，JTC 保持最后一点
  if ((node_.get_clock()->now().seconds() - stamp) > command_timeout_) {
    halt("指令断流");
    return;
  }

  // 目标速度为 0：停流即可（JTC 保持当前点），不必继续刷轨迹
  double norm2 = 0.0;
  for (double v : cmd) norm2 += v * v;
  if (norm2 < ZERO_EPS * ZERO_EPS) {
    halt("速度指令为 0");
    return;
  }

  std::vector<double> qdot;
  if (!solve_joint_velocity(src, cmd, &qdot)) {
    halt("Jacobian 不可用");
    return;
  }
  scale_to_limits(&qdot);

  // 起步（或断流后重新起步）时用当前回读播种积分器，避免从陈旧目标跳变
  const double tick_now = steady_now();
  if (!seeded_) {
    const auto cur = status_.joint_position_list(all_joints_);
    target_.assign(cur.begin(), cur.end());
    seeded_      = true;
    last_tick_s_ = tick_now;
  }
  // 积分步长用 steady_clock：定时器是 wall timer，而 Gazebo 的 /clock 只有 10Hz，
  // 用 ROS 时钟取 dt 会系统性丢步（九成的 tick 看到 0、第十拍看到 0.1s 再被上限削掉）
  const double dt = std::clamp(tick_now - last_tick_s_, 0.0, 5.0 / rate_hz_);
  last_tick_s_ = tick_now;

  std::vector<double> lead(all_joints_.size());
  for (size_t i = 0; i < all_joints_.size(); ++i) {
    target_[i] += qdot[i] * dt;
    // 下发点 = 设定点再前伸 lookahead，配 time_from_start=lookahead，使 JTC 插值斜率
    // 恰为 q̇。若直接发设定点，实际执行速度会被稀释成 q̇×dt/lookahead。
    lead[i] = target_[i] + qdot[i] * lookahead_;
    if (auto lim = limits_.limit(all_joints_[i])) {
      target_[i] = std::clamp(target_[i], lim->lower, lim->upper);
      lead[i]    = std::clamp(lead[i], lim->lower, lim->upper);
    }
  }

  publish_trajectory(lead, qdot);
  streaming_.store(true);
  status_.set_moving(true);
}

void VelocityStreamServer::publish_trajectory(const std::vector<double> & positions,
                                              const std::vector<double> & velocities)
{
  JointTrajectory traj;
  traj.joint_names = all_joints_;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions  = positions;
  pt.velocities = velocities;
  pt.time_from_start = rclcpp::Duration::from_seconds(lookahead_);
  traj.points.push_back(std::move(pt));
  traj_pub_->publish(traj);
}

void VelocityStreamServer::emergency_stop()
{
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    std::fill(std::begin(cmd_), std::end(cmd_), 0.0);
    has_cmd_ = false;   // 丢弃缓存，避免 ArmResetError 后残留指令自己接着跑
    cmd_src_ = Source::None;
  }
  streaming_.store(true);   // 强制走一次 halt（幂等）以复位状态并打日志
  halt("急停");
}

void VelocityStreamServer::halt(const char * reason)
{
  seeded_ = false;   // 积分器复位，下次起步重新用回读播种
  if (!streaming_.exchange(false)) return;   // 幂等
  // 不下发任何命令：JTC 自动保持最后一个轨迹点，原地停住
  status_.set_moving(false);
  RCLCPP_INFO(logger_, "速度流停止（%s），JTC 保持当前位置", reason);
}

}  // namespace robot_arm_node::commander
