/**
 * @file velocity_stream_server.hpp
 * @brief 速度流控制 —— 关节速度 / 末端 6 维 twist → 积分成位置流，统一走 JTC
 *
 * 两条产品总线在这里汇合，出口只有一个：
 *
 *   /robot_arm/cmd/joint_velocity (ArmJointVelocityCommand, J1-3 rad/s) ─┐
 *   /robot_arm/follow_command     (ArmFollowCommand, 末端 6 维 twist)  ──┤
 *                                   └ 6×6 几何 Jacobian DLS → q̇(6)      │
 *                                                                        ▼
 *                                        q̇ 限幅 → 积分成角度 → 按 URDF 限位夹紧
 *                                                                        ▼
 *                              /arm_controller/joint_trajectory（JTC，6 轴，50Hz）
 *
 * 【为什么速度也走 joint_trajectory（2026-08-04 改）】
 *   原先速度模式要切到独立的 arm_velocity_controller（实物对应 CiA402 PV(3)），
 *   代价是**控制器切换**，而 ros2_control 的命令接口在控制器停用后值原样保留：
 *   速度模式下机械臂走开了，位置命令缓冲还停在切换前那一刻，切回轨迹模式时被回放
 *   —— 机械臂冲回旧位姿（Gazebo 瞬移、实物是一次高速运动）。同一套切换还带来：
 *   轨迹类动作（收纳位/观察位）在速度模式下静默失效、云台要额外挂保持控制器、
 *   切换本身有 STRICT 资源竞态。
 *   改成位置流后这些问题一次性消失：全程只有 arm_controller 一个控制器在跑，
 *   没有切换就没有陈旧命令；JTC 本来就管 J1-6，末端姿态那三维（云台）自然包含在内；
 *   停止发布 = JTC 保持最后一点 = 原地停住。
 *   代价是放弃驱动器内部速度环（PV），高动态场景平滑度略逊 —— 需要时可由
 *   mode_manager_node 的 velocity_backend 参数切回 PV 后端。
 *
 * 【安全闸】任一不满足即停止下发（JTC 自动保持最后一点）：
 *   ① 当前语义模式必须是 JOINT_VELOCITY（不自动切模式，切模式只经 mode_manager）
 *   ② Commander 未处于 STOPPED / ERROR（is_stopped）
 *   ③ 指令未断流（command_timeout，默认 0.3s）
 *   ④ 笛卡尔指令还需 Jacobian 可算（URDF 就绪 + TF 查得到），算不出必须停
 *  另有：q̇ 整体等比缩放（保运动方向）、积分角度按 URDF 限位夹紧。
 *
 * @version 3.0
 * @date 2026-08-04
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <robot_arm_interfaces/msg/arm_follow_command.hpp>
#include <robot_arm_interfaces/msg/arm_joint_velocity_command.hpp>

#include "robot_arm_node/motion/jacobian.hpp"
#include "robot_arm_node/motion/joint_limits.hpp"
#include "robot_arm_node/state/status_aggregator.hpp"

namespace robot_arm_node::commander
{

class VelocityStreamServer
{
public:
  using FollowCommand   = robot_arm_interfaces::msg::ArmFollowCommand;
  using JointVelCommand = robot_arm_interfaces::msg::ArmJointVelocityCommand;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

  // node：共享其 ROS 资源；status：取当前控制模式、TF 与关节回读；is_stopped：急停/故障闸
  VelocityStreamServer(rclcpp::Node & node, state::StatusAggregator & status,
                       std::function<bool()> is_stopped);

  // 速度流是否正在下发（非零指令）—— 供 ArmStatus.is_moving 使用
  bool is_streaming() const;

  // 急停：立刻停流并丢弃缓存指令（由 ArmStop 调用）。
  // 之后 is_stopped() 会一直挡住新指令，直到 ArmResetError 复位状态机。
  void emergency_stop();

private:
  // 指令来源：两条总线互斥使用，后到的覆盖先到的
  enum class Source { None, Joint, Cartesian };

  void on_follow_command(const FollowCommand & msg);
  void on_joint_command(const JointVelCommand & msg);
  void on_tick();

  // 由缓存的指令解出 6 关节速度；失败（Jacobian 不可用等）返回 false
  bool solve_joint_velocity(Source src, const double * cmd, std::vector<double> * qdot);
  // 整体等比缩放到各自速度上限内，保运动方向
  void scale_to_limits(std::vector<double> * qdot) const;
  void publish_trajectory(const std::vector<double> & positions,
                          const std::vector<double> & velocities);
  // 停止下发（幂等）。不发任何命令 —— JTC 自动保持最后一个点。
  void halt(const char * reason);

  rclcpp::Node & node_;
  rclcpp::Logger logger_;
  state::StatusAggregator & status_;
  std::function<bool()> is_stopped_;

  motion::ArmJacobian      jacobian_;
  motion::JointLimitsCache limits_;

  // 参数
  double rate_hz_{50.0};             // 下发频率。**别调到 100Hz**：JTC 每收一条新轨迹就
                                     // 丢弃旧的重新插值，抢占太频繁反而跟不动（实测只剩
                                     // 两三成，本仓 servo_config.yaml 同样压到 50Hz）
  double command_timeout_{0.3};
  double lookahead_{0.05};           // s，轨迹点的 time_from_start，同时用于位置前伸
  double max_linear_speed_{0.2};     // m/s
  double max_angular_speed_{1.0};    // rad/s（消息里是 deg/s，进来先换算）
  double max_joint_speed_{1.0};      // rad/s，臂 J1-3
  double max_gimbal_speed_{2.0};     // rad/s，云台 J4-6
  double singularity_eps_{0.02};
  double damping_max_{0.05};
  std::string traj_topic_;
  std::vector<std::string> arm_joints_;      // 臂关节（默认 J1-3）
  std::vector<std::string> gimbal_joints_;   // 云台关节（默认 J4-6）
  std::vector<std::string> all_joints_;      // arm + gimbal，Jacobian 列顺序 / 轨迹关节顺序

  // 最新指令（订阅线程写，定时器线程读）
  // Cartesian：cmd_[0..2] 线速度 m/s，[3..5] 角速度 rad/s
  // Joint：    cmd_[0..2] 臂关节角速度 rad/s（云台恒 0）
  mutable std::mutex cmd_mtx_;
  double   cmd_[6]{0, 0, 0, 0, 0, 0};
  Source   cmd_src_{Source::None};
  double   cmd_stamp_s_{0.0};
  bool     has_cmd_{false};

  // 积分状态（只在定时器线程访问）
  std::vector<double> target_;
  bool   seeded_{false};
  double last_tick_s_{0.0};

  std::atomic<bool> streaming_{false};

  rclcpp::Subscription<FollowCommand>::SharedPtr   follow_sub_;
  rclcpp::Subscription<JointVelCommand>::SharedPtr joint_sub_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr    traj_pub_;
  rclcpp::TimerBase::SharedPtr                     timer_;
};

}  // namespace robot_arm_node::commander
