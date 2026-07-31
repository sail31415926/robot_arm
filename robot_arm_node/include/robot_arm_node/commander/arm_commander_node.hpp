/**
 * @file arm_commander_node.hpp
 * @brief Arm Commander 主节点（C++）—— 承接 Director 语义指令，翻译为关节级控制
 *
 * 对应 Python commander/arm_commander_node.py。三明治中间层：Director ←→ 本节点 ←→ Driver。
 * 承载：
 *   - 4 个 Action Server：ArmMoveToPose / ArmMoveToJoint / ArmTrajectoryShot / ArmTrackTarget
 *     （ArmMoveToJoint = 关节空间点到点，2026-07-31 新增；只动臂 J1-3，云台保持）
 *   - 4 个 Service：ArmStop / ArmEnable / ArmHoming / ArmResetError
 *   - 10Hz ArmStatus 广播 + 高层状态机 IDLE→MOVING→REACHED/STOPPED/ERROR
 *
 * 线程模型：MultiThreadedExecutor + Reentrant 回调组；Action 执行在 handle_accepted 派生
 *           的独立线程里阻塞跑（IK / 等待到位），服务回调可在执行线程同步等 future。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <robot_arm_interfaces/action/arm_move_to_pose.hpp>
#include <robot_arm_interfaces/action/arm_move_to_joint.hpp>
#include <robot_arm_interfaces/action/arm_trajectory_shot.hpp>
#include <robot_arm_interfaces/action/arm_track_target.hpp>
#include <robot_arm_interfaces/msg/arm_status.hpp>
#include <robot_arm_interfaces/msg/arm_pose.hpp>
#include <robot_arm_interfaces/srv/arm_stop.hpp>
#include <robot_arm_interfaces/srv/arm_enable.hpp>
#include <robot_arm_interfaces/srv/arm_homing.hpp>
#include <robot_arm_interfaces/srv/arm_reset_error.hpp>

#include "robot_arm_node/state/status_aggregator.hpp"
#include "robot_arm_node/commander/motion_executor.hpp"
#include "robot_arm_node/commander/execution_monitor.hpp"
#include "robot_arm_node/commander/move_to_pose_server.hpp"
#include "robot_arm_node/commander/move_to_joint_server.hpp"
#include "robot_arm_node/commander/trajectory_shot_server.hpp"
#include "robot_arm_node/commander/track_target_server.hpp"

namespace robot_arm_node::commander
{

// 高层状态机（对应 Python CommanderState）
enum class CommanderState : uint8_t { IDLE = 0, MOVING = 1, REACHED = 2, STOPPED = 3, ERROR = 4 };

class ArmCommanderNode : public rclcpp::Node
{
public:
  ArmCommanderNode();

  // 供执行线程 / 运动引擎判断是否应立即中止当前 goal 并停止下发轨迹
  bool is_stopped() const;

private:
  using ArmStatus     = robot_arm_interfaces::msg::ArmStatus;
  using ArmPose       = robot_arm_interfaces::msg::ArmPose;
  using MoveToPose    = robot_arm_interfaces::action::ArmMoveToPose;
  using MoveToJoint   = robot_arm_interfaces::action::ArmMoveToJoint;
  using TrajectoryShot= robot_arm_interfaces::action::ArmTrajectoryShot;
  using TrackTarget   = robot_arm_interfaces::action::ArmTrackTarget;
  using Trigger       = std_srvs::srv::Trigger;

  // ── 状态机 ──────────────────────────────────────────────────────────────────
  CommanderState state() const;
  void transition(CommanderState new_state);
  bool is_idle() const;
  static const char * state_name(CommanderState s);

  // ── Action 执行线程体（handle_goal/cancel/accepted 在构造体内以 lambda 绑定）────
  void mtp_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> gh);
  void mtj_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoint>> gh);
  void em_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryShot>> gh);
  void track_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> gh);

  // ── Service 回调 ─────────────────────────────────────────────────────────────
  void on_arm_stop(const std::shared_ptr<robot_arm_interfaces::srv::ArmStop::Request>,
                   std::shared_ptr<robot_arm_interfaces::srv::ArmStop::Response>);
  void on_arm_enable(const std::shared_ptr<robot_arm_interfaces::srv::ArmEnable::Request>,
                     std::shared_ptr<robot_arm_interfaces::srv::ArmEnable::Response>);
  void on_arm_homing(const std::shared_ptr<robot_arm_interfaces::srv::ArmHoming::Request>,
                     std::shared_ptr<robot_arm_interfaces::srv::ArmHoming::Response>);
  void on_arm_reset_error(const std::shared_ptr<robot_arm_interfaces::srv::ArmResetError::Request>,
                          std::shared_ptr<robot_arm_interfaces::srv::ArmResetError::Response>);

  // 同步调用驱动层 Trigger 服务（0.5s 等待可用，10s 调用超时）；不可用视为仿真跳过
  std::pair<bool, std::string> call_driver_trigger(
      const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & srv_path);

  // ── 其它 ─────────────────────────────────────────────────────────────────────
  void publish_status();
  ArmPose get_observe_pose();

  // ── 状态 ─────────────────────────────────────────────────────────────────────
  mutable std::mutex state_mtx_;
  CommanderState     state_{CommanderState::IDLE};
  std::atomic<uint32_t> cmd_counter_{0};
  bool               enabled_{false};

  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // ── 子系统（构造体内 make_unique，顺序：status → motion → monitor → servers）──
  std::unique_ptr<state::StatusAggregator> status_;
  std::unique_ptr<MotionExecutor>          motion_;
  std::unique_ptr<ExecutionMonitor>        monitor_;
  std::unique_ptr<MoveToPoseServer>        mtp_srv_;
  std::unique_ptr<MoveToJointServer>       mtj_srv_;
  std::unique_ptr<TrajectoryShotServer>    em_srv_;
  std::unique_ptr<TrackTargetServer>       track_srv_;

  // ── Action Server ────────────────────────────────────────────────────────────
  rclcpp_action::Server<MoveToPose>::SharedPtr     mtp_server_;
  rclcpp_action::Server<MoveToJoint>::SharedPtr    mtj_server_;
  rclcpp_action::Server<TrajectoryShot>::SharedPtr em_server_;
  rclcpp_action::Server<TrackTarget>::SharedPtr    track_server_;

  // ── Publisher / Timer / Service / 驱动层客户端 ───────────────────────────────
  rclcpp::Publisher<ArmStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr            status_timer_;
  rclcpp::Service<robot_arm_interfaces::srv::ArmStop>::SharedPtr       stop_srv_;
  rclcpp::Service<robot_arm_interfaces::srv::ArmEnable>::SharedPtr     enable_srv_;
  rclcpp::Service<robot_arm_interfaces::srv::ArmHoming>::SharedPtr     homing_srv_;
  rclcpp::Service<robot_arm_interfaces::srv::ArmResetError>::SharedPtr reset_error_srv_;
  rclcpp::Client<Trigger>::SharedPtr drv_enable_cli_, drv_disable_cli_, drv_recover_cli_;
};

}  // namespace robot_arm_node::commander
