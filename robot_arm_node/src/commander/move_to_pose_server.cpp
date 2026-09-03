/**
 * @file move_to_pose_server.cpp
 * @brief MoveToPoseServer 实现 —— STOWED 关节回零 / OBSERVE·SHOOTING Cartesian
 *
 * execute 按 target_pose_state 分派：STOWED→go_to_joints + 关节到位等待；OBSERVE/SHOOTING
 * →resolve_target→plan_and_execute→wait_arrival（进度分段支持 return_to_start）。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/move_to_pose_server.hpp"

#include <cmath>

#include <robot_arm_interfaces/msg/arm_status.hpp>

#include "robot_arm_node/commander/motion_policy.hpp"
#include "robot_arm_node/motion/constants.hpp"
#include "robot_arm_node/tuning.hpp"

namespace robot_arm_node::commander
{

using ArmStatus = robot_arm_interfaces::msg::ArmStatus;

// 超时 / 反馈频率 / 收纳位与时长均来自 tuning::params()（arm_params.yaml），
// 原先是本文件匿名 namespace 里的 constexpr —— 实机想改收纳位得重编。

/**
 * @brief 构造位姿运动 Action 服务端。
 *
 * @param node 宿主节点，用于取日志器。
 * @param motion 运动执行器，负责 IK 与轨迹下发。
 * @param status 状态聚合器，提供当前末端位姿。
 * @param monitor 执行监视器，负责等待到位与反馈上报。
 * @param observe_pose 观察位取值回调（由 commander 注入，可随参数热更新）。
 */
MoveToPoseServer::MoveToPoseServer(rclcpp::Node & node, MotionExecutor & motion,
                                   state::StatusAggregator & status, ExecutionMonitor & monitor,
                                   std::function<ArmPose()> observe_pose)
: node_(node), logger_(node.get_logger()), motion_(motion), status_(status),
  monitor_(monitor), observe_pose_(std::move(observe_pose))
{
}

/**
 * @brief 执行 MoveToPose 动作，按 target_pose_state 分派到三条路径。
 *
 * STOWED 走关节空间直驱（不经 IK，也不支持 return_to_start）；
 * OBSERVE / SHOOTING 走笛卡尔：解析目标 → IK + 下发 → 等待到位。
 * return_to_start 时进度分两段上报（去程 0→50%，回程 50→100%），
 * 出发位姿在动作开始时就快照下来，避免回程时已被中途运动污染。
 *
 * @param gh Action 目标句柄，用于取 goal、发反馈、查取消。
 * @return Action 结果（success / exit_reason / error_code / actual_pose）。
 */
MoveToPoseServer::Action::Result MoveToPoseServer::execute(const std::shared_ptr<GoalHandle> & gh)
{
  const auto goal  = gh->get_goal();
  const Speed & speed = speed_profile(goal->transition_speed);

  // STOWED：关节空间回零（不走 IK，不支持 return_to_start）
  if (goal->target_pose_state == Action::Goal::POSE_STATE_STOWED) {
    return execute_stowed(gh);
  }

  // OBSERVE / SHOOTING：Cartesian 目标 → IK → JointTrajectory
  const ArmPose start_pose = status_.pose();   // 出发位姿（用于 return_to_start）

  auto target_opt = resolve_target(*goal);
  if (!target_opt) {
    Action::Result r;
    r.success = false; r.exit_reason = "unreachable"; r.error_code = ArmStatus::ERR_LIMIT;
    return r;
  }
  const ArmPose target = *target_opt;
  RCLCPP_INFO(logger_, "MoveToPose 目标: x=%.3f y=%.3f z=%.3f R=%.1f P=%.1f Y=%.1f",
              target.x, target.y, target.z, target.roll, target.pitch, target.yaw);

  auto exec_result = motion_.plan_and_execute(target, speed);
  if (!exec_result.success) {
    Action::Result r;
    r.success = false; r.exit_reason = exec_result.exit_reason;
    r.error_code = ArmStatus::ERR_DRIVER; r.actual_pose = status_.pose();
    return r;
  }

  // 等待到位（去程：0→50% if return_to_start 否则 0→100%）
  auto result = wait_arrival(gh, target, exec_result.target_joints, speed,
                             0.0, goal->return_to_start ? 50.0 : 100.0);
  if (!result.success || !goal->return_to_start) return result;

  // return_to_start：原路返回出发位姿
  RCLCPP_INFO(logger_, "return_to_start: 返回出发位姿");
  auto exec_back = motion_.plan_and_execute(start_pose, speed);
  if (!exec_back.success) {
    result.success = false; result.exit_reason = "error"; result.error_code = ArmStatus::ERR_DRIVER;
    return result;
  }
  return wait_arrival(gh, start_pose, exec_back.target_joints, speed, 50.0, 100.0);
}

/**
 * @brief 执行 STOWED 收纳动作：全关节回零，走关节空间不经 IK。
 *
 * 收纳位与时长取自 tuning::params()（arm_params.yaml），实机可直接改参数而
 * 不必重编。进度反馈基于「关节接近度」而非经过时间比例 —— 收纳是固定目标，
 * 用剩余误差算进度比用时长更贴近实际。到位判据只认臂 J1-3。
 *
 * @param gh Action 目标句柄。
 * @return Action 结果。
 */
MoveToPoseServer::Action::Result MoveToPoseServer::execute_stowed(const std::shared_ptr<GoalHandle> & gh)
{
  RCLCPP_INFO(logger_, "STOWED: 全关节回零");
  const auto & tp = tuning::params();
  motion_.go_to_joints(tp.stowed_joints, tp.stowed_duration_sec);
  // 到位判据只认臂 J1-3（云台回读不收敛不该拖累成败，见 motion_policy.hpp）
  const std::vector<double> STOWED_ARM_TARGET(tp.stowed_joints.begin(),
                                              tp.stowed_joints.begin() + motion::ARM_JOINT_COUNT);

  // 到位/进度只判臂 J1-3（STOWED_ARM_TARGET）：J4-6 云台命令仍随轨迹下发，
  // 但其转发回读在云台未上电时永不收敛，不应阻塞机械臂回收
  ExecutionMonitor::WaitParams p;
  p.arrived = [this, STOWED_ARM_TARGET]() {
    auto cur = motion_.get_current_joints();
    cur.resize(STOWED_ARM_TARGET.size());
    return is_at_joints(cur, STOWED_ARM_TARGET);
  };
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  p.on_feedback = [this, gh, STOWED_ARM_TARGET](double /*elapsed*/) {
    // 反馈基于关节接近度（非 elapsed 比例）
    const auto current = motion_.get_current_joints();
    double max_err = 0.0;
    for (size_t i = 0; i < STOWED_ARM_TARGET.size(); ++i)
      max_err = std::max(max_err, std::fabs(current[i] - STOWED_ARM_TARGET[i]));
    const double progress = std::max(0.0, 100.0 - max_err / 0.1 * 100.0);
    auto fb = std::make_shared<Action::Feedback>();
    fb->progress_percent = static_cast<float>(std::min(progress, 99.9));
    fb->current_pose     = status_.pose();
    gh->publish_feedback(fb);
  };
  p.timeout_sec = tuning::params().move_to_pose_timeout_sec;
  p.feedback_hz = tuning::params().feedback_hz;
  p.label = "STOWED ";
  const auto outcome = monitor_.wait_until(p);

  Action::Result r;
  r.success     = is_reached(outcome);
  r.exit_reason = r.success ? "reached" :
                  (outcome == WaitOutcome::STOPPED ? "stopped" :
                   outcome == WaitOutcome::CANCELLED ? "cancelled" : "timeout");
  r.error_code  = r.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  r.actual_pose = status_.pose();
  return r;
}

/**
 * @brief 等待笛卡尔动作到位，并按给定区间上报进度。
 *
 * 到位判据优先用「臂 J1-3 达到本段轨迹终点关节解」：末端 gimbal_tool0 在云台
 * J4-6 之后，末端位姿 = f(J1..J6)，云台回读不收敛（未上电、跨机 DDS 不通、
 * GCU 自稳环让 IMU 角与关节指令不一致）会让笛卡尔判据永不满足，导致臂早就
 * 到位了动作还走到 timeout。拿不到关节解时才退化回笛卡尔判据并告警。
 *
 * @param gh Action 目标句柄。
 * @param target 目标位姿（用于估算时长与退化判据）。
 * @param target_joints 本段轨迹终点关节解，长度 ≥ 3 时作为到位判据。
 * @param speed 速度档位，用于按距离估算总时长（进度分母）。
 * @param p_lo 本段进度下界（%）。
 * @param p_hi 本段进度上界（%）。
 * @return Action 结果。
 */
MoveToPoseServer::Action::Result MoveToPoseServer::wait_arrival(
    const std::shared_ptr<GoalHandle> & gh, const ArmPose & target,
    const std::vector<double> & target_joints,
    const Speed & speed, double p_lo, double p_hi)
{
  // 按直线距离与档位线速度估算总时长，仅用作进度分母（不参与实际规划）
  const ArmPose cur = status_.pose();
  const double dist = std::sqrt(std::pow(target.x - cur.x, 2) +
                                std::pow(target.y - cur.y, 2) +
                                std::pow(target.z - cur.z, 2));
  const double total_dur = std::max(dist / std::max(speed.v_pos, 1e-6), 0.5);

  ExecutionMonitor::WaitParams p;
  // 到位判据 = 臂 J1-3 达到本段轨迹的终点关节解。云台 J4-6 照常跟着轨迹动，
  // 但不进判据：末端 gimbal_tool0 在云台之后，云台回读不收敛会让笛卡尔判据永不满足。
  // 退化保护：拿不到关节解（理论上只在 plan_and_execute 失败时）→ 回落到笛卡尔判据。
  if (target_joints.size() >= motion::ARM_JOINT_COUNT) {
    p.arrived = [this, target_joints]() {
      return is_at_joints_prefix(motion_.get_current_joints(), target_joints,
                                 motion::ARM_JOINT_COUNT);
    };
  } else {
    RCLCPP_WARN(logger_, "无终点关节解，退化为笛卡尔到位判据（云台未到位可能导致超时）");
    p.arrived = [this, target]() { return is_at_pose(status_.pose(), target); };
  }
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  p.on_feedback = [this, gh, p_lo, p_hi, total_dur](double elapsed) {
    const double ratio = std::min(elapsed / std::max(total_dur, 1e-6), 0.999);
    auto fb = std::make_shared<Action::Feedback>();
    fb->progress_percent = static_cast<float>(p_lo + ratio * (p_hi - p_lo));
    fb->current_pose     = status_.pose();
    gh->publish_feedback(fb);
  };
  p.timeout_sec = tuning::params().move_to_pose_timeout_sec;
  p.feedback_hz = tuning::params().feedback_hz;
  p.label = "MoveToPose ";
  const auto outcome = monitor_.wait_until(p);

  Action::Result r;
  r.success     = is_reached(outcome);
  r.exit_reason = r.success ? "reached" :
                  (outcome == WaitOutcome::STOPPED ? "stopped" :
                   outcome == WaitOutcome::CANCELLED ? "cancelled" : "timeout");
  r.error_code  = r.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  r.actual_pose = status_.pose();
  return r;
}

/**
 * @brief 解析目标位姿：OBSERVE 取观察位回调，SHOOTING 用 goal 里的绝对位姿。
 *
 * @param goal Action 目标。
 * @return 目标位姿；target_pose_state 非法时返回 std::nullopt。
 */
std::optional<ArmPose> MoveToPoseServer::resolve_target(const Action::Goal & goal)
{
  if (goal.target_pose_state == Action::Goal::POSE_STATE_OBSERVE) {
    return observe_pose_();
  }
  if (goal.target_pose_state == Action::Goal::POSE_STATE_SHOOTING) {
    return goal.target_pose;   // 绝对位姿，直接使用
  }
  RCLCPP_ERROR(logger_, "非法 target_pose_state: %d", goal.target_pose_state);
  return std::nullopt;
}

}  // namespace robot_arm_node::commander
