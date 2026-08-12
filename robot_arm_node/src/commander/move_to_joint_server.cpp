/**
 * @file move_to_joint_server.cpp
 * @brief MoveToJointServer 实现 —— 校验（个数/限位/自碰撞）→ 两点轨迹 → 等待到位
 *
 * @version 1.0
 * @date 2026-07-31
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/move_to_joint_server.hpp"

#include <chrono>
#include <cmath>

#include <robot_arm_interfaces/msg/arm_status.hpp>

#include "robot_arm_node/commander/motion_policy.hpp"
#include "robot_arm_node/motion/constants.hpp"
#include "robot_arm_node/tuning.hpp"

namespace robot_arm_node::commander
{

using ArmStatus = robot_arm_interfaces::msg::ArmStatus;
using namespace std::chrono_literals;

namespace
{
// 超时余量/下限、反馈频率、自碰撞检查等待均来自 tuning::params()（arm_params.yaml）
constexpr int VALIDITY_WARN_THROTTLE_MS = 5000;   // 纯日志节流，不值得开成参数
}  // namespace

MoveToJointServer::MoveToJointServer(rclcpp::Node & node, MotionExecutor & motion,
                                     state::StatusAggregator & status, ExecutionMonitor & monitor)
: node_(node), logger_(node.get_logger()), motion_(motion), status_(status), monitor_(monitor),
  limits_(node)
{
  validity_cli_ = node.create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
}

std::optional<std::vector<double>> MoveToJointServer::resolve_target(
    const Action::Goal & goal, const std::vector<double> & current_arm) const
{
  if (goal.target_joints.size() != motion::ARM_JOINT_COUNT) {
    RCLCPP_ERROR(logger_, "target_joints 必须 %zu 个（Joint1-3），实际 %zu 个",
                 motion::ARM_JOINT_COUNT, goal.target_joints.size());
    return std::nullopt;
  }

  std::vector<double> target(goal.target_joints.begin(), goal.target_joints.end());
  if (goal.relative) {
    for (size_t i = 0; i < target.size(); ++i) target[i] += current_arm[i];
  }
  return target;
}

bool MoveToJointServer::collision_free(const std::vector<double> & full_target)
{
  if (!validity_cli_->service_is_ready()) {
    // fail-open：move_group 未运行（无 MoveIt 场景）时不拦，节流告警
    RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), VALIDITY_WARN_THROTTLE_MS,
        "/check_state_validity 不可用（move_group 未运行？）— 关节目标未做自碰撞检查");
    return true;
  }

  auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  req->group_name = motion::PLANNING_GROUP;
  req->robot_state.joint_state.name.assign(motion::JOINT_NAMES.begin(), motion::JOINT_NAMES.end());
  req->robot_state.joint_state.position = full_target;
  req->robot_state.is_diff = false;

  auto future = validity_cli_->async_send_request(req);
  if (future.wait_for(std::chrono::duration<double>(tuning::params().validity_wait_sec)) !=
      std::future_status::ready)
  {
    validity_cli_->remove_pending_request(future);
    RCLCPP_WARN(logger_, "自碰撞检查超时（%.1fs）— 本次放行（fail-open）",
                tuning::params().validity_wait_sec);
    return true;
  }
  return future.get()->valid;
}

MoveToJointServer::Action::Result MoveToJointServer::execute(const std::shared_ptr<GoalHandle> & gh)
{
  const auto goal = gh->get_goal();
  Action::Result r;

  // 当前 6 轴回读（J4-6 用于原样保持），以及臂 J1-3 切片
  const std::vector<double> current_full = motion_.get_current_joints();
  const std::vector<double> current_arm(current_full.begin(),
                                        current_full.begin() + motion::ARM_JOINT_COUNT);
  auto fill_result_joints = [this]() {
    auto cur = motion_.get_current_joints();
    cur.resize(motion::ARM_JOINT_COUNT);
    return cur;
  };

  // ── 闸①：个数校验 ───────────────────────────────────────────────────────────
  auto target_opt = resolve_target(*goal, current_arm);
  if (!target_opt) {
    r.success = false; r.exit_reason = "invalid_goal";
    r.error_code = ArmStatus::ERR_NONE; r.actual_joints = current_arm;
    return r;
  }
  const std::vector<double> target_arm = *target_opt;

  // ── 闸②：关节限位（URDF 单一来源，未拿到 URDF 则 fail-open）──────────────────
  const std::vector<std::string> arm_names(motion::JOINT_NAMES.begin(),
                                           motion::JOINT_NAMES.begin() + motion::ARM_JOINT_COUNT);
  if (!limits_.ready()) {
    RCLCPP_WARN(logger_, "关节限位尚未从 /robot_description 解析到 — 本次不做限位校验");
  }
  std::string offender; double bad_val = 0.0; motion::JointLimit bad_lim{};
  if (!limits_.within_limits(arm_names, target_arm, &offender, &bad_val, &bad_lim)) {
    RCLCPP_ERROR(logger_, "目标超限位：%s=%.4f 不在 [%.4f, %.4f]（不下发任何指令）",
                 offender.c_str(), bad_val, bad_lim.lower, bad_lim.upper);
    r.success = false; r.exit_reason = "out_of_range";
    r.error_code = ArmStatus::ERR_LIMIT; r.actual_joints = current_arm;
    return r;
  }

  // 下发用的 6 轴目标：J1-3 = 目标，J4-6 = 当前回读（云台保持不动）
  std::vector<double> full_target = current_full;
  for (size_t i = 0; i < motion::ARM_JOINT_COUNT; ++i) full_target[i] = target_arm[i];

  // ── 闸③：自碰撞 ────────────────────────────────────────────────────────────
  if (!collision_free(full_target)) {
    RCLCPP_ERROR(logger_, "目标姿态自碰撞（/check_state_validity 判定 invalid）— 不下发");
    r.success = false; r.exit_reason = "collision";
    r.error_code = ArmStatus::ERR_LIMIT; r.actual_joints = current_arm;
    return r;
  }

  // ── 时长：goal 显式指定优先，否则按档位算 ──────────────────────────────────
  const double duration = goal->duration_sec > 0.0
      ? goal->duration_sec
      : joint_move_duration(current_arm, target_arm, goal->transition_speed);

  RCLCPP_INFO(logger_,
      "MoveToJoint 目标(%s): J1=%.4f J2=%.4f J3=%.4f rad | 时长 %.2fs（档位 %u）| 云台 J4-6 保持",
      goal->relative ? "相对" : "绝对",
      target_arm[0], target_arm[1], target_arm[2], duration, goal->transition_speed);

  motion_.go_to_joints(full_target, duration);

  // ── 等待到位（判据只看臂 J1-3）──────────────────────────────────────────────
  ExecutionMonitor::WaitParams p;
  p.arrived = [this, target_arm]() {
    auto cur = motion_.get_current_joints();
    cur.resize(motion::ARM_JOINT_COUNT);
    return is_at_joints(cur, target_arm);
  };
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  p.on_feedback = [this, gh, duration](double elapsed) {
    auto fb = std::make_shared<Action::Feedback>();
    const double ratio = std::min(elapsed / std::max(duration, 1e-6), 0.999);
    fb->progress_percent = static_cast<float>(ratio * 100.0);
    auto cur = motion_.get_current_joints();
    cur.resize(motion::ARM_JOINT_COUNT);
    fb->current_joints = cur;
    gh->publish_feedback(fb);
  };
  p.timeout_sec = std::max(duration + tuning::params().move_to_joint_timeout_margin_sec,
                           tuning::params().move_to_joint_timeout_floor_sec);
  p.feedback_hz = tuning::params().feedback_hz;
  p.label = "MoveToJoint ";
  const auto outcome = monitor_.wait_until(p);

  r.success     = is_reached(outcome);
  r.exit_reason = r.success ? "reached" :
                  (outcome == WaitOutcome::STOPPED   ? "stopped" :
                   outcome == WaitOutcome::CANCELLED ? "cancelled" : "timeout");
  r.error_code   = r.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  r.actual_joints = fill_result_joints();
  return r;
}

}  // namespace robot_arm_node::commander
