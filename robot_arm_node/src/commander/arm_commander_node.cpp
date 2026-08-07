/**
 * @file arm_commander_node.cpp
 * @brief ArmCommanderNode 实现 —— 状态机 + 3 Action Server + 4 Service + ArmStatus 广播
 *
 * 状态机 IDLE/MOVING/REACHED/STOPPED/ERROR 的转换、Action goal 接受/执行/终态判定、
 * 急停与清错、驱动层 Trigger 同步转发（仿真不可用则跳过）均在此。Action 执行体
 * （mtp/em/track_execute）在 handle_accepted 派生线程里跑，调用对应 server 的 execute。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/arm_commander_node.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "robot_arm_node/motion/constants.hpp"

namespace robot_arm_node::commander
{

using namespace std::chrono_literals;

namespace
{
// 话题 / 动作 / 服务名（对应 Python 常量）
const char * TOPIC_ARM_STATUS       = "/robot_arm/arm_status";
const char * ACTION_MOVE_TO_POSE    = "/robot_arm/move_to_pose";
const char * ACTION_MOVE_TO_JOINT   = "/robot_arm/move_to_joint";
const char * ACTION_TRAJECTORY_SHOT = "/robot_arm/trajectory_shot";
const char * ACTION_TRACK_TARGET    = "/robot_arm/track_target";
const char * TOPIC_FOLLOW_COMMAND   = "/robot_arm/follow_command";
const char * SERVICE_ARM_STOP       = "/robot_arm/stop";
const char * SERVICE_ARM_ENABLE     = "/robot_arm/enable";
const char * SERVICE_ARM_HOMING     = "/robot_arm/homing";
const char * SERVICE_ARM_RESET_ERROR= "/robot_arm/reset_error";
const char * ARM_NODE_ENABLE_SRV    = "/arm_node/enable";
const char * ARM_NODE_DISABLE_SRV   = "/arm_node/disable";
const char * ARM_NODE_RECOVER_SRV   = "/arm_node/recover";
// mode_manager_node 的速度总线急停闩锁（内部接口，非 Director 直调）
const char * VELOCITY_ESTOP_SRV     = "/robot_arm/velocity_estop";
const char * SWITCH_MODE_SRV        = "/robot_arm/switch_control_mode";

const std::vector<double> HOMING_JOINTS(6, 0.0);
constexpr double HOMING_DURATION = 4.0;
}  // namespace

ArmCommanderNode::ArmCommanderNode()
: rclcpp::Node("arm_commander")
{
  using rclcpp_action::GoalResponse;
  using rclcpp_action::CancelResponse;

  // ── 参数：OBSERVE 预定义位姿（可覆盖）─────────────────────────────────────────
  declare_parameter("pose_observe_x", 0.3);
  declare_parameter("pose_observe_y", 0.0);
  declare_parameter("pose_observe_z", 0.65);
  // 2026-07-29 云台换 V2：画面水平所需的 EEF roll 由 90° 变为 0°
  //（见 motion/geometry.hpp 的 EEF_LEVEL_ROLL 推导）。位置不变，只改 roll。
  declare_parameter("pose_observe_roll", 0.0);
  declare_parameter("pose_observe_pitch", 0.0);
  declare_parameter("pose_observe_yaw", 0.0);

  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // ── 子系统（顺序敏感）────────────────────────────────────────────────────────
  auto is_stopped_fn = [this]() { return this->is_stopped(); };
  status_  = std::make_unique<state::StatusAggregator>(*this);
  motion_  = std::make_unique<MotionExecutor>(*this, *status_, is_stopped_fn);
  monitor_ = std::make_unique<ExecutionMonitor>(get_logger(), is_stopped_fn,
                                                [this]() { motion_->stop(); });
  mtp_srv_   = std::make_unique<MoveToPoseServer>(*this, *motion_, *status_, *monitor_,
                                                  [this]() { return get_observe_pose(); });
  mtj_srv_   = std::make_unique<MoveToJointServer>(*this, *motion_, *status_, *monitor_);
  em_srv_    = std::make_unique<TrajectoryShotServer>(*this, *motion_, *status_, *monitor_, is_stopped_fn);
  track_srv_ = std::make_unique<TrackTargetServer>(*this, *status_, is_stopped_fn);
  // 速度流控制：topic 流式，不占状态机。与 action 的互斥由「控制模式」保证 ——
  // 速度流只在 JOINT_VELOCITY 模式下下发，而轨迹类动作执行前会切回 TRAJECTORY，
  // 一切模式速度流自己就停了（两者都发同一个 JTC 话题，靠模式串行化，不会打架）。
  vstream_srv_ = std::make_unique<VelocityStreamServer>(*this, *status_, is_stopped_fn);

  // ── Action Server（全部 ACCEPT_AND_EXECUTE，非空闲检查在执行线程内 abort，与 Python 一致）──
  const auto opts = rcl_action_server_get_default_options();

  mtp_server_ = rclcpp_action::create_server<MoveToPose>(
      this, ACTION_MOVE_TO_POSE,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) {
        return GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>>) {
        RCLCPP_INFO(get_logger(), "收到 MoveToPose 取消请求");
        motion_->stop();
        return CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> gh) {
        std::thread{[this, gh]() { mtp_execute(gh); }}.detach();
      },
      opts, cb_group_);

  mtj_server_ = rclcpp_action::create_server<MoveToJoint>(
      this, ACTION_MOVE_TO_JOINT,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToJoint::Goal>) {
        return GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoint>>) {
        RCLCPP_INFO(get_logger(), "收到 MoveToJoint 取消请求");
        motion_->stop();
        return CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoint>> gh) {
        std::thread{[this, gh]() { mtj_execute(gh); }}.detach();
      },
      opts, cb_group_);

  em_server_ = rclcpp_action::create_server<TrajectoryShot>(
      this, ACTION_TRAJECTORY_SHOT,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const TrajectoryShot::Goal>) {
        return GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryShot>>) {
        RCLCPP_INFO(get_logger(), "收到 TrajectoryShot 取消请求");
        motion_->stop();
        return CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryShot>> gh) {
        std::thread{[this, gh]() { em_execute(gh); }}.detach();
      },
      opts, cb_group_);

  track_server_ = rclcpp_action::create_server<TrackTarget>(
      this, ACTION_TRACK_TARGET,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const TrackTarget::Goal>) {
        return GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>>) {
        RCLCPP_INFO(get_logger(), "收到 TrackTarget 取消请求");
        track_srv_->cancel();
        return CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> gh) {
        std::thread{[this, gh]() { track_execute(gh); }}.detach();
      },
      opts, cb_group_);

  // ── ArmStatus 10Hz 广播 ──────────────────────────────────────────────────────
  status_pub_   = create_publisher<ArmStatus>(TOPIC_ARM_STATUS, 10);
  status_timer_ = create_wall_timer(100ms, [this]() { publish_status(); }, cb_group_);

  // ── 服务 ─────────────────────────────────────────────────────────────────────
  using std::placeholders::_1;
  using std::placeholders::_2;
  stop_srv_ = create_service<robot_arm_interfaces::srv::ArmStop>(
      SERVICE_ARM_STOP, std::bind(&ArmCommanderNode::on_arm_stop, this, _1, _2),
      rmw_qos_profile_services_default, cb_group_);
  enable_srv_ = create_service<robot_arm_interfaces::srv::ArmEnable>(
      SERVICE_ARM_ENABLE, std::bind(&ArmCommanderNode::on_arm_enable, this, _1, _2),
      rmw_qos_profile_services_default, cb_group_);
  homing_srv_ = create_service<robot_arm_interfaces::srv::ArmHoming>(
      SERVICE_ARM_HOMING, std::bind(&ArmCommanderNode::on_arm_homing, this, _1, _2),
      rmw_qos_profile_services_default, cb_group_);
  reset_error_srv_ = create_service<robot_arm_interfaces::srv::ArmResetError>(
      SERVICE_ARM_RESET_ERROR, std::bind(&ArmCommanderNode::on_arm_reset_error, this, _1, _2),
      rmw_qos_profile_services_default, cb_group_);

  // ── 驱动层 Trigger 客户端（实物；仿真下不可用则本地跳过）──────────────────────
  drv_enable_cli_  = create_client<Trigger>(ARM_NODE_ENABLE_SRV);
  drv_disable_cli_ = create_client<Trigger>(ARM_NODE_DISABLE_SRV);
  drv_recover_cli_ = create_client<Trigger>(ARM_NODE_RECOVER_SRV);
  vel_estop_cli_   = create_client<std_srvs::srv::SetBool>(VELOCITY_ESTOP_SRV);
  // 轨迹类动作在速度模式下会静默失效，执行前自动切回 TRAJECTORY（仍走 mode_manager 唯一入口）
  mode_switch_cli_ = create_client<SwitchControlMode>(SWITCH_MODE_SRV,
                                                      rmw_qos_profile_services_default, cb_group_);

  RCLCPP_INFO(get_logger(),
              "Arm Commander 已就绪  |  状态=%s  |  action: %s / %s / %s / %s  |  速度流: %s",
              state_name(state()), ACTION_MOVE_TO_POSE, ACTION_MOVE_TO_JOINT,
              ACTION_TRAJECTORY_SHOT, ACTION_TRACK_TARGET, TOPIC_FOLLOW_COMMAND);
}

// ── 状态机 ──────────────────────────────────────────────────────────────────────
const char * ArmCommanderNode::state_name(CommanderState s)
{
  switch (s) {
    case CommanderState::IDLE:    return "IDLE";
    case CommanderState::MOVING:  return "MOVING";
    case CommanderState::REACHED: return "REACHED";
    case CommanderState::STOPPED: return "STOPPED";
    default:                      return "ERROR";
  }
}

CommanderState ArmCommanderNode::state() const
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  return state_;
}

void ArmCommanderNode::transition(CommanderState new_state)
{
  CommanderState old;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    old = state_;
    state_ = new_state;
  }
  status_->set_moving(new_state == CommanderState::MOVING);
  RCLCPP_INFO(get_logger(), "状态: %s → %s", state_name(old), state_name(new_state));
}

bool ArmCommanderNode::is_idle() const
{
  const auto s = state();
  return s == CommanderState::IDLE || s == CommanderState::REACHED;
}

bool ArmCommanderNode::is_stopped() const
{
  return state() == CommanderState::STOPPED;
}

// ── ArmMoveToPose 执行线程 ──────────────────────────────────────────────────────
void ArmCommanderNode::mtp_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> gh)
{
  if (!is_idle()) {
    RCLCPP_WARN(get_logger(), "拒绝 goal: 当前状态=%s，非空闲", state_name(state()));
    gh->abort(std::make_shared<MoveToPose::Result>());
    return;
  }
  std::string why;
  if (!ensure_trajectory_mode(&why)) {
    auto r = std::make_shared<MoveToPose::Result>();
    r->success = false;
    r->exit_reason = "error";
    r->error_code = ArmStatus::ERR_DRIVER;
    gh->abort(r);
    return;
  }
  RCLCPP_INFO(get_logger(), "MoveToPose goal 已接受，开始执行");
  transition(CommanderState::MOVING);
  status_->set_at_pose_start(false);
  const uint32_t cmd_id = ++cmd_counter_;
  status_->set_command_state(cmd_id, ArmStatus::RESULT_EXECUTING);
  status_->set_pose_state(gh->get_goal()->target_pose_state);

  try {
    auto result = std::make_shared<MoveToPose::Result>(mtp_srv_->execute(gh));
    if (is_stopped()) {
      RCLCPP_INFO(get_logger(), "MoveToPose 执行期间被急停，goal 终止，状态保持 STOPPED");
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
      return;
    }
    if (result->success) {
      transition(CommanderState::REACHED);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_SUCCEEDED);
      gh->succeed(result);
    } else if (result->exit_reason == "cancelled") {
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->canceled(result);
    } else if (result->exit_reason == "stopped") {
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
    } else if (result->exit_reason == "unreachable") {
      RCLCPP_WARN(get_logger(), "目标不可达（IK 无解），拒绝本次 goal，恢复空闲");
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
    } else {
      transition(CommanderState::ERROR);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
      status_->set_error(result->error_code);
      gh->abort(result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MoveToPose 执行异常: %s", e.what());
    gh->abort(std::make_shared<MoveToPose::Result>());
    transition(CommanderState::ERROR);
    status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
    status_->set_error(ArmStatus::ERR_DRIVER);
  }
}

// ── ArmMoveToJoint 执行线程 ─────────────────────────────────────────────────────
// 与 mtp_execute 同构，两点不同：
//   ① 不动 current_pose_state —— 关节空间点到点是示教/标定用途，不代表产品语义上的
//      「收纳/观察/拍摄」姿态；硬套一个会让 Director 误判。需要新语义时再加枚举。
//   ② 三类前置校验失败（out_of_range / collision / invalid_goal）视同「拒绝该 goal」：
//      回 IDLE 而不是进 ERROR（与 MoveToPose 的 unreachable 一致，因为没下发任何指令，
//      设备本身没故障）。
void ArmCommanderNode::mtj_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoint>> gh)
{
  if (!is_idle()) {
    RCLCPP_WARN(get_logger(), "拒绝 goal: 当前状态=%s，非空闲", state_name(state()));
    gh->abort(std::make_shared<MoveToJoint::Result>());
    return;
  }
  std::string why;
  if (!ensure_trajectory_mode(&why)) {
    auto r = std::make_shared<MoveToJoint::Result>();
    r->success = false;
    r->exit_reason = "error";
    r->error_code = ArmStatus::ERR_DRIVER;
    gh->abort(r);
    return;
  }
  RCLCPP_INFO(get_logger(), "MoveToJoint goal 已接受，开始执行");
  transition(CommanderState::MOVING);
  const uint32_t cmd_id = ++cmd_counter_;
  status_->set_command_state(cmd_id, ArmStatus::RESULT_EXECUTING);

  try {
    auto result = std::make_shared<MoveToJoint::Result>(mtj_srv_->execute(gh));
    if (is_stopped()) {
      RCLCPP_INFO(get_logger(), "MoveToJoint 执行期间被急停，goal 终止，状态保持 STOPPED");
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
      return;
    }
    if (result->success) {
      transition(CommanderState::REACHED);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_SUCCEEDED);
      gh->succeed(result);
    } else if (result->exit_reason == "cancelled") {
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->canceled(result);
    } else if (result->exit_reason == "stopped") {
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
    } else if (result->exit_reason == "out_of_range" ||
               result->exit_reason == "collision" ||
               result->exit_reason == "invalid_goal") {
      RCLCPP_WARN(get_logger(), "关节目标被拒（%s），未下发指令，恢复空闲",
                  result->exit_reason.c_str());
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
    } else {
      transition(CommanderState::ERROR);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
      status_->set_error(result->error_code);
      gh->abort(result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MoveToJoint 执行异常: %s", e.what());
    gh->abort(std::make_shared<MoveToJoint::Result>());
    transition(CommanderState::ERROR);
    status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
    status_->set_error(ArmStatus::ERR_DRIVER);
  }
}

// ── ArmTrajectoryShot 执行线程 ──────────────────────────────────────────────────
void ArmCommanderNode::em_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryShot>> gh)
{
  if (!is_idle()) {
    RCLCPP_WARN(get_logger(), "拒绝 goal: 当前状态=%s，非空闲", state_name(state()));
    gh->abort(std::make_shared<TrajectoryShot::Result>());
    return;
  }
  std::string why;
  if (!ensure_trajectory_mode(&why)) {
    auto r = std::make_shared<TrajectoryShot::Result>();
    r->success = false;
    r->exit_reason = "error";
    r->error_code = ArmStatus::ERR_DRIVER;
    gh->abort(r);
    return;
  }
  transition(CommanderState::MOVING);
  status_->set_at_pose_start(false);
  status_->set_camera_ready(false);
  const uint32_t cmd_id = ++cmd_counter_;
  status_->set_command_state(cmd_id, ArmStatus::RESULT_EXECUTING);

  try {
    auto result = std::make_shared<TrajectoryShot::Result>(em_srv_->execute(gh));
    if (is_stopped()) {
      RCLCPP_INFO(get_logger(), "TrajectoryShot 执行期间被急停，goal 终止，状态保持 STOPPED");
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
      status_->set_camera_ready(false);
      return;
    }
    if (result->success) {
      transition(CommanderState::REACHED);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_SUCCEEDED);
      gh->succeed(result);
    } else if (result->exit_reason == "cancelled") {
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->canceled(result);
    } else if (result->exit_reason == "stopped") {
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
    } else if (result->exit_reason == "unreachable") {
      // 与 MoveToPose 一致：IK 无解是 goal 参数问题而非系统故障，
      // 拒绝本次 goal 恢复空闲，不进 ERROR（否则需要 reset_error 才能继续）
      RCLCPP_WARN(get_logger(), "目标不可达（IK 无解），拒绝本次 goal，恢复空闲");
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      gh->abort(result);
    } else {
      transition(CommanderState::ERROR);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
      status_->set_error(result->error_code);
      gh->abort(result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "TrajectoryShot 执行异常: %s", e.what());
    gh->abort(std::make_shared<TrajectoryShot::Result>());
    transition(CommanderState::ERROR);
    status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
    status_->set_error(ArmStatus::ERR_DRIVER);
  }
  status_->set_camera_ready(false);   // 运镜结束（任何原因），复位录制就绪信号
}

// ── ArmTrackTarget 执行线程（server 自行终结 goal，本层只做状态转换）────────────
void ArmCommanderNode::track_execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> gh)
{
  const auto goal = gh->get_goal();
  RCLCPP_INFO(get_logger(), "收到 TrackTarget goal: depth=%.2fm hold=%d",
              goal->desired_depth, goal->hold_on_converge);
  if (!is_idle()) {
    RCLCPP_WARN(get_logger(), "拒绝 goal: 当前状态=%s，非空闲", state_name(state()));
    gh->abort(std::make_shared<TrackTarget::Result>());
    return;
  }
  transition(CommanderState::MOVING);
  const uint32_t cmd_id = ++cmd_counter_;
  status_->set_command_state(cmd_id, ArmStatus::RESULT_EXECUTING);

  try {
    auto result = track_srv_->execute(gh);   // 内部已调 goal_handle 终态
    if (is_stopped()) {
      RCLCPP_INFO(get_logger(), "TrackTarget 执行期间被急停，状态保持 STOPPED");
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
      return;
    }
    if (result.exit_code == TrackTarget::Goal::EXIT_CANCELLED) {
      transition(CommanderState::IDLE);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
    } else if (result.exit_reason == "stopped") {
      status_->set_command_state(cmd_id, ArmStatus::RESULT_ABORTED);
    } else if (result.success) {
      transition(CommanderState::REACHED);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_SUCCEEDED);
    } else {
      transition(CommanderState::ERROR);
      status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
      status_->set_error(ArmStatus::ERR_TIMEOUT);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "TrackTarget 执行异常: %s", e.what());
    transition(CommanderState::ERROR);
    status_->set_command_state(cmd_id, ArmStatus::RESULT_FAILED);
    status_->set_error(ArmStatus::ERR_DRIVER);
  }
}

// ── ArmStop（急停）──────────────────────────────────────────────────────────────
void ArmCommanderNode::on_arm_stop(
    const std::shared_ptr<robot_arm_interfaces::srv::ArmStop::Request>,
    std::shared_ptr<robot_arm_interfaces::srv::ArmStop::Response> response)
{
  // 速度流不走状态机（topic 流式、不占 goal），所以急停要单独处理这一路：
  //   ① 停 Commander 自己的笛卡尔速度流（立刻喂 0）
  //   ② 闩住 mode_manager 的速度总线 —— Director 可能自己在直发关节速度指令，
  //      只停 ① 拦不住它；总线闩锁是速度指令的唯一必经之路。
  const bool was_streaming = vstream_srv_ && vstream_srv_->is_streaming();
  if (vstream_srv_) vstream_srv_->emergency_stop();
  latch_velocity_estop(true);

  if (state() == CommanderState::MOVING) {
    motion_->stop();
    transition(CommanderState::STOPPED);
    response->success = true;
    response->message = "已急停，状态 MOVING → STOPPED";
  } else if (was_streaming) {
    transition(CommanderState::STOPPED);
    response->success = true;
    response->message = "已急停速度流，状态 → STOPPED（需 ArmResetError 复位）";
  } else {
    // 没有 goal 在跑也没在发速度流：状态机不动，但速度总线仍已闩锁 ——
    // Director 可能正在直发关节速度，那条路不经状态机
    response->success = true;
    response->message = std::string("当前状态=") + state_name(state())
                      + "，无运动可停；速度总线已闩锁（ArmResetError 解除）";
  }
  RCLCPP_INFO(get_logger(), "ArmStop: %s", response->message.c_str());
}

// ── 轨迹类动作的模式前置条件 ──────────────────────────────────────────────────────
//   MoveToPose / MoveToJoint / TrajectoryShot 全都靠 JTC 执行；速度模式下 JTC 被停用，
//   轨迹发出去也不会动（表现为动作静默超时）。这类动作的语义本身就要求轨迹模式，
//   所以这里**自动切回 TRAJECTORY** 而不是让上层先手动切一次 —— 与「速度流不自动切模式」
//   的约定不冲突：速度流是持续控制权，动作是一次性位置指令。
//   切换仍然只经 mode_manager（唯一入口），失败则放弃执行并让调用方拿到明确原因。
bool ArmCommanderNode::ensure_trajectory_mode(std::string * why)
{
  if (status_->active_control_mode() == ControlMode::TRAJECTORY) return true;

  if (!mode_switch_cli_->wait_for_service(1s)) {
    if (why) *why = "mode_manager 不可用，无法从速度模式切回轨迹模式";
    RCLCPP_ERROR(get_logger(), "%s", why ? why->c_str() : "");
    return false;
  }
  auto req = std::make_shared<SwitchControlMode::Request>();
  req->target_mode = ControlMode::TRAJECTORY;
  auto future = mode_switch_cli_->async_send_request(req);
  if (future.wait_for(10s) != std::future_status::ready) {
    mode_switch_cli_->remove_pending_request(future);
    if (why) *why = "切回 TRAJECTORY 超时";
    RCLCPP_ERROR(get_logger(), "%s", why ? why->c_str() : "");
    return false;
  }
  auto resp = future.get();
  if (!resp || !resp->success) {
    if (why) *why = std::string("切回 TRAJECTORY 失败：") + (resp ? resp->message : "无应答");
    RCLCPP_ERROR(get_logger(), "%s", why ? why->c_str() : "");
    return false;
  }
  RCLCPP_INFO(get_logger(), "动作执行前已自动切回 TRAJECTORY 模式");
  return true;
}

// ── 速度总线急停闩锁（Commander → ModeManager）────────────────────────────────────
//   异步发：急停必须立刻返回，不能卡在等 mode_manager 应答上。
//   mode_manager 不在（如 MuJoCo 后端）时静默跳过 —— 那种后端也没有速度控制器。
void ArmCommanderNode::latch_velocity_estop(bool engage)
{
  if (!vel_estop_cli_->service_is_ready()) {
    RCLCPP_DEBUG(get_logger(), "%s 不可用，跳过速度总线闩锁", VELOCITY_ESTOP_SRV);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = engage;
  vel_estop_cli_->async_send_request(req);
}

// ── 驱动层 Trigger 同步调用 ──────────────────────────────────────────────────────
std::pair<bool, std::string> ArmCommanderNode::call_driver_trigger(
    const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & srv_path)
{
  if (!client->wait_for_service(500ms)) {
    RCLCPP_INFO(get_logger(), "%s 不可用，仿真模式跳过", srv_path.c_str());
    return {true, "仿真模式，跳过"};
  }
  auto future = client->async_send_request(std::make_shared<Trigger::Request>());
  if (future.wait_for(10s) != std::future_status::ready) {
    client->remove_pending_request(future);
    return {false, srv_path + " 调用超时"};
  }
  auto resp = future.get();
  if (!resp) return {false, srv_path + " 无应答"};
  return {resp->success, resp->message};
}

// ── ArmEnable（伺服使能）────────────────────────────────────────────────────────
void ArmCommanderNode::on_arm_enable(
    const std::shared_ptr<robot_arm_interfaces::srv::ArmEnable::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::ArmEnable::Response> response)
{
  auto [ok, msg] = request->enable
      ? call_driver_trigger(drv_enable_cli_, ARM_NODE_ENABLE_SRV)
      : call_driver_trigger(drv_disable_cli_, ARM_NODE_DISABLE_SRV);
  if (ok) enabled_ = request->enable;
  response->success = ok;
  response->message = std::string("伺服") + (request->enable ? "使能" : "下电") + ": " + msg;
  RCLCPP_INFO(get_logger(), "ArmEnable: %s", response->message.c_str());
}

// ── ArmHoming（回零）────────────────────────────────────────────────────────────
void ArmCommanderNode::on_arm_homing(
    const std::shared_ptr<robot_arm_interfaces::srv::ArmHoming::Request>,
    std::shared_ptr<robot_arm_interfaces::srv::ArmHoming::Response> response)
{
  if (!is_idle()) {
    response->success = false;
    response->message = std::string("拒绝回零：当前状态=") + state_name(state()) + "，非空闲";
    RCLCPP_WARN(get_logger(), "ArmHoming: %s", response->message.c_str());
    return;
  }
  transition(CommanderState::MOVING);
  motion_->go_to_joints(HOMING_JOINTS, HOMING_DURATION);

  // 回零无 goal_handle / 无 Feedback：仅等关节回零或急停
  // 只判臂 J1-3：云台 J4-6 转发回读在云台未上电时不收敛，不阻塞回零
  ExecutionMonitor::WaitParams p;
  p.arrived = [this]() {
    const auto cur = motion_->get_current_joints();
    double m = 0.0;
    for (size_t i = 0; i < motion::ARM_JOINT_COUNT; ++i)
      m = std::max(m, std::fabs(cur[i]));
    return m < 0.05;
  };
  p.timeout_sec = HOMING_DURATION + 2.0;
  p.feedback_hz = 0.0;
  p.poll_dt = 0.05;
  p.label = "回零 ";
  monitor_->wait_until(p);

  if (is_stopped()) {
    response->success = false;
    response->message = "回零途中被急停，已中止";
    RCLCPP_WARN(get_logger(), "ArmHoming: %s", response->message.c_str());
    return;
  }
  transition(CommanderState::IDLE);
  status_->set_pose_state(ArmStatus::POSE_STATE_STOWED);
  response->success = true;
  response->message = "回零完成";
  RCLCPP_INFO(get_logger(), "ArmHoming: %s", response->message.c_str());
}

// ── ArmResetError（清除故障）────────────────────────────────────────────────────
void ArmCommanderNode::on_arm_reset_error(
    const std::shared_ptr<robot_arm_interfaces::srv::ArmResetError::Request>,
    std::shared_ptr<robot_arm_interfaces::srv::ArmResetError::Response> response)
{
  auto [drv_ok, drv_msg] = call_driver_trigger(drv_recover_cli_, ARM_NODE_RECOVER_SRV);
  latch_velocity_estop(false);   // 解除速度总线急停闩锁（与 ArmStop 成对）

  const auto current = state();
  if (current == CommanderState::ERROR || current == CommanderState::STOPPED) {
    const uint8_t cleared = status_->get_error_code();
    transition(CommanderState::IDLE);
    status_->clear_error();
    status_->set_command_state(0, ArmStatus::RESULT_NONE);
    response->success = drv_ok;
    response->cleared_error_code = cleared;
    response->message = std::string("已复位 ") + state_name(current) + "→IDLE，驱动层: " + drv_msg;
  } else {
    response->success = drv_ok;
    response->cleared_error_code = 0;
    response->message = std::string("状态=") + state_name(current) + "，驱动层recover: " + drv_msg;
  }
  RCLCPP_INFO(get_logger(), "ArmResetError: %s", response->message.c_str());
}

// ── ArmStatus 周期发布 ──────────────────────────────────────────────────────────
void ArmCommanderNode::publish_status()
{
  auto msg = status_->build_status_message();
  msg.header.stamp = get_clock()->now();
  status_pub_->publish(msg);
}

// ── OBSERVE 预定义位姿（从 ROS param 读）────────────────────────────────────────
ArmCommanderNode::ArmPose ArmCommanderNode::get_observe_pose()
{
  ArmPose p;
  p.x     = static_cast<float>(get_parameter("pose_observe_x").as_double());
  p.y     = static_cast<float>(get_parameter("pose_observe_y").as_double());
  p.z     = static_cast<float>(get_parameter("pose_observe_z").as_double());
  p.roll  = static_cast<float>(get_parameter("pose_observe_roll").as_double());
  p.pitch = static_cast<float>(get_parameter("pose_observe_pitch").as_double());
  p.yaw   = static_cast<float>(get_parameter("pose_observe_yaw").as_double());
  return p;
}

}  // namespace robot_arm_node::commander
