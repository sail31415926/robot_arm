/**
 * @file teach_node.cpp
 * @brief TeachNode 实现
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/teach_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>

#include "robot_arm_teach/msg/motion_type.hpp"

namespace robot_arm_teach
{

using namespace std::chrono_literals;
using MotionTypeMsg = robot_arm_teach::msg::MotionType;

namespace
{
// 日志节流（ms）。纯降噪，不值得开成参数。
constexpr int WARN_THROTTLE_MS = 5000;

std::string default_storage_directory()
{
  // 与 ROS 自身的日志/参数落盘位置同源，跨用户/跨机器都成立；HOME 缺失时退回当前目录。
  const char * home = std::getenv("HOME");
  if (home && *home) return std::string(home) + "/.ros/robot_arm_teach";
  return "./robot_arm_teach_trajectories";
}
}  // namespace

// ══════════════════════════════════════════════════════════════════════════════
// 构造
// ══════════════════════════════════════════════════════════════════════════════
TeachNode::TeachNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("arm_teach", options),
  recorder_(),
  store_(""),
  planner_(),
  limits_(*this)
{
  declare_params();

  TeachRecorder::Config rc;
  rc.sample_rate_hz         = p_.sample_rate_hz;
  rc.compress_position_eps  = p_.compress_position_eps;
  rc.compress_max_gap_sec   = p_.compress_max_gap_sec;
  rc.max_duration_sec       = p_.max_duration_sec;
  rc.max_points             = static_cast<size_t>(std::max(1, p_.max_points));
  recorder_.set_config(rc);

  PlaybackPlanner::Config pc;
  pc.chunk_horizon_sec     = p_.chunk_horizon_sec;
  pc.approach_duration_sec = p_.approach_duration_sec;
  pc.hold_duration_sec     = p_.hold_duration_sec;
  planner_.set_config(pc);

  store_.set_directory(p_.storage_directory);
  if (auto r = store_.ensure_directory(); !r) {
    RCLCPP_WARN(get_logger(), "存储目录不可用：%s（保存时会再试一次）", r.message.c_str());
  }

  // 见头文件「线程与锁」：三个互斥组，服务之间串行、订阅不被阻塞
  sub_group_     = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_group_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_  = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = sub_group_;

  joint_sub_ = create_subscription<JointState>(
      TOPIC_JOINT_STATES, rclcpp::SensorDataQoS(),
      [this](const JointState & msg) { on_joint_state(msg); }, sub_opt);
  // latched：晚订阅也能立刻拿到当前模式（ModeManager 以 transient_local 发布）
  mode_sub_ = create_subscription<ControlMode>(
      TOPIC_CONTROL_MODE, rclcpp::QoS(1).transient_local().reliable(),
      [this](const ControlMode & msg) { on_control_mode(msg); }, sub_opt);
  status_sub_ = create_subscription<ArmStatus>(
      TOPIC_ARM_STATUS, rclcpp::QoS(10),
      [this](const ArmStatus & msg) { on_arm_status(msg); }, sub_opt);
  jog_sub_ = create_subscription<JogCommandMsg>(
      TOPIC_JOG, rclcpp::QoS(10),
      [this](const JogCommandMsg & msg) { on_jog_command(msg); }, sub_opt);

  state_pub_ = create_publisher<TeachStateMsg>(
      TOPIC_TEACH_STATE, rclcpp::QoS(1).transient_local().reliable());
  playback_pub_ = create_publisher<PlaybackStateMsg>(TOPIC_PLAYBACK_STATE, rclcpp::QoS(10));
  traj_pub_     = create_publisher<JointTrajectoryMsg>(TOPIC_TRAJECTORY, rclcpp::QoS(10));
  jog_out_pub_  = create_publisher<JointVelCommand>(TOPIC_JOINT_VELOCITY, rclcpp::QoS(10));

  // ★ 客户端必须放在**与服务回调不同**的组里。服务回调（service_group_，互斥）里会
  //   阻塞等 future，而应答的处理也要拿一个执行槽 —— 同组的话执行器永远调度不到应答，
  //   服务回调死等到超时（每次切模式都"超时"，而 ModeManager 那边明明已经成功了）。
  mode_cli_     = create_client<SwitchControlMode>(
      SRV_SWITCH_MODE, rmw_qos_profile_services_default, client_group_);
  validity_cli_ = create_client<GetStateValidity>(
      SRV_STATE_VALIDITY, rmw_qos_profile_services_default, client_group_);

  // ── 服务 ──────────────────────────────────────────────────────────────────
  const auto ns = std::string("/robot_arm/teach/");
  start_teach_srv_ = create_service<srv::StartTeach>(
      ns + "start_teach",
      [this](const std::shared_ptr<srv::StartTeach::Request> q,
             std::shared_ptr<srv::StartTeach::Response> s) { srv_start_teach(q, s); },
      rmw_qos_profile_services_default, service_group_);
  stop_teach_srv_ = create_service<srv::StopTeach>(
      ns + "stop_teach",
      [this](const std::shared_ptr<srv::StopTeach::Request> q,
             std::shared_ptr<srv::StopTeach::Response> s) { srv_stop_teach(q, s); },
      rmw_qos_profile_services_default, service_group_);
  pause_teach_srv_ = create_service<Trigger>(
      ns + "pause_teach",
      [this](const std::shared_ptr<Trigger::Request> q,
             std::shared_ptr<Trigger::Response> s) { srv_pause_teach(q, s); },
      rmw_qos_profile_services_default, service_group_);
  resume_teach_srv_ = create_service<Trigger>(
      ns + "resume_teach",
      [this](const std::shared_ptr<Trigger::Request> q,
             std::shared_ptr<Trigger::Response> s) { srv_resume_teach(q, s); },
      rmw_qos_profile_services_default, service_group_);
  save_srv_ = create_service<srv::SaveTrajectory>(
      ns + "save_trajectory",
      [this](const std::shared_ptr<srv::SaveTrajectory::Request> q,
             std::shared_ptr<srv::SaveTrajectory::Response> s) { srv_save_trajectory(q, s); },
      rmw_qos_profile_services_default, service_group_);
  load_srv_ = create_service<srv::LoadTrajectory>(
      ns + "load_trajectory",
      [this](const std::shared_ptr<srv::LoadTrajectory::Request> q,
             std::shared_ptr<srv::LoadTrajectory::Response> s) { srv_load_trajectory(q, s); },
      rmw_qos_profile_services_default, service_group_);
  play_srv_ = create_service<srv::PlayTrajectory>(
      ns + "play_trajectory",
      [this](const std::shared_ptr<srv::PlayTrajectory::Request> q,
             std::shared_ptr<srv::PlayTrajectory::Response> s) { srv_play_trajectory(q, s); },
      rmw_qos_profile_services_default, service_group_);
  stop_playback_srv_ = create_service<Trigger>(
      ns + "stop_playback",
      [this](const std::shared_ptr<Trigger::Request> q,
             std::shared_ptr<Trigger::Response> s) { srv_stop_playback(q, s); },
      rmw_qos_profile_services_default, service_group_);
  pause_playback_srv_ = create_service<Trigger>(
      ns + "pause_playback",
      [this](const std::shared_ptr<Trigger::Request> q,
             std::shared_ptr<Trigger::Response> s) { srv_pause_playback(q, s); },
      rmw_qos_profile_services_default, service_group_);
  resume_playback_srv_ = create_service<Trigger>(
      ns + "resume_playback",
      [this](const std::shared_ptr<Trigger::Request> q,
             std::shared_ptr<Trigger::Response> s) { srv_resume_playback(q, s); },
      rmw_qos_profile_services_default, service_group_);
  list_srv_ = create_service<srv::ListTrajectories>(
      ns + "list_trajectories",
      [this](const std::shared_ptr<srv::ListTrajectories::Request> q,
             std::shared_ptr<srv::ListTrajectories::Response> s) { srv_list_trajectories(q, s); },
      rmw_qos_profile_services_default, service_group_);
  delete_srv_ = create_service<srv::DeleteTrajectory>(
      ns + "delete_trajectory",
      [this](const std::shared_ptr<srv::DeleteTrajectory::Request> q,
             std::shared_ptr<srv::DeleteTrajectory::Response> s) { srv_delete_trajectory(q, s); },
      rmw_qos_profile_services_default, service_group_);
  speed_srv_ = create_service<srv::SetPlaybackSpeed>(
      ns + "set_playback_speed",
      [this](const std::shared_ptr<srv::SetPlaybackSpeed::Request> q,
             std::shared_ptr<srv::SetPlaybackSpeed::Response> s) { srv_set_playback_speed(q, s); },
      rmw_qos_profile_services_default, service_group_);

  // ── 定时器 ────────────────────────────────────────────────────────────────
  auto period = [](double hz) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, hz)));
  };
  sample_timer_ = create_wall_timer(period(p_.sample_rate_hz), [this]() { on_sample_tick(); },
                                    timer_group_);
  jog_timer_    = create_wall_timer(period(p_.jog_relay_rate_hz), [this]() { on_jog_tick(); },
                                    timer_group_);
  playback_timer_ = create_wall_timer(period(p_.republish_hz), [this]() { on_playback_tick(); },
                                      timer_group_);
  state_timer_  = create_wall_timer(period(p_.state_publish_hz), [this]() { on_state_tick(); },
                                    timer_group_);

  phase_ = TeachStateMsg::IDLE;
  publish_teach_state();

  RCLCPP_INFO(get_logger(),
      "示教节点就绪（应用层状态机，与 ControlMode 解耦）\n"
      "  录制  %.0fHz  压缩阈值 %.4frad  上限 %.0fs / %d 点\n"
      "  点动  %s  %.0fHz  限速 %.2frad/s  断流 %.0fms   ★ 只能动 J1-3（产品总线固定 3 轴）\n"
      "  回放  分段 %.1fs / %.0fHz  起点容差 %.3frad  自动接近 %s  速度上限 %.2frad/s\n"
      "  存储  %s\n"
      "  手拖示教 %s（实机无 effort_controller，见 doc/实机能力限制说明.md）",
      p_.sample_rate_hz, p_.compress_position_eps, p_.max_duration_sec, p_.max_points,
      p_.jog_enable_relay ? "中继开" : "中继关", p_.jog_relay_rate_hz, p_.jog_max_joint_speed,
      p_.jog_command_timeout * 1000.0,
      p_.chunk_horizon_sec, p_.republish_hz, p_.start_tolerance_rad,
      p_.auto_approach ? "开" : "关", p_.max_joint_velocity,
      store_.directory().c_str(), p_.allow_drag ? "已允许（仅仿真）" : "已禁用");
}

void TeachNode::declare_params()
{
  p_.sample_rate_hz        = declare_parameter("teach.sample_rate_hz", 50.0);
  p_.compress_position_eps = declare_parameter("teach.compress_position_eps", 0.002);
  p_.compress_max_gap_sec  = declare_parameter("teach.compress_max_gap_sec", 0.5);
  p_.max_duration_sec      = declare_parameter("teach.max_duration_sec", 600.0);
  p_.max_points            = declare_parameter("teach.max_points", 120000);
  p_.allow_drag            = declare_parameter("teach.allow_drag", false);
  p_.restore_trajectory_mode_on_stop =
      declare_parameter("teach.restore_trajectory_mode_on_stop", true);
  p_.storage_directory =
      declare_parameter("teach.storage_directory", default_storage_directory());
  // 空串 = 用默认目录。launch 里 storage_dir 默认就是空串（不做条件分支），
  // 若不在这里兜住，会把存储目录设成 ""，save 时报一个看不懂的路径错误。
  if (p_.storage_directory.empty()) p_.storage_directory = default_storage_directory();

  p_.jog_enable_relay    = declare_parameter("jog.enable_relay", true);
  p_.jog_relay_rate_hz   = declare_parameter("jog.relay_rate_hz", 50.0);
  p_.jog_command_timeout = declare_parameter("jog.command_timeout", 0.3);
  p_.jog_max_joint_speed = declare_parameter("jog.max_joint_speed", 0.4);

  p_.republish_hz           = declare_parameter("playback.republish_hz", 5.0);
  p_.chunk_horizon_sec      = declare_parameter("playback.chunk_horizon_sec", 1.0);
  p_.start_tolerance_rad    = declare_parameter("playback.start_tolerance_rad", 0.05);
  p_.auto_approach          = declare_parameter("playback.auto_approach", true);
  p_.approach_duration_sec  = declare_parameter("playback.approach_duration_sec", 3.0);
  p_.hold_duration_sec      = declare_parameter("playback.hold_duration_sec", 0.2);
  p_.finish_margin_sec      = declare_parameter("playback.finish_margin_sec", 0.3);
  p_.max_joint_velocity     = declare_parameter("playback.max_joint_velocity", 1.0);
  p_.max_joint_acceleration = declare_parameter("playback.max_joint_acceleration", 4.0);
  p_.min_speed_scale        = declare_parameter("playback.min_speed_scale", 0.1);
  p_.max_speed_scale        = declare_parameter("playback.max_speed_scale", 1.0);
  p_.collision_check_max_samples =
      declare_parameter("playback.collision_check_max_samples", 20);
  p_.validity_wait_sec = declare_parameter("playback.validity_wait_sec", 0.3);

  p_.require_arm_status      = declare_parameter("safety.require_arm_status", true);
  p_.arm_status_timeout_sec  = declare_parameter("safety.arm_status_timeout_sec", 2.0);
  p_.mode_switch_timeout_sec = declare_parameter("safety.mode_switch_timeout_sec", 2.0);

  p_.state_publish_hz = declare_parameter("state.publish_hz", 5.0);
}

// ══════════════════════════════════════════════════════════════════════════════
// 工具
// ══════════════════════════════════════════════════════════════════════════════
double TeachNode::steady_now()
{
  // 全节点的间隔/超时判断都走这里。Gazebo 的 /clock 只有 10Hz，用 ROS 时钟求 dt 会
  // 系统性丢步（robot_arm_node 的速度流积分踩过同一个坑）。
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

TeachNode::JointSnapshot TeachNode::joint_snapshot() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return snap_;
}

bool TeachNode::arm_ready(std::string * reason) const
{
  const bool has = has_arm_status_.load();
  if (!has) {
    if (p_.require_arm_status) {
      if (reason) {
        *reason = "未收到 /robot_arm/arm_status（arm_commander 未运行？）—— 无法确认急停/故障状态。"
                  "确认机械臂正常后可设 safety.require_arm_status:=false 跳过本闸";
      }
      return false;
    }
    if (reason) *reason = "未收到 arm_status，已按 require_arm_status:=false 放行";
    return true;
  }
  const double age = steady_now() - arm_status_stamp_.load();
  if (p_.arm_status_timeout_sec > 0.0 && age > p_.arm_status_timeout_sec) {
    if (reason) {
      *reason = "arm_status 已过期 " + std::to_string(age) + "s（> " +
                std::to_string(p_.arm_status_timeout_sec) + "s），无法确认当前是否急停";
    }
    return p_.require_arm_status ? false : true;
  }
  const uint8_t err = arm_error_code_.load();
  if (err != ArmStatus::ERR_NONE) {
    if (reason) {
      *reason = "机械臂处于故障/急停状态（error_code=" + std::to_string(static_cast<int>(err)) +
                "），先调 /robot_arm/reset_error 清除";
    }
    return false;
  }
  return true;
}

bool TeachNode::switch_control_mode(uint8_t target, std::string * message)
{
  // 已经在目标模式就不再折腾一次（切换本身是幂等的，但少一次跨进程往返）
  if (has_control_mode_.load() && control_mode_.load() == target) {
    if (message) *message = "已处于目标模式";
    return true;
  }
  if (!mode_cli_->service_is_ready()) {
    if (message) {
      *message = std::string("控制模式服务不可用：") + SRV_SWITCH_MODE +
                 "（mode_manager_node 未运行？）";
    }
    return false;
  }
  auto req = std::make_shared<SwitchControlMode::Request>();
  req->target_mode = target;
  auto future = mode_cli_->async_send_request(req);
  if (future.wait_for(std::chrono::duration<double>(p_.mode_switch_timeout_sec)) !=
      std::future_status::ready)
  {
    mode_cli_->remove_pending_request(future);
    if (message) *message = "切换控制模式超时";
    return false;
  }
  const auto res = future.get();
  if (message) *message = res->message;
  if (res->success) {
    control_mode_.store(res->active_mode);
    has_control_mode_.store(true);
  }
  return res->success;
}

bool TeachNode::collision_free(const TeachTrajectoryMsg & traj, std::string * detail)
{
  if (!validity_cli_->service_is_ready()) {
    // fail-open：move_group 未运行（无 MoveIt 场景）时不拦，与 MoveToJointServer 一致
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS,
        "%s 不可用（move_group 未运行？）—— 本次回放未做自碰撞检查",
        SRV_STATE_VALIDITY.c_str());
    if (detail) *detail = "自碰撞检查已跳过（服务不可用）";
    return true;
  }
  if (traj.points.empty()) return true;

  // 抽样：逐点查一条 3000 点的轨迹要 3000 次跨进程往返，回放请求会卡上几十秒。
  // 抽样漏检的风险由「示教轨迹本身是机械臂刚刚走过的路径」兜住 —— 它天然无碰撞，
  // 这道闸真正防的是**手改过的 YAML** 和跨机器复用的轨迹，那种问题通常是整段偏移，
  // 抽样能抓到。
  const size_t n = traj.points.size();
  const size_t max_samples = static_cast<size_t>(std::max(1, p_.collision_check_max_samples));
  const size_t stride = std::max<size_t>(1, (n + max_samples - 1) / max_samples);

  for (size_t i = 0; i < n; i += stride) {
    auto req = std::make_shared<GetStateValidity::Request>();
    req->group_name = PLANNING_GROUP;
    req->robot_state.joint_state.name = traj.joint_names;
    req->robot_state.joint_state.position.assign(traj.points[i].positions.begin(),
                                                 traj.points[i].positions.end());
    req->robot_state.is_diff = false;

    auto future = validity_cli_->async_send_request(req);
    if (future.wait_for(std::chrono::duration<double>(p_.validity_wait_sec)) !=
        std::future_status::ready)
    {
      validity_cli_->remove_pending_request(future);
      RCLCPP_WARN(get_logger(), "自碰撞检查第 %zu 点超时（%.1fs）—— 本点放行（fail-open）",
                  i, p_.validity_wait_sec);
      continue;
    }
    if (!future.get()->valid) {
      if (detail) {
        *detail = "第 " + std::to_string(i) + " 个点自碰撞（/check_state_validity 判定 invalid）";
      }
      return false;
    }
  }
  if (detail) *detail = "自碰撞抽样检查通过（" + std::to_string((n + stride - 1) / stride) +
                        " / " + std::to_string(n) + " 点）";
  return true;
}

ValidateResult TeachNode::preflight(const TeachTrajectoryMsg & traj, double speed_scale,
                                    const JointSnapshot & snap, bool * need_approach,
                                    std::string * approach_detail) const
{
  if (need_approach) *need_approach = false;

  // 闸1、2：结构 / 关节名 / NaN / 时间单调
  if (auto r = validate_structure(traj, teach_joint_names()); !r) return r;

  // 闸3：URDF 限位（limits 为空 → fail-open）
  const JointBoundMap bounds = limits_.bounds();
  if (bounds.empty()) {
    RCLCPP_WARN(get_logger(), "关节限位尚未从 %s 解析到 —— 本次不做限位校验",
                TOPIC_DESCRIPTION.c_str());
  }
  if (auto r = validate_limits(traj, bounds); !r) return r;

  // 闸4：速度 / 加速度（含 speed_scale 缩放）
  MotionCaps caps;
  caps.max_velocity     = p_.max_joint_velocity;
  caps.max_acceleration = p_.max_joint_acceleration;
  if (auto r = validate_speed(traj, caps, speed_scale); !r) return r;

  // 闸6：当前位置是否在起点附近
  if (!snap.has_all) {
    return ValidateResult::failure(
        "no_joint_state",
        std::string("6 轴回读不全（") + TOPIC_JOINT_STATES +
            " 缺轴）—— 无法判断是否在轨迹起点，不下发");
  }
  std::vector<double> cur(snap.positions.begin(), snap.positions.end());
  double dev = 0.0;
  if (!is_at_start(traj, cur, p_.start_tolerance_rad, JOG_JOINT_COUNT, &dev)) {
    if (!p_.auto_approach) {
      return ValidateResult::failure(
          "not_at_start",
          "当前位置离轨迹起点 " + std::to_string(dev) + "rad（容差 " +
              std::to_string(p_.start_tolerance_rad) +
              "）—— 设 playback.auto_approach:=true 可自动先走到起点");
    }
    if (need_approach) *need_approach = true;
    if (approach_detail) {
      *approach_detail = "离起点 " + std::to_string(dev) + "rad，将先用 " +
                         std::to_string(p_.approach_duration_sec) + "s 慢速接近";
    }
  }
  return ValidateResult::success();
}

void TeachNode::publish_hold(const char * why)
{
  // 调用前须持 mtx_（读 snap_）
  if (!snap_.has_all) {
    // 回读不全时**什么都不发**。发一条位置缺省为 0 的保持轨迹会让机械臂冲向 0 位，
    // 比不发危险得多（StatusAggregator 的注释里是同一个坑）。
    RCLCPP_ERROR(get_logger(), "需要下发保持轨迹（%s）但 6 轴回读不全 —— 未下发任何指令，"
                               "机械臂将由 JTC 停在最后一条轨迹的末点", why);
    return;
  }
  std::vector<double> cur(snap_.positions.begin(), snap_.positions.end());
  auto hold = planner_.make_hold(teach_joint_names(), cur);
  if (hold.points.empty()) return;
  traj_pub_->publish(hold);
  RCLCPP_INFO(get_logger(), "已下发保持轨迹（%s）：停在当前实测位置", why);
}

// ══════════════════════════════════════════════════════════════════════════════
// 订阅回调
// ══════════════════════════════════════════════════════════════════════════════
void TeachNode::on_joint_state(const JointState & msg)
{
  const auto & want = teach_joint_names();

  std::lock_guard<std::mutex> lk(mtx_);
  // ★ 逐轴合并，而不是整帧覆盖。
  //   joint_state_broadcaster 正常情况下一帧就带全 6 轴，但不能假设：换后端 / 云台单独
  //   发一路时会出现「每帧只带 3 轴」的情况。整帧覆盖的话 has_all 永远为假，
  //   症状是状态一直 RECORDING 而 point_count 不涨（采样器在等齐 6 轴）。
  //   合并 + 位掩码累计覆盖情况，两种发布方式都成立。
  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = std::find(want.begin(), want.end(), msg.name[i]);
    if (it == want.end()) continue;
    const size_t j = static_cast<size_t>(std::distance(want.begin(), it));
    if (i < msg.position.size()) {
      snap_.positions[j] = msg.position[i];
      joint_seen_mask_ |= static_cast<uint8_t>(1u << j);
    }
    if (i < msg.velocity.size()) {
      snap_.velocities[j] = msg.velocity[i];
      joint_vel_mask_ |= static_cast<uint8_t>(1u << j);
    }
  }

  constexpr uint8_t ALL_SIX = 0x3F;
  snap_.has_all = (joint_seen_mask_ == ALL_SIX);
  // 有任一轴带速度就认为回读带速度字段。云台那几轴若没有速度就保持 0 ——
  // 点动动不了云台，它本来就是被保持的，包络统计不受影响。
  snap_.has_velocity = (joint_vel_mask_ != 0);
  snap_.stamp_steady = steady_now();
  // 注：单个轴的**陈旧**不做过期处理（与 StatusAggregator 的关节缓存一致）。
  // 云台中途掉线时这里仍是最后一次回读值；那种情况由回放的急停/故障闸和
  // 云台侧自己的告警负责，不在本节点里再造一套超时判据。
}

void TeachNode::on_control_mode(const ControlMode & msg)
{
  control_mode_.store(msg.mode);
  has_control_mode_.store(true);
}

void TeachNode::on_arm_status(const ArmStatus & msg)
{
  arm_error_code_.store(msg.error_code);
  arm_status_stamp_.store(steady_now());
  has_arm_status_.store(true);
}

void TeachNode::on_jog_command(const JogCommandMsg & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  for (size_t i = 0; i < JOG_JOINT_COUNT; ++i) {
    const double v = msg.velocities[i];
    jog_cmd_[i] = std::isfinite(v)
                      ? std::clamp(v, -p_.jog_max_joint_speed, p_.jog_max_joint_speed)
                      : 0.0;
  }
  jog_stamp_   = steady_now();
  jog_has_cmd_ = true;
}

// ══════════════════════════════════════════════════════════════════════════════
// 定时器
// ══════════════════════════════════════════════════════════════════════════════
void TeachNode::on_sample_tick()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::RECORDING) return;
  if (!snap_.has_all) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS,
        "6 轴回读不全，采样暂停（云台 J4-6 未上电 / 跨机 DDS 不通？）");
    return;
  }

  const auto outcome = recorder_.sample(steady_now(), snap_.positions, snap_.velocities,
                                        snap_.has_velocity);
  switch (outcome) {
    case TeachRecorder::SampleOutcome::StoppedFull:
      phase_         = TeachStateMsg::RECORD_PAUSED;
      phase_message_ = "已达采样点上限，录制自动停止 —— 请调 stop_teach 收尾";
      RCLCPP_WARN(get_logger(), "%s", phase_message_.c_str());
      break;
    case TeachRecorder::SampleOutcome::StoppedTimeout:
      phase_         = TeachStateMsg::RECORD_PAUSED;
      phase_message_ = "已达录制时长上限，录制自动停止 —— 请调 stop_teach 收尾";
      RCLCPP_WARN(get_logger(), "%s", phase_message_.c_str());
      break;
    case TeachRecorder::SampleOutcome::Invalid:
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS,
                           "回读含 NaN / inf，本帧已丢弃");
      break;
    default:
      break;
  }
}

void TeachNode::on_jog_tick()
{
  if (!p_.jog_enable_relay) return;

  JointVelCommand out;
  bool publish = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const bool gate_open = (phase_ == TeachStateMsg::RECORDING);
    const bool fresh = jog_has_cmd_ &&
                       (steady_now() - jog_stamp_) <= p_.jog_command_timeout;

    if (gate_open && fresh) {
      out.velocities.assign(jog_cmd_.begin(), jog_cmd_.end());
      publish        = true;
      jog_zero_sent_ = false;
    } else if (!jog_zero_sent_) {
      // 闸关上或指令断流：补发一帧全 0 明确停住。
      // 光停发布也会停（VelocityStreamServer 有断流看门狗），但那要等 command_timeout；
      // 一帧 0 是**立刻**停，暂停示教时手感差别很明显。
      out.velocities.assign(JOG_JOINT_COUNT, 0.0);
      publish        = true;
      jog_zero_sent_ = true;
    }
  }
  if (publish) jog_out_pub_->publish(out);
}

void TeachNode::on_playback_tick()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::PLAYING) return;

  const double now = steady_now();

  // 急停 / 故障：立刻停在当前位置
  std::string reason;
  if (!arm_ready(&reason)) {
    finish_playback(PlaybackStateMsg::ABORTED, "回放中断：" + reason, true);
    return;
  }
  // 底层模式被别人切走了（另一个应用抢了速度总线）：不能继续往轨迹总线灌
  if (has_control_mode_.load() && control_mode_.load() != ControlMode::TRAJECTORY) {
    finish_playback(PlaybackStateMsg::ABORTED,
                    "回放中断：底层控制模式已被切离 TRAJECTORY", true);
    return;
  }

  // 接近段：等它走完再开始正式分段流
  if (play_approaching_) {
    if (now < play_approach_until_) {
      publish_playback_state(PlaybackStateMsg::PLAYING, "正在接近轨迹起点");
      return;
    }
    play_approaching_ = false;
    play_first_tick_  = true;   // 接近段结束后重新对齐时间基准
    RCLCPP_INFO(get_logger(), "已到达轨迹起点附近，开始回放");
  }

  // 第一拍只对齐基准：从服务下发到这一拍之间的墙钟不该算作"已播放"
  if (play_first_tick_) {
    play_first_tick_ = false;
    play_last_tick_  = now;
  }

  // 进度按**实际经过的墙钟时间 × 倍率**推进（而不是 tick 计数），
  // 这样中途改倍率、以及定时器抖动都自然吸收掉。
  const double dt = std::clamp(now - play_last_tick_, 0.0, 5.0 / std::max(1.0, p_.republish_hz));
  play_last_tick_ = now;
  play_phase_ += dt * play_speed_;

  const double total = play_traj_.duration_sec;
  if (play_phase_ >= total + p_.finish_margin_sec) {
    if (play_loops_left_ > 1) {
      --play_loops_left_;
      play_phase_ = 0.0;
      play_index_ = 0;
      play_first_tick_ = true;
      RCLCPP_INFO(get_logger(), "回放循环，剩余 %d 次", play_loops_left_);

      // ★ 末点与首点差得远时必须先插一段接近段。
      //   直接从首点开始下一轮的话，JTC 会在一个分段首点的时长内（默认 0.1s）
      //   把机械臂从终点拉回起点 —— 那是一次没人预期的高速运动。
      //   判据同起点闸：只看 J1-3（末端在云台之后，判据交给云台会永远不满足）。
      std::vector<double> cur(snap_.positions.begin(), snap_.positions.end());
      double dev = 0.0;
      if (snap_.has_all &&
          !is_at_start(play_traj_, cur, p_.start_tolerance_rad, JOG_JOINT_COUNT, &dev))
      {
        auto app = planner_.make_approach(play_traj_, teach_joint_names());
        if (!app.points.empty()) {
          traj_pub_->publish(app);
          play_approaching_    = true;
          play_approach_until_ = now + p_.approach_duration_sec;
          RCLCPP_INFO(get_logger(),
              "循环前先回起点（当前离首点 %.3frad > 容差 %.3frad），用 %.1fs 慢速接近",
              dev, p_.start_tolerance_rad, p_.approach_duration_sec);
          publish_playback_state(PlaybackStateMsg::PLAYING, "循环：正在回到轨迹起点");
          return;
        }
      }
    } else {
      finish_playback(PlaybackStateMsg::FINISHED, "回放完成", false);
      return;
    }
  }

  auto chunk = planner_.make_chunk(play_traj_, play_phase_, play_speed_);
  if (!chunk.trajectory.points.empty()) {
    traj_pub_->publish(chunk.trajectory);
    play_index_ = chunk.point_index;
  }
  publish_playback_state(PlaybackStateMsg::PLAYING, "");
}

void TeachNode::on_state_tick()
{
  std::lock_guard<std::mutex> lk(mtx_);
  publish_teach_state();
}

// ══════════════════════════════════════════════════════════════════════════════
// 状态广播
// ══════════════════════════════════════════════════════════════════════════════
void TeachNode::publish_teach_state()
{
  // 调用前须持 mtx_
  TeachStateMsg msg;
  msg.header.stamp    = now();
  msg.header.frame_id = "arm_base_link";
  msg.state           = phase_;
  msg.teach_mode      = recorder_.teach_mode();
  msg.trajectory_name = (phase_ == TeachStateMsg::PLAYING || phase_ == TeachStateMsg::PLAY_PAUSED)
                            ? play_traj_.name : recorder_.name();
  msg.motion_type.value =
      (phase_ == TeachStateMsg::PLAYING || phase_ == TeachStateMsg::PLAY_PAUSED)
          ? play_traj_.motion_type.value : recorder_.motion_type();
  msg.point_count = static_cast<uint32_t>(
      (phase_ == TeachStateMsg::RECORDING || phase_ == TeachStateMsg::RECORD_PAUSED)
          ? recorder_.kept_point_count()
          : (has_buffer_ ? buffer_.points.size() : 0));
  msg.elapsed_sec = (phase_ == TeachStateMsg::PLAYING || phase_ == TeachStateMsg::PLAY_PAUSED)
                        ? play_phase_ : recorder_.elapsed_sec(steady_now());
  msg.jog_relay_enabled = p_.jog_enable_relay && (phase_ == TeachStateMsg::RECORDING);
  msg.underlying_control_mode.mode = control_mode_.load();
  msg.message = phase_message_;
  state_pub_->publish(msg);
}

void TeachNode::publish_playback_state(uint8_t status, const std::string & message)
{
  // 调用前须持 mtx_
  PlaybackStateMsg msg;
  msg.header.stamp      = now();
  msg.status            = status;
  msg.trajectory_name   = play_traj_.name;
  msg.point_index       = static_cast<uint32_t>(play_index_);
  msg.point_total       = static_cast<uint32_t>(play_traj_.points.size());
  msg.elapsed_sec       = play_phase_;
  msg.total_sec         = play_traj_.duration_sec;
  msg.progress_percent  = play_traj_.duration_sec > 1e-9
      ? static_cast<float>(std::min(100.0, play_phase_ / play_traj_.duration_sec * 100.0))
      : 0.0f;
  msg.speed_scale = play_speed_;
  msg.message     = message;
  playback_pub_->publish(msg);
}

void TeachNode::finish_playback(uint8_t status, const std::string & message, bool send_hold)
{
  // 调用前须持 mtx_
  if (send_hold) publish_hold(message.c_str());
  publish_playback_state(status, message);
  phase_         = TeachStateMsg::IDLE;
  phase_message_ = message;
  play_approaching_ = false;
  play_loops_left_  = 0;
  publish_teach_state();
  if (status == PlaybackStateMsg::FINISHED) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

}  // namespace robot_arm_teach
