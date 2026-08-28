/**
 * @file teach_node.hpp
 * @brief 示教节点 —— 10 个服务 + 状态广播 + 采样/点动中继/回放三个定时器
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * 与既有工程的耦合面（刻意压到最小，全部是「用」而不是「改」）
 * ══════════════════════════════════════════════════════════════════════════════
 *   读  /joint_states                        6 轴回读（录制的唯一数据源）
 *   读  /robot_arm/control_mode  (latched)   当前底层模式，只作镜像与闸
 *   读  /robot_arm/arm_status                急停/故障判据（error_code）
 *   读  /robot_description       (latched)   URDF 关节限位
 *   写  /robot_arm/cmd/joint_velocity        点动示教（既有产品总线，3 轴）
 *   写  /arm_controller/joint_trajectory     回放（既有轨迹总线）
 *   调  /robot_arm/switch_control_mode       切模式的**唯一**入口
 *   调  /check_state_validity                自碰撞（move_group 提供，不可用则 fail-open）
 *
 * 一件都不做的事：不碰 controller_manager/switch_controller、不碰 CAN / 驱动器、
 * 不改 ControlMode.msg、不改 ModeManager / ArmCommander 的任何逻辑。
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * 示教状态与底层控制模式的关系（requirement 1 的落地）
 * ══════════════════════════════════════════════════════════════════════════════
 *   TeachState.state（IDLE/RECORDING/RECORD_PAUSED/PLAYING/PLAY_PAUSED）是**应用状态**，
 *   活在本节点里；ControlMode 是**底层控制方式**，活在 ModeManager 里。本节点只在两个
 *   时刻请求切换底层模式：
 *     start_teach(JOG) → JOINT_VELOCITY      play_trajectory → TRAJECTORY
 *   切换一律经 /robot_arm/switch_control_mode。ModeManager 完全不知道「示教」这回事，
 *   这正是解耦的目的：示教的状态机演化不会牵动控制模式仲裁。
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * 线程与锁
 * ══════════════════════════════════════════════════════════════════════════════
 *   跑在 MultiThreadedExecutor 上，三个互斥回调组：
 *     sub_group_      /joint_states 等订阅（100Hz，必须不被阻塞）
 *     timer_group_    采样 / 点动中继 / 回放 / 状态广播（彼此串行，逻辑最简单）
 *     service_group_  10 个服务（彼此串行 —— 与 ModeManager 串行化切换同一思路：
 *                     两个并发的 start_teach 没有任何合理语义）
 *   ★ 铁律：**绝不持锁等 future**。服务里要调 /switch_control_mode 或
 *     /check_state_validity 时，先在锁内取快照、出锁再等应答、拿到结果再进锁提交状态。
 *     持锁等 future 会和 timer_group_ 里的采样/回放死锁（它们要同一把锁）。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <robot_arm_interfaces/msg/arm_joint_velocity_command.hpp>
#include <robot_arm_interfaces/msg/arm_status.hpp>
#include <robot_arm_interfaces/msg/control_mode.hpp>
#include <robot_arm_interfaces/srv/switch_control_mode.hpp>

#include "robot_arm_teach/joint_limits_guard.hpp"
#include "robot_arm_teach/msg/jog_command.hpp"
#include "robot_arm_teach/msg/playback_state.hpp"
#include "robot_arm_teach/msg/teach_state.hpp"
#include "robot_arm_teach/msg/teach_trajectory.hpp"
#include "robot_arm_teach/playback_planner.hpp"
#include "robot_arm_teach/srv/delete_trajectory.hpp"
#include "robot_arm_teach/srv/list_trajectories.hpp"
#include "robot_arm_teach/srv/load_trajectory.hpp"
#include "robot_arm_teach/srv/play_trajectory.hpp"
#include "robot_arm_teach/srv/save_trajectory.hpp"
#include "robot_arm_teach/srv/set_playback_speed.hpp"
#include "robot_arm_teach/srv/start_teach.hpp"
#include "robot_arm_teach/srv/stop_teach.hpp"
#include "robot_arm_teach/teach_recorder.hpp"
#include "robot_arm_teach/teach_types.hpp"
#include "robot_arm_teach/teach_validator.hpp"
#include "robot_arm_teach/trajectory_store.hpp"

namespace robot_arm_teach
{

class TeachNode : public rclcpp::Node
{
public:
  explicit TeachNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using JointState        = sensor_msgs::msg::JointState;
  using ArmStatus         = robot_arm_interfaces::msg::ArmStatus;
  using ControlMode       = robot_arm_interfaces::msg::ControlMode;
  using JointVelCommand   = robot_arm_interfaces::msg::ArmJointVelocityCommand;
  using SwitchControlMode = robot_arm_interfaces::srv::SwitchControlMode;
  using GetStateValidity  = moveit_msgs::srv::GetStateValidity;
  using Trigger           = std_srvs::srv::Trigger;

  using TeachStateMsg    = robot_arm_teach::msg::TeachState;
  using PlaybackStateMsg = robot_arm_teach::msg::PlaybackState;
  using JogCommandMsg    = robot_arm_teach::msg::JogCommand;

  // ── 参数 ──────────────────────────────────────────────────────────────────
  struct Params
  {
    // teach.*
    double      sample_rate_hz{50.0};
    double      compress_position_eps{0.002};
    double      compress_max_gap_sec{0.5};
    double      max_duration_sec{600.0};
    int         max_points{120000};
    bool        allow_drag{false};
    bool        restore_trajectory_mode_on_stop{true};
    std::string storage_directory;

    // jog.*
    bool   jog_enable_relay{true};
    double jog_relay_rate_hz{50.0};
    double jog_command_timeout{0.3};
    double jog_max_joint_speed{0.4};

    // playback.*
    double republish_hz{5.0};
    double chunk_horizon_sec{1.0};
    double start_tolerance_rad{0.05};
    bool   auto_approach{true};
    double approach_duration_sec{3.0};
    double hold_duration_sec{0.2};
    double finish_margin_sec{0.3};
    double max_joint_velocity{1.0};
    double max_joint_acceleration{4.0};
    double min_speed_scale{0.1};
    double max_speed_scale{1.0};
    int    collision_check_max_samples{20};
    double validity_wait_sec{0.3};

    // safety.*
    bool   require_arm_status{true};
    double arm_status_timeout_sec{2.0};
    double mode_switch_timeout_sec{2.0};

    // state.*
    double state_publish_hz{5.0};
  };

  void declare_params();

  // ── 订阅回调 ──────────────────────────────────────────────────────────────
  void on_joint_state(const JointState & msg);
  void on_control_mode(const ControlMode & msg);
  void on_arm_status(const ArmStatus & msg);
  void on_jog_command(const JogCommandMsg & msg);

  // ── 定时器 ────────────────────────────────────────────────────────────────
  void on_sample_tick();
  void on_jog_tick();
  void on_playback_tick();
  void on_state_tick();

  // ── 服务 ──────────────────────────────────────────────────────────────────
  void srv_start_teach(const std::shared_ptr<srv::StartTeach::Request> req,
                       std::shared_ptr<srv::StartTeach::Response> res);
  void srv_stop_teach(const std::shared_ptr<srv::StopTeach::Request> req,
                      std::shared_ptr<srv::StopTeach::Response> res);
  void srv_pause_teach(const std::shared_ptr<Trigger::Request> req,
                       std::shared_ptr<Trigger::Response> res);
  void srv_resume_teach(const std::shared_ptr<Trigger::Request> req,
                        std::shared_ptr<Trigger::Response> res);
  void srv_save_trajectory(const std::shared_ptr<srv::SaveTrajectory::Request> req,
                           std::shared_ptr<srv::SaveTrajectory::Response> res);
  void srv_load_trajectory(const std::shared_ptr<srv::LoadTrajectory::Request> req,
                           std::shared_ptr<srv::LoadTrajectory::Response> res);
  void srv_play_trajectory(const std::shared_ptr<srv::PlayTrajectory::Request> req,
                           std::shared_ptr<srv::PlayTrajectory::Response> res);
  void srv_stop_playback(const std::shared_ptr<Trigger::Request> req,
                         std::shared_ptr<Trigger::Response> res);
  void srv_pause_playback(const std::shared_ptr<Trigger::Request> req,
                          std::shared_ptr<Trigger::Response> res);
  void srv_resume_playback(const std::shared_ptr<Trigger::Request> req,
                           std::shared_ptr<Trigger::Response> res);
  void srv_list_trajectories(const std::shared_ptr<srv::ListTrajectories::Request> req,
                             std::shared_ptr<srv::ListTrajectories::Response> res);
  void srv_delete_trajectory(const std::shared_ptr<srv::DeleteTrajectory::Request> req,
                             std::shared_ptr<srv::DeleteTrajectory::Response> res);
  void srv_set_playback_speed(const std::shared_ptr<srv::SetPlaybackSpeed::Request> req,
                              std::shared_ptr<srv::SetPlaybackSpeed::Response> res);

  // ── 内部工具 ──────────────────────────────────────────────────────────────
  static double steady_now();

  // 关节回读快照。has_all=false 表示 6 轴里有的还没收到 —— 拿它当目标下发前必须先看这个，
  // 否则缺回读的轴会被当成 0 位（StatusAggregator 的注释里同一个坑）。
  struct JointSnapshot
  {
    JointArray positions{};
    JointArray velocities{};
    bool has_all{false};
    bool has_velocity{false};
    double stamp_steady{0.0};
  };
  JointSnapshot joint_snapshot() const;

  // 急停/故障闸。ok=false 时 reason 里写清是哪种情况。
  bool arm_ready(std::string * reason) const;

  // 切底层控制模式（唯一入口）。阻塞等应答，**不得在持锁时调用**。
  bool switch_control_mode(uint8_t target, std::string * message);

  // 自碰撞抽样检查。服务不可用/超时 → fail-open 返回 true 并告警（与 MoveToJointServer 一致）。
  // **不得在持锁时调用**。
  bool collision_free(const TeachTrajectoryMsg & traj, std::string * detail);

  // 回放前置校验（闸1~4、6）。纯计算，可持锁调用。
  ValidateResult preflight(const TeachTrajectoryMsg & traj, double speed_scale,
                           const JointSnapshot & snap, bool * need_approach,
                           std::string * approach_detail) const;

  // 下发保持轨迹：明确停在当前实测位置。回读不全时退化为「什么都不发」并告警 ——
  // 发一条位置为 0 的轨迹比不发危险得多。
  void publish_hold(const char * why);

  void publish_teach_state();
  void publish_playback_state(uint8_t status, const std::string & message);

  // 结束回放（正常播完 / 停止 / 中断），统一收口
  void finish_playback(uint8_t status, const std::string & message, bool send_hold);

  // ── 成员 ──────────────────────────────────────────────────────────────────
  Params p_{};

  // 回调组（见文件头「线程与锁」）
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  // 客户端单独一组（Reentrant）：服务回调里阻塞等 future 时，应答要能在别的线程被处理，
  // 同组必死锁 —— 表现为每次切模式都"超时"，而 ModeManager 侧其实已经成功了
  rclcpp::CallbackGroup::SharedPtr client_group_;

  // 子系统
  TeachRecorder    recorder_;
  TrajectoryStore  store_;
  PlaybackPlanner  planner_;
  JointLimitsGuard limits_;

  // 订阅 / 发布
  rclcpp::Subscription<JointState>::SharedPtr     joint_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr    mode_sub_;
  rclcpp::Subscription<ArmStatus>::SharedPtr      status_sub_;
  rclcpp::Subscription<JogCommandMsg>::SharedPtr  jog_sub_;
  rclcpp::Publisher<TeachStateMsg>::SharedPtr     state_pub_;
  rclcpp::Publisher<PlaybackStateMsg>::SharedPtr  playback_pub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::Publisher<JointVelCommand>::SharedPtr   jog_out_pub_;

  // 客户端
  rclcpp::Client<SwitchControlMode>::SharedPtr mode_cli_;
  rclcpp::Client<GetStateValidity>::SharedPtr  validity_cli_;

  // 服务
  rclcpp::Service<srv::StartTeach>::SharedPtr        start_teach_srv_;
  rclcpp::Service<srv::StopTeach>::SharedPtr         stop_teach_srv_;
  rclcpp::Service<Trigger>::SharedPtr                pause_teach_srv_;
  rclcpp::Service<Trigger>::SharedPtr                resume_teach_srv_;
  rclcpp::Service<srv::SaveTrajectory>::SharedPtr    save_srv_;
  rclcpp::Service<srv::LoadTrajectory>::SharedPtr    load_srv_;
  rclcpp::Service<srv::PlayTrajectory>::SharedPtr    play_srv_;
  rclcpp::Service<Trigger>::SharedPtr                stop_playback_srv_;
  rclcpp::Service<Trigger>::SharedPtr                pause_playback_srv_;
  rclcpp::Service<Trigger>::SharedPtr                resume_playback_srv_;
  rclcpp::Service<srv::ListTrajectories>::SharedPtr  list_srv_;
  rclcpp::Service<srv::DeleteTrajectory>::SharedPtr  delete_srv_;
  rclcpp::Service<srv::SetPlaybackSpeed>::SharedPtr  speed_srv_;

  // 定时器
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr jog_timer_;
  rclcpp::TimerBase::SharedPtr playback_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // ── 共享状态（mtx_ 保护）──────────────────────────────────────────────────
  mutable std::mutex mtx_;

  uint8_t phase_{0};              // TeachState::IDLE 等
  std::string phase_message_;

  JointSnapshot snap_{};          // 最近的 /joint_states 合并结果
  // 已收到过位置 / 速度的轴位掩码（bit0 = Joint1）。逐轴合并需要它来判断
  // 「6 轴是否齐全」—— 见 on_joint_state 的注释。
  uint8_t joint_seen_mask_{0};
  uint8_t joint_vel_mask_{0};

  // 内存缓冲区：录完 / 加载进来的那一条（save 与 play 的默认输入）
  bool               has_buffer_{false};
  TeachTrajectoryMsg buffer_;

  // 回放运行期
  TeachTrajectoryMsg play_traj_;
  double play_phase_{0.0};        // 原始时间轴上的播放进度（s）
  double play_speed_{1.0};
  double play_last_tick_{0.0};    // 上次 tick 的 steady 时刻
  size_t play_index_{0};
  int    play_loops_left_{0};
  bool   play_approaching_{false};
  // 本轮回放的第一拍：只对齐时间基准、不推进进度。不然从下发到第一拍之间的那段
  // 墙钟（约 1/republish_hz）会被算成"已播放"，机械臂还没动、指令却已经跳到 0.2s 之后。
  bool   play_first_tick_{false};
  double play_approach_until_{0.0};   // 接近段结束的 steady 时刻

  // 点动指令缓存
  std::array<double, JOG_JOINT_COUNT> jog_cmd_{};
  double jog_stamp_{0.0};
  bool   jog_has_cmd_{false};
  bool   jog_zero_sent_{true};    // 暂停/结束时是否已经补发过一帧全 0

  // 底层模式镜像与臂状态（原子，读多写少，不进 mtx_ 以免与阻塞调用互相牵扯）
  std::atomic<uint8_t> control_mode_{ControlMode::TRAJECTORY};
  std::atomic<bool>    has_control_mode_{false};
  std::atomic<uint8_t> arm_error_code_{0};
  std::atomic<bool>    has_arm_status_{false};
  std::atomic<double>  arm_status_stamp_{0.0};
};

}  // namespace robot_arm_teach
