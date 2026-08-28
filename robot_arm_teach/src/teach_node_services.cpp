/**
 * @file teach_node_services.cpp
 * @brief TeachNode 的 13 个服务回调（示教 4 + 轨迹管理 4 + 回放 5）
 *
 * 与 teach_node.cpp 分文件只是为了单文件别太长，同一个类。
 *
 * ★ 本文件里所有回调都遵守 teach_node.hpp 的铁律：**不持锁等 future**。
 *   要调 /switch_control_mode 或 /check_state_validity 的地方，一律
 *   「锁内取快照 → 出锁等应答 → 再进锁提交状态」。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/teach_node.hpp"

#include <algorithm>
#include <cmath>

#include "robot_arm_teach/msg/motion_type.hpp"

namespace robot_arm_teach
{

namespace
{
using TeachStateMsg    = robot_arm_teach::msg::TeachState;
using PlaybackStateMsg = robot_arm_teach::msg::PlaybackState;
using MotionTypeMsg    = robot_arm_teach::msg::MotionType;

const char * phase_name(uint8_t p)
{
  switch (p) {
    case TeachStateMsg::RECORDING:     return "RECORDING";
    case TeachStateMsg::RECORD_PAUSED: return "RECORD_PAUSED";
    case TeachStateMsg::PLAYING:       return "PLAYING";
    case TeachStateMsg::PLAY_PAUSED:   return "PLAY_PAUSED";
    default:                           return "IDLE";
  }
}
}  // namespace

// ══════════════════════════════════════════════════════════════════════════════
// 示教录制
// ══════════════════════════════════════════════════════════════════════════════
void TeachNode::srv_start_teach(const std::shared_ptr<srv::StartTeach::Request> req,
                                std::shared_ptr<srv::StartTeach::Response> res)
{
  res->success    = false;
  res->teach_mode = req->teach_mode;

  // ── 闸①：必须空闲（不做隐式抢占 —— 正在录的东西被悄悄丢掉是最难查的那类问题）──
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (phase_ != TeachStateMsg::IDLE) {
      res->message = std::string("当前状态 ") + phase_name(phase_) +
                     "，不能开始示教（先 stop_teach / stop_playback）";
      return;
    }
  }

  // ── 闸②：机械臂无急停无故障 ────────────────────────────────────────────────
  std::string reason;
  if (!arm_ready(&reason)) {
    res->message = "机械臂未就绪：" + reason;
    RCLCPP_WARN(get_logger(), "start_teach 拒绝：%s", reason.c_str());
    return;
  }

  // ── 闸③：示教方式与底层模式 ────────────────────────────────────────────────
  uint8_t mode = req->teach_mode;
  std::string note;

  if (mode == TeachStateMsg::TEACH_MODE_DRAG && !p_.allow_drag) {
    // requirement 10：实机没有 effort_controller 时**不假装支持手拖**。
    // real.launch.py 故意不配 arm_effort_controller，ModeManager 会明确拒绝切换；
    // 与其让用户看到一条含糊的切换失败，不如在这里就说清楚。
    if (req->allow_jog_fallback) {
      mode = TeachStateMsg::TEACH_MODE_JOG;
      note = "手拖示教未启用（实机无 effort_controller），已按 allow_jog_fallback 降级为点动示教；"
             "点动只能驱动 J1-3，云台 J4-6 保持不动";
      RCLCPP_WARN(get_logger(), "%s", note.c_str());
    } else {
      res->message =
          "手拖示教未启用：实机侧没有 effort_controller（real.launch.py 故意不配），"
          "拖动会没有重力补偿而砸下去。仿真里可设 teach.allow_drag:=true 试用；"
          "要点动示教请传 teach_mode=0(JOG)，或置 allow_jog_fallback=true 自动降级。"
          "详见 doc/实机能力限制说明.md";
      RCLCPP_WARN(get_logger(), "start_teach 拒绝手拖示教（teach.allow_drag=false）");
      return;
    }
  }

  const uint8_t target_mode = (mode == TeachStateMsg::TEACH_MODE_DRAG)
                                  ? ControlMode::JOINT_EFFORT
                                  : ControlMode::JOINT_VELOCITY;

  // ── 闸④：切底层模式（唯一入口，不持锁）────────────────────────────────────
  std::string sw_msg;
  if (!switch_control_mode(target_mode, &sw_msg)) {
    if (mode == TeachStateMsg::TEACH_MODE_DRAG && req->allow_jog_fallback) {
      RCLCPP_WARN(get_logger(), "切 JOINT_EFFORT 失败（%s），按 allow_jog_fallback 降级为点动",
                  sw_msg.c_str());
      mode = TeachStateMsg::TEACH_MODE_JOG;
      note = "切 JOINT_EFFORT 被拒（" + sw_msg + "），已降级为点动示教";
      if (!switch_control_mode(ControlMode::JOINT_VELOCITY, &sw_msg)) {
        res->message = "降级后切 JOINT_VELOCITY 仍失败：" + sw_msg;
        return;
      }
    } else {
      res->message = "切换控制模式失败：" + sw_msg;
      RCLCPP_ERROR(get_logger(), "start_teach 失败：%s", res->message.c_str());
      return;
    }
  }

  // 手拖模式下**本节点一个力矩指令都不发**：重力补偿 + 速度阻尼由 mode_manager_node
  // 的 effort 节拍（effort_rate_hz，默认 100Hz，g(q) − d·q̇）持续生成。
  // 这里发任何东西都只是"重力之上的增量"，示教要的恰恰是增量为 0；
  // 而**绝不能**发字面 0 力矩——那在力矩模式下等于自由下垂（CLAUDE.md 记过实测：
  // J2 从 +0.500 直接砸到下限）。
  if (mode == TeachStateMsg::TEACH_MODE_DRAG) {
    RCLCPP_WARN(get_logger(),
        "手拖示教（仅仿真）：本节点不发力矩指令，重力补偿由 mode_manager_node 的 "
        "effort 节拍持续生成。若 gravity_compensation=false 请立即停止 —— 机械臂会下垂");
  }

  // ── 名字 ──────────────────────────────────────────────────────────────────
  std::string name = req->name.empty() ? ("teach_" + compact_timestamp_now()) : req->name;
  if (!is_valid_trajectory_name(name)) {
    res->message = "轨迹名不合法：只允许 [A-Za-z0-9_-]、长度 1..64";
    return;
  }

  uint8_t mt = req->motion_type.value;
  if (motion_type_to_string(mt).empty()) {
    res->message = "未知的 motion_type 取值 " + std::to_string(static_cast<int>(mt));
    return;
  }

  // ── 提交 ──────────────────────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(mtx_);
    recorder_.start(name, mt, mode, target_mode, req->description, steady_now());
    phase_ = TeachStateMsg::RECORDING;
    phase_message_ = note.empty() ? "示教录制中" : note;
    // 点动指令缓存清零：上一轮示教残留的指令不能让这一轮一开闸就动起来
    jog_cmd_.fill(0.0);
    jog_has_cmd_   = false;
    jog_zero_sent_ = true;
    publish_teach_state();
  }

  res->success         = true;
  res->trajectory_name = name;
  res->teach_mode      = mode;
  res->message = note.empty()
      ? (mode == TeachStateMsg::TEACH_MODE_JOG
             ? "点动示教已开始：往 /robot_arm/teach/jog 发 3 轴角速度（只驱动 J1-3，"
               "云台 J4-6 保持不动并被原样录制）"
             : "手拖示教已开始（仅仿真）")
      : note;
  RCLCPP_INFO(get_logger(), "示教开始：%s（%s，%s）", name.c_str(),
              motion_type_to_string(mt).c_str(), teach_mode_to_string(mode).c_str());
}

void TeachNode::srv_stop_teach(const std::shared_ptr<srv::StopTeach::Request>,
                               std::shared_ptr<srv::StopTeach::Response> res)
{
  res->success = false;

  bool ok = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (phase_ != TeachStateMsg::RECORDING && phase_ != TeachStateMsg::RECORD_PAUSED) {
      res->message = std::string("当前状态 ") + phase_name(phase_) + "，没有正在进行的示教";
      return;
    }

    // 先把点动闸关上并补一帧全 0：录制结束的第一件事是让机械臂停住，
    // 而不是先去做数据处理（那要几毫秒，期间手还按着摇杆就还在动）
    JointVelCommand zero;
    zero.velocities.assign(JOG_JOINT_COUNT, 0.0);
    if (p_.jog_enable_relay) jog_out_pub_->publish(zero);
    jog_zero_sent_ = true;
    jog_has_cmd_   = false;

    res->trajectory_name = recorder_.name();
    res->raw_point_count = static_cast<uint32_t>(recorder_.raw_point_count());

    TeachTrajectoryMsg traj;
    ok = recorder_.finish(steady_now(), &traj);
    if (ok) {
      buffer_        = std::move(traj);
      has_buffer_    = true;
      res->point_count  = static_cast<uint32_t>(buffer_.points.size());
      res->duration_sec = buffer_.duration_sec;
    }
    phase_ = TeachStateMsg::IDLE;
    phase_message_ = ok ? "示教已结束，轨迹在内存缓冲区（调 save_trajectory 落盘）"
                        : "示教已结束但采样点不足（< 2），未产出轨迹";
    publish_teach_state();
  }

  // 切回 TRAJECTORY（不持锁）。留在 JOINT_VELOCITY 也不会动，但后续轨迹类动作
  // 会自己再切一次；这里主动切回去让交接干净，也避免误发的速度指令生效。
  if (p_.restore_trajectory_mode_on_stop) {
    std::string sw_msg;
    if (!switch_control_mode(ControlMode::TRAJECTORY, &sw_msg)) {
      RCLCPP_WARN(get_logger(), "示教结束后切回 TRAJECTORY 失败：%s", sw_msg.c_str());
    }
  }

  res->success = ok;
  if (!ok) {
    res->message = "采样点不足（< 2 个），未产出轨迹。检查 /joint_states 6 轴是否齐全，"
                   "以及录制时机械臂是否真的动了";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }
  res->message = "示教结束，共 " + std::to_string(res->point_count) + " 点（原始 " +
                 std::to_string(res->raw_point_count) + " 点），时长 " +
                 std::to_string(res->duration_sec) + "s；调 save_trajectory 落盘";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void TeachNode::srv_pause_teach(const std::shared_ptr<Trigger::Request>,
                                std::shared_ptr<Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::RECORDING) {
    res->success = false;
    res->message = std::string("当前状态 ") + phase_name(phase_) + "，不在录制中";
    return;
  }
  recorder_.pause(steady_now());
  phase_         = TeachStateMsg::RECORD_PAUSED;
  phase_message_ = "示教已暂停（时间轴冻结，点动闸已关）";

  // 关闸的同时补一帧全 0 —— 立刻停住。只靠停止转发也会停，但要等
  // velocity_stream.command_timeout（默认 0.3s），暂停时那 0.3s 的余程手感很差。
  JointVelCommand zero;
  zero.velocities.assign(JOG_JOINT_COUNT, 0.0);
  if (p_.jog_enable_relay) jog_out_pub_->publish(zero);
  jog_zero_sent_ = true;

  publish_teach_state();
  res->success = true;
  res->message = "已暂停，已录 " + std::to_string(recorder_.kept_point_count()) + " 点";
}

void TeachNode::srv_resume_teach(const std::shared_ptr<Trigger::Request>,
                                 std::shared_ptr<Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::RECORD_PAUSED) {
    res->success = false;
    res->message = std::string("当前状态 ") + phase_name(phase_) + "，不在暂停中";
    return;
  }
  if (recorder_.auto_stopped()) {
    res->success = false;
    res->message = "录制已因达到时长/点数上限自动停止，无法继续 —— 请 stop_teach 收尾";
    return;
  }
  recorder_.resume(steady_now());
  phase_         = TeachStateMsg::RECORDING;
  phase_message_ = "示教录制中";
  publish_teach_state();
  res->success = true;
  res->message = "已继续录制";
}

// ══════════════════════════════════════════════════════════════════════════════
// 轨迹管理
// ══════════════════════════════════════════════════════════════════════════════
void TeachNode::srv_save_trajectory(const std::shared_ptr<srv::SaveTrajectory::Request> req,
                                    std::shared_ptr<srv::SaveTrajectory::Response> res)
{
  res->success = false;

  TeachTrajectoryMsg traj;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (req->use_trajectory) {
      traj = req->trajectory;
    } else {
      if (!has_buffer_) {
        res->message = "内存缓冲区为空：先 stop_teach 录一条，或 load_trajectory 载入，"
                       "或置 use_trajectory=true 直接给轨迹";
        return;
      }
      traj = buffer_;
    }
  }

  if (!req->name.empty()) traj.name = req->name;
  if (traj.name.empty()) {
    res->message = "轨迹名为空（请求里没给，缓冲区里也没有）";
    return;
  }
  if (req->motion_type_override) {
    if (motion_type_to_string(req->motion_type.value).empty()) {
      res->message = "未知的 motion_type 取值 " +
                     std::to_string(static_cast<int>(req->motion_type.value));
      return;
    }
    traj.motion_type = req->motion_type;
  }
  if (req->meta_override) traj.motion_meta = req->motion_meta;

  // 落盘前跑完整静态校验：磁盘上不留一条自己都通不过校验的轨迹。
  // 速度闸按倍率 1.0 判 —— 录制时超了速，回放时靠降速也救不回来（降速只降倍率，
  // 不改轨迹本身的形状），所以这时候就该拦。
  if (auto r = validate_structure(traj, teach_joint_names()); !r) {
    res->message = "校验未通过（" + r.reason + "）：" + r.detail;
    RCLCPP_WARN(get_logger(), "save_trajectory 拒绝：%s", res->message.c_str());
    return;
  }
  if (auto r = validate_limits(traj, limits_.bounds()); !r) {
    res->message = "校验未通过（" + r.reason + "）：" + r.detail;
    RCLCPP_WARN(get_logger(), "save_trajectory 拒绝：%s", res->message.c_str());
    return;
  }
  MotionCaps caps{p_.max_joint_velocity, p_.max_joint_acceleration};
  if (auto r = validate_speed(traj, caps, 1.0); !r) {
    res->message = "校验未通过（" + r.reason + "）：" + r.detail;
    RCLCPP_WARN(get_logger(), "save_trajectory 拒绝：%s", res->message.c_str());
    return;
  }

  const auto sr = store_.save(traj, req->overwrite);
  res->success     = sr.ok;
  res->message     = sr.ok ? "已保存" : sr.message;
  res->path        = sr.path;
  res->point_count = static_cast<uint32_t>(traj.points.size());
  if (sr.ok) {
    // 落盘成功后把缓冲区同步成刚存的这一条（名字/元数据可能被 override 改过），
    // 免得紧接着 play 用的还是改前的版本
    std::lock_guard<std::mutex> lk(mtx_);
    buffer_     = traj;
    has_buffer_ = true;
    RCLCPP_INFO(get_logger(), "轨迹已保存：%s（%zu 点）", sr.path.c_str(), traj.points.size());
  } else {
    RCLCPP_WARN(get_logger(), "轨迹保存失败：%s", sr.message.c_str());
  }
}

void TeachNode::srv_load_trajectory(const std::shared_ptr<srv::LoadTrajectory::Request> req,
                                    std::shared_ptr<srv::LoadTrajectory::Response> res)
{
  TeachTrajectoryMsg traj;
  const auto lr = store_.load(req->name, &traj);
  if (!lr) {
    res->success = false;
    res->message = lr.message;
    RCLCPP_WARN(get_logger(), "load_trajectory 失败：%s", lr.message.c_str());
    return;
  }
  res->trajectory = traj;

  // 读进来就校验一遍。坏文件照样把内容返回（便于人工比对哪里不对），只是 success=false。
  const auto vr = validate_structure(traj, teach_joint_names());
  res->success = vr.ok;
  res->message = vr.ok
      ? ("已载入 " + std::to_string(traj.points.size()) + " 点，时长 " +
         std::to_string(traj.duration_sec) + "s")
      : ("已读出文件但校验未通过（" + vr.reason + "）：" + vr.detail);

  if (vr.ok) {
    std::lock_guard<std::mutex> lk(mtx_);
    buffer_     = std::move(traj);
    has_buffer_ = true;
    RCLCPP_INFO(get_logger(), "轨迹已载入：%s", req->name.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
  }
}

void TeachNode::srv_list_trajectories(const std::shared_ptr<srv::ListTrajectories::Request> req,
                                      std::shared_ptr<srv::ListTrajectories::Response> res)
{
  std::vector<TrajectoryStore::Summary> items;
  std::vector<std::string> invalid;
  const auto lr = store_.list(&items, &invalid);

  res->success = lr.ok;
  res->message = lr.message;
  for (const auto & s : items) {
    if (req->filter_by_motion_type && s.motion_type != req->motion_type_filter) continue;
    res->names.push_back(s.name);
    res->created_at.push_back(s.created_at);
    res->motion_types.push_back(s.motion_type);
    res->point_counts.push_back(s.point_count);
    res->durations_sec.push_back(s.duration_sec);
  }
  res->invalid_names = invalid;
  if (!invalid.empty()) {
    RCLCPP_WARN(get_logger(), "存储目录里有 %zu 个文件解析失败（见 invalid_names）",
                invalid.size());
  }
}

void TeachNode::srv_delete_trajectory(const std::shared_ptr<srv::DeleteTrajectory::Request> req,
                                      std::shared_ptr<srv::DeleteTrajectory::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    // 正在回放的那条不让删 —— 文件删了内存里那份还在跑，状态与磁盘不一致最容易看错
    if ((phase_ == TeachStateMsg::PLAYING || phase_ == TeachStateMsg::PLAY_PAUSED) &&
        play_traj_.name == req->name)
    {
      res->success = false;
      res->message = "该轨迹正在回放，拒绝删除（先 stop_playback）";
      return;
    }
  }
  const auto rr = store_.remove(req->name);
  res->success = rr.ok;
  res->message = rr.ok ? "已删除" : rr.message;
  res->path    = rr.path;
  if (rr.ok) RCLCPP_INFO(get_logger(), "轨迹已删除：%s", rr.path.c_str());
}

// ══════════════════════════════════════════════════════════════════════════════
// 回放
// ══════════════════════════════════════════════════════════════════════════════
void TeachNode::srv_play_trajectory(const std::shared_ptr<srv::PlayTrajectory::Request> req,
                                    std::shared_ptr<srv::PlayTrajectory::Response> res)
{
  res->success     = false;
  res->exit_reason = "invalid_trajectory";

  // ── 取轨迹 ────────────────────────────────────────────────────────────────
  TeachTrajectoryMsg traj;
  if (req->name.empty()) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_buffer_) {
      res->exit_reason = "not_found";
      res->message = "内存缓冲区为空且未指定 name：先 load_trajectory 或录一条";
      return;
    }
    traj = buffer_;
  } else {
    const auto lr = store_.load(req->name, &traj);
    if (!lr) {
      res->exit_reason = "not_found";
      res->message     = lr.message;
      return;
    }
  }
  res->point_count = static_cast<uint32_t>(traj.points.size());

  // ── 倍率规范化 ────────────────────────────────────────────────────────────
  double scale = req->speed_scale;
  if (normalize_speed_scale(&scale, p_.min_speed_scale, p_.max_speed_scale)) {
    RCLCPP_WARN(get_logger(), "speed_scale 已夹到 %.2f（允许区间 [%.2f, %.2f]）",
                scale, p_.min_speed_scale, p_.max_speed_scale);
  }
  const int loops = std::max(1, req->loop_count);

  // ── 闸①：必须空闲 ────────────────────────────────────────────────────────
  JointSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (phase_ != TeachStateMsg::IDLE) {
      res->exit_reason = "busy";
      res->message = std::string("当前状态 ") + phase_name(phase_) + "，不能开始回放";
      return;
    }
    snap = snap_;
  }

  // ── 闸②~⑥：结构 / NaN / 限位 / 速度加速度 / 起点距离（纯计算）─────────────
  bool need_approach = false;
  std::string approach_detail;
  const auto vr = preflight(traj, scale, snap, &need_approach, &approach_detail);
  if (!vr) {
    res->exit_reason = vr.reason;
    res->message     = vr.detail;
    RCLCPP_WARN(get_logger(), "play_trajectory 拒绝（%s）：%s", vr.reason.c_str(),
                vr.detail.c_str());
    return;
  }

  // ── 闸⑦：急停 / 故障 ─────────────────────────────────────────────────────
  std::string reason;
  if (!arm_ready(&reason)) {
    res->exit_reason = "estopped";
    res->message     = "机械臂未就绪：" + reason;
    RCLCPP_WARN(get_logger(), "play_trajectory 拒绝：%s", reason.c_str());
    return;
  }

  // ── 闸⑤：自碰撞抽样（阻塞调服务，不持锁）──────────────────────────────────
  std::string coll_detail;
  if (!collision_free(traj, &coll_detail)) {
    res->exit_reason = "collision";
    res->message     = coll_detail;
    RCLCPP_ERROR(get_logger(), "play_trajectory 拒绝：%s", coll_detail.c_str());
    return;
  }

  res->duration_sec = traj.duration_sec / scale * loops;

  if (req->dry_run) {
    res->success     = true;
    res->exit_reason = "dry_run_ok";
    res->message = "前置校验全部通过（未下发任何指令）。" + coll_detail +
                   (need_approach ? ("；" + approach_detail) : std::string());
    RCLCPP_INFO(get_logger(), "play_trajectory dry_run 通过：%s", traj.name.c_str());
    return;
  }

  // ── 闸⑧：切 TRAJECTORY（唯一入口，不持锁）────────────────────────────────
  std::string sw_msg;
  if (!switch_control_mode(ControlMode::TRAJECTORY, &sw_msg)) {
    res->exit_reason = "mode_switch_failed";
    res->message     = "切 TRAJECTORY 失败：" + sw_msg;
    RCLCPP_ERROR(get_logger(), "play_trajectory 失败：%s", res->message.c_str());
    return;
  }

  // ── 闸⑨：提交并下发第一段 ────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(mtx_);
    // 重新确认状态没被别的服务改掉（服务组互斥，理论上不会；但这里的代价只有几行）
    if (phase_ != TeachStateMsg::IDLE) {
      res->exit_reason = "busy";
      res->message = std::string("当前状态 ") + phase_name(phase_) + "，不能开始回放";
      return;
    }
    play_traj_       = std::move(traj);
    play_phase_      = 0.0;
    play_speed_      = scale;
    play_index_      = 0;
    play_loops_left_ = loops;
    play_last_tick_  = steady_now();
    play_first_tick_ = true;
    phase_           = TeachStateMsg::PLAYING;

    if (need_approach) {
      auto app = planner_.make_approach(play_traj_, teach_joint_names());
      if (!app.points.empty()) {
        traj_pub_->publish(app);
        play_approaching_    = true;
        play_approach_until_ = steady_now() + p_.approach_duration_sec;
        RCLCPP_INFO(get_logger(), "先走接近段：%s", approach_detail.c_str());
      }
    }
    phase_message_ = "回放中：" + play_traj_.name;
    publish_teach_state();
    publish_playback_state(PlaybackStateMsg::PLAYING, "开始回放");
  }

  res->success     = true;
  res->exit_reason = "accepted";
  res->message = "回放已开始（倍率 " + std::to_string(scale) + "，循环 " +
                 std::to_string(loops) + " 次）" +
                 (need_approach ? ("；" + approach_detail) : std::string());
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void TeachNode::srv_pause_playback(const std::shared_ptr<Trigger::Request>,
                                   std::shared_ptr<Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::PLAYING) {
    res->success = false;
    res->message = std::string("当前状态 ") + phase_name(phase_) + "，没有正在回放的轨迹";
    return;
  }
  phase_         = TeachStateMsg::PLAY_PAUSED;
  phase_message_ = "回放已暂停";
  // requirement 9：**必须**下发当前位置保持轨迹。发空轨迹不会让 JTC 停下 ——
  // 它会把手上那条剩下的部分继续执行完，表现为"点了暂停还在走"。
  publish_hold("回放暂停");
  publish_playback_state(PlaybackStateMsg::PAUSED, "已暂停并保持当前位置");
  publish_teach_state();
  res->success = true;
  res->message = "已暂停（进度 " + std::to_string(play_phase_) + " / " +
                 std::to_string(play_traj_.duration_sec) + "s）";
}

void TeachNode::srv_resume_playback(const std::shared_ptr<Trigger::Request>,
                                    std::shared_ptr<Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::PLAY_PAUSED) {
    res->success = false;
    res->message = std::string("当前状态 ") + phase_name(phase_) + "，回放不在暂停中";
    return;
  }
  phase_         = TeachStateMsg::PLAYING;
  phase_message_ = "回放中：" + play_traj_.name;
  // 重置 tick 基准：否则下一拍会把整个暂停时长当成"已播放"，进度直接跳过去一大段
  play_last_tick_ = steady_now();
  publish_playback_state(PlaybackStateMsg::PLAYING, "已继续");
  publish_teach_state();
  res->success = true;
  res->message = "已继续回放";
}

void TeachNode::srv_stop_playback(const std::shared_ptr<Trigger::Request>,
                                  std::shared_ptr<Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (phase_ != TeachStateMsg::PLAYING && phase_ != TeachStateMsg::PLAY_PAUSED) {
    res->success = false;
    res->message = std::string("当前状态 ") + phase_name(phase_) + "，没有正在回放的轨迹";
    return;
  }
  finish_playback(PlaybackStateMsg::STOPPED, "回放已停止（保持当前位置）", true);
  res->success = true;
  res->message = "已停止";
}

void TeachNode::srv_set_playback_speed(const std::shared_ptr<srv::SetPlaybackSpeed::Request> req,
                                       std::shared_ptr<srv::SetPlaybackSpeed::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  res->active_speed_scale = play_speed_;

  if (phase_ != TeachStateMsg::PLAYING && phase_ != TeachStateMsg::PLAY_PAUSED) {
    res->success = false;
    res->message = std::string("当前状态 ") + phase_name(phase_) + "，没有正在回放的轨迹";
    return;
  }

  double scale = req->speed_scale;
  if (normalize_speed_scale(&scale, p_.min_speed_scale, p_.max_speed_scale)) {
    RCLCPP_WARN(get_logger(), "speed_scale 已夹到 %.2f（允许区间 [%.2f, %.2f]）",
                scale, p_.min_speed_scale, p_.max_speed_scale);
  }

  // 新倍率同样要过速度/加速度闸；不过就保持原倍率（不静默限幅 —— 用户以为提速成功了，
  // 实际被悄悄削回去，是最难解释的那种行为）
  MotionCaps caps{p_.max_joint_velocity, p_.max_joint_acceleration};
  if (auto r = validate_speed(play_traj_, caps, scale); !r) {
    res->success = false;
    res->message = "该倍率会超限（" + r.reason + "）：" + r.detail + "；已保持原倍率 " +
                   std::to_string(play_speed_);
    return;
  }

  play_speed_ = scale;
  res->success            = true;
  res->active_speed_scale = scale;
  res->message = "倍率已改为 " + std::to_string(scale) +
                 "（当前分段走完后生效，约 " + std::to_string(p_.chunk_horizon_sec) +
                 "s 内；要立刻生效就先 pause 再 resume）";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

}  // namespace robot_arm_teach
