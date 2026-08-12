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
  // 下发频率。50Hz 是 2026-08-04 在仿真里定的值，保持不变。
  //
  // 【2026-08-12 实机抖动排查记录，改这里前先读】JTC 每收一条新单点轨迹就丢弃旧的、
  // 从**当前实际状态**重新插值，所以"发布周期 vs 计划时长(lookahead_)"的比例直接决定
  // 手感，而两端都不好：
  //   · 50Hz/20ms 周期 + 50ms 时长：每条计划只执行 40% 就被替换，且计划开头是样条最慢
  //     的一段 → 跟踪率仅 71%，设定点被迫领先实测 L≈0.097rad 才能维持速度，
  //     实测速度在 0～0.246 之间摆动（指令 0.15），标准差 0.048。
  //   · 20Hz（周期 == 时长）：每条计划走完，跟踪率回到 100.4%、L 降到 0.022rad，
  //     但每段样条自身的加减速形状暴露出来，纹波反而更大（标准差 0.117，
  //     −0.06～0.58 之间摆）。
  // 另外试过并否掉的两条：把前瞻基准改成实测位置（L 消失但臂几乎不动，0.0064rad/s
  //   —— L 正是驱动位置环产生速度的必要误差）；把时长按真实距离/q̇ 拉长到 1.0s
  //   （每周期只执行样条更小的比例，臂同样不动）。
  // 结论：抖动是"单点轨迹 + 每周期重规划"方案的固有问题，调常数只能在
  //   跟踪率和纹波之间挪，治本要换驱动器内部速度环（PV(3)）——
  //   即 mode_manager_node 的 velocity_backend:=velocity_controller。
  rate_hz_           = node_.declare_parameter("velocity_stream.rate_hz", 50.0);
  command_timeout_   = node_.declare_parameter("velocity_stream.command_timeout", 0.3);
  lookahead_         = node_.declare_parameter("velocity_stream.lookahead", 0.05);
  // 停止点的 time_from_start。比 lookahead_ 长一些，给 JTC/驱动器一段确定的减速区间；
  // 太短接近零时长阶跃，太长则松手后"软绵绵"地才停住。
  stop_time_         = node_.declare_parameter("velocity_stream.stop_time", 0.15);
  // 设定点允许领先实测位置的上限（rad）。这是**兜底闸**而非主修复 ——
  // 消除"松手猛冲"靠的是 halt() 里显式停在实测位置；本闸另外封住设定点被无界拉开的
  // 情况（关节顶限位、负载重跟不动、驱动器报警不动了：设定点还在积分，关节原地不动）。
  //
  // 定值要够宽，否则会把正常点动限速：稳态滞后 ≈ q̇ × 伺服时间常数，
  // 最高点动 1.0rad/s（max_joint_speed）× 约 0.1s ≈ 0.1rad，所以 0.10 正好卡边界。
  // 取 0.20rad 留一倍余量：正常点动碰不到它，异常时又有上界。0 = 不限（老行为）。
  max_lag_           = node_.declare_parameter("velocity_stream.max_lag", 0.20);
  // 关节加速度上限（rad/s²），用于把阶跃速度指令平滑成梯形加减速。
  // 2.0 时从 0 爬到 0.3rad/s 约 150ms。0 = 不限（老行为）。
  max_accel_         = node_.declare_parameter("velocity_stream.max_accel", 2.0);
  // 单点轨迹时长的上界（s）。见 on_tick 里"下发时长"一段：时长按 (lead−实测)/q̇ 算，
  // 关节被堵住时这个比值会爆掉，须封顶，否则 JTC 会以极慢的斜率磨。
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
    prev_qdot_.assign(all_joints_.size(), 0.0);   // 斜率限幅从 0 起爬，起步不顿
    seeded_      = true;
    last_tick_s_ = tick_now;
  }
  // 积分步长用 steady_clock：定时器是 wall timer，而 Gazebo 的 /clock 只有 10Hz，
  // 用 ROS 时钟取 dt 会系统性丢步（九成的 tick 看到 0、第十拍看到 0.1s 再被上限削掉）
  const double dt = std::clamp(tick_now - last_tick_s_, 0.0, 5.0 / rate_hz_);
  last_tick_s_ = tick_now;

  // 速度斜率限幅（= 加速度上限）。GUI 的按住/松手是**阶跃**速度指令，直接积分会让
  // JTC 每拍收到一条斜率突变的新轨迹，实机上表现为起停顿挫、点动过程抖动。
  // 限住 q̇ 的变化率相当于给指令加梯形加减速。0 = 不限（老行为）。
  if (prev_qdot_.size() != qdot.size()) prev_qdot_.assign(qdot.size(), 0.0);
  if (max_accel_ > 0.0) {
    // 限幅步长不能直接用 dt：播种那一拍 last_tick_s_ 刚被设成 tick_now，dt 恰好是 0，
    // 若此时跳过限幅，prev_qdot_ 会被整个阶跃值灌满，后面就没有可爬的斜坡了
    //（实测症状：第一拍就是满速 0.300，梯形加速完全不生效）。
    // 位置积分仍用真实 dt（那一拍确实没有时间流过，不该前进）。
    const double dv_max = max_accel_ * (dt > 0.0 ? dt : 1.0 / rate_hz_);
    for (size_t i = 0; i < qdot.size(); ++i) {
      qdot[i] = prev_qdot_[i] + std::clamp(qdot[i] - prev_qdot_[i], -dv_max, dv_max);
    }
  }
  prev_qdot_ = qdot;

  // 开环积分的防跑飞闸：不让设定点领先实测位置超过 max_lag_。
  // 实机上关节位置滞后于设定点（IP 模式 + 伺服动态），而 target_ 只在起步时播种、
  // 流期间从不与回读对齐 —— 滞后量会一路累积。松手时那段累积差就是"猛冲"的幅度；
  // 关节顶到限位或负载过大跟不动时更明显（设定点继续走，关节原地不动）。
  //
  // ★ 实现方式很关键：**只"冻结积分"，绝不把实测位置写进 target_**。
  //   曾经写成 target_ = clamp(target_, meas ± max_lag)，实机上抖得很厉害 ——
  //   一旦闸生效，位置指令就成了实测位置的函数：① 编码器噪声/量化直接进指令；
  //   ② 构成 cmd = meas + max_lag → 伺服追 → meas 上升 → cmd 上升 的闭环，
  //   回路增益约 1 且含伺服滞后，临界稳定 → 自激振动。
  //   现在的做法只是拒绝把差距**继续拉大**，指令仍是纯开环积分量，噪声进不来。
  std::vector<double> meas;
  const bool have_meas = max_lag_ > 0.0 && status_.has_joint_positions(all_joints_);
  if (have_meas) meas = status_.joint_position_list(all_joints_);

  std::vector<double> lead(all_joints_.size());
  for (size_t i = 0; i < all_joints_.size(); ++i) {
    const double step = qdot[i] * dt;
    target_[i] += step;
    if (have_meas) {
      const double lag = target_[i] - meas[i];       // 设定点领先量（带符号）
      // 只在「差距已超限、且本拍还在朝拉大方向走」时撤销这一步；
      // 反向（把差距缩小、或把关节从限位拉回来）永不受限。
      if (std::fabs(lag) > max_lag_ && (lag > 0.0) == (step > 0.0) && step != 0.0) {
        target_[i] -= step;
        RCLCPP_WARN_THROTTLE(logger_, *node_.get_clock(), 2000,
            "'%s' 设定点已领先实测 %.3f rad（上限 %.3f）—— 暂停积分。"
            "关节可能顶到限位/负载过重跟不动，或 max_lag 设得太小",
            all_joints_[i].c_str(), lag, max_lag_);
      }
    }
    // 下发点 = 设定点再前伸 lookahead。时长不再固定用 lookahead_，见下方 traj_time。
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
                                              const std::vector<double> & velocities,
                                              double duration_s)
{
  JointTrajectory traj;
  traj.joint_names = all_joints_;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions  = positions;
  pt.velocities = velocities;
  // duration_s <= 0 时用 lookahead_（流期间的常规点）；停止点单独给一个更长的
  // stop_time_，让 JTC 有一段确定的减速区间，而不是零时长阶跃。
  pt.time_from_start = rclcpp::Duration::from_seconds(duration_s > 0.0 ? duration_s : lookahead_);
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

  // ★ 必须显式下发一条「停在这里」的轨迹，不能靠 JTC 自己保持最后一点。
  //
  // 为什么（实机 2026-08-12 实测：松手瞬间电机猛冲）：流期间最后发出去的那一点是
  // **前伸点** lead = target_ + q̇·lookahead，且 velocities = q̇（非零）。什么都不发的话，
  // JTC 保持的就是这个「比设定点还靠前、末端速度非零」的目标 —— 松手后伺服继续朝它冲。
  // 更要紧的是 target_ 是**开环积分**的：实机上关节位置滞后于设定点（CANopen IP +
  // 伺服动态），滞后量随点动时长累积，于是那个悬空目标离实际位置可能很远，松手瞬间
  // 变成阶跃指令 → 全速补齐 = 猛冲。
  // Gazebo 看不到这个问题：位置接口是 SetPosition（运动学瞬移），实际位置恒等于设定点，
  // 差值只有 q̇·lookahead 且瞬移完成 —— 又一个"仿真结论不能外推到实物"的例子。
  //
  // 停止点取**实测位置**而不是 target_：target_ 含累积滞后，拿它当目标仍会往前走一段。
  if (status_.has_joint_positions(all_joints_)) {
    const auto cur = status_.joint_position_list(all_joints_);
    publish_trajectory(std::vector<double>(cur.begin(), cur.end()),
                       std::vector<double>(all_joints_.size(), 0.0),
                       stop_time_);
    // 设定点也拉回实测，避免下一次起步前这段差值以别的路径漏出去
    target_.assign(cur.begin(), cur.end());
    RCLCPP_INFO(logger_, "速度流停止（%s），已下发「停在实测位置」轨迹（%.2fs，零终端速度）",
                reason, stop_time_);
  } else {
    // 没有回读：joint_position_list() 会把缺失关节记成 0，拿它下发等于命令臂摆到零位。
    // 退化为发「设定点」（至少去掉 lookahead 前伸和非零终端速度）。
    publish_trajectory(target_, std::vector<double>(all_joints_.size(), 0.0), stop_time_);
    RCLCPP_WARN(logger_, "速度流停止（%s），但缺 /joint_states 回读 —— "
                         "退化为按设定点停（可能残留跟踪误差）", reason);
  }
  status_.set_moving(false);
}

}  // namespace robot_arm_node::commander
