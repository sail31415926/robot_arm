/**
 * @file teach_recorder.cpp
 * @brief TeachRecorder 实现 —— 时间轴 / 暂停 / 阈值压缩 / 末点补齐
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/teach_recorder.hpp"

#include <algorithm>
#include <cmath>

#include "robot_arm_teach/msg/motion_type.hpp"
#include "robot_arm_teach/teach_validator.hpp"

namespace robot_arm_teach
{

namespace
{
using TeachPointMsg = robot_arm_teach::msg::TeachPoint;

// 采样间隔闸的容差系数。定时器抖动会让实际间隔略小于名义周期，按 0.9 倍放行，
// 否则每隔几拍就会被判 TooSoon，实际采样率降到名义值的一半左右。
constexpr double PERIOD_TOLERANCE = 0.9;
// 速度变号的判定死区（rad/s）。回读噪声会让静止的轴在 0 附近来回变号，
// 不设死区的话「方向反转」这条例外会保留几乎所有点，压缩失效。
constexpr double SIGN_FLIP_EPS = 0.01;

bool finite_all(const JointArray & a)
{
  return std::all_of(a.begin(), a.end(), [](double v) { return std::isfinite(v); });
}

int sign_with_deadband(double v)
{
  if (v > SIGN_FLIP_EPS) return 1;
  if (v < -SIGN_FLIP_EPS) return -1;
  return 0;
}
}  // namespace

TeachRecorder::TeachRecorder()
: TeachRecorder(Config{})
{
}

TeachRecorder::TeachRecorder(Config cfg)
: cfg_(cfg)
{
}

void TeachRecorder::set_config(const Config & cfg)
{
  cfg_ = cfg;
  if (cfg_.sample_rate_hz < 1.0) cfg_.sample_rate_hz = 1.0;
}

void TeachRecorder::start(const std::string & name, uint8_t motion_type, uint8_t teach_mode,
                          uint8_t control_mode, const std::string & description,
                          double steady_now)
{
  name_        = name;
  description_ = description;
  motion_type_ = motion_type;
  teach_mode_  = teach_mode;
  control_mode_ = control_mode;
  created_at_  = iso8601_now();

  t0_            = steady_now;
  paused_accum_  = 0.0;
  pause_start_   = 0.0;
  last_sample_t_ = 0.0;
  has_last_      = false;
  has_pending_   = false;
  raw_count_     = 0;
  points_.clear();

  recording_    = true;
  paused_       = false;
  auto_stopped_ = false;
}

bool TeachRecorder::pause(double steady_now)
{
  if (!recording_ || paused_) return false;
  paused_      = true;
  pause_start_ = steady_now;
  return true;
}

bool TeachRecorder::resume(double steady_now)
{
  if (!recording_ || !paused_) return false;
  // 把暂停这段墙钟时间从轨迹时间轴上扣掉：否则回放时会在暂停点原地"等"同样长的时间
  // （JTC 会照着 time_from_start 的空洞插值，表现为一段极慢的漂移，不是静止）。
  paused_accum_ += std::max(0.0, steady_now - pause_start_);
  paused_ = false;
  return true;
}

void TeachRecorder::abort()
{
  recording_ = false;
  paused_    = false;
  points_.clear();
  has_pending_ = false;
  has_last_    = false;
  raw_count_   = 0;
}

double TeachRecorder::elapsed_sec(double steady_now) const
{
  if (!recording_) {
    return points_.empty() ? 0.0 : points_.back().time_from_start;
  }
  // 暂停期间时间轴冻结在暂停开始那一刻
  const double wall = paused_ ? pause_start_ : steady_now;
  return std::max(0.0, wall - t0_ - paused_accum_);
}

bool TeachRecorder::must_keep(double t, const JointArray & pos, const JointArray & vel) const
{
  if (points_.empty()) return true;                    // 例外3：首点恒保留
  if (cfg_.compress_position_eps <= 0.0) return true;  // 关闭压缩

  const auto & last = points_.back();
  if (t - last.time_from_start >= cfg_.compress_max_gap_sec) return true;   // 例外2：锚点

  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    // 例外1：任一轴速度变号 = 运动方向反转 = 轨迹拐点，压掉就削角
    if (sign_with_deadband(vel[j]) != 0 &&
        sign_with_deadband(last.velocities[j]) != 0 &&
        sign_with_deadband(vel[j]) != sign_with_deadband(last.velocities[j])) {
      return true;
    }
    if (std::fabs(pos[j] - last.positions[j]) >= cfg_.compress_position_eps) return true;
  }
  return false;
}

TeachRecorder::SampleOutcome TeachRecorder::sample(double steady_now, const JointArray & positions,
                                                   const JointArray & velocities,
                                                   bool velocity_valid)
{
  if (!recording_) return SampleOutcome::NotRecording;
  if (paused_)     return SampleOutcome::Paused;

  // 坏数据一律整帧丢弃。让 NaN 进轨迹的后果是：回放前置校验能拦住（闸②），
  // 但那时用户已经录完了，白录一遍；当场丢弃并让点数不涨，用户马上看得出不对。
  if (!finite_all(positions) || !finite_all(velocities)) return SampleOutcome::Invalid;

  const double t = std::max(0.0, steady_now - t0_ - paused_accum_);

  // 采样间隔闸（steady_clock 求间隔，不用 ROS 时钟 —— 见头文件注释）
  const double period = 1.0 / std::max(1.0, cfg_.sample_rate_hz);
  if (has_last_ && (t - last_sample_t_) < period * PERIOD_TOLERANCE) {
    return SampleOutcome::TooSoon;
  }

  // 超限自动停止：先判再写，保证 max_points / max_duration_sec 是硬上界
  if (points_.size() >= cfg_.max_points) {
    recording_ = false; auto_stopped_ = true;
    return SampleOutcome::StoppedFull;
  }
  if (cfg_.max_duration_sec > 0.0 && t > cfg_.max_duration_sec) {
    recording_ = false; auto_stopped_ = true;
    return SampleOutcome::StoppedTimeout;
  }

  // 速度：回读没给就用相邻原始样本的位置差分补。直接记 0 会让包络统计全 0，
  // 回放的速度闸（闸4）就形同虚设 —— 而缺速度字段的后端恰恰最需要那道闸。
  JointArray vel = velocities;
  if (!velocity_valid) {
    if (has_last_ && t > last_sample_t_) {
      const double dt = t - last_sample_t_;
      for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
        vel[j] = (positions[j] - last_raw_pos_[j]) / dt;
      }
    } else {
      vel.fill(0.0);   // 首帧无从差分
    }
  }

  TeachPointMsg pt;
  pt.positions      = positions;
  pt.velocities     = vel;
  pt.time_from_start = t;

  const bool keep = must_keep(t, positions, vel);

  last_sample_t_ = t;
  last_raw_pos_  = positions;
  has_last_      = true;
  ++raw_count_;
  pending_       = pt;
  has_pending_   = true;

  if (keep) {
    points_.push_back(pt);
    return SampleOutcome::Kept;
  }
  return SampleOutcome::Compressed;
}

bool TeachRecorder::finish(double steady_now, TeachTrajectoryMsg * out)
{
  // 时长以末点的 time_from_start 为准，不用墙钟 —— 停止请求到达与末次采样之间的
  // 那点空档（最多一个采样周期）不该算进轨迹时长，否则回放末尾会多一段无点的等待。
  (void)steady_now;
  recording_ = false;
  paused_    = false;

  // 末次样本被压缩掉时补上：否则轨迹终点停在某个中间位置，与机械臂实际停的地方不符，
  // 而 end_positions 又是回放前置校验和上层检索的依据。
  if (has_pending_ && !points_.empty() &&
      pending_.time_from_start > points_.back().time_from_start + 1e-6) {
    points_.push_back(pending_);
  }

  if (!out) return false;
  // 一个点的轨迹回放起来毫无意义（JTC 收到单点会当作"去这个位置"），
  // 不如明确失败让用户重录。
  if (points_.size() < 2) return false;

  TeachTrajectoryMsg traj;
  traj.format_version = TeachTrajectoryMsg::FORMAT_VERSION;
  traj.name           = name_;
  traj.created_at     = created_at_;
  traj.description    = description_;
  traj.joint_names    = teach_joint_names();
  traj.points         = points_;
  traj.motion_type.value = motion_type_;
  // motion_meta 留默认值：录制时拿不到运镜参数（那是上层的意图，不是实测量），
  // 由 save_trajectory 的 meta_override 补，或由上层自己填。
  traj.start_positions = points_.front().positions;
  traj.end_positions   = points_.back().positions;

  const Envelope env = compute_envelope(traj);
  traj.max_velocity     = env.max_velocity;
  traj.max_acceleration = env.max_acceleration;
  traj.duration_sec     = points_.back().time_from_start;

  traj.teach_control_mode.mode = control_mode_;
  traj.teach_mode              = teach_mode_;
  traj.sample_rate_hz          = cfg_.sample_rate_hz;
  traj.raw_point_count         = static_cast<uint32_t>(raw_count_);
  traj.compress_position_eps   = cfg_.compress_position_eps;

  *out = std::move(traj);
  return true;
}

}  // namespace robot_arm_teach
