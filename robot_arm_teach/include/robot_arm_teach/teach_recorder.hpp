/**
 * @file teach_recorder.hpp
 * @brief 示教采样器 —— 时间轴 / 暂停 / 位置阈值压缩 / 包络统计（纯逻辑，无 ROS 依赖）
 *
 * 职责边界：本类**不订阅任何话题、不起定时器**。节点在 steady_clock 定时器里把最新的
 * 6 轴回读喂进 sample()，时间由调用方以 steady 秒数注入 —— 时间源作为参数而不是内部
 * 直接取，是为了让单元测试能把 200 秒的录制过程在几微秒内跑完。
 *
 * ★ 为什么时间一律用 steady_clock（而不是 node->now()）
 *   Gazebo 的 /clock 只有 10Hz，而采样是 wall timer：用 ROS 时钟求 dt 的话九成的 tick
 *   看到 dt=0、第十拍看到 0.1s，时间轴会系统性失真（robot_arm_node 的速度流积分踩过
 *   同一个坑，见 velocity_stream_server.cpp 的注释）。示教轨迹的时间轴一旦失真，
 *   回放速度就整体错了，而且错得很隐蔽 —— 位置全对，只是快慢不对。
 *
 * ★ 压缩规则（requirement：位置阈值压缩，但不能破坏时间顺序和速度限制）
 *   丢点只丢「与上一个保留点几乎重合」的样本，因此：
 *     · 时间顺序：只做子集选取，从不重排、不改 time_from_start → 单调性天然保持。
 *     · 速度限制：两个保留点之间的平均速度 = 被丢掉那一段的速度平均值，必然不超过
 *       该段内的峰值 → 压缩只可能让速度包络变小，不可能变大。
 *     · 形状：三条例外必须保留，否则会削掉拐角 ——
 *         ① 任一轴速度**变号**（运动方向反转，正是轨迹的拐点）
 *         ② 距上一个保留点超过 compress_max_gap_sec（长时间静止也要留锚点，
 *            否则会出现一个跨越十几秒的长弦，JTC 插值时把静止段变成缓慢漂移）
 *         ③ 首点与末点恒保留（起点/终点是回放前置校验的依据，不能被优化掉）
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot_arm_teach/msg/teach_trajectory.hpp"
#include "robot_arm_teach/teach_types.hpp"

namespace robot_arm_teach
{

class TeachRecorder
{
public:
  struct Config
  {
    // 名义采样频率（Hz）。建议 50 或 100：50Hz 与既有速度流下发频率同阶，
    // 100Hz 更细但文件大一倍且带进更多回读噪声。
    double sample_rate_hz{50.0};
    // 位置阈值（rad）。0 = 不压缩。0.002rad ≈ 0.11°，小于实机回读噪声与静差量级。
    double compress_position_eps{0.002};
    // 保留点之间的最大时间间隔（s）。见类注释里压缩规则的例外②。
    double compress_max_gap_sec{0.5};
    // 单条轨迹时长上限（s）。到点自动停止录制并告警 —— 忘了按 stop_teach 的话，
    // 内存会一直涨（50Hz × 6 轴 × 若干小时）。
    double max_duration_sec{600.0};
    // 原始采样点数上限，与 max_duration_sec 双保险（频率被调高时时长上限可能先不触发）。
    size_t max_points{120000};
  };

  // sample() 的结论。节点据此决定是否告警/自动收尾。
  enum class SampleOutcome
  {
    Kept,              // 已作为保留点写入
    Compressed,        // 有效样本，但被位置阈值压掉（仍计入 raw_point_count）
    NotRecording,      // 当前不在录制态，整帧丢弃
    Paused,            // 录制暂停中，整帧丢弃（时间轴同时冻结）
    TooSoon,           // 未到采样间隔（定时器抖动导致的重复触发）
    Invalid,           // 含 NaN / inf，整帧丢弃（绝不让坏数据进轨迹）
    StoppedFull,       // 已达 max_points，本帧未写入，录制已自动停止
    StoppedTimeout,    // 已达 max_duration_sec，本帧未写入，录制已自动停止
  };

  // 注：默认构造与单参构造分开声明（而不是给默认参数 `Config cfg = Config{}`），
  // 是绕开 GCC 对「嵌套类的默认成员初始值被用作外层类默认参数」的已知 bug
  // （GCC PR 88300 类似场景，11.4 上实测触发 "default member initializer ...
  // required before the end of its enclosing class"）。
  TeachRecorder();
  explicit TeachRecorder(Config cfg);

  void set_config(const Config & cfg);
  const Config & config() const { return cfg_; }

  // ── 状态机 ────────────────────────────────────────────────────────────────
  // 开始录制。重复调用会丢弃上一次未 finish 的数据（节点侧已有 IDLE 闸，这里只兜底）。
  void start(const std::string & name, uint8_t motion_type, uint8_t teach_mode,
             uint8_t control_mode, const std::string & description, double steady_now);
  // 暂停：冻结时间轴。已在暂停态时返回 false（幂等但如实上报）。
  bool pause(double steady_now);
  // 继续：把暂停时长累加进偏移，使 time_from_start 不出现空洞。
  bool resume(double steady_now);
  // 丢弃当前录制（不产出轨迹）
  void abort();

  bool recording() const { return recording_; }
  bool paused() const { return paused_; }
  // 因超限自动停止过（供节点决定是否要在状态里挂一条告警）
  bool auto_stopped() const { return auto_stopped_; }

  // ── 采样 ──────────────────────────────────────────────────────────────────
  // velocity_valid=false 时（/joint_states 没带 velocity 字段，或云台是开环回显）
  // 速度由相邻**原始**样本的位置差分补齐 —— 不能直接记 0，否则包络统计全 0，
  // 回放的速度闸就形同虚设。
  SampleOutcome sample(double steady_now, const JointArray & positions,
                       const JointArray & velocities, bool velocity_valid);

  // ── 收尾 ──────────────────────────────────────────────────────────────────
  // 补上末点（若末次样本被压缩掉了）→ 统计包络 → 填齐元数据。
  // 点数 < 2 时返回 false（一条只有一个点的轨迹回放起来毫无意义，不如明确失败）。
  bool finish(double steady_now, TeachTrajectoryMsg * out);

  // ── 运行期查询（供状态广播）──────────────────────────────────────────────
  double elapsed_sec(double steady_now) const;
  size_t raw_point_count() const { return raw_count_; }
  size_t kept_point_count() const { return points_.size(); }
  const std::string & name() const { return name_; }
  uint8_t motion_type() const { return motion_type_; }
  uint8_t teach_mode() const { return teach_mode_; }

private:
  // 是否必须保留这一帧（见类注释的压缩规则）
  bool must_keep(double t, const JointArray & pos, const JointArray & vel) const;

  Config cfg_{};

  bool recording_{false};
  bool paused_{false};
  bool auto_stopped_{false};

  std::string name_;
  std::string description_;
  uint8_t motion_type_{0};
  uint8_t teach_mode_{0};
  uint8_t control_mode_{0};
  std::string created_at_;

  double t0_{0.0};              // 录制起点（steady 秒）
  double paused_accum_{0.0};    // 累计暂停时长（s），从时间轴里扣掉
  double pause_start_{0.0};     // 本次暂停开始的 steady 时刻

  // 上一帧**原始**样本（不管是否被压缩掉）：采样间隔闸与速度差分都以它为基准
  double     last_sample_t_{0.0};
  bool       has_last_{false};
  JointArray last_raw_pos_{};

  // 最近一帧有效样本。末次采样若被压缩掉，finish() 用它补末点 ——
  // 否则轨迹终点会停在压缩前的某个中间位置，与机械臂实际停的地方不符。
  bool                                has_pending_{false};
  robot_arm_teach::msg::TeachPoint     pending_{};

  size_t raw_count_{0};
  std::vector<robot_arm_teach::msg::TeachPoint> points_;   // 保留点
};

}  // namespace robot_arm_teach
