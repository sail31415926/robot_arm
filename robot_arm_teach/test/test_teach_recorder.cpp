/**
 * @file test_teach_recorder.cpp
 * @brief TeachRecorder 单元测试 —— 时间轴 / 暂停 / 阈值压缩 / 末点补齐
 *
 * 时间由测试注入（TeachRecorder 不自己取时钟），所以一段 700 秒的录制在这里几微秒跑完。
 * 这也是为什么采样器被做成「不订阅、不起定时器」的纯逻辑类。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "robot_arm_teach/msg/motion_type.hpp"
#include "robot_arm_teach/msg/teach_state.hpp"
#include "robot_arm_teach/teach_recorder.hpp"
#include "robot_arm_teach/teach_validator.hpp"

using namespace robot_arm_teach;   // NOLINT(build/namespaces)
using Outcome    = TeachRecorder::SampleOutcome;
using MotionType = robot_arm_teach::msg::MotionType;
using TeachState = robot_arm_teach::msg::TeachState;

namespace
{
JointArray pos_all(double v)
{
  JointArray a;
  a.fill(v);
  return a;
}

TeachRecorder::Config cfg(double eps, double gap = 1e9)
{
  TeachRecorder::Config c;
  c.sample_rate_hz        = 50.0;
  c.compress_position_eps = eps;
  c.compress_max_gap_sec  = gap;   // 默认给个天文数字，好把「位置阈值」这条规则单独测出来
  c.max_duration_sec      = 600.0;
  c.max_points            = 120000;
  return c;
}

void start(TeachRecorder & r, double t0 = 100.0)
{
  r.start("unit", MotionType::DOLLY, TeachState::TEACH_MODE_JOG, 1, "", t0);
}
}  // namespace

// ── 基本录制 ────────────────────────────────────────────────────────────────
TEST(TeachRecorder, RecordsMonotonicTrajectoryThatPassesValidation)
{
  TeachRecorder r(cfg(0.0));   // 关闭压缩
  start(r);
  for (int i = 0; i < 10; ++i) {
    const double t = 100.0 + i * 0.02;
    EXPECT_EQ(r.sample(t, pos_all(i * 0.01), pos_all(0.5), true), Outcome::Kept) << i;
  }
  TeachTrajectoryMsg traj;
  ASSERT_TRUE(r.finish(100.2, &traj));
  EXPECT_EQ(traj.points.size(), 10u);
  EXPECT_EQ(traj.format_version, TeachTrajectoryMsg::FORMAT_VERSION);
  EXPECT_EQ(traj.joint_names, teach_joint_names());
  EXPECT_EQ(traj.motion_type.value, MotionType::DOLLY);
  EXPECT_EQ(traj.teach_mode, TeachState::TEACH_MODE_JOG);
  EXPECT_NEAR(traj.points.front().time_from_start, 0.0, 1e-12);   // 时间轴从 0 起
  EXPECT_NEAR(traj.duration_sec, 0.18, 1e-9);
  EXPECT_NEAR(traj.start_positions[0], 0.0, 1e-12);
  EXPECT_NEAR(traj.end_positions[0], 0.09, 1e-9);
  // 录出来的东西必须能过回放前置校验，否则录了也回放不了
  EXPECT_TRUE(validate_structure(traj, teach_joint_names()).ok);
}

TEST(TeachRecorder, RejectsSamplesWhenNotRecordingOrPaused)
{
  TeachRecorder r(cfg(0.0));
  EXPECT_EQ(r.sample(1.0, pos_all(0.0), pos_all(0.0), true), Outcome::NotRecording);
  start(r);
  ASSERT_TRUE(r.pause(100.0));
  EXPECT_EQ(r.sample(100.02, pos_all(0.0), pos_all(0.0), true), Outcome::Paused);
}

// ── 采样间隔闸 ──────────────────────────────────────────────────────────────
TEST(TeachRecorder, RateGateDropsTooSoonSamples)
{
  TeachRecorder r(cfg(0.0));   // 50Hz → 周期 0.02s，容差 0.9 倍 → 0.018s
  start(r);
  EXPECT_EQ(r.sample(100.000, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.005, pos_all(0.0), pos_all(0.0), true), Outcome::TooSoon);
  EXPECT_EQ(r.sample(100.019, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
}

// ── 暂停：时间轴冻结，不留空洞 ──────────────────────────────────────────────
TEST(TeachRecorder, PauseFreezesTimeAxis)
{
  TeachRecorder r(cfg(0.0));
  start(r);
  ASSERT_EQ(r.sample(100.00, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
  ASSERT_EQ(r.sample(100.02, pos_all(0.1), pos_all(0.0), true), Outcome::Kept);

  ASSERT_TRUE(r.pause(100.03));
  // 暂停期间 elapsed 冻结（不管墙钟走了多久）
  EXPECT_NEAR(r.elapsed_sec(130.0), 0.03, 1e-9);
  ASSERT_TRUE(r.resume(130.03));   // 暂停了整整 30 秒

  ASSERT_EQ(r.sample(130.05, pos_all(0.2), pos_all(0.0), true), Outcome::Kept);

  TeachTrajectoryMsg traj;
  ASSERT_TRUE(r.finish(130.06, &traj));
  ASSERT_EQ(traj.points.size(), 3u);
  // 第三个点的时间戳应当是 0.05 而不是 30.05 —— 那 30 秒被扣掉了。
  // 不扣的话，回放时 JTC 会照着时间轴的空洞插值，把"暂停"变成一段 30 秒的缓慢漂移。
  EXPECT_NEAR(traj.points[2].time_from_start, 0.05, 1e-9);
  EXPECT_TRUE(validate_structure(traj, teach_joint_names()).ok);
}

TEST(TeachRecorder, PauseResumeAreNotIdempotentSilently)
{
  TeachRecorder r(cfg(0.0));
  start(r);
  EXPECT_TRUE(r.pause(100.1));
  EXPECT_FALSE(r.pause(100.2));    // 已在暂停态，如实返回 false
  EXPECT_TRUE(r.resume(100.3));
  EXPECT_FALSE(r.resume(100.4));
}

// ── 位置阈值压缩 ────────────────────────────────────────────────────────────
TEST(TeachRecorder, CompressionDropsNearIdenticalSamples)
{
  TeachRecorder r(cfg(0.01));   // 阈值 0.01rad
  start(r);
  ASSERT_EQ(r.sample(100.00, pos_all(0.000), pos_all(0.0), true), Outcome::Kept);
  // 位移远小于阈值 + 速度都在死区内 → 压掉
  EXPECT_EQ(r.sample(100.02, pos_all(0.001), pos_all(0.0), true), Outcome::Compressed);
  EXPECT_EQ(r.sample(100.04, pos_all(0.002), pos_all(0.0), true), Outcome::Compressed);
  // 累计超过阈值 → 保留
  EXPECT_EQ(r.sample(100.06, pos_all(0.020), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.raw_point_count(), 4u);
  EXPECT_EQ(r.kept_point_count(), 2u);
}

// 压缩只做子集选取 → 时间顺序不可能被破坏，速度包络只会变小不会变大
TEST(TeachRecorder, CompressionNeverIncreasesVelocityEnvelope)
{
  TeachTrajectoryMsg dense, sparse;
  {
    TeachRecorder r(cfg(0.0));   // 不压缩
    start(r);
    for (int i = 0; i < 60; ++i) {
      r.sample(100.0 + i * 0.02, pos_all(std::sin(i * 0.05) * 0.05), pos_all(0.0), false);
    }
    ASSERT_TRUE(r.finish(101.2, &dense));
  }
  {
    TeachRecorder r(cfg(0.01));   // 压缩
    start(r);
    for (int i = 0; i < 60; ++i) {
      r.sample(100.0 + i * 0.02, pos_all(std::sin(i * 0.05) * 0.05), pos_all(0.0), false);
    }
    ASSERT_TRUE(r.finish(101.2, &sparse));
  }
  EXPECT_LT(sparse.points.size(), dense.points.size());
  EXPECT_TRUE(validate_structure(sparse, teach_joint_names()).ok);   // 单调性保持

  const auto ed = compute_envelope(dense);
  const auto es = compute_envelope(sparse);
  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    // 两保留点之间的平均速度 = 被丢掉那段的速度平均值，必然不超过该段峰值
    EXPECT_LE(es.max_velocity[j], ed.max_velocity[j] + 1e-9) << j;
  }
}

// 拐角必须保留：任一轴速度变号 = 运动方向反转，压掉就削角
TEST(TeachRecorder, CompressionKeepsDirectionReversal)
{
  TeachRecorder r(cfg(1e9));   // 阈值大到「只按位置」永远不会保留
  start(r);
  ASSERT_EQ(r.sample(100.00, pos_all(0.0), pos_all(+0.5), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.02, pos_all(0.0), pos_all(+0.5), true), Outcome::Compressed);
  // 速度由 +0.5 变 −0.5 → 拐点，必须保留
  EXPECT_EQ(r.sample(100.04, pos_all(0.0), pos_all(-0.5), true), Outcome::Kept);
}

// 长时间静止也要留锚点，否则会出现跨十几秒的长弦
TEST(TeachRecorder, CompressionKeepsAnchorAfterMaxGap)
{
  TeachRecorder r(cfg(1e9, /*gap=*/0.5));
  start(r);
  ASSERT_EQ(r.sample(100.00, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.20, pos_all(0.0), pos_all(0.0), true), Outcome::Compressed);
  EXPECT_EQ(r.sample(100.60, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);   // 超过 0.5s
}

// 末次样本被压缩掉时，finish 必须补上 —— 否则轨迹终点与机械臂实际停的地方不符，
// 而 end_positions 又是前置校验和上层检索的依据
TEST(TeachRecorder, FinishAppendsCompressedLastSample)
{
  TeachRecorder r(cfg(0.01));
  start(r);
  ASSERT_EQ(r.sample(100.00, pos_all(0.000), pos_all(0.0), true), Outcome::Kept);
  ASSERT_EQ(r.sample(100.02, pos_all(0.050), pos_all(0.0), true), Outcome::Kept);
  ASSERT_EQ(r.sample(100.04, pos_all(0.051), pos_all(0.0), true), Outcome::Compressed);

  TeachTrajectoryMsg traj;
  ASSERT_TRUE(r.finish(100.05, &traj));
  ASSERT_EQ(traj.points.size(), 3u);
  EXPECT_NEAR(traj.end_positions[0], 0.051, 1e-9);
  EXPECT_NEAR(traj.points.back().time_from_start, 0.04, 1e-9);
}

// ── 坏数据 ──────────────────────────────────────────────────────────────────
TEST(TeachRecorder, DropsNonFiniteSamplesEntirely)
{
  TeachRecorder r(cfg(0.0));
  start(r);
  auto bad = pos_all(0.0);
  bad[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(r.sample(100.00, bad, pos_all(0.0), true), Outcome::Invalid);
  EXPECT_EQ(r.kept_point_count(), 0u);
  EXPECT_EQ(r.raw_point_count(), 0u);   // 坏帧连原始计数都不进
}

// ── 速度补齐 ────────────────────────────────────────────────────────────────
// velocity_valid=false 时必须由位置差分补齐。记 0 的话包络统计全 0，
// 回放的速度闸就形同虚设 —— 而缺速度字段的后端恰恰最需要那道闸。
TEST(TeachRecorder, DerivesVelocityWhenJointStateHasNone)
{
  TeachRecorder r(cfg(0.0));
  start(r);
  r.sample(100.00, pos_all(0.00), pos_all(0.0), false);
  r.sample(100.02, pos_all(0.01), pos_all(0.0), false);   // 0.01 / 0.02 = 0.5 rad/s
  r.sample(100.04, pos_all(0.02), pos_all(0.0), false);

  TeachTrajectoryMsg traj;
  ASSERT_TRUE(r.finish(100.05, &traj));
  EXPECT_NEAR(traj.points[0].velocities[0], 0.0, 1e-12);   // 首帧无从差分
  EXPECT_NEAR(traj.points[1].velocities[0], 0.5, 1e-9);
  EXPECT_NEAR(traj.points[2].velocities[0], 0.5, 1e-9);
  EXPECT_NEAR(traj.max_velocity[0], 0.5, 1e-9);
}

// ── 超限自动停止 ────────────────────────────────────────────────────────────
TEST(TeachRecorder, StopsAtDurationLimit)
{
  auto c = cfg(0.0);
  c.max_duration_sec = 0.10;
  TeachRecorder r(c);
  start(r);
  EXPECT_EQ(r.sample(100.00, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.08, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.20, pos_all(0.0), pos_all(0.0), true), Outcome::StoppedTimeout);
  EXPECT_FALSE(r.recording());
  EXPECT_TRUE(r.auto_stopped());
}

TEST(TeachRecorder, StopsAtPointLimit)
{
  auto c = cfg(0.0);
  c.max_points = 2;
  TeachRecorder r(c);
  start(r);
  EXPECT_EQ(r.sample(100.00, pos_all(0.0), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.02, pos_all(0.1), pos_all(0.0), true), Outcome::Kept);
  EXPECT_EQ(r.sample(100.04, pos_all(0.2), pos_all(0.0), true), Outcome::StoppedFull);
  EXPECT_TRUE(r.auto_stopped());
}

// ── 收尾的退化情形 ──────────────────────────────────────────────────────────
TEST(TeachRecorder, FinishFailsWithFewerThanTwoPoints)
{
  TeachRecorder r(cfg(0.0));
  start(r);
  TeachTrajectoryMsg traj;
  EXPECT_FALSE(r.finish(100.1, &traj));      // 一个点都没采

  start(r);
  r.sample(100.0, pos_all(0.0), pos_all(0.0), true);
  EXPECT_FALSE(r.finish(100.1, &traj));      // 只有一个点，回放起来毫无意义
}

TEST(TeachRecorder, AbortDiscardsEverything)
{
  TeachRecorder r(cfg(0.0));
  start(r);
  r.sample(100.00, pos_all(0.0), pos_all(0.0), true);
  r.sample(100.02, pos_all(0.1), pos_all(0.0), true);
  r.abort();
  EXPECT_FALSE(r.recording());
  EXPECT_EQ(r.kept_point_count(), 0u);
}
