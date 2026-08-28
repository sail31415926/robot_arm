/**
 * @file test_playback_planner.cpp
 * @brief PlaybackPlanner 单元测试 —— 分段 / 倍率缩放 / 保持轨迹 / 接近段
 *
 * 重点测三件在实机上会出事的性质：
 *   ① 分段永远推进（稀疏轨迹不能让流式下发卡住）
 *   ② 段内首点 time_from_start 永远 > 0（零时长段 = JTC 除零 / 无限加速度）
 *   ③ make_hold 永远产出一个点，绝不产出空轨迹（空轨迹不会让 JTC 停下）
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include <cmath>

#include <gtest/gtest.h>

#include "robot_arm_teach/playback_planner.hpp"

using namespace robot_arm_teach;   // NOLINT(build/namespaces)

namespace
{
// n 点、间隔 dt 的轨迹；第 j 轴第 i 点位置 = i*0.01 + j，速度恒 1.0
TeachTrajectoryMsg make_traj(size_t n = 11, double dt = 0.1)
{
  TeachTrajectoryMsg t;
  t.format_version = TeachTrajectoryMsg::FORMAT_VERSION;
  t.name           = "planner_test";
  t.joint_names    = teach_joint_names();
  for (size_t i = 0; i < n; ++i) {
    robot_arm_teach::msg::TeachPoint p;
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      p.positions[j]  = static_cast<double>(i) * 0.01 + static_cast<double>(j);
      p.velocities[j] = 1.0;
    }
    p.time_from_start = static_cast<double>(i) * dt;
    t.points.push_back(p);
  }
  t.start_positions = t.points.front().positions;
  t.end_positions   = t.points.back().positions;
  t.duration_sec    = t.points.back().time_from_start;
  return t;
}

double secs(const trajectory_msgs::msg::JointTrajectoryPoint & p)
{
  return static_cast<double>(p.time_from_start.sec) +
         static_cast<double>(p.time_from_start.nanosec) * 1e-9;
}

PlaybackPlanner::Config cfg(double horizon = 0.5)
{
  PlaybackPlanner::Config c;
  c.chunk_horizon_sec     = horizon;
  c.min_point_dt          = 0.005;
  c.approach_duration_sec = 3.0;
  c.hold_duration_sec     = 0.2;
  return c;
}
}  // namespace

// ── 分段基本行为 ────────────────────────────────────────────────────────────
TEST(PlaybackPlanner, FirstChunkCoversHorizonAndStartsInTheFuture)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto traj = make_traj();          // 0.0 .. 1.0s，每 0.1s 一点

  const auto c = pl.make_chunk(traj, 0.0, 1.0);
  ASSERT_EQ(c.trajectory.joint_names, teach_joint_names());
  // (0, 0.5] → t = 0.1 .. 0.5，共 5 点。t=0 那点不进段：零时长段会让 JTC 除零
  ASSERT_EQ(c.trajectory.points.size(), 5u);
  EXPECT_NEAR(secs(c.trajectory.points.front()), 0.1, 1e-6);
  EXPECT_NEAR(secs(c.trajectory.points.back()), 0.5, 1e-6);
  EXPECT_NEAR(c.next_phase, 0.5, 1e-9);
  EXPECT_FALSE(c.finished);
  // header.stamp 留 0 = JTC 约定「立即开始」，不盖节点时钟（避免跨进程时钟偏差）
  EXPECT_EQ(c.trajectory.header.stamp.sec, 0);
  EXPECT_EQ(c.trajectory.header.stamp.nanosec, 0u);
}

TEST(PlaybackPlanner, ChunkPointsAreStrictlyIncreasingAndPositive)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto c = pl.make_chunk(make_traj(), 0.0, 1.0);
  double prev = 0.0;
  for (const auto & p : c.trajectory.points) {
    const double s = secs(p);
    EXPECT_GT(s, prev);
    prev = s;
  }
  EXPECT_GT(secs(c.trajectory.points.front()), 0.0);
}

TEST(PlaybackPlanner, SubsequentChunkContinuesFromPhase)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto traj = make_traj();
  const auto c = pl.make_chunk(traj, 0.5, 1.0);
  ASSERT_EQ(c.trajectory.points.size(), 5u);     // (0.5, 1.0] → 0.6 .. 1.0
  EXPECT_NEAR(secs(c.trajectory.points.front()), 0.1, 1e-6);   // 相对下发时刻
  EXPECT_TRUE(c.finished);                       // 已含末点
  EXPECT_EQ(c.point_index, traj.points.size());
}

TEST(PlaybackPlanner, ChunkAfterEndIsEmptyAndFinished)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto c = pl.make_chunk(make_traj(), 1.0, 1.0);
  EXPECT_TRUE(c.trajectory.points.empty());
  EXPECT_TRUE(c.finished);
}

TEST(PlaybackPlanner, EmptyTrajectoryIsFinishedNotCrashing)
{
  PlaybackPlanner pl(cfg());
  TeachTrajectoryMsg empty;
  empty.joint_names = teach_joint_names();
  const auto c = pl.make_chunk(empty, 0.0, 1.0);
  EXPECT_TRUE(c.trajectory.points.empty());
  EXPECT_TRUE(c.finished);
}

// ★ 稀疏轨迹（压缩后点很少）不能让流式下发卡住：视野里没点也要带上紧随其后的那一个，
//   否则 phase 推不动、机械臂停在半路而状态还是 PLAYING
TEST(PlaybackPlanner, SparseTrajectoryStillAdvances)
{
  PlaybackPlanner pl(cfg(/*horizon=*/0.05));     // 视野比点间距（1.0s）小得多
  const auto traj = make_traj(3, 1.0);           // 0.0 / 1.0 / 2.0
  const auto c = pl.make_chunk(traj, 0.0, 1.0);
  ASSERT_EQ(c.trajectory.points.size(), 1u);
  EXPECT_NEAR(c.next_phase, 1.0, 1e-9);          // 推进了
  EXPECT_FALSE(c.finished);
}

// ── 倍率缩放 ────────────────────────────────────────────────────────────────
// 时间轴压缩 s 倍：段内时间 ÷s、各点速度 ×s。降速（s<1）→ 时间变长、速度变小。
TEST(PlaybackPlanner, SpeedScaleStretchesTimeAndShrinksVelocity)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto traj = make_traj();

  const auto full = pl.make_chunk(traj, 0.0, 1.0);
  const auto half = pl.make_chunk(traj, 0.0, 0.5);

  // 覆盖的是同一段原始时间轴（视野以原始时间计），所以点数相同
  ASSERT_EQ(half.trajectory.points.size(), full.trajectory.points.size());
  EXPECT_NEAR(secs(half.trajectory.points.front()), 0.2, 1e-6);   // 0.1 / 0.5
  EXPECT_NEAR(secs(half.trajectory.points.back()), 1.0, 1e-6);    // 0.5 / 0.5
  EXPECT_NEAR(half.trajectory.points.front().velocities[0], 0.5, 1e-9);   // 1.0 × 0.5
  EXPECT_NEAR(full.trajectory.points.front().velocities[0], 1.0, 1e-9);
}

TEST(PlaybackPlanner, NonPositiveSpeedScaleFallsBackToOriginalSpeed)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto traj = make_traj();
  const auto c = pl.make_chunk(traj, 0.0, 0.0);
  ASSERT_FALSE(c.trajectory.points.empty());
  EXPECT_NEAR(secs(c.trajectory.points.front()), 0.1, 1e-6);
  EXPECT_NEAR(c.trajectory.points.front().velocities[0], 1.0, 1e-9);
}

TEST(PlaybackPlanner, ChunkCarriesAllSixJointsAndNoAccelerations)
{
  PlaybackPlanner pl(cfg(0.5));
  const auto c = pl.make_chunk(make_traj(), 0.0, 1.0);
  ASSERT_FALSE(c.trajectory.points.empty());
  EXPECT_EQ(c.trajectory.points.front().positions.size(), TEACH_JOINT_COUNT);
  EXPECT_EQ(c.trajectory.points.front().velocities.size(), TEACH_JOINT_COUNT);
  // 刻意不填加速度：robot_arm_node 的 build_joint_trajectory 也只给位置+速度
  //（五次样条经验证劣于三次）。两条路径保持一致，免得手感不同。
  EXPECT_TRUE(c.trajectory.points.front().accelerations.empty());
}

// ── 保持轨迹（暂停 / 停止）──────────────────────────────────────────────────
// requirement 9：暂停**必须**下发当前位置保持轨迹。发空轨迹不会让 JTC 停下 ——
// 它会把手上那条剩下的部分继续执行完，表现为"点了暂停还在走"。
TEST(PlaybackPlanner, HoldProducesExactlyOnePointAtCurrentPositionWithZeroVelocity)
{
  PlaybackPlanner pl(cfg());
  const std::vector<double> cur{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const auto hold = pl.make_hold(teach_joint_names(), cur);

  ASSERT_EQ(hold.points.size(), 1u);
  EXPECT_EQ(hold.joint_names, teach_joint_names());
  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    EXPECT_NEAR(hold.points[0].positions[j], cur[j], 1e-12);
    // 速度必须显式给 0：不给的话 JTC 会插出一条带速度的样条，暂停时还会再溜一小段
    EXPECT_NEAR(hold.points[0].velocities[j], 0.0, 1e-12);
  }
  EXPECT_NEAR(secs(hold.points[0]), 0.2, 1e-6);
}

// 回读不全时产出空轨迹，由调用方转为「什么都不发 + 报错」——
// 发一条位置缺省为 0 的保持轨迹会让机械臂冲向 0 位，比不发危险得多
TEST(PlaybackPlanner, HoldIsEmptyWhenReadbackIncomplete)
{
  PlaybackPlanner pl(cfg());
  const std::vector<double> partial{0.1, 0.2, 0.3};
  EXPECT_TRUE(pl.make_hold(teach_joint_names(), partial).points.empty());
}

// ── 接近段 ──────────────────────────────────────────────────────────────────
TEST(PlaybackPlanner, ApproachTargetsTrajectoryStart)
{
  PlaybackPlanner pl(cfg());
  const auto traj = make_traj();
  const auto app = pl.make_approach(traj, teach_joint_names());

  ASSERT_EQ(app.points.size(), 1u);
  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    EXPECT_NEAR(app.points[0].positions[j], traj.points.front().positions[j], 1e-12);
    EXPECT_NEAR(app.points[0].velocities[j], 0.0, 1e-12);   // 停在起点，等下一段再起步
  }
  EXPECT_NEAR(secs(app.points[0]), 3.0, 1e-6);
}

TEST(PlaybackPlanner, ApproachOnEmptyTrajectoryIsEmpty)
{
  PlaybackPlanner pl(cfg());
  TeachTrajectoryMsg empty;
  empty.joint_names = teach_joint_names();
  EXPECT_TRUE(pl.make_approach(empty, teach_joint_names()).points.empty());
}

// ── 配置兜底 ────────────────────────────────────────────────────────────────
TEST(PlaybackPlanner, ZeroConfigValuesAreSanitized)
{
  PlaybackPlanner pl;
  PlaybackPlanner::Config bad;
  bad.chunk_horizon_sec     = 0.0;
  bad.min_point_dt          = 0.0;
  bad.hold_duration_sec     = 0.0;
  bad.approach_duration_sec = -1.0;
  pl.set_config(bad);

  EXPECT_GT(pl.config().chunk_horizon_sec, 0.0);
  EXPECT_GT(pl.config().min_point_dt, 0.0);
  EXPECT_GT(pl.config().hold_duration_sec, 0.0);
  EXPECT_GT(pl.config().approach_duration_sec, 0.0);
}
