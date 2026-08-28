/**
 * @file test_trajectory_store.cpp
 * @brief TrajectoryStore 单元测试 —— YAML 往返 / 覆盖策略 / 越界防护 / 坏文件
 *
 * 往返测试是这一层的核心：轨迹落盘再读回来必须**逐字段一致**，否则回放的是另一条轨迹，
 * 而这种错误在实机上表现为"动作和示教时不一样"，极难归因。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "robot_arm_teach/msg/motion_meta.hpp"
#include "robot_arm_teach/msg/motion_type.hpp"
#include "robot_arm_teach/msg/teach_state.hpp"
#include "robot_arm_teach/trajectory_store.hpp"

using namespace robot_arm_teach;   // NOLINT(build/namespaces)
namespace fs = std::filesystem;

using MotionMeta = robot_arm_teach::msg::MotionMeta;
using MotionType = robot_arm_teach::msg::MotionType;
using TeachState = robot_arm_teach::msg::TeachState;

namespace
{
TeachTrajectoryMsg make_arc_traj(const std::string & name)
{
  TeachTrajectoryMsg t;
  t.format_version = TeachTrajectoryMsg::FORMAT_VERSION;
  t.name           = name;
  t.created_at     = "2026-08-24T15:04:05";
  t.description    = "环绕运镜示教（单元测试）";
  t.joint_names    = teach_joint_names();

  for (size_t i = 0; i < 5; ++i) {
    robot_arm_teach::msg::TeachPoint p;
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      p.positions[j]  = 0.001 * static_cast<double>(i) + 0.1 * static_cast<double>(j);
      p.velocities[j] = 0.05 * static_cast<double>(j + 1);
    }
    p.time_from_start = 0.02 * static_cast<double>(i);
    t.points.push_back(p);
  }
  t.start_positions = t.points.front().positions;
  t.end_positions   = t.points.back().positions;
  t.duration_sec    = t.points.back().time_from_start;
  t.max_velocity.fill(0.3);
  t.max_acceleration.fill(1.2);

  t.motion_type.value          = MotionType::ARC;
  t.teach_mode                 = TeachState::TEACH_MODE_JOG;
  t.teach_control_mode.mode    = 1;      // ControlMode::JOINT_VELOCITY
  t.sample_rate_hz             = 50.0;
  t.raw_point_count            = 42;
  t.compress_position_eps      = 0.002;

  auto & m = t.motion_meta;
  m.subject_label            = "person_0";
  m.arc_target_point         = {0.4, 0.0, 0.6};
  m.arc_radius_m             = 0.55;
  m.arc_start_angle_deg      = -30.0;
  m.arc_end_angle_deg        = 30.0;
  m.arc_rotation_direction   = MotionMeta::ARC_CCW;
  m.dolly_direction          = {1.0, 0.0, 0.0};
  m.dolly_distance_m         = -0.2;
  m.dolly_keep_camera_facing = true;
  m.truck_direction          = {0.0, 1.0, 0.0};
  m.truck_distance_m         = 0.15;
  m.truck_follow_policy      = MotionMeta::FOLLOW_LOCK_SUBJECT;
  m.crane_height_delta_m     = 0.1;
  m.crane_direction          = MotionMeta::CRANE_UP;
  return t;
}

// 每个测试用独立子目录，互不干扰
class StoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = (fs::temp_directory_path() /
            ("robot_arm_teach_ut_" + std::string(
                ::testing::UnitTest::GetInstance()->current_test_info()->name()))).string();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  void TearDown() override
  {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  std::string dir_;
};
}  // namespace

// ── 往返 ────────────────────────────────────────────────────────────────────
TEST_F(StoreTest, RoundTripPreservesEveryField)
{
  TrajectoryStore store(dir_);
  const auto src = make_arc_traj("arc_01");

  const auto sr = store.save(src, /*overwrite=*/false);
  ASSERT_TRUE(sr.ok) << sr.message;
  EXPECT_TRUE(fs::exists(sr.path));
  EXPECT_TRUE(store.exists("arc_01"));
  // 原子写入不应留下临时文件
  EXPECT_FALSE(fs::exists(sr.path + ".tmp"));

  TeachTrajectoryMsg got;
  const auto lr = store.load("arc_01", &got);
  ASSERT_TRUE(lr.ok) << lr.message;

  EXPECT_EQ(got.format_version, src.format_version);
  EXPECT_EQ(got.name, src.name);
  EXPECT_EQ(got.created_at, src.created_at);
  EXPECT_EQ(got.description, src.description);
  EXPECT_EQ(got.joint_names, src.joint_names);
  EXPECT_EQ(got.motion_type.value, MotionType::ARC);
  EXPECT_EQ(got.teach_mode, src.teach_mode);
  EXPECT_EQ(got.teach_control_mode.mode, src.teach_control_mode.mode);
  EXPECT_EQ(got.raw_point_count, src.raw_point_count);
  EXPECT_NEAR(got.sample_rate_hz, src.sample_rate_hz, 1e-12);
  EXPECT_NEAR(got.compress_position_eps, src.compress_position_eps, 1e-12);
  EXPECT_NEAR(got.duration_sec, src.duration_sec, 1e-12);

  ASSERT_EQ(got.points.size(), src.points.size());
  for (size_t i = 0; i < src.points.size(); ++i) {
    EXPECT_NEAR(got.points[i].time_from_start, src.points[i].time_from_start, 1e-12) << i;
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      EXPECT_NEAR(got.points[i].positions[j], src.points[i].positions[j], 1e-12);
      EXPECT_NEAR(got.points[i].velocities[j], src.points[i].velocities[j], 1e-12);
    }
  }
  for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
    EXPECT_NEAR(got.start_positions[j], src.start_positions[j], 1e-12);
    EXPECT_NEAR(got.end_positions[j], src.end_positions[j], 1e-12);
    EXPECT_NEAR(got.max_velocity[j], src.max_velocity[j], 1e-12);
    EXPECT_NEAR(got.max_acceleration[j], src.max_acceleration[j], 1e-12);
  }
}

// 运镜元数据（DOLLY/TRUCK/ARC/CRANE 四段 + 三个枚举）必须一并往返
TEST_F(StoreTest, RoundTripPreservesMotionMeta)
{
  TrajectoryStore store(dir_);
  const auto src = make_arc_traj("arc_meta");
  ASSERT_TRUE(store.save(src, false).ok);

  TeachTrajectoryMsg got;
  ASSERT_TRUE(store.load("arc_meta", &got).ok);
  const auto & a = src.motion_meta;
  const auto & b = got.motion_meta;

  EXPECT_EQ(b.subject_label, a.subject_label);
  EXPECT_NEAR(b.arc_radius_m, a.arc_radius_m, 1e-12);
  EXPECT_NEAR(b.arc_start_angle_deg, a.arc_start_angle_deg, 1e-12);
  EXPECT_NEAR(b.arc_end_angle_deg, a.arc_end_angle_deg, 1e-12);
  EXPECT_EQ(b.arc_rotation_direction, MotionMeta::ARC_CCW);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(b.arc_target_point[i], a.arc_target_point[i], 1e-12);
    EXPECT_NEAR(b.dolly_direction[i], a.dolly_direction[i], 1e-12);
    EXPECT_NEAR(b.truck_direction[i], a.truck_direction[i], 1e-12);
  }
  EXPECT_NEAR(b.dolly_distance_m, a.dolly_distance_m, 1e-12);
  EXPECT_TRUE(b.dolly_keep_camera_facing);
  EXPECT_NEAR(b.truck_distance_m, a.truck_distance_m, 1e-12);
  EXPECT_EQ(b.truck_follow_policy, MotionMeta::FOLLOW_LOCK_SUBJECT);
  EXPECT_NEAR(b.crane_height_delta_m, a.crane_height_delta_m, 1e-12);
  EXPECT_EQ(b.crane_direction, MotionMeta::CRANE_UP);
}

// 枚举在文件里存的是**名字**而不是数字：数字换了枚举值老文件会被静默误读
TEST_F(StoreTest, EnumsAreStoredAsNamesNotNumbers)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.save(make_arc_traj("enum_check"), false).ok);

  std::ifstream ifs(store.path_for("enum_check"));
  const std::string text((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("motion_type: ARC"), std::string::npos) << text;
  EXPECT_NE(text.find("CCW"), std::string::npos);
  EXPECT_NE(text.find("LOCK_SUBJECT"), std::string::npos);
}

// ── 覆盖策略 ────────────────────────────────────────────────────────────────
TEST_F(StoreTest, RefusesOverwriteUnlessAsked)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.save(make_arc_traj("dup"), false).ok);

  const auto again = store.save(make_arc_traj("dup"), /*overwrite=*/false);
  EXPECT_FALSE(again.ok);
  EXPECT_NE(again.message.find("已存在"), std::string::npos) << again.message;

  EXPECT_TRUE(store.save(make_arc_traj("dup"), /*overwrite=*/true).ok);
}

// ── 名字白名单：越界路径必须挡在拼路径之前 ──────────────────────────────────
TEST_F(StoreTest, RejectsPathTraversalNames)
{
  TrajectoryStore store(dir_);
  auto t = make_arc_traj("ok");

  t.name = "../evil";
  EXPECT_FALSE(store.save(t, true).ok);
  t.name = "sub/dir";
  EXPECT_FALSE(store.save(t, true).ok);
  t.name = "";
  EXPECT_FALSE(store.save(t, true).ok);

  TeachTrajectoryMsg got;
  EXPECT_FALSE(store.load("../../etc/passwd", &got).ok);
  EXPECT_FALSE(store.remove("../evil").ok);
  EXPECT_TRUE(store.path_for("../evil").empty());
}

// ── 缺失 / 坏文件 ───────────────────────────────────────────────────────────
TEST_F(StoreTest, LoadMissingFails)
{
  TrajectoryStore store(dir_);
  TeachTrajectoryMsg got;
  const auto r = store.load("nope", &got);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.message.find("不存在"), std::string::npos) << r.message;
}

TEST_F(StoreTest, LoadRejectsUnsupportedFormatVersion)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.ensure_directory().ok);
  {
    std::ofstream ofs(store.path_for("future"));
    ofs << "format_version: 999\nname: future\njoint_names: [Joint1]\npoints: []\n";
  }
  TeachTrajectoryMsg got;
  const auto r = store.load("future", &got);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.message.find("不受支持"), std::string::npos) << r.message;
}

TEST_F(StoreTest, LoadRejectsWrongPointDimension)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.ensure_directory().ok);
  {
    std::ofstream ofs(store.path_for("badpoint"));
    ofs << "format_version: 1\nname: badpoint\n"
        << "joint_names: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]\n"
        << "motion_type: FREEFORM\nteach_mode: JOG\n"
        << "points:\n  - {t: 0.0, p: [0,0,0], v: [0,0,0,0,0,0]}\n";
  }
  TeachTrajectoryMsg got;
  const auto r = store.load("badpoint", &got);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.message.find("p / v"), std::string::npos) << r.message;
}

TEST_F(StoreTest, LoadRejectsUnknownMotionType)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.ensure_directory().ok);
  {
    std::ofstream ofs(store.path_for("badenum"));
    ofs << "format_version: 1\nname: badenum\n"
        << "joint_names: [Joint1]\nmotion_type: PAN\npoints: []\n";
  }
  TeachTrajectoryMsg got;
  const auto r = store.load("badenum", &got);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.message.find("motion_type"), std::string::npos) << r.message;
}

// ── 列目录 ──────────────────────────────────────────────────────────────────
TEST_F(StoreTest, ListReturnsSortedSummariesAndReportsBadFiles)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.save(make_arc_traj("b_second"), false).ok);
  ASSERT_TRUE(store.save(make_arc_traj("a_first"), false).ok);
  {
    std::ofstream ofs(store.path_for("broken"));
    ofs << "this is: [not valid yaml\n";
  }
  // 非 .yaml 的文件应被忽略
  {
    std::ofstream ofs((fs::path(dir_) / "notes.txt").string());
    ofs << "hello\n";
  }

  std::vector<TrajectoryStore::Summary> items;
  std::vector<std::string> invalid;
  ASSERT_TRUE(store.list(&items, &invalid).ok);

  ASSERT_EQ(items.size(), 2u);
  EXPECT_EQ(items[0].name, "a_first");     // 按文件名排序
  EXPECT_EQ(items[1].name, "b_second");
  EXPECT_EQ(items[0].motion_type, MotionType::ARC);
  EXPECT_EQ(items[0].point_count, 5u);
  EXPECT_NEAR(items[0].duration_sec, 0.08, 1e-9);

  ASSERT_EQ(invalid.size(), 1u);
  EXPECT_EQ(invalid[0], "broken");
}

TEST_F(StoreTest, ListOnMissingDirectoryIsNotAnError)
{
  TrajectoryStore store(dir_);   // SetUp 已把目录删掉
  std::vector<TrajectoryStore::Summary> items;
  std::vector<std::string> invalid;
  const auto r = store.list(&items, &invalid);
  EXPECT_TRUE(r.ok);             // 一条都没存过不是错误
  EXPECT_TRUE(items.empty());
}

// ── 删除 ────────────────────────────────────────────────────────────────────
TEST_F(StoreTest, RemoveDeletesFileAndReportsMissing)
{
  TrajectoryStore store(dir_);
  ASSERT_TRUE(store.save(make_arc_traj("gone"), false).ok);

  const auto r = store.remove("gone");
  EXPECT_TRUE(r.ok);
  EXPECT_FALSE(store.exists("gone"));
  EXPECT_FALSE(fs::exists(r.path));

  EXPECT_FALSE(store.remove("gone").ok);   // 再删一次
}

TEST_F(StoreTest, EnsureDirectoryCreatesNestedPath)
{
  const std::string nested = (fs::path(dir_) / "a" / "b" / "c").string();
  TrajectoryStore store(nested);
  ASSERT_TRUE(store.ensure_directory().ok);
  EXPECT_TRUE(fs::is_directory(nested));
}
