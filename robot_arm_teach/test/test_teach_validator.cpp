/**
 * @file test_teach_validator.cpp
 * @brief teach_validator 单元测试 —— 回放前置校验的每一道闸都要有反例
 *
 * 这些测试的价值在于：校验逻辑写错的后果是让一条坏轨迹去驱动实物机械臂。
 * 所以每道闸都要有「正例 + 反例」，尤其是 NaN、非单调时间、按名重排这三类
 * ——它们在实机上表现为运动异常，而不是明确的报错。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "robot_arm_teach/teach_validator.hpp"

using namespace robot_arm_teach;   // NOLINT(build/namespaces) —— 测试文件里图省事

namespace
{
// 一条 n 点、dt 间隔、各轴同步匀速走 step 的合法轨迹
TeachTrajectoryMsg make_traj(size_t n = 5, double dt = 0.02, double step = 0.001)
{
  TeachTrajectoryMsg t;
  t.format_version = TeachTrajectoryMsg::FORMAT_VERSION;
  t.name           = "unit_test";
  t.joint_names    = teach_joint_names();
  for (size_t i = 0; i < n; ++i) {
    robot_arm_teach::msg::TeachPoint p;
    for (size_t j = 0; j < TEACH_JOINT_COUNT; ++j) {
      p.positions[j]  = static_cast<double>(i) * step;
      p.velocities[j] = step / dt;
    }
    p.time_from_start = static_cast<double>(i) * dt;
    t.points.push_back(p);
  }
  if (!t.points.empty()) {
    t.start_positions = t.points.front().positions;
    t.end_positions   = t.points.back().positions;
    t.duration_sec    = t.points.back().time_from_start;
  }
  return t;
}

JointBoundMap wide_bounds()
{
  JointBoundMap b;
  for (const auto & n : teach_joint_names()) b[n] = JointBound{-3.0, 3.0};
  return b;
}
}  // namespace

// ── 闸①②：结构 / 关节名 / NaN / 时间单调 ────────────────────────────────────
TEST(TeachValidator, AcceptsWellFormedTrajectory)
{
  EXPECT_TRUE(validate_structure(make_traj(), teach_joint_names()).ok);
}

TEST(TeachValidator, RejectsEmptyTrajectory)
{
  auto t = make_traj();
  t.points.clear();
  const auto r = validate_structure(t, teach_joint_names());
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "invalid_trajectory");
}

TEST(TeachValidator, RejectsUnsupportedFormatVersion)
{
  auto t = make_traj();
  t.format_version = TeachTrajectoryMsg::FORMAT_VERSION + 1;
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);
  t.format_version = 0;
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);
}

TEST(TeachValidator, RejectsWrongJointCount)
{
  auto t = make_traj();
  t.joint_names.pop_back();
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);
}

// 关键：**不按名重排**。名字顺序被打乱的轨迹必须拒绝，不能"聪明地"修正 ——
// 猜错就是让机械臂按错的轴走。
TEST(TeachValidator, RejectsPermutedJointNamesInsteadOfReordering)
{
  auto t = make_traj();
  std::swap(t.joint_names[0], t.joint_names[1]);
  const auto r = validate_structure(t, teach_joint_names());
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "invalid_trajectory");
}

TEST(TeachValidator, RejectsNaNAndInf)
{
  auto t = make_traj();
  t.points[2].positions[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);

  t = make_traj();
  t.points[1].velocities[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);

  t = make_traj();
  t.points[4].time_from_start = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);
}

TEST(TeachValidator, RejectsNonMonotonicTime)
{
  auto t = make_traj();
  t.points[3].time_from_start = t.points[2].time_from_start;   // 相等也不行
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);

  t = make_traj();
  t.points[3].time_from_start = t.points[1].time_from_start;   // 倒退
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);
}

TEST(TeachValidator, RejectsNegativeTime)
{
  auto t = make_traj();
  t.points[0].time_from_start = -0.01;
  EXPECT_FALSE(validate_structure(t, teach_joint_names()).ok);
}

// ── 闸③：URDF 限位 ──────────────────────────────────────────────────────────
TEST(TeachValidator, LimitsPassWithinBounds)
{
  EXPECT_TRUE(validate_limits(make_traj(), wide_bounds()).ok);
}

TEST(TeachValidator, LimitsRejectOutOfRange)
{
  auto t = make_traj();
  t.points[2].positions[1] = 9.0;
  const auto r = validate_limits(t, wide_bounds());
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "out_of_range");
}

// URDF 未就绪（空 map）时必须 fail-open —— 把机械臂卡死比不校验更糟，
// 与 robot_arm_node 的 JointLimitsCache 策略一致。
TEST(TeachValidator, LimitsFailOpenWhenUrdfMissing)
{
  auto t = make_traj();
  t.points[2].positions[1] = 9.0;
  EXPECT_TRUE(validate_limits(t, JointBoundMap{}).ok);
}

// 某轴不在 map 里（continuous / 无 limit 声明）= 无界，不校验该轴
TEST(TeachValidator, LimitsSkipUnknownJoint)
{
  auto t = make_traj();
  t.points[2].positions[0] = 99.0;
  JointBoundMap b = wide_bounds();
  b.erase(teach_joint_names()[0]);
  EXPECT_TRUE(validate_limits(t, b).ok);
}

// ── 闸④：速度 / 加速度 + 倍率缩放 ───────────────────────────────────────────
TEST(TeachValidator, EnvelopeUsesPositionDiffWhenVelocityFieldIsZero)
{
  // /joint_states 不带 velocity 的后端会录出全 0 的 velocities。
  // 若包络只看 velocities，速度闸就形同虚设 —— 这里断言差分兜住了。
  auto t = make_traj(5, 0.02, 0.01);   // 0.01rad / 0.02s = 0.5rad/s
  for (auto & p : t.points) p.velocities.fill(0.0);
  const auto env = compute_envelope(t);
  EXPECT_NEAR(env.max_velocity[0], 0.5, 1e-9);
}

TEST(TeachValidator, SpeedGateRejectsOverLimit)
{
  auto t = make_traj(5, 0.02, 0.05);   // 2.5 rad/s
  MotionCaps caps{1.0, 100.0};
  const auto r = validate_speed(t, caps, 1.0);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "over_speed");
}

// 降速必然更安全：同一条超速轨迹在 0.2 倍率下应当通过速度闸
TEST(TeachValidator, SlowingDownMakesOverSpeedTrajectoryPass)
{
  auto t = make_traj(5, 0.02, 0.05);   // 2.5 rad/s
  MotionCaps caps{1.0, 100.0};
  EXPECT_FALSE(validate_speed(t, caps, 1.0).ok);
  EXPECT_TRUE(validate_speed(t, caps, 0.2).ok);   // 2.5 × 0.2 = 0.5
}

// 加速度按倍率的**平方**缩放（时间轴压缩 s 倍 → q̈ ×s²）
TEST(TeachValidator, AccelerationScalesQuadratically)
{
  Envelope e;
  e.max_velocity.fill(1.0);
  e.max_acceleration.fill(2.0);
  const auto s = scale_envelope(e, 0.5);
  EXPECT_NEAR(s.max_velocity[0], 0.5, 1e-12);
  EXPECT_NEAR(s.max_acceleration[0], 0.5, 1e-12);   // 2.0 × 0.25
}

// ── 闸⑥：起点距离（只看 J1-3）───────────────────────────────────────────────
TEST(TeachValidator, IsAtStartOnlyChecksArmJoints)
{
  const auto t = make_traj();
  // J1-3 完全对上，云台 J4-6 差很远 —— 仍应判"在起点"。
  // 末端在云台之后，云台有静差就永远到不了，判据交给云台会让每次回放都失败。
  std::vector<double> cur{0.0, 0.0, 0.0, 1.5, -1.5, 1.5};
  double dev = -1.0;
  EXPECT_TRUE(is_at_start(t, cur, 0.01, JOG_JOINT_COUNT, &dev));
  EXPECT_NEAR(dev, 0.0, 1e-12);
}

TEST(TeachValidator, IsAtStartRejectsFarArmPosition)
{
  const auto t = make_traj();
  std::vector<double> cur{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  double dev = 0.0;
  EXPECT_FALSE(is_at_start(t, cur, 0.01, JOG_JOINT_COUNT, &dev));
  EXPECT_NEAR(dev, 0.5, 1e-12);
}

// ── 倍率规范化 ──────────────────────────────────────────────────────────────
TEST(TeachValidator, SpeedScaleZeroMeansOriginalSpeedAndIsNotAClamp)
{
  // 0 是 ros2 service call 不填该字段时的默认值 → 当作「原速」，且**不算夹紧**
  //（否则最常见的调用方式每次都会刷一条无意义的告警）
  double s = 0.0;
  EXPECT_FALSE(normalize_speed_scale(&s, 0.1, 1.0));
  EXPECT_NEAR(s, 1.0, 1e-12);

  s = -3.0;
  EXPECT_FALSE(normalize_speed_scale(&s, 0.1, 1.0));
  EXPECT_NEAR(s, 1.0, 1e-12);
}

TEST(TeachValidator, SpeedScaleIsClampedAndReported)
{
  double s = 2.5;
  EXPECT_TRUE(normalize_speed_scale(&s, 0.1, 1.0));
  EXPECT_NEAR(s, 1.0, 1e-12);

  s = 0.01;
  EXPECT_TRUE(normalize_speed_scale(&s, 0.1, 1.0));
  EXPECT_NEAR(s, 0.1, 1e-12);

  s = 0.5;
  EXPECT_FALSE(normalize_speed_scale(&s, 0.1, 1.0));
  EXPECT_NEAR(s, 0.5, 1e-12);
}

// ── 名字白名单（save / delete 直接拼路径，必须挡住越界）─────────────────────
TEST(TeachTypes, TrajectoryNameWhitelist)
{
  EXPECT_TRUE(is_valid_trajectory_name("shot_dolly-01"));
  EXPECT_TRUE(is_valid_trajectory_name("A"));
  EXPECT_FALSE(is_valid_trajectory_name(""));
  EXPECT_FALSE(is_valid_trajectory_name("../../etc/passwd"));
  EXPECT_FALSE(is_valid_trajectory_name("a/b"));
  EXPECT_FALSE(is_valid_trajectory_name("has space"));
  EXPECT_FALSE(is_valid_trajectory_name("dot.name"));
  EXPECT_FALSE(is_valid_trajectory_name(std::string(65, 'x')));
  EXPECT_TRUE(is_valid_trajectory_name(std::string(64, 'x')));
}

TEST(TeachTypes, MotionTypeRoundTrip)
{
  for (const char * name : {"FREEFORM", "DOLLY", "TRUCK", "ARC", "CRANE"}) {
    uint8_t v = 255;
    ASSERT_TRUE(motion_type_from_string(name, &v)) << name;
    EXPECT_EQ(motion_type_to_string(v), name);
  }
  uint8_t v = 42;
  EXPECT_FALSE(motion_type_from_string("PAN", &v));
  EXPECT_EQ(v, 42);                          // 解析失败不改出参
  EXPECT_TRUE(motion_type_to_string(200).empty());   // 未知值 → 空串
}
