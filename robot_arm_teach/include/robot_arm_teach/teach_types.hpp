/**
 * @file teach_types.hpp
 * @brief 示教包的共享类型与常量（header-only）—— 拓扑 / 话题 / 名字校验的单一来源
 *
 * 对齐 robot_arm_node/motion/constants.hpp 的做法：拓扑与话题名写成常量（必须与 URDF、
 * 与既有产品总线逐字一致，做成运行期可改只会多一个漂移源），而阈值/频率一律走 ROS 参数
 * （见 config/teach_params.yaml）。
 *
 * ★ 本包对既有工程的依赖只有「消息类型 + 话题/服务名」，没有任何头文件或库级耦合：
 *   robot_arm_node 的 CMakeLists 没有 ament_export_targets，下游链不到它的
 *   JointLimitsCache / motion 库。所以本包自己解析 /robot_description
 *   （见 joint_limits_guard.hpp）—— 限位的真相源仍然是 URDF 这一份，
 *   只是解析代码各有一份，不是把限位数值抄成第二份常量。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "robot_arm_teach/msg/teach_point.hpp"
#include "robot_arm_teach/msg/teach_trajectory.hpp"

namespace robot_arm_teach
{

// 全包共用的消息别名。放在这里（而不是各头文件各写一份）是为了只有**一个**定义点 ——
// 同名别名散在多处时，哪天要换类型会漏改，而 C++ 允许同类型重复 typedef，漏改不报错。
using TeachTrajectoryMsg = robot_arm_teach::msg::TeachTrajectory;
using TeachPointMsg      = robot_arm_teach::msg::TeachPoint;

// ── 机器人拓扑 ──────────────────────────────────────────────────────────────
// 6 轴整体录制：J1-3 是机械臂（CANopen），J4-6 是云台（经 GimbalForwardingInterface
// 回读）。示教录 6 轴、回放发 6 轴，与 arm_controller claim 的关节集合一致。
constexpr size_t TEACH_JOINT_COUNT = 6;

// 可点动的轴数。产品总线 ArmJointVelocityCommand 固定 3 个值（Joint1-3），
// VelocityStreamServer 对其他长度直接丢弃 —— 所以点动示教动不了云台 J4-6，
// 它们只能保持当前角度被动录进去。这不是本包的选择，是既有总线的形状。
constexpr size_t JOG_JOINT_COUNT = 3;

inline const std::vector<std::string> & teach_joint_names()
{
  static const std::vector<std::string> kNames{
      "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  return kNames;
}

inline const std::string PLANNING_GROUP = "arm";

// ── 既有产品总线（只用、不改）────────────────────────────────────────────────
inline const std::string TOPIC_JOINT_STATES   = "/joint_states";
inline const std::string TOPIC_TRAJECTORY     = "/arm_controller/joint_trajectory";
inline const std::string TOPIC_JOINT_VELOCITY = "/robot_arm/cmd/joint_velocity";
inline const std::string TOPIC_CONTROL_MODE   = "/robot_arm/control_mode";
inline const std::string TOPIC_ARM_STATUS     = "/robot_arm/arm_status";
inline const std::string SRV_SWITCH_MODE      = "/robot_arm/switch_control_mode";
inline const std::string SRV_STATE_VALIDITY   = "/check_state_validity";
inline const std::string TOPIC_DESCRIPTION    = "/robot_description";

// ── 本包新增的接口（全部挂在 /robot_arm/teach/ 命名空间下）──────────────────
inline const std::string TOPIC_TEACH_STATE    = "/robot_arm/teach/state";
inline const std::string TOPIC_PLAYBACK_STATE = "/robot_arm/teach/playback";
inline const std::string TOPIC_JOG            = "/robot_arm/teach/jog";

// ── 关节限位（URDF 解析结果的容器）──────────────────────────────────────────
struct JointBound
{
  double lower{0.0};    // rad
  double upper{0.0};    // rad
};
// 空 map = 未拿到 URDF。调用方须 fail-open（放行并告警），与 robot_arm_node 的
// JointLimitsCache 策略一致 —— 把机械臂卡死比不校验更糟。
using JointBoundMap = std::map<std::string, JointBound>;

// ── 运动学包络上限（回放闸④用）─────────────────────────────────────────────
struct MotionCaps
{
  double max_velocity{1.0};        // rad/s，逐轴
  double max_acceleration{4.0};    // rad/s^2，逐轴
};

using JointArray = std::array<double, TEACH_JOINT_COUNT>;

// ── 轨迹名白名单 ────────────────────────────────────────────────────────────
// 名字直接当文件名用，必须挡住 "/" 与 ".."，否则 delete/save 能越界操作任意路径。
// 允许 [A-Za-z0-9_-]，长度 1..64。
bool is_valid_trajectory_name(const std::string & name);

// 运镜类型 ⇄ 字符串（YAML 里存名字而不是数字：数字换了枚举值老文件就静默读错，
// 而名字读不出来会明确报错）
std::string motion_type_to_string(uint8_t value);
// 解析失败返回 false，*out 不变
bool motion_type_from_string(const std::string & text, uint8_t * out);

std::string teach_mode_to_string(uint8_t value);
bool teach_mode_from_string(const std::string & text, uint8_t * out);

// ISO-8601 本地时间戳（用于 created_at 与自动生成的轨迹名）。
// 用 system_clock —— 这是给人看的时间戳，不参与任何间隔计算（间隔一律 steady_clock）。
std::string iso8601_now();
// 适合做文件名的紧凑时间戳，如 20260824_150405
std::string compact_timestamp_now();

}  // namespace robot_arm_teach
