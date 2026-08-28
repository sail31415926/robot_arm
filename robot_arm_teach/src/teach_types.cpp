/**
 * @file teach_types.cpp
 * @brief teach_types.hpp 的实现 —— 名字白名单、枚举 ⇄ 字符串、时间戳
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/teach_types.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <cctype>
#include <cstdio>

#include "robot_arm_teach/msg/motion_type.hpp"
#include "robot_arm_teach/msg/teach_state.hpp"

namespace robot_arm_teach
{

namespace
{
using MotionType = robot_arm_teach::msg::MotionType;
using TeachState = robot_arm_teach::msg::TeachState;

// localtime 的可移植封装。localtime() 返回的是共享静态缓冲，多线程下会打架 ——
// 状态广播和保存可能在不同执行线程里同时取时间戳。
std::tm local_time_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm out{};
#if defined(_WIN32)
  localtime_s(&out, &t);
#else
  localtime_r(&t, &out);
#endif
  return out;
}
}  // namespace

bool is_valid_trajectory_name(const std::string & name)
{
  if (name.empty() || name.size() > 64) return false;
  return std::all_of(name.begin(), name.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}

std::string motion_type_to_string(uint8_t value)
{
  switch (value) {
    case MotionType::DOLLY: return "DOLLY";
    case MotionType::TRUCK: return "TRUCK";
    case MotionType::ARC:   return "ARC";
    case MotionType::CRANE: return "CRANE";
    case MotionType::FREEFORM: return "FREEFORM";
    default: return "";      // 空串 = 未知值，调用方按错误处理
  }
}

bool motion_type_from_string(const std::string & text, uint8_t * out)
{
  if (!out) return false;
  if (text == "FREEFORM") { *out = MotionType::FREEFORM; return true; }
  if (text == "DOLLY")    { *out = MotionType::DOLLY;    return true; }
  if (text == "TRUCK")    { *out = MotionType::TRUCK;    return true; }
  if (text == "ARC")      { *out = MotionType::ARC;      return true; }
  if (text == "CRANE")    { *out = MotionType::CRANE;    return true; }
  return false;
}

std::string teach_mode_to_string(uint8_t value)
{
  switch (value) {
    case TeachState::TEACH_MODE_JOG:  return "JOG";
    case TeachState::TEACH_MODE_DRAG: return "DRAG";
    default: return "";
  }
}

bool teach_mode_from_string(const std::string & text, uint8_t * out)
{
  if (!out) return false;
  if (text == "JOG")  { *out = TeachState::TEACH_MODE_JOG;  return true; }
  if (text == "DRAG") { *out = TeachState::TEACH_MODE_DRAG; return true; }
  return false;
}

std::string iso8601_now()
{
  const std::tm tm = local_time_now();
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return buf;
}

std::string compact_timestamp_now()
{
  const std::tm tm = local_time_now();
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return buf;
}

}  // namespace robot_arm_teach
