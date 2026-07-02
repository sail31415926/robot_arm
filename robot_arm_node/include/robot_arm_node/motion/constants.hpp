/**
 * @file constants.hpp
 * @brief motion 层共享常量（C++, header-only）—— 拓扑/坐标系/话题/时序/限幅的单一来源
 *
 * 对应原 Python arm_motion/constants.py。产品栈（motion/state/commander）转 C++ 后
 * 规划层常量的唯一来源：机器人拓扑（关节名 / 规划组 / 末端 / 基座）、Ruckig 与 IK 时序、
 * 默认运动限制、速度流限幅、话题与服务名。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <string>
#include <vector>

namespace robot_arm_node::motion
{

// ── 机器人拓扑 ──────────────────────────────────────────────────────────────
inline const std::vector<std::string> JOINT_NAMES{
    "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
inline const std::string PLANNING_GROUP = "arm";
inline const std::string EEF_LINK       = "tool0";
inline const std::string BASE_FRAME     = "arm_base_link";

// ── Ruckig / IK 时序 ────────────────────────────────────────────────────────
constexpr double STREAM_DT    = 0.01;   // s，Ruckig 步长（100Hz）
constexpr double IK_SAMPLE_DT = 0.03;   // s，IK 采样步长
constexpr int    IK_DECIMATE  = 3;      // round(IK_SAMPLE_DT / STREAM_DT)
constexpr double IK_TIMEOUT_S = 0.05;   // 单次 IK 超时

// ── 默认运动限制 ────────────────────────────────────────────────────────────
constexpr double DEFAULT_V_POS = 0.05, DEFAULT_A_POS = 0.10, DEFAULT_J_POS = 1.00;
constexpr double DEFAULT_V_ORI = 0.10, DEFAULT_A_ORI = 0.20, DEFAULT_J_ORI = 2.00;

// ── 速度流限幅 ──────────────────────────────────────────────────────────────
constexpr double MAX_V_LIN = 0.30;   // m/s
constexpr double MAX_V_ANG = 1.00;   // rad/s

// ── 话题 / 服务 ─────────────────────────────────────────────────────────────
inline const std::string TRAJ_TOPIC        = "/arm_controller/joint_trajectory";
inline const std::string TWIST_TOPIC       = "/servo_node/delta_twist_cmds";
inline const std::string IK_SERVICE        = "/compute_ik";
inline const std::string JOINT_STATE_TOPIC = "/joint_states";

}  // namespace robot_arm_node::motion
