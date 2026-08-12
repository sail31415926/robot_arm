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
// 前 3 个是机械臂 CANopen 关节（J1-3，实时反馈），后 3 个是云台关节（J4-6，
// 经 GimbalForwardingInterface 转发回读，云台未上电/无反馈时位置不收敛）。
// 关节空间的到位/回零判据只看前 ARM_JOINT_COUNT 个，云台状态不阻塞机械臂动作。
constexpr size_t ARM_JOINT_COUNT = 3;
inline const std::string PLANNING_GROUP = "arm";
// 2026-07-28 云台换 V2：末端 = 云台 Joint6 后的安装板 gimbal_tool0（= SRDF 规划组 tip），
// 臂自身的 tool0 只是 Link3 +X 300mm 的机械法兰，不是规划末端。
inline const std::string EEF_LINK       = "gimbal_tool0";
inline const std::string BASE_FRAME     = "arm_base_link";

// ── Ruckig 时序 ─────────────────────────────────────────────────────────────
constexpr double STREAM_DT    = 0.01;   // s，Ruckig 步长（100Hz）
// IK 采样步长 / 抽取比 / 单次超时已于 2026-08-12 移入 tuning::params()
//（ik.sample_dt / ik.timeout_s，抽取比由 sample_dt / STREAM_DT 推导），
// 由 robot_arm_bringup/config/arm_params.yaml 配置。此处不再保留副本 —— 留一份
// constexpr 就等于多一个会和 YAML 打架的来源。

// ── 默认运动限制 ────────────────────────────────────────────────────────────
constexpr double DEFAULT_V_POS = 0.05, DEFAULT_A_POS = 0.10, DEFAULT_J_POS = 1.00;
constexpr double DEFAULT_V_ORI = 0.10, DEFAULT_A_ORI = 0.20, DEFAULT_J_ORI = 2.00;

// ── 速度流限幅 ──────────────────────────────────────────────────────────────
constexpr double MAX_V_LIN = 0.30;   // m/s
constexpr double MAX_V_ANG = 1.00;   // rad/s
// 产品接口 ArmFollowCommand（笛卡尔速度控制）的默认线速度上限，比调试链的 MAX_V_LIN 保守。
// 与 ArmFollowCommand.msg 注释里承诺的 [-0.2, 0.2] m/s 一致；可由参数
// cartesian_velocity.max_linear_speed 覆盖。
constexpr double MAX_V_LIN_FOLLOW = 0.20;   // m/s

// ── 话题 / 服务 ─────────────────────────────────────────────────────────────
inline const std::string TRAJ_TOPIC        = "/arm_controller/joint_trajectory";
inline const std::string TWIST_TOPIC       = "/servo_node/delta_twist_cmds";
inline const std::string IK_SERVICE        = "/compute_ik";
inline const std::string JOINT_STATE_TOPIC = "/joint_states";

}  // namespace robot_arm_node::motion
