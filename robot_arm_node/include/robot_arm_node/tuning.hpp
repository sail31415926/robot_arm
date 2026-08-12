/**
 * @file tuning.hpp
 * @brief 可调参数的单一入口（C++）—— 启动时从 YAML 灌入，运行期只读
 *
 * 为什么有这个文件：这些量原来是散在 motion_policy.hpp / constants.hpp / 各 action server
 * 匿名 namespace 里的 constexpr，实机上想动一个到位容差或速度档位就得改源码重编。现在全部
 * 挪到这里，由 arm_commander 节点在构造第一步从 ROS 参数读入，值写在
 *   robot_arm_bringup/config/arm_params.yaml        （全后端默认，仿真直接用这一份）
 *   robot_arm_bringup/config/arm_params_real.yaml   （只写实机要覆盖的那几项）
 *
 * ⚠️ YAML 里的节点键必须是 **arm_commander**（节点名），不是 arm_commander_node（可执行文件名）。
 *    写错不会报错，参数**静默不生效**，只是全部跑默认值 —— 排查时先 `ros2 param list /arm_commander`
 *    再 `ros2 param get /arm_commander <名>` 对一下。
 *
 * ⚠️ 进程内单例（params()）。为什么不把结构体从构造函数一路传进 6 个 server：那要改每个
 *    server 的 ctor 签名和 motion 层的默认实参，改动面大得多，而 motion_policy.hpp 的
 *    `speed_profile(k)` / `is_at_pose(c,t)` 这类免参调用点正是它的价值所在。代价是两条硬约束：
 *      1. declare_and_load() 必须在**任何子系统构造之前**调用（已放在 ArmCommanderNode
 *         构造函数第一行），否则 server 拿到的是结构体默认值而不是 YAML 值；
 *      2. 一个进程只能有一条臂（arm_commander 本来就是独立进程）。
 *    运行期不再改（没有 on_set_parameters 回调）：容差/档位在动作执行中途变会让到位判据
 *    与规划用的限制不自洽。要改值就改 YAML 重启节点。
 *
 * 不在这里的东西：机器人拓扑（关节名/规划组/末端/基座 frame）与话题名仍在 constants.hpp，
 * 它们必须与 URDF/SRDF 逐字一致，做成运行期可改只会多一个漂移源。
 *
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robot_arm_node::tuning
{

// 笛卡尔速度档位：位置与姿态各一组「速度 / 加速度 / 加加速度」，喂给 Ruckig
struct Speed
{
  double v_pos, a_pos, j_pos;   // m/s, m/s², m/s³
  double v_ori, a_ori, j_ori;   // rad/s, rad/s², rad/s³
};

struct Params
{
  // ── 到位容差 ────────────────────────────────────────────────────────────────
  // 实机静差比仿真大得多，容差给太紧会让动作走到 timeout（哪怕臂实际已经到位）
  double position_tolerance_m{0.01};
  double orientation_tolerance_deg{2.0};
  double joint_tolerance_rad{0.02};

  // ── 笛卡尔速度档位（ArmMoveToPose / ArmTrajectoryShot 的 SLOW/NORMAL/FAST）──
  Speed speed_slow  {0.02, 0.05, 0.50, 0.05, 0.10, 1.00};
  Speed speed_normal{0.05, 0.10, 1.00, 0.10, 0.20, 2.00};
  Speed speed_fast  {0.10, 0.20, 2.00, 0.20, 0.40, 4.00};

  // ── 关节空间档位（**平均**角速度 rad/s；ArmMoveToJoint 用它反算时长）────────
  // 峰值 ≈ 1.875 × 平均（JTC 五次多项式插值），FAST 峰值 ≈ 2.25 rad/s，
  // 仍低于 URDF 里 J1-3 的 velocity=3.14 机械限。
  double joint_speed_slow_rps{0.30};
  double joint_speed_normal_rps{0.60};
  double joint_speed_fast_rps{1.20};
  double joint_min_duration_sec{0.5};    // 短距离下限，避免除出极小时长
  double joint_max_duration_sec{30.0};   // 上限，兜住异常大位移

  // ── IK / 时序 ───────────────────────────────────────────────────────────────
  double ik_timeout_s{0.05};             // 单次 /compute_ik 超时
  double ik_sample_dt{0.03};             // IK 采样步长（s）
  // 派生量，**不单独配**：= round(ik_sample_dt / motion::STREAM_DT)。
  // 两个都开成独立参数必然出现「采样 0.03 而抽取 5」这种自相矛盾的组合。
  int ik_decimate{3};

  // ── action 超时与反馈频率 ───────────────────────────────────────────────────
  double feedback_hz{10.0};                      // 三个 action 共用的 feedback 发布频率
  double move_to_pose_timeout_sec{30.0};
  double trajectory_shot_timeout_sec{60.0};
  double move_to_joint_timeout_margin_sec{5.0};  // 超时 = 计划时长 + 余量
  double move_to_joint_timeout_floor_sec{10.0};  // 但不低于这个下限
  double validity_wait_sec{0.3};                 // /check_state_validity 等应答上限

  // ── 固定动作 ────────────────────────────────────────────────────────────────
  double stowed_duration_sec{2.0};       // STOWED 收纳（关节空间直发）时长
  double homing_duration_sec{4.0};       // ArmHoming 回零时长
  double dwell_at_start_sec{1.0};        // 运镜正式开拍前在起点的停留
  // 收纳位 / 回零位（6 轴，rad）。样机机械零点与"该收到哪"未必一致，故可调。
  std::vector<double> stowed_joints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> homing_joints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // 档位键 → 限制（0=SLOW，2=FAST，其余含 1=NORMAL 与非法值都落 NORMAL，与旧行为一致）
  const Speed & speed(uint8_t key) const;
  double joint_speed_rps(uint8_t key) const;
};

/// 进程内唯一实例。declare_and_load() 之前读到的是结构体默认值。
Params & params();

/// 声明并读入全部参数（在节点构造最开始调一次）。越界/尺寸不对的值会退回默认值并告警。
void declare_and_load(rclcpp::Node & node);

}  // namespace robot_arm_node::tuning
