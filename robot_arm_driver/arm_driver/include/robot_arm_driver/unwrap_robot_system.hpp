/**
 * @file unwrap_robot_system.hpp
 * @brief UnwrapRobotSystem —— 带单圈绝对编码器 2π 折算的 RobotSystem
 *
 * 背景：RB200-CA 输出轴为 19bit 单圈绝对编码器，上电时 6064 只能给出一圈内的
 * 绝对位置（落在 [0, 2π)，恒非负）。关节物理上停在零点负侧 −θ 时反馈读成
 * +2π−θ，直接喂给 JTC 后任何"走到 0"的轨迹都会朝负方向扫过近一整圈、顶到
 * 物理限位（回零永远同一个方向的根因，2026-07 实机复现）。
 *
 * 方案：继承 canopen_ros2_control::RobotSystem，在 read/write 做**对称**折算：
 *   read : report = raw − 2π·k，k = round((raw − center)/2π)；center 取该关节
 *          URDF position 命令接口 (min+max)/2。各关节行程 < 2π ⇒ k 唯一，
 *          折算边界 center±π 落在物理不可达区（硬限位之外），不会在运动中越界。
 *   write: cmd_raw = cmd + 2π·k（用本周期 read 的 k），命令回到驱动器自身
 *          坐标系 —— 只折算读侧会让 JTC 起点与驱动器坐标差 2π 直接超差。
 * 位置本就在窗内时 k=0，行为与原 RobotSystem 完全一致。
 *
 * 所属：robot_arm_driver（自研定制，不改 vendored ros2_canopen，见 VENDOR.md 约定）
 *
 * @version 1.0
 * @date 2026-07-16
 * @copyright Copyright (c) 2026 EMEET
 */
#pragma once

#include <vector>

#include "canopen_ros2_control/robot_system.hpp"

namespace robot_arm_driver
{

class UnwrapRobotSystem : public canopen_ros2_control::RobotSystem
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// 换模时把位置目标播种为当前实测位置，避免陈旧命令被回放（无冲击换模）
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  /// 每关节折算状态，与 robot_motor_data_ 一一对应
  struct Unwrap
  {
    double center = 0.0;  ///< 折算窗中心 = (min+max)/2，rad
    double offset = 0.0;  ///< 2π·k，read 更新、write 加回
    long   k      = 0;    ///< 当前圈偏移（变化时打日志用）
  };
  std::vector<Unwrap> unwrap_;
  std::vector<double> saved_targets_;  ///< write 期间暂存原命令（避免偏移累加）
};

}  // namespace robot_arm_driver
