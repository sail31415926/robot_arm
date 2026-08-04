/**
 * @file unwrap_robot_system.cpp
 * @brief UnwrapRobotSystem 实现（设计依据见头文件）
 *
 * @version 1.0
 * @date 2026-07-16
 * @copyright Copyright (c) 2026 EMEET
 */
#include "robot_arm_driver/unwrap_robot_system.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robot_arm_driver
{

namespace
{
constexpr double TWO_PI = 2.0 * M_PI;
}

hardware_interface::CallbackReturn UnwrapRobotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  const auto ret = canopen_ros2_control::RobotSystem::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  unwrap_.assign(robot_motor_data_.size(), Unwrap{});
  saved_targets_.assign(robot_motor_data_.size(),
                        std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < robot_motor_data_.size(); ++i)
  {
    const std::string & name = robot_motor_data_[i].joint_name;
    for (const auto & joint : info_.joints)
    {
      if (joint.name != name)
      {
        continue;
      }
      for (const auto & ci : joint.command_interfaces)
      {
        if (ci.name != hardware_interface::HW_IF_POSITION)
        {
          continue;
        }
        if (ci.min.empty() || ci.max.empty())
        {
          RCLCPP_WARN(robot_system_logger,
                      "'%s' 的 position 命令接口未声明 min/max，折算窗中心取 0",
                      name.c_str());
          break;
        }
        const double lo = std::stod(ci.min);
        const double hi = std::stod(ci.max);
        if (hi - lo >= TWO_PI)
        {
          RCLCPP_ERROR(robot_system_logger,
                       "'%s' 行程 [%.3f, %.3f] ≥ 2π，单圈折算不成立",
                       name.c_str(), lo, hi);
          return hardware_interface::CallbackReturn::ERROR;
        }
        unwrap_[i].center = 0.5 * (lo + hi);
        break;
      }
      break;
    }
    RCLCPP_INFO(robot_system_logger,
                "'%s' 绝对编码器 2π 折算已启用，窗中心 %.3f rad",
                name.c_str(), unwrap_[i].center);
  }
  return ret;
}

hardware_interface::return_type UnwrapRobotSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto ret = canopen_ros2_control::RobotSystem::read(time, period);
  if (ret != hardware_interface::return_type::OK)
  {
    return ret;
  }

  for (size_t i = 0; i < robot_motor_data_.size(); ++i)
  {
    auto & d = robot_motor_data_[i];
    auto & u = unwrap_[i];
    if (!std::isfinite(d.actual_position))
    {
      continue;  // 尚无反馈：保持上一周期 offset
    }
    const double raw = d.actual_position;
    const long k = std::lround((raw - u.center) / TWO_PI);
    if (k != u.k)
    {
      RCLCPP_WARN(robot_system_logger,
                  "'%s' 反馈绕圈折算：raw=%.3f rad → %.3f rad（k=%ld）",
                  d.joint_name.c_str(), raw, raw - TWO_PI * k, k);
      u.k = k;
    }
    u.offset = TWO_PI * static_cast<double>(k);
    d.actual_position = raw - u.offset;
  }
  return ret;
}

hardware_interface::return_type UnwrapRobotSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto ret =
    canopen_ros2_control::RobotSystem::perform_command_mode_switch(
      start_interfaces, stop_interfaces);
  if (ret != hardware_interface::return_type::OK)
  {
    return ret;
  }

  // ── 无冲击换模（2026-08-04 加）─────────────────────────────────────────────
  // 命令接口的值在控制器停用后**原样保留**。速度模式下机械臂会走开，而位置命令缓冲
  // 仍停在进入速度模式前那一刻的值；等切回位置模式（IP），write_target() 立刻把这个
  // 陈旧目标写给驱动器 —— 机械臂会以插补速度冲回旧位置。仿真里表现为瞬移，实物上
  // 就是一次没人预期的高速运动。
  // 这里在位置命令接口被重新 claim 的瞬间，把目标播种成当前实测位置（= 保持不动），
  // 真正的新命令会在下一个控制周期由控制器覆盖。
  for (const auto & iface : start_interfaces)
  {
    for (size_t i = 0; i < robot_motor_data_.size(); ++i)
    {
      auto & d = robot_motor_data_[i];
      if (iface != d.joint_name + "/" + hardware_interface::HW_IF_POSITION)
      {
        continue;
      }
      if (!std::isfinite(d.actual_position))
      {
        RCLCPP_WARN(robot_system_logger,
                    "'%s' 尚无位置反馈，换模无法播种目标（可能出现跳变）",
                    d.joint_name.c_str());
        break;
      }
      d.target_position = d.actual_position;   // 折算坐标系，与上层命令同系
      RCLCPP_INFO(robot_system_logger, "'%s' 换回位置模式，目标已播种为当前位置 %.4f rad",
                  d.joint_name.c_str(), d.actual_position);
      break;
    }
  }
  return ret;
}

hardware_interface::return_type UnwrapRobotSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 上层命令在折算坐标系，写驱动器前平移回其自身坐标系；写完还原命令缓冲，
  // 避免控制器未刷新时偏移逐周期累加。
  for (size_t i = 0; i < robot_motor_data_.size(); ++i)
  {
    saved_targets_[i] = robot_motor_data_[i].target_position;
    if (std::isfinite(saved_targets_[i]) && unwrap_[i].offset != 0.0)
    {
      robot_motor_data_[i].target_position = saved_targets_[i] + unwrap_[i].offset;
    }
  }
  const auto ret = canopen_ros2_control::RobotSystem::write(time, period);
  for (size_t i = 0; i < robot_motor_data_.size(); ++i)
  {
    robot_motor_data_[i].target_position = saved_targets_[i];
  }
  return ret;
}

}  // namespace robot_arm_driver

PLUGINLIB_EXPORT_CLASS(robot_arm_driver::UnwrapRobotSystem, hardware_interface::SystemInterface)
