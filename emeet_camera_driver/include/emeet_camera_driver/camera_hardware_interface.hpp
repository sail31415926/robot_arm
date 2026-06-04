/**
 * @file camera_hardware_interface.hpp
 * @brief ros2_control SystemInterface for EMEET PTZ camera (HID protocol)
 *
 * 设计说明：
 *   read()  — open-loop echo：将上一周期发出的 cmd_pos_ 回显为 pos_
 *             不查询 HID，消除阻塞，控制循环稳定跑 100 Hz
 *   write() — 调用 set_position_absolute()，发送 HID 绝对位置命令
 *
 * 配合 camera_controller 的 open_loop_control: true 使用：
 *   JTC 不做闭环位置修正，完全按轨迹前馈执行，消除反馈滞后引起的抖动
 *
 * URDF <ros2_control> 参数：
 *   camera_type   "auto" | "pixy" | "e7002" | "piko"  (default: auto)
 *   sim_mode      "true"  — 仅回显命令，不连 HID（Gazebo 仿真用）
 *                 "false" — 连接实物 HID 设备
 *
 * @version 1.1
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <string>
#include <vector>

#include "emeet_camera_driver/device_discovery.hpp"
#include "emeet_camera_driver/hid_interface.hpp"

namespace emeet_camera {

class CameraHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CameraHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::unique_ptr<HidInterface> hid_;
    std::string camera_type_{"auto"};
    bool sim_mode_{false};

    // Per-joint storage, ordered by info_.joints
    std::vector<double> pos_;
    std::vector<double> vel_;
    std::vector<double> cmd_pos_;  // position commands → HID
    std::vector<double> cmd_vel_;  // velocity commands (exported for JTC, not used by HID)

    int pan_idx_{-1};   // index of Joint4 in pos_/cmd_pos_
    int tilt_idx_{-1};  // index of Joint6 in pos_/cmd_pos_

    rclcpp::Logger logger_{rclcpp::get_logger("CameraHardwareInterface")};
    rclcpp::Clock  clock_{RCL_STEADY_TIME};
};

}  // namespace emeet_camera
