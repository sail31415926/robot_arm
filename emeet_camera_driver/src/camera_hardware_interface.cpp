/**
 * @file camera_hardware_interface.cpp
 * @brief ros2_control SystemInterface implementation for EMEET PTZ camera
 */

#include "emeet_camera_driver/camera_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(
    emeet_camera::CameraHardwareInterface,
    hardware_interface::SystemInterface)

namespace emeet_camera {

// ── on_init ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
CameraHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto param = [&](const std::string & key, const std::string & def) {
        auto it = info.hardware_parameters.find(key);
        return (it != info.hardware_parameters.end()) ? it->second : def;
    };

    camera_type_ = param("camera_type", "auto");
    sim_mode_    = (param("sim_mode", "false") == "true");

    const size_t n = info_.joints.size();
    pos_.assign(n, 0.0);
    vel_.assign(n, 0.0);
    cmd_pos_.assign(n, 0.0);
    cmd_vel_.assign(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        const auto & name = info_.joints[i].name;
        if (name == "Joint4" || name == "joint4") pan_idx_  = static_cast<int>(i);
        if (name == "Joint6" || name == "joint6") tilt_idx_ = static_cast<int>(i);
    }

    if (pan_idx_ < 0 || tilt_idx_ < 0) {
        RCLCPP_ERROR(logger_,
            "Joint4 (pan) and Joint6 (tilt) must be listed in the <ros2_control> block. "
            "pan_idx=%d  tilt_idx=%d", pan_idx_, tilt_idx_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_,
        "on_init OK — camera_type='%s'  sim_mode=%s  joints=%zu",
        camera_type_.c_str(), sim_mode_ ? "true" : "false", n);
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export interfaces ────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
CameraHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
        si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    }
    return si;
}

std::vector<hardware_interface::CommandInterface>
CameraHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
        ci.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    }
    return ci;
}

// ── lifecycle ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
CameraHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    if (sim_mode_) {
        for (size_t i = 0; i < cmd_pos_.size(); ++i) cmd_pos_[i] = pos_[i];
        RCLCPP_INFO(logger_, "Activated in simulation mode.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    auto result = DeviceDiscovery::discover(camera_type_);
    if (!result) {
        RCLCPP_WARN(logger_,
            "EMEET camera not found (camera_type='%s'). "
            "Activating in no-hardware mode — Joint4-6 will hold zero until camera is connected.",
            camera_type_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hid_ = std::make_unique<HidInterface>(
        result->vid, result->pid, result->hid_device);

    if (!hid_->connect()) {
        RCLCPP_WARN(logger_,
            "HID connect failed at '%s'. "
            "Activating in no-hardware mode. Try: sudo chmod 666 %s",
            result->hid_device.c_str(), result->hid_device.c_str());
        hid_.reset();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hid_->set_tracking_mode(0x00);

    if (!hid_->set_position_absolute(0.0f, 0.0f)) {
        RCLCPP_WARN(logger_, "Gimbal zero-on-activate failed.");
    }

    // Seed command interfaces at zero (matching the zeroing command above)
    cmd_pos_[pan_idx_]  = 0.0;
    cmd_pos_[tilt_idx_] = 0.0;

    RCLCPP_INFO(logger_,
        "Activated — VID=0x%04X PID=0x%04X HID=%s",
        result->vid, result->pid, result->hid_device.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
CameraHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (hid_) {
        hid_->stop_velocity_mode();
        hid_->disconnect();
        hid_.reset();
    }
    RCLCPP_INFO(logger_, "Deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read: open-loop echo ──────────────────────────────────────────────────────
// 不查询 HID — 把上一周期的 cmd 回显为 state，保证 100 Hz 控制循环不阻塞。
// 配合 camera_controller 的 open_loop_control: true 使用。

hardware_interface::return_type
CameraHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (size_t i = 0; i < pos_.size(); ++i) {
        pos_[i] = cmd_pos_[i];
        vel_[i] = cmd_vel_[i];
    }
    return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type
CameraHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (sim_mode_ || !hid_ || !hid_->is_connected()) {
        return hardware_interface::return_type::OK;
    }

    // URDF Joint4/Joint6 axis 为 (0,0,-1)，HID 电机正向与 URDF 视觉方向相反 →
    // 取负使 MoveIt 规划方向与电机实际旋转方向一致。
    const float pan_deg = static_cast<float>(
        std::clamp(-cmd_pos_[pan_idx_] * (180.0 / M_PI),
                   -static_cast<double>(MOTOR_PAN_MAX_DEG),
                    static_cast<double>(MOTOR_PAN_MAX_DEG)));

    const float tilt_deg = static_cast<float>(
        std::clamp(-cmd_pos_[tilt_idx_] * (180.0 / M_PI),
                   -static_cast<double>(MOTOR_TILT_MAX_DEG),
                    static_cast<double>(MOTOR_TILT_MAX_DEG)));

    if (!hid_->set_position_absolute(pan_deg, tilt_deg)) {
        RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
            "HID write failed (pan=%.2f°  tilt=%.2f°)", pan_deg, tilt_deg);
    }
    return hardware_interface::return_type::OK;
}

}  // namespace emeet_camera
