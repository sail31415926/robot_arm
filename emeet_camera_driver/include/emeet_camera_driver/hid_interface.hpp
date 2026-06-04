/**
 * @file hid_interface.hpp
 * @brief HidInterface 类声明：EMEET 云台 HID 协议常量、命令及接口定义
 *
 * 协议常量：HID_REPORT_ID, CMD_SET_POSITION_ABS, CMD_SET_VELOCITY 等
 * 电机限位：MOTOR_PAN_MAX_DEG=180°，MOTOR_TILT_MAX_DEG=45°
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#ifndef EMEET_CAMERA_DRIVER__HID_INTERFACE_HPP_
#define EMEET_CAMERA_DRIVER__HID_INTERFACE_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <hidapi/hidapi.h>

namespace emeet_camera {

// HID Protocol Constants (translated from Python)
constexpr uint8_t HID_REPORT_ID = 0x09;
constexpr uint8_t HID_TYPE_DEFAULT = 0x63;
constexpr uint8_t HID_SUBTYPE_DEFAULT = 0x01;

// Command bytes MUST match the validated Pixy/E7002 HID protocol constants:
// - pixy_hw_proto/scripts/protocol/constants.py
// - e7002_hw_proto/scripts/protocol/constants.py
constexpr uint8_t CMD_SET_POSITION_ABS = 0x00;
constexpr uint8_t CMD_GET_POSITION = 0x01;
constexpr uint8_t CMD_GET_STATUS = 0x02;
constexpr uint8_t CMD_SET_DEVICE_MODE = 0x03;  // tracking mode
constexpr uint8_t CMD_SET_VELOCITY = 0x10;
constexpr uint8_t CMD_SET_POSITION_REL = 0x11;
constexpr uint8_t CMD_SET_MOVEMENT_COMBINED = 0x20;  // combined movement (used by set_velocity)

constexpr uint8_t MOTOR_YAW   = 0x01;
constexpr uint8_t MOTOR_PITCH = 0x02;

// Physical travel limits of the pan-tilt mechanism (degrees)
constexpr float MOTOR_PAN_MAX_DEG  = 180.0f;
constexpr float MOTOR_TILT_MAX_DEG =  45.0f;

class HidInterface {
public:
    HidInterface(uint16_t vid, uint16_t pid, const std::string& device_path = "");
    ~HidInterface();

    bool connect();
    void disconnect();
    bool is_connected() const { return handle_ != nullptr; }

    bool set_position_absolute(float yaw_deg, float pitch_deg);
    bool set_position_relative(float yaw_delta, float pitch_delta);
    bool set_velocity(float yaw_dps, float pitch_dps, float zoom_rate = 0.0f);
    bool set_tracking_mode(uint8_t mode); // 0: Standard, 1: Follow

    bool read_report(std::vector<uint8_t>& out_data, int timeout_ms = 50);
    bool get_position(uint8_t motor_id, float& out_target_deg, float& out_current_deg);
    bool get_position_simple(uint8_t motor_id, float& out_current_deg);

    void set_velocity_mode(float yaw_dps, float pitch_dps);
    void stop_velocity_mode();
    void update_velocity_position();

private:
    uint16_t vid_;
    uint16_t pid_;
    std::string device_path_;
    hid_device* handle_;

    bool send_raw_report(const uint8_t* data, size_t length);
    std::vector<uint8_t> build_report(
        uint8_t command,
        const uint8_t* data24,
        size_t data_len,
        size_t total_len = 0,
        size_t current_len = 0);

    // Velocity mode state
    float vel_yaw_dps_{0.0f};
    float vel_pitch_dps_{0.0f};
    float last_pan_deg_{0.0f};
    float last_tilt_deg_{0.0f};
    bool velocity_mode_active_{false};
    std::chrono::steady_clock::time_point last_vel_update_;
};

} // namespace emeet_camera

#endif // EMEET_CAMERA_DRIVER__HID_INTERFACE_HPP_
