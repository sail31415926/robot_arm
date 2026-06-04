/**
 * @file hid_interface.cpp
 * @brief EMEET 云台 HID 协议通信层，封装 hidapi 实现 PTZ 电机控制
 *
 * 功能：
 *   - 设备连接：按 VID/PID 及接口号自动匹配正确的 hidraw 节点
 *   - 位置控制：set_position_absolute / set_position_relative（单位：deg）
 *   - 速度控制：set_velocity_mode + update_velocity_position（100 Hz 位置积分）
 *   - 位置读取：get_position / get_position_simple（单次 HID 请求-应答）
 *   - 追踪模式：set_tracking_mode（Standard=0x00 / Follow=0x01）
 *
 * 支持机型接口号（PID → HID 接口）：
 *   Pixy  0x00C0 → interface 4
 *   E7002 0x00DD → interface 2
 *   Piko  0x0101 → interface 2
 *
 * 电机 ID：
 *   MOTOR_YAW   0x01  Pan（joint4）
 *   MOTOR_PITCH 0x02  Tilt（joint6）
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#include "emeet_camera_driver/hid_interface.hpp"
#include <cstring>
#include <iostream>
#include <vector>

namespace emeet_camera {

HidInterface::HidInterface(uint16_t vid, uint16_t pid, const std::string& device_path)
    : vid_(vid), pid_(pid), device_path_(device_path), handle_(nullptr) {
    hid_init();
}

HidInterface::~HidInterface() {
    disconnect();
    hid_exit();
}

bool HidInterface::connect() {
    if (handle_) return true;

    // 1) Try open_path if caller provided a path hint.
    if (!device_path_.empty()) {
        handle_ = hid_open_path(device_path_.c_str());
        if (handle_) {
            hid_set_nonblocking(handle_, 1);
            return true;
        }
    }

    // 2) Robust fallback: enumerate and open the first matching interface.
    // On some Linux bindings (and in the user's verified environment), the hidapi
    // "path" is NOT the /dev/hidrawX node, so using hid_enumerate is required.
    hid_device_info* devs = hid_enumerate(vid_, pid_);
    hid_device_info* cur = devs;

    const hid_device_info* best = nullptr;
    int target_if = -1;
    if (pid_ == 0x00C0) target_if = 4;      // Pixy PTZ control commonly lives on interface 4
    else if (pid_ == 0x00DD) target_if = 2; // E7002 PTZ control commonly lives on interface 2
    else if (pid_ == 0x0101) target_if = 2; // Piko PTZ control commonly lives on interface 2

    while (cur) {
        if (!best) best = cur;
        if (target_if != -1 && cur->interface_number == target_if) {
            best = cur;
            break;
        }
        cur = cur->next;
    }

    if (best && best->path) {
        handle_ = hid_open_path(best->path);
    }
    hid_free_enumeration(devs);

    // 3) Last resort: plain vid/pid open (may pick a different interface).
    if (!handle_) {
        handle_ = hid_open(vid_, pid_, NULL);
    }

    if (!handle_) return false;
    hid_set_nonblocking(handle_, 1);
    return true;
}

void HidInterface::disconnect() {
    if (handle_) {
        hid_close(handle_);
        handle_ = nullptr;
    }
}

bool HidInterface::send_raw_report(const uint8_t* data, size_t length) {
    if (!handle_) return false;
    int res = hid_write(handle_, data, length);
    if (res < 0) {
        std::cerr << "HID write failed: " << (res) << std::endl;
        return false;
    }
    if (res == 0) {
        std::cerr << "HID write returned 0 (no bytes sent)" << std::endl;
        return false;
    }
    return true;
}

std::vector<uint8_t> HidInterface::build_report(
    uint8_t command,
    const uint8_t* data24,
    size_t data_len,
    size_t total_len,
    size_t current_len) {
    std::vector<uint8_t> report(32, 0);
    if (total_len == 0) total_len = data_len;
    if (current_len == 0) current_len = data_len;

    report[0] = HID_REPORT_ID;
    report[1] = HID_TYPE_DEFAULT;
    report[2] = HID_SUBTYPE_DEFAULT;
    report[3] = command;
    report[4] = 0; // Index
    report[5] = static_cast<uint8_t>(total_len & 0xFF);
    report[6] = static_cast<uint8_t>((total_len >> 8) & 0xFF);
    report[7] = static_cast<uint8_t>(current_len & 0xFF);

    if (data24 && data_len > 0) {
        size_t copy_len = (data_len > 24) ? 24 : data_len;
        memcpy(&report[8], data24, copy_len);
    }
    return report;
}

bool HidInterface::set_position_absolute(float yaw_deg, float pitch_deg) {
    bool success = true;
    
    // Yaw
    {
        uint8_t data[24] = {0};
        data[0] = MOTOR_YAW;
        memcpy(&data[1], &yaw_deg, sizeof(float));
        auto rpt = build_report(CMD_SET_POSITION_ABS, data, 24);
        if (!send_raw_report(rpt.data(), rpt.size())) success = false;
    }

    // Pitch
    {
        uint8_t data[24] = {0};
        data[0] = MOTOR_PITCH;
        memcpy(&data[1], &pitch_deg, sizeof(float));
        auto rpt = build_report(CMD_SET_POSITION_ABS, data, 24);
        if (!send_raw_report(rpt.data(), rpt.size())) success = false;
    }

    return success;
}

bool HidInterface::set_position_relative(float yaw_delta, float pitch_delta) {
    bool success = true;

    // Yaw
    {
        uint8_t data[24] = {0};
        data[0] = MOTOR_YAW;
        memcpy(&data[1], &yaw_delta, sizeof(float));
        auto rpt = build_report(CMD_SET_POSITION_REL, data, 24);
        if (!send_raw_report(rpt.data(), rpt.size())) success = false;
    }

    // Pitch
    {
        uint8_t data[24] = {0};
        data[0] = MOTOR_PITCH;
        memcpy(&data[1], &pitch_delta, sizeof(float));
        auto rpt = build_report(CMD_SET_POSITION_REL, data, 24);
        if (!send_raw_report(rpt.data(), rpt.size())) success = false;
    }

    return success;
}

bool HidInterface::set_velocity(float yaw_dps, float pitch_dps, float zoom_rate) {
    uint8_t data[24] = {0};
    memcpy(&data[0], &yaw_dps, sizeof(float));
    memcpy(&data[4], &pitch_dps, sizeof(float));
    memcpy(&data[8], &zoom_rate, sizeof(float));
    auto rpt = build_report(CMD_SET_MOVEMENT_COMBINED, data, 24);
    return send_raw_report(rpt.data(), rpt.size());
}

bool HidInterface::set_tracking_mode(uint8_t mode) {
    uint8_t data[24] = {0};
    data[0] = mode;
    auto rpt = build_report(CMD_SET_DEVICE_MODE, data, 24, 1, 1);
    return send_raw_report(rpt.data(), rpt.size());
}

bool HidInterface::read_report(std::vector<uint8_t>& out_data, int timeout_ms) {
    if (!handle_) return false;
    out_data.resize(32);
    int res = hid_read_timeout(handle_, out_data.data(), 32, timeout_ms);
    if (res < 0) {
        out_data.clear();
        return false;
    }
    if (res == 0) {
        out_data.clear();
        return false;
    }
    out_data.resize(res);
    return true;
}

bool HidInterface::get_position(uint8_t motor_id, float& out_target_deg, float& out_current_deg) {
    if (!handle_) return false;

    uint8_t data[24] = {0};
    data[0] = motor_id;
    auto rpt = build_report(CMD_GET_POSITION, data, 1);
    if (!send_raw_report(rpt.data(), rpt.size())) return false;

    std::vector<uint8_t> resp;
    if (!read_report(resp, 50)) return false;

    if (resp.size() < 17) return false;
    if (resp[3] != CMD_GET_POSITION) return false;
    if (resp[8] != motor_id) return false;

    float target_pos, current_pos;
    memcpy(&target_pos, &resp[9], sizeof(float));
    memcpy(&current_pos, &resp[13], sizeof(float));

    out_target_deg = target_pos;
    out_current_deg = current_pos;
    return true;
}

bool HidInterface::get_position_simple(uint8_t motor_id, float& out_current_deg) {
    float target, current;
    if (!get_position(motor_id, target, current)) return false;
    out_current_deg = current;
    return true;
}

void HidInterface::set_velocity_mode(float yaw_dps, float pitch_dps) {
    vel_yaw_dps_ = yaw_dps;
    vel_pitch_dps_ = pitch_dps;

    if (!velocity_mode_active_) {
        float pan_deg = 0.0f, tilt_deg = 0.0f;
        if (get_position_simple(MOTOR_YAW, pan_deg)) {
            last_pan_deg_ = pan_deg;
        }
        if (get_position_simple(MOTOR_PITCH, tilt_deg)) {
            last_tilt_deg_ = tilt_deg;
        }
        velocity_mode_active_ = true;
        last_vel_update_ = std::chrono::steady_clock::now();
    }
}

void HidInterface::stop_velocity_mode() {
    vel_yaw_dps_ = 0.0f;
    vel_pitch_dps_ = 0.0f;
    velocity_mode_active_ = false;
}

void HidInterface::update_velocity_position() {
    if (!velocity_mode_active_) return;
    if (std::abs(vel_yaw_dps_) < 0.01f && std::abs(vel_pitch_dps_) < 0.01f) return;

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_vel_update_).count();
    last_vel_update_ = now;

    if (dt <= 0.0f || dt > 0.5f) return;

    last_pan_deg_ += vel_yaw_dps_ * dt;
    last_tilt_deg_ += vel_pitch_dps_ * dt;

    last_pan_deg_  = std::max(-MOTOR_PAN_MAX_DEG,  std::min(MOTOR_PAN_MAX_DEG,  last_pan_deg_));
    last_tilt_deg_ = std::max(-MOTOR_TILT_MAX_DEG, std::min(MOTOR_TILT_MAX_DEG, last_tilt_deg_));

    set_position_absolute(last_pan_deg_, last_tilt_deg_);
}

} // namespace emeet_camera
