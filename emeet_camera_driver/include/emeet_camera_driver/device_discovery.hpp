/**
 * @file device_discovery.hpp
 * @brief DeviceDiscovery 类声明：基于 libudev 的 EMEET USB 设备自动发现
 *
 * DeviceInfo 结构体包含 video_device、hid_device、model、vid、pid。
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#ifndef EMEET_CAMERA_DRIVER__DEVICE_DISCOVERY_HPP_
#define EMEET_CAMERA_DRIVER__DEVICE_DISCOVERY_HPP_

#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace emeet_camera {

struct DeviceInfo {
    std::string video_device;
    std::string hid_device;
    std::string model; // pixy, e7002, piko
    uint16_t vid;
    uint16_t pid;
};

class DeviceDiscovery {
public:
    /**
     * @brief Discover EMEET devices based on target model or auto-detect.
     * @param target_model "pixy", "e7002", "piko", or "auto"
     * @return DeviceInfo if found, std::nullopt otherwise.
     */
    static std::optional<DeviceInfo> discover(const std::string& target_model = "auto");

    static std::string findVideoDeviceByVidPid(uint16_t vid, uint16_t pid);
    // Some devices expose multiple capture nodes (e.g. dual lens). Return all capture-capable /dev/video*.
    static std::vector<std::string> findVideoDevicesByVidPid(uint16_t vid, uint16_t pid);
    static std::string findHidDeviceByVidPid(uint16_t vid, uint16_t pid);
    static std::string findVideoDeviceByName(const std::string& name_substring);

};

} // namespace emeet_camera

#endif // EMEET_CAMERA_DRIVER__DEVICE_DISCOVERY_HPP_
