/**
 * @file device_discovery.cpp
 * @brief 基于 libudev 的 EMEET USB 设备自动发现，返回 V4L2 视频节点与 HID 控制节点路径
 *
 * 功能：
 *   - discover()：按型号（pixy / e7002 / piko / auto）枚举 USB 设备
 *   - findVideoDevicesByVidPid()：匹配 ID_V4L_CAPABILITIES=capture 过滤非采集节点
 *   - findVideoDeviceByVidPid()：返回首个采集节点（复用上函数）
 *   - findHidDeviceByVidPid()：遍历 hidraw 子系统，匹配父级 VID/PID
 *   - findVideoDeviceByName()：按 ID_MODEL 模糊匹配视频节点
 *
 * 已知 VID/PID：
 *   VID 0x328F（EMEET）
 *   Pixy  PID 0x00C0
 *   E7002 PID 0x00DD
 *   Piko  PID 0x0101
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#include "emeet_camera_driver/device_discovery.hpp"
#include <libudev.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <algorithm>

namespace emeet_camera {

const uint16_t EMEET_VID = 0x328F;
const uint16_t PIXY_PID = 0x00C0;
const uint16_t E7002_PID = 0x00DD;
const uint16_t PIKO_PID = 0x0101;

std::optional<DeviceInfo> DeviceDiscovery::discover(const std::string& target_model) {
    std::vector<std::string> models_to_check;
    if (target_model == "auto") {
        models_to_check = {"pixy", "e7002", "piko"};
    } else {
        models_to_check = {target_model};
    }

    for (const auto& model : models_to_check) {
        DeviceInfo info;
        info.model = model;
        if (model == "pixy") {
            info.vid = EMEET_VID;
            info.pid = PIXY_PID;
            info.video_device = findVideoDeviceByVidPid(info.vid, info.pid);
            info.hid_device = findHidDeviceByVidPid(info.vid, info.pid);
        } else if (model == "e7002") {
            info.vid = EMEET_VID;
            info.pid = E7002_PID;
            info.video_device = findVideoDeviceByVidPid(info.vid, info.pid);
            info.hid_device = findHidDeviceByVidPid(info.vid, info.pid);
        } else if (model == "piko") {
            info.vid = EMEET_VID;
            info.pid = PIKO_PID;
            info.video_device = findVideoDeviceByVidPid(info.vid, info.pid);
            // Piko Dual might have HID, let's try to find it
            info.hid_device = findHidDeviceByVidPid(info.vid, info.pid);
        }

        if (!info.video_device.empty()) {
            return info;
        }
    }

    return std::nullopt;
}

std::string DeviceDiscovery::findVideoDeviceByVidPid(uint16_t vid, uint16_t pid) {
    auto results = findVideoDevicesByVidPid(vid, pid);
    return results.empty() ? "" : results.front();
}

std::vector<std::string> DeviceDiscovery::findVideoDevicesByVidPid(uint16_t vid, uint16_t pid) {
    struct udev *udev = udev_new();
    if (!udev) return {};

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;

    std::vector<std::string> results;

    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        struct udev_device *usb_dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if (usb_dev) {
            const char *vendor = udev_device_get_sysattr_value(usb_dev, "idVendor");
            const char *product = udev_device_get_sysattr_value(usb_dev, "idProduct");

            if (vendor && product) {
                uint16_t v = std::stoi(vendor, nullptr, 16);
                uint16_t p = std::stoi(product, nullptr, 16);

                if (v == vid && p == pid) {
                    const char *devnode = udev_device_get_devnode(dev);
                    if (devnode) {
                        const char *caps = udev_device_get_property_value(dev, "ID_V4L_CAPABILITIES");
                        if (caps) {
                            std::string cap_str = caps;
                            if (cap_str.find("capture") != std::string::npos) {
                                results.emplace_back(devnode);
                            }
                        } else {
                            // Fallback: if capabilities not available, include it and let V4L2 open fail fast.
                            results.emplace_back(devnode);
                        }
                    }
                }
            }
        }
        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    // stable order, remove duplicates
    std::sort(results.begin(), results.end());
    results.erase(std::unique(results.begin(), results.end()), results.end());
    return results;
}

std::string DeviceDiscovery::findHidDeviceByVidPid(uint16_t vid, uint16_t pid) {
    struct udev *udev = udev_new();
    if (!udev) return "";

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "hidraw");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;

    std::string result = "";

    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        // More robust traversal: check all parents for VID/PID
        struct udev_device *parent = dev;
        bool found = false;
        while (parent) {
            const char *vendor = udev_device_get_sysattr_value(parent, "idVendor");
            const char *product = udev_device_get_sysattr_value(parent, "idProduct");

            if (vendor && product) {
                try {
                    uint16_t v = std::stoi(vendor, nullptr, 16);
                    uint16_t p = std::stoi(product, nullptr, 16);
                    if (v == vid && p == pid) {
                        const char *devnode = udev_device_get_devnode(dev);
                        if (devnode) {
                            result = devnode;
                            found = true;
                            break;
                        }
                    }
                } catch (...) {
                    // Ignore stoi errors
                }
            }
            parent = udev_device_get_parent(parent);
        }

        if (!found) {
            // Fallback: check properties directly on the hidraw device
            const char *vendor_id = udev_device_get_property_value(dev, "ID_VENDOR_ID");
            const char *model_id = udev_device_get_property_value(dev, "ID_MODEL_ID");
            if (vendor_id && model_id) {
                try {
                    uint16_t v = std::stoi(vendor_id, nullptr, 16);
                    uint16_t p = std::stoi(model_id, nullptr, 16);
                    if (v == vid && p == pid) {
                        const char *devnode = udev_device_get_devnode(dev);
                        if (devnode) {
                            result = devnode;
                            found = true;
                        }
                    }
                } catch (...) {}
            }
        }

        udev_device_unref(dev);
        if (found) break;
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return result;
}

std::string DeviceDiscovery::findVideoDeviceByName(const std::string& name_substring) {
    struct udev *udev = udev_new();
    if (!udev) return "";

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;

    std::string result = "";
    std::string target = name_substring;
    std::transform(target.begin(), target.end(), target.begin(), ::tolower);

    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        const char *model = udev_device_get_property_value(dev, "ID_MODEL");
        if (model) {
            std::string model_str = model;
            std::transform(model_str.begin(), model_str.end(), model_str.begin(), ::tolower);
            if (model_str.find(target) != std::string::npos) {
                const char *devnode = udev_device_get_devnode(dev);
                if (devnode) {
                    result = devnode;
                    udev_device_unref(dev);
                    break;
                }
            }
        }
        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return result;
}

} // namespace emeet_camera
