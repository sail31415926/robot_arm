/**
 * @file v4l2_device.hpp
 * @brief V4l2Device 类声明：Linux V4L2 摄像头设备抽象接口
 *
 * VideoBuffer 结构体：mmap 缓冲区指针与长度。
 * 支持 MJPEG / YUYV 格式，内存映射零拷贝采集。
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#ifndef EMEET_CAMERA_DRIVER__V4L2_DEVICE_HPP_
#define EMEET_CAMERA_DRIVER__V4L2_DEVICE_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <linux/videodev2.h>

namespace emeet_camera {

struct VideoBuffer {
    void *start;
    size_t length;
    uint32_t index;
};

class V4l2Device {
public:
    explicit V4l2Device(std::string device_path);
    ~V4l2Device();

    bool open_device();
    void close_device();

    bool init_device(int width, int height, uint32_t pixel_format = V4L2_PIX_FMT_MJPEG, int fps = 30);
    bool start_streaming();
    bool stop_streaming();

    /**
     * @brief Dequeue a buffer, copy data, and re-enqueue.
     * @param out_data Vector to store the captured frame data.
     * @return true if capture successful.
     */
    bool capture_frame(std::vector<uint8_t>& out_data);

    bool set_control(uint32_t id, int32_t value);
    int32_t get_control(uint32_t id);

    int get_width() const { return width_; }
    int get_height() const { return height_; }
    uint32_t get_pixel_format() const { return pixel_format_; }

private:
    std::string device_path_;
    int fd_;
    int width_;
    int height_;
    uint32_t pixel_format_;
    int fps_;

    std::vector<VideoBuffer> buffers_;

    bool request_buffers(int count);
    bool init_mmap();
};

} // namespace emeet_camera

#endif // EMEET_CAMERA_DRIVER__V4L2_DEVICE_HPP_
