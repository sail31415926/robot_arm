/**
 * @file v4l2_device.cpp
 * @brief Linux V4L2 摄像头设备抽象，封装开流/采帧/参数设置全流程
 *
 * 功能：
 *   - 设备管理：open_device / close_device，非阻塞模式（O_NONBLOCK）
 *   - 格式初始化：init_device 设置分辨率、像素格式（MJPEG / YUYV）、帧率
 *   - 内存映射缓冲区：request_buffers + init_mmap，默认 4 个缓冲区
 *   - 流控制：start_streaming（VIDIOC_STREAMON）/ stop_streaming（VIDIOC_STREAMOFF + munmap）
 *   - 帧采集：capture_frame 出队 → 复制数据 → 入队，EAGAIN 静默跳过
 *   - 参数控制：set_control / get_control（亮度、曝光等 V4L2 CID）
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#include "emeet_camera_driver/v4l2_device.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>

namespace emeet_camera {

V4l2Device::V4l2Device(std::string device_path)
    : device_path_(std::move(device_path)), fd_(-1), width_(0), height_(0), pixel_format_(0), fps_(0) {}

V4l2Device::~V4l2Device() {
    close_device();
}

bool V4l2Device::open_device() {
    fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
        std::cerr << "Cannot open " << device_path_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct v4l2_capability cap;
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        std::cerr << "VIDIOC_QUERYCAP failed on " << device_path_ << ": " << strerror(errno) << std::endl;
        close(fd_);
        fd_ = -1;
        return false;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << device_path_ << " does not support VIDEO_CAPTURE" << std::endl;
        close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void V4l2Device::close_device() {
    stop_streaming();
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool V4l2Device::init_device(int width, int height, uint32_t pixel_format, int fps) {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "VIDIOC_S_FMT failed: " << strerror(errno) << std::endl;
        return false;
    }

    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    pixel_format_ = fmt.fmt.pix.pixelformat;

    struct v4l2_streamparm streamparm;
    memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = fps;

    if (ioctl(fd_, VIDIOC_S_PARM, &streamparm) < 0) {
        std::cerr << "VIDIOC_S_PARM failed: " << strerror(errno) << std::endl;
    }
    fps_ = streamparm.parm.capture.timeperframe.denominator / streamparm.parm.capture.timeperframe.numerator;

    return request_buffers(4);
}

bool V4l2Device::request_buffers(int count) {
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << std::endl;
        return false;
    }

    if (req.count < 2) {
        std::cerr << "Insufficient buffer memory" << std::endl;
        return false;
    }

    buffers_.resize(req.count);
    return init_mmap();
}

bool V4l2Device::init_mmap() {
    for (uint32_t i = 0; i < buffers_.size(); ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QUERYBUF failed: " << strerror(errno) << std::endl;
            return false;
        }

        buffers_[i].length = buf.length;
        buffers_[i].index = i;
        buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

        if (buffers_[i].start == MAP_FAILED) {
            std::cerr << "mmap failed: " << strerror(errno) << std::endl;
            return false;
        }
    }
    return true;
}

bool V4l2Device::start_streaming() {
    for (uint32_t i = 0; i < buffers_.size(); ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << std::endl;
            return false;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "VIDIOC_STREAMON failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool V4l2Device::stop_streaming() {
    if (fd_ < 0) return true;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);

    for (auto& buffer : buffers_) {
        if (buffer.start) {
            munmap(buffer.start, buffer.length);
            buffer.start = nullptr;
        }
    }
    buffers_.clear();
    return true;
}

bool V4l2Device::capture_frame(std::vector<uint8_t>& out_data) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN) {
            std::cerr << "VIDIOC_DQBUF failed: " << strerror(errno) << std::endl;
        }
        return false;
    }

    if (buf.bytesused == 0) {
        // Empty buffer, re-enqueue and skip
        ioctl(fd_, VIDIOC_QBUF, &buf);
        return false;
    }

    out_data.assign((uint8_t*)buffers_[buf.index].start, (uint8_t*)buffers_[buf.index].start + buf.bytesused);

    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        std::cerr << "VIDIOC_QBUF failed (buffer slot " << buf.index << "): " << strerror(errno) << std::endl;
        // Retry once
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF retry failed, buffer slot " << buf.index << " leaked" << std::endl;
            return false;
        }
    }

    return true;
}

bool V4l2Device::set_control(uint32_t id, int32_t value) {
    struct v4l2_control ctrl;
    ctrl.id = id;
    ctrl.value = value;
    if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        std::cerr << "VIDIOC_S_CTRL failed for id " << id << ": " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

int32_t V4l2Device::get_control(uint32_t id) {
    struct v4l2_control ctrl;
    ctrl.id = id;
    if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) < 0) {
        std::cerr << "VIDIOC_G_CTRL failed for id " << id << ": " << strerror(errno) << std::endl;
        return 0;
    }
    return ctrl.value;
}

} // namespace emeet_camera
