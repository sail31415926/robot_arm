/**
 * @file camera_driver_node.cpp
 * @brief EMEET PTZ 摄像头 ROS 2 驱动节点，集成 V4L2 视频流与 HID 云台控制
 *
 * 功能：
 *   - V4L2 采集：MJPEG / YUYV，支持 raw 与 compressed 两路发布
 *   - HID PTZ 控制：绝对位置模式 + 速度积分模式（100 Hz 更新）
 *   - 关节状态发布：位置差分估算速度，10 Hz
 *   - 多点轨迹执行：按 time_from_start 依次发送 HID 位置命令
 *   - 启动时强制 Standard 追踪模式，防止云台自动跟随
 *
 * 话题接口：
 *   发布  /joint_states        sensor_msgs/JointState     rad / rad/s
 *   发布  ~/state              emeet_camera_driver/PtzState  rad
 *   发布  /camera/image_raw/compressed  CompressedImage
 *   发布  ~/image_raw          sensor_msgs/Image          BGR8
 *   订阅  ~/cmd_vel            geometry_msgs/Twist        angular.z=pan rad/s, angular.y=tilt rad/s
 *   订阅  ~/joint_trajectory   trajectory_msgs/JointTrajectory  rad
 *
 * 关键参数（见 config/params.yaml）：
 *   camera_type, video_device, width, height, fps, pixel_format
 *   publish_display_raw, publish_display_compressed
 *   go_to_zero_on_startup, force_standard_mode_on_startup
 *   absolute_move_guard_sec
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */


#include "emeet_camera_driver/camera_driver_node.hpp"
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace emeet_camera {

CameraDriverNode::CameraDriverNode() : Node("emeet_camera_node")
{
    // ── Parameters ────────────────────────────────────────────────────────
    this->declare_parameter("camera_type", "auto");
    this->declare_parameter("video_device", "auto");
    this->declare_parameter("width", 1920);
    this->declare_parameter("height", 1080);
    this->declare_parameter("fps", 30);
    this->declare_parameter("pixel_format", "MJPEG");
    this->declare_parameter("publish_display_raw", true);
    this->declare_parameter("publish_display_compressed", false);
    this->declare_parameter("brightness", 128);
    this->declare_parameter("exposure_value", 156);
    this->declare_parameter("go_to_zero_on_startup", true);
    this->declare_parameter("force_standard_mode_on_startup", true);
    this->declare_parameter("tracking_mode_standard_byte", 0);
    this->declare_parameter("tracking_mode_set_delay_sec", 2.5);
    this->declare_parameter("tracking_mode_set_retries", 3);
    this->declare_parameter("tracking_mode_set_retry_interval_sec", 1.5);
    this->declare_parameter("absolute_move_guard_sec", 3.0);
    // 与 ros2_control 联用时设 false，由 joint_state_broadcaster 接管关节状态发布
    // 用字符串声明，避免 launch LaunchConfiguration 传入 'true'/'false' 字符串时类型不匹配
    this->declare_parameter<std::string>("publish_joint_states", "true");
    // true = 完全跳过 HID 初始化，仅做视频流（与 ros2_control HID 插件共存时必须设 true）
    this->declare_parameter<std::string>("disable_hid", "false");

    // ── Publishers ────────────────────────────────────────────────────────
    auto qos = rclcpp::SensorDataQoS();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    image_pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/image_raw/compressed", 10);
    joint_pub_     = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    ptz_state_pub_ = this->create_publisher<emeet_camera_driver::msg::PtzState>("~/state", 10);

    // ── Subscriptions ─────────────────────────────────────────────────────
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", 10,
        std::bind(&CameraDriverNode::cmd_vel_cb, this, std::placeholders::_1));
    joint_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "~/joint_trajectory", 10,
        std::bind(&CameraDriverNode::joint_trajectory_cb, this, std::placeholders::_1));

    init();

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CameraDriverNode::on_set_parameters, this, std::placeholders::_1));
}

CameraDriverNode::~CameraDriverNode()
{
    if (velocity_timer_)    velocity_timer_->cancel();
    if (tracking_mode_timer_) tracking_mode_timer_->cancel();
    if (display_timer_)     display_timer_->cancel();
    if (trajectory_timer_)  trajectory_timer_->cancel();
    if (hid_dev_) {
        hid_dev_->stop_velocity_mode();
        hid_dev_->disconnect();
    }
    if (display_dev_) display_dev_->close_device();
}

// ── Initialisation ──────────────────────────────────────────────────────────
void CameraDriverNode::init()
{
    const std::string type = this->get_parameter("camera_type").as_string();
    display_video_device_   = this->get_parameter("video_device").as_string();
    display_width_          = this->get_parameter("width").as_int();
    display_height_         = this->get_parameter("height").as_int();
    display_fps_            = this->get_parameter("fps").as_int();
    display_pixel_format_   = this->get_parameter("pixel_format").as_string();
    publish_display_raw_        = this->get_parameter("publish_display_raw").as_bool();
    publish_display_compressed_ = this->get_parameter("publish_display_compressed").as_bool();
    absolute_move_guard_sec_    = this->get_parameter("absolute_move_guard_sec").as_double();
    publish_joint_states_       = (this->get_parameter("publish_joint_states").as_string() != "false");
    ignore_cmd_vel_until_       = this->now();

    const uint32_t pixel_format = (display_pixel_format_ == "YUYV")
        ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;

    auto discovery_res = DeviceDiscovery::discover(type);
    if (!discovery_res) {
        RCLCPP_ERROR(this->get_logger(), "Failed to discover EMEET camera (type='%s')", type.c_str());
        return;
    }
    device_info_ = *discovery_res;
    RCLCPP_INFO(this->get_logger(),
        "Detected camera: %s  VID=0x%04X  PID=0x%04X  Video=%s  HID=%s",
        device_info_.model.c_str(), device_info_.vid, device_info_.pid,
        device_info_.video_device.c_str(), device_info_.hid_device.c_str());

    force_standard_mode_on_startup_ = this->get_parameter("force_standard_mode_on_startup").as_bool();
    tracking_mode_standard_byte_    = static_cast<uint8_t>(
        this->get_parameter("tracking_mode_standard_byte").as_int());
    tracking_mode_max_attempts_ = std::max(1,
        static_cast<int>(this->get_parameter("tracking_mode_set_retries").as_int()));
    tracking_mode_attempt_  = 0;
    go_to_zero_on_startup_  = this->get_parameter("go_to_zero_on_startup").as_bool();

    // ── HID / PTZ ─────────────────────────────────────────────────────────
    // disable_hid=true 时跳过 HID 初始化，仅做视频流（与 ros2_control 插件共存必须设此项）
    const bool disable_hid = (this->get_parameter("disable_hid").as_string() == "true");
    if (disable_hid) {
        RCLCPP_INFO(this->get_logger(),
            "HID disabled (disable_hid=true). PTZ controlled by ros2_control plugin.");
    } else {
    hid_dev_ = std::make_unique<HidInterface>(
        device_info_.vid, device_info_.pid, device_info_.hid_device);

    if (hid_dev_->connect()) {
        RCLCPP_INFO(this->get_logger(), "HID connected.");

        if (force_standard_mode_on_startup_) {
            enforce_tracking_mode_standard_tick();
        }
        if (go_to_zero_on_startup_) {
            if (hid_dev_->set_position_absolute(0.0f, 0.0f)) {
                RCLCPP_INFO(this->get_logger(), "Gimbal zeroing command sent.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send gimbal zeroing command.");
            }
        }

        ptz_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CameraDriverNode::publish_ptz_state, this));

        velocity_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { if (hid_dev_) hid_dev_->update_velocity_position(); });

        // Deferred tracking-mode retry loop
        if (force_standard_mode_on_startup_ &&
            tracking_mode_attempt_ < tracking_mode_max_attempts_)
        {
            const double delay_sec = this->get_parameter("tracking_mode_set_delay_sec").as_double();
            const double retry_sec = this->get_parameter("tracking_mode_set_retry_interval_sec").as_double();
            const auto first_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.0, delay_sec)));
            const auto retry_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.1, retry_sec)));

            // One-shot timer → after firing, installs the repeating retry timer
            auto one_shot = this->create_wall_timer(first_delay, [this, retry_period]() {
                tracking_mode_timer_->cancel();  // cancel one-shot
                enforce_tracking_mode_standard_tick();
                // Install repeating retry
                tracking_mode_timer_ = this->create_wall_timer(
                    retry_period,
                    std::bind(&CameraDriverNode::enforce_tracking_mode_standard_tick, this));
            });
            tracking_mode_timer_ = one_shot;  // keep one-shot alive via member variable

            RCLCPP_INFO(this->get_logger(),
                "Tracking mode retry: delay=%.1fs, interval=%.1fs, max=%d",
                delay_sec, retry_sec, tracking_mode_max_attempts_);
        }
    } else {
        if (device_info_.hid_device.empty()) {
            RCLCPP_WARN(this->get_logger(), "No HID interface found. PTZ disabled.");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to connect HID at %s. Check permissions (sudo chmod 666 %s).",
                device_info_.hid_device.c_str(), device_info_.hid_device.c_str());
        }
    }
    }  // end: if (!disable_hid)

    // ── V4L2 display stream ───────────────────────────────────────────────
    const std::string display_path = (display_video_device_ != "auto")
        ? display_video_device_ : device_info_.video_device;

    display_dev_ = std::make_unique<V4l2Device>(display_path);
    if (display_dev_->open_device() &&
        display_dev_->init_device(display_width_, display_height_, pixel_format, display_fps_) &&
        display_dev_->start_streaming())
    {
        RCLCPP_INFO(this->get_logger(),
            "Display streaming: %s  %dx%d @ %d FPS  fmt=%s",
            display_path.c_str(), display_width_, display_height_,
            display_fps_, display_pixel_format_.c_str());
        is_running_ = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to start display stream!");
        return;
    }

    const int timer_ms = std::max(1, 1000 / std::max(1, display_fps_));
    display_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&CameraDriverNode::capture_and_publish_display, this));
}

// ── Tracking mode enforcement ────────────────────────────────────────────────
void CameraDriverNode::enforce_tracking_mode_standard_tick()
{
    if (!hid_dev_ || !hid_dev_->is_connected() || !force_standard_mode_on_startup_) {
        if (tracking_mode_timer_) tracking_mode_timer_->cancel();
        return;
    }
    if (tracking_mode_attempt_ >= tracking_mode_max_attempts_) {
        RCLCPP_INFO(this->get_logger(),
            "Tracking mode enforcement done (%d attempts).", tracking_mode_attempt_);
        if (tracking_mode_timer_) tracking_mode_timer_->cancel();
        return;
    }
    ++tracking_mode_attempt_;
    const bool ok = hid_dev_->set_tracking_mode(tracking_mode_standard_byte_);
    RCLCPP_INFO(this->get_logger(),
        "SET_DEVICE_MODE attempt %d/%d  mode=0x%02X  ok=%s",
        tracking_mode_attempt_, tracking_mode_max_attempts_,
        tracking_mode_standard_byte_, ok ? "true" : "false");

    if (tracking_mode_attempt_ >= tracking_mode_max_attempts_) {
        if (tracking_mode_timer_) tracking_mode_timer_->cancel();
    }
}

// ── Video capture ────────────────────────────────────────────────────────────
static bool decode_to_bgr(const std::vector<uint8_t>& data, uint32_t fmt,
                           int w, int h, cv::Mat& out)
{
    if (fmt == V4L2_PIX_FMT_MJPEG) {
        out = cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR);
        return !out.empty();
    }
    if (fmt == V4L2_PIX_FMT_YUYV) {
        if (data.size() < static_cast<size_t>(w * h * 2)) return false;
        cv::Mat yuyv(h, w, CV_8UC2, const_cast<uint8_t*>(data.data()));
        cv::cvtColor(yuyv, out, cv::COLOR_YUV2BGR_YUYV);
        return !out.empty();
    }
    return false;
}

void CameraDriverNode::capture_and_publish_display()
{
    if (!is_running_) return;

    std::vector<uint8_t> frame_data;
    if (!display_dev_ || !display_dev_->capture_frame(frame_data)) return;

    auto header = std_msgs::msg::Header{};
    header.stamp    = this->now();
    header.frame_id = "camera_optical_frame";

    if (publish_display_compressed_) {
        sensor_msgs::msg::CompressedImage c;
        c.header = header;
        c.format = (display_dev_->get_pixel_format() == V4L2_PIX_FMT_MJPEG) ? "jpeg" : "yuv422";
        c.data   = frame_data;
        image_pub_compressed_->publish(c);
    }

    if (publish_display_raw_) {
        cv::Mat frame;
        if (!decode_to_bgr(frame_data, display_dev_->get_pixel_format(),
                           display_dev_->get_width(), display_dev_->get_height(), frame)) return;

        sensor_msgs::msg::Image msg;
        msg.header       = header;
        msg.height       = frame.rows;
        msg.width        = frame.cols;
        msg.encoding     = "bgr8";
        msg.is_bigendian = false;
        msg.step         = static_cast<uint32_t>(frame.step);
        msg.data.assign(frame.data, frame.data + frame.step * frame.rows);
        image_pub_->publish(msg);
    }
}

// ── PTZ state publisher (10 Hz) ──────────────────────────────────────────────
void CameraDriverNode::publish_ptz_state()
{
    if (!hid_dev_ || !hid_dev_->is_connected()) return;

    float pan_deg = 0.0f, tilt_deg = 0.0f;
    if (!hid_dev_->get_position_simple(MOTOR_YAW,   pan_deg))  return;
    if (!hid_dev_->get_position_simple(MOTOR_PITCH, tilt_deg)) return;

    const double pan_rad  = pan_deg  * (M_PI / 180.0);
    const double tilt_rad = tilt_deg * (M_PI / 180.0);

    // Differentiate position to fill velocity field
    sensor_msgs::msg::JointState js;
    js.header.stamp    = this->now();
    js.header.frame_id = "camera_ptz_base";
    js.name = {"Joint4", "Joint6"};
    js.position = {pan_rad, tilt_rad};

    const rclcpp::Time now_stamp(js.header.stamp);
    const double dt = (now_stamp - last_ptz_stamp_).seconds();
    if (dt > 0.01 && last_ptz_stamp_.nanoseconds() > 0) {
        js.velocity = {
            (pan_rad  - last_pan_rad_)  / dt,
            (tilt_rad - last_tilt_rad_) / dt
        };
    } else {
        js.velocity = {0.0, 0.0};
    }
    last_ptz_stamp_ = now_stamp;
    last_pan_rad_   = pan_rad;
    last_tilt_rad_  = tilt_rad;

    if (publish_joint_states_) {
        joint_pub_->publish(js);
    }

    emeet_camera_driver::msg::PtzState ptz;
    ptz.pan  = static_cast<float>(pan_rad);
    ptz.tilt = static_cast<float>(tilt_rad);
    ptz.zoom = 0.0f;
    ptz_state_pub_->publish(ptz);
}

// ── cmd_vel handler ──────────────────────────────────────────────────────────
void CameraDriverNode::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (this->now() < ignore_cmd_vel_until_) return;
    if (!hid_dev_ || !hid_dev_->is_connected()) return;

    // Input: rad/s (ROS convention) → convert to deg/s for HID protocol
    double yaw   = msg->angular.z * (180.0 / M_PI);
    double pitch = msg->angular.y * (180.0 / M_PI);

    if (std::abs(yaw)   < VEL_DEADBAND_DPS) yaw   = 0.0;
    if (std::abs(pitch) < VEL_DEADBAND_DPS) pitch = 0.0;
    yaw   = std::clamp(yaw,   -VEL_MAX_DPS, VEL_MAX_DPS);
    pitch = std::clamp(pitch, -VEL_MAX_DPS, VEL_MAX_DPS);

    if (std::abs(yaw) < VEL_ZERO_DPS && std::abs(pitch) < VEL_ZERO_DPS) {
        hid_dev_->stop_velocity_mode();
    } else {
        hid_dev_->set_velocity_mode(static_cast<float>(yaw), static_cast<float>(pitch));
    }
}

// ── Joint trajectory handler ─────────────────────────────────────────────────
void CameraDriverNode::joint_trajectory_cb(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty() || msg->joint_names.empty()) return;
    if (!hid_dev_ || !hid_dev_->is_connected()) return;

    hid_dev_->stop_velocity_mode();
    ignore_cmd_vel_until_ = this->now() +
        rclcpp::Duration::from_seconds(std::max(0.0, absolute_move_guard_sec_));

    // Cancel any ongoing trajectory
    if (trajectory_timer_) trajectory_timer_->cancel();
    pending_trajectory_    = *msg;
    trajectory_point_idx_  = 0;
    trajectory_start_time_ = this->now();

    // Execute the first point immediately
    execute_next_trajectory_point();

    // Schedule remaining points at 10 ms resolution
    if (pending_trajectory_.points.size() > 1) {
        trajectory_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CameraDriverNode::execute_next_trajectory_point, this));
    }
}

void CameraDriverNode::execute_next_trajectory_point()
{
    if (trajectory_point_idx_ >= pending_trajectory_.points.size()) {
        if (trajectory_timer_) trajectory_timer_->cancel();
        return;
    }

    const auto& point = pending_trajectory_.points[trajectory_point_idx_];

    // First point always executes immediately (preserves original behavior).
    // Subsequent points wait until their time_from_start has elapsed.
    if (trajectory_point_idx_ > 0) {
        const rclcpp::Duration tfs(
            point.time_from_start.sec,
            static_cast<uint32_t>(point.time_from_start.nanosec));
        if (this->now() < trajectory_start_time_ + tfs) return;
    }

    double pan_rad = 0.0, tilt_rad = 0.0;
    for (size_t i = 0; i < pending_trajectory_.joint_names.size(); ++i) {
        if (i >= point.positions.size()) break;
        if (pending_trajectory_.joint_names[i] == "Joint4") {
            pan_rad  = std::clamp(point.positions[i],
                           -PAN_MAX_DEG  * (M_PI / 180.0),
                            PAN_MAX_DEG  * (M_PI / 180.0));
        } else if (pending_trajectory_.joint_names[i] == "Joint6") {
            tilt_rad = std::clamp(point.positions[i],
                           -TILT_MAX_DEG * (M_PI / 180.0),
                            TILT_MAX_DEG * (M_PI / 180.0));
        }
    }

    // HID interface uses degrees internally
    const float pan_deg_f  = static_cast<float>(pan_rad  * (180.0 / M_PI));
    const float tilt_deg_f = static_cast<float>(tilt_rad * (180.0 / M_PI));

    if (hid_dev_->set_position_absolute(pan_deg_f, tilt_deg_f))
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Trajectory point %zu/%zu: pan=%.3f rad  tilt=%.3f rad",
            trajectory_point_idx_ + 1, pending_trajectory_.points.size(),
            pan_rad, tilt_rad);
    } else {
        RCLCPP_WARN(this->get_logger(), "Trajectory: HID write failed at point %zu",
            trajectory_point_idx_);
    }

    ++trajectory_point_idx_;
    if (trajectory_point_idx_ >= pending_trajectory_.points.size()) {
        if (trajectory_timer_) trajectory_timer_->cancel();
    }
}

// ── Parameter callback ────────────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult CameraDriverNode::on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : parameters) {
        if (p.get_name() == "brightness") {
            if (display_dev_) display_dev_->set_control(V4L2_CID_BRIGHTNESS, p.as_int());
        } else if (p.get_name() == "exposure_value") {
            if (display_dev_) display_dev_->set_control(V4L2_CID_EXPOSURE_ABSOLUTE, p.as_int());
        }
    }
    return result;
}

}  // namespace emeet_camera

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<emeet_camera::CameraDriverNode>());
    rclcpp::shutdown();
    return 0;
}
