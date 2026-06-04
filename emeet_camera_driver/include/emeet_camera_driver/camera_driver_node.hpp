/**
 * @file camera_driver_node.hpp
 * @brief CameraDriverNode 类声明：EMEET PTZ 摄像头 ROS 2 驱动节点
 *
 * @version 1.0
 * @date 2026-05-29
 * @copyright Copyright (c) 2026 EMEET
 */

#ifndef EMEET_CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
#define EMEET_CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <memory>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "emeet_camera_driver/msg/ptz_state.hpp"

#include "emeet_camera_driver/v4l2_device.hpp"
#include "emeet_camera_driver/hid_interface.hpp"
#include "emeet_camera_driver/device_discovery.hpp"

namespace emeet_camera {

// PTZ hardware limits — position limits sourced from hid_interface.hpp
static constexpr double PAN_MAX_DEG      = MOTOR_PAN_MAX_DEG;
static constexpr double TILT_MAX_DEG     = MOTOR_TILT_MAX_DEG;
// Velocity limits (cmd_vel topic uses rad/s; driver converts to deg/s internally)
static constexpr double VEL_MAX_DPS      = 100.0;
static constexpr double VEL_DEADBAND_DPS =   0.5;
static constexpr double VEL_ZERO_DPS     =  0.01;

class CameraDriverNode : public rclcpp::Node {
public:
    CameraDriverNode();
    ~CameraDriverNode();

private:
    void init();
    void capture_and_publish_display();
    void publish_ptz_state();
    void enforce_tracking_mode_standard_tick();
    void execute_next_trajectory_point();

    // ROS 2 Callbacks
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter> & parameters);
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_compressed_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<emeet_camera_driver::msg::PtzState>::SharedPtr ptz_state_pub_;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr display_timer_;
    rclcpp::TimerBase::SharedPtr ptz_timer_;
    rclcpp::TimerBase::SharedPtr tracking_mode_timer_;
    rclcpp::TimerBase::SharedPtr velocity_timer_;
    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // Hardware
    std::unique_ptr<V4l2Device> display_dev_;
    std::unique_ptr<HidInterface> hid_dev_;

    // State
    DeviceInfo device_info_;
    bool is_running_{false};

    // Display stream parameters
    int display_width_{1920};
    int display_height_{1080};
    int display_fps_{30};
    std::string display_pixel_format_{"MJPEG"};
    std::string display_video_device_{"auto"};
    bool publish_display_raw_{true};
    bool publish_display_compressed_{false};
    bool publish_joint_states_{true};   // false = ros2_control 模式，由 joint_state_broadcaster 接管

    // Tracking mode enforcement (avoid device self-motion / auto-follow)
    bool force_standard_mode_on_startup_{true};
    bool go_to_zero_on_startup_{true};
    uint8_t tracking_mode_standard_byte_{0x00};
    int tracking_mode_attempt_{0};
    int tracking_mode_max_attempts_{3};

    // Absolute position guard: suppress stale cmd_vel after a trajectory command
    double absolute_move_guard_sec_{3.0};
    rclcpp::Time ignore_cmd_vel_until_{0, 0, RCL_ROS_TIME};

    // Multi-point trajectory execution state
    trajectory_msgs::msg::JointTrajectory pending_trajectory_;
    size_t trajectory_point_idx_{0};
    rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};

    // Velocity differentiation for joint_state publication
    rclcpp::Time last_ptz_stamp_{0, 0, RCL_ROS_TIME};
    double last_pan_rad_{0.0};
    double last_tilt_rad_{0.0};
};

}  // namespace emeet_camera

#endif  // EMEET_CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
