/**
 * @file arm_hardware_interface.hpp
 * @brief ros2_control SystemInterface for eMeetArm J1-3（CANopen / CiA402）
 *
 * 把原 arm_node.cpp 里已实测的 CANopen 逻辑重挂到 ros2_control 生命周期，
 * 让实物臂 J1-3 由标准 JointTrajectoryController 驱动。对标 robot_gimbal_driver
 * 的 CameraHardwareInterface（同为 SystemInterface + sim_mode 开关）。
 *
 * 设计要点：
 *   - read()  不直接做 SDO：由后台线程（feedback_fast_ms，默认 50Hz）轮询
 *             getPosition/getVelocity 缓存，read() 只拷缓存 → 不阻塞 100Hz 控制环。
 *   - write() IP 模式：逐关节 sendInterpolationData + 一条 sendSYNC（多轴同步）；
 *             PV 模式：setTargetVelocity。插补由 JTC 完成，HAL 只流式下发。
 *   - prepare/perform_command_mode_switch：position↔velocity 接口切换 → DS402 IP↔PV。
 *   - 每个 CanopenMotorDriver 独占一个 mutex，串行化该轴 CAN 访问（后台反馈线程
 *     与控制环并发，SDO 读与 PDO/SDO 写不能在同一 fd 上竞争）。
 *   - sim_mode=true：不连 CAN，read() 回显 cmd → state（干跑 / 无硬件调试）。
 *
 * URDF <ros2_control> 硬件参数（<hardware><param>）：
 *   can_interface / sdo_timeout_ms / master_node_id / heartbeat_ms /
 *   feedback_fast_ms / ip_period_ms / enable_stagger_ms / pp_accel / pp_decel / sim_mode
 * 每关节参数（<joint><param>）：node_id / counts_per_rev / max_velocity
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 EMEET
 */

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "canopen_motor_driver/canopen_motor_driver.hpp"
#include "canopen_motor_driver/motor_unit_converter.hpp"

namespace robot_arm_driver {

class ArmHardwareInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    enum class Mode { IP, PV };

    /** @brief 逐关节切 DS402 模式（含错峰上电防涌流）；per-driver mutex 内部加锁 */
    bool setAllMode(Mode m);
    void startFeedback();
    void stopFeedback();
    void feedbackLoop();

    // ── 配置（URDF hardware param）──────────────────────────────────────────
    std::string can_if_{"can0"};
    int     sdo_timeout_ms_{500};
    int     ip_period_ms_{10};
    int     feedback_fast_ms_{20};
    int     enable_stagger_ms_{150};
    int     heartbeat_ms_{100};
    double  pp_accel_{1.0};
    double  pp_decel_{1.0};
    uint8_t master_node_id_{127};
    bool    sim_mode_{false};

    // ── 每关节 ──────────────────────────────────────────────────────────────
    std::vector<std::unique_ptr<arm::CanopenMotorDriver>> drivers_;
    std::vector<arm::MotorUnitConverter>                  converters_;
    std::vector<std::unique_ptr<std::mutex>>              can_mtx_;   // 串行化每轴 CAN 访问
    std::vector<int64_t> node_ids_;
    std::vector<int64_t> counts_per_rev_;
    std::vector<double>  max_vel_;

    // ── ros2_control 接口缓冲（按 info_.joints 顺序）─────────────────────────
    std::vector<double> pos_, vel_, cmd_pos_, cmd_vel_;

    // ── 反馈缓存（后台线程写，read() 读）────────────────────────────────────
    std::vector<double> fb_pos_, fb_vel_;
    std::mutex          fb_mtx_;
    std::thread         fb_thread_;
    std::atomic<bool>   fb_running_{false};

    Mode              mode_{Mode::IP};
    std::atomic<bool> active_{false};
    double            hb_accum_s_{0.0};

    rclcpp::Logger logger_{rclcpp::get_logger("ArmHardwareInterface")};
};

}  // namespace robot_arm_driver
