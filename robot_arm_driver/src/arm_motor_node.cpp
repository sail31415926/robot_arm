/**
 * @file arm_motor_node.cpp
 * @brief 单关节 CANopen 电机节点（CiA402），向上提供标准 ROS2 SI 单位接口
 *
 * 封装 CanopenMotorDriver，支持全部 CiA402 运动模式：
 *   - PP  轮廓位置模式：绝对位置，相对零点
 *   - PV  轮廓速度模式：持续速度控制
 *   - PT  轮廓力矩模式：力矩控制
 *   - IP  插补位置模式：需以固定周期持续推送位置点
 *   - HM  硬件回零模式：自动寻找限位开关，驱动器内部清零
 *
 * 服务接口（状态控制，有应答）：
 *   ~/enable          ~/disable         ~/recover
 *   ~/position_mode   ~/velocity_mode   ~/torque_mode   ~/ip_mode
 *   ~/homing          ~/set_home
 *
 * 话题接口（指令与反馈，持续流式）：
 *   订阅  ~/cmd_pos（std_msgs/Float64，rad）    — PP / IP 模式位置指令
 *   订阅  ~/cmd_vel（std_msgs/Float64，rad/s）  — PV 模式速度指令
 *   订阅  ~/cmd_eff（std_msgs/Float64，Nm）     — PT 模式力矩指令
 *   发布  ~/joint_states（sensor_msgs/JointState）  — 位置/速度/力矩反馈，100ms 周期
 *   发布  ~/mode（std_msgs/String）              — 当前运动模式名称
 *   发布  ~/status（diagnostic_msgs/DiagnosticStatus）— 就绪状态，100ms 周期
 *
 * 所有运动参数、零点策略、回零配置均通过 robot_arm_driver/config/motors.yaml 配置，更换电机无需改代码。
 * 多关节部署时，joint1 节点负责发送主站心跳（heartbeat_ms > 0），其余节点设为 0。
 *
 * 启动方式：
 *   单关节调试（无命名空间）：
 *     ros2 run robot_arm_driver arm_motor_node \
 *       --ros-args --params-file install/robot_arm_driver/share/robot_arm_driver/config/motors.yaml
 *     服务路径：/arm_motor_node/enable  /arm_motor_node/position_mode ...
 *
 *   多关节（加命名空间，推荐用 launch 文件）：
 *     ros2 launch robot_arm_bringup motor.launch.py
 *     服务路径：/joint1/arm_motor_node/enable  /joint2/arm_motor_node/enable ...
 *
 *   调试 GUI：
 *     ros2 run robot_arm_driver motor_test_gui
 *
 * 所属模块：robot_arm_driver/src/
 *
 * @version 1.3
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 EMEET
 */

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "canopen_motor_driver/canopen_motor_driver.hpp"
#include "canopen_motor_driver/motor_unit_converter.hpp"

using namespace std::chrono_literals;
using Trigger          = std_srvs::srv::Trigger;
using JointState       = sensor_msgs::msg::JointState;
using Float64          = std_msgs::msg::Float64;
using String           = std_msgs::msg::String;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue         = diagnostic_msgs::msg::KeyValue;

class ArmMotorNode : public rclcpp::Node {
public:
    ArmMotorNode()
        : Node("arm_motor_node"),
          drv_(declare_parameter<std::string>("can_interface", "can0"),
               static_cast<uint8_t>(declare_parameter<int>("node_id", 1)),
               declare_parameter<int>("sdo_timeout_ms", 500)),
          joint_name_(declare_parameter<std::string>("joint_name", "joint")),
          conv_(declare_parameter<int>("counts_per_rev", 524288))
    {
        declare_parameter<double>("profile_velocity", 0.5);  // rad/s
        declare_parameter<double>("profile_accel",    0.5);  // rad/s²
        declare_parameter<double>("profile_decel",    0.5);  // rad/s²
        declare_parameter<int>("master_node_id",  127);      // 主站心跳节点 ID
        declare_parameter<int>("heartbeat_ms",    100);      // 心跳周期 ms
        rated_torque_ = declare_parameter<double>("rated_torque", 0.0);  // Nm
        bool zero_on_start = declare_parameter<bool>("zero_on_start", false);
        declare_parameter<double>("max_torque",       1.0);   // 额定力矩倍数（0~3.0）
        declare_parameter<int>   ("torque_slope_ms",  0);     // 力矩斜坡 ms
        declare_parameter<int>   ("ip_period_ms",     10);    // IP 插补周期 ms
        declare_parameter<int>   ("homing_method",     1);
        declare_parameter<double>("homing_fast_vel",   0.3);   // rad/s
        declare_parameter<double>("homing_slow_vel",   0.05);  // rad/s
        declare_parameter<double>("homing_accel",      0.2);   // rad/s²
        declare_parameter<double>("homing_offset",     0.0);   // rad
        declare_parameter<int>   ("homing_timeout_ms", 30000);

        if (!drv_.init()) {
            RCLCPP_FATAL(get_logger(),
                "无法打开 CAN 接口 '%s'，请确认接口存在且进程有 CAP_NET_RAW 权限",
                get_parameter("can_interface").as_string().c_str());
            throw std::runtime_error("CAN init failed");
        }

        RCLCPP_INFO(get_logger(),
            "CAN 接口已打开  node_id=%ld  joint=%s  counts_per_rev=%ld",
            get_parameter("node_id").as_int(), joint_name_.c_str(),
            conv_.countsPerRev());
        RCLCPP_INFO(get_logger(),
            "运动参数  vel=%.3f rad/s  accel=%.3f rad/s²  decel=%.3f rad/s²",
            get_parameter("profile_velocity").as_double(),
            get_parameter("profile_accel").as_double(),
            get_parameter("profile_decel").as_double());

        if (zero_on_start) {
            home_offset_ = drv_.getPosition();
            RCLCPP_INFO(get_logger(), "上电清零：home_offset = %d counts (%.4f rad)",
                home_offset_, conv_.ppToRad(home_offset_));
        }

        pub_js_     = create_publisher<JointState>("~/joint_states", 10);
        pub_mode_   = create_publisher<String>("~/mode", 10);
        pub_status_ = create_publisher<DiagnosticStatus>("~/status", 10);
        hardware_id_ = get_parameter("can_interface").as_string() + "/node" +
                       std::to_string(get_parameter("node_id").as_int());

        sub_cmd_pos_ = create_subscription<Float64>("~/cmd_pos", 10,
            [this](const Float64::SharedPtr msg) {
                if (mode_ == Mode::PP) {
                    drv_.moveToPosition(conv_.radToPP(msg->data) + home_offset_,
                                        /*relative=*/false, /*wait=*/false);
                } else if (mode_ == Mode::IP) {
                    // IP 模式：只更新目标值，由专属定时器以固定周期发送 PDO+SYNC
                    ip_target_pos_ = conv_.radToPP(msg->data) + home_offset_;
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "收到位置指令但当前不在 PP/IP 模式（当前: %s）",
                        mode_ == Mode::PV ? "PV" : mode_ == Mode::PT ? "PT" : "NONE");
                }
            });

        sub_cmd_vel_ = create_subscription<Float64>("~/cmd_vel", 10,
            [this](const Float64::SharedPtr msg) {
                if (mode_ == Mode::PV) {
                    drv_.setTargetVelocity(conv_.radToVelPPSigned(msg->data));
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "收到速度指令但当前不在 PV 模式，请先调用 ~/velocity_mode");
                }
            });

        sub_cmd_eff_ = create_subscription<Float64>("~/cmd_eff", 10,
            [this](const Float64::SharedPtr msg) {
                if (mode_ != Mode::PT) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "收到力矩指令但当前不在 PT 模式，请先调用 ~/torque_mode");
                    return;
                }
                if (rated_torque_ <= 0.0) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "rated_torque 未配置（当前为 0），力矩指令被忽略");
                    return;
                }
                auto cmd = static_cast<int16_t>(
                    std::clamp(msg->data / rated_torque_ * 1000.0, -3000.0, 3000.0));
                drv_.setTargetTorque(cmd);
            });

        srv_enable_ = create_service<Trigger>("~/enable",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                res->success = drv_.enable();
                res->message = res->success ? "伺服已使能" : "使能失败，请检查驱动器状态";
            });

        srv_disable_ = create_service<Trigger>("~/disable",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                if (timer_ip_) { timer_ip_->cancel(); timer_ip_.reset(); }
                setMode(Mode::NONE);
                res->success = drv_.disable();
                res->message = res->success ? "伺服已禁用" : "禁用失败";
            });

        srv_recover_ = create_service<Trigger>("~/recover",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                if (timer_ip_) { timer_ip_->cancel(); timer_ip_.reset(); }
                setMode(Mode::NONE);
                drv_.resetFault();
                rclcpp::sleep_for(200ms);
                res->success = drv_.enable();
                res->message = res->success ? "故障复位并重新使能成功" : "复位后使能失败";
            });

        srv_pp_mode_ = create_service<Trigger>("~/position_mode",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                auto vel   = conv_.radToVelPP(get_parameter("profile_velocity").as_double());
                auto accel = conv_.radToAccPP(get_parameter("profile_accel").as_double());
                auto decel = conv_.radToAccPP(get_parameter("profile_decel").as_double());
                res->success = drv_.setProfilePositionMode(vel, accel, decel);
                if (res->success) setMode(Mode::PP);
                res->message = res->success ? "已切换到轮廓位置模式(PP)" : "模式切换失败";
            });

        srv_ip_mode_ = create_service<Trigger>("~/ip_mode",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                auto period  = static_cast<uint8_t>(
                    std::clamp<int64_t>(get_parameter("ip_period_ms").as_int(), 1, 20));
                auto max_vel = conv_.radToVelPP(get_parameter("profile_velocity").as_double());
                if (timer_ip_) { timer_ip_->cancel(); timer_ip_.reset(); }
                res->success = drv_.setInterpolatedPositionMode(period, max_vel);
                if (res->success) {
                    setMode(Mode::IP);
                    ip_target_pos_  = drv_.getPosition();
                    ip_current_pos_ = ip_target_pos_;
                    // 每周期最大允许位移 = profile_velocity * period
                    ip_max_step_ = std::max<int32_t>(1,
                        static_cast<int32_t>(max_vel) * period / 1000);
                    RCLCPP_INFO(get_logger(),
                        "IP 模式  period=%dms  max_vel=%u pp/s  每周期上限=%d counts",
                        period, max_vel, ip_max_step_);
                    timer_ip_ = create_wall_timer(
                        std::chrono::milliseconds(period),
                        [this]() {
                            if (mode_ != Mode::IP) return;
                            // 速度限幅：每周期 ip_current_pos_ 朝 ip_target_pos_ 移动不超过 ip_max_step_
                            int64_t delta = int64_t(ip_target_pos_) - int64_t(ip_current_pos_);
                            if (delta >  ip_max_step_) delta =  ip_max_step_;
                            if (delta < -ip_max_step_) delta = -ip_max_step_;
                            ip_current_pos_ += static_cast<int32_t>(delta);
                            drv_.setInterpolatedPosition(ip_current_pos_);
                        });
                }
                res->message = res->success
                    ? "已切换到插补位置模式(IP)，每周期 " + std::to_string(period) +
                      "ms 内位移限幅在 profile_velocity 内"
                    : "模式切换失败";
            });

        srv_pt_mode_ = create_service<Trigger>("~/torque_mode",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                auto max_t   = static_cast<uint16_t>(
                    std::clamp(get_parameter("max_torque").as_double(), 0.0, 3.0) * 1000.0);
                auto slope   = static_cast<uint32_t>(get_parameter("torque_slope_ms").as_int());
                auto max_vel = conv_.radToVelPP(get_parameter("profile_velocity").as_double());
                res->success = drv_.setProfileTorqueMode(max_t, slope, max_vel);
                if (res->success) setMode(Mode::PT);
                res->message = res->success ? "已切换到轮廓力矩模式(PT)，~/target 单位: Nm"
                                            : "模式切换失败";
            });

        srv_homing_ = create_service<Trigger>("~/homing",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                auto method   = static_cast<uint8_t>(get_parameter("homing_method").as_int());
                auto fast_vel = conv_.radToVelPP(get_parameter("homing_fast_vel").as_double());
                auto slow_vel = std::max(1u, conv_.radToVelPP(get_parameter("homing_slow_vel").as_double()));
                auto accel    = conv_.radToAccPP(get_parameter("homing_accel").as_double());
                auto offset   = conv_.radToPP   (get_parameter("homing_offset").as_double());
                auto timeout  = static_cast<int>(get_parameter("homing_timeout_ms").as_int());

                if (!drv_.setHomingMode(method, fast_vel, slow_vel, accel, offset)) {
                    res->success = false;
                    res->message = "回零模式配置失败，请检查驱动器状态";
                    return;
                }
                RCLCPP_INFO(get_logger(), "回零启动  method=%d  fast=%.3f rad/s  slow=%.3f rad/s",
                    method,
                    get_parameter("homing_fast_vel").as_double(),
                    get_parameter("homing_slow_vel").as_double());

                res->success = drv_.startHoming(timeout, [this]() {
                    drv_.sendHeartbeat(master_node_id_);
                });
                if (res->success) {
                    home_offset_ = 0;  // 驱动器已内部清零，软件偏移归零
                    setMode(Mode::NONE);
                    RCLCPP_INFO(get_logger(), "回零完成，请重新调用 position_mode 或 velocity_mode");
                    res->message = "回零完成，驱动器位置已清零";
                } else {
                    res->message = "回零超时或错误（状态字 bit13），请检查限位开关接线";
                }
            });

        srv_set_home_ = create_service<Trigger>("~/set_home",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                home_offset_ = drv_.getPosition();
                res->success = true;
                res->message = "零点已设置，当前位置记为 0 rad";
                RCLCPP_INFO(get_logger(), "set_home：home_offset = %d counts (%.4f rad)",
                    home_offset_, conv_.ppToRad(home_offset_));
            });

        srv_pv_mode_ = create_service<Trigger>("~/velocity_mode",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                auto accel = conv_.radToAccPP(get_parameter("profile_accel").as_double());
                auto decel = conv_.radToAccPP(get_parameter("profile_decel").as_double());
                res->success = drv_.setProfileVelocityMode(accel, decel);
                if (res->success) setMode(Mode::PV);
                res->message = res->success ? "已切换到轮廓速度模式(PV)" : "模式切换失败";
            });

        // 快反馈：仅读位置（1次SDO），与 IP 控制周期对齐
        auto fast_ms = declare_parameter<int>("feedback_fast_ms", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(fast_ms),
            [this]() { publishFastFeedback(); });

        // 慢反馈：读速度/力矩/状态字（3次SDO），频率低避免阻塞控制
        timer_slow_ = create_wall_timer(200ms, [this]() { publishSlowFeedback(); });

        auto hb_ms = get_parameter("heartbeat_ms").as_int();
        master_node_id_ = static_cast<uint8_t>(get_parameter("master_node_id").as_int());
        if (hb_ms > 0) {
            timer_hb_ = create_wall_timer(
                std::chrono::milliseconds(hb_ms),
                [this]() { drv_.sendHeartbeat(master_node_id_); });
        }

        RCLCPP_INFO(get_logger(), "arm_motor_node 启动完成");
        RCLCPP_INFO(get_logger(), "  ~/cmd_pos — 位置指令 rad（PP/IP 模式）");
        RCLCPP_INFO(get_logger(), "  ~/cmd_vel — 速度指令 rad/s（PV 模式）");
        RCLCPP_INFO(get_logger(), "  ~/cmd_eff — 力矩指令 Nm（PT 模式）");
        RCLCPP_INFO(get_logger(), "  ~/joint_states  — 位置(rad) / 速度(rad/s) / 力矩(Nm) 反馈");
        RCLCPP_INFO(get_logger(), "  力矩换算  rated_torque=%.3f Nm  (0x6077 ‰ → Nm)",
            rated_torque_);
    }

private:
    enum class Mode { NONE, PP, PV, PT, IP };

    void setMode(Mode m) {
        mode_ = m;
        String msg;
        switch (m) {
            case Mode::PP: msg.data = "PP"; break;
            case Mode::PV: msg.data = "PV"; break;
            case Mode::PT: msg.data = "PT"; break;
            case Mode::IP: msg.data = "IP"; break;
            default:       msg.data = "NONE"; break;
        }
        pub_mode_->publish(msg);
    }

    // 快反馈：每 feedback_fast_ms 执行一次，仅读位置（1次SDO）
    void publishFastFeedback() {
        cached_pos_ = drv_.getPosition();
        auto msg = JointState();
        msg.header.stamp = now();
        msg.name     = { joint_name_ };
        msg.position = { conv_.ppToRad(cached_pos_ - home_offset_) };
        msg.velocity = { conv_.ppToRadS(cached_vel_) };
        msg.effort   = { static_cast<double>(cached_torque_) / 1000.0 * rated_torque_ };
        pub_js_->publish(msg);
    }

    // 慢反馈：每 200ms 执行一次，读速度/力矩/状态字（3次SDO）
    void publishSlowFeedback() {
        cached_vel_    = drv_.getVelocity();
        cached_torque_ = drv_.getTorque();
        cached_sw_     = drv_.getStatusWord();

        bool fault   = cached_sw_ & 0x0008u;
        bool enabled = (cached_sw_ & 0x0004u) && !fault;

        // 故障时读取错误码 0x603Fh，便于诊断
        uint16_t err_code = 0;
        if (fault) {
            uint8_t buf[4]{}; uint8_t len = 0;
            if (drv_.readSDO(0x603Fu, 0, buf, len))
                err_code = uint16_t(buf[0]) | uint16_t(buf[1]) << 8;
        }

        DiagnosticStatus msg;
        msg.hardware_id = hardware_id_;
        msg.name        = joint_name_;

        if (fault) {
            msg.level   = DiagnosticStatus::ERROR;
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "驱动器故障 ER.%03X，请调用 ~/recover 复位", err_code);
            msg.message = buf;
        } else if (!enabled) {
            msg.level   = DiagnosticStatus::ERROR;
            msg.message = "伺服未使能，请调用 ~/enable";
        } else if (mode_ == Mode::NONE) {
            msg.level   = DiagnosticStatus::WARN;
            msg.message = "已使能但未设置运动模式";
        } else {
            msg.level   = DiagnosticStatus::OK;
            msg.message = "就绪";
        }

        auto kv = [](const std::string& k, const std::string& v) {
            KeyValue p; p.key = k; p.value = v; return p;
        };
        msg.values = {
            kv("enabled", enabled ? "true" : "false"),
            kv("fault",   fault   ? "true" : "false"),
            kv("mode",    mode_ == Mode::PP ? "PP" :
                          mode_ == Mode::PV ? "PV" :
                          mode_ == Mode::PT ? "PT" :
                          mode_ == Mode::IP ? "IP" : "NONE"),
            kv("position_rad", std::to_string(conv_.ppToRad(cached_pos_ - home_offset_))),
        };
        pub_status_->publish(msg);
    }

    arm::CanopenMotorDriver  drv_;
    std::string              joint_name_;
    arm::MotorUnitConverter  conv_;
    Mode                     mode_{ Mode::NONE };
    double                   rated_torque_{ 0.0 };   // Nm
    int32_t                  home_offset_{ 0 };       // 零点编码器计数
    std::string              hardware_id_;             // can_interface/nodeN

    rclcpp::Publisher<JointState>::SharedPtr        pub_js_;
    rclcpp::Publisher<String>::SharedPtr            pub_mode_;
    rclcpp::Publisher<DiagnosticStatus>::SharedPtr  pub_status_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_cmd_pos_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_cmd_vel_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_cmd_eff_;
    rclcpp::Service<Trigger>::SharedPtr       srv_enable_;
    rclcpp::Service<Trigger>::SharedPtr       srv_disable_;
    rclcpp::Service<Trigger>::SharedPtr       srv_recover_;
    rclcpp::Service<Trigger>::SharedPtr       srv_pp_mode_;
    rclcpp::Service<Trigger>::SharedPtr       srv_pv_mode_;
    rclcpp::Service<Trigger>::SharedPtr       srv_ip_mode_;
    rclcpp::Service<Trigger>::SharedPtr       srv_pt_mode_;
    rclcpp::Service<Trigger>::SharedPtr       srv_homing_;
    rclcpp::Service<Trigger>::SharedPtr       srv_set_home_;
    rclcpp::TimerBase::SharedPtr              timer_;
    rclcpp::TimerBase::SharedPtr              timer_slow_;
    rclcpp::TimerBase::SharedPtr              timer_hb_;
    rclcpp::TimerBase::SharedPtr              timer_ip_;
    int32_t                                   ip_target_pos_{ 0 };   // cmd_pos 写入的"理想"目标
    int32_t                                   ip_current_pos_{ 0 };  // 实际下发到驱动器的限速后位置
    int32_t                                   ip_max_step_{ 1000 };  // 每周期最大 pp 步长
    // 慢反馈缓存（由 200ms 定时器更新，快反馈直接使用）
    int32_t  cached_pos_{ 0 };
    int32_t  cached_vel_{ 0 };
    int16_t  cached_torque_{ 0 };
    uint16_t cached_sw_{ 0 };
    uint8_t                                   master_node_id_{ 127 };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ArmMotorNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
