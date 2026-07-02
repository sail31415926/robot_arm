/**
 * @file arm_driver_services.cpp
 * @brief 使能 / 失能 / 故障恢复伴生服务节点（分层重构 决策① 方案 A）
 *
 * HAL 化后臂由 ros2_control 驱动，DS402 的使能/失能/复位藏在 ArmHardwareInterface 的
 * on_activate / on_deactivate 里，没法像旧 arm_node 那样直接暴露服务。本薄节点把
 * commander 期望的 /arm_node/{enable,disable,recover}（std_srvs/Trigger）翻译成
 * ros2_control 的「硬件组件生命周期 + 控制器切换」操作，使 commander 的
 * ArmEnable / ArmResetError 无需改动即可工作。
 *
 *   enable  → 组件切 active（on_activate: DS402 使能 + 切 IP 模式）+ 重新激活 arm_controller
 *   disable → 先停 arm_controller，再把组件切 inactive（on_deactivate: 电机失力矩）
 *   recover → 组件 inactive→active 循环（on_activate 内含 resetFault）+ 重新激活 arm_controller
 *
 * 参数：
 *   hardware_component  默认 "eMeetArm_hardware"（须与 URDF <ros2_control name=...> 一致）
 *   controller          默认 "arm_controller"
 *   controller_manager  默认 "/controller_manager"
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 EMEET
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;
using Trigger          = std_srvs::srv::Trigger;
using SetHwState       = controller_manager_msgs::srv::SetHardwareComponentState;
using SwitchController = controller_manager_msgs::srv::SwitchController;

class ArmDriverServices : public rclcpp::Node {
public:
    ArmDriverServices() : rclcpp::Node("arm_driver_services")
    {
        hw_         = declare_parameter<std::string>("hardware_component", "eMeetArm_hardware");
        controller_ = declare_parameter<std::string>("controller",         "arm_controller");
        cm_         = declare_parameter<std::string>("controller_manager", "/controller_manager");

        // Reentrant：服务回调里同步等待 client future，需多线程执行器 + 可重入组
        cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        set_state_cli_ = create_client<SetHwState>(
            cm_ + "/set_hardware_component_state", rmw_qos_profile_services_default, cbg_);
        switch_cli_ = create_client<SwitchController>(
            cm_ + "/switch_controller", rmw_qos_profile_services_default, cbg_);

        srv_enable_ = create_service<Trigger>("/arm_node/enable",
            std::bind(&ArmDriverServices::onEnable, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_);
        srv_disable_ = create_service<Trigger>("/arm_node/disable",
            std::bind(&ArmDriverServices::onDisable, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_);
        srv_recover_ = create_service<Trigger>("/arm_node/recover",
            std::bind(&ArmDriverServices::onRecover, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_);

        RCLCPP_INFO(get_logger(),
            "arm_driver_services 就绪  hw=%s  controller=%s  cm=%s\n"
            "  /arm_node/enable  /arm_node/disable  /arm_node/recover",
            hw_.c_str(), controller_.c_str(), cm_.c_str());
    }

private:
    // ── 服务回调 ────────────────────────────────────────────────────────────
    void onEnable(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
    {
        bool ok = setHwState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
        if (ok) switchController({controller_}, {});   // 重新激活 arm_controller（best-effort）
        res->success = ok;
        res->message = ok ? "已使能（组件 active + arm_controller 激活）" : "使能失败";
        RCLCPP_INFO(get_logger(), "enable: %s", res->message.c_str());
    }

    void onDisable(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
    {
        switchController({}, {controller_});           // 先停控制器，避免对失能臂下发指令
        bool ok = setHwState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
        res->success = ok;
        res->message = ok ? "已失能（arm_controller 停 + 组件 inactive）" : "失能失败";
        RCLCPP_INFO(get_logger(), "disable: %s", res->message.c_str());
    }

    void onRecover(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
    {
        // inactive→active 循环：on_deactivate 失能，on_activate 内含 resetFault + 重使能
        bool ok = setHwState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
        if (ok) ok = setHwState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
        if (ok) switchController({controller_}, {});
        res->success = ok;
        res->message = ok ? "已恢复（inactive→active，含 resetFault + arm_controller 激活）"
                          : "恢复失败";
        RCLCPP_INFO(get_logger(), "recover: %s", res->message.c_str());
    }

    // ── ros2_control 服务封装（同步等待，Reentrant 组 + 多线程执行器）─────────
    bool setHwState(uint8_t id, const std::string & label)
    {
        if (!set_state_cli_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "set_hardware_component_state 不可用");
            return false;
        }
        auto req = std::make_shared<SetHwState::Request>();
        req->name = hw_;
        req->target_state.id    = id;
        req->target_state.label = label;
        auto future = set_state_cli_->async_send_request(req);
        if (future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_WARN(get_logger(), "set_hardware_component_state(%s) 超时", label.c_str());
            return false;
        }
        return future.get()->ok;
    }

    bool switchController(const std::vector<std::string> & activate,
                          const std::vector<std::string> & deactivate)
    {
        if (!switch_cli_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "switch_controller 不可用");
            return false;
        }
        auto req = std::make_shared<SwitchController::Request>();
        req->activate_controllers   = activate;
        req->deactivate_controllers = deactivate;
        req->strictness = SwitchController::Request::BEST_EFFORT;
        auto future = switch_cli_->async_send_request(req);
        if (future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_WARN(get_logger(), "switch_controller 超时");
            return false;
        }
        return future.get()->ok;
    }

    std::string hw_, controller_, cm_;
    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::Client<SetHwState>::SharedPtr       set_state_cli_;
    rclcpp::Client<SwitchController>::SharedPtr  switch_cli_;
    rclcpp::Service<Trigger>::SharedPtr srv_enable_, srv_disable_, srv_recover_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmDriverServices>();
    rclcpp::executors::MultiThreadedExecutor exec;   // 服务回调内同步等 client，须多线程
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
