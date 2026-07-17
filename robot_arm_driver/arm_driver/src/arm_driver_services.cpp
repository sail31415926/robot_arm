/**
 * @file arm_driver_services.cpp
 * @brief 使能 / 失能 / 故障恢复伴生服务节点（分层重构 决策① 方案 A）
 *
 * HAL 化后臂由 ros2_control 驱动，CiA402 的使能/失能/复位藏在 canopen_ros2_control/
 * RobotSystem 的 on_activate（init_motor）/ on_deactivate（halt_motor）里，没法像旧
 * arm_node 那样直接暴露服务。本薄节点把 commander 期望的
 * /arm_node/{enable,disable,recover}（std_srvs/Trigger）翻译成 ros2_control 的
 * 「硬件组件生命周期 + 控制器切换」操作，使 commander 的 ArmEnable / ArmResetError
 * 无需改动即可工作。
 *
 *   enable  → 组件切 active（on_activate: 402 状态机走到 Operation Enabled）+ 激活 arm_controller
 *   disable → 先停 arm_controller，再把组件切 inactive（on_deactivate: halt，电机失力矩）
 *   recover → 组件 inactive→active 循环（402 状态机含故障复位路径）+ 重新激活 arm_controller
 *
 * v1.1 新增电机运行模式切换 /arm_node/set_mode_{pp,ip,pv}（std_srvs/Trigger，静止时切换）：
 *   pp → 确保 arm_controller(JTC,position) active，逐关节调 /joint_N/position_mode
 *        （PP=1，驱动器按 6081/6083 自规划，适合关节滑块点到点）
 *   ip → 确保 arm_controller active，逐关节调 /joint_N/interpolated_position_mode
 *        （IP=7，跟随上位机 10ms 插补流，适合轨迹/运镜；bus.yml 默认即 IP）
 *   pv → switch_controller: arm_controller ⇄ arm_velocity_controller
 *        （velocity 接口被 claim 时 RobotSystem 自动切 PV=3；不可在 position 被
 *         claim 时直调驱动 velocity_mode 服务，否则 write 环会喂入无效速度）
 *   机制依据：canopen_ros2_control 的 write_target() 按驱动器**当前模式**分发
 *   （PP/IP 均消费 position 命令），故 PP↔IP 可在 JTC 不动的情况下热切换。
 *
 * 参数：
 *   hardware_component  默认 "eMeetArm_hardware"（须与 URDF <ros2_control name=...> 一致）
 *   controller          默认 "arm_controller"
 *   velocity_controller 默认 "arm_velocity_controller"
 *   controller_manager  默认 "/controller_manager"
 *   joint_nodes         默认 ["joint_1","joint_2","joint_3"]（402 驱动节点名，见 bus.yml）
 *
 * @version 1.1
 * @date 2026-07-10
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
        vel_controller_ = declare_parameter<std::string>("velocity_controller", "arm_velocity_controller");
        cm_         = declare_parameter<std::string>("controller_manager", "/controller_manager");
        joint_nodes_ = declare_parameter<std::vector<std::string>>(
            "joint_nodes", {"joint_1", "joint_2", "joint_3"});

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

        // 电机模式切换：每关节的 402 驱动节点自带 Trigger 服务，这里做编排
        for (const auto & j : joint_nodes_) {
            pp_clis_.push_back(create_client<Trigger>(
                "/" + j + "/position_mode", rmw_qos_profile_services_default, cbg_));
            ip_clis_.push_back(create_client<Trigger>(
                "/" + j + "/interpolated_position_mode", rmw_qos_profile_services_default, cbg_));
        }
        srv_mode_pp_ = create_service<Trigger>("/arm_node/set_mode_pp",
            std::bind(&ArmDriverServices::onModePP, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_);
        srv_mode_ip_ = create_service<Trigger>("/arm_node/set_mode_ip",
            std::bind(&ArmDriverServices::onModeIP, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_);
        srv_mode_pv_ = create_service<Trigger>("/arm_node/set_mode_pv",
            std::bind(&ArmDriverServices::onModePV, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_);

        RCLCPP_INFO(get_logger(),
            "arm_driver_services 就绪  hw=%s  controller=%s  cm=%s\n"
            "  /arm_node/enable  /arm_node/disable  /arm_node/recover\n"
            "  /arm_node/set_mode_pp  /arm_node/set_mode_ip  /arm_node/set_mode_pv",
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
        // 先停控制器释放已 claim 的接口（与 disable 一致——组件被占用时做生命周期
        // 循环易半途失败），再 inactive→active（on_deactivate 失能，on_activate 内含
        // resetFault + 重使能）；控制器激活不以组件循环成败为前提，避免任一步失败后
        // 系统留在"控制器停/组件错乱"的无头状态，只能重启才能救
        switchController({}, {controller_});
        bool hw_ok = setHwState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
        if (hw_ok) hw_ok = setHwState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
        const bool ctrl_ok = switchController({controller_}, {});
        res->success = hw_ok && ctrl_ok;
        res->message = res->success
            ? "已恢复（控制器停→组件 inactive→active→控制器激活，含 resetFault）"
            : std::string("恢复失败（组件循环") + (hw_ok ? "OK" : "失败")
              + "，控制器激活" + (ctrl_ok ? "OK" : "失败") + "，看日志定位）";
        RCLCPP_INFO(get_logger(), "recover: %s", res->message.c_str());
    }

    // ── 电机模式切换（静止时调用；PP/IP 热切换，PV 走控制器切换）─────────────
    void onModePP(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
    {
        // 先确保位置控制器在管（从 PV 切回时经由 IP 过渡，静止无碍）
        bool ok = switchController({controller_}, {vel_controller_});
        ok = ok && callAll(pp_clis_, "position_mode");
        res->success = ok;
        res->message = ok ? "电机已切 PP(1)：驱动器自规划（6081 限速）" : "PP 切换失败";
        RCLCPP_INFO(get_logger(), "set_mode_pp: %s", res->message.c_str());
    }

    void onModeIP(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
    {
        bool ok = switchController({controller_}, {vel_controller_});
        ok = ok && callAll(ip_clis_, "interpolated_position_mode");
        res->success = ok;
        res->message = ok ? "电机已切 IP(7)：跟随上位机 10ms 插补流" : "IP 切换失败";
        RCLCPP_INFO(get_logger(), "set_mode_ip: %s", res->message.c_str());
    }

    void onModePV(Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res)
    {
        // velocity 接口被 claim 时 RobotSystem::perform_command_mode_switch 自动切 PV(3)
        bool ok = switchController({vel_controller_}, {controller_});
        res->success = ok;
        res->message = ok ? "电机已切 PV(3)：arm_velocity_controller 接管（速度伺服）"
                          : "PV 切换失败（arm_velocity_controller 是否已加载？）";
        RCLCPP_INFO(get_logger(), "set_mode_pv: %s", res->message.c_str());
    }

    bool callAll(std::vector<rclcpp::Client<Trigger>::SharedPtr> & clis, const char * what)
    {
        bool ok = true;
        for (auto & cli : clis) {
            if (!cli->wait_for_service(1s)) {
                RCLCPP_WARN(get_logger(), "%s: 服务 %s 不可用", what, cli->get_service_name());
                ok = false;
                continue;
            }
            auto future = cli->async_send_request(std::make_shared<Trigger::Request>());
            if (future.wait_for(5s) != std::future_status::ready) {
                RCLCPP_WARN(get_logger(), "%s: %s 超时", what, cli->get_service_name());
                ok = false;
                continue;
            }
            ok = future.get()->success && ok;
        }
        return ok;
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

    std::string hw_, controller_, vel_controller_, cm_;
    std::vector<std::string> joint_nodes_;
    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::Client<SetHwState>::SharedPtr       set_state_cli_;
    rclcpp::Client<SwitchController>::SharedPtr  switch_cli_;
    std::vector<rclcpp::Client<Trigger>::SharedPtr> pp_clis_, ip_clis_;
    rclcpp::Service<Trigger>::SharedPtr srv_enable_, srv_disable_, srv_recover_;
    rclcpp::Service<Trigger>::SharedPtr srv_mode_pp_, srv_mode_ip_, srv_mode_pv_;
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
