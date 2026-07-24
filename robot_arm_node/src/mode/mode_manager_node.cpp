/**
 * @file mode_manager_node.cpp
 * @brief 控制模式仲裁器（Mode Manager）—— 语义控制模式的唯一权威，后端无感。
 *
 * 【为什么需要它】
 *   「位置/速度/力矩」是一根贯穿各层的正交维度：在驱动器层是 CiA402 的 IP/PV/PT，
 *   在 ros2_control 层是 position/velocity/effort 命令接口，在应用层是不同任务意图。
 *   RobotSystem 硬约束「每关节同一时刻只能 claim 一个命令接口 = 只能处于一种 402 模式」，
 *   所以模式切换 = ros2_control 控制器切换。过去这件事散落在驱动层伴生节点
 *   arm_driver_services 的 /arm_node/set_mode_*（Trigger、无语义、只在实物存在、只认 J1-3）。
 *   本节点把它上提为产品层、后端无感（Gazebo/实物都有 controller_manager）的一等公民：
 *   上层只调 /robot_arm/switch_control_mode(ControlMode)，不关心底层控制器名/402 模式。
 *
 * 【职责】
 *   ① 唯一切换入口：/robot_arm/switch_control_mode（SwitchControlMode.srv），串行化执行
 *   ② bumpless 播种：切到速度/力矩前先喂 0（切回轨迹由 JTC 自动锁当前位姿，无需播种）
 *   ③ 原子切换：controller_manager/switch_controller（STRICT，停旧启新）
 *   ④ latched 广播当前模式：/robot_arm/control_mode（晚订阅者也能拿到当前值）
 *   ⑤ 速度/力矩总线看门狗：上层发 /robot_arm/cmd/joint_{velocity,effort}，本节点转发到
 *      控制器命令话题；命令断流超时 → 自动喂 0（ForwardCommandController 不自动归零，
 *      失控风险，必须兜底）。转发单点也让安全逻辑集中于一处。
 *
 * 【约定】
 *   - 必须在机械臂静止时切换；速度/力矩模式下云台 J4-6 不受控（arm_controller 被停），保持当前位置。
 *   - 驱动层 /arm_node/set_mode_pp|ip（402 profile 微调，PP↔IP 不换控制器）仍是实物专属细化，
 *     与本节点不冲突：TRAJECTORY 模式默认走 IP，需要驱动器自规划点到点时再单独调 set_mode_pp。
 *   - JOINT_EFFORT/ADMITTANCE 为 P3 预留：effort_controller 默认未配置（参数留空），
 *     切换时若目标控制器名为空则明确拒绝。
 *
 * 参数（默认值适配 real / gazebo 两后端，名称已对齐）：
 *   controller_manager      默认 "/controller_manager"
 *   trajectory_controller   默认 "arm_controller"（JTC，TRAJECTORY 模式）
 *   velocity_controller     默认 "arm_velocity_controller"（JOINT_VELOCITY 模式）
 *   effort_controller       默认 ""（P3；空 = 未配置，切 JOINT_EFFORT 会被拒）
 *   velocity_input_topic    默认 "/robot_arm/cmd/joint_velocity"（产品面向的速度总线）
 *   velocity_command_topic  默认 "/arm_velocity_controller/commands"（控制器实际命令话题）
 *   effort_input_topic      默认 "/robot_arm/cmd/joint_effort"
 *   effort_command_topic    默认 "/arm_effort_controller/commands"
 *   num_command_joints      默认 3（速度/力矩 claim 的关节数 J1-3，用于播种/归零向量长度）
 *   watchdog_timeout        默认 0.3（s，速度/力矩命令断流超时归零）
 *   default_mode            默认 0（TRAJECTORY，与 launch 默认激活 arm_controller 一致）
 *
 * @version 1.0
 * @date 2026-07-23
 * @copyright Copyright (c) 2026 EMEET
 */

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <robot_arm_interfaces/msg/control_mode.hpp>
#include <robot_arm_interfaces/srv/switch_control_mode.hpp>

namespace robot_arm_node::mode
{

using namespace std::chrono_literals;
using ControlMode       = robot_arm_interfaces::msg::ControlMode;
using SwitchControlMode = robot_arm_interfaces::srv::SwitchControlMode;
using SwitchController   = controller_manager_msgs::srv::SwitchController;
using Float64MultiArray  = std_msgs::msg::Float64MultiArray;

class ModeManagerNode : public rclcpp::Node
{
public:
  ModeManagerNode() : rclcpp::Node("mode_manager_node")
  {
    cm_          = declare_parameter<std::string>("controller_manager", "/controller_manager");
    traj_ctrl_   = declare_parameter<std::string>("trajectory_controller", "arm_controller");
    vel_ctrl_    = declare_parameter<std::string>("velocity_controller", "arm_velocity_controller");
    eff_ctrl_    = declare_parameter<std::string>("effort_controller", "");
    vel_in_topic_  = declare_parameter<std::string>("velocity_input_topic", "/robot_arm/cmd/joint_velocity");
    vel_cmd_topic_ = declare_parameter<std::string>("velocity_command_topic", "/arm_velocity_controller/commands");
    eff_in_topic_  = declare_parameter<std::string>("effort_input_topic", "/robot_arm/cmd/joint_effort");
    eff_cmd_topic_ = declare_parameter<std::string>("effort_command_topic", "/arm_effort_controller/commands");
    num_joints_    = static_cast<size_t>(declare_parameter<int>("num_command_joints", 3));
    watchdog_timeout_ = declare_parameter<double>("watchdog_timeout", 0.3);
    current_mode_  = static_cast<uint8_t>(declare_parameter<int>("default_mode", ControlMode::TRAJECTORY));

    // 服务回调里同步等 switch_controller 的 future → 需多线程执行器 + 可重入组
    cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    switch_cli_ = create_client<SwitchController>(
        cm_ + "/switch_controller", rmw_qos_profile_services_default, cbg_);

    // latched：晚订阅者（commander/GUI）也能立刻拿到当前模式
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    mode_pub_ = create_publisher<ControlMode>("/robot_arm/control_mode", latched);

    switch_srv_ = create_service<SwitchControlMode>(
        "/robot_arm/switch_control_mode",
        std::bind(&ModeManagerNode::onSwitch, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cbg_);

    // 速度/力矩：产品面向输入总线 → 控制器命令话题（仅对应模式下转发）
    vel_cmd_pub_ = create_publisher<Float64MultiArray>(vel_cmd_topic_, rclcpp::QoS(10));
    vel_in_sub_  = create_subscription<Float64MultiArray>(
        vel_in_topic_, rclcpp::QoS(10),
        [this](const Float64MultiArray & m) { onCmdInput(ControlMode::JOINT_VELOCITY, m); });
    eff_cmd_pub_ = create_publisher<Float64MultiArray>(eff_cmd_topic_, rclcpp::QoS(10));
    eff_in_sub_  = create_subscription<Float64MultiArray>(
        eff_in_topic_, rclcpp::QoS(10),
        [this](const Float64MultiArray & m) { onCmdInput(ControlMode::JOINT_EFFORT, m); });

    // 看门狗：速度/力矩模式下命令断流超时 → 归零兜底
    last_cmd_time_ = now();
    watchdog_ = create_wall_timer(50ms, std::bind(&ModeManagerNode::onWatchdog, this), cbg_);

    publishMode();   // 广播初始模式（= launch 默认激活的控制器，无需切换）
    RCLCPP_INFO(get_logger(),
        "mode_manager_node 就绪  cm=%s\n"
        "  服务  /robot_arm/switch_control_mode\n"
        "  广播  /robot_arm/control_mode (latched)  初始模式=%s\n"
        "  速度总线  %s → %s（看门狗 %.0fms）",
        cm_.c_str(), modeName(current_mode_), vel_in_topic_.c_str(),
        vel_cmd_topic_.c_str(), watchdog_timeout_ * 1000.0);
  }

private:
  // ── 模式 → 控制器名 ─────────────────────────────────────────────────────────
  std::string controllerFor(uint8_t mode) const
  {
    switch (mode) {
      case ControlMode::TRAJECTORY:     return traj_ctrl_;
      case ControlMode::JOINT_VELOCITY: return vel_ctrl_;
      case ControlMode::JOINT_EFFORT:   return eff_ctrl_;
      case ControlMode::ADMITTANCE:     return traj_ctrl_;  // 导纳底层仍是位置总线
      default:                          return "";
    }
  }

  static const char * modeName(uint8_t mode)
  {
    switch (mode) {
      case ControlMode::TRAJECTORY:     return "TRAJECTORY";
      case ControlMode::JOINT_VELOCITY: return "JOINT_VELOCITY";
      case ControlMode::JOINT_EFFORT:   return "JOINT_EFFORT";
      case ControlMode::ADMITTANCE:     return "ADMITTANCE";
      default:                          return "UNKNOWN";
    }
  }

  static bool isStreamingMode(uint8_t mode)
  {
    return mode == ControlMode::JOINT_VELOCITY || mode == ControlMode::JOINT_EFFORT;
  }

  // ── 切换服务 ────────────────────────────────────────────────────────────────
  void onSwitch(SwitchControlMode::Request::ConstSharedPtr req,
                SwitchControlMode::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lk(switch_mtx_);   // 串行化：任一时刻只准一次切换
    const uint8_t target = req->target_mode;
    const uint8_t from   = current_mode_;

    if (target == from) {
      res->success = true;
      res->active_mode = current_mode_;
      res->message = std::string("已处于 ") + modeName(target) + " 模式（无需切换）";
      return;
    }

    const std::string new_ctrl = controllerFor(target);
    if (new_ctrl.empty()) {
      res->success = false;
      res->active_mode = current_mode_;
      res->message = std::string("模式 ") + modeName(target)
                   + " 未配置控制器（effort/力矩为 P3 预留，需 URDF 加 effort 接口 + 配置 effort_controller）";
      RCLCPP_WARN(get_logger(), "拒绝切换到 %s: %s", modeName(target), res->message.c_str());
      return;
    }
    const std::string old_ctrl = controllerFor(from);

    // 切到流式模式前，若两模式复用同一控制器（如 TRAJECTORY↔ADMITTANCE），只需更新语义
    if (new_ctrl == old_ctrl) {
      current_mode_ = target;
      publishMode();
      res->success = true;
      res->active_mode = current_mode_;
      res->message = std::string("已切到 ") + modeName(target) + "（复用控制器 " + new_ctrl + "）";
      RCLCPP_INFO(get_logger(), "%s → %s（同控制器）", modeName(from), modeName(target));
      return;
    }

    const bool ok = switchController({new_ctrl}, {old_ctrl});
    if (!ok) {
      res->success = false;
      res->active_mode = current_mode_;   // 切换失败，停留在原模式
      res->message = std::string("switch_controller 失败：activate ") + new_ctrl
                   + " / deactivate " + old_ctrl
                   + "（控制器是否已加载？后端是否有 controller_manager？MuJoCo 无）";
      RCLCPP_ERROR(get_logger(), "%s → %s 失败", modeName(from), modeName(target));
      return;
    }

    current_mode_ = target;
    // bumpless 播种：进入速度/力矩模式立即喂 0，并给看门狗一个宽限期
    if (isStreamingMode(target)) {
      publishZero(target);
      last_cmd_time_ = now();
      watchdog_stopped_ = false;
    }
    publishMode();
    res->success = true;
    res->active_mode = current_mode_;
    res->message = std::string("已切到 ") + modeName(target)
                 + "（activate " + new_ctrl + " / deactivate " + old_ctrl + "）";
    RCLCPP_INFO(get_logger(), "%s → %s OK", modeName(from), modeName(target));
  }

  // ── 速度/力矩输入总线 → 控制器命令话题（仅对应模式转发）──────────────────────
  void onCmdInput(uint8_t stream_mode, const Float64MultiArray & msg)
  {
    if (current_mode_ != stream_mode) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "当前非 %s 模式，忽略 %s 命令（先调 /robot_arm/switch_control_mode）",
          modeName(stream_mode), modeName(stream_mode));
      return;
    }
    (stream_mode == ControlMode::JOINT_VELOCITY ? vel_cmd_pub_ : eff_cmd_pub_)->publish(msg);
    last_cmd_time_ = now();
    watchdog_stopped_ = false;
  }

  // ── 看门狗：流式模式命令断流 → 归零一次（ForwardCommandController 不自动归零）──
  void onWatchdog()
  {
    if (!isStreamingMode(current_mode_) || watchdog_stopped_) return;
    if ((now() - last_cmd_time_).seconds() < watchdog_timeout_) return;
    publishZero(current_mode_);
    watchdog_stopped_ = true;   // 只归零一次，等新命令再解除
    RCLCPP_WARN(get_logger(), "%s 命令断流 >%.0fms，已归零兜底（等待新命令）",
                modeName(current_mode_), watchdog_timeout_ * 1000.0);
  }

  void publishZero(uint8_t mode)
  {
    Float64MultiArray z;
    z.data.assign(num_joints_, 0.0);
    (mode == ControlMode::JOINT_VELOCITY ? vel_cmd_pub_ : eff_cmd_pub_)->publish(z);
  }

  void publishMode()
  {
    ControlMode m;
    m.mode = current_mode_;
    mode_pub_->publish(m);
  }

  // ── controller_manager/switch_controller 封装（同步等待，STRICT）─────────────
  bool switchController(const std::vector<std::string> & activate,
                        const std::vector<std::string> & deactivate)
  {
    if (!switch_cli_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "switch_controller 服务不可用（%s）", cm_.c_str());
      return false;
    }
    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers   = activate;
    req->deactivate_controllers = deactivate;
    req->strictness = SwitchController::Request::STRICT;
    auto future = switch_cli_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "switch_controller 超时");
      return false;
    }
    return future.get()->ok;
  }

  // 参数
  std::string cm_, traj_ctrl_, vel_ctrl_, eff_ctrl_;
  std::string vel_in_topic_, vel_cmd_topic_, eff_in_topic_, eff_cmd_topic_;
  size_t num_joints_{3};
  double watchdog_timeout_{0.3};

  // 状态（切换经 switch_mtx_ 串行化；current_mode_ 单写多读，切换/回调都在同一 Reentrant 组）
  std::mutex switch_mtx_;
  uint8_t current_mode_{ControlMode::TRAJECTORY};
  rclcpp::Time last_cmd_time_;
  bool watchdog_stopped_{true};

  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<SwitchController>::SharedPtr switch_cli_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::Service<SwitchControlMode>::SharedPtr switch_srv_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr vel_cmd_pub_, eff_cmd_pub_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr vel_in_sub_, eff_in_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace robot_arm_node::mode

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_arm_node::mode::ModeManagerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;   // 服务回调内同步等 client，须多线程
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
