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
 *   ⑥ 速度总线安全闸（2026-08-04 随笛卡尔速度控制上线）：逐轴限幅 max_joint_velocity +
 *      接近 URDF 关节限位时线性减速到 0。速度控制器不像 JTC 有轨迹校验，撞限位没人拦；
 *      这里是速度指令的唯一必经之路，安全逻辑就集中在这一处（无论指令来自 Director 点动
 *      还是 Commander 的笛卡尔速度换算）。
 *
 * 【约定】
 *   - 必须在机械臂静止时切换；速度/力矩模式下云台 J4-6 不受控（arm_controller 被停），保持当前位置。
 *   - 速度总线类型是产品接口 robot_arm_interfaces/ArmJointVelocityCommand（不是裸数组）；
 *     力矩总线仍是 Float64MultiArray（P3 未落地，等 effort 接口配好再一并类型化）。
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
 *   command_joint_names     默认 [Joint1, Joint2, Joint3]（速度/力矩 claim 的关节，顺序 =
 *                           控制器命令数组顺序；同时决定播种/归零向量长度与限位刹车对象）
 *   max_joint_velocity      默认 1.0（rad/s，速度指令逐轴限幅）
 *   limit_brake_zone        默认 0.15（rad，距关节限位小于此值开始线性减速，到限位处为 0）
 *   watchdog_timeout        默认 0.3（s，速度/力矩命令断流超时归零）
 *   default_mode            默认 0（TRAJECTORY，与 launch 默认激活 arm_controller 一致）
 *
 * @version 1.0
 * @date 2026-07-23
 * @copyright Copyright (c) 2026 EMEET
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <robot_arm_interfaces/msg/arm_joint_velocity_command.hpp>
#include <robot_arm_interfaces/msg/control_mode.hpp>
#include <robot_arm_interfaces/srv/switch_control_mode.hpp>

#include "robot_arm_node/motion/gravity_model.hpp"
#include "robot_arm_node/motion/joint_limits.hpp"

namespace robot_arm_node::mode
{

using namespace std::chrono_literals;
using ControlMode       = robot_arm_interfaces::msg::ControlMode;
using JointVelCommand   = robot_arm_interfaces::msg::ArmJointVelocityCommand;
using SwitchControlMode = robot_arm_interfaces::srv::SwitchControlMode;
using SwitchController   = controller_manager_msgs::srv::SwitchController;
using Float64MultiArray  = std_msgs::msg::Float64MultiArray;
using SetBool            = std_srvs::srv::SetBool;
using JointTrajectory    = trajectory_msgs::msg::JointTrajectory;

class ModeManagerNode : public rclcpp::Node
{
public:
  ModeManagerNode() : rclcpp::Node("mode_manager_node")
  {
    cm_          = declare_parameter<std::string>("controller_manager", "/controller_manager");
    traj_ctrl_   = declare_parameter<std::string>("trajectory_controller", "arm_controller");
    vel_ctrl_    = declare_parameter<std::string>("velocity_controller", "arm_velocity_controller");
    // 速度后端（2026-08-04 新增）：
    //   "trajectory"（默认）—— JOINT_VELOCITY 复用 arm_controller，**不切控制器**；
    //     速度指令由 Commander 的 VelocityStreamServer 积分成位置流走 JTC。
    //     没有控制器切换就没有陈旧命令回放（切回轨迹不再跳变）、轨迹类动作随时可用、
    //     云台不需要额外的保持控制器。
    //   "velocity_controller" —— 老路径：切到 arm_velocity_controller（实物 CiA402 PV(3)），
    //     驱动器内部速度环、延迟更低，代价是上面那些切换副作用。留作高动态场景备选。
    velocity_backend_ = declare_parameter<std::string>("velocity_backend", "trajectory");
    const bool vel_via_traj = velocity_backend_ == "trajectory";
    eff_ctrl_    = declare_parameter<std::string>("effort_controller", "");
    vel_in_topic_  = declare_parameter<std::string>("velocity_input_topic", "/robot_arm/cmd/joint_velocity");
    vel_cmd_topic_ = declare_parameter<std::string>("velocity_command_topic", "/arm_velocity_controller/commands");
    eff_in_topic_  = declare_parameter<std::string>("effort_input_topic", "/robot_arm/cmd/joint_effort");
    eff_cmd_topic_ = declare_parameter<std::string>("effort_command_topic", "/arm_effort_controller/commands");
    hold_ctrls_    = declare_parameter<std::vector<std::string>>(
        "hold_controllers", std::vector<std::string>{});
    cmd_joints_    = declare_parameter<std::vector<std::string>>(
        "command_joint_names", std::vector<std::string>{"Joint1", "Joint2", "Joint3"});
    num_joints_    = cmd_joints_.size();
    // 力矩总线的关节表单独一份：arm_effort_controller 只 claim 臂 J1-3，
    // 而速度/轨迹的关节表将来可能扩到别的轴，别让两者互相牵连。
    eff_joints_    = declare_parameter<std::vector<std::string>>(
        "effort_joint_names", cmd_joints_);
    max_joint_vel_ = declare_parameter<double>("max_joint_velocity", 1.0);
    // 力矩额外总限幅（N·m）。<=0 = 不额外限，只用 URDF <limit effort>。
    // 调试限力时把它调小，比改 URDF 安全（URDF 是多方共用的真相源）。
    max_joint_eff_ = declare_parameter<double>("max_joint_effort", 0.0);
    // 重力补偿：下发力矩 = g(q) + 用户增量。默认开 —— 关掉它力矩模式几乎不可用
    // （零指令即自由下垂，实测 J2 直接从 +0.500 掉到下限 -0.981）。
    // 留开关是为了对比实验和排查（想看纯开环力矩时关掉）。
    grav_enabled_  = declare_parameter<bool>("gravity_compensation", true);
    // 力矩输出频率。JTC 那条 50Hz 的教训（发太快会一直重启轨迹）不适用于这里 ——
    // ForwardCommandController 只是把最后一帧写进命令接口，没有轨迹重规划。
    // 100Hz 与 Gazebo 的 controller_manager update_rate 对齐。
    eff_rate_hz_   = declare_parameter<double>("effort_rate_hz", 100.0);
    // 关节阻尼系数（N·m·s/rad）。0 = 纯重力补偿（会缓慢漂移，见 effortTick 注释）。
    // 调大 = 更"粘"、更稳但手动拖动更费力；调过头会在采样频率上抖。
    eff_damping_   = declare_parameter<double>("effort_damping", 1.5);
    brake_zone_    = declare_parameter<double>("limit_brake_zone", 0.15);
    watchdog_timeout_ = declare_parameter<double>("watchdog_timeout", 0.3);
    traj_cmd_topic_ = declare_parameter<std::string>(
        "trajectory_command_topic", "/arm_controller/joint_trajectory");
    traj_joints_ = declare_parameter<std::vector<std::string>>(
        "trajectory_joint_names",
        std::vector<std::string>{"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"});
    resync_time_ = declare_parameter<double>("resync_time", 0.3);
    current_mode_  = static_cast<uint8_t>(declare_parameter<int>("default_mode", ControlMode::TRAJECTORY));
    via_traj_      = vel_via_traj;

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

    // 速度/力矩：产品面向输入总线 → 控制器命令话题（仅对应模式下转发）。
    // trajectory 后端下这条转发链不启用 —— 速度总线归 Commander 的 VelocityStreamServer，
    // 由它积分成位置流走 JTC；这里再订阅一份会变成双重驱动。
    if (!vel_via_traj) {
      vel_cmd_pub_ = create_publisher<Float64MultiArray>(vel_cmd_topic_, rclcpp::QoS(10));
      vel_in_sub_  = create_subscription<JointVelCommand>(
          vel_in_topic_, rclcpp::QoS(10),
          [this](const JointVelCommand & m) { onVelocityInput(m); });
    }
    eff_cmd_pub_ = create_publisher<Float64MultiArray>(eff_cmd_topic_, rclcpp::QoS(10));
    eff_in_sub_  = create_subscription<Float64MultiArray>(
        eff_in_topic_, rclcpp::QoS(10),
        [this](const Float64MultiArray & m) { onEffortInput(m); });

    // 急停闩锁：Commander 的 ArmStop / ArmResetError 经此服务通知本节点。
    // 速度模式下 ArmStop 必须真的能停车 —— 而速度指令的发布方可能是 Commander
    // （笛卡尔速度流），也可能是 Director 直发关节速度，光靠 Commander 停自己那条流
    // 拦不住后者。闩在总线这个必经之路上：一旦置位，立刻喂 0 并丢弃所有速度指令，
    // 直到 ArmResetError 解除。
    estop_srv_ = create_service<SetBool>(
        "/robot_arm/velocity_estop",
        [this](SetBool::Request::ConstSharedPtr req, SetBool::Response::SharedPtr res) {
          estop_.store(req->data);
              if (req->data && isStreamingMode(current_mode_)) publishZero(current_mode_);
          res->success = true;
          res->message = req->data ? "速度总线已急停闩锁（喂 0 并丢弃指令）" : "速度总线急停已解除";
          RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        },
        rmw_qos_profile_services_default, cbg_);

    traj_cmd_pub_ = create_publisher<JointTrajectory>(traj_cmd_topic_, rclcpp::QoS(10));

    // 限位刹车：URDF 限位（latched /robot_description）+ /joint_states 当前位置
    limits_ = std::make_unique<motion::JointLimitsCache>(*this);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState & m) { onJointState(m); });

    // 重力补偿：同样从 latched /robot_description 建模（限位/惯量单一真相源都是 URDF）
    if (grav_enabled_) {
      gravity_ = std::make_unique<motion::GravityModel>(*this);
    }
    user_eff_.assign(eff_joints_.size(), 0.0);
    // 力矩输出节拍。常驻而非按模式起停：定时器生命周期跟着模式切换走容易漏关/重复建，
    // 而 effortTick() 首行就在非力矩模式下 return，100Hz 空转的代价可以忽略。
    eff_timer_ = create_wall_timer(
        std::chrono::microseconds(static_cast<int64_t>(1e6 / std::max(1.0, eff_rate_hz_))),
        std::bind(&ModeManagerNode::effortTick, this), cbg_);

    // 看门狗：速度/力矩模式下命令断流超时 → 归零兜底
    last_cmd_time_ = now();
    watchdog_ = create_wall_timer(50ms, std::bind(&ModeManagerNode::onWatchdog, this), cbg_);

    publishMode();   // 广播初始模式（= launch 默认激活的控制器，无需切换）
    RCLCPP_INFO(get_logger(),
        "mode_manager_node 就绪  cm=%s\n"
        "  服务  /robot_arm/switch_control_mode\n"
        "  广播  /robot_arm/control_mode (latched)  初始模式=%s\n"
        "  速度总线  %s (ArmJointVelocityCommand, %zu 轴) → %s\n"
        "            看门狗 %.0fms  限幅 %.2frad/s  限位减速区 %.3frad",
        cm_.c_str(), modeName(current_mode_), vel_in_topic_.c_str(), num_joints_,
        vel_cmd_topic_.c_str(), watchdog_timeout_ * 1000.0, max_joint_vel_, brake_zone_);
  }

private:
  // ── 模式 → 控制器名 ─────────────────────────────────────────────────────────
  std::string controllerFor(uint8_t mode) const
  {
    switch (mode) {
      case ControlMode::TRAJECTORY:     return traj_ctrl_;
      // trajectory 后端下速度模式复用轨迹控制器 —— 与 TRAJECTORY 同名，
      // onSwitch 里 new_ctrl==old_ctrl 的分支会把切换降级成「只更新语义」，不动控制器
      case ControlMode::JOINT_VELOCITY: return via_traj_ ? traj_ctrl_ : vel_ctrl_;
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

    // 云台保持控制器（hold_controllers）：进入流式模式时与速度/力矩控制器一起激活，
    // 回轨迹模式时一起停用。
    //   为什么需要：速度控制器只 claim 臂 J1-3，而 arm_controller 在 Gazebo/实物配置里
    //   claim 的是 J1-6 —— 一旦被停，云台 J4-6 在 Gazebo 里就没人命令，会在重力下垂，
    //   末端 gimbal_tool0 随之漂移（笛卡尔速度控制的末端轨迹直接被带偏）。挂一个只管
    //   J4-6 的 JTC（激活即锁当前位姿）把「速度模式下云台保持当前位置」这条承诺做实。
    //   实物默认留空：云台由板端 robot_gimbal_node_v2 自己保持，不需要这层。
    //
    //   两条编排规则（都是踩出来的）：
    //   ① **停** 保持控制器必须和主切换在同一次 STRICT 调用里原子完成：分两次调的话，
    //      第一次 deactivate 还没被 controller_manager 的实时循环应用，第二次就要
    //      activate 同样 claim J4-6 的 arm_controller，STRICT 判资源冲突而失败，
    //      机械臂卡在速度模式里回不去（实测复现）。
    //   ② **启** 只能在主切换之后单独调（BEST_EFFORT）：arm_controller 停用前 J4-6
    //      被它占着，保持控制器起不来。
    //   保持控制器的死活由本节点自己记账（holds_active_）—— 它是唯一的操作者；
    //   万一被外部手动动过导致 STRICT 失败，下面有一次「不带保持控制器」的重试兜底。
    const bool to_stream   = isStreamingMode(target);
    const bool from_stream = isStreamingMode(from);

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

    // 复位轨迹要用**切换前**的实测位置：切换后硬件会在一两拍内把陈旧位置命令下发，
    // 那时再读回读已经是跳变后的值了（照着它复位等于把跳变固化下来，实测踩过）。
    std::vector<double> pre_switch_pos;
    if (!to_stream && from_stream) pre_switch_pos = snapshotTrajectoryJoints();

    std::vector<std::string> deactivate{old_ctrl};
    if (!to_stream && from_stream && holds_active_) {
      deactivate.insert(deactivate.end(), hold_ctrls_.begin(), hold_ctrls_.end());
    }

    bool ok = switchController({new_ctrl}, deactivate);
    if (!ok && deactivate.size() > 1) {
      // 兜底：保持控制器可能被外部动过（手动 stop / 没加载），别让它拖死主切换
      RCLCPP_WARN(get_logger(), "带保持控制器的切换失败，退回只切主控制器重试一次");
      holds_active_ = false;
      ok = switchController({new_ctrl}, {old_ctrl});
    }
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
    if (!to_stream && from_stream) {
      holds_active_ = false;              // 已随主切换一并停用
      resyncTrajectoryTo(pre_switch_pos); // 见函数注释：把 JTC 拉回切换前的位置
    }
    // bumpless 播种：进入速度/力矩模式立即喂 0，并给看门狗一个宽限期
    if (isStreamingMode(target)) {
      publishZero(target);
      last_cmd_time_ = now();
      watchdog_stopped_ = false;
      // 进入流式模式：主切换已让出 J4-6，此时才能挂保持控制器（激活即锁当前位姿）
      if (!from_stream && !hold_ctrls_.empty() && !holds_active_) {
        holds_active_ = switchController(hold_ctrls_, {},
                                         SwitchController::Request::BEST_EFFORT);
      }
    }
    publishMode();
    res->success = true;
    res->active_mode = current_mode_;
    res->message = std::string("已切到 ") + modeName(target)
                 + "（activate " + new_ctrl + " / deactivate " + old_ctrl + "）";
    RCLCPP_INFO(get_logger(), "%s → %s OK", modeName(from), modeName(target));
  }

  // ── 速度输入总线（产品接口类型）→ 限幅 + 限位刹车 → 控制器命令话题 ──────────────
  void onVelocityInput(const JointVelCommand & msg)
  {
    if (estop_.load()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "速度总线处于急停闩锁，指令已丢弃（调 /robot_arm/reset_error 解除）");
      return;
    }
    if (current_mode_ != ControlMode::JOINT_VELOCITY) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "当前非 JOINT_VELOCITY 模式，忽略速度命令（先调 /robot_arm/switch_control_mode）");
      return;
    }
    if (msg.velocities.size() != num_joints_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "速度命令长度 %zu ≠ 命令关节数 %zu，已丢弃（顺序须为 %s）",
          msg.velocities.size(), num_joints_, jointListStr().c_str());
      return;
    }

    Float64MultiArray out;
    out.data = msg.velocities;
    for (size_t i = 0; i < num_joints_; ++i) {
      out.data[i] = std::clamp(out.data[i], -max_joint_vel_, max_joint_vel_);
      out.data[i] = brakeNearLimit(cmd_joints_[i], out.data[i]);
    }

    vel_cmd_pub_->publish(out);
    last_cmd_time_ = now();
    watchdog_stopped_ = false;
  }

  // ── 换回轨迹模式后，把 JTC 目标重设为当前实测位置 ────────────────────────────
  //   ros2_control 的命令接口在控制器停用后**值原样保留**：速度模式下机械臂走开了，
  //   而 arm_controller 的位置命令缓冲还停在进入速度模式前那一刻。JTC 重新激活后，
  //   硬件下一拍就把这个陈旧目标下发 —— 机械臂会冲回旧位姿（Gazebo 里是瞬移）。
  //   根治要在硬件层换模时把目标播种成当前位置：实物侧已经这么做了
  //   （robot_arm_driver UnwrapRobotSystem::perform_command_mode_switch）；
  //   Gazebo 的 GazeboSystem 是第三方且 pImpl 私有，改不到，只能在这里补救 ——
  //   激活后立刻发一条「回到当前位置」的轨迹，把机械臂拉回来。
  //   所以**仿真里切回轨迹模式仍会看到一次快速的往返**，实物没有。
  // 取 traj_joints_ 的当前回读快照；任一关节缺回读则返回空表（调用方跳过复位）
  std::vector<double> snapshotTrajectoryJoints()
  {
    std::vector<double> out;
    std::lock_guard<std::mutex> lk(joint_mtx_);
    for (const auto & j : traj_joints_) {
      auto it = joint_pos_.find(j);
      if (it == joint_pos_.end()) {
        RCLCPP_WARN(get_logger(), "换模复位：没有 '%s' 的回读，跳过复位轨迹", j.c_str());
        return {};
      }
      out.push_back(it->second);
    }
    return out;
  }

  void resyncTrajectoryTo(const std::vector<double> & positions)
  {
    if (traj_joints_.empty() || positions.size() != traj_joints_.size()) return;

    JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    traj.joint_names = traj_joints_;
    pt.positions = positions;
    pt.time_from_start = rclcpp::Duration::from_seconds(resync_time_);
    traj.points.push_back(std::move(pt));

    // 连发几帧：激活后头几拍 JTC 可能还没就绪，丢一两帧不影响
    for (int i = 0; i < 5; ++i) {
      traj_cmd_pub_->publish(traj);
      std::this_thread::sleep_for(10ms);
    }
    RCLCPP_INFO(get_logger(), "换回轨迹模式：已下发「保持当前位置」轨迹（%.2fs）复位 JTC 目标",
                resync_time_);
  }

  // ── 力矩输入总线（2026-08-11 开通，仿真先行；仍是裸数组，待类型化）───────────────
  //   与速度总线的安全闸对齐：急停闩锁 → 模式闸 → 长度校验 → 限幅 → 限位刹车。
  //   单位 N·m（与 URDF <limit effort> 一致）。实物侧 6071 是「0.1% 额定转矩」，
  //   N·m ↔ 0.1% 的换算要落在这一层（vendored ros2_canopen 的 scale_eff_* 上游未实现，
  //   按 VENDOR.md 的约定不改 vendored 源码）—— 需要手册里的额定转矩才能定系数，
  //   所以现在只跑仿真，real.launch.py 不配 effort_controller。
  //   用户指令只是「在重力之上的增量」，不直接下发 —— 存起来，由 effortTick()
  //   以固定频率与 g(q) 相加后统一发。这样「不发指令」= 原地悬停而不是自由下垂。
  void onEffortInput(const Float64MultiArray & msg)
  {
    if (estop_.load()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "力矩总线处于急停闩锁，指令已丢弃（调 /robot_arm/reset_error 解除）");
      return;
    }
    if (current_mode_ != ControlMode::JOINT_EFFORT) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "当前非 JOINT_EFFORT 模式，忽略力矩命令（先调 /robot_arm/switch_control_mode）");
      return;
    }
    if (msg.data.size() != eff_joints_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "力矩命令长度 %zu ≠ 力矩关节数 %zu，已丢弃（顺序须为 %s）",
          msg.data.size(), eff_joints_.size(), effJointListStr().c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lk(eff_mtx_);
      user_eff_ = msg.data;
    }
    last_cmd_time_ = now();
    watchdog_stopped_ = false;
  }

  // ── 力矩输出节拍：g(q) + 用户增量 → 限幅 → 限位刹车 → 控制器 ──────────────────
  //   为什么必须是定时器而不是「收到指令才发」：g(q) 随位形变，而且**不发指令时也
  //   必须持续下发** —— ForwardCommandController 会一直把最后一帧命令写给硬件，
  //   位形一变那帧重力力矩就不再平衡；更要紧的是「用户松手」不能退化成零力矩，
  //   否则又变成自由下垂（这正是加重力补偿要解决的问题）。
  void effortTick()
  {
    if (current_mode_ != ControlMode::JOINT_EFFORT) return;

    std::vector<double> user;
    {
      std::lock_guard<std::mutex> lk(eff_mtx_);
      user = user_eff_;
    }
    if (user.size() != eff_joints_.size()) user.assign(eff_joints_.size(), 0.0);

    // 急停 / 断流：用户增量清零，但**重力补偿继续** —— 急停的语义是「停住」，
    // 而力矩模式下"停住"就是维持 g(q)；喂 0 才是让它砸下去。
    if (estop_.load() || watchdog_stopped_) {
      std::fill(user.begin(), user.end(), 0.0);
    }

    std::map<std::string, double> q, qd;
    {
      std::lock_guard<std::mutex> lk(joint_mtx_);
      q  = joint_pos_;
      qd = joint_vel_;
    }

    std::vector<double> grav(eff_joints_.size(), 0.0);
    if (grav_enabled_ && gravity_) {
      if (!gravity_->gravity(q, eff_joints_, grav)) {
        // 拿不到 g(q)（模型未就绪 / 回读不全）→ 不下发任何力矩。
        // fail-closed：宁可让 JTC 之前的保持失效（关节靠限位停住），也不能把
        // 「只有用户增量、没有重力项」的力矩发下去 —— 那等于主动往下推。
        std::fill(grav.begin(), grav.end(), 0.0);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "重力补偿不可用（模型未就绪或 /joint_states 位形不全），力矩输出已抑制");
        return;
      }
    }

    Float64MultiArray out;
    out.data.resize(eff_joints_.size());
    for (size_t i = 0; i < eff_joints_.size(); ++i) {
      // 上限取 URDF <limit effort>。**拿不到限值就整条丢弃（fail-closed）** ——
      // 这与位置校验的 fail-open 约定故意相反：力矩模式没有位置闭环，限值是唯一的
      // 安全边界，缺了它还下发等于放弃兜底；而写死一个常量就是制造第二份真相
      // （见 joint_limits.hpp 开头的说明），改了 URDF 必然漏同步。
      auto lim = limits_->limit(eff_joints_[i]);
      if (!lim || lim->effort <= 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "取不到 '%s' 的 URDF <limit effort>，力矩输出已抑制（URDF 未就绪或未给 effort）",
            eff_joints_[i].c_str());
        return;
      }
      const double cap = max_joint_eff_ > 0.0 ? std::min(lim->effort, max_joint_eff_)
                                              : lim->effort;
      // 重力项若本身就顶到限幅，说明该轴的 <limit effort> 撑不住自重 —— 补偿会被
      // 削弱、关节仍会往下走。这是配置问题（URDF 限值/负载不匹配），必须让人知道。
      if (grav_enabled_ && std::fabs(grav[i]) > cap) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "'%s' 的重力力矩 %.2f N·m 超出限幅 %.2f N·m —— 补偿被削弱，该轴仍会下坠"
            "（检查 URDF <limit effort> 或 max_joint_effort）",
            eff_joints_[i].c_str(), grav[i], cap);
      }
      // 阻尼项 −d·q̇：纯重力补偿是无耗散的，模型与仿真的任何残差（离散化、惯量
      // 差异、命令延迟一拍）都会积分成持续漂移 —— 实测无阻尼时 8s 内 J2 上飘
      // 0.100 rad、J3 上飘 0.046 rad，松手后 J1 还会一直滑。加一点速度负反馈把
      // 能量耗掉，「悬停」才真的停得住，手动拖动时的手感也更像有阻尼的真实关节。
      // 只对本轴速度做反馈，不做全耦合 —— 目的是耗散而非解耦。
      double damp = 0.0;
      if (eff_damping_ > 0.0) {
        auto it = qd.find(eff_joints_[i]);
        if (it != qd.end()) damp = -eff_damping_ * it->second;
      }
      out.data[i] = std::clamp(grav[i] + user[i] + damp, -cap, cap);
      // 复用速度那套方向性衰减：只削「朝限位方向」的力矩，反向（把关节从限位拉回来）
      // 不受限。力矩模式下位置不闭环，这是唯一能阻止硬顶限位的软措施。
      // 注意作用在总量上：顶到限位时连重力项一起削，让关节软着落在限位上而不是硬砸。
      out.data[i] = brakeNearLimit(eff_joints_[i], out.data[i]);
    }
    eff_cmd_pub_->publish(out);
  }

  // ── 限位刹车：朝限位方向的速度在减速区内线性衰减，到限位处为 0 ──────────────────
  //   反向（撤离限位）不受限，否则一旦压到限位就再也开不回来。
  //   URDF 未就绪 / 该轴无限位 / 无该轴回读 → 不拦（fail-open，与 JointLimitsCache 约定一致）。
  double brakeNearLimit(const std::string & joint, double v)
  {
    if (brake_zone_ <= 0.0 || std::fabs(v) < 1e-9) return v;
    auto lim = limits_->limit(joint);
    if (!lim) return v;

    double pos;
    {
      std::lock_guard<std::mutex> lk(joint_mtx_);
      auto it = joint_pos_.find(joint);
      if (it == joint_pos_.end()) return v;
      pos = it->second;
    }

    const double margin = v > 0.0 ? (lim->upper - pos) : (pos - lim->lower);
    if (margin >= brake_zone_) return v;

    const double scale = std::clamp(margin / brake_zone_, 0.0, 1.0);
    if (scale <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "%s 已抵限位（pos=%.3f, [%.3f, %.3f]），朝限位方向的速度已归零",
          joint.c_str(), pos, lim->lower, lim->upper);
    }
    return v * scale;
  }

  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    std::lock_guard<std::mutex> lk(joint_mtx_);
    const size_t n = std::min(msg.name.size(), msg.position.size());
    for (size_t i = 0; i < n; ++i) joint_pos_[msg.name[i]] = msg.position[i];
    // 速度回读供力矩模式的阻尼项用（纯重力补偿没有耗散，残差会积分成漂移）
    const size_t nv = std::min(msg.name.size(), msg.velocity.size());
    for (size_t i = 0; i < nv; ++i) joint_vel_[msg.name[i]] = msg.velocity[i];
  }

  std::string jointListStr() const
  {
    std::string s;
    for (const auto & j : cmd_joints_) s += (s.empty() ? "" : ", ") + j;
    return s;
  }

  std::string effJointListStr() const
  {
    std::string s;
    for (const auto & j : eff_joints_) s += (s.empty() ? "" : ", ") + j;
    return s;
  }

  // ── 看门狗：流式模式命令断流 → 归零一次（ForwardCommandController 不自动归零）──
  void onWatchdog()
  {
    // trajectory 后端下速度总线不经本节点（归 Commander 的 VelocityStreamServer，
    // 它自己有断流看门狗），这里不该报「命令断流」——否则一进速度模式就刷一条假告警
    if (via_traj_ && current_mode_ == ControlMode::JOINT_VELOCITY) return;
    if (!isStreamingMode(current_mode_) || watchdog_stopped_) return;
    if ((now() - last_cmd_time_).seconds() < watchdog_timeout_) return;
    publishZero(current_mode_);
    watchdog_stopped_ = true;   // 只归零一次，等新命令再解除
    RCLCPP_WARN(get_logger(), "%s 命令断流 >%.0fms，已归零兜底（等待新命令）",
                modeName(current_mode_), watchdog_timeout_ * 1000.0);
  }

  //   「归零」在两种流式模式下语义不同：
  //     速度模式：零速度 = 停住 → 直接把 0 写给控制器。
  //     力矩模式：零力矩 = **自由下垂**，不是停住。所以这里只把「用户增量」清零，
  //               不碰控制器命令 —— 由 effortTick() 继续发 g(q) 把臂托住。
  //               （早期版本这里对力矩也发字面 0，效果就是急停/断流时臂直接砸下去。）
  void publishZero(uint8_t mode)
  {
    if (mode == ControlMode::JOINT_EFFORT) {
      std::lock_guard<std::mutex> lk(eff_mtx_);
      user_eff_.assign(eff_joints_.size(), 0.0);
      return;
    }
    auto pub = vel_cmd_pub_;
    if (!pub) return;   // trajectory 后端下速度控制器不参与，没有这个发布者
    Float64MultiArray z;
    z.data.assign(num_joints_, 0.0);
    pub->publish(z);
  }

  void publishMode()
  {
    ControlMode m;
    m.mode = current_mode_;
    mode_pub_->publish(m);
  }

  // ── controller_manager/switch_controller 封装（同步等待）─────────────────────
  //   主切换用 STRICT（原子停旧启新，失败即整体回滚）；
  //   保持控制器（云台）用 BEST_EFFORT，状态不符不许拖垮模式切换。
  bool switchController(const std::vector<std::string> & activate,
                        const std::vector<std::string> & deactivate,
                        uint8_t strictness = SwitchController::Request::STRICT)
  {
    if (!switch_cli_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "switch_controller 服务不可用（%s）", cm_.c_str());
      return false;
    }
    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers   = activate;
    req->deactivate_controllers = deactivate;
    req->strictness = strictness;
    auto future = switch_cli_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "switch_controller 超时");
      return false;
    }
    return future.get()->ok;
  }

  // 参数
  std::string cm_, traj_ctrl_, vel_ctrl_, eff_ctrl_;
  std::string velocity_backend_;
  bool via_traj_{true};   // velocity_backend == "trajectory"（速度走 JTC 位置流）
  std::string vel_in_topic_, vel_cmd_topic_, eff_in_topic_, eff_cmd_topic_;
  std::vector<std::string> hold_ctrls_;   // 流式模式期间陪跑的保持控制器（云台）
  bool holds_active_{false};              // 保持控制器当前是否已激活（本节点自己记账）
  std::vector<std::string> cmd_joints_;
  size_t num_joints_{3};
  std::vector<std::string> eff_joints_;   // 力矩总线的关节表（arm_effort_controller 的 joints）
  double max_joint_vel_{1.0};
  double max_joint_eff_{0.0};             // N·m；<=0 = 只用 URDF <limit effort>
  bool   grav_enabled_{true};             // 力矩模式是否叠加 g(q)
  double eff_rate_hz_{100.0};             // 力矩输出节拍
  double eff_damping_{1.5};               // 关节阻尼 N·m·s/rad（0 = 纯重力补偿）
  std::unique_ptr<motion::GravityModel> gravity_;
  rclcpp::TimerBase::SharedPtr eff_timer_;
  std::vector<double> user_eff_;          // 用户力矩增量（N·m），由 effortTick 消费
  std::mutex eff_mtx_;
  double brake_zone_{0.15};
  double watchdog_timeout_{0.3};
  std::string traj_cmd_topic_;
  std::vector<std::string> traj_joints_;
  double resync_time_{0.3};

  // 限位刹车所需的两份数据：URDF 限位 + 实时关节位置
  std::unique_ptr<motion::JointLimitsCache> limits_;
  std::mutex joint_mtx_;
  std::map<std::string, double> joint_pos_;
  std::map<std::string, double> joint_vel_;

  // 状态（切换经 switch_mtx_ 串行化；current_mode_ 单写多读，切换/回调都在同一 Reentrant 组）
  std::mutex switch_mtx_;
  std::atomic<bool> estop_{false};   // 急停闩锁（ArmStop 置位 / ArmResetError 解除）
  uint8_t current_mode_{ControlMode::TRAJECTORY};
  rclcpp::Time last_cmd_time_;
  bool watchdog_stopped_{true};

  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<SwitchController>::SharedPtr switch_cli_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::Service<SwitchControlMode>::SharedPtr switch_srv_;
  rclcpp::Service<SetBool>::SharedPtr           estop_srv_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr vel_cmd_pub_, eff_cmd_pub_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr  traj_cmd_pub_;
  rclcpp::Subscription<JointVelCommand>::SharedPtr   vel_in_sub_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr eff_in_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
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
