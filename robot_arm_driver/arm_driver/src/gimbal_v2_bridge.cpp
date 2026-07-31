/**
 * @file gimbal_v2_bridge.cpp
 * @brief 云台 V2 适配节点：臂侧 ros2_control 转发约定 ⇄ robot_gimbal_interfaces_v2 语义接口
 *
 * 2026-07-28 云台由 V1（二轴 eMeetCamera / USB HID）换为 V2（C-200T 三轴 GCU 云台）后，
 * 云台执行节点（robot_gimbal_node_v2，串口唯一拥有者）**不在本工作空间运行**，
 * 而是跑在云台自己的板端。臂侧持有全 6 轴 URDF（J1-3 臂 + J4-6 云台 V2）做规划，
 * 规划结果经话题发给云台、云台状态再经话题收回来。本节点就是这层翻译：
 *
 *   ┌ 臂侧 ────────────────────────────────────────────────────────────────┐
 *   │ arm_controller (JTC, 6 轴)                                            │
 *   │        │ write()                                                      │
 *   │        ▼                                                              │
 *   │ robot_gimbal_driver_v2/GimbalForwardingInterface（变化检测）           │
 *   │        │ sensor_msgs/JointState  /robot_gimbal_v2/forward_cmd            │
 *   └────────┼──────────────────────────────────────────────────────────────┘
 *            ▼                              ← 本节点 →
 *     GimbalCommand(POSITION)  /robot_gimbal_v2/gimbal_cmd    ──▶ 云台板端
 *     GimbalStatus             /robot_gimbal_v2/status        ◀── 云台板端
 *            │
 *   ┌────────▼──────────────────────────────────────────────────────────────┐
 *   │ sensor_msgs/JointState  /robot_gimbal_v2/joint_states_raw                │
 *   │        │ GimbalForwardingInterface.read() 回填                        │
 *   │        ▼                                                              │
 *   │ joint_state_broadcaster → 统一 6 轴 /joint_states（TF 完整）           │
 *   └───────────────────────────────────────────────────────────────────────┘
 *
 * 轴映射（与 V2 的 GimbalCommand.msg / robot_gimbal_node 一致）：
 *   pan  = Joint4 (Yaw)   roll = Joint5 (Roll)   tilt = Joint6 (Pitch)
 * 单位一律 rad / rad·s⁻¹；度换算与每轴符号翻转由云台板端的 robot_gimbal_node 负责，
 * 本节点**不做任何单位换算或取负**。
 *
 * ══ 参考系换算（absolute_mode，2026-07-29 新增）═══════════════════════════════
 * 关键差异：臂侧 URDF 的 Joint4-6 是**相对云台基座**的关节角；而云台板端的位置指令
 * 走 GCU「绝对角」（IMU 惯性系，见 robot_gimbal_node.cpp 的 angle_spec 注释），
 * 回读默认也是 cam_angle（IMU 绝对角，use_motor_angle=false）。
 * 云台基座骑在机械臂上，会随 Joint1／臂位形一起转，两个参考系相差「基座姿态」。
 *
 * 症状（2026-07-29 实机）：环绕运镜时 Joint4 方向相反 —— 环绕中基座偏航与云台相对
 * pan 天然反向变化（臂往一边转、云台往另一边补才能盯住球心），把相对角当绝对角
 * 下发，云台就朝反方向甩。仿真里 GazeboSystem 按关节角施加，所以正常。
 *
 * 换算（本节点做的事）：
 *   云台头姿态（相对基座）f(q) = Rz(-J4)·Ry(-J5)·Rx(J6)
 *     ↑ 由 URDF 导出并数值核对：R_head_base(0,0,0)=I，Joint4 绕基座 -Z、
 *       Joint5 绕 -Y、Joint6 绕 +X，故为标准 ZYX 内旋欧拉组、闭式可逆。
 *   指令： q_abs = f⁻¹( R_base_world · f(q_rel) )     发 pan/roll/tilt
 *   回读： q_rel = f⁻¹( R_base_world⁻¹ · f(q_abs) )   发 joint_states_raw
 *   R_base_world 由 TF（world_frame ← gimbal_base_frame）取。
 *   基座与世界对齐时该换算退化为恒等 —— 与云台单机标定（gimbal_sim.urdf.xacro
 *   把基座挂在 world 上、rpy=0）时的行为完全一致，故板端 sign_*／限位不用重标。
 *
 * ⚠ yaw 基准：IMU 的 yaw 没有绝对零参考（GO_ZERO 也不归 yaw）。本节点的换算保证
 *   **增量正确**（这就是环绕反向的修复），但 pan 的常值基准由 IMU 上电时刻决定；
 *   若发现 /joint_states 的 Joint4 与实物机械角差一个常数，用 yaw_datum_rad 补。
 *   roll／tilt 以重力为基准，是真绝对角，无此问题。
 *   TF 取不到时自动退化为直通（相对角原样下发）并节流告警。
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * 为什么不直接把 forward_cmd 发给云台板端（V2 node 本身也订阅 forward_cmd）：
 * 走 GimbalCommand 语义接口才能进云台侧的仲裁状态机
 * （FREEZE > 显式位置类 > 转发流 > 速度流），并拿到 GimbalStatus 的健康字段。
 *
 * 参数：
 *   forward_cmd_topic   默认 "/robot_gimbal_v2/forward_cmd"     （订阅，来自转发插件）
 *   gimbal_cmd_topic    默认 "/robot_gimbal_v2/gimbal_cmd"      （发布，给云台板端）
 *   status_topic        默认 "/robot_gimbal_v2/status"          （订阅，来自云台板端）
 *   joint_states_topic  默认 "/robot_gimbal_v2/joint_states_raw"（发布，回填转发插件）
 *   joint_names         默认 ["Joint4","Joint5","Joint6"]（顺序即 pan/roll/tilt）
 *   max_angular_vel     默认 0.0（≤0 = 用云台驱动层默认）
 *   status_timeout_sec  默认 1.0（超时未收到 GimbalStatus 则节流告警）
 *   absolute_mode       默认 true（相对关节角 ⇄ 绝对姿态换算；false = 直通，旧行为）
 *   world_frame         默认 "arm_base_link"（惯性参考系）
 *   gimbal_base_frame   默认 "gimbal_base_link"
 *   yaw_datum_rad       默认 0.0（pan 常值基准补偿，见上方 yaw 基准说明）
 *   tf_timeout_sec      默认 0.1（查 TF 的等待上限）
 *
 * @date 2026-07-28
 * @copyright Copyright (c) 2026 EMEET
 */

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_gimbal_interfaces_v2/msg/gimbal_command.hpp>
#include <robot_gimbal_interfaces_v2/msg/gimbal_status.hpp>

namespace
{
constexpr size_t AXES = 3;   // pan / roll / tilt

// 云台头姿态（相对云台基座）= Rz(-J4)·Ry(-J5)·Rx(J6)
// 由 URDF 导出并数值核对：R_head_base(0,0,0)=I，J4 绕基座 -Z / J5 绕 -Y / J6 绕 +X。
tf2::Matrix3x3 head_rot(double j4, double j5, double j6)
{
  tf2::Quaternion q;                 // setRPY 用固定轴 rpy（= Rz(y)Ry(p)Rx(r)）
  q.setRPY(j6, -j5, -j4);            // roll=+J6, pitch=-J5, yaw=-J4
  return tf2::Matrix3x3(q);
}

// head_rot 的闭式逆：姿态 → (J4, J5, J6)
std::array<double, AXES> head_angles(const tf2::Matrix3x3 & M)
{
  const double m20 = M[2][0];
  const double j5 = std::asin(std::clamp(m20, -1.0, 1.0));           // pitch = -J5
  const double j4 = -std::atan2(M[1][0], M[0][0]);                   // yaw   = -J4
  const double j6 = std::atan2(M[2][1], M[2][2]);                    // roll   = +J6
  return {j4, j5, j6};
}
}

class GimbalV2Bridge : public rclcpp::Node
{
public:
  using JointState    = sensor_msgs::msg::JointState;
  using GimbalCommand = robot_gimbal_interfaces_v2::msg::GimbalCommand;
  using GimbalStatus  = robot_gimbal_interfaces_v2::msg::GimbalStatus;

  GimbalV2Bridge() : Node("gimbal_v2_bridge")
  {
    const auto forward_cmd_topic  = declare_parameter<std::string>(
      "forward_cmd_topic", "/robot_gimbal_v2/forward_cmd");
    const auto gimbal_cmd_topic   = declare_parameter<std::string>(
      "gimbal_cmd_topic", "/robot_gimbal_v2/gimbal_cmd");
    const auto status_topic       = declare_parameter<std::string>(
      "status_topic", "/robot_gimbal_v2/status");
    const auto joint_states_topic = declare_parameter<std::string>(
      "joint_states_topic", "/robot_gimbal_v2/joint_states_raw");

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"Joint4", "Joint5", "Joint6"});
    if (joint_names_.size() != AXES) {
      RCLCPP_FATAL(get_logger(),
        "joint_names 必须是 3 个（pan/roll/tilt 顺序），实际 %zu 个", joint_names_.size());
      throw std::runtime_error("invalid joint_names");
    }

    max_angular_vel_    = declare_parameter<double>("max_angular_vel", 0.0);
    status_timeout_sec_ = declare_parameter<double>("status_timeout_sec", 1.0);

    absolute_mode_      = declare_parameter<bool>("absolute_mode", true);
    world_frame_        = declare_parameter<std::string>("world_frame", "arm_base_link");
    gimbal_base_frame_  = declare_parameter<std::string>("gimbal_base_frame", "gimbal_base_link");
    yaw_datum_rad_      = declare_parameter<double>("yaw_datum_rad", 0.0);
    tf_timeout_sec_     = declare_parameter<double>("tf_timeout_sec", 0.1);

    if (absolute_mode_) {
      tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    }

    cmd_pub_ = create_publisher<GimbalCommand>(gimbal_cmd_topic, rclcpp::QoS(10).reliable());
    js_pub_  = create_publisher<JointState>(joint_states_topic, rclcpp::QoS(10));

    forward_cmd_sub_ = create_subscription<JointState>(
      forward_cmd_topic, rclcpp::QoS(10).reliable(),
      std::bind(&GimbalV2Bridge::on_forward_cmd, this, std::placeholders::_1));
    status_sub_ = create_subscription<GimbalStatus>(
      status_topic, rclcpp::QoS(10),
      std::bind(&GimbalV2Bridge::on_status, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "云台 V2 适配就绪\n"
      "  %s (JointState) → %s (GimbalCommand.POSITION)\n"
      "  %s (GimbalStatus) → %s (JointState)\n"
      "  轴映射: pan=%s  roll=%s  tilt=%s\n"
      "  参考系: %s（相对关节角 ⇄ 绝对姿态，TF %s ← %s，yaw 基准补偿 %.4f rad）",
      forward_cmd_topic.c_str(), gimbal_cmd_topic.c_str(),
      status_topic.c_str(), joint_states_topic.c_str(),
      joint_names_[0].c_str(), joint_names_[1].c_str(), joint_names_[2].c_str(),
      absolute_mode_ ? "absolute_mode=true" : "absolute_mode=false（直通，旧行为）",
      world_frame_.c_str(), gimbal_base_frame_.c_str(), yaw_datum_rad_);
  }

private:
  // 取 R_base_world（world_frame ← gimbal_base_frame 的旋转）。
  // 取不到返回 false，调用方退化为直通（相对角原样收发）。
  bool base_rot(tf2::Matrix3x3 & R)
  {
    if (!absolute_mode_ || !tf_buffer_) return false;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        world_frame_, gimbal_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      const auto & o = tf.transform.rotation;
      R = tf2::Matrix3x3(tf2::Quaternion(o.x, o.y, o.z, o.w));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "查不到 TF %s ← %s（%s）—— 退化为直通模式（相对角原样收发）",
        world_frame_.c_str(), gimbal_base_frame_.c_str(), e.what());
      return false;
    }
  }

  // 相对关节角 → 云台绝对姿态角（发给板端）
  std::array<double, AXES> to_absolute(const std::array<double, AXES> & rel)
  {
    tf2::Matrix3x3 Rbw;
    if (!base_rot(Rbw)) return rel;
    auto abs_q = head_angles(Rbw * head_rot(rel[0], rel[1], rel[2]));
    abs_q[0] += yaw_datum_rad_;
    return abs_q;
  }

  // 云台绝对姿态角 → 相对关节角（回填 joint_states_raw）
  std::array<double, AXES> to_relative(const std::array<double, AXES> & abs_in)
  {
    tf2::Matrix3x3 Rbw;
    if (!base_rot(Rbw)) return abs_in;
    return head_angles(
      Rbw.transpose() * head_rot(abs_in[0] - yaw_datum_rad_, abs_in[1], abs_in[2]));
  }

  // ── 指令方向：forward_cmd（JointState）→ GimbalCommand(POSITION) ─────────────
  // 转发插件只在指令变化超阈值时才发，故本回调天然是事件流，不需再节流。
  // 消息里缺某一轴（或该轴为 NaN）时，用最近一次的目标值补齐；还没有目标值时
  // 退回该轴的实测回读，避免把 0 当成"回中"下发。
  void on_forward_cmd(JointState::ConstSharedPtr msg)
  {
    std::array<double, AXES> tgt = last_target_;
    bool any = false;

    for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
      for (size_t a = 0; a < AXES; ++a) {
        if (msg->name[k] != joint_names_[a]) continue;
        const double p = msg->position[k];
        if (std::isfinite(p)) { tgt[a] = p; any = true; }
      }
    }
    if (!any) return;

    for (size_t a = 0; a < AXES; ++a) {
      if (!std::isfinite(tgt[a])) tgt[a] = std::isfinite(cur_[a]) ? cur_[a] : 0.0;
    }

    // 相对关节角 → 绝对姿态角（板端位置指令走 GCU 绝对角）
    const auto snd = to_absolute(tgt);

    GimbalCommand cmd;
    cmd.mode            = GimbalCommand::POSITION;
    cmd.pan             = static_cast<float>(snd[0]);
    cmd.roll            = static_cast<float>(snd[1]);
    cmd.tilt            = static_cast<float>(snd[2]);
    cmd.max_angular_vel = static_cast<float>(max_angular_vel_);
    cmd_pub_->publish(cmd);

    last_target_ = tgt;
    RCLCPP_DEBUG(get_logger(),
      "关节角(相对) [%.4f %.4f %.4f] → GimbalCommand(绝对) pan=%.4f roll=%.4f tilt=%.4f",
      tgt[0], tgt[1], tgt[2], snd[0], snd[1], snd[2]);
  }

  // ── 状态方向：GimbalStatus → joint_states_raw（JointState）────────────────────
  // 直接按云台的回报节奏转发（不另设定时器），保持"真实回读"语义，
  // 转发插件的 read() 只取最近一帧。
  void on_status(GimbalStatus::ConstSharedPtr msg)
  {
    // 绝对姿态角 → 相对关节角（URDF 语义）
    const std::array<double, AXES> rel =
      to_relative({msg->pan, msg->roll, msg->tilt});
    cur_ = rel;

    JointState js;
    // 云台板端已盖过时间戳；这里沿用它，便于诊断链路延迟。
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      js.header.stamp = now();
    } else {
      js.header.stamp = msg->header.stamp;
    }
    js.name     = joint_names_;
    js.position = {rel[0], rel[1], rel[2]};
    // 角速度：云台自身的角速度回报是惯性系量，基座静止时等于关节角速度；
    // 基座运动时严格换算需要基座角速度（当前 TF 不提供），故原样透传并接受该近似
    // （速度只用于 joint_states 显示与到位判据，不进控制环）。
    js.velocity = {msg->pan_vel, msg->roll_vel, msg->tilt_vel};
    js_pub_->publish(js);

    if (msg->has_hw_fault) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "云台上报硬件故障 hw_err=%u（需返厂检修）", msg->hw_err);
    }
    last_status_ = now();
    got_status_  = true;
  }

  // 看门狗只告警、不改行为：回读断流时转发插件会退化为指令回显，
  // 这里把"云台话题没连上"这件事显式说出来，避免静默假动。
  void check_status_timeout()
  {
    if (!got_status_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "尚未收到任何 GimbalStatus —— 云台板端节点是否在运行 / 同网段?");
      return;
    }
    if ((now() - last_status_).seconds() > status_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GimbalStatus 断流 > %.1fs，J4-6 状态已停止更新", status_timeout_sec_);
    }
  }

  std::vector<std::string> joint_names_;
  double max_angular_vel_{0.0};
  double status_timeout_sec_{1.0};

  bool        absolute_mode_{true};
  std::string world_frame_;
  std::string gimbal_base_frame_;
  double      yaw_datum_rad_{0.0};
  double      tf_timeout_sec_{0.1};
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::array<double, AXES> last_target_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::array<double, AXES> cur_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  bool got_status_{false};
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<GimbalCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<JointState>::SharedPtr    js_pub_;
  rclcpp::Subscription<JointState>::SharedPtr forward_cmd_sub_;
  rclcpp::Subscription<GimbalStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

public:
  void start_watchdog()
  {
    watchdog_ = create_wall_timer(std::chrono::seconds(1),
      std::bind(&GimbalV2Bridge::check_status_timeout, this));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalV2Bridge>();
  node->start_watchdog();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
