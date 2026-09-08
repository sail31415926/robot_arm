/**
 * @file status_aggregator.cpp
 * @brief StatusAggregator 实现 —— 订阅 /joint_states + TF2 → ArmStatus
 *
 * on_joint_state 写关节缓存 + 追加位姿历史；pose() 每次 TF
 * lookup（失败返回零位姿）； twist()
 * 用位姿历史首末差分估速；build_status_message 汇总位姿/速度/flag。互斥保护，
 * TransformListener 传裸指针避 bad_weak_ptr（详见构造函数注释）。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/state/status_aggregator.hpp"

#include <tf2/time.h>

#include <algorithm>
#include <cmath>

#include "robot_arm_node/motion/constants.hpp"
#include "robot_arm_node/motion/geometry.hpp"

namespace robot_arm_node::state {

using motion::BASE_FRAME;
using motion::EEF_LINK;
using motion::JOINT_NAMES;
using motion::JOINT_STATE_TOPIC;

/**
 * @brief 构造状态聚合器：订阅关节状态与控制模式，建立 TF2 监听。
 *
 * 关节缓存先按 JOINT_NAMES 全部填 0 —— 后续回调只更新「已存在的键」，
 * 这样 /joint_states 里混进的其他关节不会污染本臂的缓存。
 *
 * 控制模式话题用 transient_local：mode_manager_node 是 latched 广播，
 * 晚订阅也能立刻拿到当前模式，不必等下一次切换。
 *
 * TransformListener 传 &node_ 裸指针而不是 shared_from_this()：本对象常在
 * node 自身构造期内被创建，那时 weak_ptr 还没就绪，shared_from_this() 会抛
 * bad_weak_ptr。listener 只是瞬时借 node 的接口订阅 /tf，不持有所有权。
 *
 * @param node 宿主节点。
 */
StatusAggregator::StatusAggregator(rclcpp::Node& node)
    : node_(node), logger_(node.get_logger()) {
    // ──
    // 关节状态缓存初始化（全零）────────────────────────────────────────────────
    for (const auto& name : JOINT_NAMES) {
        joint_positions_[name] = 0.0;
        joint_velocities_[name] = 0.0;
    }

    joint_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATE_TOPIC, 10, [this](const sensor_msgs::msg::JointState& msg) {
            this->on_joint_state(msg);
        });

    // 当前控制模式：mode_manager_node latched 广播，晚订阅也能立刻拿到当前值
    mode_sub_ = node_.create_subscription<ControlMode>(
        "/robot_arm/control_mode", rclcpp::QoS(1).transient_local().reliable(),
        [this](const ControlMode& msg) {
            std::lock_guard<std::mutex> lk(state_mtx_);
            active_control_mode_ = msg.mode;
        });

    // ── TF2
    // ──────────────────────────────────────────────────────────────────────
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
    // 传裸指针而非 shared_from_this()：后者在「node
    // 自身构造期内创建本对象」时会抛 bad_weak_ptr。TransformListener 只是瞬时用
    // node 取接口订阅 /tf，不持有所有权。
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, &node_);

    RCLCPP_INFO(logger_, "StatusAggregator 就绪");
}

/**
 * @brief 处理关节状态消息并记录末端位姿历史。
 * @param msg 收到的关节状态消息。
 */
// ── 关节状态订阅
// ────────────────────────────────────────────────────────────────
void StatusAggregator::on_joint_state(const sensor_msgs::msg::JointState& msg) {
    const double now = node_.get_clock()->now().seconds();

    {
        std::lock_guard<std::mutex> lk(joint_mtx_);
        const size_t m =
            std::min(msg.name.size(),
                     std::min(msg.position.size(), msg.velocity.size()));
        for (size_t i = 0; i < m; ++i) {
            auto it_p = joint_positions_.find(msg.name[i]);
            if (it_p != joint_positions_.end()) {
                it_p->second = msg.position[i];
                joint_velocities_[msg.name[i]] = msg.velocity[i];
            }
        }
    }

    // 记录位姿历史（用于速度估算）
    if (auto p = lookup_pose()) {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        if (pose_history_.size() >= VELOCITY_WINDOW_SIZE)
            pose_history_.pop_front();
        pose_history_.emplace_back(now, std::move(*p));
    }
}

/**
 * @brief 获取当前末端位姿，查询失败时返回零位姿。
 * @return 当前末端位姿或默认构造的零位姿。
 */
// ── 末端位姿查询
// ────────────────────────────────────────────────────────────────
ArmPose StatusAggregator::pose() const {
    if (auto p = lookup_pose()) return *p;
    return ArmPose{};  // 零位姿（字段默认 0）
}

/**
 * @brief 通过 TF2 查询末端相对于基座的位姿。
 * @return 查询成功时返回位姿，否则返回空值。
 */
std::optional<ArmPose> StatusAggregator::lookup_pose() const {
    try {
        const auto t = tf_buffer_->lookupTransform(BASE_FRAME, EEF_LINK,
                                                   tf2::TimePointZero);
        return transform_to_pose(t.transform);
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
}

/**
 * @brief 将 TF 变换转换为 ArmPose。
 * @param tf 待转换的几何变换。
 * @return 转换后的末端位姿，角度单位为度。
 */
ArmPose StatusAggregator::transform_to_pose(
    const geometry_msgs::msg::Transform& tf) {
    ArmPose p;
    p.x = static_cast<float>(tf.translation.x);
    p.y = static_cast<float>(tf.translation.y);
    p.z = static_cast<float>(tf.translation.z);
    const auto rpy = motion::quat_to_rpy(tf.rotation.x, tf.rotation.y,
                                         tf.rotation.z, tf.rotation.w);
    p.roll = static_cast<float>(rpy[0] * 180.0 / M_PI);
    p.pitch = static_cast<float>(rpy[1] * 180.0 / M_PI);
    p.yaw = static_cast<float>(rpy[2] * 180.0 / M_PI);
    return p;
}

/**
 * @brief 根据位姿历史首末样本估算末端速度。
 * @return 线速度和角速度；历史不足或时间间隔无效时返回零值。
 */
// ── 末端速度估算
// ────────────────────────────────────────────────────────────────
ArmTwist StatusAggregator::twist() const {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    if (pose_history_.size() < 2) return ArmTwist{};

    const auto& [t0, p0] = pose_history_.front();
    const auto& [t1, p1] = pose_history_.back();
    const double dt = t1 - t0;
    if (dt < 1e-6) return ArmTwist{};

    ArmTwist tw;
    tw.vx = static_cast<float>((p1.x - p0.x) / dt);
    tw.vy = static_cast<float>((p1.y - p0.y) / dt);
    tw.vz = static_cast<float>((p1.z - p0.z) / dt);
    tw.wroll = static_cast<float>((p1.roll - p0.roll) / dt);
    tw.wpitch = static_cast<float>((p1.pitch - p0.pitch) / dt);
    tw.wyaw = static_cast<float>((p1.yaw - p0.yaw) / dt);
    return tw;
}

/**
 * @brief 获取全部缓存的关节位置。
 * @return 关节名称到位置的映射副本。
 */
// ── 关节状态查询
// ────────────────────────────────────────────────────────────────
std::map<std::string, double> StatusAggregator::joint_positions() const {
    std::lock_guard<std::mutex> lk(joint_mtx_);
    return joint_positions_;
}

/**
 * @brief 获取全部缓存的关节速度。
 * @return 关节名称到速度的映射副本。
 */
std::map<std::string, double> StatusAggregator::joint_velocities() const {
    std::lock_guard<std::mutex> lk(joint_mtx_);
    return joint_velocities_;
}

/**
 * @brief 按指定名称获取关节位置。
 * @param names 要查询的关节名称列表。
 * @return 与输入名称顺序对应的位置列表，未知关节返回零。
 */
std::vector<double> StatusAggregator::joint_position_list(
    const std::vector<std::string>& names) const {
    std::lock_guard<std::mutex> lk(joint_mtx_);
    std::vector<double> out;
    out.reserve(names.size());
    for (const auto& n : names) {
        auto it = joint_positions_.find(n);
        out.push_back(it != joint_positions_.end() ? it->second : 0.0);
    }
    return out;
}

/**
 * @brief 按指定名称获取关节速度。
 * @param names 要查询的关节名称列表。
 * @return 与输入名称顺序对应的速度列表（rad/s），未知关节返回零。
 */
std::vector<double> StatusAggregator::joint_velocity_list(
    const std::vector<std::string>& names) const {
    std::lock_guard<std::mutex> lk(joint_mtx_);
    std::vector<double> out;
    out.reserve(names.size());
    for (const auto& n : names) {
        auto it = joint_velocities_.find(n);
        out.push_back(it != joint_velocities_.end() ? it->second : 0.0);
    }
    return out;
}

/**
 * @brief 检查指定关节是否均存在于位置缓存中。
 * @param names 要检查的关节名称列表。
 * @return 名称列表非空且所有关节均存在时返回 true。
 */
bool StatusAggregator::has_joint_positions(
    const std::vector<std::string>& names) const {
    std::lock_guard<std::mutex> lk(joint_mtx_);
    for (const auto& n : names) {
        if (joint_positions_.find(n) == joint_positions_.end()) return false;
    }
    return !names.empty();
}

/**
 * @brief 获取当前控制模式。
 * @return 当前控制模式值。
 */
uint8_t StatusAggregator::active_control_mode() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return active_control_mode_;
}

/**
 * @brief 获取机械臂运动状态。
 * @return 正在运动时返回 true。
 */
// ── 运动 / 运镜 / 跟随 flag
// ──────────────────────────────────────────────────────
bool StatusAggregator::is_moving() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return is_moving_;
}

/**
 * @brief 设置机械臂运动状态。
 * @param moving 新的运动状态。
 */
void StatusAggregator::set_moving(bool moving) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    is_moving_ = moving;
}

/** @brief 获取是否处于动作起始位姿。 @return 起始位姿状态。 */
bool StatusAggregator::at_pose_start() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return at_pose_start_;
}

/** @brief 设置动作起始位姿状态。 @param at_start 新状态。 */
void StatusAggregator::set_at_pose_start(bool at_start) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    at_pose_start_ = at_start;
}

/** @brief 获取相机就绪状态。 @return 相机就绪时返回 true。 */
bool StatusAggregator::camera_ready() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return camera_ready_;
}

/** @brief 设置相机就绪状态。 @param ready 新的就绪状态。 */
void StatusAggregator::set_camera_ready(bool ready) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    camera_ready_ = ready;
}

/**
 * @brief 更新视觉跟随状态及误差。
 * @param tracking 是否正在跟随。
 * @param img_err 图像误差。
 * @param depth_err_m 深度误差，单位为米。
 */
void StatusAggregator::set_tracking(bool tracking, double img_err,
                                    double depth_err_m) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    is_tracking_ = tracking;
    tracking_img_err_ = img_err;
    tracking_depth_err_m_ = depth_err_m;
}

/**
 * @brief 更新当前命令的执行状态。
 * @param command_id 正在执行的命令标识。
 * @param result 命令执行结果。
 */
// ── 命令执行状态
// ────────────────────────────────────────────────────────────────
void StatusAggregator::set_command_state(uint32_t command_id, uint8_t result) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    executing_command_id_ = command_id;
    command_result_ = result;
}

/** @brief 设置当前位姿状态。 @param state 新的位姿状态。 */
void StatusAggregator::set_pose_state(uint8_t state) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_pose_state_ = state;
}

/** @brief 设置错误码。 @param error_code 新的错误码。 */
void StatusAggregator::set_error(uint8_t error_code) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    error_code_ = error_code;
}

/** @brief 获取当前错误码。 @return 当前错误码。 */
uint8_t StatusAggregator::get_error_code() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return error_code_;
}

/** @brief 清除当前错误并恢复为无错误状态。 */
void StatusAggregator::clear_error() {
    std::lock_guard<std::mutex> lk(state_mtx_);
    error_code_ = ArmStatus::ERR_NONE;
}

/**
 * @brief 汇总当前状态并构建 ArmStatus 消息。
 * @return 包含位姿、速度及各状态标志的状态消息。
 */
// ── 构建 ArmStatus 消息
// ─────────────────────────────────────────────────────────
ArmStatus StatusAggregator::build_status_message() const {
    ArmStatus msg;
    msg.header.frame_id = BASE_FRAME;

    // pose()/twist()/is_moving() 各自取锁，此处不能再持
    // state_mtx_（否则与它们死锁）
    msg.arm_pose = pose();
    msg.arm_twist = twist();

    std::lock_guard<std::mutex> lk(state_mtx_);
    msg.current_pose_state = current_pose_state_;
    msg.error_code = error_code_;
    msg.active_control_mode = active_control_mode_;
    msg.executing_command_id = executing_command_id_;
    msg.command_result = command_result_;
    msg.is_moving = is_moving_;
    msg.arm_at_target = !is_moving_;  // TODO: 改为基于目标位姿的比较
    msg.arm_at_pose_start = at_pose_start_;
    msg.camera_ready = camera_ready_;
    msg.is_tracking = is_tracking_;
    msg.tracking_img_err = static_cast<float>(tracking_img_err_);
    msg.tracking_depth_err_m = static_cast<float>(tracking_depth_err_m_);
    return msg;
}

}  // namespace robot_arm_node::state
