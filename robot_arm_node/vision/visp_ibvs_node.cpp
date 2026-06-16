/**
 * @file visp_ibvs_node.cpp
 * @brief 基于 ViSP + Pinocchio 的图像视觉伺服（IBVS）C++ 节点
 *
 * 将 IBVS 图像居中 + 深度保持的相机速度指令，通过 Pinocchio 加权 Jacobian
 * 分配到六个关节，实现"云台快速响应、机械臂辅助大范围偏差"的宏-微协同控制：
 *   - vpFeaturePoint + vpAdaptiveGain  完整 6DOF ViSP 控制律
 *   - 独立 Vz 深度校正项              log(Z/Z*) 误差驱动
 *   - 加权伪逆 W⁻¹Jᵀ(JW⁻¹Jᵀ + λI)⁻¹  机械臂重权(100)，云台轻权(3)
 *   - 动态权重                         云台近限位自动涨价，任务平滑转给机械臂
 *   - 零空间云台回中                   N·(-K_null·Δq_gimbal) 持续软驱动
 *
 * 话题接口：
 *   订阅  /red_detector/feature   (geometry_msgs/PointStamped) x_norm, y_norm, depth
 *   订阅  /joint_states            (sensor_msgs/JointState)
 *   发布  /arm_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
 *
 * 参数（desired_* 支持 ros2 param set 运行时修改，其余改后重编译）：
 *   robot_description  [string]  URDF XML，由 launch 文件传入
 *   desired_depth      [double]  期望保持距离（m），默认见 DEFAULT_DESIRED_DEPTH
 *   desired_x          [double]  期望图像位置 x（归一化），默认 0
 *   desired_y          [double]  期望图像位置 y（归一化），默认 0
 *
 * @version 2.0
 * @date 2026-06-16
 * @copyright Copyright (c) 2026 eMeet
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpColVector.h>
using namespace visp;

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
namespace pin = pinocchio;

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>

// ── 可调参数 ──────────────────────────────────────────────────────────────────
// desired_* 三项同时作为 ROS2 参数（ros2 param set 运行时修改）
static constexpr double DEFAULT_DESIRED_DEPTH = 0.3;    // 期望距离（m）
static constexpr double DEFAULT_DESIRED_X     = 0.0;    // 期望图像 x（0=中心）
static constexpr double DEFAULT_DESIRED_Y     = 0.0;    // 期望图像 y（0=中心）
// 深度独立校正增益：Vz_cam = DEPTH_GAIN × log(Z/Z*)
static constexpr double DEPTH_GAIN            = 1.0;
// ViSP 自适应增益曲线：误差→0 时 lambda=4，误差大时 lambda=0.4
static constexpr double LAMBDA_0              = 4.0;
static constexpr double LAMBDA_INF            = 0.4;
static constexpr double LAMBDA_SLOPE          = 30.0;
// 加权 Jacobian：机械臂(J1-J3)重权，云台(J4-J6)轻权
static constexpr double W_ARM                 = 100.0;
static constexpr double W_GIMBAL              = 3.0;
// 云台关节近限位时的动态涨价斜率（使用率超过 ~70% 后快速上升）
static constexpr double W_DYN_K               = 15.0;
// 阻尼最小二乘正则化（避免奇异点爆速）
static constexpr double DAMPING_SQ            = 1e-4;
// 零空间云台回中增益（仅作用于 J4-J6）
static constexpr double K_NULL_GIMBAL         = 0.3;
// 单关节速度上限（rad/s）
static constexpr double MAX_JOINT_VEL         = 0.8;
// 控制周期（s）— 与 create_wall_timer 一致
static constexpr double CTRL_DT               = 0.02;   // 50 Hz
// 特征超时：超过此时间无新特征则停止运动
static constexpr double FEATURE_TIMEOUT       = 0.5;    // s
// 收敛阈值
static constexpr double IMG_STOP_TH           = 0.005;
static constexpr double DEPTH_STOP_TH         = 0.02;
// Pinocchio 使用的相机帧名
static constexpr const char* CAM_FRAME        = "camera_optical_frame";
// ─────────────────────────────────────────────────────────────────────────────

// 控制器关节顺序（与 arm_controller/controllers.yaml 一致）
static const std::array<std::string, 6> JOINT_NAMES = {
    "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
};

// 云台关节中立位目标（J4=0, J5=0, J6 范围 [-1.5, 0.5] 中点 = -0.5）
static const std::array<double, 6> Q_NULL_TARGET = {0, 0, 0, 0.0, 0.0, 0.0};

class VispIbvsNode : public rclcpp::Node
{
public:
    VispIbvsNode()
    : Node("visp_ibvs_node"),
      q_curr_(Eigen::VectorXd::Zero(6)),
      pin_q_(Eigen::VectorXd::Zero(6))
    {
        // ── ROS2 参数 ────────────────────────────────────────────────────────
        this->declare_parameter("robot_description", std::string(""));
        this->declare_parameter("desired_depth",     DEFAULT_DESIRED_DEPTH);
        this->declare_parameter("desired_x",         DEFAULT_DESIRED_X);
        this->declare_parameter("desired_y",         DEFAULT_DESIRED_Y);
        this->declare_parameter("paused",            false);   // true 时挂起控制循环，供手动位置指令使用

        desired_depth_ = this->get_parameter("desired_depth").as_double();
        desired_x_     = this->get_parameter("desired_x").as_double();
        desired_y_     = this->get_parameter("desired_y").as_double();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if      (p.get_name() == "desired_depth") desired_depth_ = p.as_double();
                    else if (p.get_name() == "desired_x")     desired_x_     = p.as_double();
                    else if (p.get_name() == "desired_y")     desired_y_     = p.as_double();
                    else if (p.get_name() == "paused")        paused_        = p.as_bool();
                }
                updateDesired();
                RCLCPP_INFO(get_logger(), "desired updated: x=%.3f y=%.3f depth=%.3fm",
                    desired_x_, desired_y_, desired_depth_);
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });

        // ── ViSP 图像任务 ────────────────────────────────────────────────────
        task_.setServo(vpServo::EYEINHAND_CAMERA);
        task_.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
        vpAdaptiveGain lam;
        lam.initStandard(LAMBDA_0, LAMBDA_INF, LAMBDA_SLOPE);
        task_.setLambda(lam);
        task_.addFeature(p_curr_, p_desired_);
        updateDesired();

        // ── Pinocchio 运动学模型 ──────────────────────────────────────────────
        initPinocchio();

        // ── ROS2 话题 ────────────────────────────────────────────────────────
        jsub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { onJointState(m); });

        feat_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/red_detector/feature", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr m) { onFeature(m); });

        traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);

        ctrl_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(CTRL_DT * 1000)),
            [this]() { controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "visp_ibvs_node ready | %d DOF | desired=(%.2f,%.2f) depth=%.2fm",
            pin_model_.nv, desired_x_, desired_y_, desired_depth_);
    }

    ~VispIbvsNode() override { task_.kill(); }

private:
    // ─────────────────────────────────────────────────────────────────────────
    void initPinocchio()
    {
        const std::string urdf = this->get_parameter("robot_description").as_string();
        if (urdf.empty()) {
            RCLCPP_FATAL(get_logger(), "robot_description parameter is empty — pass it from launch");
            throw std::runtime_error("robot_description empty");
        }

        pin::urdf::buildModelFromXML(urdf, pin_model_);
        pin_data_ = pin::Data(pin_model_);

        if (pin_model_.nv != 6) {
            RCLCPP_WARN(get_logger(), "Expected 6 DOF, got %d — check URDF", pin_model_.nv);
        }

        // 相机帧 ID
        if (pin_model_.existFrame(CAM_FRAME)) {
            cam_frame_id_ = pin_model_.getFrameId(CAM_FRAME);
        } else {
            RCLCPP_WARN(get_logger(), "Frame '%s' not found, fallback to tool0", CAM_FRAME);
            cam_frame_id_ = pin_model_.getFrameId("tool0");
        }

        // 关节名 → Pinocchio 速度向量下标
        for (pin::JointIndex ji = 1; ji < (pin::JointIndex)pin_model_.njoints; ++ji) {
            joint_to_vidx_[pin_model_.names[ji]] = (int)pin_model_.idx_vs[ji];
        }

        RCLCPP_INFO(get_logger(), "Pinocchio model loaded: %d joints, cam_frame_id=%zu",
            pin_model_.nv, cam_frame_id_);
    }

    // ─────────────────────────────────────────────────────────────────────────
    void onFeature(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!std::isfinite(msg->point.z) || msg->point.z <= 0.0) return;
        feat_x_ = msg->point.x;
        feat_y_ = msg->point.y;
        feat_z_ = msg->point.z;
        last_feat_time_ = now();
        has_feat_ = true;
    }

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            auto it = joint_to_vidx_.find(msg->name[i]);
            if (it != joint_to_vidx_.end()) {
                pin_q_[it->second]  = msg->position[i];
                q_curr_[it->second] = msg->position[i];
            }
        }
        q_valid_ = true;
    }

    // ── 50 Hz 主控制循环 ──────────────────────────────────────────────────────
    void controlLoop()
    {
        if (paused_) return;   // 手动位置指令期间挂起，避免覆盖轨迹
        if (!q_valid_) return;
        if (!has_feat_ || (now() - last_feat_time_).seconds() > FEATURE_TIMEOUT) {
            publishStop();
            return;
        }

        const double depth_err = std::log(feat_z_ / desired_depth_);
        const double img_err   = std::hypot(feat_x_ - desired_x_, feat_y_ - desired_y_);

        if (img_err < IMG_STOP_TH && std::abs(depth_err) < DEPTH_STOP_TH) {
            publishStop();
            return;
        }

        // ── ViSP 相机速度（图像居中，相机坐标系）───────────────────────────
        p_curr_.buildFrom(feat_x_, feat_y_, feat_z_);
        vpColVector vc_visp = task_.computeControlLaw();
        vc_visp[2] += DEPTH_GAIN * depth_err;   // 独立深度校正

        Eigen::Matrix<double, 6, 1> v_c;
        for (int i = 0; i < 6; ++i) v_c[i] = vc_visp[i];

        // ── Pinocchio Jacobian（相机系，LOCAL 参考系）──────────────────────
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, pin_model_.nv);
        J.setZero();
        pin::computeFrameJacobian(pin_model_, pin_data_, pin_q_,
                                   cam_frame_id_, pin::LOCAL, J);

        // ── 动态加权矩阵 ───────────────────────────────────────────────────
        const Eigen::Matrix<double, 6, 6> W     = computeWeightMatrix();
        const Eigen::Matrix<double, 6, 6> W_inv = W.inverse();

        // ── 阻尼加权伪逆 J_wpinv = W⁻¹Jᵀ(JW⁻¹Jᵀ + λI)⁻¹ ─────────────────
        Eigen::Matrix<double, 6, 6> JWJt = J * W_inv * J.transpose();
        JWJt.diagonal().array() += DAMPING_SQ;
        const Eigen::Matrix<double, 6, 6> J_wpinv = W_inv * J.transpose() * JWJt.inverse();

        // ── 主任务关节速度 ─────────────────────────────────────────────────
        Eigen::Matrix<double, 6, 1> q_dot = J_wpinv * v_c;

        // ── 零空间云台回中（J4-J6，下标 3-5）─────────────────────────────
        const Eigen::Matrix<double, 6, 6> N =
            Eigen::Matrix<double, 6, 6>::Identity() - J_wpinv * J;
        Eigen::Matrix<double, 6, 1> null_grad = Eigen::Matrix<double, 6, 1>::Zero();
        for (int i = 3; i < 6; ++i) {
            null_grad[i] = K_NULL_GIMBAL * (q_curr_[i] - Q_NULL_TARGET[i]);
        }
        q_dot -= N * null_grad;

        // ── 单关节速度限幅 ─────────────────────────────────────────────────
        for (int i = 0; i < 6; ++i) {
            q_dot[i] = std::clamp(q_dot[i], -MAX_JOINT_VEL, MAX_JOINT_VEL);
        }

        publishTrajectory(q_dot);
    }


    // ── 动态权重矩阵 ──────────────────────────────────────────────────────────
    Eigen::Matrix<double, 6, 6> computeWeightMatrix() const
    {
        Eigen::Matrix<double, 6, 6> W = Eigen::Matrix<double, 6, 6>::Zero();
        const auto& lb = pin_model_.lowerPositionLimit;
        const auto& ub = pin_model_.upperPositionLimit;

        for (int i = 0; i < 6; ++i) {
            double w = (i < 3) ? W_ARM : W_GIMBAL;
            if (i >= 3) {
                const double mid   = (lb[i] + ub[i]) * 0.5;
                const double range = (ub[i] - lb[i]) * 0.5;
                const double ratio = (range > 1e-6) ?
                    std::abs(q_curr_[i] - mid) / range : 0.0;
                w *= (1.0 + W_DYN_K * ratio * ratio);
            }
            W(i, i) = w;
        }
        return W;
    }

    // ── 期望特征更新 ──────────────────────────────────────────────────────────
    void updateDesired()
    {
        p_desired_.buildFrom(desired_x_, desired_y_, desired_depth_);
    }

    // ── 发布关节轨迹（前向欧拉积分一步）──────────────────────────────────────
    void publishTrajectory(const Eigen::Matrix<double, 6, 1>& q_dot)
    {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = now();
        msg.joint_names.assign(JOINT_NAMES.begin(), JOINT_NAMES.end());

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.time_from_start = rclcpp::Duration::from_seconds(CTRL_DT);
        pt.positions.resize(6);
        pt.velocities.resize(6);

        const auto& lb = pin_model_.lowerPositionLimit;
        const auto& ub = pin_model_.upperPositionLimit;

        for (int i = 0; i < 6; ++i) {
            pt.positions[i]  = std::clamp(q_curr_[i] + q_dot[i] * CTRL_DT, lb[i], ub[i]);
            pt.velocities[i] = q_dot[i];
        }

        msg.points.push_back(pt);
        traj_pub_->publish(msg);
    }

    // 原地保持当前位置，速度为零
    void publishStop()
    {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = now();
        msg.joint_names.assign(JOINT_NAMES.begin(), JOINT_NAMES.end());

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.time_from_start = rclcpp::Duration::from_seconds(CTRL_DT);
        pt.positions.resize(6);
        pt.velocities.assign(6, 0.0);
        for (int i = 0; i < 6; ++i) pt.positions[i] = q_curr_[i];

        msg.points.push_back(pt);
        traj_pub_->publish(msg);
    }

    // ── ViSP ──────────────────────────────────────────────────────────────────
    vpServo        task_;
    vpFeaturePoint p_curr_, p_desired_;
    double desired_depth_{DEFAULT_DESIRED_DEPTH};
    double desired_x_{DEFAULT_DESIRED_X};
    double desired_y_{DEFAULT_DESIRED_Y};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // ── Pinocchio ─────────────────────────────────────────────────────────────
    pin::Model      pin_model_;
    pin::Data       pin_data_;
    pin::FrameIndex cam_frame_id_{0};
    std::unordered_map<std::string, int> joint_to_vidx_;

    // ── 状态缓存 ──────────────────────────────────────────────────────────────
    Eigen::VectorXd q_curr_;        // 关节位置（Pinocchio 速度顺序 = 控制器顺序）
    Eigen::VectorXd pin_q_;         // Pinocchio 配置向量（用于 FK/Jacobian）
    bool paused_{false};
    bool q_valid_{false};

    double feat_x_{0.0}, feat_y_{0.0}, feat_z_{0.5};
    bool has_feat_{false};
    rclcpp::Time last_feat_time_;

    // ── ROS2 接口 ─────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     jsub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr feat_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VispIbvsNode>());
    rclcpp::shutdown();
    return 0;
}
