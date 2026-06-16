/**
 * @file visp_ibvs_node.cpp
 * @brief 基于 ViSP + Pinocchio 的图像视觉伺服（IBVS）C++ 节点
 *
 * 将 IBVS 图像居中 + 深度保持的相机速度指令，通过 Pinocchio Jacobian 分级分配：
 *   任务分级（Task Priority）控制，云台绝对优先：
 *   - 第一优先级（J4-J6 云台）：加权伪逆直接求解完整图像误差，小阻尼、响应快
 *   - 第二优先级（J1-J3 机械臂）：大阻尼伪逆仅补偿云台覆盖不了的残差，平时近零
 *   - 云台动态权重                云台关节近限位自动涨价，抑制极端姿态
 *   - 零空间云台回中              N·(-K_null·Δq_gimbal) 保持云台居中
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
static constexpr double DEPTH_GAIN            = 0.5;
// ViSP 自适应增益曲线：误差→0 时 lambda=4，误差大时 lambda=0.4
static constexpr double LAMBDA_0              = 4.0;
static constexpr double LAMBDA_INF            = 1.5;
static constexpr double LAMBDA_SLOPE          = 30.0;
// 云台（J4-J6）基础权重（动态乘以涨价因子）
static constexpr double W_GIMBAL              = 1.0;
// 云台关节近限位时的动态涨价斜率（使用率超过 ~70% 后快速上升）
static constexpr double W_DYN_K               = 15.0;
// 云台阻尼（小 → 响应快，第一优先级）
static constexpr double DAMPING_SQ            = 1e-4;
// 机械臂阻尼（大 → 仅在云台残差足够大时才显著运动，第二优先级）
static constexpr double DAMPING_ARM_SQ        = 0.1;
// 云台极小软衰减增益（防关节极端漂移，不影响跟踪）
static constexpr double K_NULL_GIMBAL         = 0.01;
// 球坐标约束增益（m/s per m error）：俯仰角/方位角约束的位置误差增益
static constexpr double K_SPHERE              = 0.8;
// 单关节速度上限（rad/s）
static constexpr double MAX_JOINT_VEL         = 1.5;
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
        this->declare_parameter("paused",            false);
        // 球坐标约束参数（GUI 可调，ros2 param set 运行时修改）
        this->declare_parameter("desired_elevation",   0.5);   // rad ≈ 28.6°
        this->declare_parameter("desired_azimuth",     0.0);   // rad
        this->declare_parameter("constrain_elevation", false);
        this->declare_parameter("constrain_azimuth",   false);

        desired_depth_ = this->get_parameter("desired_depth").as_double();
        desired_x_     = this->get_parameter("desired_x").as_double();
        desired_y_     = this->get_parameter("desired_y").as_double();
        desired_elevation_   = this->get_parameter("desired_elevation").as_double();
        desired_azimuth_     = this->get_parameter("desired_azimuth").as_double();
        constrain_elevation_ = this->get_parameter("constrain_elevation").as_bool();
        constrain_azimuth_   = this->get_parameter("constrain_azimuth").as_bool();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if      (p.get_name() == "desired_depth")        desired_depth_       = p.as_double();
                    else if (p.get_name() == "desired_x")            desired_x_           = p.as_double();
                    else if (p.get_name() == "desired_y")            desired_y_           = p.as_double();
                    else if (p.get_name() == "paused")               paused_              = p.as_bool();
                    else if (p.get_name() == "desired_elevation")    desired_elevation_   = p.as_double();
                    else if (p.get_name() == "desired_azimuth")      desired_azimuth_     = p.as_double();
                    else if (p.get_name() == "constrain_elevation")  constrain_elevation_ = p.as_bool();
                    else if (p.get_name() == "constrain_azimuth")    constrain_azimuth_   = p.as_bool();
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
        // ── 分离 Jacobian ──────────────────────────────────────────────────
        const Eigen::Matrix<double, 6, 3> J_g = J.rightCols(3);  // 云台 J4-J6
        const Eigen::Matrix<double, 6, 3> J_a = J.leftCols(3);   // 机械臂 J1-J3

        // ── 第一优先级：云台处理图像误差（v_c 只含图像跟踪速度）───────────
        const Eigen::Matrix<double, 3, 3> W_g     = computeGimbalWeight();
        const Eigen::Matrix<double, 3, 3> W_g_inv = W_g.inverse();
        Eigen::Matrix<double, 6, 6> A_g = J_g * W_g_inv * J_g.transpose();
        A_g.diagonal().array() += DAMPING_SQ;
        const Eigen::Matrix<double, 3, 6> J_g_wpinv = W_g_inv * J_g.transpose() * A_g.inverse();
        Eigen::Matrix<double, 3, 1> q_dot_g = J_g_wpinv * v_c;

        // 云台软衰减回中
        for (int i = 0; i < 3; ++i)
            q_dot_g[i] -= K_NULL_GIMBAL * (q_curr_[i + 3] - Q_NULL_TARGET[i + 3]);

        // ── 第二优先级：机械臂 = 图像残差 + 球坐标位置约束 ──────────────
        // 球坐标约束只走机械臂路径（J1-J3 平移相机位置），不经过云台
        // 约定与 arm_utils.sphere_to_cart 一致（Z-up）：
        //   φ = desired_elevation（仰角，0=水平，+π/2=正上方）
        //   θ = desired_azimuth  （方位角，θ=0 = 近侧/臂方向）
        //   theta_ref = atan2(-P.y,-P.x)，theta_world = theta_ref + θ
        const Eigen::Matrix<double, 6, 1> v_res = v_c - J_g * q_dot_g;
        Eigen::Matrix<double, 6, 1> v_arm = v_res;

        if (constrain_elevation_ || constrain_azimuth_) {
            const pin::SE3& T = pin_data_.oMf[cam_frame_id_];
            const Eigen::Vector3d P_cam3(feat_x_ * feat_z_, feat_y_ * feat_z_, feat_z_);
            const Eigen::Vector3d P_base = T.rotation() * P_cam3 + T.translation();
            const Eigen::Vector3d C      = T.translation();

            // 相机相对目标的向量（世界系）
            const Eigen::Vector3d C_rel = C - P_base;
            const double r_h = std::hypot(C_rel.x(), C_rel.y());   // 水平距离
            const double r   = C_rel.norm();                         // 空间距离

            // 当前球坐标角
            const double phi_curr   = std::atan2(C_rel.z(), std::max(r_h, 1e-6));
            const double theta_curr = std::atan2(C_rel.y(), C_rel.x());

            // 期望方位角：从目标指向相机的水平角（世界系 XY）
            const double th_world = std::atan2(-P_base.y(), -P_base.x()) + desired_azimuth_;

            Eigen::Vector3d dC_world = Eigen::Vector3d::Zero();

            if (constrain_elevation_) {
                // 仰角误差 → 沿经线切线方向，纯仰角分量，与方位/深度完全解耦
                const double d_phi = desired_elevation_ - phi_curr;
                const Eigen::Vector3d meridional(
                    -std::sin(phi_curr) * std::cos(theta_curr),
                    -std::sin(phi_curr) * std::sin(theta_curr),
                     std::cos(phi_curr));
                dC_world += std::max(r, desired_depth_) * d_phi * meridional;
            }

            if (constrain_azimuth_) {
                // 方位角误差 → 沿纬线切线方向，纯水平切向，无径向无竖向分量
                double d_theta = th_world - theta_curr;
                d_theta -= 2.0 * M_PI * std::round(d_theta / (2.0 * M_PI));  // 归一化到 [-π, π]
                const Eigen::Vector3d tangent(
                    -std::sin(theta_curr),
                     std::cos(theta_curr),
                     0.0);
                dC_world += std::max(r_h, desired_depth_) * d_theta * tangent;
            }

            const Eigen::Vector3d dC_cam = T.rotation().transpose() * dC_world;
            v_arm[0] += K_SPHERE * dC_cam[0];
            v_arm[1] += K_SPHERE * dC_cam[1];
            v_arm[2] += K_SPHERE * dC_cam[2];
        }

        Eigen::Matrix<double, 6, 6> A_a = J_a * J_a.transpose();
        A_a.diagonal().array() += DAMPING_ARM_SQ;
        const Eigen::Matrix<double, 3, 6> J_a_pinv = J_a.transpose() * A_a.inverse();
        const Eigen::Matrix<double, 3, 1> q_dot_a  = J_a_pinv * v_arm;

        // ── 合并 ──────────────────────────────────────────────────────────
        Eigen::Matrix<double, 6, 1> q_dot;
        q_dot.head(3) = q_dot_a;
        q_dot.tail(3) = q_dot_g;

        // ── 单关节速度限幅 ─────────────────────────────────────────────────
        for (int i = 0; i < 6; ++i) {
            q_dot[i] = std::clamp(q_dot[i], -MAX_JOINT_VEL, MAX_JOINT_VEL);
        }

        publishTrajectory(q_dot);
    }


    // ── 云台动态权重矩阵（3×3，J4-J6）────────────────────────────────────────
    // 云台关节越接近限位，权重越大（越贵），该关节速度自动收敛到更小
    Eigen::Matrix<double, 3, 3> computeGimbalWeight() const
    {
        Eigen::Matrix<double, 3, 3> W = Eigen::Matrix<double, 3, 3>::Zero();
        const auto& lb = pin_model_.lowerPositionLimit;
        const auto& ub = pin_model_.upperPositionLimit;

        for (int i = 0; i < 3; ++i) {
            const int ji   = i + 3;
            const double mid   = (lb[ji] + ub[ji]) * 0.5;
            const double range = (ub[ji] - lb[ji]) * 0.5;
            const double ratio = (range > 1e-6) ?
                std::abs(q_curr_[ji] - mid) / range : 0.0;
            W(i, i) = W_GIMBAL * (1.0 + W_DYN_K * ratio * ratio);
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

    // ── 球坐标约束 ────────────────────────────────────────────────────────────
    double desired_elevation_{0.5};     // rad，俯仰角（0=水平，π/2=正上方）
    double desired_azimuth_{0.0};       // rad，方位角（0=+X 方向）
    bool   constrain_elevation_{false};
    bool   constrain_azimuth_{false};

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
