/**
 * @file jacobian.cpp
 * @brief ArmJacobian 实现 —— URDF 取关节轴 + TF 取位形 → 几何线速度
 * Jacobian；DLS 伪逆
 *
 * @version 1.0
 * @date 2026-08-04
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/motion/jacobian.hpp"

#include <tf2/time.h>
#include <urdf/model.h>

#include <Eigen/SVD>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robot_arm_node::motion {

/**
 * @namespace robot_arm_node::motion
 * @brief 机械臂运动控制相关功能命名空间，包含雅可比矩阵计算、逆运动学求解等
 */

namespace {
/**
 * @brief 从 TransformStamped 消息中提取旋转四元数和平移向量
 * @details TF → (旋转, 平移)。base_frame ← link 的位姿。
 *
 * @param[in]  t  TransformStamped 消息，包含旋转与平移信息
 * @param[out] q  提取的旋转四元数（已归一化）
 * @param[out] p  提取的平移向量
 */
// TF → (旋转, 平移)。base_frame ← link 的位姿。
void split(const geometry_msgs::msg::TransformStamped& t, Eigen::Quaterniond& q,
           Eigen::Vector3d& p) {
    q = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x,
                           t.transform.rotation.y, t.transform.rotation.z);
    q.normalize();
    p = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y,
                        t.transform.translation.z);
}
}  // namespace


/**
 * @brief ArmJacobian 构造函数
 * @details 初始化机械臂雅可比计算器，订阅URDF描述话题（latched）。
 *          采用与 JointLimitsCache 相同的 QoS 策略（transient_local + reliable）
 *          以确保能接收 robot_state_publisher 的 latched 消息。
 * @param[in] node ROS 2 节点引用，用于创建订阅者和获取日志/时钟
 * @param[in] topic URDF 描述字符串的订阅话题名称（通常为 "robot_description"）
 * @note 构造后需等待 ready() 返回 true 才能使用 geometric() 方法
 * @see on_description(), ready()
 */
ArmJacobian::ArmJacobian(rclcpp::Node& node, const std::string& topic)
    : logger_(node.get_logger()), clock_(node.get_clock()) {
    // 与 JointLimitsCache 同款 QoS：匹配 robot_state_publisher 的 latched 发布
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();

    sub_ = node.create_subscription<std_msgs::msg::String>(
        topic, qos,
        [this](const std_msgs::msg::String& msg) { on_description(msg); });

    RCLCPP_INFO(logger_, "ArmJacobian: 等待 '%s'（latched）解析关节轴",
                topic.c_str());
}

/**
 * @brief URDF 描述回调函数
 * @details 解析接收到的 URDF 机械臂模型，提取所有可动关节（revolute/continuous/prismatic）
 *          的几何信息（子连杆名、关节轴方向、关节类型）并缓存。
 *          解析成功后 ready_ 标志置为 true。
 * @param[in] msg 包含 URDF XML 字符串的消息
 * @note 线程安全：使用互斥锁保护 geom_ 和 ready_ 的更新
 * @see on_description()
 */
void ArmJacobian::on_description(const std_msgs::msg::String& msg) {
    urdf::Model model;
    if (!model.initString(msg.data)) {
        RCLCPP_ERROR(logger_,
                     "ArmJacobian: URDF 解析失败，笛卡尔速度控制不可用");
        return;
    }

    std::map<std::string, JointGeom> parsed;
    for (const auto& [name, joint] : model.joints_) {
        if (!joint) continue;
        const bool revolute = joint->type == urdf::Joint::REVOLUTE ||
                              joint->type == urdf::Joint::CONTINUOUS;
        const bool prismatic = joint->type == urdf::Joint::PRISMATIC;
        if (!revolute && !prismatic)
            continue;  // fixed / floating / planar 不参与

        JointGeom g;
        g.child_link = joint->child_link_name;
        g.prismatic = prismatic;
        // URDF 的 <axis> 表达在关节坐标系 = child link 坐标系
        g.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
        if (g.axis.norm() < 1e-9)
            g.axis = Eigen::Vector3d::UnitZ();  // URDF 缺省轴
        g.axis.normalize();
        parsed[name] = g;
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        geom_ = std::move(parsed);
        ready_ = !geom_.empty();
    }
    RCLCPP_INFO(logger_, "ArmJacobian: 已解析 %zu 个可动关节的轴向",
                geom_.size());
}

/**
 * @brief 检查 ArmJacobian 初始化状态
 * @details 返回是否已成功解析 URDF 并缓存了可动关节的几何信息。
 *          只有当此方法返回 true 时，geometric() 和 dls_solve() 等方法才能正常使用。
 * @return true 如果已准备就绪（URDF已解析），false 如果还在等待 URDF 消息
 * @note 线程安全
 */
bool ArmJacobian::ready() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return ready_;
}

/**
 * @brief      基于TF实时查询，计算机械臂几何雅可比矩阵(空间雅可比 / base坐标系)
 * @details    几何雅可比，所有向量均表达于 base_frame 坐标系。
 *             支持旋转关节(revolute)与移动关节(prismatic)；
 *             通过查询每个关节子连杆相对于基座的TF，获取关节轴方向与位置，
 *             按标准雅可比公式逐列填充。
 *             失败场景：类未就绪、关节名不在URDF缓存、任意TF变换查找失败，返回
 * std::nullopt。 返回矩阵维度：[6 × n_joint]，上三行‑线速度，下三行‑角速度。
 * @param[in]  tf              tf2缓冲区，用于查询连杆之间坐标变换
 * @param[in]  joint_names 需要计算雅可比的关节名称序列，顺序决定雅可比列顺序
 * @param[in]  base_frame      雅可比参考基座坐标系名称
 * @param[in]  eef_link        末端执行器连杆名称
 * @return     std::optional<Eigen::MatrixXd>
 *             成功则返回6×N几何雅可比矩阵；失败返回 std::nullopt
 * @note
 *  - 旋转关节列：
 *    \f[
 *    \boldsymbol{J}_{i}=
 *    \begin{bmatrix}
 *    \boldsymbol{z}_i\times(\boldsymbol{p}_{ee}-\boldsymbol{p}_i)\\
 *    \boldsymbol{z}_i
 *    \end{bmatrix}
 *    \f]
 *  - 移动关节列：
 *    \f[
 *    \boldsymbol{J}_{i}=
 *    \begin{bmatrix}
 *    \boldsymbol{z}_i\\
 *    \boldsymbol{0}
 *    \end{bmatrix}
 *    \f]
 *  - 内部加锁读取URDF关节几何缓存 geom_
 *  - TF查询使用 TimePointZero，取最新可用变换
 */
std::optional<Eigen::MatrixXd> ArmJacobian::geometric(
    const tf2_ros::Buffer& tf, const std::vector<std::string>& joint_names,
    const std::string& base_frame, const std::string& eef_link) const {
    std::vector<JointGeom> chain;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!ready_) return std::nullopt;
        chain.reserve(joint_names.size());
        for (const auto& n : joint_names) {
            auto it = geom_.find(n);
            if (it == geom_.end()) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                     "ArmJacobian: URDF 里没有关节 '%s'",
                                     n.c_str());
                return std::nullopt;
            }
            chain.push_back(it->second);
        }
    }

    Eigen::Vector3d p_ee;
    Eigen::Quaterniond q_ee;
    try {
        split(tf.lookupTransform(base_frame, eef_link, tf2::TimePointZero),
              q_ee, p_ee);
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                             "ArmJacobian: TF %s ← %s 查询失败：%s",
                             base_frame.c_str(), eef_link.c_str(), e.what());
        return std::nullopt;
    }

    Eigen::MatrixXd J(6, static_cast<Eigen::Index>(chain.size()));
    for (size_t i = 0; i < chain.size(); ++i) {
        Eigen::Vector3d p_i;
        Eigen::Quaterniond q_i;
        try {
            split(tf.lookupTransform(base_frame, chain[i].child_link,
                                     tf2::TimePointZero),
                  q_i, p_i);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000, "ArmJacobian: TF %s ← %s 查询失败：%s",
                base_frame.c_str(), chain[i].child_link.c_str(), e.what());
            return std::nullopt;
        }
        const Eigen::Vector3d z =
            q_i * chain[i].axis;  // 关节轴在 base 系下的方向
        const auto c = static_cast<Eigen::Index>(i);
        if (chain[i].prismatic) {
            J.block<3, 1>(0, c) = z;  // 线速度 = 轴方向
            J.block<3, 1>(3, c) =
                Eigen::Vector3d::Zero();  // 移动副不产生角速度
        } else {
            J.block<3, 1>(0, c) = z.cross(p_ee - p_i);  // 线速度 = z × r
            J.block<3, 1>(3, c) = z;                    // 角速度 = 轴方向
        }
    }
    return J;
}

/**
 * @brief   阻尼最小二乘法(DLS)求解雅可比伪逆，用于机械臂速度级逆运动学
 * @details
 * 阻尼最小二乘(Damped‑Least‑Squares)，避免雅可比矩阵奇异位形下求解抖动。
 *          通过最小奇异值自适应调节阻尼系数；当最小奇异值低于阈值 eps
 * 时开启阻尼。 求解公式：\f$ \dot{\boldsymbol{q}} =
 * \boldsymbol{J}^T\left(\boldsymbol{J}\boldsymbol{J}^T+\lambda^2
 * \boldsymbol{I}\right)^{-1}\boldsymbol{\xi} \f$
 * @param[in]  J           雅可比矩阵，维度 (task_dim, joint_dim)
 * @param[in]  xi          任务空间速度向量 \f$\boldsymbol{\xi}\f$，维度
 * (task_dim,)
 * @param[in]  eps         奇异判定阈值；最小奇异值小于该值时启用阻尼
 * @param[in]  lambda_max  阻尼系数最大值 \f$\lambda_{max}\f$
 * @param[out] sigma_min   输出参数，矩阵 J 的最小奇异值；可为 nullptr
 * @return     Eigen::VectorXd  关节速度解 \f$\dot{\boldsymbol{q}}\f$，维度
 * (joint_dim,)
 * @note       阻尼项计算:
 *             \f[
 *             r = \sigma_{\min}/\varepsilon,\quad
 *             \lambda^2=(1-r^2)\lambda_{\max}^2
 *             \f]
 *             采用 LDLT 分解求解正定方程组，适合实时控制。
 */
Eigen::VectorXd ArmJacobian::dls_solve(const Eigen::MatrixXd& J,
                                       const Eigen::VectorXd& xi, double eps,
                                       double lambda_max, double* sigma_min) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const double s_min =
        svd.singularValues().size() > 0 ? svd.singularValues().minCoeff() : 0.0;
    if (sigma_min) *sigma_min = s_min;

    double lambda2 = 0.0;
    if (eps > 0.0 && s_min < eps) {
        const double r = s_min / eps;  // ∈ [0, 1)
        lambda2 = (1.0 - r * r) * lambda_max *
                  lambda_max;  // σ_min→0 时取 lambda_max²
    }

    const Eigen::MatrixXd A =
        J * J.transpose() +
        lambda2 * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    return J.transpose() * A.ldlt().solve(xi);
}

}  // namespace robot_arm_node::motion
