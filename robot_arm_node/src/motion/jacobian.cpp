/**
 * @file jacobian.cpp
 * @brief ArmJacobian 实现 —— URDF 取关节轴 + TF 取位形 → 几何线速度 Jacobian；DLS 伪逆
 *
 * @version 1.0
 * @date 2026-08-04
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/motion/jacobian.hpp"

#include <Eigen/SVD>
#include <urdf/model.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>

namespace robot_arm_node::motion
{

namespace
{
// TF → (旋转, 平移)。base_frame ← link 的位姿。
void split(const geometry_msgs::msg::TransformStamped & t,
           Eigen::Quaterniond & q, Eigen::Vector3d & p)
{
  q = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x,
                         t.transform.rotation.y, t.transform.rotation.z);
  q.normalize();
  p = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z);
}
}  // namespace

ArmJacobian::ArmJacobian(rclcpp::Node & node, const std::string & topic)
: logger_(node.get_logger()), clock_(node.get_clock())
{
  // 与 JointLimitsCache 同款 QoS：匹配 robot_state_publisher 的 latched 发布
  rclcpp::QoS qos(1);
  qos.transient_local().reliable();

  sub_ = node.create_subscription<std_msgs::msg::String>(
      topic, qos, [this](const std_msgs::msg::String & msg) { on_description(msg); });

  RCLCPP_INFO(logger_, "ArmJacobian: 等待 '%s'（latched）解析关节轴", topic.c_str());
}

void ArmJacobian::on_description(const std_msgs::msg::String & msg)
{
  urdf::Model model;
  if (!model.initString(msg.data)) {
    RCLCPP_ERROR(logger_, "ArmJacobian: URDF 解析失败，笛卡尔速度控制不可用");
    return;
  }

  std::map<std::string, JointGeom> parsed;
  for (const auto & [name, joint] : model.joints_) {
    if (!joint) continue;
    const bool revolute  = joint->type == urdf::Joint::REVOLUTE ||
                           joint->type == urdf::Joint::CONTINUOUS;
    const bool prismatic = joint->type == urdf::Joint::PRISMATIC;
    if (!revolute && !prismatic) continue;   // fixed / floating / planar 不参与

    JointGeom g;
    g.child_link = joint->child_link_name;
    g.prismatic  = prismatic;
    // URDF 的 <axis> 表达在关节坐标系 = child link 坐标系
    g.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
    if (g.axis.norm() < 1e-9) g.axis = Eigen::Vector3d::UnitZ();   // URDF 缺省轴
    g.axis.normalize();
    parsed[name] = g;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    geom_  = std::move(parsed);
    ready_ = !geom_.empty();
  }
  RCLCPP_INFO(logger_, "ArmJacobian: 已解析 %zu 个可动关节的轴向", geom_.size());
}

bool ArmJacobian::ready() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return ready_;
}

std::optional<Eigen::MatrixXd> ArmJacobian::geometric(
    const tf2_ros::Buffer & tf, const std::vector<std::string> & joint_names,
    const std::string & base_frame, const std::string & eef_link) const
{
  std::vector<JointGeom> chain;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ready_) return std::nullopt;
    chain.reserve(joint_names.size());
    for (const auto & n : joint_names) {
      auto it = geom_.find(n);
      if (it == geom_.end()) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                             "ArmJacobian: URDF 里没有关节 '%s'", n.c_str());
        return std::nullopt;
      }
      chain.push_back(it->second);
    }
  }

  Eigen::Vector3d p_ee;
  Eigen::Quaterniond q_ee;
  try {
    split(tf.lookupTransform(base_frame, eef_link, tf2::TimePointZero), q_ee, p_ee);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "ArmJacobian: TF %s ← %s 查询失败：%s",
                         base_frame.c_str(), eef_link.c_str(), e.what());
    return std::nullopt;
  }

  Eigen::MatrixXd J(6, static_cast<Eigen::Index>(chain.size()));
  for (size_t i = 0; i < chain.size(); ++i) {
    Eigen::Vector3d p_i;
    Eigen::Quaterniond q_i;
    try {
      split(tf.lookupTransform(base_frame, chain[i].child_link, tf2::TimePointZero), q_i, p_i);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "ArmJacobian: TF %s ← %s 查询失败：%s",
                           base_frame.c_str(), chain[i].child_link.c_str(), e.what());
      return std::nullopt;
    }
    const Eigen::Vector3d z = q_i * chain[i].axis;          // 关节轴在 base 系下的方向
    const auto c = static_cast<Eigen::Index>(i);
    if (chain[i].prismatic) {
      J.block<3, 1>(0, c) = z;                       // 线速度 = 轴方向
      J.block<3, 1>(3, c) = Eigen::Vector3d::Zero(); // 移动副不产生角速度
    } else {
      J.block<3, 1>(0, c) = z.cross(p_ee - p_i);     // 线速度 = z × r
      J.block<3, 1>(3, c) = z;                       // 角速度 = 轴方向
    }
  }
  return J;
}

Eigen::VectorXd ArmJacobian::dls_solve(const Eigen::MatrixXd & J, const Eigen::VectorXd & xi,
                                       double eps, double lambda_max, double * sigma_min)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  const double s_min = svd.singularValues().size() > 0 ? svd.singularValues().minCoeff() : 0.0;
  if (sigma_min) *sigma_min = s_min;

  double lambda2 = 0.0;
  if (eps > 0.0 && s_min < eps) {
    const double r = s_min / eps;                       // ∈ [0, 1)
    lambda2 = (1.0 - r * r) * lambda_max * lambda_max;  // σ_min→0 时取 lambda_max²
  }

  const Eigen::MatrixXd A =
      J * J.transpose() + lambda2 * Eigen::MatrixXd::Identity(J.rows(), J.rows());
  return J.transpose() * A.ldlt().solve(xi);
}

}  // namespace robot_arm_node::motion
