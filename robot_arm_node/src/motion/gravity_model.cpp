/**
 * @file   gravity_model.cpp
 * @brief  GravityModel 实现 —— pinocchio 建模 + computeGeneralizedGravity
 */
#include "robot_arm_node/motion/gravity_model.hpp"

#include <algorithm>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace robot_arm_node::motion
{

struct GravityModel::Impl
{
  pinocchio::Model model;
  // data 随 model 走；gravity() 里会写它，故 gravity() 需要持锁（见头文件说明）
  mutable pinocchio::Data data;
  bool ready{false};

  // 关节名 → (idx_q, idx_v)。q 用于填位形，v 用于取 g 向量分量。
  // 二者对旋转关节都是 1 维且一一对应，但分开存以免将来加多自由度关节时踩坑。
  std::map<std::string, std::pair<int, int>> joint_index;
  int nq{0};
};

GravityModel::GravityModel(rclcpp::Node & node, const std::string & topic)
: logger_(node.get_logger().get_child("gravity_model")),
  impl_(std::make_unique<Impl>())
{
  // URDF 是 latched（TRANSIENT_LOCAL）发布的，晚订阅也能收到最后一帧
  sub_ = node.create_subscription<std_msgs::msg::String>(
      topic,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String & m) { onDescription(m); });
}

GravityModel::~GravityModel() = default;

void GravityModel::onDescription(const std_msgs::msg::String & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (impl_->ready) return;   // 只建一次：URDF 在一次运行里不会变

  try {
    pinocchio::Model model;
    // 固定基座（不给 root joint）：URDF 的根是 world，经 fixed joint 连到 arm_base_link
    pinocchio::urdf::buildModelFromXML(msg.data, model);

    std::map<std::string, std::pair<int, int>> index;
    for (pinocchio::JointIndex j = 1; j < model.joints.size(); ++j) {
      const auto & jm = model.joints[j];
      if (jm.nq() == 0) continue;      // 固定关节等无自由度的跳过
      index[model.names[j]] = {jm.idx_q(), jm.idx_v()};
    }

    impl_->model = std::move(model);
    impl_->data  = pinocchio::Data(impl_->model);
    impl_->joint_index = std::move(index);
    impl_->nq = impl_->model.nq;
    impl_->ready = true;

    std::string names;
    for (const auto & [n, _] : impl_->joint_index) names += (names.empty() ? "" : ", ") + n;
    // 质量合计只作日志自检（0 kg 说明 URDF 没有 <inertial>，重力补偿会全是 0）——
    // 直接累加 model.inertias，省掉 center-of-mass.hpp 这个额外的重头
    double total_mass = 0.0;
    for (const auto & I : impl_->model.inertias) total_mass += I.mass();
    RCLCPP_INFO(logger_,
        "重力模型就绪：nq=%d nv=%d 质量合计 %.3f kg  可动关节 [%s]",
        impl_->model.nq, impl_->model.nv, total_mass, names.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "pinocchio 建模失败，重力补偿不可用：%s", e.what());
    impl_->ready = false;
  }
}

bool GravityModel::ready() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return impl_->ready;
}

std::vector<std::string> GravityModel::movable_joints() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<std::string> out;
  for (const auto & [n, _] : impl_->joint_index) out.push_back(n);
  return out;
}

bool GravityModel::gravity(const std::map<std::string, double> & positions,
                           const std::vector<std::string> & joints,
                           std::vector<double> & out) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!impl_->ready) return false;

  // 位形必须完整：模型里每个可动关节都得有回读。
  // 缺就整体放弃 —— 用默认值（0）顶替缺失关节会算出一个错误的重力力矩，
  // 那是主动施加的错误力，比"不补偿"危险得多。
  Eigen::VectorXd q = Eigen::VectorXd::Zero(impl_->nq);
  for (const auto & [name, idx] : impl_->joint_index) {
    auto it = positions.find(name);
    if (it == positions.end()) return false;
    q[idx.first] = it->second;
  }

  pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q);

  out.clear();
  out.reserve(joints.size());
  for (const auto & name : joints) {
    auto it = impl_->joint_index.find(name);
    if (it == impl_->joint_index.end()) return false;
    out.push_back(impl_->data.g[it->second.second]);
  }
  return true;
}

}  // namespace robot_arm_node::motion
