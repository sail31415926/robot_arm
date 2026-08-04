/**
 * @file jacobian.hpp
 * @brief 末端线速度 Jacobian（几何法，TF + URDF）+ DLS 伪逆 —— 笛卡尔速度控制的运动学内核
 *
 * 【为什么自己算而不用 MoveIt / pinocchio】
 *   需要的只是「臂 J1-3 对末端原点的 3×3 线速度 Jacobian」。几何法一行公式就够：
 *   转动轴 i 对末端线速度的贡献 = z_i × (p_ee − p_i)，其中 z_i 是关节轴在 base 系下的
 *   方向、p_i 是关节原点、p_ee 是末端原点 —— 三者全都能从 TF 直接查到，轴向量从 URDF 取。
 *   这样既不用拉 moveit_core（还得等 move_group 起来才有 SRDF），也不用引入 pinocchio，
 *   而且**天然吃到云台 J4-6 的当前角度**：末端 gimbal_tool0 在云台之后，TF 里就是实时位形，
 *   Jacobian 自动包含云台造成的偏置臂长，无需单独建模。
 *
 * 【数据来源】
 *   - 关节轴 / 子链接名：`/robot_description`（std_msgs/String，latched，与 JointLimitsCache 同源）
 *   - 位形：TF（robot_state_publisher 由 /joint_states 实时广播）
 *
 * 【就绪时序】构造后首帧 URDF 未必立刻到；`ready()==false` 或 TF 查询失败时 linear()
 *   返回 nullopt，调用方须**停车**（速度控制不能 fail-open，否则拿旧 Jacobian 会跑飞）。
 *
 * @version 1.0
 * @date 2026-08-04
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>

namespace robot_arm_node::motion
{

class ArmJacobian
{
public:
  // node：共享其 ROS 资源（/robot_description 订阅归属该节点）
  explicit ArmJacobian(rclcpp::Node & node,
                       const std::string & topic = "/robot_description");

  // URDF 是否已解析成功（收到首帧且目标关节均已登记）
  bool ready() const;

  /**
   * @brief base_frame 系下、eef_link 处、对 joint_names 各轴的 6×N 几何 Jacobian
   *
   * 上 3 行是线速度、下 3 行是角速度：转动副的列 = [z_i × (p_ee − p_i); z_i]，
   * 移动副的列 = [z_i; 0]。角速度部分与末端取哪个点无关，只看关节轴方向。
   *
   * @return URDF 未就绪 / 关节未登记 / TF 查询失败 → nullopt（调用方须停车）
   */
  std::optional<Eigen::MatrixXd> geometric(const tf2_ros::Buffer & tf,
                                           const std::vector<std::string> & joint_names,
                                           const std::string & base_frame,
                                           const std::string & eef_link) const;

  /**
   * @brief 阻尼最小二乘伪逆：q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ ξ
   *
   * λ 按最小奇异值自适应：σ_min ≥ eps 时 λ=0（精确解），越接近奇异越大，
   * 在 σ_min=0 处取 lambda_max。这样只在奇异点附近牺牲跟踪精度换取有界的关节速度。
   *
   * @param xi 目标速度，维数须等于 J.rows()（3=只线速度，6=完整 twist）
   * @param sigma_min 出参，本次的最小奇异值（可用于上报可操作度 / 告警），可传 nullptr
   */
  static Eigen::VectorXd dls_solve(const Eigen::MatrixXd & J, const Eigen::VectorXd & xi,
                                   double eps, double lambda_max, double * sigma_min = nullptr);

private:
  struct JointGeom
  {
    std::string child_link;         // 关节所在坐标系（URDF 中 joint 的 child link frame）
    Eigen::Vector3d axis{0, 0, 1};  // 关节轴，表达在 child_link 系
    bool prismatic{false};          // true=移动副（线速度直接是 z），false=转动副（z × r）
  };

  void on_description(const std_msgs::msg::String & msg);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;   // 节流告警用（linear() 是 const，故存 shared_ptr）
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  mutable std::mutex mtx_;
  std::map<std::string, JointGeom> geom_;
  bool ready_{false};
};

}  // namespace robot_arm_node::motion
