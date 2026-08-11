/**
 * @file   gravity_model.hpp
 * @brief  重力补偿模型 —— 从 /robot_description 建 pinocchio 模型，算 g(q)
 *
 * 为什么需要：力矩模式（JOINT_EFFORT）下关节只受「指令力矩 + 重力」支配，没有位置
 * 闭环。零指令力矩 = 自由下垂 —— 实测切进力矩模式后 J2 立刻从 +0.500 掉到 -0.981
 * （下限）、J3 从 -1.000 掉到 +0.020（上限）。所以力矩模式要能用，必须先把重力抵掉：
 *   下发力矩 = g(q) + 用户增量
 * 这样「零用户力矩」的语义才是「原地停住」，与速度模式的「零速度 = 停住」对齐。
 *
 * 限位/惯量的单一真相源仍是 URDF（与 JointLimitsCache 同一约定）：模型直接从
 * latched 的 /robot_description 建，不写死任何连杆参数 —— 改了 URDF（换云台、
 * 重标零点、调质量）自动跟随，不会留下第二份真相。
 *
 * 单位：N·m，与 URDF <limit effort> 一致。实物侧 CiA402 的 6071 是「0.1% 额定转矩」，
 * 那层换算要在**本模型之后**做（见 mode_manager_node 的力矩总线注释）。
 */
#ifndef ROBOT_ARM_NODE__MOTION__GRAVITY_MODEL_HPP_
#define ROBOT_ARM_NODE__MOTION__GRAVITY_MODEL_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_arm_node::motion
{

/**
 * 重力补偿模型。订阅 /robot_description（latched）建模，随后可反复查询 g(q)。
 *
 * 线程安全：build 在订阅回调里做，gravity() 可能在定时器线程调用，内部加锁。
 */
class GravityModel
{
public:
  /// node：共享其 ROS 资源（订阅归属该节点）。topic 默认 /robot_description。
  explicit GravityModel(rclcpp::Node & node,
                        const std::string & topic = "/robot_description");
  ~GravityModel();

  /// 模型是否已就绪（收到首帧 URDF 且 pinocchio 建模成功）
  bool ready() const;

  /**
   * 算重力力矩。
   *
   * @param positions  关节名 → 当前角度(rad)。**必须覆盖模型里所有可动关节** ——
   *                   缺任何一个都返回 false（宁可不补也不能拿残缺位形去算：
   *                   用错位形算出的重力力矩是错误的主动力，比不补更危险）。
   * @param joints     要取哪几个关节的重力分量（顺序即返回顺序）
   * @param out        输出 g(q)，N·m，长度 = joints.size()
   * @return           成功 true；模型未就绪 / 位形不全 / 关节名不认识 → false
   */
  bool gravity(const std::map<std::string, double> & positions,
               const std::vector<std::string> & joints,
               std::vector<double> & out) const;

  /// 模型里所有可动关节名（诊断用）
  std::vector<std::string> movable_joints() const;

private:
  void onDescription(const std_msgs::msg::String & msg);

  rclcpp::Logger logger_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  // pinocchio 类型不进头文件（避免把它的模板重量传染给所有包含者，编译时间会炸）
  struct Impl;
  std::unique_ptr<Impl> impl_;
  mutable std::mutex mtx_;
};

}  // namespace robot_arm_node::motion

#endif  // ROBOT_ARM_NODE__MOTION__GRAVITY_MODEL_HPP_
