/**
 * @file arm_commander_main.cpp
 * @brief arm_commander_node 可执行入口（headless）
 *
 * 对应 Python arm_commander_node.py 的 main()。MultiThreadedExecutor：Action 执行线程
 * 阻塞跑 IK / 等待，服务回调同步等 future，均需多线程 spin。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_arm_node/commander/arm_commander_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_arm_node::commander::ArmCommanderNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
