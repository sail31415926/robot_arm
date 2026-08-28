/**
 * @file teach_main.cpp
 * @brief arm_teach_node 可执行入口
 *
 * ★ 必须用 MultiThreadedExecutor。单线程执行器下，服务回调里阻塞等
 *   /robot_arm/switch_control_mode 的应答会把整个执行器堵死 —— 应答永远轮不到处理，
 *   每次切模式都"超时"，而 ModeManager 那边其实早就成功了。
 *   回调组的划分见 teach_node.hpp 的「线程与锁」。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_arm_teach/teach_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<robot_arm_teach::TeachNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
