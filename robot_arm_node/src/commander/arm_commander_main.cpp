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

/**
 * @brief arm_commander_node 进程入口。
 *
 * 使用 MultiThreadedExecutor 而非单线程 spin：Action 执行线程会阻塞式跑 IK 与
 * 等待到位，服务回调又要同步等 future，单线程会自锁。
 *
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return 进程退出码，正常退出为 0。
 */
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
