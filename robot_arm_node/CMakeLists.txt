cmake_minimum_required(VERSION 3.16)
project(robot_arm_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_arm_interfaces REQUIRED)

add_executable(robot_arm_node src/main.cpp)

ament_target_dependencies(robot_arm_node
  rclcpp
  robot_arm_interfaces
)

install(TARGETS robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
