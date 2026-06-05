cmake_minimum_required(VERSION 3.8)
project(robot_arm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(include)

# ── CANopen 驱动库（SocketCAN，无外部依赖）──────────────────────────────────
add_library(canopen_motor_driver src/canopen_motor_driver.cpp)
target_include_directories(canopen_motor_driver PUBLIC include)

# ── arm_motor_node ────────────────────────────────────────────────────────────
add_executable(arm_motor_node src/arm_motor_node.cpp)
target_link_libraries(arm_motor_node canopen_motor_driver)
ament_target_dependencies(arm_motor_node rclcpp std_msgs std_srvs sensor_msgs diagnostic_msgs)
install(TARGETS arm_motor_node DESTINATION lib/${PROJECT_NAME})

# ── arm_node（JointTrajectory / joint_states 接口）───────────────────────────
add_executable(arm_node src/arm_node.cpp)
target_link_libraries(arm_node canopen_motor_driver)
ament_target_dependencies(arm_node rclcpp rclcpp_action sensor_msgs trajectory_msgs std_srvs control_msgs)
install(TARGETS arm_node DESTINATION lib/${PROJECT_NAME})

# ── set_encoder_zero（绝对编码器零点标定工具）────────────────────────────────
add_executable(set_encoder_zero src/set_encoder_zero.cpp)
target_link_libraries(set_encoder_zero canopen_motor_driver)
install(TARGETS set_encoder_zero DESTINATION lib/${PROJECT_NAME})

# ── set_position_limit（软件位置限位标定工具）────────────────────────────────
add_executable(set_position_limit src/set_position_limit.cpp)
target_link_libraries(set_position_limit canopen_motor_driver)
install(TARGETS set_position_limit DESTINATION lib/${PROJECT_NAME})

# ── Python 调试工具 ───────────────────────────────────────────────────────────
install(PROGRAMS scripts/motor_test_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_test_gui
)

# ── 配置文件 ──────────────────────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
