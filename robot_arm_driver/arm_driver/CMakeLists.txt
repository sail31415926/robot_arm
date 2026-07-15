cmake_minimum_required(VERSION 3.8)
project(robot_arm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(lely_core_libraries REQUIRED)

# ── CANopen 总线配置（ros2_canopen）──────────────────────────────────────────
# config/canopen/{bus.yml, RB200-CA.eds} → dcfgen 生成 master.dcf，
# 一并安装到 share/${PROJECT_NAME}/config/canopen/，供 URDF RobotSystem 插件引用。
generate_dcf(canopen)

# ── arm_driver_services（使能/复位伴生服务节点，Trigger → controller_manager）──
add_executable(arm_driver_services src/arm_driver_services.cpp)
ament_target_dependencies(arm_driver_services
  rclcpp std_srvs controller_manager_msgs lifecycle_msgs)
install(TARGETS arm_driver_services DESTINATION lib/${PROJECT_NAME})

# ── 独立标定/调参工具（src/tools/，SocketCAN 直连，先停栈再跑）────────────────
# 共享头在 include/robot_arm_driver/（包内自用，不 install/export）：
#   sdo_client.hpp（极简 SDO 客户端）、motor_unit_converter.hpp（单位换算，
#   自研栈保留件，单一权威）
# set_encoder_zero  编码器零点标定（HM 方法 35，断电保持）
# set_motor_limits  速度/加速度上限、软限位、最大转矩 查看与设定（rad 单位输入）
# unit_convert      命令行换算器 rad↔pp（改 bus.yml 时用）
add_executable(set_encoder_zero src/tools/set_encoder_zero.cpp)
add_executable(set_motor_limits src/tools/set_motor_limits.cpp)
add_executable(unit_convert src/tools/unit_convert.cpp)
foreach(tool set_encoder_zero set_motor_limits unit_convert)
  target_include_directories(${tool} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endforeach()
install(TARGETS set_encoder_zero set_motor_limits unit_convert DESTINATION lib/${PROJECT_NAME})

# ── 自测栈（launch/test_arm.launch.py，mock/vcan假从站/真机 三合一）────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(FILES config/test_controllers.yaml config/can.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
