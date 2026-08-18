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
find_package(canopen_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(canopen_interfaces REQUIRED)
# find_package(robot_gimbal_interfaces_v2 REQUIRED)  # 仅 gimbal_v2_bridge 用，已停用（见下）
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ── 源码分类（2026-08-17 整理）────────────────────────────────────────────────
# src/hardware/    ros2_control 硬件插件（随栈加载）
# src/nodes/       常驻 ROS 节点（伴生服务、调试后端）
# src/tools/       一次性 CLI 工具（SocketCAN 直连，先停栈再跑）
# src/deprecated/  停用但保留参考的源码（不编译）
# scripts/         Python 脚本（GUI 等，install(PROGRAMS) 登记）

# ── CANopen 总线配置（ros2_canopen）──────────────────────────────────────────
# config/canopen/{bus.yml, RB200-CA.eds} → dcfgen 生成 master.dcf，
# 一并安装到 share/${PROJECT_NAME}/config/canopen/，供 URDF RobotSystem 插件引用。
generate_dcf(canopen)

# ── UnwrapRobotSystem（ros2_control 硬件插件：RobotSystem + 绝对编码器 2π 折算）──
# 上电反馈落 [0,2π) 时按 URDF 行程窗就近归位，读写对称，保证回零走最短方向。
# URDF real 后端与 test_arm.launch.py 均引用 robot_arm_driver/UnwrapRobotSystem。
add_library(unwrap_robot_system SHARED src/hardware/unwrap_robot_system.cpp)
target_include_directories(unwrap_robot_system PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(unwrap_robot_system
  rclcpp canopen_ros2_control hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface unwrap_robot_system_plugin.xml)
install(TARGETS unwrap_robot_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ── arm_driver_services（使能/复位伴生服务节点，Trigger → controller_manager）──
add_executable(arm_driver_services src/nodes/arm_driver_services.cpp)
ament_target_dependencies(arm_driver_services
  rclcpp std_srvs controller_manager_msgs lifecycle_msgs)
install(TARGETS arm_driver_services DESTINATION lib/${PROJECT_NAME})

# ── gimbal_v2_bridge：2026-07-31 起不再编译 ─────────────────────────────────────
# 臂侧对云台的接触面只有两条 sensor_msgs/JointState 话题：发 /robot_gimbal_v2/forward_cmd、
# 收 /robot_gimbal_v2/joint_states_raw（转发插件自己收发），云台板端 robot_gimbal_node_v2
# 原生就认这两个话题 —— 臂侧不需要任何适配节点，也不需要云台的自定义消息包。
# 这个桥（src/deprecated/gimbal_v2_bridge.cpp，翻译成 GimbalCommand/GimbalStatus）会造成双重驱动，
# 且把转发流升级成 POSITION 从而解冻板端 FROZEN，故停用；源码保留供日后 absolute_mode
# 换算参考（重启用须改成不与板端原生话题重叠的接法，并同时恢复下面的依赖）。
# 详见 docs/云台控制路径融合方案.md 第 0 节与 robot_arm_bringup/launch/real.launch.py v3.1。
# add_executable(gimbal_v2_bridge src/deprecated/gimbal_v2_bridge.cpp)
# ament_target_dependencies(gimbal_v2_bridge
#   rclcpp sensor_msgs robot_gimbal_interfaces_v2 tf2 tf2_ros)
# install(TARGETS gimbal_v2_bridge DESTINATION lib/${PROJECT_NAME})

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

# ── 单电机调试平台（launch/motor_debug.launch.py，一实例一电机）───────────────
# motor_debug_backend  C++ 总线后端：独占 SocketCAN，402 状态机 + PP/PV/PT + 置零
#                      + SDO 读写服务，接口复用 canopen_interfaces
# motor_debug_gui.py   PyQt5 前端：只做界面，经话题/服务与后端交互，不碰 CAN
#                      （新增 .py 必须在这里 install(PROGRAMS) 登记，否则运行时找不到）
add_executable(motor_debug_backend src/nodes/motor_debug_backend.cpp)
target_include_directories(motor_debug_backend PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(motor_debug_backend
  rclcpp std_srvs std_msgs sensor_msgs canopen_interfaces)
install(TARGETS motor_debug_backend DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/motor_debug_gui.py DESTINATION lib/${PROJECT_NAME})

# ── 自测栈（launch/test_arm.launch.py，mock/vcan假从站/真机 三合一）────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(FILES config/test_controllers.yaml config/can.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
