cmake_minimum_required(VERSION 3.8)
project(robot_arm_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# cmake 3.22 的 FindBoost._Boost_COMPONENT_HEADERS 没有 python310 的头文件映射，
# eigenpy（pinocchio 的依赖）会触发一条无害警告。
# 预先定义内部变量使其跳过"No header defined"分支，消除警告。
set(Boost_NO_WARN_NEW_VERSIONS ON)
set(_Boost_PYTHON310_HEADERS "boost/python.hpp")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_arm_interfaces REQUIRED)


# ══════════════════════════════════════════════════════════════════════════════
# 产品栈转 C++（motion → state → commander）。第一刀：motion 规划层 C++ 库。
# 目录：include/robot_arm_node/motion/*.hpp + src/motion/*.cpp
#   constants.hpp / geometry.hpp（header-only）、planning（Ruckig 球面路点）、
#   trajectory（decimate + JointTrajectory 构建）。
#   solve_ik（异步 /compute_ik）与 solve_and_send 待 kinematics 落地后补。
# ══════════════════════════════════════════════════════════════════════════════
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ruckig REQUIRED)
# joint_limits.cpp：从 /robot_description 解析 URDF 取关节限位（不写死常量，
# 限位会随实机标定平移——见 2026-07-28 J2/J3 零点重标定）
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
# jacobian.cpp：TF + URDF 几何法算末端线速度 Jacobian（笛卡尔速度控制内核）。
# 用 Eigen 做 3×3 SVD / DLS 求解；tf2_ros 提供位形查询。
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(robot_arm_motion SHARED
  src/motion/jacobian.cpp
  src/motion/joint_limits.cpp
  src/motion/kinematics.cpp
  src/motion/planning.cpp
  src/motion/trajectory.cpp
)
target_include_directories(robot_arm_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_motion PUBLIC cxx_std_17)
ament_target_dependencies(robot_arm_motion
  rclcpp trajectory_msgs builtin_interfaces geometry_msgs moveit_msgs urdf std_msgs tf2_ros)
target_link_libraries(robot_arm_motion ruckig::ruckig Eigen3::Eigen)

install(TARGETS robot_arm_motion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ══════════════════════════════════════════════════════════════════════════════
# 产品栈第二刀：state 层。StatusAggregator —— 订阅 /joint_states + TF2 → ArmStatus。
#   非节点，共享调用方（未来 C++ commander）的 ROS 资源；比 motion 重（tf2 + interfaces）
#   故独立成库，链接 robot_arm_motion 复用 geometry / constants。
# ══════════════════════════════════════════════════════════════════════════════
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(robot_arm_state SHARED
  src/state/status_aggregator.cpp
)
target_include_directories(robot_arm_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_state PUBLIC cxx_std_17)
ament_target_dependencies(robot_arm_state
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros builtin_interfaces robot_arm_interfaces)
target_link_libraries(robot_arm_state robot_arm_motion)

install(TARGETS robot_arm_state
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ══════════════════════════════════════════════════════════════════════════════
# 产品栈第三刀：commander 层。状态机 + 4 rclcpp_action Server + 4 Service + ArmStatus 广播。
#   库 robot_arm_commander（motion_policy/execution_monitor/motion_executor/4 server/node）
#   链 robot_arm_state + robot_arm_motion；可执行 arm_commander_node（headless）。
# ══════════════════════════════════════════════════════════════════════════════
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(robot_arm_commander SHARED
  src/commander/execution_monitor.cpp
  src/commander/motion_executor.cpp
  src/commander/move_to_pose_server.cpp
  src/commander/move_to_joint_server.cpp
  src/commander/trajectory_shot_server.cpp
  src/commander/track_target_server.cpp
  src/commander/velocity_stream_server.cpp
  src/commander/arm_commander_node.cpp
)
target_include_directories(robot_arm_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_commander PUBLIC cxx_std_17)
ament_target_dependencies(robot_arm_commander
  rclcpp rclcpp_action rcl_interfaces std_srvs geometry_msgs trajectory_msgs
  moveit_msgs builtin_interfaces robot_arm_interfaces)
target_link_libraries(robot_arm_commander robot_arm_state robot_arm_motion)

install(TARGETS robot_arm_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(arm_commander_node src/commander/arm_commander_main.cpp)
target_compile_features(arm_commander_node PRIVATE cxx_std_17)
ament_target_dependencies(arm_commander_node rclcpp)
target_link_libraries(arm_commander_node robot_arm_commander)
install(TARGETS arm_commander_node DESTINATION lib/${PROJECT_NAME})

# ══════════════════════════════════════════════════════════════════════════════
# 控制模式仲裁器：语义控制模式（TRAJECTORY/JOINT_VELOCITY/JOINT_EFFORT/ADMITTANCE）的
#   唯一权威，后端无感。切换 = controller_manager/switch_controller；带 bumpless 播种 +
#   速度/力矩总线看门狗。独立可执行，不依赖 commander 库。
# ══════════════════════════════════════════════════════════════════════════════
find_package(controller_manager_msgs REQUIRED)   # std_msgs 已在 motion 段 find_package

add_executable(mode_manager_node src/mode/mode_manager_node.cpp)
target_compile_features(mode_manager_node PRIVATE cxx_std_17)
ament_target_dependencies(mode_manager_node
  rclcpp std_msgs std_srvs sensor_msgs trajectory_msgs controller_manager_msgs
  robot_arm_interfaces)
# 链 robot_arm_motion：复用 JointLimitsCache 做速度总线的限位刹车（限位单一来源 = URDF）
target_link_libraries(mode_manager_node robot_arm_motion)
install(TARGETS mode_manager_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

# 注：本包为**产品层**（C++ motion/state/commander）。
#     Python 调试控制器/GUI/工具 + 视觉感知（visp_ibvs_node/red_box_detector，
#     暂归调试）已迁至 robot_arm_debug 包（2026-07-13）；
#     MuJoCo 仿真桥已迁至 robot_arm_mujoco（2026-07-10）。

ament_package()
