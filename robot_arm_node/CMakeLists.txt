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

add_library(robot_arm_motion SHARED
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
  rclcpp trajectory_msgs builtin_interfaces geometry_msgs moveit_msgs)
target_link_libraries(robot_arm_motion ruckig::ruckig)

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
# 产品栈第三刀：commander 层。状态机 + 3 rclcpp_action Server + 4 Service + ArmStatus 广播。
#   库 robot_arm_commander（motion_policy/execution_monitor/motion_executor/3 server/node）
#   链 robot_arm_state + robot_arm_motion；可执行 arm_commander_node（headless）。
# ══════════════════════════════════════════════════════════════════════════════
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(robot_arm_commander SHARED
  src/commander/execution_monitor.cpp
  src/commander/motion_executor.cpp
  src/commander/move_to_pose_server.cpp
  src/commander/trajectory_shot_server.cpp
  src/commander/track_target_server.cpp
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

install(DIRECTORY include/ DESTINATION include)


# ── gui/：所有 GUI 节点（rename 为无扩展名可执行，ros2 run 调起）────────────
foreach(script
  joint_position_gui
  cartesian_moveit_gui
  cartesian_realtime_ik_gui
  cartesian_trajectory_gui
  cartesian_velocity_gui
  spherical_orbit_gui
  visp_ibvs_gui
)
  install(PROGRAMS python/gui/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── python/controllers/：调试控制节点（留 Python，供 GUI import 或独立运行）────────
install(PROGRAMS
  python/controllers/joint_position_controller_node.py
  python/controllers/cartesian_moveit_controller_node.py
  python/controllers/cartesian_realtime_ik_controller_node.py
  python/controllers/cartesian_velocity_controller_node.py
  python/controllers/cartesian_trajectory_controller_node.py
  python/controllers/spherical_orbit_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── python/vision/：视觉感知节点（留 Python；红方块检测器，喂特征给 C++ visp_ibvs_node）──
install(PROGRAMS python/vision/red_box_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME red_box_detector
)
# 注：Python 版 IBVS（ibvs_control_node + IBVS_Controller/）已被 C++ visp_ibvs_node 取代并删除。

# 注：Python 产品栈（commander/ + arm_motion/）已被 C++ motion/state/commander 取代并删除
#     （Gazebo 端到端验证通过后清理，2026-07-02）。几何数学复用见上方 robot_arm_geometry。

# ── python/simulation/：MuJoCo 仿真（留 Python）──────────────────────────────
foreach(script
  mujoco_node
  mujoco_data_recorder
)
  install(PROGRAMS python/simulation/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── python/tools/：工具脚本（留 Python）───────────────────────────────────────
install(PROGRAMS python/tools/arm_utils.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS python/tools/arm_trajectory_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_trajectory_bridge
)

# ── python/tests/ ─────────────────────────────────────────────────────────────
install(PROGRAMS python/tests/commander_test_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME commander_test_gui
)

# ── vision/：ViSP + Pinocchio IBVS C++ 节点 ──────────────────────────────────
find_package(VISP QUIET)
find_package(pinocchio QUIET)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros2_algo_vision_interfaces QUIET)

if(VISP_FOUND AND pinocchio_FOUND)
  add_executable(visp_ibvs_node src/vision/visp_ibvs_node.cpp)
  target_compile_features(visp_ibvs_node PRIVATE cxx_std_17)
  target_include_directories(visp_ibvs_node PRIVATE
    ${VISP_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS})
  if(ros2_algo_vision_interfaces_FOUND)
    target_compile_definitions(visp_ibvs_node PRIVATE HAS_PERCEPTION_REPORT)
    ament_target_dependencies(visp_ibvs_node
      rclcpp geometry_msgs sensor_msgs trajectory_msgs ros2_algo_vision_interfaces)
    message(STATUS "ros2_algo_vision_interfaces found — PerceptionReport support enabled")
  else()
    ament_target_dependencies(visp_ibvs_node
      rclcpp geometry_msgs sensor_msgs trajectory_msgs)
    message(STATUS "ros2_algo_vision_interfaces not found — PerceptionReport support disabled")
  endif()
  target_link_libraries(visp_ibvs_node ${VISP_LIBRARIES} pinocchio::pinocchio)
  install(TARGETS visp_ibvs_node DESTINATION lib/${PROJECT_NAME})
  message(STATUS "ViSP (${VISP_VERSION}) + Pinocchio found — visp_ibvs_node will be built")
else()
  message(WARNING "VISP or Pinocchio not found — visp_ibvs_node skipped. Install ros-humble-visp ros-humble-pinocchio.")
endif()

ament_package()
