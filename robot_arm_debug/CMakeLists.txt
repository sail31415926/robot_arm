cmake_minimum_required(VERSION 3.8)
project(robot_arm_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# cmake 3.22 的 FindBoost 没有 python310 头文件映射，eigenpy（pinocchio 依赖）
# 会触发无害警告；预定义内部变量跳过"No header defined"分支
set(Boost_NO_WARN_NEW_VERSIONS ON)
set(_Boost_PYTHON310_HEADERS "boost/python.hpp")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ══════════════════════════════════════════════════════════════════════════════
# 调试/开发工具包：Python 调试控制器 + PyQt GUI + 视觉感知（暂归调试）。
# 产品层（C++ motion/state/commander）在 robot_arm_node，部署时无需本包。
# 注意：GUI 进程内 import 同目录的 controller 模块，二者必须装进同一个
# lib/${PROJECT_NAME}；新增 .py 记得加进下方 install(PROGRAMS)。
# ══════════════════════════════════════════════════════════════════════════════

# ── python/gui/：所有 GUI 节点（rename 为无扩展名可执行，ros2 run 调起）────────
foreach(script
  joint_position_gui
  cartesian_moveit_gui
  cartesian_realtime_ik_gui
  cartesian_trajectory_gui
  cartesian_velocity_gui
  spherical_orbit_gui
  visp_ibvs_gui
  commander_test_gui
)
  install(PROGRAMS python/gui/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── python/controllers/：调试控制节点（留 Python，供 GUI import 或独立运行）────
install(PROGRAMS
  python/controllers/joint_position_controller_node.py
  python/controllers/cartesian_moveit_controller_node.py
  python/controllers/cartesian_realtime_ik_controller_node.py
  python/controllers/cartesian_velocity_controller_node.py
  python/controllers/cartesian_trajectory_controller_node.py
  python/controllers/spherical_orbit_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── python/vision/：视觉感知（红方块检测器，喂特征给 C++ visp_ibvs_node）──────
install(PROGRAMS python/vision/red_box_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME red_box_detector
)

# ── python/tools/：进程内几何工具库（controllers import 用）──────────────────
install(PROGRAMS python/tools/arm_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── src/vision/：ViSP + Pinocchio IBVS C++ 节点 ───────────────────────────────
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
