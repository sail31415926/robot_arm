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
  install(PROGRAMS gui/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── controllers/：控制节点（供 GUI import 或独立运行）────────────────────────────
install(PROGRAMS
  controllers/joint_position_controller_node.py
  controllers/cartesian_moveit_controller_node.py
  controllers/cartesian_realtime_ik_controller_node.py
  controllers/cartesian_velocity_controller_node.py
  controllers/cartesian_trajectory_controller_node.py
  controllers/spherical_orbit_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── vision/：视觉感知节点 ─────────────────────────────────────────────────────
foreach(script
  ibvs_control_node
  red_box_detector
)
  install(PROGRAMS vision/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── vision/IBVS_Controller：IBVS 算法库（共享资源）──────────────────────────
install(DIRECTORY vision/IBVS_Controller
  DESTINATION share/${PROJECT_NAME}
)

# ── commander/：Arm Commander 中间层 ─────────────────────────────────────────
install(PROGRAMS commander/arm_commander_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_commander_node
)
install(PROGRAMS
  commander/__init__.py
  commander/move_to_pose_server.py
  commander/trajectory_shot_server.py
  commander/track_target_server.py
  commander/motion_executor.py
  commander/status_aggregator.py
  DESTINATION lib/${PROJECT_NAME}/commander
)

# ── simulation/：MuJoCo 仿真 ─────────────────────────────────────────────────
foreach(script
  mujoco_node
  mujoco_data_recorder
)
  install(PROGRAMS simulation/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── tools/：工具脚本 ─────────────────────────────────────────────────────────
install(PROGRAMS tools/arm_utils.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS tools/arm_trajectory_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_trajectory_bridge
)

# ── tests/ ────────────────────────────────────────────────────────────────────
install(PROGRAMS tests/commander_test_gui.py
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
  add_executable(visp_ibvs_node vision/visp_ibvs_node.cpp)
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
