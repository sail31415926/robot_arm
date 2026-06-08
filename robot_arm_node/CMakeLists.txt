cmake_minimum_required(VERSION 3.8)
project(robot_arm_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_arm_interfaces REQUIRED)

# ── 主节点 ────────────────────────────────────────────────────────────────────
add_executable(robot_arm_node src/main.cpp)
ament_target_dependencies(robot_arm_node rclcpp robot_arm_interfaces)
install(TARGETS robot_arm_node DESTINATION lib/${PROJECT_NAME})

# ── Python 共享工具模块（保留 .py 后缀，供其他脚本 import）─────────────────────
install(PROGRAMS scripts/arm_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── controllers/：GUI 运动控制器 ──────────────────────────────────────────────
foreach(script
  arm_slider_controller
  cartesian_controller
  cartesian_realtime_controller
  cartesian_ruckig_ik_streamer
  cartesian_ruckig_streamer
  cartesian_velocity_controller
  spherical_orbit_streamer
)
  install(PROGRAMS scripts/controllers/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── simulation/：MuJoCo 仿真 ──────────────────────────────────────────────────
foreach(script
  mujoco_node
  mujoco_data_recorder
)
  install(PROGRAMS scripts/simulation/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── vision/：视觉感知 ─────────────────────────────────────────────────────────
foreach(script
  ibvs_control_node
  red_box_detector
)
  install(PROGRAMS scripts/vision/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── tools/：基础设施 & 调试 ───────────────────────────────────────────────────
foreach(script
  arm_trajectory_bridge
  pose_command_debug
)
  install(PROGRAMS scripts/tools/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── 测试工具脚本 ──────────────────────────────────────────────────────────────
install(PROGRAMS tests/pose_command_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_command_publisher
)

# ── 资源安装 ──────────────────────────────────────────────────────────────────
install(DIRECTORY scripts/vision/IBVS_Controller
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
