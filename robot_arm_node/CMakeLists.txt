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
install(PROGRAMS scripts/tools/arm_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── gui/：调试用 GUI（rename 为无扩展名可执行，ros2 run 调起）─────────────────
foreach(script
  joint_position_gui
  cartesian_moveit_gui
  cartesian_realtime_ik_gui
  cartesian_trajectory_gui
  cartesian_velocity_gui
  spherical_orbit_gui
)
  install(PROGRAMS scripts/gui/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── controllers/：独立控制 node（装成可 import 的 .py 模块，供 gui/ 与 arm_commander 复用）
#    也可直接 headless 运行：ros2 run robot_arm_node <name>.py
install(PROGRAMS
  scripts/controllers/joint_position_controller_node.py
  scripts/controllers/cartesian_moveit_controller_node.py
  scripts/controllers/cartesian_realtime_ik_controller_node.py
  scripts/controllers/cartesian_velocity_controller_node.py
  scripts/controllers/cartesian_trajectory_controller_node.py
  scripts/controllers/spherical_orbit_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── commander/：Arm Commander 中间层（可 headless 运行或被 GUI import）─────────
install(PROGRAMS scripts/commander/arm_commander_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_commander_node
)
install(PROGRAMS
  scripts/commander/__init__.py
  scripts/commander/move_to_pose_server.py
  scripts/commander/trajectory_shot_server.py
  scripts/commander/motion_executor.py
  scripts/commander/status_aggregator.py
  DESTINATION lib/${PROJECT_NAME}/commander
)

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
)
  install(PROGRAMS scripts/tools/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# ── 测试工具脚本 ──────────────────────────────────────────────────────────────
install(PROGRAMS tests/commander_test_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME commander_test_gui
)

# ── 资源安装 ──────────────────────────────────────────────────────────────────
install(DIRECTORY scripts/vision/IBVS_Controller
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
