cmake_minimum_required(VERSION 3.16)
project(robot_arm_teach)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)          # /check_state_validity（自碰撞闸）
find_package(urdf REQUIRED)                 # joint_limits_guard：解析 /robot_description
find_package(robot_arm_interfaces REQUIRED) # ControlMode / ArmStatus / ArmJointVelocityCommand
find_package(yaml-cpp REQUIRED)             # trajectory_store：轨迹落盘

# ══════════════════════════════════════════════════════════════════════════════
# 接口定义就放在本包内（而不是塞进 robot_arm_interfaces）
#   理由：requirement「新功能尽量只放在 robot_arm_teach 包中」。示教的消息不是产品总线的
#   一部分，混进 robot_arm_interfaces 会让每个用臂的包都被迫重编。代价是本包既生成接口
#   又编库，所以下面必须用 rosidl_get_typesupport_target 手动链 —— 同包内
#   ament_target_dependencies(... robot_arm_teach) 是拿不到自己的。
#
#   ★ 刻意**没有**动 robot_arm_interfaces/msg/ControlMode.msg：示教是上层应用状态，
#     不是底层控制方式，不该出现在 ControlMode 的枚举里（见 msg/TeachState.msg 注释）。
#     本包只是把 ControlMode 当**字段类型**引用，是纯粹的单向依赖。
# ══════════════════════════════════════════════════════════════════════════════
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionType.msg"
  "msg/MotionMeta.msg"
  "msg/TeachPoint.msg"
  "msg/TeachTrajectory.msg"
  "msg/TeachState.msg"
  "msg/PlaybackState.msg"
  "msg/JogCommand.msg"
  "srv/StartTeach.srv"
  "srv/StopTeach.srv"
  "srv/SaveTrajectory.srv"
  "srv/LoadTrajectory.srv"
  "srv/PlayTrajectory.srv"
  "srv/ListTrajectories.srv"
  "srv/DeleteTrajectory.srv"
  "srv/SetPlaybackSpeed.srv"
  DEPENDENCIES std_msgs robot_arm_interfaces
)
rosidl_get_typesupport_target(teach_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ══════════════════════════════════════════════════════════════════════════════
# 第一层：core —— 纯逻辑，**不依赖 rclcpp**
#   校验 / 采样压缩 / 分段规划 / YAML 持久化。不碰节点、不碰话题，因此可以在 CI 里
#   直接 gtest（见 test/）。回放前置校验写错的后果是让坏轨迹驱动实物机械臂，
#   这一层必须能测到。
# ══════════════════════════════════════════════════════════════════════════════
add_library(robot_arm_teach_core SHARED
  src/teach_types.cpp
  src/teach_validator.cpp
  src/teach_recorder.cpp
  src/playback_planner.cpp
  src/trajectory_store.cpp
)
target_include_directories(robot_arm_teach_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_teach_core PUBLIC cxx_std_17)
ament_target_dependencies(robot_arm_teach_core
  trajectory_msgs builtin_interfaces std_msgs robot_arm_interfaces)
target_link_libraries(robot_arm_teach_core ${teach_typesupport} ${YAML_CPP_LIBRARIES})
target_include_directories(robot_arm_teach_core PUBLIC ${YAML_CPP_INCLUDE_DIRS})

install(TARGETS robot_arm_teach_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ══════════════════════════════════════════════════════════════════════════════
# 第二层：node —— ROS 接线
#   订阅 /joint_states、切模式、发轨迹总线、13 个服务。所有对既有工程的耦合都在这一层，
#   而且全部是「用既有接口」：不碰 controller_manager、不碰 CAN、不改 ModeManager。
# ══════════════════════════════════════════════════════════════════════════════
add_library(robot_arm_teach_node SHARED
  src/joint_limits_guard.cpp
  src/teach_node.cpp
  src/teach_node_services.cpp
)
target_include_directories(robot_arm_teach_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_teach_node PUBLIC cxx_std_17)
ament_target_dependencies(robot_arm_teach_node
  rclcpp std_msgs std_srvs sensor_msgs trajectory_msgs builtin_interfaces
  moveit_msgs urdf robot_arm_interfaces)
target_link_libraries(robot_arm_teach_node robot_arm_teach_core ${teach_typesupport})

install(TARGETS robot_arm_teach_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(arm_teach_node src/teach_main.cpp)
target_compile_features(arm_teach_node PRIVATE cxx_std_17)
ament_target_dependencies(arm_teach_node rclcpp)
target_link_libraries(arm_teach_node robot_arm_teach_node)
install(TARGETS arm_teach_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config doc DESTINATION share/${PROJECT_NAME})

# ──────────────────────────────────────────────────────────────────────────────
# 示教面板（PyQt5）—— 纯客户端，只调本包的 13 个服务 + 发 JogCommand。
#   本包是 ament_cmake（要编 C++ 和生成接口），所以 Python 脚本用 install(PROGRAMS)
#   手动装，而不是 ament_python 的 entry_points。RENAME 去掉 .py 后缀，让
#   `ros2 run robot_arm_teach teach_gui` 与其它可执行文件的用法一致。
#   消息/服务的 Python 绑定由上面的 rosidl_generate_interfaces 一并生成，
#   无需额外声明就能 `from robot_arm_teach.srv import StartTeach`。
# ──────────────────────────────────────────────────────────────────────────────
install(PROGRAMS python/teach_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME teach_gui)

# ══════════════════════════════════════════════════════════════════════════════
# 单元测试（只测 core 层）
#   跑法：colcon build --packages-select robot_arm_teach
#         colcon test --packages-select robot_arm_teach && colcon test-result --verbose
# ══════════════════════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_teach_validator test/test_teach_validator.cpp)
  target_link_libraries(test_teach_validator robot_arm_teach_core ${teach_typesupport})

  ament_add_gtest(test_teach_recorder test/test_teach_recorder.cpp)
  target_link_libraries(test_teach_recorder robot_arm_teach_core ${teach_typesupport})

  ament_add_gtest(test_playback_planner test/test_playback_planner.cpp)
  target_link_libraries(test_playback_planner robot_arm_teach_core ${teach_typesupport})

  ament_add_gtest(test_trajectory_store test/test_trajectory_store.cpp)
  target_link_libraries(test_trajectory_store robot_arm_teach_core ${teach_typesupport})
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_arm_teach_core robot_arm_teach_node)
ament_export_dependencies(rosidl_default_runtime rclcpp trajectory_msgs robot_arm_interfaces)

ament_package()
