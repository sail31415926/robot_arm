cmake_minimum_required(VERSION 3.8)
project(emeet_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV   REQUIRED libudev)
pkg_check_modules(HIDAPI REQUIRED hidapi-libusb)

# ── Message generation ────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PtzState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
  include
  ${UDEV_INCLUDE_DIRS}
  ${HIDAPI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ── Shared HID + device discovery library ────────────────────────────────────
add_library(emeet_hid_common SHARED
  src/hid_interface.cpp
  src/device_discovery.cpp
)
target_include_directories(emeet_hid_common PUBLIC
  include
  ${UDEV_INCLUDE_DIRS}
  ${HIDAPI_INCLUDE_DIRS}
)
target_link_libraries(emeet_hid_common
  ${UDEV_LIBRARIES}
  ${HIDAPI_LIBRARIES}
)
ament_target_dependencies(emeet_hid_common rclcpp)

# ── ros2_control hardware plugin ─────────────────────────────────────────────
add_library(emeet_camera_hardware_interface SHARED
  src/camera_hardware_interface.cpp
)
target_include_directories(emeet_camera_hardware_interface PUBLIC include)
target_link_libraries(emeet_camera_hardware_interface emeet_hid_common)
ament_target_dependencies(emeet_camera_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(
  hardware_interface emeet_camera_hardware_plugin.xml)

# ── Camera driver node ────────────────────────────────────────────────────────
add_executable(camera_driver_node
  src/camera_driver_node.cpp
  src/v4l2_device.cpp
)
target_link_libraries(camera_driver_node
  emeet_hid_common
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(camera_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  rcl_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(camera_driver_node "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  emeet_hid_common
  emeet_camera_hardware_interface
  camera_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES emeet_camera_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config test docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/build.sh
  scripts/setup_local_env.sh
  test/camera_test_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
