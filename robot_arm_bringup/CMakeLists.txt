cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# launch_common 工厂模块（robot_arm_bringup.launch_common），三后端 launch 共用
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})

ament_package()
