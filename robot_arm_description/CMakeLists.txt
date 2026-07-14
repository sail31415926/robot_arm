cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY mujoco  DESTINATION share/${PROJECT_NAME})

ament_package()
