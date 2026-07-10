cmake_minimum_required(VERSION 3.8)
project(robot_arm_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch resources DESTINATION share/${PROJECT_NAME})

# scripts/：MuJoCo 仿真桥（Python，rename 为无扩展名可执行，ros2 run 调起）
foreach(script
  mujoco_node
  mujoco_data_recorder
)
  install(PROGRAMS scripts/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

ament_package()
