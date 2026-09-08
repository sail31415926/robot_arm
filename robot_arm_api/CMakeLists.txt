cmake_minimum_required(VERSION 3.8)
project(robot_arm_api)

# 机械臂对外接口的 Python 客户端库 + demo（与 robot_arm_bringup 一样：ament_cmake + ament_python_install_package）
#   robot_arm_api/arm_commander_client.py  库：ArmCommanderClient / GimbalV2Client / ArmApi / execute_plan
#                                          → 上层 `from robot_arm_api import ArmApi`
#   robot_arm_api/arm_commander_demo.py    命令行 demo → `ros2 run robot_arm_api arm_commander_demo.py`
#   examples/                              JSON 运镜步骤表示例
# 只依赖接口包（robot_arm_interfaces 必需；robot_gimbal_interfaces_v2 可选），板上单独编译即可。
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/arm_commander_demo.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY examples
  DESTINATION share/${PROJECT_NAME})

ament_package()
