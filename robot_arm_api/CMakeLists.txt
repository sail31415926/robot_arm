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

# 单元测试（pytest）：reach_check 的正解 / 逆解 / 可达判定 / 余量 / 步骤表预检 / 能力卡。
# 夹具用 xacro 现场展开 arm.urdf.xacro（含云台 V2），需要 robot_arm_description /
# robot_gimbal_description_v2 已安装；缺失时整组 skip。参考正解用 pinocchio（venv 里有），缺失时 skip 对拍项。
#   colcon test --packages-select robot_arm_api && colcon test-result --verbose
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_reach_check test/test_reach_check.py
    TIMEOUT 300)
  ament_add_pytest_test(test_llm_shot_loop test/test_llm_shot_loop.py
    TIMEOUT 120)
endif()

ament_package()
