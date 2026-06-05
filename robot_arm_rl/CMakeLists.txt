cmake_minimum_required(VERSION 3.8)
project(robot_arm_rl)

find_package(ament_cmake REQUIRED)

# ── Python 脚本安装 ───────────────────────────────────────────────────────────
install(PROGRAMS
  scripts/train_sac.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME train_sac
)

install(PROGRAMS
  scripts/deploy/rl_policy_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rl_policy_node
)

# ── 共享资源（模型权重、配置）─────────────────────────────────────────────────
install(DIRECTORY scripts/envs
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY scripts/deploy
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
