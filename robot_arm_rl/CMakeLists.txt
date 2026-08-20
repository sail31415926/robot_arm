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

install(PROGRAMS
  scripts/eval_policy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME eval_policy
)

install(PROGRAMS
  scripts/sim_to_sim_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_to_sim_test
)

install(PROGRAMS
  scripts/deploy/subject_mover.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME subject_mover
)

# ── 共享资源（模型权重、配置）─────────────────────────────────────────────────
install(DIRECTORY scripts/envs
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY scripts/deploy
  DESTINATION lib/${PROJECT_NAME}
)
# 模型权重只装 best/ 与归一化统计量；回放池(300MB)/checkpoint/归档不进 install
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "replay_buffer.pkl" EXCLUDE
  PATTERN "*_steps.zip" EXCLUDE
  PATTERN "archive_*" EXCLUDE
  PATTERN "snapshot_*" EXCLUDE
)

install(DIRECTORY launch sim
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
