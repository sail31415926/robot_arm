# ArmCommand.msg
# 机械臂控制指令，发送给运动执行器（Motion Executor）
# 发布方：Director（通过 OutputCallbacks::on_motion_cmd 封装在 MotionCommand 中）
# 订阅方：Motion Executor → 机械臂驱动层

# ── 控制模式常量 ──────────────────────────────────────────────
uint8 POSITION = 0  # 位置模式：以平滑曲线运动到目标高度 height_m
uint8 VELOCITY = 1  # 速度模式：以 velocity_mps 恒速运动直到外部停止
uint8 FREEZE   = 2  # 冻结模式：保持当前位置，忽略 height_m/velocity_mps

# ── 字段 ──────────────────────────────────────────────────────
# 控制模式，取值为上方常量之一
uint8   mode

# 目标高度，单位：米，有效范围 [0.3, 1.2]
# 仅 mode == POSITION 时有效；超出范围时驱动层自动钳位
float32 height_m

# 运动速度，单位：米/秒，有效范围 [0.0, 0.2]
# 仅 mode == VELOCITY 时有效；正值向上，负值向下
float32 velocity_mps
