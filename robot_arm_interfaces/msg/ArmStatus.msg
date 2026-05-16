# ArmStatus.msg
# 机械臂实时状态，由 Motion Executor 上报

# 机械臂当前高度，单位：米（编码器反馈）
float32 arm_height_m

# 机械臂是否已到达目标高度（|实际 - 目标| < 容差）
bool    arm_at_target
