# PoseCommand.msg
# 机械臂控制指令，发送给机械臂执行器
# 发布方：Director
# 订阅方：机械臂驱动层

# 目标姿态
uint8 POSE_STATE_STOWED   = 0     # 收纳位 SHUTDOWN
uint8 POSE_STATE_OBSERVE  = 1     # 观察位 IDLE/POSITION
uint8 POSE_STATE_SHOOTING = 2     # 拍摄位 COMPOSING/RECORDING (目标高度，关节角由逆运动学确定)
uint8 target_pose_state # 目标姿态

# 转换速度
uint8 SPEED_SLOW    = 0
uint8 SPEED_NORMAL  = 1
uint8 SPEED_FAST    = 2
uint8 transition_speed  # 转换速度

# POSE_STATE_SHOOTING 时的目标坐标，在Director的配置参数中配置好低位、中位、高位的坐标
float32 target_pose_x    # 基于机械臂底座的坐标
float32 target_pose_y    # 基于机械臂底座的坐标
float32 target_pose_z    # 基于机械臂底座的坐标

# 云台水平角（Pan），单位：度，范围 [-180, 180]，正值向右转
float32 pan_deg
# 云台俯仰角（Tilt），单位：度，范围 [-90, 45]，正值向上仰
float32 tilt_deg