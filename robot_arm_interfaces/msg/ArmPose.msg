# ArmPose.msg
# 机械臂末端位姿（基于机械臂底座坐标系 base_link），公共结构体，被多个接口复用
float32 x        # 单位：米
float32 y        # 单位：米
float32 z        # 单位：米
float32 roll     # 横滚角（Roll），单位：度，范围 [-180, 180]
float32 pitch    # 俯仰角（Pitch），单位：度，范围 [-90, 90]，正值抬头
float32 yaw      # 水平角（Pan/Yaw），单位：度，范围 [-180, 180]，正值向右转
