# ArmTwist.msg
# 机械臂末端速度（基于机械臂底座坐标系 base_link），公共结构体，被多个接口复用
float32 vx       # 线速度，单位：米/秒，正值向前
float32 vy       # 线速度，单位：米/秒，正值向左
float32 vz       # 线速度，单位：米/秒，正值向上
float32 wroll    # 绕 Roll 轴角速度，单位：度/秒
float32 wpitch   # 绕 Pitch 轴角速度，单位：度/秒
float32 wyaw     # 绕 Yaw 轴角速度，单位：度/秒
