# ArmFollowCommand.msg
# 机械臂末端速度跟随指令（基于机械臂底座坐标系）
# 发布方：Director
# 订阅方：Motion Executor
# 线速度有效范围 [-0.2, 0.2] m/s；不使用的轴填 0

# 末端目标速度
ArmTwist twist