# MotionCommand.msg
# 9.2.2.3 章节
# 9.8.0 章节
# 机械臂运镜控制指令，发送给机械臂执行器
# 发布方：Director
# 订阅方：机械臂驱动层
uint8 MODE_CRANE = 0    # 抬升下降
uint8 MODE_DOLLY = 1    # 推拉
uint8 MODE_TRUCK = 2    # 横移
uint8 MODE_ARC = 3      # 环绕

# 运动类型
uint8 motion_type
uint8 speed_profile         # 选择速度曲线 0: SMOOTH，缓入缓出； 1: LINEAR 匀速
# 抬升下降、推拉、横移运镜参数，>0抬升，>0前推，>0右移
float32 move_distance_m     # 移动距离
# 环绕运镜参数
float32 arc_radius_m        # 环绕半径
float32 arc_degree          # 环绕多少度
