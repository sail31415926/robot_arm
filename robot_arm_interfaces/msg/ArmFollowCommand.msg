# ArmFollowCommand.msg
# 机械臂末端速度跟随指令（笛卡尔速度控制，基于机械臂底座坐标系 arm_base_link）
# 发布方：Director
# 订阅方：Arm Commander（VelocityStreamServer）
#
# 【实现路径】Commander 用 **J1-6 的 6×6 几何 Jacobian** 做 DLS 伪逆，把 6 维 twist 一次解算成
#             全部关节速度，再积分成角度，以 50Hz 位置流发 /arm_controller/joint_trajectory
#             （与位置模式**同一个控制器**，不切控制器）。位置与姿态的耦合由 Jacobian 统一处理。
# 【前提】必须先调 /robot_arm/switch_control_mode 切到 ControlMode.JOINT_VELOCITY，
#         否则 Commander 忽略本消息并告警。
# 【断流】超过 velocity_stream.command_timeout（默认 0.3s）没有新指令即停流，JTC 保持最后一点
#         （原地停住）。持续运动须周期发布（建议 50-100Hz）；停止可直接停发布，也可发一帧全 0。
# 【限幅】线速度逐轴限幅 velocity_stream.max_linear_speed（默认 0.2 m/s）、角速度逐轴限幅
#         max_angular_speed（默认 1.0 rad/s ≈ 57°/s）；换算出的 6 个关节速度只要有一个超限，
#         就整体等比缩放，保持末端运动方向不变。积分出的角度按 URDF 关节限位夹紧。

# 末端目标速度（完整 6 自由度）。
#   线速度 vx/vy/vz：m/s，由臂 J1-3 承担
#   角速度 wroll/wpitch/wyaw：**度/秒**，绕 arm_base_link 的 X/Y/Z 轴，主要由云台 J4-6 承担
#     （不是 RPY 角速率，是角速度矢量在 base 系的三个分量）
#   两者不是分开算的：6×6 Jacobian 一次解出 6 个关节速度，位置/姿态耦合自动补偿
# 注意：云台接近万向锁位形（J5 贴近 ±90°）或某轴压到限位时，姿态方向会退化 ——
#       DLS 会给出可行的近似解，但跟踪精度下降；必要时先用 ArmMoveToPose 把云台摆回中位。
ArmTwist twist
