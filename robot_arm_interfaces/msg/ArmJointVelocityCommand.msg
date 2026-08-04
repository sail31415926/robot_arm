# ArmJointVelocityCommand.msg
# 关节速度指令 —— 产品总线 /robot_arm/cmd/joint_velocity
# 发布方：Director（点动/遥操）或 Arm Commander（笛卡尔速度经 Jacobian 换算后也走这条总线）
# 订阅方：Arm Commander 的 VelocityStreamServer（限幅 + 积分成角度 + 按 URDF 限位夹紧，
#         以 50Hz 位置流发 /arm_controller/joint_trajectory —— 与位置模式同一个控制器）
#
# 【前提】必须先调 /robot_arm/switch_control_mode 切到 ControlMode.JOINT_VELOCITY，
#         否则 Commander 忽略本消息并告警（模式闸，防止误发的速度指令让机械臂动起来）。
# 【断流】超过 velocity_stream.command_timeout（默认 0.3s）没有新指令即停流，JTC 保持最后一点
#         （原地停住）。故持续运动必须周期发布（建议 50-100Hz）。
#         停止可以直接停发布，也可以发一帧全 0 —— 两者都是原地停住。
# 【安全】逐轴限幅到 velocity_stream.max_joint_speed；积分出的角度按 URDF 关节限位夹紧
#         （到限位自然停住，反向撤离不受限）。

# 目标关节角速度（rad/s），顺序固定为 Joint1 / Joint2 / Joint3，必须 3 个。
# 正方向与 URDF 关节轴一致。云台 J4-6 不在此列 —— 本消息只驱动臂，云台保持当前角度不动
#（要连姿态一起动，用 ArmFollowCommand 的 6 维 twist）。
float64[] velocities
