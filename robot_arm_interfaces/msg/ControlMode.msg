# ControlMode.msg
# 机械臂的「语义控制模式」——产品层对控制方式的统一抽象，后端无感。
# 由 mode_manager_node 维护，切换 = ros2_control 控制器切换 + 驱动器 CiA402 模式切换。
#
# 这一维度与「哪个后端（Gazebo/MuJoCo/实物）」「哪个应用」正交：
# 上层只表达「用哪种模式」，不关心底层怎么实现。
# 本消息既作 /robot_arm/control_mode 话题类型（latched 广播当前模式），
# 也作 SwitchControlMode 服务、ArmStatus.active_control_mode 字段的取值定义。

# ── 模式常量 ─────────────────────────────────────────────────────────────
uint8 TRAJECTORY     = 0   # 关节轨迹/位置：JTC → IP(7)，走 /arm_controller/joint_trajectory
                           #   用于点位切换、运镜、MoveIt 规划（默认模式，延迟不敏感）
uint8 JOINT_VELOCITY = 1   # 关节速度：JointGroupVelocityController → PV(3)
                           #   走 /robot_arm/cmd/joint_velocity（带看门狗）；点动/遥操/视觉伺服
uint8 JOINT_EFFORT   = 2   # 关节力矩：JointGroupEffortController → PT(4)【P3，需 URDF 加 effort 接口】
uint8 ADMITTANCE     = 3   # 导纳柔顺：末端力 → 位置微调，底层仍走位置总线【P3，需 F/T 传感器】

# 当前/目标模式（取上面常量之一）
uint8 mode
