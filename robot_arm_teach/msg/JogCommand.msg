# JogCommand.msg
# 点动示教的输入 —— 发布于 /robot_arm/teach/jog，由本包的点动中继转发到产品总线
# /robot_arm/cmd/joint_velocity（类型 ArmJointVelocityCommand）。
#
# ★ 为什么不让上层直接发产品总线：中继在转发前做三件示教特有的事 ——
#   ① 状态闸：只在 RECORDING 且未暂停时放行（暂停即停止转发，机械臂原地停住）
#   ② 示教限速：夹到 jog.max_joint_speed（比 velocity_stream.max_joint_speed 更保守）
#   ③ 失效保护看门狗：jog.command_timeout 内没有新指令就停止转发
#   直接发产品总线也能动（VelocityStreamServer 有自己的限幅和断流看门狗），录制照样
#   记得到 —— 只是绕过了上面三道示教侧的闸。中继可用 jog.enable_relay:=false 关掉。
#
# ⚠️ 只有 3 个值：产品总线 ArmJointVelocityCommand 固定 3 轴（Joint1-3），
#    长度不符会被 VelocityStreamServer 直接丢弃。云台 J4-6 **无法点动**，
#    示教期间它们保持当前角度，只是被原样录进 TeachPoint。详见 doc/实机能力限制说明.md。

float64[3] velocities   # 目标关节角速度（rad/s），顺序 Joint1 / Joint2 / Joint3
