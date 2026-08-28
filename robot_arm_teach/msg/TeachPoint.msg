# TeachPoint.msg
# 示教轨迹的单个采样点 —— 完整 6 轴关节位置 + 速度 + 单调递增时间戳。
#
# ★ 为什么不存末端位姿：末端 gimbal_tool0 在云台 J4-6 之后（见 robot_arm/CLAUDE.md），
#   位姿 = f(J1..J6)。存位姿回放时要重新 IK，一旦云台回读有静差就复现不出原轨迹；
#   而关节轨迹可以原样喂给 arm_controller（JTC），是无损的。
#   要看末端位姿用 FK 现算，不在这里冗余存储。

float64[6] positions        # rad，顺序固定 Joint1..Joint6
float64[6] velocities       # rad/s，同序；/joint_states 无速度字段时由相邻位置差分补齐
float64    time_from_start  # s，相对轨迹起点，**严格单调递增**（校验器会拒绝非单调轨迹）
