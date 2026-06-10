# ArmStatus.msg
# 机械臂实时状态，由 Arm Commander 上报
# 发布方：Arm Commander → 上层（Director）

# 上报时间戳与坐标系（frame_id 统一填 base_link）
std_msgs/Header header

# ── 当前状态常量（与 ArmMoveToPose.Goal.POSE_STATE_* 对应）─────────
uint8 POSE_STATE_STOWED   = 0   # 收纳位
uint8 POSE_STATE_OBSERVE  = 1   # 观察位
uint8 POSE_STATE_SHOOTING = 2   # 拍摄位
# 当前所处姿态状态
uint8   current_pose_state

# ── 错误码常量 ────────────────────────────────────────────────
uint8 ERR_NONE        = 0   # 正常
uint8 ERR_LIMIT       = 1   # 触发限位
uint8 ERR_DRIVER      = 2   # 驱动层故障
uint8 ERR_TIMEOUT     = 3   # 运动超时
# 当前错误码
uint8   error_code

# ── 命令执行结果（与命令消息的 command_id 关联）────────────────
uint8 RESULT_NONE      = 0   # 空闲，无命令
uint8 RESULT_EXECUTING = 1   # 执行中
uint8 RESULT_SUCCEEDED = 2   # 成功完成
uint8 RESULT_FAILED    = 3   # 失败（详见 error_code）
uint8 RESULT_ABORTED   = 4   # 被急停 / 取消
# 当前或最近一条命令的 id（对应命令消息中的 command_id）
uint32  executing_command_id
# 该命令的执行结果
uint8   command_result

# ── 实时反馈 ──────────────────────────────────────────────────
# 末端当前位姿
ArmPose  arm_pose

# 末端当前速度
ArmTwist arm_twist

# 机械臂是否正在运动
bool    is_moving

# 机械臂是否已到达目标（|实际 - 目标| < 容差），到达 = true
bool    arm_at_target
