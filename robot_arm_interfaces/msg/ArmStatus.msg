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

# ── 当前控制模式（取值见 ControlMode 常量）─────────────────────────────
# 0=TRAJECTORY 1=JOINT_VELOCITY 2=JOINT_EFFORT 3=ADMITTANCE
# 由 mode_manager_node 经 /robot_arm/control_mode 广播，Commander 汇入本字段
uint8   active_control_mode

# 机械臂是否正在运动
bool    is_moving

# 机械臂是否已到达目标（|实际 - 目标| < 容差），到达 = true
bool    arm_at_target

# 机械臂是否已到达运镜起始点（直线/环绕运镜步骤1 到位 = true；
# 每条新指令开始时复位为 false）
bool    arm_at_pose_start

# 摄像头录制就绪：给定轨迹后机械臂到达起始点起、至运镜结束期间为 true，其余为 false
bool    camera_ready

# ── 目标跟随状态（IBVS 运行期间有效）─────────────────────────
# 是否正在执行视觉伺服跟随（ArmTrackTarget goal 活跃期间为 true）
bool    is_tracking
# 当前图像误差（归一化欧氏距离）；非跟随状态下为 0.0
float32 tracking_img_err
# 当前深度误差（m）；非跟随状态下为 0.0
float32 tracking_depth_err_m
