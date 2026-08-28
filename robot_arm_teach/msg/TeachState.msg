# TeachState.msg
# 示教状态机的当前状态 —— latched 广播于 /robot_arm/teach/state。
#
# ★ 这是**上层应用状态**，刻意不做进 robot_arm_interfaces/ControlMode.msg。
#   ControlMode（TRAJECTORY / JOINT_VELOCITY / JOINT_EFFORT / ADMITTANCE）说的是
#   「关节现在被哪条通路驱动」，是底层控制方式；示教说的是「用户正在录/在放」，
#   是应用阶段。两者正交：RECORDING 时底层是 JOINT_VELOCITY，PLAYING 时底层是
#   TRAJECTORY，同一个 ControlMode 也可以对应 IDLE（别的应用在用速度总线）。
#   混成一个枚举会让 ModeManager 需要知道「示教」这个业务概念，正是要避免的耦合。
#   underlying_control_mode 只是把当前底层模式**镜像**过来方便调试，不是本状态机的一部分。

std_msgs/Header header

# ── 状态常量 ────────────────────────────────────────────────────────────────
uint8 IDLE          = 0   # 空闲：既没在录也没在放
uint8 RECORDING     = 1   # 正在录制（采样器在跑，点动中继放行）
uint8 RECORD_PAUSED = 2   # 录制暂停（采样器停、点动中继关闸，时间轴冻结）
uint8 PLAYING       = 3   # 正在回放
uint8 PLAY_PAUSED   = 4   # 回放暂停（已下发保持轨迹，机械臂停在当前位置）

uint8  state

# ── 示教方式常量 ────────────────────────────────────────────────────────────
uint8 TEACH_MODE_JOG  = 0   # 点动示教：经 /robot_arm/cmd/joint_velocity 摇/按着走（当前唯一实机可用）
uint8 TEACH_MODE_DRAG = 1   # 手拖示教：需 JOINT_EFFORT/ADMITTANCE，实机未具备，见 doc/实机能力限制说明.md

uint8  teach_mode

# ── 运行期信息 ──────────────────────────────────────────────────────────────
string trajectory_name          # 当前正在录/放的轨迹名（IDLE 时为空）
MotionType motion_type          # 当前录制指定的运镜类型
uint32 point_count              # 已录制/已加载的采样点数
float64 elapsed_sec             # 录制已用时长（暂停期间不增长）；回放期为已播放时长
bool   jog_relay_enabled        # 点动中继当前是否放行（仅 RECORDING 且未暂停时为 true）

# 当前底层控制模式的镜像（取 ControlMode 常量），只读、仅供调试对照
robot_arm_interfaces/ControlMode underlying_control_mode

string message                  # 人类可读的补充说明（上次操作结果 / 降级原因等）
