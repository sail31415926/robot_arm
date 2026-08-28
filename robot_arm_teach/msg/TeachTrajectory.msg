# TeachTrajectory.msg
# 一条完整的示教轨迹 —— 内存与 YAML 落盘（doc/轨迹格式说明.md）共用同一套字段。
#
# ★ 记录的是**完整 6 轴关节轨迹**，不是末端位姿序列（理由见 TeachPoint.msg）。
#   DOLLY/TRUCK/ARC/CRANE 的参数只作 motion_meta 元数据保存，不参与回放执行。

# ── 格式版本 ────────────────────────────────────────────────────────────────
# 每次改动字段语义就 +1。加载时版本大于本常量直接拒绝（宁可报错也不要按旧语义误读
# 新文件 —— 轨迹是要驱动实物机械臂的）；小于则走兼容分支。
uint16 FORMAT_VERSION = 1
uint16 format_version

# ── 身份 ────────────────────────────────────────────────────────────────────
string name          # 轨迹名，同时是文件名（<name>.yaml），须满足 [A-Za-z0-9_-]{1,64}
string created_at    # 创建时间，ISO-8601 本地时间（如 2026-08-24T15:04:05）
string description   # 自由备注（可空）

# ── 关节 ────────────────────────────────────────────────────────────────────
# 必须 6 个，顺序与 points 里的数组一致；回放前逐个与 /joint_states 的名字比对，
# 不一致直接拒绝执行（不做"按位置猜"的重排——猜错就是机械臂走错轴）
string[] joint_names

# ── 轨迹数据 ────────────────────────────────────────────────────────────────
TeachPoint[] points

# ── 运镜语义（元数据）──────────────────────────────────────────────────────
MotionType motion_type
MotionMeta motion_meta

# ── 起点 / 终点（= points 首末点的位置，冗余存储便于免遍历检索与前置校验）────
float64[6] start_positions
float64[6] end_positions

# ── 运动学包络（录制结束时按实测数据统计，逐轴取绝对值峰值）──────────────
# 回放前用它与 URDF 限位/参数上限比对；降速时按 speed_scale 线性/平方缩放后复核。
float64[6] max_velocity        # rad/s
float64[6] max_acceleration    # rad/s^2
float64    duration_sec        # 轨迹总时长 = 末点 time_from_start

# ── 示教时的底层控制模式 ────────────────────────────────────────────────────
# 取 ControlMode 常量（点动示教 = JOINT_VELOCITY）。记录它是为了回放/复现时知道
# 这条轨迹是在什么控制通路下采到的 —— 速度模式录的轨迹带位置环滞后，
# 与力矩/导纳拖动录的手感不同，混用时排查问题要先看这个字段。
robot_arm_interfaces/ControlMode teach_control_mode
# 示教方式（取 TeachState.TEACH_MODE_* 常量）
uint8 teach_mode

# ── 采样信息 ────────────────────────────────────────────────────────────────
float64 sample_rate_hz         # 录制时的名义采样频率（50 / 100）
uint32  raw_point_count        # 压缩前的原始采样点数（points.size() 是压缩后的）
float64 compress_position_eps  # 当时用的位置阈值（rad），0 = 未压缩
