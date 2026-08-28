# MotionMeta.msg
# 运镜元数据 —— 只是**标签**，不是执行依据。
#
# ★ 关键约定：回放执行的永远是 TeachTrajectory.points 里录下的完整 6 轴关节轨迹，
#   本消息的任何字段都不参与轨迹生成，也不会在回放时被重新解算。
#   把参数存下来是为了：① 上层按运镜类型检索/挑选轨迹；② 人看得懂这条轨迹当初是怎么示教的；
#   ③ 将来若要做「参数化重生成」有原始意图可查。
#   所以字段全空也不影响回放 —— 与 MotionType 不匹配同样只是元数据不准，不会让机械臂走错。
#
# 各段按 TeachTrajectory.motion_type 取用；无关段留默认值即可。

# ── ARC 旋转方向 ────────────────────────────────────────────────────────────
uint8 ARC_CW  = 0    # 俯视顺时针
uint8 ARC_CCW = 1    # 俯视逆时针
# ── CRANE 升降方向 ──────────────────────────────────────────────────────────
uint8 CRANE_UP   = 0
uint8 CRANE_DOWN = 1
# ── TRUCK 目标跟随策略 ──────────────────────────────────────────────────────
uint8 FOLLOW_NONE         = 0    # 不跟随，纯平移（相机朝向随臂位形自然变化）
uint8 FOLLOW_LOCK_SUBJECT = 1    # 横移中始终把主体锁在画面中心
uint8 FOLLOW_PARALLEL     = 2    # 保持相机光轴方向不变，做纯平行横移

# ── 通用 ────────────────────────────────────────────────────────────────────
# 被摄主体的标签/坐标系名（可空）。仅作记录，示教时若没有主体就留空字符串。
string subject_label

# ── DOLLY（推/拉）──────────────────────────────────────────────────────────
float64[3] dolly_direction          # base_link 系单位向量；示教时的推进方向
float64    dolly_distance_m         # 推进总距离（m），拉远为负
bool       dolly_keep_camera_facing # 全程是否保持相机朝向主体

# ── TRUCK（横移）───────────────────────────────────────────────────────────
float64[3] truck_direction          # base_link 系单位向量；横移方向
float64    truck_distance_m         # 横移总距离（m），反向为负
uint8      truck_follow_policy      # 取 FOLLOW_* 常量

# ── ARC（环绕）─────────────────────────────────────────────────────────────
float64[3] arc_target_point         # 环绕目标点 / 球心（base_link 系，m）
float64    arc_radius_m             # 环绕半径（m）
float64    arc_start_angle_deg      # 起始方位角（deg），定义与 ArmTrajectoryShot 的 azimuth 一致
float64    arc_end_angle_deg        # 终止方位角（deg）
uint8      arc_rotation_direction   # 取 ARC_* 常量

# ── CRANE（升降）───────────────────────────────────────────────────────────
float64    crane_height_delta_m     # 末端高度变化量（m），带符号
uint8      crane_direction          # 取 CRANE_* 常量（与 delta 符号冗余，便于检索）
