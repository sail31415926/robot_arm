# MotionType.msg
# 示教轨迹的运镜类型 —— 上层应用语义，与底层 ControlMode（TRAJECTORY/JOINT_VELOCITY/...）
# 完全正交：ControlMode 说的是「怎么驱动关节」，MotionType 说的是「这条轨迹是什么镜头」。
#
# 取值与 algo_llm_planner 的 Schema 运镜枚举同源（DOLLY/TRUCK/ARC 在那边是带方向后缀的
# 10 值 enum，如 DOLLY_IN / TRUCK_LEFT / ARC_LEFT），本消息只保留**类型**，方向与幅度
# 放在 MotionMeta 里 —— 因为示教录下来的是实测关节轨迹，方向已经体现在数据本身，
# 元数据只是给上层检索和复用时看的标签。
#
# FREEFORM 是缺省值：自由拖动/点动出来的轨迹不归属任何运镜类型。

uint8 FREEFORM = 0    # 自由示教，无运镜语义
uint8 DOLLY    = 1    # 推/拉（沿光轴前后）
uint8 TRUCK    = 2    # 横移（垂直光轴左右平移）
uint8 ARC      = 3    # 环绕（绕被摄主体做圆弧）
uint8 CRANE    = 4    # 升降（沿竖直方向升降）

uint8 value
