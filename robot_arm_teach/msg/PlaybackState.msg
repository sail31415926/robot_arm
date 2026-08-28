# PlaybackState.msg
# 回放进度反馈 —— 发布于 /robot_arm/teach/playback（默认 10Hz，仅回放期间发布）。

std_msgs/Header header

# ── 回放状态常量 ────────────────────────────────────────────────────────────
uint8 PLAYING  = 0
uint8 PAUSED   = 1
uint8 FINISHED = 2   # 正常播完
uint8 STOPPED  = 3   # 被 stop_playback 停止
uint8 ABORTED  = 4   # 被急停/故障中断

uint8  status

string  trajectory_name
uint32  point_index        # 当前已越过的采样点序号
uint32  point_total        # 总采样点数
float64 elapsed_sec        # 轨迹时间轴上的已播放时长（受 speed_scale 影响后的实际墙钟折算值）
float64 total_sec          # 轨迹时间轴总时长（原始，未按 speed_scale 缩放）
float32 progress_percent   # [0.0, 100.0]
float64 speed_scale        # 当前速度倍率（1.0 = 原速；<1 降速）
string  message
