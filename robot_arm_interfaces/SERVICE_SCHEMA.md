# robot_arm_interfaces — 消息结构文档

## 通信拓扑

```
Director ──topic(/robot_arm/pose_cmd)────► ArmNode  ← 目标姿态（收纳/观察/拍摄）
Director ──topic(/robot_arm/motion_cmd)──► ArmNode  ← 运镜控制（抬升/推拉/横移/环绕）
Director ──topic(/robot_arm/follow_cmd)──► ArmNode  ← 跟随速度控制（三轴速度）
Director ──topic(/robot_arm/arm_cmd)─────► ArmNode  ← 基础高度控制（位置/速度/冻结）
ArmNode  ──topic(/robot_arm/arm_status)──► Director ← 实时状态反馈（高度 + 到位标志）
```

---

## ArmPoseCommand.msg（话题 `/robot_arm/pose_cmd`）

```
ArmPoseCommand
│
├── target_pose_state: uint8        目标姿态，取值为以下常量之一
│   ├── POSE_STATE_STOWED   = 0    收纳位（关机/停机时使用）
│   ├── POSE_STATE_OBSERVE  = 1    观察位（待机/定位状态）
│   └── POSE_STATE_SHOOTING = 2    拍摄位（构图/录制，目标坐标由下方字段指定）
│
├── transition_speed: uint8         姿态切换速度，取值为以下常量之一
│   ├── SPEED_SLOW   = 0           慢速
│   ├── SPEED_NORMAL = 1           正常
│   └── SPEED_FAST   = 2           快速
│
├── target_pose_x: float32          拍摄位目标坐标 x（基于机械臂底座坐标系，米）
│   │                               仅 target_pose_state == SHOOTING 时有效
├── target_pose_y: float32          拍摄位目标坐标 y（基于机械臂底座坐标系，米）
├── target_pose_z: float32          拍摄位目标坐标 z（基于机械臂底座坐标系，米）
│   │                               低/中/高位预设坐标在 Director 配置参数中定义
│
├── pan_deg: float32                云台水平角（Pan），单位：度，范围 [-180, 180]
│   │                               正值向右转
└── tilt_deg: float32               云台俯仰角（Tilt），单位：度，范围 [-90, 45]
                                    正值向上仰
```

---

## ArmMotionCommand.msg（话题 `/robot_arm/motion_cmd`）

```
ArmMotionCommand
│
├── motion_type: uint8              运镜类型，取值为以下常量之一
│   ├── MODE_CRANE = 0             抬升/下降运镜
│   ├── MODE_DOLLY = 1             推/拉运镜
│   ├── MODE_TRUCK = 2             横移运镜
│   └── MODE_ARC   = 3             环绕运镜
│
├── speed_profile: uint8            速度曲线
│   ├── 0 (SMOOTH)                 缓入缓出
│   └── 1 (LINEAR)                 匀速
│
├── move_distance_m: float32        移动距离（米）
│   │                               CRANE/DOLLY/TRUCK 模式有效
│   │                               >0 抬升 | >0 前推 | >0 右移
│
├── arc_radius_m: float32           环绕半径（米）
│   │                               MODE_ARC 时有效
└── arc_degree: float32             环绕角度（度）
                                    MODE_ARC 时有效
```

---

## ArmFollowCommand.msg（话题 `/robot_arm/follow_cmd`）

```
ArmFollowCommand
│
├── speed_x_mps: float32            X 轴速度（米/秒）
├── speed_y_mps: float32            Y 轴速度（米/秒）
└── speed_z_mps: float32            Z 轴速度（米/秒）
```

---

## ArmCommand.msg（话题 `/robot_arm/arm_cmd`）

```
ArmCommand
│
├── mode: uint8                     控制模式，取值为以下常量之一
│   ├── POSITION = 0               位置模式：平滑运动到目标高度 height_m
│   ├── VELOCITY = 1               速度模式：以 velocity_mps 恒速运动直到外部停止
│   └── FREEZE   = 2               冻结模式：保持当前位置，忽略 height_m/velocity_mps
│
├── height_m: float32               目标高度（米），有效范围 [0.3, 1.2]
│   │                               仅 mode == POSITION 时有效；超出范围驱动层自动钳位
└── velocity_mps: float32           运动速度（米/秒），有效范围 [0.0, 0.2]
                                    仅 mode == VELOCITY 时有效；正值向上，负值向下
```

---

## /robot_arm/arm_status 话题（ArmStatus.msg）

```
ArmStatus
│
├── arm_height_m: float32           机械臂当前高度（米，编码器反馈）
└── arm_at_target: bool             是否已到达目标位置
                                    |实际高度 - 目标高度| < 容差 时为 true
```