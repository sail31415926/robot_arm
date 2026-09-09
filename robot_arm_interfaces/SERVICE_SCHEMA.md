# robot_arm_interfaces — 消息结构文档

> 结构定义的**唯一权威**是 `msg/ srv/ action/` 目录下的接口文件（含逐字段注释）；
> 本文档只做通信拓扑速查与公共结构体展开，接口语义与执行流程详见 [README.md](README.md)。

## 通信拓扑

```text
Director ──action(/robot_arm/move_to_pose)────► Commander   位姿切换（收纳/观察/拍摄）
Director ──action(/robot_arm/move_to_joint)───► Commander   关节空间点到点（只动臂 J1-3）
Director ──action(/robot_arm/trajectory_shot)─► Commander   运镜执行（直线/球面环绕）
Director ──action(/robot_arm/track_target)────► Commander   视觉跟随（IBVS 启停）
Director ──service(/robot_arm/stop)───────────► Commander   软件急停（MOVING→STOPPED）
Director ──service(/robot_arm/enable)─────────► Driver      伺服上电/下电（Commander 透传）
Director ──service(/robot_arm/homing)─────────► Driver      回零（Commander 透传）
Director ──service(/robot_arm/reset_error)────► Driver      清故障 + Commander 状态复位
Commander ──topic(/robot_arm/arm_status)──────► Director    实时状态广播（10Hz）
Director ──topic(/robot_arm/follow_command)───► Commander   末端 6 维 twist 流（笛卡尔速度控制）
Director ──topic(/robot_arm/cmd/joint_velocity)► Commander   关节速度流（J1-3）
Director ──service(/robot_arm/switch_control_mode)► ModeManager 切控制模式（速度控制前置）
ModeManager ─topic(/robot_arm/control_mode)───► 全体        当前控制模式（latched）
```

> 两条速度流最终和位置类动作**汇到同一条总线**：`follow_command` 先经 Commander 的
> 6×6 Jacobian DLS 换算成 6 个关节速度，和 Director 直发的 `cmd/joint_velocity` 一起
> 由 `VelocityStreamServer` 限幅 → 积分成角度 → 按 URDF 限位夹紧，以 50Hz 位置流发
> `/arm_controller/joint_trajectory`（与 MoveToPose / MoveToJoint / TrajectoryShot 同一个控制器）。
> 速度模式**不切控制器**，切模式只更新语义标志。

---

## 公共结构体

### ArmPose.msg（末端位姿，base_link 坐标系）

```text
ArmPose
│
├── x / y / z: float32              位置（米）
├── roll: float32                   横滚角，单位：度，范围 [-180, 180]
├── pitch: float32                  俯仰角，单位：度，范围 [-90, 90]，正值抬头
└── yaw: float32                    水平角，单位：度，范围 [-180, 180]，正值向右转
```

### ArmTwist.msg（末端速度，base_link 坐标系）

```text
ArmTwist
│
├── vx / vy / vz: float32           线速度（米/秒），正方向：前 / 左 / 上
└── wroll / wpitch / wyaw: float32  角速度（度/秒）
```

---

## ArmStatus.msg（话题 `/robot_arm/arm_status`，10Hz）

```text
ArmStatus
│
├── header: std_msgs/Header         时间戳 + frame_id（统一填 base_link）
│
├── current_pose_state: uint8       当前语义姿态
│   ├── POSE_STATE_STOWED   = 0    收纳位
│   ├── POSE_STATE_OBSERVE  = 1    观察位
│   └── POSE_STATE_SHOOTING = 2    拍摄位
│
├── error_code: uint8               错误码
│   ├── ERR_NONE    = 0            正常
│   ├── ERR_LIMIT   = 1            触发限位
│   ├── ERR_DRIVER  = 2            驱动层故障
│   └── ERR_TIMEOUT = 3            运动超时
│
├── executing_command_id: uint32    当前/最近命令 id（0 = 上层不关心）
├── command_result: uint8           NONE=0 / EXECUTING=1 / SUCCEEDED=2 / FAILED=3 / ABORTED=4
│
├── arm_pose: ArmPose               末端当前位姿
├── arm_twist: ArmTwist             末端当前速度
├── is_moving: bool                 是否正在运动
├── arm_at_target: bool             已到达目标（|实际-目标| < 容差）
├── arm_at_pose_start: bool         已到达运镜起始点（起点停顿期间为 true）
├── camera_ready: bool              摄像头录制就绪窗口（到达运镜起点起、至运镜结束）
├── is_tracking: bool               视觉伺服跟随中（ArmTrackTarget goal 活跃期间为 true）
├── tracking_img_err: float32       跟随中的图像误差（归一化欧氏距离，非跟随时 0.0）
└── tracking_depth_err_m: float32   跟随中的深度误差（米，非跟随时 0.0）
```

---

## ArmFollowCommand.msg（话题 `/robot_arm/follow_command`）

> 笛卡尔速度控制入口（2026-08-04 开通）。订阅方 = Arm Commander 的 CartesianVelocityServer。
> 前置：先切 `ControlMode.JOINT_VELOCITY`；须 50-100Hz 持续发布；停止 = 发一帧全 0。

```text
ArmFollowCommand
│
└── twist: ArmTwist                 末端目标速度（arm_base_link 系，完整 6 自由度）
                                    vx/vy/vz     线速度 m/s，主要由臂 J1-3 承担，默认上限 0.2
                                    wroll/wpitch/wyaw  角速度 **度/秒**（绕 base 系 X/Y/Z 的
                                                 角速度矢量，非 RPY 角速率），主要由云台 J4-6 承担
```

> 6 维一次解算：Commander 用 J1-6 的 6×6 几何 Jacobian 做 DLS 伪逆解出全部 6 个关节速度，
> 积分成角度后在**同一条轨迹**里下发，位置/姿态耦合由 Jacobian 精确补偿。

---

## ArmJointVelocityCommand.msg（话题 `/robot_arm/cmd/joint_velocity`）

> 关节速度控制入口（2026-08-04 开通）。订阅方 = Arm Commander 的 `VelocityStreamServer`
> （限幅 → 积分成角度 → 按 URDF 限位夹紧 → 50Hz 位置流走 `/arm_controller/joint_trajectory`）。
> 前置与发布节奏同 `ArmFollowCommand`。

```text
ArmJointVelocityCommand
│
└── velocities: float64[]           目标关节角速度 rad/s，顺序固定 Joint1/Joint2/Joint3，
                                    必须 3 个（长度不符整帧丢弃并告警）；
                                    云台 J4-6 不在此列，保持当前角度不动
```

---

## Action / Service 一览

| 接口文件 | 名称 | 要点 |
| --- | --- | --- |
| `action/ArmMoveToPose.action` | `/robot_arm/move_to_pose` | 姿态切换；exit_reason: reached / timeout / cancelled / stopped / unreachable / error |
| `action/ArmMoveToJoint.action` | `/robot_arm/move_to_joint` | 关节空间点到点（不过 IK，只动臂 J1-3、云台保持）；限位与自碰撞校验不过则不下发；exit_reason: reached / out_of_range / collision / invalid_goal / timeout / cancelled / stopped / error |
| `action/ArmTrajectoryShot.action` | `/robot_arm/trajectory_shot` | 直线 / 球面环绕运镜；exit_reason: reached / timeout / cancelled / stopped / error |
| `action/ArmTrackTarget.action` | `/robot_arm/track_target` | IBVS 跟随启停；exit_code: CONVERGED / FEATURE_LOST / TIMEOUT / CANCELLED / ERROR |
| `srv/ArmStop.srv` | `/robot_arm/stop` | 软件急停：停 action（MOVING→STOPPED）+ 停速度流（需 ArmResetError 复位后才能再动） |
| `srv/ArmEnable.srv` | `/robot_arm/enable` | 伺服上电 / 下电（透传驱动层） |
| `srv/ArmHoming.srv` | `/robot_arm/homing` | 回零（阻塞至到位或急停） |
| `srv/ArmResetError.srv` | `/robot_arm/reset_error` | 驱动层 recover + Commander ERROR/STOPPED→IDLE（动作超时不进 ERROR，不需要它） |

各 action 的 Goal / Result / Feedback 字段树见接口文件内注释与 [README.md](README.md) 的接口清单章节。
