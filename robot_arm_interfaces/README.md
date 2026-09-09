# robot_arm_interfaces

机械臂**上层（Director）↔ 中间层（Arm Commander）** 的语义级接口定义。
不面向硬件驱动层 —— 驱动层使用标准消息（`JointState` / `JointTrajectory` /
`Float64` / `DiagnosticStatus`），与本包解耦。

## 分层数据流

三个角色：

- **Director（上层）** —— 决策、语义意图（"去拍摄位""做个环绕"）
- **Arm Commander（中间层，`arm_commander` 节点 / `robot_arm_node` 包）** —— Cartesian 规划 / IK / 轨迹生成 / 状态聚合
- **Driver（下层，`robot_arm_driver`）** —— CANopen 硬件控制

```text
                    ┌──────────────────────────────┐
                    │        Director (上层)         │
                    └──────────────────────────────┘
  action ↓                ↑ ArmStatus           ↓ service
  (MoveToPose /          │ (10Hz)               │ (运维直达驱动)
   TrajectoryShot /      │                      │
   TrackTarget)          │                      │
                 ↓       │                      │
       ┌─────────────────────────┐              │
       │    Arm Commander        │              │
       │  - 状态机 IDLE/MOVING   │              │
       │  - IK + Ruckig OTG      │              │
       │  - 状态聚合 (TF2+Joint) │              │
       └─────────────────────────┘              │
   JointTrajectory ↓   ↑ JointState/TF2        ↓
                 │     │              ┌──────────────────┐
                 ↓     │              │   Driver (下层)   │
                 └─────┴────────────▶│   CANopen 硬件    │
                                      └──────────────────┘
```

数据流向规律：

- **运动类（action / topic）** 走 Director ↔ Arm Commander（需要 IK / 规划）。
- **运维类（service）** 由 Director 直达 Driver（上电 / 回零 / 清错是硬件操作）。
- **ArmStop** 作用于 Commander：软停 action（MOVING → STOPPED）**并停掉速度流**
  （速度流是 topic 流、不占状态机，所以单独处理）；**ArmResetError** 在透传驱动层 recover
  的同时，把 Commander 从 ERROR / STOPPED 复位回 IDLE，速度流随之解禁。
- Arm Commander 向下用关节级标准消息（`JointTrajectory`），
  向上把驱动反馈（`JointState` / TF2）合成为语义级 `ArmStatus`。

## Commander 状态机

Arm Commander 内部维护一个五态状态机（`CommanderState`），决定是否接受新 goal、
以及 `ArmStatus.is_moving` / `command_result` 的取值：

| 状态 | 值 | 含义 |
| --- | --- | --- |
| `IDLE` | 0 | 空闲，可接受新 goal |
| `MOVING` | 1 | 正在执行 goal（MoveToPose / TrajectoryShot / TrackTarget），`is_moving=true` |
| `REACHED` | 2 | 上一条 goal 成功到位；等同空闲，可直接接受新 goal |
| `STOPPED` | 3 | 被 `ArmStop` 急停；拒绝新 goal，需 `ArmResetError` 复位 |
| `ERROR` | 4 | **设备异常**（驱动层 / IK 服务失败、执行线程抛异常）；拒绝新 goal，需 `ArmResetError` 复位。**动作超时不在此列**（2026-09-08 起，见下） |

```text
                 goal 接受              成功
  IDLE / REACHED ─────────▶ MOVING ──────────▶ REACHED（视同空闲）
       ▲  ▲                  │ │
       │  │  取消 / IK 无解 / 限位校验不过（ABORTED）
       │  │  超时未到位 / 跟随丢失（FAILED，result 带原因）
       │  └──────────────────┘ │
       │                       ├── ArmStop ───▶ STOPPED ──┐
       │                       └── 驱动/服务异常 ▶ ERROR ──┤
       └───────────────── ArmResetError ──────────────────┘
```

转换规则（与代码一一对应，见 `arm_commander_node.cpp`）：

- **接受 goal**：仅当状态为 `IDLE` 或 `REACHED`（两者都算"空闲"）；否则直接 abort 该 goal。
  三个 action 共用此规则，同一时刻最多一条 goal 在执行。
- **MOVING → REACHED**：goal 成功完成，`command_result = SUCCEEDED`。
- **MOVING → IDLE（goal 被拒，`command_result = ABORTED`）**：上层取消（`"cancelled"`）、
  MoveToPose / TrajectoryShot 起止点 IK 无解（`"unreachable"`）、MoveToJoint 限位 / 自碰撞
  校验不过（`"out_of_range"` / `"collision"` / `"invalid_goal"`）。没下发任何指令，设备没故障。
- **MOVING → IDLE（goal 失败，`command_result = FAILED`）**：**2026-09-08 起**，动作在限时内
  没到位（`"timeout"`，含 TrackTarget 的 `"timeout"` / `"feature_lost"`）**不再进 ERROR**。
  result 里带 `exit_reason` / `error_code=ERR_TIMEOUT` 供调用方打印，`ArmStatus.error_code`
  **不置**；下一条正确的指令可直接执行，不需要 `ArmResetError`。
  超时那一刻若臂已静止且残差在放宽容差内（`tolerance.settle_*`），按 `"reached"` 收尾。
- **MOVING → STOPPED**：执行期间收到 `ArmStop`（软停轨迹 + goal abort，
  `command_result = ABORTED`）。非 MOVING 状态下调用 `ArmStop` 为空操作（应答"无需急停"）。
- **MOVING → ERROR**：**只有设备异常**才进 —— 驱动层 / IK 服务失败（`"error"`）或执行线程
  抛异常；`command_result = FAILED`，同时置 `ArmStatus.error_code`。这是需要人来干预的情形。
- **STOPPED / ERROR → IDLE**：仅由 `ArmResetError` 触发 —— 先透传驱动层 recover，
  再复位 Commander 状态、清 `error_code`、`command_result` 回 `RESULT_NONE`。
  在其余状态下调用 `ArmResetError` 只透传驱动层 recover，不改变 Commander 状态。

## 通信模式选择原则

| 模式 | 适用场景 |
| --- | --- |
| **Topic** | 高频连续、无需逐条确认：状态广播（`ArmStatus`）、速度流（`ArmFollowCommand` / `ArmJointVelocityCommand`） |
| **Service** | 瞬时请求-应答、立即返回：运维操作（上电/回零/清错/急停） |
| **Action** | 有时长、需进度反馈、可取消：位姿切换、运镜执行 |

## 公共结构体

末端位姿与速度统一用自定义六自由度结构体（RPY 度制），多处复用：

- `ArmPose`（msg）—— 末端位姿：x/y/z（米）+ roll/pitch/yaw（度，base_link 坐标系）
- `ArmTwist`（msg）—— 末端速度：vx/vy/vz（米/秒）+ wroll/wpitch/wyaw（度/秒）

## 接口清单

### Topic（持续流）

| 消息 | 方向 | 说明 |
| --- | --- | --- |
| `ArmFollowCommand` | Director → Commander | **笛卡尔速度控制**：末端 6 维 twist（`ArmTwist`）持续流，Commander 经 6×6 Jacobian 换算下发（2026-08-04 开通） |
| `ArmJointVelocityCommand` | Director → Commander | **关节速度控制**：J1-3 角速度持续流（2026-08-04 开通） |
| `ArmStatus` | Commander → Director | 位姿、速度、运动状态、错误码、命令执行结果、到位标志（到达目标 / 到达运镜起始点 / 摄像头录制就绪）（10Hz 周期广播） |

#### 速度控制 —— `ArmFollowCommand` / `ArmJointVelocityCommand`（2026-08-04 开通）

两条速度流总线，**共用同一套下游处理**（Commander 的 `VelocityStreamServer`），
区别只在入口的抽象层级 —— 一条给关节角速度，一条给末端 twist：

```text
Director ─ArmFollowCommand (末端 6 维 twist)──┐
                                             │ 6×6 几何 Jacobian + DLS 伪逆 → q̇(6)
Director ─ArmJointVelocityCommand (J1-3)─────┤ （关节速度直给臂那三轴，云台 q̇=0）
                                             ▼
                        Commander · VelocityStreamServer
                        限幅 → 积分成角度 → 按 URDF 限位夹紧
                                             ▼
                  /arm_controller/joint_trajectory（JTC，6 轴，50Hz 位置流）
                                             ▲
                  位置类动作（MoveToPose / MoveToJoint / TrajectoryShot）也走这里
```

> **速度和位置共用同一个控制器、同一条总线**（2026-08-04 改）。原先速度模式要切到独立的
> `arm_velocity_controller`（实物 CiA402 PV(3)），控制器切换带来三个副作用：切回时陈旧位置
> 命令被回放导致机械臂冲回旧位姿、轨迹类动作在速度模式下静默失效、云台要额外挂保持控制器。
> 改成位置流后这些一次性消失。代价是放弃驱动器内部速度环，高动态场景平滑度略逊 ——
> 需要时用 `mode_manager_node` 的 `velocity_backend:=velocity_controller` 切回 PV 后端。

**与位置类动作的关系**：`ArmMoveToPose`（收纳位/观察位/拍摄位）、`ArmMoveToJoint`、
`ArmTrajectoryShot` 在速度模式下会**自动切回 `TRAJECTORY` 再执行**（这类动作本来就要 JTC，
不切的话轨迹发出去不动、只会等到超时）。所以从速度模式直接下发这些动作是允许的，
上层不需要先手动切模式；反过来，速度流**不会**自动切模式（持续控制权必须显式申请）。

**换模不跳变**：切回 `TRAJECTORY` 时机械臂**原地不动**（实测残差 0.0000 rad，全程无偏离）
—— 因为压根没有控制器切换，模式切换只是更新语义标志。

**使用三步曲**（顺序不能变）：

1. `/robot_arm/switch_control_mode` 切到 `ControlMode.JOINT_VELOCITY`
   —— 这是**模式闸**：不切模式时速度指令一律被忽略并告警，防止误发的指令让机械臂动起来
   （速度流**不会**自动切模式，持续控制权必须显式申请；切模式只经 ModeManager）。
2. 以 **50-100Hz 持续发布**速度指令。低于 ~3Hz 会被断流看门狗（0.3s）判为掉线而停流。
3. 停止：**停发布**或**发一帧全 0**都可以，两者都是原地停住（停流后 JTC 保持最后一个点）。
   用完切回 `TRAJECTORY`。

**完整 6 自由度**：线速度 `vx/vy/vz`（m/s）由臂 J1-3 出，角速度 `wroll/wpitch/wyaw`
（**度/秒**，绕 base 系 X/Y/Z 轴的角速度矢量，不是 RPY 角速率）由云台 J4-6 出。
两段不是分开算的 —— Commander 用 J1-6 的 6×6 几何 Jacobian 一次解出全部 6 个关节速度，
位置与姿态的耦合（比如臂一动就把末端朝向带偏）由 Jacobian 自动补偿；6 个关节速度积分成角度后
在**同一条轨迹**里下发，所以补偿是精确的（早期臂走速度控制器、云台走位置流的双通道方案，
两条路动态特性不一致，姿态一直有残差）。

> 云台接近万向锁（J5 贴近 ±90°）或某轴压到限位时姿态方向会退化：DLS 给的是可行近似解，
> 跟踪精度下降。必要时先用 `ArmMoveToPose` 把云台摆回中位。

**安全闸**（任一不满足即停止下发；停流即 JTC 保持最后一点 = 原地停住）：

| 闸门 | 位置 | 行为 |
| --- | --- | --- |
| 控制模式 | Commander | 非 `JOINT_VELOCITY` 一律停流并告警 |
| 急停 | Commander | `ArmStop` 立刻停流并丢弃缓存指令，`ArmResetError` 解除 |
| 断流看门狗 | Commander，0.3s | 超时停流，JTC 保持最后一点（原地停住） |
| 逐轴限幅 | Commander | `max_joint_speed` / `max_gimbal_speed` |
| 关节限位 | Commander | 积分出的角度按 URDF 限位夹紧，到限位自然停住，反向撤离不受限 |
| 奇异点 | Commander | DLS 阻尼（`singularity_eps` / `damping_max`），q̇ 整体等比缩放保方向 |
| Jacobian 不可用 | Commander | URDF 未就绪 / TF 查不到 → **停流**（速度控制不能 fail-open） |

**Commander 侧参数**（`arm_commander` 节点，前缀 `velocity_stream.`）：
`rate_hz`(50) / `command_timeout`(0.3) / `lookahead`(0.05) / `max_linear_speed`(0.2 m/s) /
`max_angular_speed`(1.0 rad/s) / `max_joint_speed`(1.0 rad/s) / `max_gimbal_speed`(2.0 rad/s) /
`singularity_eps`(0.02) / `damping_max`(0.05) / `arm_joint_names`(J1-3) /
`gimbal_joint_names`(J4-6) / `trajectory_topic`。

> `rate_hz` **不要调到 100Hz**：JTC 每收到一条新轨迹就丢弃旧的重新插值，抢占太频繁反而
> 跟不动（实测只剩指令的两三成，压到 50Hz 后回到 98~100%）。本仓 `servo_config.yaml`
> 把 MoveIt Servo 压到 50Hz 是同一个原因。

**调试**：`commander_test_gui` 的「速度控制」面板 —— 模式切换按钮 + 三行点动
（关节速度 / 末端线速度 / 末端角速度），按住即动、松手即停并自动补 0 帧。

#### `ArmStatus` —— 状态广播（Commander → Director，10Hz）

```text
uint8    current_pose_state   # 当前语义姿态 STOWED=0 / OBSERVE=1 / SHOOTING=2
uint8    error_code           # ERR_NONE=0 / ERR_LIMIT=1 / ERR_DRIVER=2 / ERR_TIMEOUT=3
uint32   executing_command_id # 当前/最近命令 id（与 command_id 关联，0=上层不关心）
uint8    command_result       # RESULT_NONE=0 / EXECUTING=1 / SUCCEEDED=2 / FAILED=3 / ABORTED=4
ArmPose  arm_pose             # 末端当前位姿
ArmTwist arm_twist            # 末端当前速度
bool     is_moving            # 是否正在运动
bool     arm_at_target        # 是否已到达目标点（|实际 - 目标| < 容差）
bool     arm_at_pose_start    # 是否已到达运镜起始点（见下方说明）
bool     camera_ready         # 摄像头录制就绪（见下方说明）
bool     is_tracking          # 视觉伺服跟随中（ArmTrackTarget goal 活跃期间为 true）
float32  tracking_img_err     # 跟随中的图像误差（归一化欧氏距离，非跟随时 0.0）
float32  tracking_depth_err_m # 跟随中的深度误差（m，非跟随时 0.0）
```

- **`arm_at_target`**：到达目标点为 `true`。
- **`arm_at_pose_start`**：运镜（LINEAR / ORBIT）PTP 到达起始点后置 `true`，并在起点
  停顿 `DWELL_AT_START_SEC`（默认 1.0s）期间保持 `true`；停顿结束（开始执行轨迹）即复位
  `false`，故轨迹执行期间与到达目标后均为 `false`。每条新指令开始时也复位 `false`。
  消费者（Director / 录制节点）可据其上升沿在运镜起点触发录制等动作。
- **`camera_ready`**：**摄像头录制就绪窗口**。运镜（LINEAR / ORBIT）PTP 到达起始点时与
  `arm_at_pose_start` 同时置 `true`，但**不随停顿结束而复位**，而是持续保持 `true` 直到
  整条轨迹执行结束（成功、取消或异常均会触发复位）。每条新 `ArmTrajectoryShot` 指令开始时
  也复位 `false`。时序如下：

  ```text
  新 TrajectoryShot goal ──→ camera_ready = false（复位）
  PTP 到达起始点        ──→ camera_ready = true（与 arm_at_pose_start 同时）
    ├ 起点停顿 1s（arm_at_pose_start=true，camera_ready=true）
    └ 停顿结束（arm_at_pose_start=false，camera_ready 保持 true）
  轨迹运动中（LINEAR / ORBIT 运镜）── camera_ready = true
  运镜结束（任何原因）  ──→ camera_ready = false
  ```

  Director / 摄像头录制节点应在 `camera_ready` 上升沿开始录制、下降沿停止录制。

### Service（请求-应答）

| 服务 | 方向 | 说明 |
| --- | --- | --- |
| `ArmEnable` | Director → Driver | 伺服上电 / 下电 |
| `ArmHoming` | Director → Driver | 回零（阻塞，回零结束后应答） |
| `ArmResetError` | Director → Commander → Driver | 清除驱动层故障，复位 Commander 状态机（ERROR/STOPPED → IDLE），速度流随之解禁 |
| `ArmStop` | Director → Commander | 软件急停：停 action（MOVING → STOPPED）+ 停速度流（`ArmResetError` 复位后解禁） |

> `ArmEnable` / `ArmHoming` / `ArmResetError` 为运维直通接口，由 Commander 透传到 Driver，不经过轨迹规划层。

### Action（带 goal / feedback / result / cancel）

方向：**goal：Director（client）→ Arm Commander（server）；feedback · result：Commander → Director**

#### `ArmMoveToPose` —— 位姿切换

```text
Goal:
  uint8  target_pose_state   # STOWED=0 / OBSERVE=1 / SHOOTING=2
  uint8  transition_speed    # SLOW=0 / NORMAL=1 / FAST=2
  bool   return_to_start     # 到达目标后自动返回出发位姿
  ArmPose target_pose        # 仅 SHOOTING 时有效（绝对末端位姿）

Result:
  bool   success
  string exit_reason         # "reached" | "timeout" | "cancelled" | "stopped" | "unreachable" | "error"
  uint8  error_code
  ArmPose actual_pose

Feedback:
  float32 progress_percent   # [0.0, 100.0]，含 return_to_start 回程
  ArmPose current_pose
```

- **STOWED**：关节空间回零，不走 IK
- **OBSERVE**：预设观察位（Commander 内置，可通过 ROS param 覆盖）
- **SHOOTING**：绝对笛卡尔坐标（由 Director 指定）
- **return_to_start**：到位后自动原路返回出发点
- **到位判据只看臂 J1-3**（2026-07-31 改）：规划与下发仍是 6 轴（云台跟着一起动），但成败只判臂到达 IK 终点解。原因是末端 `gimbal_tool0` 在云台之后，云台回读不收敛会让末端位姿判据永不满足、动作全部超时。`Result.actual_pose` / `Feedback.current_pose` 仍是真实末端位姿（含云台偏差），仅作展示不参与判定。

#### `ArmMoveToJoint` —— 关节空间点到点（2026-07-31 新增）

```text
Goal:
  float64[] target_joints    # 目标关节角（rad），必须 3 个：Joint1 / Joint2 / Joint3
  uint8     transition_speed # SLOW=0 / NORMAL=1 / FAST=2
  bool      relative         # true=相对当前关节角的增量；false=绝对角
  float64   duration_sec     # >0 直接指定时长（覆盖档位）；<=0 按档位自动计算

Result:
  bool      success
  string    exit_reason      # "reached" | "out_of_range" | "collision" | "invalid_goal"
                             #  | "timeout" | "cancelled" | "stopped" | "error"
  uint8     error_code
  float64[] actual_joints    # 实际到达的关节角（J1-3）

Feedback:
  float32   progress_percent # [0.0, 100.0]
  float64[] current_joints   # 当前关节角（J1-3）
```

- **与 `ArmMoveToPose` 的分工**：本动作直接给关节角、**不过 IK**，用于示教/标定/绕开奇异点或明确要求某个臂形；笛卡尔目标（含 STOWED/OBSERVE 预设）仍走 `ArmMoveToPose`。
- **只动臂 J1-3**：云台 J4-6 由 Commander 以「当前回读」原样填充下发 = 保持不动（对比 `ArmMoveToPose` 的 STOWED 会把 6 轴全部归零）；到位判据也只看 J1-3，故云台未上电（回读不收敛）不会卡住本动作。
- **三道安全闸**，任一不过都**不下发任何指令**、回 IDLE（不进 ERROR，因为设备没故障）：
  1. 个数校验 → `invalid_goal`
  2. 关节限位 → `out_of_range`（限位从 `/robot_description` 解析 URDF 得到，**不写死常量**：限位会随实机标定平移，见 2026-07-28 J2/J3 零点重标定）
  3. 自碰撞 → `collision`（`/check_state_validity`，move_group 未运行时 fail-open + 告警）
- **时长**：`duration_sec>0` 时直接用；否则 `max|Δq| / 档位角速度`（0.3 / 0.6 / 1.2 rad·s⁻¹，见 `commander/motion_policy.hpp`），夹在 [0.5, 30] s。
- **不改 `current_pose_state`**：关节空间点到点是示教/标定用途，不代表产品语义上的收纳/观察/拍摄姿态。

#### `ArmTrajectoryShot` —— 运镜执行

```text
Goal:
  uint8   motion_type        # MOTION_LINEAR=0 / MOTION_ORBIT=1
  uint8   transition_speed   # SLOW=0 / NORMAL=1 / FAST=2
  bool    return_to_start    # 完成后原路返回出发点

  # MOTION_LINEAR 参数（直线运镜，两端绝对 Cartesian 位姿）
  ArmPose linear_start_pose  # 起始末端位姿（base_link 坐标系）
  ArmPose linear_end_pose    # 终止末端位姿

  # MOTION_ORBIT 参数（球面环绕运镜，相机始终朝向球心）
  float32 orbit_center_x/y/z  # 被摄主体位置（球心，米）
  float32 azimuth_start/end_deg  # 水平方向角起止（度，0=近侧）
  float32 elevation_start/end_deg # 俯仰角起止（度，正值向上）
  float32 radius_start/end_m      # 球面半径起止（米）

Result:
  bool   success
  string exit_reason         # "reached" | "timeout" | "cancelled" | "stopped" | "error"
  uint8  error_code

Feedback:
  float32 progress_percent
  float32 elapsed_sec
  ArmPose current_pose
  float32 current_azimuth/elevation_deg  # ORBIT 时实时球坐标
  float32 current_radius_m
```

**MOTION_LINEAR 执行流程：**

1. PTP 移动到 `linear_start_pose`（IK + JointTrajectory）
2. 到位后置 `arm_at_pose_start=true` / `camera_ready=true`，在起点停顿 `DWELL_AT_START_SEC`；停顿结束复位 `arm_at_pose_start=false`（`camera_ready` 保持 `true`）
3. PTP 移动到 `linear_end_pose`（`camera_ready` 持续 `true`）
4. 若 `return_to_start`，返回 `linear_start_pose`
5. 轨迹结束（任何原因）→ `camera_ready=false`

**MOTION_ORBIT 执行流程：**

1. PTP 移动到起始球坐标（IK + JointTrajectory）
2. 到位后置 `arm_at_pose_start=true` / `camera_ready=true`，在起点停顿 `DWELL_AT_START_SEC`；停顿结束复位 `arm_at_pose_start=false`（`camera_ready` 保持 `true`）
3. **Ruckig 1-DOF** 球面轨道运动（插值 θ/φ/r → 批量 IK → 多路点 JointTrajectory，`camera_ready` 持续 `true`）
4. 若 `return_to_start`，Ruckig 原路返回起始球坐标
5. 轨迹结束（任何原因）→ `camera_ready=false`

#### `ArmTrackTarget` —— 目标视觉跟随（IBVS）

```text
Goal:
  float32 desired_depth      # 期望保持距离（m），0.0 = 沿用节点默认值
  float32 desired_x / y      # 期望目标在图像中的位置（归一化，0.0 = 画面中心）
  bool    constrain_height   # 是否锁定相机高度（J1-J3 负责升降）
  float32 desired_height     # 期望相机高度（arm_base 系 Z，米），constrain_height=false 时忽略
  bool    hold_on_converge   # true=收敛后保持跟随等上层 cancel；false=收敛即 succeed 退出
  float32 total_timeout_sec  # 总超时（s），0.0 = 永不超时

Result:
  bool    success            # converged 与 cancelled 均视为 true，lost/timeout/error 为 false
  uint8   exit_code          # EXIT_CONVERGED=0 / FEATURE_LOST=1 / TIMEOUT=2 / CANCELLED=3 / ERROR=4
  string  exit_reason        # "converged" | "feature_lost" | "timeout" | "cancelled" | "error"
  float32 final_img_err      # 退出时图像误差（归一化欧氏距离）
  float32 final_depth_err_m  # 退出时深度误差（m）
  ArmPose final_pose         # 退出时末端位姿

Feedback:（10Hz）
  float32 img_err / depth_err_m / elapsed_sec
  bool    is_converged
  ArmPose current_pose
```

- `send_goal` 启动跟随（IBVS 输出速度指令），`cancel_goal` 停止跟随并保持当前位置
- 特征丢失超 0.5s / 达到 `total_timeout_sec` 会自动退出，无需上层 cancel
- 跟随期间 `ArmStatus.is_tracking = true`

## ROS Topic / Action / Service 汇总

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| `/robot_arm/arm_status` | `ArmStatus` topic | Commander → Director |
| `/robot_arm/follow_command` | `ArmFollowCommand` topic | Director → Commander（笛卡尔速度流） |
| `/robot_arm/cmd/joint_velocity` | `ArmJointVelocityCommand` topic | Director → Commander（关节速度流） |
| `/robot_arm/control_mode` | `ControlMode` topic (latched) | ModeManager → 全体（当前语义控制模式） |
| `/robot_arm/switch_control_mode` | `SwitchControlMode` service | Director → ModeManager（切控制模式，速度控制的前置步骤） |
| `/robot_arm/move_to_pose` | `ArmMoveToPose` action | Director → Commander |
| `/robot_arm/move_to_joint` | `ArmMoveToJoint` action | Director → Commander |
| `/robot_arm/trajectory_shot` | `ArmTrajectoryShot` action | Director → Commander |
| `/robot_arm/track_target` | `ArmTrackTarget` action | Director → Commander |
| `/robot_arm/stop` | `ArmStop` service | Director → Commander |
| `/robot_arm/enable` | `ArmEnable` service | Director → Driver |
| `/robot_arm/homing` | `ArmHoming` service | Director → Driver |
| `/robot_arm/reset_error` | `ArmResetError` service | Director → Driver |

## 约定

- **坐标系**：所有位姿基于机械臂底座坐标系，`ArmStatus.header.frame_id` 统一填 `base_link`。
- **单位**：长度米、速度米/秒、角度度（RPY）/ 弧度（球坐标 theta/phi 内部表示）。
- **速度档位**：`SLOW / NORMAL / FAST` 三档统一常量值（0/1/2），`ArmMoveToPose` / `ArmMoveToJoint` / `ArmTrajectoryShot` 共用相同常量；但**量纲不同** —— 笛卡尔动作是末端线速度/角速度（m·s⁻¹ / rad·s⁻¹，见 `motion_policy.hpp` 的 `Speed`），`ArmMoveToJoint` 是关节角速度（rad·s⁻¹，`JOINT_SPEED_*_RPS`）。
- **球坐标系**：方位角 θ=0 为近侧（相机到主体方向与主体到世界原点方向相同），正值顺时针；俯仰角 φ 正值向上，范围 (-90°, 90°)。
- **command_id**：由上层单调递增分配，`0` 表示上层不关心执行结果。仅用于 action 与
  `ArmStatus.executing_command_id` 的关联；两条速度流是持续控制、不带 command_id
  （逐帧命令 id 对速度流没有意义）。

## Arm Commander 实现位置

Commander 已于 2026-07 由 Python 全量移植为 C++（`robot_arm_node` 包），调试 GUI 拆到 `robot_arm_debug` 包：

| 文件 | 职责 |
| --- | --- |
| `robot_arm_node/src/commander/arm_commander_node.cpp` | 主节点：状态机 + 4 Action Server + 4 Service + ArmStatus 广播 |
| `robot_arm_node/src/commander/move_to_pose_server.cpp` | `ArmMoveToPose` 执行逻辑 |
| `robot_arm_node/src/commander/move_to_joint_server.cpp` | `ArmMoveToJoint` 执行逻辑（限位/自碰撞校验 + 关节空间点到点） |
| `robot_arm_node/src/motion/joint_limits.cpp` | 关节限位单一来源：解析 `/robot_description`（latched）取 URDF lower/upper |
| `robot_arm_node/src/commander/trajectory_shot_server.cpp` | `ArmTrajectoryShot` 执行逻辑（直线 / 球面轨道） |
| `robot_arm_node/src/commander/track_target_server.cpp` | `ArmTrackTarget` 执行逻辑（IBVS 视觉跟随启停） |
| `robot_arm_node/src/commander/velocity_stream_server.cpp` | 速度流：两条速度总线 → 6×6 Jacobian DLS → 积分成位置流走 JTC |
| `robot_arm_node/src/motion/jacobian.cpp` | 6×6 几何 Jacobian（TF + URDF）+ DLS 伪逆 |
| `robot_arm_node/src/mode/mode_manager_node.cpp` | 控制模式仲裁：唯一切换入口 + latched 广播当前模式（默认后端下只更新语义，不切控制器） |
| `robot_arm_node/src/commander/motion_executor.cpp` | 共享运动引擎：IK / Ruckig / JointTrajectory |
| `robot_arm_node/src/commander/execution_monitor.cpp` | 执行监视：统一等待循环 / 超时 / 取消 / 急停联动（到位判据由各 server 注入，统一只判臂 J1-3） |
| `robot_arm_node/src/state/status_aggregator.cpp` | 状态聚合：JointState + TF2 → ArmStatus |
| `robot_arm_debug/python/gui/commander_test_gui.py` | Director 视角调试 GUI：5 个面板（ArmStatus 监控 + MoveToPose + **MoveToJoint**（J1-3 滑块 / 读当前值 / 相对增量 / 时长）+ **速度控制**（模式切换 + 关节/末端点动）+ TrajectoryShot）+ 急停/清错/回零/使能 |

启动方式：

```bash
# Gazebo 仿真 + Arm Commander + 调试 GUI（一键）
ros2 launch robot_arm_gazebo gazebo.launch.py controller:=commander

# 单独启动 Commander（配合已有仿真）
ros2 run robot_arm_node arm_commander_node

# 单独启动调试 GUI
ros2 run robot_arm_debug commander_test_gui
```
