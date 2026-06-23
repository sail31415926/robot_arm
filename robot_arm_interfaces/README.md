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
  action/topic ↓         ↑ ArmStatus           ↓ service
  (MoveToPose /          │ (10Hz)               │ (运维直达驱动)
   TrajectoryShot /      │                      │
   FollowCommand)        │                      │
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
- **ArmStop** 同时作用于 Commander（软停 + 状态复位）。
- Arm Commander 向下用关节级标准消息（`JointTrajectory`），
  向上把驱动反馈（`JointState` / TF2）合成为语义级 `ArmStatus`。

## 通信模式选择原则

| 模式 | 适用场景 |
| --- | --- |
| **Topic** | 高频连续、无需逐条确认：状态广播（`ArmStatus`）、速度流（`ArmFollowCommand`） |
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
| `ArmFollowCommand` | Director → Commander | 末端速度跟随（`ArmTwist`），持续速度流（IBVS / 手动点动） |
| `ArmStatus` | Commander → Director | 位姿、速度、运动状态、错误码、命令执行结果、到位标志（到达目标 / 到达运镜起始点 / 摄像头录制就绪）（10Hz 周期广播） |

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
| `ArmResetError` | Director → Driver | 清除驱动层故障 |
| `ArmStop` | Director → Commander | 软件急停（MOVING → STOPPED）；或复位（ERROR/STOPPED → IDLE） |

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
  string exit_reason         # "reached" | "timeout" | "cancelled" | "error"
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
  string exit_reason         # "reached" | "timeout" | "cancelled" | "error"
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

## ROS Topic / Action / Service 汇总

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| `/robot_arm/arm_status` | `ArmStatus` topic | Commander → Director |
| `/robot_arm/follow_command` | `ArmFollowCommand` topic | Director → Commander |
| `/robot_arm/move_to_pose` | `ArmMoveToPose` action | Director → Commander |
| `/robot_arm/trajectory_shot` | `ArmTrajectoryShot` action | Director → Commander |
| `/robot_arm/stop` | `ArmStop` service | Director → Commander |
| `/robot_arm/enable` | `ArmEnable` service | Director → Driver |
| `/robot_arm/homing` | `ArmHoming` service | Director → Driver |
| `/robot_arm/reset_error` | `ArmResetError` service | Director → Driver |

## 约定

- **坐标系**：所有位姿基于机械臂底座坐标系，`ArmStatus.header.frame_id` 统一填 `base_link`。
- **单位**：长度米、速度米/秒、角度度（RPY）/ 弧度（球坐标 theta/phi 内部表示）。
- **速度档位**：`SLOW / NORMAL / FAST` 三档统一常量值（0/1/2），`ArmMoveToPose` 与 `ArmTrajectoryShot` 共用相同含义。
- **球坐标系**：方位角 θ=0 为近侧（相机到主体方向与主体到世界原点方向相同），正值顺时针；俯仰角 φ 正值向上，范围 (-90°, 90°)。
- **command_id**：由上层单调递增分配，`0` 表示上层不关心执行结果。

## Arm Commander 实现位置

| 文件 | 职责 |
| --- | --- |
| `robot_arm_node/scripts/commander/arm_commander_node.py` | 主节点：状态机 + Action Server + ArmStatus 发布 + ArmStop 服务 |
| `robot_arm_node/scripts/commander/move_to_pose_server.py` | `ArmMoveToPose` 执行逻辑 |
| `robot_arm_node/scripts/commander/trajectory_shot_server.py` | `ArmTrajectoryShot` 执行逻辑（直线 / 球面轨道） |
| `robot_arm_node/scripts/commander/motion_executor.py` | 共享运动引擎：IK / Ruckig / JointTrajectory / 速度流 |
| `robot_arm_node/scripts/commander/status_aggregator.py` | 状态聚合：JointState + TF2 → ArmStatus |
| `robot_arm_node/tests/commander_test_gui.py` | Director 视角调试 GUI（ArmMoveToPose + ArmTrajectoryShot + 点动 + 急停/复位） |

启动方式：

```bash
# Gazebo 仿真 + Arm Commander + 调试 GUI（一键）
ros2 launch robot_arm_bringup gazebo.launch.py controller:=commander

# 单独启动 Commander（配合已有仿真）
ros2 run robot_arm_node arm_commander_node

# 单独启动调试 GUI
ros2 run robot_arm_node commander_test_gui
```
