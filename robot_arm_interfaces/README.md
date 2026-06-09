# robot_arm_interfaces

机械臂**上层（Director）↔ 中间层（Arm Commander）** 的语义级接口定义。
不面向硬件驱动层 —— 驱动层使用标准消息（`JointState` / `JointTrajectory` /
`Float64` / `DiagnosticStatus`），与本包解耦。

## 分层数据流

三个角色：

- **Director（上层）** —— 决策、语义意图（"去拍摄位""做个环绕"）
- **Arm Commander（中间层，`arm_commander` 节点 / `robot_arm_node` 包）** —— 笛卡尔规划 / IK / 轨迹生成
- **Driver（下层，`robot_arm_driver`）** —— CANopen 硬件控制

```text
                    ┌─────────────────────────┐
                    │   Director (上层)        │
                    └─────────────────────────┘
        goal/cmd ↓        ↑ status/result      ↓ service 请求
                 │        │                    │ (运维直达驱动)
                 ↓        │                    │
       ┌──────────────────────────┐            │
       │ Arm Commander (中间层)    │            │
       └──────────────────────────┘            │
   JointTrajectory ↓   ↑ JointState            ↓
                 │     │              ┌──────────────────┐
                 ↓     │              │  Driver (下层)    │
                 └─────┴─────────────▶│  CANopen 硬件     │
                                      └──────────────────┘
```

数据流向规律：

- **运动类（topic / action）** 走 Director ↔ Arm Commander（需要 IK / 规划）。
- **运维类（service）** 由 Director 直达 Driver（上电 / 回零 / 清错是硬件操作）。
- Arm Commander 向下用关节级标准消息（`JointTrajectory` / `Float64`），
  向上把驱动反馈（`JointState` / `DiagnosticStatus`）合成为语义级 `ArmStatus`。

## 通信模式选择原则

- **Topic** —— 高频连续、无需逐条确认：状态广播、速度流。
- **Service** —— 瞬时请求-应答、立即返回结果：运维、急停。
- **Action** —— 有时长、需进度反馈、可取消：到位、运镜。

## 公共结构体

末端位姿与速度统一用自定义六自由度结构体（rpy 度制，与命令接口一致），多处复用：

- `ArmPose`（msg）—— 末端位姿：x/y/z（米）+ roll/pitch/yaw（度）
- `ArmTwist`（msg）—— 末端速度：vx/vy/vz（米/秒）+ wroll/wpitch/wyaw（度/秒）

被 `ArmStatus`、`ArmFollowCommand`、`ArmMoveToPose` 引用。

## 接口清单

### Topic（持续流）

- `ArmFollowCommand`（msg）—— 末端速度跟随（`ArmTwist`），持续速度流（如视觉伺服 / 跟随）
  - 方向：**Director → Arm Commander**
- `ArmStatus`（msg）—— 位姿（`ArmPose`）、速度（`ArmTwist`）、运动中、到位、错误码、命令执行结果（周期广播）
  - 方向：**Arm Commander → Director**

### Service（请求-应答）

- `ArmEnable`（srv）—— 伺服上电 / 下电
- `ArmHoming`（srv）—— 回零
- `ArmResetError`（srv）—— 清除故障
- `ArmStop`（srv）—— 软件级急停（停止当前运动）
- 方向：**request：Director → Driver；response：Driver → Director**

### Action（带 goal/feedback/result/cancel）

- `ArmMoveToPose`（action）—— 姿态切换（收纳/观察/拍摄），等待到位
- `ArmExecuteMotion`（action）—— 运镜（抬升/推拉/横移/环绕），等待完成
- 方向：**goal：Director(client) → Arm Commander(server)；feedback·result：Arm Commander → Director**

`ArmStatus.executing_command_id` 回显命令的 `command_id`，配合
`command_result`（EXECUTING/SUCCEEDED/FAILED/ABORTED）让上层确认某条离散命令的结果。

## 约定

- **坐标系**：所有位姿基于机械臂底座坐标系，`ArmStatus.header.frame_id` 统一填 `base_link`。
- **单位**：长度米（`_m`）、速度米/秒（`_mps`）、角度度。
- **范围越界**：消息内的范围仅为约定，越界由驱动层钳位（详见各字段注释）。
- **command_id**：由上层单调递增分配，`0` 表示上层不关心该命令的执行结果。

## 已知缺口 / 后续

- **Arm Commander 消费节点尚未落地**：本包接口需要一个节点把语义命令翻译为
  关节级命令、并把驱动反馈合成为 `ArmStatus`，否则链路未打通。
- **服务端未实现**：service / 其余 action 的 server 端（`robot_arm_node` / `robot_arm_driver`）
  仍待实现，目前仅为接口定义。`ArmMoveToPose` 已有调试用 server/client
  （`scripts/tools/pose_command_debug.py` / `tests/pose_command_publisher.py`）。
