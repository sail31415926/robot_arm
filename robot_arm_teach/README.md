# robot_arm_teach

eMeet 机械臂**示教模块**：示教记录 → 轨迹保存 → 轨迹回放 → DOLLY / TRUCK / ARC / CRANE 运镜类型管理。

独立 ROS2 包，与臂侧的协作**全部经既有接口**完成，没有修改 `robot_arm` 里任何一个既有文件。

---

## 文档

| 文档 | 内容 |
| --- | --- |
| [doc/仿真使用说明.md](doc/仿真使用说明.md) | 编译、启动、完整录制-回放流程、排查表 |
| [doc/实机能力限制说明.md](doc/实机能力限制说明.md) | **做不到什么**：手拖示教为何在实机被拒、点动为何只能动 J1-3、各道闸的 fail-open 边界 |
| [doc/轨迹格式说明.md](doc/轨迹格式说明.md) | YAML 格式逐字段说明、压缩语义、手工编辑注意事项 |

字段与接口语义的**权威定义在 `msg/*.msg` 和 `srv/*.srv` 的注释里**，本 README 只给全图。

---

## 一分钟上手

```bash
# 终端① 既有整机栈（示教节点不重复拉底座）
ros2 launch robot_arm_bringup bringup.launch.py backend:=gazebo controller:=commander
# 终端② 示教节点 + 操作面板（gui:=true 才起面板，默认只起节点）
ros2 launch robot_arm_teach teach.launch.py gui:=true

# 录 → 点动 → 停 → 存 → 放
ros2 service call /robot_arm/teach/start_teach robot_arm_teach/srv/StartTeach \
  "{name: 'demo', motion_type: {value: 1}, teach_mode: 0}"
ros2 topic pub -r 20 /robot_arm/teach/jog robot_arm_teach/msg/JogCommand "{velocities: [0.15, 0, 0]}"
ros2 service call /robot_arm/teach/stop_teach  robot_arm_teach/srv/StopTeach "{}"
ros2 service call /robot_arm/teach/save_trajectory robot_arm_teach/srv/SaveTrajectory \
  "{name: 'demo', overwrite: true}"
ros2 service call /robot_arm/teach/play_trajectory robot_arm_teach/srv/PlayTrajectory \
  "{name: 'demo', speed_scale: 0.5}"
```

---

## 设计要点

### 1. 示教状态与底层控制模式是两个维度，刻意不合并

`ControlMode`（`TRAJECTORY` / `JOINT_VELOCITY` / `JOINT_EFFORT` / `ADMITTANCE`）说的是
**关节现在被哪条通路驱动**，由 `mode_manager_node` 仲裁。
示教状态（`IDLE` / `RECORDING` / `RECORD_PAUSED` / `PLAYING` / `PLAY_PAUSED`）说的是
**用户正在录还是在放**，活在本包里。

两者正交：`RECORDING` 时底层是 `JOINT_VELOCITY`，`PLAYING` 时底层是 `TRAJECTORY`，
同一个 `ControlMode` 也可以对应 `IDLE`（别的应用在用速度总线）。

所以**没有**往 `robot_arm_interfaces/msg/ControlMode.msg` 里加 `TEACH`。
加了就等于让 ModeManager 知道「示教」这个业务概念——示教状态机以后每次演化都会牵动
控制模式仲裁，那是最不该有的耦合。`TeachState.underlying_control_mode` 只是把
当前底层模式**镜像**过来方便调试，不是状态机的一部分。

### 2. 耦合面：只用既有接口，一个都不改

```
读  /joint_states                        6 轴回读（录制的唯一数据源）
读  /robot_arm/control_mode  (latched)   当前底层模式，只作镜像与闸
读  /robot_arm/arm_status                急停/故障判据（error_code）
读  /robot_description       (latched)   URDF 关节限位
写  /robot_arm/cmd/joint_velocity        点动示教（既有产品总线，3 轴）
写  /arm_controller/joint_trajectory     回放（既有轨迹总线）
调  /robot_arm/switch_control_mode       切模式的唯一入口
调  /check_state_validity                自碰撞（move_group 提供，不可用则 fail-open）
```

一件都不做：不碰 `controller_manager/switch_controller`、不碰 CAN / 驱动器、
不改 `ControlMode.msg`、不改 ModeManager / ArmCommander 的任何逻辑。

### 3. 分两层：纯逻辑可测，ROS 接线单薄

| 层 | 文件 | 依赖 |
| --- | --- | --- |
| `robot_arm_teach_core` | `teach_types` / `teach_validator` / `teach_recorder` / `playback_planner` / `trajectory_store` | 只依赖生成的消息 + yaml-cpp，**不依赖 rclcpp** |
| `robot_arm_teach_node` | `joint_limits_guard` / `teach_node` / `teach_node_services` | 全部 ROS 耦合都在这一层 |

回放前置校验、压缩规则、分段规划写错的后果是让坏轨迹驱动实物机械臂，所以这三样
必须能在 CI 里跑到——这就是分层的理由。`test/` 下 4 个 gtest 全部只测 core 层，
不需要起任何节点（时间由测试注入，一段 700 秒的录制在几微秒内跑完）。

```bash
colcon test --packages-select robot_arm_teach && colcon test-result --verbose
```

---

## 操作面板（PyQt5）

命令行那套始终可用，但录一条轨迹要敲五六条 `ros2 service call`，所以本包带一个面板：

```bash
ros2 launch robot_arm_teach teach.launch.py gui:=true   # 节点 + 面板
ros2 run robot_arm_teach teach_gui                      # 面板单独起（节点已在跑）
```

面板是 **纯客户端** —— 只调本包的 13 个服务、只发 `JogCommand`，不碰任何总线、
不自己算轨迹。所以它崩了对机械臂没有影响（点动会被节点侧 0.3s 断流看门狗停住），
关掉它示教功能一点不少。源码 [python/teach_gui.py](python/teach_gui.py)。

分区对应五步流程：**① 录制 → ② 点动 J1-3 → ③ 轨迹列表 → ④ 保存 → ⑤ 回放**，
顶部是 `TeachState` 全字段回显，底部日志原样打印每次服务应答的 `exit_reason`
（回放被哪道闸拦下全靠它）。

四个值得知道的实现选择：

- **按钮按状态机联动**：`IDLE` 才让点「开始回放」，`RECORDING` 才让拖点动滑块。
  节点侧本来就会拒绝非法操作（返回 `busy`），界面先说清楚只是省一次往返。
- **点动滑块松手自动归零**，且只在 `jog_relay_enabled` 为真时才发布 —— 手离开就停。
- **回放框留空 = 回放内存缓冲区那条**。`stop_teach` 不落盘，所以刚录完的轨迹只在
  缓冲区里；「用缓冲区」按钮就是一键清空这个框。
- **服务调用全异步**（`call_async` + `add_done_callback`）：回放前置校验要跑最多 20 次
  自碰撞往返，同步等会把界面冻住几秒。done_callback 跑在 rclpy 线程里，只 emit 信号
  不碰控件。

⚠️ 改了 `python/teach_gui.py` 要重新 `colcon build` —— 它是 `install(PROGRAMS ... RENAME)`
装的，`--symlink-install` 对这种情况是复制而不是软链，改源码不会自动生效。

⚠️ 无显示环境（`DISPLAY` 未设 / ssh 无 X 转发）别开 `gui:=true`，Qt 起不来会直接退出。
VMware 虚拟机里若 Gazebo/RViz 崩在 `svga_surface_destroy` 或 SIGSEGV，
用软件渲染：`export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_X11_NO_MITSHM=1`。

---

## 接口一览

### 服务（全部挂在 `/robot_arm/teach/` 下）

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `start_teach` | `StartTeach` | 开始录制。查空闲 → 查急停 → 切底层模式 → 起采样器 → 开点动闸 |
| `stop_teach` | `StopTeach` | 结束录制。压缩 → 统计包络 → 留在内存缓冲区（**不自动落盘**） |
| `pause_teach` | `std_srvs/Trigger` | 暂停。时间轴冻结、点动闸关、立刻补发全 0 |
| `resume_teach` | `std_srvs/Trigger` | 继续 |
| `save_trajectory` | `SaveTrajectory` | 落盘。可顺手覆盖 `motion_type` / 补 `motion_meta`。校验不过**拒绝写入** |
| `load_trajectory` | `LoadTrajectory` | 从磁盘载入内存缓冲区，不做任何运动 |
| `play_trajectory` | `PlayTrajectory` | 回放。九道前置闸（见下），`dry_run: true` 只校验不下发 |
| `stop_playback` | `std_srvs/Trigger` | 停止并保持当前位置 |
| `pause_playback` | `std_srvs/Trigger` | 暂停并保持当前位置 |
| `resume_playback` | `std_srvs/Trigger` | 继续 |
| `set_playback_speed` | `SetPlaybackSpeed` | 回放中改倍率（下一分段生效） |
| `list_trajectories` | `ListTrajectories` | 列摘要，可按运镜类型过滤 |
| `delete_trajectory` | `DeleteTrajectory` | 删除。正在回放的那条拒绝删 |

### 话题

| 话题 | 类型 | 方向 |
| --- | --- | --- |
| `/robot_arm/teach/state` | `TeachState`（latched） | 出：示教状态机 |
| `/robot_arm/teach/playback` | `PlaybackState` | 出：回放进度 |
| `/robot_arm/teach/jog` | `JogCommand` | 入：点动输入（3 轴，J1-3） |

---

## 回放的九道闸

任一不过就**不下发任何指令**，`exit_reason` 指明是哪一道：

| # | 闸 | 失败时的 `exit_reason` |
| --- | --- | --- |
| ① | 关节名逐字一致、维度均为 6（**不按名重排**） | `invalid_trajectory` |
| ② | NaN / inf 检查 + `time_from_start` 严格单调递增 | `invalid_trajectory` |
| ③ | 每点位置在 URDF 关节限位内（URDF 未就绪则 fail-open + 告警） | `out_of_range` |
| ④ | 逐轴速度/加速度 ≤ 上限（按倍率 ×s / ×s² 复核） | `over_speed` |
| ⑤ | 自碰撞抽样送 `/check_state_validity`（服务不可用则 fail-open） | `collision` |
| ⑥ | 当前 J1-3 在轨迹起点附近，否则先走接近段或拒绝 | `not_at_start` |
| ⑦ | 无急停、无故障（读 `arm_status.error_code`） | `estopped` |
| ⑧ | 经 `/robot_arm/switch_control_mode` 切到 `TRAJECTORY` | `mode_switch_failed` |
| ⑨ | 经既有轨迹总线分段下发 | — |

回读不全（`/joint_states` 缺轴）时返回 `no_joint_state`；正在录/放时返回 `busy`。

---

## 三个容易误解的实现选择

**回放为什么要分段流式下发？**
一次把整条轨迹发给 JTC 最省事，但那样**回放中途什么都做不了**——JTC 只认最后收到的
那条轨迹。默认 1.0s 视野 / 5Hz 滚动，暂停在 200ms 内生效。
别把 `republish_hz` 往上调：JTC 每收新轨迹就丢弃旧的重新插值，高频抢占纯浪费
（`robot_arm_node` 的速度流为此从 100Hz 压到 50Hz）。

**暂停为什么要发一条「保持轨迹」？**
因为**发空轨迹不会让 JTC 停下**，它会把手上那条剩下的部分继续执行完，
表现为「点了暂停还在走」。所以暂停/停止时下发一条单点、速度为 0、
位置=当前实测值的轨迹。回读不全时则**什么都不发**并报错——发一条位置缺省为 0 的
保持轨迹会让机械臂冲向 0 位，比不发危险得多。

**为什么本包自己解析 URDF，而不复用 `robot_arm_node` 的 `JointLimitsCache`？**
那边的 CMakeLists 只有 `install(TARGETS ...)`，没有 `ament_export_targets`，
下游 `find_package` 拿不到可链接的目标。要复用就得改它的 CMakeLists，
而「尽量不动既有文件」是本次的硬要求。代价可接受：**限位数值的真相源仍然只有 URDF
一份**，重复的只是几十行解析代码。哪天那边导出了目标，把 `joint_limits_guard.*`
删掉换成它即可——接口是刻意对齐的。

---

## 实机限制（摘要，详见 [doc/实机能力限制说明.md](doc/实机能力限制说明.md)）

- **手拖示教实机不可用**。需要 `JOINT_EFFORT` / `ADMITTANCE`；实机 `real.launch.py`
  故意不配 `effort_controller`，ModeManager 会明确拒绝。`teach.allow_drag` 默认 `false`，
  传 `teach_mode: 1` 会被直接拒绝并说明替代做法——**不做"看起来能用"的静默降级**。
- **点动只能动 J1-3**。产品总线 `ArmJointVelocityCommand` 固定 3 个值，
  云台 J4-6 保持当前角度但会被原样录进轨迹。
- **点动手感有纹波**，这是既有速度通路的性质（实机实测跟踪率 71%~74%），
  而且会一并录进轨迹。
- **限位闸和自碰撞闸都会 fail-open**（URDF / `move_group` 不在时放行并告警）。
  看到那条告警时，闸实际上是关着的。
