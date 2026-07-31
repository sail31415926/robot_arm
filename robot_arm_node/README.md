# robot_arm_node

机械臂驱动节点，接收高层运动控制指令，驱动机械臂到达目标高度，并实时发布机械臂状态。

> **当前状态**：commander 产品栈（arm_commander_node：motion / state / commander），驱动经 `/arm_controller/joint_trajectory` 总线与三后端解耦。

---

## 节点信息

| 项目 | 值 |
| --- | --- |
| 节点名 | `robot_arm_node` |
| 包名 | `robot_arm_node` |
| 可执行文件 | `robot_arm_node` |

---

## 订阅话题

### `/robot_arm/arm_cmd`

**消息类型**：`robot_arm_interfaces/msg/ArmCommand`

Director 通过 `/robot_chassis/motion_cmd` 中的 `arm` 子字段下发，由底盘节点或独立转发层路由至此话题。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `mode` | `uint8` | 控制模式：`POSITION=0`（目标高度）/ `VELOCITY=1`（速度控制）/ `STOP=2` |
| `height_m` | `float32` | 目标高度（m，`POSITION` 模式有效） |
| `velocity_mps` | `float32` | 运动速度（m/s，`VELOCITY` 模式有效） |

---

## 发布话题

### `/robot_arm/arm_status`

**消息类型**：`robot_arm_interfaces/msg/ArmStatus`

机械臂实时状态，Director 通过 `/robot_chassis/motion_status` 汇总后轮询使用。

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 时间戳 |
| `arm_height_m` | `float32` | 机械臂当前高度（m，编码器反馈） |
| `arm_at_target` | `bool` | 是否已到达目标高度（`| 实际-目标 | < 容差`）  |

---

## 依赖

| 包 | 说明 |
| --- | --- |
| `rclcpp` | ROS2 C++ 客户端库 |
| `robot_arm_interfaces` | 接口定义包（ArmCommand / ArmStatus） |

---

## Arm Commander 接口参考

Arm Commander 是机械臂的中间层状态机，对外暴露 **4 个 Action 接口**：

| Action | 语义 | 目标空间 |
| :--- | :--- | :--- |
| `ArmMoveToPose` | 单点位姿切换（收纳 / 观察 / 拍摄） | 笛卡尔（走 IK） |
| `ArmMoveToJoint` | 关节空间点到点（示教 / 标定 / 绕奇异点）**2026-07-31 新增** | 关节（不过 IK，只动臂 J1-3） |
| `ArmTrajectoryShot` | 多段运镜轨迹（直线 / 球面环绕） | 笛卡尔（Ruckig + 批量 IK） |
| `ArmTrackTarget` | IBVS 视觉跟随启停 | 图像误差闭环 |

外加 4 个 Service（`enable` / `stop` / `reset_error` / `homing`）与 10 Hz 的 `/robot_arm/arm_status`。

> **到位判据统一只认臂 J1-3**（2026-07-31 改）：规划与下发都是 6 轴（云台跟着一起动），
> 但成败只判臂关节是否到达本段轨迹的终点解。末端 `gimbal_tool0` 在云台 J4-6 之后，
> 用末端位姿判定会被云台回读的静差拖累到全部超时（真机实测：云台偏差可让末端 pitch
> 差 38°，而位置只差 3 mm）。`actual_pose` / `current_pose` 仍是真实末端位姿，仅作展示。

### 前置条件

```bash
# 终端 1：启动 commander 模式
ros2 launch robot_arm_bringup real.launch.py controller:=commander

# 终端 2：使能伺服
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: true}"

# 确认状态正常（error_code 应为 0）
ros2 topic echo /robot_arm/arm_status --once
```

### 状态监控与基础控制

```bash
# 实时状态
ros2 topic echo /robot_arm/arm_status

# 使能 / 下电
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: true}"
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: false}"

# 急停 / 清除故障 / 回零
ros2 service call /robot_arm/stop        robot_arm_interfaces/srv/ArmStop {}
ros2 service call /robot_arm/reset_error robot_arm_interfaces/srv/ArmResetError {}
ros2 service call /robot_arm/homing      robot_arm_interfaces/srv/ArmHoming {}
```

`arm_status` 字段说明：

| 字段 | 含义 | 枚举值 |
| :--- | :--- | :--- |
| `current_pose_state` | 当前姿态 | 0=STOWED  1=OBSERVE  2=SHOOTING |
| `error_code` | 错误码 | 0=NONE  1=LIMIT  2=DRIVER  3=TIMEOUT |
| `command_result` | 命令结果 | 0=NONE  1=EXECUTING  2=SUCCEEDED  3=FAILED  4=ABORTED |
| `is_moving` | 是否运动中 | true / false |

### ArmMoveToPose — 姿态切换

接口：`/robot_arm/move_to_pose`，速度枚举：`transition_speed` — 0=SLOW  1=NORMAL  2=FAST

```bash
# STOWED 收纳位
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 0, transition_speed: 1, return_to_start: false}"

# OBSERVE 观察位
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 1, transition_speed: 1, return_to_start: false}"

# SHOOTING 自定义拍摄位（XYZ 单位：m；Roll/Pitch/Yaw 单位：°）
# 注意 roll=0 才是画面水平（云台 V2 的末端 gimbal_tool0 约定，见下方「位姿约定」）
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 2, transition_speed: 1, return_to_start: false,
    target_pose: {x: 0.30, y: 0.00, z: 0.50, roll: 0.0, pitch: 10.0, yaw: 0.0}}"

# 到位后自动返回起点
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 1, transition_speed: 1, return_to_start: true}"
```

### ArmMoveToJoint — 关节空间点到点

接口：`/robot_arm/move_to_joint`。直接给关节角、**不过 IK**，用于示教 / 标定 / 绕开奇异位形。
`target_joints` 必须 **3 个**（Joint1/2/3，单位 rad）；云台 J4-6 由 Commander 用当前回读填充下发
= **保持不动**（与 `ArmMoveToPose` 的 STOWED 不同，后者会把 6 轴全部归零）。

```bash
# 绝对角（-f 打印 feedback：progress + current_joints）
ros2 action send_goal -f /robot_arm/move_to_joint \
  robot_arm_interfaces/action/ArmMoveToJoint \
  "{target_joints: [0.5, 1.0, -1.2], transition_speed: 1, relative: false, duration_sec: 0.0}"

# 相对增量：J2 抬 0.3 rad，FAST 档
ros2 action send_goal /robot_arm/move_to_joint \
  robot_arm_interfaces/action/ArmMoveToJoint \
  "{target_joints: [0.0, 0.3, 0.0], transition_speed: 2, relative: true}"

# 指定时长（覆盖档位）：5 秒慢慢走
ros2 action send_goal /robot_arm/move_to_joint \
  robot_arm_interfaces/action/ArmMoveToJoint \
  "{target_joints: [0.0, 0.5, -0.8], duration_sec: 5.0}"
```

`duration_sec <= 0` 时按档位算时长：`max|Δq| / 档位角速度`（SLOW 0.3 / NORMAL 0.6 / FAST 1.2 rad·s⁻¹），
夹在 [0.5, 30] s。三道校验任一不过都**不下发任何指令**、状态回 IDLE：

| exit_reason | 触发条件 | 排查 |
| :--- | :--- | :--- |
| `invalid_goal` | `target_joints` 个数 ≠ 3 | 只给 J1-3，别给 6 个 |
| `out_of_range` | 超关节限位 | 限位从 `/robot_description` 解析 URDF 得到，日志会打出越界轴与区间 |
| `collision` | 目标姿态自碰撞 | `/check_state_validity` 判定；move_group 未运行时该检查 fail-open 并告警 |

### ArmTrajectoryShot — 轨迹运镜

接口：`/robot_arm/trajectory_shot`，运动类型：`motion_type` — 0=LINEAR  1=ORBIT

```bash
# 直线运镜 LINEAR：从起始位姿平滑运动到终止位姿
# 终点 x 由 0.60 改为 0.50：z=0.60 时 x 的可达上界约 0.52（rpy=0），0.60 整条线无解
ros2 action send_goal /robot_arm/trajectory_shot \
  robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 0, transition_speed: 1, return_to_start: false,
    linear_start_pose: {x: 0.2, y: 0.0, z: 0.60, roll: 0.0, pitch: 0.0, yaw: 0.0},
    linear_end_pose:   {x: 0.4, y: 0.0, z: 0.60, roll: 0.0, pitch: 0.0, yaw: 0.0}}"

# 球面环绕运镜 ORBIT：末端始终朝向球心
ros2 action send_goal /robot_arm/trajectory_shot \
  robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 1, transition_speed: 2, return_to_start: false,
    orbit_center_x: 0.60, orbit_center_y: 0.00, orbit_center_z: 0.50,
    azimuth_start_deg: -30.0, elevation_start_deg: 20.0, radius_start_m: 0.4,
    azimuth_end_deg:    30.0, elevation_end_deg:   20.0,  radius_end_m:   0.3}"
```

ORBIT 参数说明：

| 参数 | 含义 | 单位 |
| :--- | :--- | :--- |
| `orbit_center_x/y/z` | 被摄主体位置（球心） | m |
| `azimuth_start/end_deg` | 起止水平方位角 | ° |
| `elevation_start/end_deg` | 起止俯仰角 | ° |
| `radius_start/end_m` | 起止半径（可实现变焦距效果） | m |

`return_to_start: true` 可加入任意轨迹命令，执行完毕后自动返回起点。

### 位姿约定（2026-07-29 云台换 V2 后更新）

所有 `target_pose` / `linear_*_pose` 的 XYZ+RPY 都是 **`arm_base_link` → `gimbal_tool0`**
（云台 Joint6 之后的安装板，= MoveIt 规划组 tip；**不是**臂法兰 `tool0`）。

| 项 | 值 | 说明 |
| :--- | :--- | :--- |
| 末端 link | `gimbal_tool0` | 臂 J1-3 + 云台 J4-6 共 6 DOF |
| **画面水平的 roll** | **0°** | V1 二轴云台时代是 90°，换 V2 后变 0°（相机光轴仍是末端 +X 轴，但末端→光学系的固定旋转变了）；实现见 `motion/geometry.hpp` 的 `EEF_LEVEL_ROLL` |
| pitch | 负值 = 俯视 | 0° = 光轴水平 |
| 相机光学系 | `Cam0` | 别名 `camera_optical_frame`；+Z 光轴 / +Y 图像下 / +X 图像右 |

**填 roll=90 的后果**：SHOOTING / LINEAR 直接 IK 无解（`command_result: 3` FAILED）；
OBSERVE 能解但画面歪 90°。

### 可达域速查（rpy=0，`gimbal_tool0`，MoveIt IK 实测）

| 高度 z | 沿 +X 可达范围 |
| :--- | :--- |
| 0.60 m | x ∈ [0.12, 0.52] |
| 0.55 m | x ∈ [0.10, 0.54] |
| 0.50 m | x ∈ [0.10, 0.56] |
| 0.45 m | x ∈ [0.10, 0.58] |

（y=0、步长 0.02m 扫描所得；z 的绝对上界约 0.88m，但那里只能用近乎垂直的姿态。
换云台/改零点后这张表要重测。）

### 推荐测试顺序

```text
使能伺服 → echo arm_status 确认 error_code=0
  → MTP OBSERVE（验证基本运动）
  → MTP SHOOTING（验证 IK 和自定义位姿）
  → TSS LINEAR（验证直线插值）
  → TSS ORBIT（验证 Ruckig 环绕）
  → 急停测试
  → 下电
```

### 常见问题

| 现象 | 原因 | 处理 |
| :--- | :--- | :--- |
| action 等待超时 | move_group 未启动 | 确认 commander 模式启动日志有 `move_group` |
| `error_code: 2` DRIVER | CAN 通信异常 | 检查 CAN 接口是否 up，`candump can0` 验证 |
| `error_code: 1` LIMIT | 目标超出关节限位 | 减小 XYZ 幅度，先用 OBSERVE 位验证 |
| `command_result: 3` FAILED | IK 无解 | 先查 **roll 是不是填了 90**（应为 0，见「位姿约定」）；再对照「可达域速查」确认 XYZ 在范围内；最后考虑奇异点 |
| 画面歪斜 ~90° | roll 填了 90（V1 遗留约定） | 改成 roll=0 |
| 环绕运镜走一半报 IK 无解 | 同上，朝向 roll 不对导致整段无解 | 用 `aim_quat` 生成朝向，别手填 roll |

