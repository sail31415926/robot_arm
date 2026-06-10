# commander_test_gui 测试说明

Arm Commander 上层接口的完整命令行测试参考，对应 `commander_test_gui.py` 的全部功能。

## 前置条件

```bash
# 1. 启动 commander 模式
source ~/zoulongyou_work/install/setup.bash
ros2 launch robot_arm_bringup real.launch.py controller:=commander

# 2. 使能伺服（另一个终端）
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: true}"

# 3. 确认状态正常（error_code 应为 0）
ros2 topic echo /robot_arm/arm_status --once
```

---

## 一、状态与控制

### 实时状态监控

```bash
ros2 topic echo /robot_arm/arm_status
```

字段含义：

| 字段 | 含义 | 枚举值 |
|------|------|--------|
| `current_pose_state` | 当前姿态 | 0=STOWED 1=OBSERVE 2=SHOOTING |
| `error_code` | 错误码 | 0=NONE 1=LIMIT 2=DRIVER 3=TIMEOUT |
| `command_result` | 命令结果 | 0=NONE 1=EXECUTING 2=SUCCEEDED 3=FAILED 4=ABORTED |
| `is_moving` | 是否运动中 | true/false |

### 使能 / 下电

```bash
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: true}"
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: false}"
```

### 急停

```bash
ros2 service call /robot_arm/stop robot_arm_interfaces/srv/ArmStop {}
```

### 清除故障

```bash
ros2 service call /robot_arm/reset_error robot_arm_interfaces/srv/ArmResetError {}
```

### 回零

```bash
ros2 service call /robot_arm/homing robot_arm_interfaces/srv/ArmHoming {}
```

---

## 二、ArmMoveToPose — 姿态切换

接口：`/robot_arm/move_to_pose`（`robot_arm_interfaces/action/ArmMoveToPose`）

速度枚举：`transition_speed` — 0=SLOW  1=NORMAL  2=FAST

### STOWED 收纳位（state=0）

```bash
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 0, transition_speed: 1, return_to_start: false}"
```

### OBSERVE 观察位（state=1）

```bash
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 1, transition_speed: 1, return_to_start: false}"
```

### SHOOTING 自定义拍摄位（state=2，需填 target_pose）

单位：XYZ 为米，Roll/Pitch/Yaw 为度。

```bash
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 2, transition_speed: 1, return_to_start: false,
    target_pose: {x: 0.30, y: 0.00, z: 0.50, roll: 90.0, pitch: 10.0, yaw: 0.0}}"
```

### 到位后自动返回起点

```bash
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 1, transition_speed: 1, return_to_start: true}"
```

---

## 三、ArmTrajectoryShot — 轨迹运镜

接口：`/robot_arm/trajectory_shot`（`robot_arm_interfaces/action/ArmTrajectoryShot`）

运动类型：`motion_type` — 0=LINEAR  1=ORBIT

### 直线运镜 LINEAR（motion_type=0）

从起始位姿平滑运动到终止位姿。

```bash
ros2 action send_goal /robot_arm/trajectory_shot \
  robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 0, transition_speed: 1, return_to_start: false,
    linear_start_pose: {x: 0.30, y: 0.0, z: 0.60, roll: 90.0, pitch: 0.0, yaw: 0.0},
    linear_end_pose:   {x: 0.30, y: 0.0, z: 0.40, roll: 90.0, pitch: 0.0, yaw: 0.0}}"
```

### 球面环绕运镜 ORBIT（motion_type=1）

以 `orbit_center` 为被摄主体，从起始球坐标环绕到终止球坐标，末端始终朝向球心。

参数说明：

| 参数 | 含义 | 单位 |
|------|------|------|
| `orbit_center_x/y/z` | 被摄主体位置（球心） | m |
| `azimuth_start/end_deg` | 起止水平方位角 | ° |
| `elevation_start/end_deg` | 起止俯仰角 | ° |
| `radius_start/end_m` | 起止半径（可变焦距） | m |

```bash
ros2 action send_goal /robot_arm/trajectory_shot \
  robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 1, transition_speed: 1, return_to_start: false,
    orbit_center_x: 0.60, orbit_center_y: 0.00, orbit_center_z: 0.50,
    azimuth_start_deg: -30.0, elevation_start_deg: -10.0, radius_start_m: 0.45,
    azimuth_end_deg:    30.0, elevation_end_deg:   30.0,  radius_end_m:   0.20}"
```

### 轨迹完成后返回起点

将 `return_to_start: true` 加入任意轨迹命令即可。

---

## 四、推荐测试顺序

```
使能伺服
  → echo arm_status 确认 error_code=0
  → MTP OBSERVE（验证基本运动）
  → MTP SHOOTING（验证 IK 和自定义位姿）
  → TSS LINEAR（验证直线插值）
  → TSS ORBIT（验证 Ruckig 环绕）
  → 急停测试
  → 下电
```

---

## 五、常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| action 等待超时 | move_group 未启动 | 确认 commander 模式启动日志有 `move_group` |
| `error_code: 2` DRIVER | CAN 通信异常 | 检查 CAN 接口是否 up，`candump can2` 验证 |
| `error_code: 1` LIMIT | 目标超出关节限位 | 减小 XYZ 幅度，先用 OBSERVE 位验证 |
| `command_result: 3` FAILED | IK 无解 | 调整目标位姿，避免奇异点或工作空间边界 |
