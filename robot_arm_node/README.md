# robot_arm_node

机械臂驱动节点，接收高层运动控制指令，驱动机械臂到达目标高度，并实时发布机械臂状态。

> **当前状态**：stub 实现，接口已定义，驱动层待对接硬件 SDK。

---

## 节点信息

| 项目 | 值 |
|---|---|
| 节点名 | `robot_arm_node` |
| 包名 | `robot_arm_node` |
| 可执行文件 | `robot_arm_node` |

---

## 订阅话题

### `/robot_arm/arm_cmd`
**消息类型**：`robot_arm_interfaces/msg/ArmCommand`

Director 通过 `/robot_chassis/motion_cmd` 中的 `arm` 子字段下发，由底盘节点或独立转发层路由至此话题。

| 字段 | 类型 | 说明 |
|---|---|---|
| `mode` | `uint8` | 控制模式：`POSITION=0`（目标高度）/ `VELOCITY=1`（速度控制）/ `STOP=2` |
| `height_m` | `float32` | 目标高度（m，`POSITION` 模式有效） |
| `velocity_mps` | `float32` | 运动速度（m/s，`VELOCITY` 模式有效） |

---

## 发布话题

### `/robot_arm/arm_status`
**消息类型**：`robot_arm_interfaces/msg/ArmStatus`

机械臂实时状态，Director 通过 `/robot_chassis/motion_status` 汇总后轮询使用。

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | `std_msgs/Header` | 时间戳 |
| `arm_height_m` | `float32` | 机械臂当前高度（m，编码器反馈） |
| `arm_at_target` | `bool` | 是否已到达目标高度（`|实际-目标| < 容差`） |

---

## 依赖

| 包 | 说明 |
|---|---|
| `rclcpp` | ROS2 C++ 客户端库 |
| `robot_arm_interfaces` | 接口定义包（ArmCommand / ArmStatus） |
