# IBVS 控制链路（visp_ibvs_node）话题流图

> 适用节点：`visp_ibvs_node.cpp`（ViSP + Pinocchio，50 Hz）
> 启动方式：`ros2 launch robot_arm_bringup real.launch.py controller:=visp_ibvs`

---

## 完整数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│  red_box_detector.py                                                │
│  摄像头图像 → OpenCV HSV 检测目标                                    │
│                                                                     │
│  发布  /red_detector/feature  (geometry_msgs/PointStamped)          │
│          .x = x_norm    归一化图像横坐标 [-1, 1]                     │
│          .y = y_norm    归一化图像纵坐标 [-1, 1]                     │
│          .z = depth     目标距离（米）                               │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ /red_detector/feature
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  visp_ibvs_node  （50 Hz wall_timer，CTRL_DT = 0.02 s）             │
│                                                                     │
│  输入 ①  /red_detector/feature  → 图像误差 (ex, ey)、深度误差 ez    │
│  输入 ②  /joint_states          → q_curr[6]（只读 position 字段）   │
│                                                                     │
│  计算流程：                                                          │
│    ViSP vpServo          → 图像雅可比 × 误差 → 相机速度 v_c (6D)    │
│    + 深度校正            → v_c[2] += DEPTH_GAIN × log(z / z*)      │
│    + 画面水平校正        → v_c[5] += K_LEVEL  × roll_err           │
│    Pinocchio Jacobian    → J(6×6，camera_optical_frame)             │
│    任务分级分配：                                                    │
│      第一优先级 J4-6（云台）→ 加权阻尼伪逆，快速消除图像误差         │
│      第二优先级 J1-3（机械臂）→ 大阻尼伪逆，仅补云台残差            │
│      + 高度约束（可选）  → J1-3 追加世界 Z 速度分量                 │
│    单关节限幅            → ±MAX_JOINT_VEL (1.5 rad/s)              │
│                                                                     │
│  输出：单点 JointTrajectory（J1-6，每帧覆盖上一帧）                  │
│    positions[i]  = q_curr[i] + q_dot[i] × 0.02 s  （前向积分）     │
│    velocities[i] = q_dot[i]                         （速度前馈）    │
│    time_from_start = 20 ms                                          │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ /arm_controller/joint_trajectory
                               │ trajectory_msgs/JointTrajectory（J1-6）
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  arm_node  （motion_mode = "pv"）                                   │
│  onTrajectory() 收到后立即分流                                       │
│                                                                     │
│   ┌──────────────────────────────┐  ┌──────────────────────────────┐│
│   │  J1-3  PV 模式               │  │  J4-6  分流转发              ││
│   │                              │  │                              ││
│   │  velocities[0-2]             │  │  positions[3-5]              ││
│   │  → radToVelPPSigned()        │  │  velocities[3-5]             ││
│   │  → setTargetVelocity(0x60FF) │  │  → /gimbal_controller/       ││
│   │    ×3（CANopen SDO 写入）    │  │     joint_trajectory         ││
│   │                              │  │    (JointTrajectory J4-6)    ││
│   │  看门狗 100 ms：             │  │                              ││
│   │  无新帧 → setTargetVelocity  │  │                              ││
│   │          (0) 停零            │  │                              ││
│   └──────────────┬───────────────┘  └──────────────┬───────────────┘│
└──────────────────┼──────────────────────────────────┼───────────────┘
                   │                                  │
                   │ CAN 总线  PV 模式                │ ROS2 topic
                   │ 直接写电机速度指令               │
                   ▼                                  ▼
         Joint1 / 2 / 3              ┌───────────────────────────────┐
         CANopen 电机                │  gimbal_controller            │
         转动                        │  JointTrajectoryController     │
                   │                │  open_loop_control = true      │
                   │                │  command_interfaces:           │
                   │                │    position + velocity         │
                   │                │  （不做位置闭环，直接透传）      │
                   │                └──────────────┬────────────────┘
                   │                               │ position + velocity
                   │                               │ command_interface
                   │                               ▼
                   │                     Camera HID 硬件接口
                   │                     Joint4 / 5 / 6
                   │                     云台舵机转动
                   │                               │
                   │   /joint_states (J1-3)         │ /joint_states (J4-6)
                   │   arm_node 从 CANopen 读取      │ joint_state_broadcaster
                   │   position  0x6064             │ 从 ros2_control 读取
                   │   velocity  0x606C（真实值）    │ position + velocity
                   ▼                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  /joint_states  （两个发布者合并到同一 topic）                        │
│  J1-3  ← arm_node（CANopen 真实反馈，含真实速度）                    │
│  J4-6  ← joint_state_broadcaster（ros2_control 硬件接口反馈）        │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               └─────────► visp_ibvs_node 输入 ②
                                           更新 q_curr[6]，驱动下一帧控制
```

---

## J1-3 vs J4-6 控制方式对比

| | J1-3（机械臂） | J4-6（云台） |
|---|---|---|
| **执行节点** | arm_node（PV 模式） | ros2_control gimbal_controller |
| **控制模式** | CANopen PV（速度直接控制） | JTC open_loop（速度前馈+位置透传） |
| **IBVS 用到的字段** | `velocities[0-2]` | `positions[3-5]` + `velocities[3-5]` |
| **闭环方式** | 驱动器内部速度环 | 无位置闭环（open_loop_control=true） |
| **反馈来源** | CANopen 0x606C（真实速度） | ros2_control 硬件接口 |

> 对 50 Hz IBVS 单点短帧（time_from_start=20ms）而言，
> J4-6 开环 JTC 下速度前馈是主导信号，效果与 J1-3 PV 模式等价。

---

## 关键参数速查

| 参数 | 位置 | 默认值 | 含义 |
|---|---|---|---|
| `CTRL_DT` | visp_ibvs_node.cpp | 0.02 s | 控制周期（50 Hz） |
| `FEATURE_TIMEOUT` | visp_ibvs_node.cpp | 0.5 s | 特征超时停止阈值 |
| `MAX_JOINT_VEL` | visp_ibvs_node.cpp | 1.5 rad/s | 单关节速度上限 |
| `LAMBDA_0` | visp_ibvs_node.cpp | 8.0 | ViSP 自适应增益（小误差段） |
| `LAMBDA_INF` | visp_ibvs_node.cpp | 4.0 | ViSP 自适应增益（大误差段，追动目标） |
| `pv_watchdog_ms` | arm.yaml | 100 ms | PV 模式看门狗超时 |
| `motion_mode` | arm.yaml / launch | `"pv"` | 由 `controller:=visp_ibvs` 自动推导 |

---

## 启动与调试

```bash
# 一键启动（IBVS 模式，arm_node 自动切 PV）
ros2 launch robot_arm_bringup real.launch.py controller:=visp_ibvs

# 运行时调整 IBVS 参数（无需重启）
ros2 param set /visp_ibvs_node desired_depth 0.35
ros2 param set /visp_ibvs_node desired_height 0.5
ros2 param set /visp_ibvs_node constrain_height true
ros2 param set /visp_ibvs_node paused true   # 暂停 IBVS，手动调位

# 查看话题数据
ros2 topic echo /red_detector/feature
ros2 topic echo /arm_controller/joint_trajectory
ros2 topic echo /joint_states
```
