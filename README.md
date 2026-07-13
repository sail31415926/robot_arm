# eMeet Robot Arm — ROS 2 Package (Humble)

## 项目信息

| 项目属性 | 详情 |
| :--- | :--- |
| **项目名称** | eMeet Robot Arm ROS 2 Package |
| **版本** | v1.4.0 |
| **发布日期** | 2026-07-07 |
| **支持平台** | Ubuntu 22.04 LTS · ROS 2 Humble · Python 3.10 |
| **设备** | eMeet 6 轴机械臂（Joint1-3 CANopen + Joint4-6 相机云台 HID） |
| **语言** | C++ / Python / MATLAB |

---

## 包结构

```text
robot_arm/
├── robot_arm_interfaces/    # 自定义 msg / srv / action（ArmStatus、ArmMoveToPose、ArmTrajectoryShot…）
├── robot_arm_description/   # URDF / SRDF / MJCF / mesh / RViz / kinematics（pick_ik）配置
├── robot_arm_driver/        # C++ CANopen 驱动（J1-3）：arm_node（独立节点）+ ArmHardwareInterface（ros2_control 插件）
├── robot_arm_node/          # ★产品层：C++ 产品栈（motion / state / commander，arm_commander_node）
├── robot_arm_debug/         # ☆调试层：Python 调试控制器 / GUI / 工具 + 视觉感知（visp_ibvs、红块检测，暂归调试）
├── robot_arm_bringup/       # 统一启动入口（bringup/real/moveit launch）+ MoveIt 配置 + 三后端共享上层栈定义
├── robot_arm_gazebo/        # Gazebo 仿真后端：gazebo.launch.py + worlds / models 资产
├── robot_arm_mujoco/        # MuJoCo 仿真后端：mujoco.launch.py + mujoco_node 仿真桥
└── robot_arm_matlab/        # MATLAB 离线分析工具箱（Simscape 导出）
```

---

## 快速启动

三种运行环境共用同一套接口，均以 `controller:=<模式>` 选控制方式（模式见下表，默认 `joint_position`）：

```bash
ros2 launch robot_arm_bringup display.launch.py    # 仅 RViz 显示 URDF
ros2 launch robot_arm_gazebo gazebo.launch.py     # Gazebo 仿真
ros2 launch robot_arm_mujoco mujoco.launch.py     # MuJoCo 仿真
ros2 launch robot_arm_bringup real.launch.py       # 实物（arm_node + 云台 + MoveIt + 视频流）
ros2 launch robot_arm_bringup moveit.launch.py     # 单独 move_group + RViz
ros2 launch robot_arm_bringup motor.launch.py      # 仅底层电机控制（不含 MoveIt）
```

```bash
# 选控制模式 / 相机型号（示例）
ros2 launch robot_arm_gazebo gazebo.launch.py controller:=spherical_orbit
ros2 launch robot_arm_bringup real.launch.py   controller:=commander      # 中间层（默认含测试 GUI）
ros2 launch robot_arm_bringup real.launch.py   camera_type:=pixy          # 相机型号（默认 auto）
```

---

## 控制模式一览

| 模式 `controller:=` | 节点 | 机制 | 环境 |
| :--- | :--- | :--- | :--- |
| `joint_position` | `joint_position_controller_node` (py) | 直接发 JointTrajectory | Gazebo / MuJoCo / 实物 |
| `cartesian_moveit` | `cartesian_moveit_controller_node` (py) | MoveIt `compute_cartesian_path` | Gazebo / MuJoCo / 实物 |
| `cartesian_realtime_ik` | `cartesian_realtime_ik_controller_node` (py) | 实时 IK（`/compute_ik`） | Gazebo / MuJoCo / 实物 |
| `cartesian_trajectory` | `cartesian_trajectory_controller_node` (py) | Ruckig OTG → 批量 IK | Gazebo / MuJoCo / 实物 |
| `spherical_orbit` | `spherical_orbit_controller_node` (py) | Ruckig 球面轨迹（相机始终对中） | Gazebo / MuJoCo / 实物 |
| `cartesian_velocity` | `cartesian_velocity_controller_node` (py) | TwistStamped → MoveIt Servo / MuJoCo DLS | Gazebo / MuJoCo |
| `visp_ibvs_control`¹ | `visp_ibvs_node` (C++) + `red_box_detector` (py) | ViSP + Pinocchio 视觉闭环 | Gazebo / 实物 |
| `commander` | `arm_commander_node` (C++) | 状态机 + 3 Action（见下节） | Gazebo / MuJoCo / 实物 |

> 前 6 种为 Python 调试模式（节点被 GUI 进程内 import，故保留 Python）；`commander`（产品中间层）与 `visp_ibvs_control`（视觉伺服）为 C++ 产品栈。
> ¹ 实物后端该模式参数名为 `visp_ibvs`（见 `real.launch.py`）。

---

## 架构说明

整体设计的核心：**所有上层节点把命令汇聚到一条命令总线，所有后端把状态汇聚到一条反馈总线**。
Gazebo / MuJoCo / 实物三套后端共用同一对总线接口，上层控制逻辑对「跑仿真还是跑实物」无感。

### 层级跨框架图

纵向是抽象层级（L0→L4），横向标注每层用到的框架；两条总线横穿所有框架，是解耦上层与后端的关键。

```text
═══════════════════════════════════════════════════════════════════════════════════
 L4 应用/指挥层           外部业务 Director（下发拍摄/姿态任务、读状态）
                          └ commander_test_gui —— 测试用 Director（走 ROS Action）
───────────────────────────────────────────────────────────────────────────────────
 L3 产品中间层            arm_commander_node (C++)  单一状态机 IDLE/MOVING/REACHED/…
   [ROS2 Action/Srv/Topic]  ├ move_to_pose_server     姿态切换（关节 / IK 目标）
                            ├ trajectory_shot_server  直线 / 球面运镜（Ruckig OTG）
                            ├ track_target_server     视觉跟随（转调 visp 节点）
                            ├ execution_monitor       到位 / 超时判定
                            └ status_aggregator       10 Hz 汇聚 /robot_arm/arm_status
───────────────────────────────────────────────────────────────────────────────────
 L2 运动生成层  [MoveIt]        move_group   /compute_ik · /compute_cartesian_path
                [MoveIt Servo]  servo_node   TwistStamped → 关节增量
                [ROS2 / py]     controllers ×6  joint / cartesian_* / spherical_orbit
                [ViSP+Pinocchio]visp_ibvs_node (C++)  IBVS 视觉闭环
                                └ red_box_detector  图像 → /red_detector/feature
═══════════════════════════════════════════════════════════════════════════════════
   ▼ 命令总线  /arm_controller/joint_trajectory            (trajectory_msgs/JointTrajectory)
              /arm_controller/follow_joint_trajectory     (Action，MoveIt 执行轨迹用)
   ▲ 反馈总线  /joint_states                               (sensor_msgs/JointState, 6 轴)
═══════════════════════════════════════════════════════════════════════════════════
 L1 控制/后端层（三选一，总线接口一致，上层无感切换）
   [Gazebo]  gazebo_ros2_control → arm_controller (JTC, 6 轴) → 物理仿真
   [MuJoCo]  mujoco_node (py 桥)   订阅轨迹 / Servo，回填 joint_states + 相机图像
   [Real]    ros2_control 单 CM：J1-3 → RobotSystem（CANopen）
             J4-6 → GimbalForwardingInterface（转发）⇄ robot_gimbal_node（HID 独占）
───────────────────────────────────────────────────────────────────────────────────
 L0 物理/驱动     Gazebo 物理引擎   │   MuJoCo 物理   │   SocketCAN can0 + 云台 HID/V4L2
═══════════════════════════════════════════════════════════════════════════════════
```

> **实物 J1-3 驱动路径（唯一）：** ros2_control HAL —— `canopen_ros2_control/RobotSystem`
> （ros2_canopen，CiA402/SocketCAN，总线配置见 `robot_arm_driver/arm_driver/config/canopen/bus.yml`）
> + `arm_controller`(JTC) + 伴生 `arm_driver_services`；`real.launch.py` 单 controller_manager 管全 6 轴。
> 驱动层自测：`ros2 launch robot_arm_driver test_arm.launch.py`（mock/vcan 假从站/真机三合一，
> 详见 docs/ros2_canopen迁移.md）。旧 `arm_node`（自研 CANopenLinux 栈）已于 2026-07 下线。

### 关节划分

实物为单 controller_manager + 单 `arm_controller`(JTC) claim 全 6 轴（跨两个硬件组件），
命令入口与 Gazebo/MuJoCo 完全一致：`/arm_controller/joint_trajectory`（支持部分关节轨迹）。

| 关节 | 驱动（硬件组件） | 反馈来源 | 命令入口 |
| :--- | :--- | :--- | :--- |
| Joint1–3 | `canopen_ros2_control/RobotSystem`（ros2_canopen，CiA402/SocketCAN） | `joint_state_broadcaster` → `/joint_states` | `/arm_controller/joint_trajectory` |
| Joint4–6 | `robot_gimbal_driver/GimbalForwardingInterface`（转发插件，无 HID）⇄ `robot_gimbal_node`（唯一 HID 拥有者，真实回读） | `joint_state_broadcaster` → `/joint_states` | `/arm_controller/joint_trajectory`（同一控制器） |

### 各节点职责与数据传输

| 节点（可执行） | 包·语言 | 职责 | 关键输入 → 输出 |
| :--- | :--- | :--- | :--- |
| `arm_commander_node` | robot_arm_node · C++ | 产品中间层状态机，把「拍摄/姿态」意图翻译成轨迹 | `/robot_arm/{move_to_pose,trajectory_shot,track_target}` Action、`/joint_states`、`/compute_ik` → `/arm_controller/joint_trajectory`、`/robot_arm/arm_status` |
| `controllers ×6` | robot_arm_debug · py | 调试控制：关节滑块 / 笛卡尔 / 实时 IK / Ruckig 点到点 / 球面运镜 / 速度点动 | GUI 滑块、`/joint_states`、`/compute_ik`·`/compute_cartesian_path` → `/arm_controller/joint_trajectory`（`cartesian_velocity` 改发 `/servo_node/delta_twist_cmds`） |
| `visp_ibvs_node` | robot_arm_debug · C++ | ViSP+Pinocchio 图像伺服，加权 Jacobian 直接算关节速度 | `/red_detector/feature`（或外部 `perception_topic`）、`/joint_states` → `/arm_controller/joint_trajectory` |
| `red_box_detector` | robot_arm_debug · py | OpenCV 红块检测，产出归一化像素特征 + 深度 | `/camera/camera_sensor/image_raw` → `/red_detector/feature`、`/red_detector/image` |
| `mujoco_node` | robot_arm_mujoco · py | MuJoCo 仿真桥：物理步进 + 相机渲染 | `/arm_controller/joint_trajectory`、`/servo_node/delta_twist_cmds` → `/joint_states`、`/camera/camera_sensor/image_raw` |
| `move_group` | MoveIt | 运动规划 / IK / 笛卡尔路径 | 规划请求 → `/compute_ik`、`/compute_cartesian_path` 服务；`/arm_controller/follow_joint_trajectory` 执行 |
| `servo_node` | MoveIt Servo | 实时笛卡尔速度 → 关节增量 | `/servo_node/delta_twist_cmds` → `/arm_controller/joint_trajectory` |
| `ros2_control_node` + `arm_controller`(JTC) | controller_manager · C++ | 【实物】单 CM 管全 6 轴：J1-3 CANopen（RobotSystem，position→IP 模式）+ J4-6 云台（GimbalForwardingInterface 转发 ⇄ robot_gimbal_node，见 docs/云台控制路径融合方案.md） | `/arm_controller/joint_trajectory`、`/arm_controller/follow_joint_trajectory` Action → CAN/HID + `/joint_states`(6 轴) |
| `arm_driver_services` | robot_arm_driver · C++ | 【实物】使能/失能/故障恢复服务（转发 controller_manager 硬件组件状态） | `/arm_node/{enable,disable,recover}`(Trigger) → CM 组件 active↔inactive |
| `arm_controller` (JTC) | Gazebo / ros2_control | 【Gazebo】6 轴关节轨迹控制器驱动仿真模型 | `/arm_controller/joint_trajectory` → 物理 + `/joint_states` |
| `robot_camera_node` | robot_gimbal_node · C++ | 【实物】V4L2 视频流（仅占 V4L2，与 HID 不冲突） | 相机 → `/camera/image_raw/compressed` |
| `robot_state_publisher` | ROS 2 | URDF + 关节角 → TF 树 | `/joint_states` → `/tf` |

### 数据流（三条链路）

- **命令下行**：上层节点（controllers / commander / visp / servo）统一发 `/arm_controller/joint_trajectory`；后端（Gazebo `arm_controller` / MuJoCo `mujoco_node` / 实物 `arm_controller`(单 CM 全 6 轴)）择一消费。MoveIt 规划结果走 `/arm_controller/follow_joint_trajectory` Action。
- **状态上行**：后端统一回填 `/joint_states`（6 轴，实物由单 `joint_state_broadcaster` 汇聚两个硬件组件）→ 供上层闭环、`robot_state_publisher` 出 TF、commander 汇聚成 `/robot_arm/arm_status`（10 Hz）。
- **视觉闭环**：相机图像（仿真 `/camera/camera_sensor/image_raw`，实物 `/camera/image_raw/compressed`）→ `red_box_detector` → `/red_detector/feature`（`geometry_msgs/PointStamped`，x/y 为归一化像素、z 为深度）→ `visp_ibvs_node` → 命令总线。

---

## 主要依赖

**系统基础**：Ubuntu 22.04 + ROS 2 Humble + Python 3.10；`colcon` / `ament_cmake` 编译（接口包用 `rosidl`）；C++ 驱动基于内核 SocketCAN + 随包编译的 CANopen 协议栈。

**ROS 2（apt，不进 venv）**：`rclcpp` / `rclpy` / `rclcpp_action`、消息包（`sensor_msgs` `geometry_msgs` `trajectory_msgs` `control_msgs` `moveit_msgs` 等）、`ros-humble-moveit` + `pick_ik`、`ros2_control` / `ros2_controllers`、`gazebo_ros` / `gazebo_ros2_control`、`cv_bridge` / `image_transport`、`tf2_ros` / `tf-transformations`、`robot_state_publisher` / `xacro`、`robot_gimbal_driver`（云台插件）、`python3-tk`。

**Python（pip，见 [`requirements.txt`](requirements.txt)）** —— 核心运行依赖：

| 依赖 | 版本 | 用途 |
| :--- | :--- | :--- |
| `numpy` | 1.26.4（**必须 <2**） | 数值计算（全包） |
| `PyYAML` | 5.4.1 | 配置读取 |
| `ruckig` | 0.17.3 | 在线轨迹生成（jerk-limited OTG） |
| `opencv-python-headless` | 4.13.0.92 | 视觉 / IBVS / 红盒检测 |
| `PyQt5` | 5.15.11 | 电机测试 / 滑块 GUI |

> 可选：`mujoco` 3.8.1（跑 MuJoCo 仿真时）；RL 训练（`robot_arm_rl`）另需 `gymnasium` / `stable-baselines3` / `torch`(CPU) / `tensorboard`。
> `scipy` / `matplotlib` / `pinocchio` 在本包无 `import`，无需安装（存在于共享 venv 仅因别的项目）。

---

## 环境配置

一键脚本创建 `--system-site-packages` 的 venv 并装齐 pip 依赖（source ROS → 建/复用 venv → 装 CPU 版 `torch` + `requirements.txt` → 校验可 import 且 `numpy<2`）。apt 前置依赖（moveit / ros2_control / gazebo 等）需自行装好，脚本只检查不安装。

```bash
bash src/E7009/robot_arm/setup_venv.sh
# 可选指定路径：VENV_DIR=/path/to/venv ROS_SETUP=/opt/ros/humble/setup.bash bash .../setup_venv.sh
```

每个新终端：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash   # 编译后
```

---

## 编译

```bash
# 编译所有机械臂相关包
colcon build --symlink-install --packages-select \
  robot_arm_interfaces robot_arm_driver robot_arm_description \
  robot_arm_bringup robot_arm_node robot_arm_debug robot_arm_gazebo robot_arm_mujoco

# 生效
source install/setup.bash
```

---

## Arm Commander 接口参考

Arm Commander（`controller:=commander`）是中间层状态机，对外暴露 3 个 Action —— `ArmMoveToPose`（姿态切换）、`ArmTrajectoryShot`（运镜轨迹）、`ArmTrackTarget`（视觉跟随），4 个 Service（`enable` / `stop` / `reset_error` / `homing`），并以 10 Hz 发布 `/robot_arm/arm_status`。

### 启动与基础控制

```bash
ros2 launch robot_arm_bringup real.launch.py controller:=commander
ros2 service call /robot_arm/enable robot_arm_interfaces/srv/ArmEnable "{enable: true}"
ros2 topic echo  /robot_arm/arm_status --once      # 确认 error_code=0

# 急停 / 清故障 / 回零
ros2 service call /robot_arm/stop        robot_arm_interfaces/srv/ArmStop {}
ros2 service call /robot_arm/reset_error robot_arm_interfaces/srv/ArmResetError {}
ros2 service call /robot_arm/homing      robot_arm_interfaces/srv/ArmHoming {}
```

`arm_status` 字段：

| 字段 | 含义 | 枚举值 |
| :--- | :--- | :--- |
| `current_pose_state` | 当前姿态 | 0=STOWED  1=OBSERVE  2=SHOOTING |
| `error_code` | 错误码 | 0=NONE  1=LIMIT  2=DRIVER  3=TIMEOUT |
| `command_result` | 命令结果 | 0=NONE  1=EXECUTING  2=SUCCEEDED  3=FAILED  4=ABORTED |
| `is_moving` | 是否运动中 | true / false |

### ArmMoveToPose — 姿态切换

`/robot_arm/move_to_pose`；`target_pose_state`: 0=STOWED 1=OBSERVE 2=SHOOTING，`transition_speed`: 0=SLOW 1=NORMAL 2=FAST。SHOOTING 需给 `target_pose`（XYZ 单位 m，RPY 单位 °）；任意命令加 `return_to_start: true` 执行完自动回起点。

```bash
ros2 action send_goal /robot_arm/move_to_pose robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 2, transition_speed: 1, return_to_start: false,
    target_pose: {x: 0.30, y: 0.0, z: 0.50, roll: 90.0, pitch: 10.0, yaw: 0.0}}"
```

### ArmTrajectoryShot — 轨迹运镜

`/robot_arm/trajectory_shot`；`motion_type`: 0=LINEAR（直线）1=ORBIT（球面环绕，末端始终朝向球心）。

```bash
# LINEAR：起→止位姿平滑直线
ros2 action send_goal /robot_arm/trajectory_shot robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 0, transition_speed: 1, return_to_start: false,
    linear_start_pose: {x: 0.30, y: 0.0, z: 0.60, roll: 90.0, pitch: 0.0, yaw: 0.0},
    linear_end_pose:   {x: 0.30, y: 0.0, z: 0.40, roll: 90.0, pitch: 0.0, yaw: 0.0}}"

# ORBIT：绕球心环绕（起止半径不同即变焦距）
ros2 action send_goal /robot_arm/trajectory_shot robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 1, transition_speed: 1, return_to_start: false,
    orbit_center_x: 0.60, orbit_center_y: 0.0, orbit_center_z: 0.50,
    azimuth_start_deg: -30.0, elevation_start_deg: -10.0, radius_start_m: 0.45,
    azimuth_end_deg:    30.0, elevation_end_deg:   30.0,  radius_end_m:   0.20}"
```

> ORBIT 参数：`orbit_center_*`=球心/被摄主体（m），`azimuth/elevation_*_deg`=起止方位角/俯仰角（°），`radius_*_m`=起止半径（m）。

### 常见问题

| 现象 | 原因 | 处理 |
| :--- | :--- | :--- |
| action 等待超时 | move_group 未启动 | 确认 commander 模式启动日志有 `move_group` |
| `error_code: 2` DRIVER | CAN 通信异常 | 检查 CAN 接口是否 up，`candump can0` 验证 |
| `error_code: 1` LIMIT | 目标超出关节限位 | 减小 XYZ 幅度，先用 OBSERVE 位验证 |
| `command_result: 3` FAILED | IK 无解 | 调整目标位姿，避免奇异点或工作空间边界 |
