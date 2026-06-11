# eMeet Robot Arm — ROS 2 Package (Humble)

## 项目信息

| 项目属性 | 详情 |
| :--- | :--- |
| **项目名称** | eMeet Robot Arm ROS 2 Package |
| **版本** | v1.3.0 |
| **发布日期** | 2026-06-11 09:10 |
| **支持平台** | Ubuntu 22.04 LTS |
| **ROS 2 版本** | Humble |
| **设备兼容** | eMeet 机械臂（6 轴：Joint1-3 CANopen + Joint4-6 相机云台） |
| **编程语言** | C++ / Python / MATLAB |

---

## 包结构

```bash
src/E7009/robot_arm/
├── robot_arm_interfaces/       # 自定义消息 / 服务 / 动作定义
│   ├── msg/                    #   ArmStatus、ArmPose、ArmTwist 等
│   ├── srv/                    #   ArmEnable、ArmHoming、ArmStop 等
│   └── action/                 #   ArmMoveToPose、ArmTrajectoryShot
│
├── robot_arm_description/      # 机械臂静态模型资源
│   ├── urdf/                   #   eMeetArm_models.urdf（6-DOF）
│   ├── srdf/                   #   碰撞组、规划组定义
│   ├── xml/                    #   eMeetArm.xml（MuJoCo MJCF）
│   ├── meshes/                 #   base_link + Link1-6.STL
│   ├── config/                 #   controllers / joint_limits / kinematics（pick_ik）
│   └── rviz/                   #   URDF 可视化 / MoveIt 配置
│
├── robot_arm_driver/           # C++ CANopen 硬件驱动（Joint1–3）
│   ├── include/                #   CiA402 驱动器头文件
│   ├── src/                    #   arm_node / arm_motor_node / CANopenLinux
│   ├── config/                 #   arm.yaml / motors.yaml
│   └── scripts/                #   motor_test_gui.py（PyQt5）
│
├── robot_arm_node/             # Python 应用层控制节点
│   └── scripts/
│       ├── controllers/        #   8 种控制模式节点（joint / cartesian / orbit / ibvs…）
│       ├── gui/                #   各控制模式对应 GUI（PyQt5 / tkinter）
│       ├── vision/             #   红框检测、IBVS 视觉伺服
│       ├── simulation/         #   MuJoCo ↔ ROS 2 桥接节点
│       ├── commander/          #   Arm Commander 中间层（状态机、Action Server）
│       └── tools/              #   轨迹桥接、工具函数
│
├── robot_arm_bringup/          # 启动入口 & 仿真资源
│   ├── launch/                 #   display / motor / real / gazebo / mujoco / moveit
│   ├── config/moveit/          #   OMPL / Servo / 控制器映射配置
│   └── sim/                    #   Gazebo world、ArUco 模型、MuJoCo 资源
│
└── robot_arm_matlab/           # MATLAB 离线分析工具箱
    └── eMeetArm_models/        #   Simscape Multibody 导出的 ROS 2 包
```

---

## 快速启动

### URDF 可视化

```bash
ros2 launch robot_arm_bringup display.launch.py
```

### Gazebo 仿真

```bash
# 默认（关节滑块模式）
ros2 launch robot_arm_bringup gazebo.launch.py

# 指定控制模式
ros2 launch robot_arm_bringup gazebo.launch.py controller:=joint_position        # 关节滑块
ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_moveit      # MoveIt 笛卡尔直线
ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_realtime_ik # 滑块即时 IK
ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_trajectory  # Ruckig 点到点 + 环绕
ros2 launch robot_arm_bringup gazebo.launch.py controller:=spherical_orbit       # 球面轨道运镜
ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian_velocity    # 笛卡尔速度点动
ros2 launch robot_arm_bringup gazebo.launch.py controller:=ibvs_control          # 红色方块 IBVS 闭环
ros2 launch robot_arm_bringup gazebo.launch.py controller:=commander             # Arm Commander 中间层
```

### MuJoCo 仿真

```bash
ros2 launch robot_arm_bringup mujoco.launch.py
ros2 launch robot_arm_bringup mujoco.launch.py controller:=cartesian_moveit
```

### 真实硬件

```bash
# 一键启动（arm_node + ros2_control + MoveIt + GUI + 视频流）
ros2 launch robot_arm_bringup real.launch.py

# 指定控制模式（支持全部 8 种，同 Gazebo）
ros2 launch robot_arm_bringup real.launch.py controller:=spherical_orbit
ros2 launch robot_arm_bringup real.launch.py controller:=cartesian_trajectory

# 指定相机型号（默认 auto）
ros2 launch robot_arm_bringup real.launch.py camera_type:=pixy

# 仅启动底层电机控制（不含 MoveIt）
ros2 launch robot_arm_bringup motor.launch.py
```

### MoveIt2 运动规划

```bash
# 独立启动 move_group + RViz
ros2 launch robot_arm_bringup moveit.launch.py

# 配合仿真使用
ros2 launch robot_arm_bringup moveit.launch.py use_sim_time:=true
```

### 测试工具

```bash
ros2 launch robot_arm_bringup test_ros.launch.py
ros2 run robot_arm_driver motor_test_gui

# Arm Commander 功能测试（需分两个终端）
ros2 launch robot_arm_bringup gazebo.launch.py controller:=commander
ros2 run robot_arm_node commander_test_gui
```

---

## 控制模式一览

| 模式参数 | 控制节点 | 控制机制 | GUI | 支持环境 |
| :--- | :--- | :--- | :---: | :--- |
| `joint_position` | `controllers/joint_position_controller_node.py` | 直接发布 JointTrajectory | PyQt5 滑块 | Gazebo / MuJoCo / 实物 |
| `cartesian_moveit` | `controllers/cartesian_moveit_controller_node.py` | MoveIt `compute_cartesian_path` | tkinter | Gazebo / MuJoCo / 实物 |
| `cartesian_realtime_ik` | `controllers/cartesian_realtime_ik_controller_node.py` | 实时 IK（`/compute_ik`）→ JointTrajectory | tkinter | Gazebo / MuJoCo / 实物 |
| `cartesian_trajectory` | `controllers/cartesian_trajectory_controller_node.py` | Ruckig OTG → 批量 IK → JointTrajectory | tkinter | Gazebo / MuJoCo / 实物 |
| `spherical_orbit` | `controllers/spherical_orbit_controller_node.py` | Ruckig 球面轨迹 → IK → JointTrajectory（相机始终对中） | tkinter | Gazebo / MuJoCo / 实物 |
| `cartesian_velocity` | `controllers/cartesian_velocity_controller_node.py` | TwistStamped → MoveIt Servo / MuJoCo DLS | tkinter 点动 | Gazebo / MuJoCo |
| `ibvs_control` | `vision/ibvs_control_node.py` + `vision/red_box_detector.py` | IBVS 视觉闭环 → TwistStamped | — | Gazebo |
| `commander` | `commander/arm_commander_node.py` | ArmMoveToPose / ArmTrajectoryShot Action Server | commander_test_gui | Gazebo / MuJoCo / 实物 |

---

## 架构说明

### 包职责分层

```text
┌──────────────────────────────────────────────────────────────┐
│  Director / 上层应用（外部调用方）                             │
├──────────────────────────────────────────────────────────────┤
│  commander/  — Arm Commander 中间层                           │
│              ArmMoveToPose / ArmTrajectoryShot Action Server  │
│              状态机（IDLE / MOVING / REACHED / ERROR）         │
├──────────────────────────────────────────────────────────────┤
│  controllers/ + vision/  — 控制模式节点 & IBVS 视觉伺服       │
├──────────────────────────────────────────────────────────────┤
│  robot_arm_interfaces  — 自定义消息 / 服务 / 动作定义          │
├──────────────────────────────────────────────────────────────┤
│  robot_arm_driver  — C++ 硬件驱动（arm_node / CAN）           │
├──────────────────────────────────────────────────────────────┤
│  Linux SocketCAN（can0）                                      │
└──────────────────────────────────────────────────────────────┘
  robot_arm_description  — URDF / SRDF / mesh / RViz（静态资源）
  robot_arm_bringup      — launch / MoveIt config / sim（启动入口）
  robot_arm_matlab       — MATLAB 离线分析工具
```

### 关节划分

| 关节 | 驱动方式 | 控制器 |
| :--- | :--- | :--- |
| Joint1–Joint3 | RB200-CA CANopen 电机（SocketCAN） | `arm_node`（JointTrajectory Action Server） |
| Joint4–Joint6 | HID 相机云台（`robot_gimbal` 包） | `ros2_control` / `robot_gimbal_driver/CameraHardwareInterface` |

> **实物启动说明：** `real.launch.py` 同时启动 `robot_camera_node`（`robot_gimbal_node` 包）提供视频流，该节点仅占用 V4L2，与 ros2_control HID 接口无冲突。

### 核心话题 & 动作

| 名称 | 类型 | 用途 |
| :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态反馈 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 轨迹命令输入 |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` Action | MoveIt 轨迹执行 |
| `/gimbal_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 相机云台命令 |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 相机视频流（来自 robot_gimbal_node） |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | MoveIt Servo 速度输入 |
| `/arm_vel_cmd` | `geometry_msgs/TwistStamped` | IBVS / 速度控制指令输入 |
| `/robot_arm/arm_status` | `robot_arm_interfaces/ArmStatus` | Commander 状态反馈（10 Hz） |
| `/robot_arm/arm_move_to_pose` | `robot_arm_interfaces/ArmMoveToPose` Action | 单点位移指令（Commander） |
| `/robot_arm/arm_trajectory_shot` | `robot_arm_interfaces/ArmTrajectoryShot` Action | 多段运镜轨迹指令（Commander） |
| `/compute_ik` | `moveit_msgs/GetPositionIK` Service | 逆运动学求解 |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` Service | 笛卡尔路径规划 |
| `/execute_trajectory` | `moveit_msgs/ExecuteTrajectory` Action | 轨迹执行 |

---

## 主要依赖

依赖分三层：**系统基础** → **ROS 2（apt）** → **Python（pip / venv）**。
ROS 包由 `source /opt/ros/humble/setup.bash` 提供；Python 第三方库装在
`--system-site-packages` 的 venv 中，叠加在系统 ROS 之上。

### 系统基础

| 项目 | 要求 |
| :--- | :--- |
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| Python | 3.10 |
| 编译 | `colcon` + `ament_cmake`；接口包用 `rosidl` |
| C++ 驱动底层 | SocketCAN（内核自带）+ 随包编译的 `robot_arm_driver/src/CANopenLinux/` 协议栈 |

### ROS 2 依赖（apt，不进 venv）

| 依赖 | 用途 |
| :--- | :--- |
| `rclcpp` / `rclpy` / `rclcpp_action` | ROS 2 客户端库 |
| 消息：`sensor_msgs` `std_msgs` `std_srvs` `geometry_msgs` `trajectory_msgs` `control_msgs` `diagnostic_msgs` `moveit_msgs` | 接口/通信 |
| `ros-humble-moveit`（`moveit_core` / `moveit_ros_planning`） | 运动规划与 MoveIt 集成 |
| `pick_ik` | IK 求解器 |
| `ros2_control` / `ros2_controllers` | 控制器管理（Gazebo / 实物） |
| `gazebo_ros` / `gazebo_ros2_control` | Gazebo ↔ ROS 2 桥接 |
| `cv_bridge` / `image_transport` | 图像采集与处理 |
| `tf2_ros` / `ros-humble-tf-transformations` / `message_filters` | TF 变换与消息同步 |
| `robot_state_publisher` / `xacro` | 模型发布 |
| `launch` / `launch_ros` / `ament_index_python` | 启动系统 |
| `robot_gimbal_driver` | 相机云台 ros2_control 插件（Joint4-6） |
| `python3-tk`（apt） | tkinter GUI 框架 |

### Python 依赖（pip，见 [`requirements.txt`](requirements.txt)）

已按实际 `import` 精简：`scipy` / `matplotlib` / `pinocchio` 在 robot_arm 中
无任何引用，**不需要**（它们存在于共享 venv 仅因别的项目）。

**核心运行依赖**（机械臂控制 / 视觉 / GUI 必需）：

| 依赖 | 版本 | 用途 | 子包 |
| :--- | :--- | :--- | :--- |
| `numpy` | 1.26.4（**必须 <2**） | 数值计算 | 全部 |
| `PyYAML` | 5.4.1 | 配置读取 | node |
| `ruckig` | 0.17.3 | 在线轨迹生成（jerk-limited OTG） | node |
| `opencv-python-headless` | 4.13.0.92 | `cv2` 视觉 / IBVS / 红盒检测 | node |
| `PyQt5` | 5.15.11 | 电机测试 / 滑块控制 GUI | driver / node |

**可选依赖**（按需安装；不跑对应功能可不装）：

| 依赖 | 版本 | 用途 | 子包 |
| :--- | :--- | :--- | :--- |
| `mujoco` | 3.8.1 | MuJoCo 仿真（只跑实物/Gazebo 可不装） | node |
| `gymnasium` | 1.0.0 | RL 环境 | rl |
| `stable-baselines3` | 2.4.1 | SAC 训练 | rl |
| `tensorboard` | 2.20.0 | 训练日志 | rl |
| `torch` | 2.12.0（默认 CPU 版） | RL 后端 | rl |

---

## 环境配置

提供一键脚本，自动创建 `--system-site-packages` 的 venv 并安装上表的 pip 依赖：

```bash
# 默认在工作区根目录使用 .venv（复用现有共享 venv）
bash src/E7009/robot_arm/setup_venv.sh

# 可选：指定 venv 路径 / ROS setup
VENV_DIR=/path/to/venv ROS_SETUP=/opt/ros/humble/setup.bash \
  bash src/E7009/robot_arm/setup_venv.sh
```

脚本会：source ROS Humble → 创建/复用 venv（带 `--system-site-packages`，
保证 `rclpy` 等可用）→ 安装 CPU 版 `torch` + `requirements.txt` → 校验关键库
可 import 并强制检查 `numpy<2`。apt 前置依赖（moveit / tf-transformations /
ros2-control / gazebo 等）需提前装好，脚本只检查不自动安装。

> 注：`torch` 默认装 CPU 版以复现已验证环境；如需 GPU，改用对应 CUDA 轮子。

使用环境时（每个新终端）：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash   # 编译后才能用本工作区的包
```

---

## 编译

```bash
# 编译所有机械臂相关包
colcon build --symlink-install --packages-select \
  robot_arm_interfaces robot_arm_driver robot_arm_description \
  robot_arm_bringup robot_arm_node

# 生效
source install/setup.bash
```

---

## Arm Commander 接口参考

Arm Commander 是机械臂的中间层状态机，对外暴露两个 Action 接口：
`ArmMoveToPose`（单点位移）和 `ArmTrajectoryShot`（多段运镜轨迹）。

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
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 2, transition_speed: 1, return_to_start: false,
    target_pose: {x: 0.30, y: 0.00, z: 0.50, roll: 90.0, pitch: 10.0, yaw: 0.0}}"

# 到位后自动返回起点
ros2 action send_goal /robot_arm/move_to_pose \
  robot_arm_interfaces/action/ArmMoveToPose \
  "{target_pose_state: 1, transition_speed: 1, return_to_start: true}"
```

### ArmTrajectoryShot — 轨迹运镜

接口：`/robot_arm/trajectory_shot`，运动类型：`motion_type` — 0=LINEAR  1=ORBIT

```bash
# 直线运镜 LINEAR：从起始位姿平滑运动到终止位姿
ros2 action send_goal /robot_arm/trajectory_shot \
  robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 0, transition_speed: 1, return_to_start: false,
    linear_start_pose: {x: 0.30, y: 0.0, z: 0.60, roll: 90.0, pitch: 0.0, yaw: 0.0},
    linear_end_pose:   {x: 0.30, y: 0.0, z: 0.40, roll: 90.0, pitch: 0.0, yaw: 0.0}}"

# 球面环绕运镜 ORBIT：末端始终朝向球心
ros2 action send_goal /robot_arm/trajectory_shot \
  robot_arm_interfaces/action/ArmTrajectoryShot \
  "{motion_type: 1, transition_speed: 1, return_to_start: false,
    orbit_center_x: 0.60, orbit_center_y: 0.00, orbit_center_z: 0.50,
    azimuth_start_deg: -30.0, elevation_start_deg: -10.0, radius_start_m: 0.45,
    azimuth_end_deg:    30.0, elevation_end_deg:   30.0,  radius_end_m:   0.20}"
```

ORBIT 参数说明：

| 参数 | 含义 | 单位 |
| :--- | :--- | :--- |
| `orbit_center_x/y/z` | 被摄主体位置（球心） | m |
| `azimuth_start/end_deg` | 起止水平方位角 | ° |
| `elevation_start/end_deg` | 起止俯仰角 | ° |
| `radius_start/end_m` | 起止半径（可实现变焦距效果） | m |

`return_to_start: true` 可加入任意轨迹命令，执行完毕后自动返回起点。

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
| `command_result: 3` FAILED | IK 无解 | 调整目标位姿，避免奇异点或工作空间边界 |
