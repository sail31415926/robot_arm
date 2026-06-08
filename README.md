# eMeet Robot Arm — ROS 2 Package Family (Humble)

## 项目信息

| 项目属性 | 详情 |
| :--- | :--- |
| **项目名称** | eMeet Robot Arm ROS 2 Package Family |
| **版本** | v1.2.0 |
| **发布日期** | 2026-06-05 |
| **支持平台** | Ubuntu 22.04 LTS |
| **ROS 2 版本** | Humble |
| **设备兼容** | eMeet 机械臂（6 轴：Joint1-3 CANopen + Joint4-6 相机云台） |
| **编程语言** | C++ / Python / MATLAB |

---

## 包结构

```bash
src/E7009/robot_arm/
│
├── robot_arm_interfaces/           # 消息定义包
│   └── msg/
│       ├── ArmCommand.msg
│       ├── ArmFollowCommand.msg
│       ├── ArmMotionCommand.msg
│       ├── ArmPoseCommand.msg      # 位姿指令（含 pan_deg / tilt_deg）
│       └── ArmStatus.msg
│
├── robot_arm_driver/               # C++ 硬件驱动包
│   ├── include/canopen_motor_driver/
│   │   ├── canopen_motor_driver.hpp    # CiA402 CANopen 驱动器接口（纯 C++，无 ROS 依赖）
│   │   └── motor_unit_converter.hpp   # rad ↔ encoder counts 单位转换
│   ├── src/
│   │   ├── canopen_motor_driver.cpp   # SocketCAN CiA402 实现（SDO/PDO/NMT/心跳）
│   │   ├── arm_node.cpp               # 3 关节集成控制节点（JointTrajectory Action Server）
│   │   ├── arm_motor_node.cpp         # 单关节 CANopen 控制节点
│   │   ├── set_encoder_zero.cpp       # 编码器零点校准工具
│   │   ├── set_position_limit.cpp     # 软件位置限位工具
│   │   └── CANopenLinux/              # CANopen Linux 协议栈（含 CANopenNode 子库）
│   ├── config/
│   │   ├── arm.yaml                   # arm_node 参数（CAN 接口、编码器分辨率、力矩限制）
│   │   └── motors.yaml                # arm_motor_node 每关节参数（node_id、加减速、心跳）
│   └── scripts/
│       └── motor_test_gui.py          # 电机测试 GUI（PyQt5）
│
├── robot_arm_description/          # 机械臂模型包（静态资源）
│   ├── urdf/
│   │   ├── eMeetArm_models.urdf       # 主 URDF 模型
│   │   └── eMeetArm_models.csv        # 运动学链参数表
│   ├── srdf/
│   │   └── eMeetArm_models.srdf       # 语义机器人描述（碰撞组、零位）
│   ├── xml/
│   │   └── eMeetArm.xml               # MuJoCo MJCF 模型（robot_arm_description 版本）
│   ├── meshes/                        # 3D 网格（base_link + Link1-6.STL）
│   ├── config/
│   │   ├── controllers.yaml           # ros2_control 控制器配置
│   │   ├── joint_limits.yaml          # 关节速度/加速度限制
│   │   ├── kinematics.yaml            # IK 求解器配置（pick_ik）
│   │   └── joint_names_eMeetArm_models.yaml
│   └── rviz/
│       ├── eMeetArm_models.rviz       # URDF 可视化配置
│       └── moveit.rviz                # MoveIt 规划配置
│
├── robot_arm_bringup/              # 启动 & 配置 & 仿真资源包
│   ├── launch/
│   │   ├── display.launch.py          # URDF 可视化
│   │   ├── motor.launch.py            # 真实硬件：起 3 个 arm_motor_node（Joint1-3）
│   │   ├── real.launch.py             # 真实硬件：一键启动（arm_node + ros2_control + MoveIt + GUI + 视频流）
│   │   ├── gazebo.launch.py           # Gazebo 仿真：物理引擎 + ros2_control + 9 种控制模式
│   │   ├── mujoco.launch.py           # MuJoCo 仿真 + MoveIt
│   │   ├── moveit.launch.py           # 独立 MoveIt2（move_group + RViz）
│   │   ├── camera_view.launch.py      # 相机图像查看
│   │   ├── red_box_detect.launch.py   # 红色方块识别（独立启动）
│   │   └── test_ros.launch.py         # 基础 ROS 2 通信测试
│   ├── config/
│   │   └── moveit/
│   │       ├── planning_pipeline.yaml          # OMPL 运动规划器配置
│   │       ├── servo_config.yaml               # MoveIt Servo 配置
│   │       ├── moveit_controllers.yaml         # Gazebo 控制器映射
│   │       ├── moveit_controllers_mujoco.yaml  # MuJoCo 控制器映射
│   │       └── moveit_controllers_real.yaml    # 真实硬件控制器映射
│   └── sim/
│       ├── gazebo/
│       │   ├── worlds/emeet_arm.world           # Gazebo 世界文件
│       │   └── models/aruco_marker_0/           # ArUco 标记模型
│       └── mujoco/
│           ├── eMeetArm.xml                     # MuJoCo MJCF 模型（bringup 版本）
│           └── orbit_camera_pose.html           # 球面轨道相机位姿可视化
│
├── robot_arm_node/                 # 应用层控制节点包
│   ├── src/main.cpp                   # 主节点入口
│   ├── scripts/
│   │   ├── arm_utils.py               # 通用四元数数学工具（rpy_to_quat / quat_slerp 等）
│   │   ├── controllers/
│   │   │   ├── arm_slider_controller.py          # 关节滑块控制 GUI（PyQt5）
│   │   │   ├── cartesian_controller.py           # 笛卡尔精确路径控制 GUI（tkinter）
│   │   │   ├── cartesian_realtime_controller.py  # 实时笛卡尔 IK 控制 GUI（tkinter）
│   │   │   ├── cartesian_ruckig_streamer.py      # Ruckig OTG + MoveIt Servo（TwistStamped 流控）
│   │   │   ├── cartesian_ruckig_ik_streamer.py   # Ruckig OTG + 批量 IK + JointTrajectory
│   │   │   ├── cartesian_velocity_controller.py  # 笛卡尔速度控制器（IBVS 接口）
│   │   │   └── spherical_orbit_streamer.py       # 球面轨道相机对中控制
│   │   ├── simulation/
│   │   │   ├── mujoco_node.py                    # MuJoCo ↔ ROS 2 桥接节点
│   │   │   └── mujoco_data_recorder.py           # MuJoCo 仿真数据采集（IRIS 训练格式）
│   │   ├── tools/
│   │   │   ├── arm_trajectory_bridge.py          # 轨迹拆分桥接（Joint1-3→CAN + Joint4-6→云台）
│   │   │   └── pose_command_debug.py             # ArmPoseCommand 执行器（无 GUI）
│   │   └── vision/
│   │       ├── ibvs_controller.py                # IBVS 视觉伺服（红色方块居中）
│   │       └── red_box_detector.py               # 红色方块识别与特征发布
│   ├── tests/
│   │   └── pose_command_publisher.py             # ArmPoseCommand 发布测试 GUI
│   └── third_party/
│       └── IBVS_Controller/                      # IBVS 核心算法库
│
└── robot_arm_matlab/               # MATLAB 工具箱（离线分析与规划）
    ├── eMeetArm_cartesian_gui.m       # 笛卡尔空间 GUI 控制
    ├── eMeetArm_cartesian_traj.m      # 笛卡尔轨迹规划
    ├── eMeetArm_joint_control.m       # 关节空间控制
    ├── eMeetArm_singularity.m         # 奇异性分析
    ├── eMeetArm_workspace*.m          # 工作空间分析（边界 / 椭球 / IK / 可视化）
    ├── RTB.mltbx                      # Robotics Toolbox 工具箱文件
    ├── eMeetArm_MATLAB指南.md
    └── eMeetArm_models/               # Simscape Multibody 导出的 ROS 2 包
```

---

## 快速启动

### URDF 可视化

```bash
ros2 launch robot_arm_bringup display.launch.py
```

### Gazebo 仿真

```bash
# 默认滑块控制模式
ros2 launch robot_arm_bringup gazebo.launch.py

# 指定控制模式
ros2 launch robot_arm_bringup gazebo.launch.py controller:=cartesian
ros2 launch robot_arm_bringup gazebo.launch.py controller:=ruckig
ros2 launch robot_arm_bringup gazebo.launch.py controller:=ibvs_control
ros2 launch robot_arm_bringup gazebo.launch.py controller:=pose_command_debug
```

### MuJoCo 仿真

```bash
ros2 launch robot_arm_bringup mujoco.launch.py
ros2 launch robot_arm_bringup mujoco.launch.py controller:=cartesian
```

### 真实硬件

```bash
# 一键启动（arm_node + ros2_control + MoveIt + GUI + 视频流）
ros2 launch robot_arm_bringup real.launch.py

# 指定控制模式
ros2 launch robot_arm_bringup real.launch.py controller:=sphere_orbit
ros2 launch robot_arm_bringup real.launch.py controller:=ruckig_ik

# 指定相机型号
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

# ArmPoseCommand 调试（需分两个终端）
ros2 launch robot_arm_bringup gazebo.launch.py controller:=pose_command_debug
ros2 run robot_arm_node pose_command_publisher
```

---

## 控制模式一览

| 模式 | 脚本 | 控制机制 | GUI | 支持环境 |
| :--- | :--- | :--- | :---: | :--- |
| **slider** | `controllers/arm_slider_controller.py` | 直接发布 JointTrajectory | PyQt5 滑块 | Gazebo / MuJoCo / 实物 |
| **cartesian** | `controllers/cartesian_controller.py` | MoveIt `compute_cartesian_path` + `execute_trajectory` | tkinter | Gazebo / MuJoCo / 实物 |
| **realtime** | `controllers/cartesian_realtime_controller.py` | 实时 IK（`/compute_ik`）→ JointTrajectory | tkinter 滑块 | Gazebo / MuJoCo / 实物 |
| **ruckig** | `controllers/cartesian_ruckig_streamer.py` | Ruckig OTG → TwistStamped → MoveIt Servo | tkinter | Gazebo / MuJoCo |
| **ruckig_ik** | `controllers/cartesian_ruckig_ik_streamer.py` | Ruckig OTG → 批量 IK → JointTrajectory | tkinter | Gazebo / MuJoCo / 实物 |
| **sphere_orbit** | `controllers/spherical_orbit_streamer.py` | Ruckig 球面轨迹 → IK → JointTrajectory（相机始终对中） | tkinter | Gazebo / MuJoCo / 实物 |
| **velocity** | `controllers/cartesian_velocity_controller.py` | TwistStamped → MoveIt Servo / MuJoCo DLS | tkinter 点动 | Gazebo / MuJoCo |
| **ibvs_control** | `vision/ibvs_controller.py` + `vision/red_box_detector.py` | IBVS 视觉闭环 → TwistStamped | — | Gazebo |
| **pose_command_debug** | `tools/pose_command_debug.py` | 订阅 `ArmPoseCommand` → Ruckig IK 执行 | 配合 publisher GUI | Gazebo |

---

## 架构说明

### 包职责分层

```text
┌──────────────────────────────────────────────────────┐
│  robot_arm_node  — 应用层控制脚本（Python GUI / 视觉） │
├──────────────────────────────────────────────────────┤
│  robot_arm_interfaces  — 自定义消息定义               │
├──────────────────────────────────────────────────────┤
│  robot_arm_driver  — C++ 硬件驱动（arm_node / CAN）   │
├──────────────────────────────────────────────────────┤
│  Linux SocketCAN（can0）                              │
└──────────────────────────────────────────────────────┘
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
| `/arm_vel_cmd` | `geometry_msgs/TwistStamped` | IBVS 速度指令输入 |
| `/robot_arm/pose_command` | `robot_arm_interfaces/ArmPoseCommand` | 位姿状态机指令 |
| `/compute_ik` | `moveit_msgs/GetPositionIK` Service | 逆运动学求解 |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` Service | 笛卡尔路径规划 |
| `/execute_trajectory` | `moveit_msgs/ExecuteTrajectory` Action | 轨迹执行 |

---

## 主要依赖

| 依赖 | 用途 |
| :--- | :--- |
| `rclcpp` / `rclpy` | ROS 2 客户端库 |
| `moveit_core` / `moveit_ros_planning` | 运动规划与 MoveIt 集成 |
| `pick_ik` | IK 求解器 |
| `ruckig` | 在线轨迹生成（jerk-limited OTG，pip 安装） |
| `ros2_control` | 控制器管理（Gazebo / 实物） |
| `gazebo_ros` | Gazebo ↔ ROS 2 桥接 |
| `mujoco` | 物理仿真（pip 安装） |
| `robot_gimbal_driver` | 相机云台 ros2_control 插件（Joint4-6） |
| OpenCV（`cv_bridge`、`image_transport`） | 图像采集与处理 |
| Qt5 / tkinter | GUI 框架 |

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
