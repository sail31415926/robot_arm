# robot_arm — ROS 2 Package (Humble)

## 项目信息

| 项目属性 | 详情 |
| :--- | :--- |
| **项目名称** | robot_arm ROS 2 Package |
| **版本** | v1.5.0 |
| **发布日期** | 2026-07-13 |
| **支持平台** | Ubuntu 22.04 LTS · ROS 2 Humble · Python 3.10 |
| **设备** | robot_arm 6 轴机械臂（Joint1-3 CANopen + Joint4-6 云台 V2：C-200T 三轴 GCU，串口，执行节点在云台板端） |
| **语言** | C++ / Python / MATLAB |

---

## 包结构

```text
robot_arm/
├── robot_arm_interfaces/    # 自定义 msg / srv / action（ArmStatus、ArmMoveToPose、ArmMoveToJoint、ArmTrajectoryShot…）
├── robot_arm_description/   # 纯数据包：URDF / mesh / MJCF / RViz 显示配置
├── robot_arm_moveit_config/ # MoveIt 配置包（社区标准布局）：SRDF / kinematics(pick_ik) / joint_limits /
│                            #   OMPL 管线 / 三后端 moveit_controllers / servo 配置 / moveit.launch.py
├── robot_arm_driver/        # CANopen 驱动（J1-3，ros2_canopen）：bus.yml/EDS + arm_driver_services + 标定工具
├── robot_arm_node/          # ★产品层：C++ 产品栈（motion / state / commander，arm_commander_node）
├── robot_arm_debug/         # ☆调试层：Python 调试控制器 / GUI / 工具 + 视觉感知（visp_ibvs、红块检测，暂归调试）
├── robot_arm_bringup/       # 统一启动入口（bringup/real launch）+ ros2_control 控制器配置 + launch_common 共享工厂
├── robot_arm_gazebo/        # Gazebo 仿真后端：gazebo.launch.py + worlds / models 资产
├── robot_arm_mujoco/        # MuJoCo 仿真后端：mujoco.launch.py + mujoco_node 仿真桥
└── robot_arm_matlab/        # MATLAB 离线分析工具箱（Simscape 导出）
```

---

## 编译

> 前置：ROS + venv 环境先就绪（见下文「环境配置」）；每个新终端按顺序
> `source /opt/ros/humble/setup.bash` → `source .venv/bin/activate`。
>
> **包名必须跟在选择参数后面**，直接 `colcon build xxx包名` 会报
> `unrecognized arguments`。两个常用选择参数：
> `--packages-select`（只编列出的包本身，依赖须已编过——日常增量用）、
> `--packages-up-to`（连带所有未编译的依赖一起构建——新环境首次编译用，
> 记得 `--parallel-workers 2` 限流）。

### 编译（开发机完整版：调试 GUI + 仿真后端 + 实机）

```bash
colcon build --symlink-install --packages-select \
  robot_arm_interfaces robot_arm_driver robot_arm_description robot_arm_moveit_config \
  robot_arm_bringup robot_arm_node robot_arm_debug robot_arm_gazebo robot_arm_mujoco \
  robot_gimbal_interfaces_v2 robot_gimbal_driver_v2 robot_gimbal_description_v2

# 生效（注意：编译出新包后每个已开终端都要重新 source）
source install/setup.bash
```

### 实机编译（板上部署最小集）

只需 6 个 robot_arm 包 + 云台 V2 的 3 包（接口/转发插件/描述，J4-6 依赖，
另仓库 `robot_gimbal_V2`）：

```bash
colcon build --symlink-install --packages-select \
  robot_arm_interfaces robot_arm_driver robot_arm_description robot_arm_moveit_config \
  robot_arm_bringup robot_arm_node \
  robot_gimbal_interfaces_v2 robot_gimbal_driver_v2 robot_gimbal_description_v2

source install/setup.bash
```

> **`robot_gimbal_node_v2` 不在臂侧编译清单里**（这是 V2 与 V1 最大的部署差异）：
> 云台执行节点是串口唯一拥有者，跑在**云台板端**（LubanCat，`/dev/ttyS0`），
> 臂侧只经话题与它收发（板端**原生**订阅 `forward_cmd` / 发布 `joint_states_raw`，
> 转发插件直连，不需要中间适配节点）。臂侧要的是接口（`robot_arm_driver` 编译期依赖）、
> 转发插件（URDF 加载）、描述包（`arm.urdf.xacro` include 云台 macro）三样。
> 因此**臂侧与云台板是两台机器**，必须同网段 + 同 `ROS_DOMAIN_ID`
> 且**不能设** `ROS_LOCALHOST_ONLY=1`，否则回读收不到（转发插件会一直警告
> `No feedback from robot_gimbal_node yet`，退化为指令回显）。
>
> 适用 `real.launch.py controller:=commander gui:=false` 等产品路径。
> 若用**调试控制模式**（`joint_position` / `cartesian_*` / `spherical_orbit` 等
> GUI，含默认的 `controller:=joint_position`）或 `visp_ibvs`、commander 测试
> GUI，**另需编译 `robot_arm_debug`**，否则 launch 在 t=3s 拉起 GUI 时报
> `package 'robot_arm_debug' not found` 整体退出。
>
> **全新工作空间首次编译**：`robot_arm_driver` 依赖 vendored 的 ros2_canopen
>（约 10 个包），`--packages-select` 不会自动构建依赖——首次请改用
> `--packages-up-to robot_arm_bringup robot_arm_node robot_gimbal_driver_v2`
>（并限制并行度，编译很重）；日常增量编译用上面的列表即可。

---

## 快速启动

三种运行环境共用同一套接口，均以 `controller:=<模式>` 选控制方式（模式见下表，默认 `joint_position`）：

```bash
ros2 launch robot_arm_bringup display.launch.py    # 仅 RViz 显示 URDF
ros2 launch robot_arm_gazebo gazebo.launch.py     # Gazebo 仿真
ros2 launch robot_arm_mujoco mujoco.launch.py     # MuJoCo 仿真
ros2 launch robot_arm_bringup real.launch.py       # 实物（ros2_control HAL + 云台 + MoveIt）
ros2 launch robot_arm_moveit_config moveit.launch.py   # 单独 move_group + RViz
ros2 launch robot_arm_driver test_arm.launch.py    # 驱动层自测（mock/vcan/真机，不含 MoveIt）
```

```bash
# 选控制模式 / 开关调试 GUI（示例）
ros2 launch robot_arm_gazebo gazebo.launch.py controller:=spherical_orbit
ros2 launch robot_arm_bringup real.launch.py   controller:=commander      # 中间层（默认含测试 GUI）
ros2 launch robot_arm_bringup real.launch.py   controller:=commander gui:=false   # 无 GUI（板上部署）
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
| `commander` | `arm_commander_node` (C++) | 状态机 + 4 Action（见下节） | Gazebo / MuJoCo / 实物 |

> 前 6 种为 Python 调试模式（节点被 GUI 进程内 import，故保留 Python）；`commander`（产品中间层）与 `visp_ibvs_control`（视觉伺服）为 C++ 产品栈。
> ¹ 实物后端该模式参数名为 `visp_ibvs`（见 `real.launch.py`）。
>
> **注意：`controller:=` 与「控制模式」是两个正交的轴。** `controller:=` 选的是**上层控制方式**
> （哪个 GUI / 规划器 / 中间层，多数产出 JointTrajectory）；下面的 `ControlMode` 是**底层作动模式**
> （position / velocity / effort，即哪个 ros2_control 控制器 active）。上表大多数运行在 `TRAJECTORY` 模式下。

---

## 控制模式仲裁（ModeManager）

**「位置 / 速度 / 力矩」是一根贯穿各层的正交维度**：驱动器层是 CiA402 的 IP/PV/PT，ros2_control 层是
`position`/`velocity`/`effort` 命令接口，应用层是不同任务意图。`RobotSystem` 硬约束「每关节同一时刻只能
claim 一个命令接口 = 只能处于一种 402 模式」，故**模式切换 = ros2_control 控制器切换**。

`mode_manager_node`（C++，`robot_arm_node`，`real`/`gazebo` 常驻）是**语义控制模式的唯一权威、后端无感**：
上层只调一个服务切模式，不关心底层控制器名 / 402 模式。

| ControlMode | 控制器（active） | 402 模式 | 命令总线（上层发） | 状态 |
| :--- | :--- | :--- | :--- | :--- |
| `TRAJECTORY`（0，默认） | `arm_controller`(JTC) | IP(7) | `/arm_controller/joint_trajectory` | ✅ |
| `JOINT_VELOCITY`（1） | `arm_velocity_controller` | PV(3) | `/robot_arm/cmd/joint_velocity`（Float64MultiArray，J1-3，带看门狗） | ✅ |
| `JOINT_EFFORT`（2） | `arm_effort_controller` | PT(4) | `/robot_arm/cmd/joint_effort` | ⏳ P3（需 URDF 加 effort 接口） |
| `ADMITTANCE`（3） | `arm_controller`(JTC) | IP(7) | 末端力 → 位置微调，底层仍走位置总线 | ⏳ P3（需 F/T 传感器） |

```bash
# 查询当前模式（latched，随时可读）
ros2 topic echo /robot_arm/control_mode --once            # mode: 0=TRAJECTORY 1=VELOCITY ...
# 切换模式（唯一入口；必须静止时调用）
ros2 service call /robot_arm/switch_control_mode robot_arm_interfaces/srv/SwitchControlMode "{target_mode: 1}"
# 速度点动（切到 VELOCITY 后，发到产品总线；断流 >300ms 自动归零兜底）
ros2 topic pub -r 50 /robot_arm/cmd/joint_velocity std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0]}"
```

ModeManager 职责：① 唯一切换入口 `/robot_arm/switch_control_mode`（串行化）；② bumpless 播种（切到速度/力矩
前先喂 0，切回轨迹由 JTC 自动锁当前位姿）；③ 原子切换 `controller_manager/switch_controller`(STRICT)；
④ latched 广播 `/robot_arm/control_mode`（Commander 汇入 `arm_status.active_control_mode`）；⑤ 速度/力矩
总线看门狗（`ForwardCommandController` 不自动归零，断流即失控，必须兜底）。

> **约定**：必须静止时切换；速度/力矩模式下**云台 J4-6 不受控**（`arm_controller` 被停），保持当前位置。
> 驱动层 `/arm_node/set_mode_pp|ip`（402 profile 微调，不换控制器）仍是实物专属细化，与 ModeManager 不冲突。
> `JOINT_EFFORT` / `ADMITTANCE` 为 P3 预留（`effort_controller` 默认未配置，切换会被明确拒绝）。

---

## 架构说明

整体设计的核心：**所有上层节点把命令汇聚到一条命令总线，所有后端把状态汇聚到一条反馈总线**。
Gazebo / MuJoCo / 实物三套后端共用同一对总线接口，上层控制逻辑对「跑仿真还是跑实物」无感。

### 层级跨框架图

纵向是抽象层级（L0→L4），横向标注每层用到的框架；两条总线横穿所有框架，是解耦上层与后端的关键。

```text
═══════════════════════════════════════════════════════════════════════════════════
 L4 应用/指挥层           外部业务 Director（下发拍摄/姿态任务、读状态）
                          └ commander_test_gui —— 测试用 Director（走 ROS Action，4 面板：
                                                    status/MoveToPose/MoveToJoint/TrajectoryShot）
───────────────────────────────────────────────────────────────────────────────────
 L3 产品中间层            arm_commander_node (C++)  单一状态机 IDLE/MOVING/REACHED/…
   [ROS2 Action/Srv/Topic]  ├ move_to_pose_server     姿态切换（关节 / IK 目标）
                            ├ move_to_joint_server    关节空间点到点（只动臂 J1-3）
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
   ⊕ 模式仲裁  mode_manager_node   /robot_arm/switch_control_mode (Srv) · /robot_arm/control_mode (latched)
                                   切模式 = switch_controller（TRAJECTORY↔VELOCITY↔EFFORT，后端无感）
   ▼ 命令总线  [位置] /arm_controller/joint_trajectory       (trajectory_msgs/JointTrajectory)
              [位置] /arm_controller/follow_joint_trajectory (Action，MoveIt 执行轨迹用)
              [速度] /robot_arm/cmd/joint_velocity          (Float64MultiArray，J1-3，ModeManager relay+看门狗)
   ▲ 反馈总线  /joint_states                               (sensor_msgs/JointState, 6 轴)
═══════════════════════════════════════════════════════════════════════════════════
 L1 控制/后端层（三选一，总线接口一致，上层无感切换）
   [Gazebo]  gazebo_ros2_control → arm_controller (JTC, 6 轴) → 物理仿真
   [MuJoCo]  mujoco_node (py 桥)   订阅轨迹 / Servo，回填 joint_states + 相机图像
   [Real]    ros2_control 单 CM：J1-3 → RobotSystem（CANopen）
             J4-6 → GimbalForwardingInterface（转发，变化检测）
                    → 〖跨机 DDS〗→ 云台板端 robot_gimbal_node_v2（串口独占）
───────────────────────────────────────────────────────────────────────────────────
 L0 物理/驱动     Gazebo 物理引擎   │   MuJoCo 物理   │   SocketCAN can0 + 云台板端 GCU 串口
═══════════════════════════════════════════════════════════════════════════════════
```

> **实物 J1-3 驱动路径（唯一）：** ros2_control HAL —— `canopen_ros2_control/RobotSystem`
> （ros2_canopen，CiA402/SocketCAN，总线配置见 `robot_arm_driver/arm_driver/config/canopen/bus.yml`），
> 加上 `arm_controller`(JTC) 与伴生 `arm_driver_services`；`real.launch.py` 单 controller_manager 管全 6 轴。
> 驱动层自测：`ros2 launch robot_arm_driver test_arm.launch.py`（mock/vcan 假从站/真机三合一，
> 详见 docs/ros2_canopen迁移.md）。旧 `arm_node`（自研 CANopenLinux 栈）已于 2026-07 下线。

### 关节划分

实物为单 controller_manager + 单 `arm_controller`(JTC) claim 全 6 轴（跨两个硬件组件），
命令入口与 Gazebo/MuJoCo 完全一致：`/arm_controller/joint_trajectory`（支持部分关节轨迹）。

| 关节 | 驱动（硬件组件） | 反馈来源 | 命令入口 |
| :--- | :--- | :--- | :--- |
| Joint1–3 | `canopen_ros2_control/RobotSystem`（ros2_canopen，CiA402/SocketCAN） | `joint_state_broadcaster` → `/joint_states` | `/arm_controller/joint_trajectory` |
| Joint4–6 | `robot_gimbal_driver_v2/GimbalForwardingInterface`（转发插件，不碰串口）→〖跨机 DDS〗→ 云台板端 `robot_gimbal_node_v2`（唯一串口拥有者，真实回读） | `joint_state_broadcaster` → `/joint_states`（J4-6 为板端真实回读；跨机不通时退化为开环回显） | `/arm_controller/joint_trajectory`（同一控制器） |

> **到位判据只认臂 J1-3。** Commander 的所有等待循环（`ArmMoveToPose` / `ArmTrajectoryShot` /
> `ArmMoveToJoint`）都用「臂关节到达本段轨迹终点解」判定成败，**不用末端位姿** —— 末端
> `gimbal_tool0` 在云台 J4-6 之后，云台回读不收敛（板端未上电 / 跨机 DDS 不通 / GCU 自稳
> 环静差）会让笛卡尔判据永不满足，动作全部超时报错。云台仍随 6 轴轨迹一起规划下发，
> 只是不参与判定。

### 各节点职责与数据传输

| 节点（可执行） | 包·语言 | 职责 | 关键输入 → 输出 |
| :--- | :--- | :--- | :--- |
| `arm_commander_node` | robot_arm_node · C++ | 产品中间层状态机，把「拍摄/姿态」意图翻译成轨迹 | `/robot_arm/{move_to_pose,move_to_joint,trajectory_shot,track_target}` Action、`/joint_states`、`/compute_ik` → `/arm_controller/joint_trajectory`、`/robot_arm/arm_status` |
| `mode_manager_node` | robot_arm_node · C++ | 语义控制模式仲裁（TRAJECTORY/JOINT_VELOCITY/…），后端无感；切模式 = switch_controller + bumpless 播种 + 速度总线看门狗 | `/robot_arm/switch_control_mode`(Srv)、`/robot_arm/cmd/joint_velocity` → `controller_manager/switch_controller`、`/arm_velocity_controller/commands`、`/robot_arm/control_mode`(latched) |
| `controllers ×6` | robot_arm_debug · py | 调试控制：关节滑块 / 笛卡尔 / 实时 IK / Ruckig 点到点 / 球面运镜 / 速度点动 | GUI 滑块、`/joint_states`、`/compute_ik`·`/compute_cartesian_path` → `/arm_controller/joint_trajectory`（`cartesian_velocity` 改发 `/servo_node/delta_twist_cmds`） |
| `visp_ibvs_node` | robot_arm_debug · C++ | ViSP+Pinocchio 图像伺服，加权 Jacobian 直接算关节速度 | `/red_detector/feature`（或外部 `perception_topic`）、`/joint_states` → `/arm_controller/joint_trajectory` |
| `red_box_detector` | robot_arm_debug · py | OpenCV 红块检测，产出归一化像素特征 + 深度 | `/camera/camera_sensor/image_raw` → `/red_detector/feature`、`/red_detector/image` |
| `mujoco_node` | robot_arm_mujoco · py | MuJoCo 仿真桥：物理步进 + 相机渲染 | `/arm_controller/joint_trajectory`、`/servo_node/delta_twist_cmds` → `/joint_states`、`/camera/camera_sensor/image_raw` |
| `move_group` | MoveIt | 运动规划 / IK / 笛卡尔路径 | 规划请求 → `/compute_ik`、`/compute_cartesian_path` 服务；`/arm_controller/follow_joint_trajectory` 执行 |
| `servo_node` | MoveIt Servo | 实时笛卡尔速度 → 关节增量 | `/servo_node/delta_twist_cmds` → `/arm_controller/joint_trajectory` |
| `ros2_control_node` + `arm_controller`(JTC) | controller_manager · C++ | 【实物】单 CM 管全 6 轴：J1-3 CANopen（RobotSystem，position→IP 模式）+ J4-6 云台 V2（GimbalForwardingInterface 变化检测转发，见 docs/云台控制路径融合方案.md） | `/arm_controller/joint_trajectory`、`/arm_controller/follow_joint_trajectory` Action → CAN + `/robot_gimbal_v2/forward_cmd` + `/joint_states`(6 轴) |
| `arm_driver_services` | robot_arm_driver · C++ | 【实物】使能/失能/故障恢复服务（转发 controller_manager 硬件组件状态） | `/arm_node/{enable,disable,recover}`(Trigger) → CM 组件 active↔inactive |
| `arm_controller` (JTC) | Gazebo / ros2_control | 【Gazebo】6 轴关节轨迹控制器驱动仿真模型 | `/arm_controller/joint_trajectory` → 物理 + `/joint_states` |
| ~~`gimbal_v2_bridge`~~ | robot_arm_driver · C++ | 【2026-07-31 停止启用，源码保留】云台 V2 适配节点。板端 `robot_gimbal_node_v2` 已**原生**收发臂侧转发约定，再经它翻译一遍会双重驱动：命令送两遍，且它把「转发流」升级成 `GimbalCommand.POSITION`——板端 POSITION 会解冻 FROZEN，等于让 JTC 的保持流解冻 FREEZE，破坏仲裁优先级。其 `absolute_mode`（相对关节角 ⇄ IMU 绝对姿态角）若要重启用，须改成不与板端原生话题重叠的接法 | — |
| ~~`robot_camera_node`~~ | — | 【已下线】V1 二轴云台的 V4L2 视频流节点，随 2026-07-28 云台换代 V2 一并删除；V2 主相机（Cam0）由**云台板端**自行发布 | — |
| `robot_state_publisher` | ROS 2 | URDF + 关节角 → TF 树 | `/joint_states` → `/tf` |

### 数据流（三条链路）

- **命令下行**：上层节点（controllers / commander / visp / servo）统一发 `/arm_controller/joint_trajectory`；后端（Gazebo `arm_controller` / MuJoCo `mujoco_node` / 实物 `arm_controller`(单 CM 全 6 轴)）择一消费。MoveIt 规划结果走 `/arm_controller/follow_joint_trajectory` Action。
- **状态上行**：后端统一回填 `/joint_states`（6 轴，实物由单 `joint_state_broadcaster` 汇聚两个硬件组件）→ 供上层闭环、`robot_state_publisher` 出 TF、commander 汇聚成 `/robot_arm/arm_status`（10 Hz）。
- **视觉闭环**：相机图像（仿真 `/camera/camera_sensor/image_raw`，实物 `/camera/image_raw/compressed`）→ `red_box_detector` → `/red_detector/feature`（`geometry_msgs/PointStamped`，x/y 为归一化像素、z 为深度）→ `visp_ibvs_node` → 命令总线。

---

## 主要依赖

**系统基础**：Ubuntu 22.04 + ROS 2 Humble + Python 3.10；`colcon` / `ament_cmake` 编译（接口包用 `rosidl`）；C++ 驱动基于内核 SocketCAN + 随包编译的 CANopen 协议栈。

**ROS 2（apt，不进 venv）**：`rclcpp` / `rclpy` / `rclcpp_action`、消息包（`sensor_msgs` `geometry_msgs` `trajectory_msgs` `control_msgs` `moveit_msgs` 等）、`ros-humble-moveit` + `pick_ik`、`ros2_control` / `ros2_controllers`、`gazebo_ros` / `gazebo_ros2_control`、`cv_bridge` / `image_transport`、`tf2_ros` / `tf-transformations`、`robot_state_publisher` / `xacro`、`robot_gimbal_driver_v2`（云台 V2 转发插件，源码在 `robot_gimbal_V2` 仓库）、`python3-tk`。

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

## Arm Commander 接口参考

Arm Commander（`controller:=commander`）是中间层状态机，对外暴露 4 个 Action —— `ArmMoveToPose`（姿态切换，笛卡尔）、`ArmMoveToJoint`（关节空间点到点）、`ArmTrajectoryShot`（运镜轨迹）、`ArmTrackTarget`（视觉跟随），4 个 Service（`enable` / `stop` / `reset_error` / `homing`），并以 10 Hz 发布 `/robot_arm/arm_status`。

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

### ArmMoveToJoint — 关节空间点到点

`/robot_arm/move_to_joint`；直接给关节角、**不过 IK**，用于示教/标定/绕开奇异点。`target_joints` 必须 **3 个**（Joint1/2/3，单位 rad）；`relative: true` 时是增量；`duration_sec > 0` 覆盖档位，`<= 0` 按档位（0.3 / 0.6 / 1.2 rad·s⁻¹）算时长。

**云台 J4-6 保持不动**（Commander 用当前回读填充下发），到位判据也只看 J1-3 —— 云台未上电不会卡住动作。这点与 `ArmMoveToPose` 的 STOWED 不同，后者会把 6 轴全部归零。

```bash
# 绝对角
ros2 action send_goal -f /robot_arm/move_to_joint robot_arm_interfaces/action/ArmMoveToJoint \
  "{target_joints: [0.5, 1.0, -1.2], transition_speed: 1, relative: false, duration_sec: 0.0}"

# 相对增量（J2 抬 0.3 rad，FAST）
ros2 action send_goal /robot_arm/move_to_joint robot_arm_interfaces/action/ArmMoveToJoint \
  "{target_joints: [0.0, 0.3, 0.0], transition_speed: 2, relative: true}"
```

三道校验任一不过都**不下发任何指令**、状态回 IDLE：个数不对 → `invalid_goal`；超关节限位 → `out_of_range`（限位从 `/robot_description` 解析 URDF，随实机标定自动跟随，不写死）；自碰撞 → `collision`（`/check_state_validity`；move_group 未运行时 fail-open 并告警）。

调试 GUI（`controller:=commander` 默认带）里有对应面板：3 个滑块 + 当前值只读框 + 「↧ 读当前值」（把实测灌进滑块，示教先摆后调最顺手）。

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
