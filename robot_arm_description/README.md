# robot_arm_description

robot_arm 6 轴机械臂描述包。提供 URDF/xacro、网格、运动学配置和控制器配置。

---

## 目录结构

```bash
robot_arm_description/
├── urdf/
│   ├── arm.urdf.xacro          # 机械臂宏定义（对外接口）
│   ├── arm_sim.urdf.xacro      # 仿真入口（内部 launch 使用）
│   └── eMeetArm_models.urdf    # SolidWorks 原始导出（参考/备用）
├── meshes/
│   └── base_link.STL / Link1~6.STL
├── mujoco/
│   └── eMeetArm.xml            # MuJoCo MJCF 模型（meshdir 复用本包 meshes/）
├── scripts/
│   └── simplify_meshes.py      # mesh 减面工具（开发用，不随包安装）
└── rviz/
    └── eMeetArm_models.rviz    # display 可视化配置
```

> 本包是**纯数据包**（URDF/mesh/MJCF/RViz 显示配置）。SRDF、kinematics（pick_ik）、
> joint_limits 等 MoveIt 配置在 `robot_arm_moveit_config`；ros2_control 控制器配置
> `controllers{,_real}.yaml` 在 `robot_arm_bringup/config/`。

---

## TF 树

```bash
world
  └─ arm_base_joint (fixed)
       └─ arm_base_link
            ├─ Joint1 → Link1
            │    └─ Joint2 → Link2
            │         └─ Joint3 → Link3
            │              └─ Joint4 → Link4
            │                   └─ Joint5 → Link5
            │                        └─ Joint6 → Link6
            │                             └─ tool0_joint → tool0
            │                                  └─ camera_optical_joint → camera_optical_frame
```

关节分组：

| 关节 | 类型 | 驱动 | 仿真 |
| --- | --- | --- | --- |
| Joint1–3 | revolute | CANopen（`canopen_ros2_control/RobotSystem`，ros2_canopen） | GazeboSystem |
| Joint4–6 | revolute | `robot_gimbal_driver_v2/GimbalForwardingInterface`（转发插件，不碰串口；实际执行者是**云台板端**的 robot_gimbal_node_v2，见 docs/云台控制路径融合方案.md） | GazeboSystem（`gazebo_camera=true`） |

---

## 核心文件说明

### `urdf/arm.urdf.xacro` — 对外接口

定义 `emeet_arm` xacro 宏，**本包对外唯一暴露的 URDF 接口**。

```xml
<xacro:include filename="$(find robot_arm_description)/urdf/arm.urdf.xacro"/>

<xacro:emeet_arm
    parent="base_link"
    xyz="0 0 0.31"
    rpy="0 0 0"
    sim_mode="false"
    gazebo_camera="false"
    controllers_yaml=""/>
```

宏参数说明：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `parent` | —（必填） | 挂载父 link 名称 |
| `xyz` | `0 0 0.31` | 安装位置偏移（m） |
| `rpy` | `0 0 0` | 安装姿态偏移（rad） |
| `sim_mode` | `true` | `false`=转发模式（须有云台板端 robot_gimbal_node_v2 且跨机 DDS 通），`true`=纯指令回显（无实物场景） |
| `gazebo_camera` | `false` | `true`=Gazebo 托管 Joint4-6，`false`=GimbalForwardingInterface（转发插件） |
| `controllers_yaml` | `''` | 非空时注入 `gazebo_ros2_control` 插件 |

外部包（底盘包）的 `full_robot.urdf.xacro` 示例：

```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 底盘 -->
  <xacro:include filename="$(find chassis_description)/urdf/chassis.urdf.xacro"/>
  <xacro:chassis/>

  <!-- 机械臂挂载到底盘 base_link -->
  <xacro:include filename="$(find robot_arm_description)/urdf/arm.urdf.xacro"/>
  <xacro:emeet_arm
      parent="base_link"
      xyz="0 0 0.31"
      rpy="0 0 0"
      sim_mode="false"
      gazebo_camera="false"
      controllers_yaml=""/>

</robot>
```

### `urdf/arm_sim.urdf.xacro` — 仿真入口（内部使用）

以 `world` 为根节点的独立完整 robot 文档，供 Gazebo / MuJoCo / display / MoveIt launch 文件加载。外部包**不需要**使用此文件，使用 `arm.urdf.xacro` 宏即可。

```python
# launch 文件中加载方式
import xacro

robot_description = xacro.process_file(
    os.path.join(desc_share, 'urdf', 'arm_sim.urdf.xacro'),
    mappings={
        'sim_mode':         'true',    # 仿真
        'gazebo_camera':    'true',    # Gazebo 模式
        'controllers_yaml': '/path/to/controllers.yaml',
    }
).toxml()
```

| 参数 | Gazebo | MuJoCo / display / MoveIt | 实物 |
| ------ | -------- | -------------------------- | ------ |
| `sim_mode` | `true` | `true` | `false` |
| `gazebo_camera` | `true` | `false` | `false` |
| `controllers_yaml` | 实际路径 | `''` | `''` |

---

## 控制器配置（`robot_arm_bringup/config/controllers.yaml`）

| 控制器 | 关节 | 使用场景 |
| -------- | ------ | ---------- |
| `joint_state_broadcaster` | 全部 | 所有模式 |
| `arm_controller` | Joint1–6 | Gazebo / MuJoCo 仿真 |
| `gimbal_controller` | Joint4–6 | 仅云台单独调试保留定义（与 arm_controller 抢 J4-6 接口，二者不可同时 active） |

实物用 `robot_arm_bringup/config/controllers_real.yaml`（单 CM 管全 6 轴）：`arm_controller` claim
Joint1-6，J4-6 已禁用轨迹/到点容差——反馈是云台真实回读（经转发插件回传，
有话题滞后 + 设备自规划滞后），不禁用会 abort 整条 6 轴轨迹。旧 arm_node
（自研 CANopen 栈）已于 2026-07 下线。

---

## 运动学（`robot_arm_moveit_config/config/kinematics.yaml`）

使用 **pick_ik** 全局 IK 求解器：

- `position_threshold`: 1 mm
- `orientation_threshold`: 0.01 rad
- `kinematics_solver_timeout`: 50 ms

---

## 与其他包的关系

```bash
robot_arm_description          提供 URDF/xacro、mesh、MJCF（纯数据）
  ↑ include arm.urdf.xacro
外部底盘包                     组合完整机器人 URDF
  ↑ 加载 arm_sim.urdf.xacro
robot_arm_bringup              launch 入口 + ros2_control 控制器配置
robot_arm_moveit_config        SRDF / kinematics / joint_limits / MoveIt 配置
```

本包对底盘/外部系统**零感知**，所有外部集成通过 `arm.urdf.xacro` 宏参数完成。
