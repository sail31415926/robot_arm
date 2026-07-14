# robot_arm_moveit_config

eMeetArm 的 MoveIt 配置包（社区标准 `<robot>_moveit_config` 布局，手写、非 Setup Assistant 生成）。
URDF 本体在 `robot_arm_description`（`.setup_assistant` 已声明来源，兼容 `MoveItConfigsBuilder`）。

## 内容

```text
robot_arm_moveit_config/
├── .setup_assistant                    # URDF/SRDF 来源声明（MoveItConfigsBuilder 兼容）
├── config/
│   ├── eMeetArm_models.srdf            # 规划组定义（arm: arm_base_link→tool0）
│   ├── kinematics.yaml                 # pick_ik 求解器
│   ├── joint_limits.yaml               # MoveIt 关节速度/加速度限制（robot_description_planning）
│   ├── planning_pipeline.yaml          # OMPL 规划管线（RRTConnect/RRT）
│   ├── moveit_controllers.yaml         # gazebo 后端：Ros2ControlManager
│   ├── moveit_controllers_real.yaml    # 实物后端：SimpleControllerManager + FJT action
│   ├── moveit_controllers_mujoco.yaml  # MuJoCo 后端：SimpleControllerManager（mujoco_node 提供 action）
│   ├── servo_config.yaml               # MoveIt Servo（launch 里须包 moveit_servo 命名空间！）
│   └── moveit.rviz                     # MotionPlanning RViz 配置
└── launch/
    └── moveit.launch.py                # move_group + RViz（gazebo include；独立调试入口）
```

## 用法

```bash
ros2 launch robot_arm_moveit_config moveit.launch.py                          # move_group + RViz（仿真时钟）
ros2 launch robot_arm_moveit_config moveit.launch.py use_sim_time:=false rviz:=false  # 实物/无显示器
```

三后端接入方式（controller 配置为分叉点，其余配置共用）：

- **gazebo**：`gazebo.launch.py` include 本包 `moveit.launch.py`（`moveit_controllers.yaml`）
- **real**：`real.launch.py` 经 `robot_arm_bringup.launch_common.move_group_node` 工厂自起
  move_group（`moveit_controllers_real.yaml`，不 include 本包 launch——Ros2ControlManager
  会找不到 Joint1-3 控制器）
- **mujoco**：`mujoco.launch.py` 同 real 方式（`moveit_controllers_mujoco.yaml`）

## 注意

- `servo_config.yaml` 顶层是裸参数：launch 加载时**必须**包成 `{'moveit_servo': ...}`，
  Humble 的 servo 只从 `moveit_servo.` 前缀读参数，裸传会整体静默回退 Panda 默认值。
  三后端现均经 `launch_common.servo_node` 工厂处理，勿绕过。
- 关节限位真源是 URDF（`robot_arm_description`），`joint_limits.yaml` 是 MoveIt
  规划侧的收紧版；驱动器侧软限位在 `robot_arm_driver` 的 bus.yml（607D）。
