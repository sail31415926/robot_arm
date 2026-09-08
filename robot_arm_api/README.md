# robot_arm_api — 机械臂对外接口的 Python 客户端库 + 调用 Demo

给**上层**（Director、大模型运镜规划、任何想直接调机械臂的人）用：把 Arm Commander 对外暴露的
`/robot_arm/*` action / service / topic 包成**阻塞式方法**，附云台 V2 直连客户端（云台 = 臂 J4-6）、
「JSON 运镜步骤表」执行器和命令行 demo。每次下发都会把等价的
`ros2 action send_goal / service call / topic pub` 打印到日志，可直接复制到终端复现。

```text
robot_arm_api/
├── robot_arm_api/
│   ├── arm_commander_client.py   ★ 库：ArmCommanderClient / GimbalV2Client / ArmApi / execute_plan
│   ├── arm_commander_demo.py     命令行 demo（子命令 = 库方法；plan = JSON 步骤表；demo = 小幅全流程）
│   └── __init__.py               re-export
├── examples/shot_plan_example.json   JSON 运镜步骤表示例
├── CMakeLists.txt / package.xml
└── README.md
```

两个文件头都有函数汇总，每个函数都是 doxygen 注释（`@brief / @param / @return`）。
**只依赖接口包**（`robot_arm_interfaces` 必需，`robot_gimbal_interfaces_v2` 可选），属于板上部署最小集
（见 [robot_arm/README.md](../README.md) 编译清单）；与 `robot_arm_bringup` 一样用
`ament_cmake + ament_python_install_package`，所以同时是模块（`from robot_arm_api import ArmApi`）和可执行
（`ros2 run robot_arm_api arm_commander_demo.py`）。

## 编译

```bash
cd ~/E7009_sail_ws            # 板上是自己的 overlay（~/Wqh_ws）
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select robot_arm_api
source install/setup.bash
ros2 run robot_arm_api arm_commander_demo.py --help
```

改 `.py` 不用重编（symlink-install）；新增文件 / 改 CMakeLists 才要重编。

## 运行前置

| 单元 | 在哪跑 | 启动命令 | 提供的接口 |
| --- | --- | --- | --- |
| 机械臂 Commander | 臂侧 Jetson | `ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false` | `/robot_arm/*`（4 action + 5 service + 状态 / 速度流 topic） |
| 云台 V2（可选，直连接口才需要） | 云台板（LubanCat） | `ros2 launch robot_gimbal_bringup_v2 gimbal_only.launch.py` | `/robot_gimbal_v2/*` |

两台机器同网段、同 `ROS_DOMAIN_ID`，且**都不能**设 `ROS_LOCALHOST_ONLY=1`（否则 `status` 显示「未收到
/robot_gimbal_v2/status」）。臂上电、CAN 复位见 `~/eMeetWork_sail/实物启动及指令.txt` 与 `robot_arm/CLAUDE.md`。

开发机无硬件时用 mock 臂跑通机械臂接口（云台 server 不在，对应调用 5s 内报「不可用」）：

```bash
ros2 launch robot_arm_bringup real.launch.py controller:=commander gui:=false arm_sim_mode:=true
```

## 命令行用法

任何子命令加 `--dry-run`：只打印等效 ros2 指令、不下发、不等 server。`--no-cli` 关掉等效指令打印。

```bash
R="ros2 run robot_arm_api arm_commander_demo.py"
$R status                                    # 臂 / 云台状态一屏
$R enable  |  $R disable  |  $R homing  |  $R reset-error  |  $R stop
$R observe --speed normal                    # 观察位；--return-to-start 到位后原路返回
$R stow                                      # 收纳位（6 轴关节回零）
$R pose 0.25 0.0 0.65 0 0 0 --speed slow     # 绝对末端位姿 x y z roll pitch yaw（米 / 度）
$R move-rel 0.0 0.0 0.05                     # 相对当前末端位姿 dx dy dz [--droll --dpitch --dyaw]
$R joint 0.2 0.5 -0.8 [--relative] [--duration-sec 3]   # 关节空间点到点 J1 J2 J3（rad，不过 IK）
$R dolly 0.10  |  $R truck -0.08  |  $R crane 0.05      # 推拉 / 横移 / 升降运镜（米），可 --return-to-start
$R orbit 0.6 0.0 0.7 -30 30 --radius 0.4 --elevation 0  # 球面环绕：球心 xyz、方位起止（度）
$R jog 0.05 0 0 --wyaw 10 --duration 1.0     # 末端速度点动（m/s、deg/s），自动切速度模式再切回
$R jog-joint 0.2 0 0 --duration 1.0          # 关节速度点动（rad/s）
$R mode velocity | trajectory                # 手动切控制模式
$R track-start --depth 1.0  |  $R track-stop # 视觉跟随
$R gimbal-rotate 20 nan -10                  # 云台 pan roll tilt（度，nan = 该轴不动），等到位
$R gimbal-jog 0.3 0 0 --duration 1.0         # 云台速度点动（rad/s）
$R gimbal-freeze | gimbal-go-zero | gimbal-start | gimbal-stop | gimbal-gyro-calib
$R gimbal-forward-enable off|on              # 关/开机械臂转发流对云台的控制权
$R plan examples/shot_plan_example.json [--continue-on-error]   # 执行 JSON 步骤表
$R demo                                      # 小幅度全流程演示（观察位 → 推/横移/升 5cm 并返回 → 收纳）
```

## Python 用法（上层模块直接 import）

```python
from robot_arm_api import ArmApi, make_pose

with ArmApi() as api:                    # 一个节点 + 后台执行器；退出 with 自动收尾
    arm = api.arm
    if not arm.wait_ready():             # 等 Commander 上线 + 首帧 ArmStatus
        raise SystemExit('机械臂未就绪')
    arm.enable()
    arm.move_to_observe()
    arm.dolly(0.10, speed='slow')                                  # 推镜 10cm
    arm.truck(-0.08, return_to_start=True)                         # 右移 8cm 再回来
    arm.move_to_pose(make_pose(0.25, 0.0, 0.65, 0, 0, 0))          # 绝对位姿（米 / 度）
    arm.arc_around(center=(0.6, 0.0, 0.7), radius_m=0.4,           # 环绕主体 -30° → +30°
                   az_start_deg=-30, az_end_deg=30,
                   on_camera_ready=lambda: print('开始录像'))       # 到达起拍点时回调
    arm.jog_cartesian(vx=0.05, duration_sec=1.0)                   # 末端速度点动（自动切模式）
    if api.gimbal:                                                 # 云台直连（可选）
        api.gimbal.rotate_to_deg(pan_deg=20, tilt_deg=-10)
    arm.move_to_stowed()
```

所有阻塞调用返回 `CallResult`：`bool(r)` 是否成功，`r.reason` 是 action 的 `exit_reason`
（`reached / timeout / cancelled / stopped / unreachable / out_of_range / collision …`）或 service 的 `message`，
`r.result` 是原始 Result / Response。**goal 被拒绝**（reason 含「goal 被拒绝」）说明 Commander 在 STOPPED /
ERROR 或正在执行，先 `arm.stop()` / `arm.reset_error()`。

## JSON 运镜步骤表（大模型输出 → 接口）

顶层是数组，每项 `{"op": ..., 参数...}`，参数名与方法形参一致；`speed` 取 `slow/normal/fast`，
`return_to_start` 布尔。示例见 [examples/shot_plan_example.json](examples/shot_plan_example.json)。
Python 里 `execute_plan(api, steps)`，命令行 `plan 文件`。

| op | 参数 | 调用 |
| --- | --- | --- |
| `enable` / `disable` / `homing` / `reset_error` / `stop` | `on` | `ArmCommanderClient.enable(...)` 等 |
| `mode` | `mode: trajectory \| velocity` | `switch_control_mode` |
| `stow` / `observe` | `speed, return_to_start` | `move_to_stowed / move_to_observe` |
| `pose` | `x y z roll pitch yaw, speed, return_to_start` | `move_to_pose` |
| `move_rel` | `dx dy dz droll dpitch dyaw, speed` | `move_relative` |
| `joint` | `j1 j2 j3, speed, relative, duration_sec` | `move_to_joint` |
| `dolly` / `truck` / `crane` | `distance_m, speed, return_to_start` | 同名方法 |
| `linear` | `start{...} end{...}` 或 `dx dy dz …`（从当前位姿出发） | `shot_linear / shot_linear_from_current` |
| `arc` | `center[3] radius_m az_start_deg az_end_deg elevation_deg` | `arc_around` |
| `orbit` | `center[3] az_start_deg az_end_deg el_start_deg el_end_deg r_start_m r_end_m` | `shot_orbit` |
| `jog` / `jog_joint` | `vx vy vz wroll wpitch wyaw duration_sec` / `v1 v2 v3 duration_sec` | `jog_cartesian / jog_joint` |
| `track_start` / `track_stop` | `desired_depth_m, hold_on_converge, total_timeout_sec …` | 视觉跟随 |
| `gimbal_rotate` / `gimbal_rotate_rad` | `pan roll tilt（度 / rad，缺省 = 不动）, timeout_sec` | `GimbalV2Client.rotate_to*` |
| `gimbal_jog` | `pan_vel roll_vel tilt_vel duration_sec` | `jog` |
| `gimbal_freeze` / `gimbal_go_zero` / `gimbal_start` / `gimbal_stop` / `gimbal_gyro_calib` | — | 同名方法 |
| `gimbal_forward_enable` | `enable` | `set_forward_cmd_enable` |
| `wait` / `wait_camera_ready` | `seconds` / `timeout_sec` | — |

## 方法 ↔ ROS 接口对照

| 方法 | ROS 接口 | 类型 |
| --- | --- | --- |
| `ArmCommanderClient.get_status/get_pose/is_moving/...` | `/robot_arm/arm_status` | `ArmStatus` topic（10Hz） |
| `get_joints` | `/joint_states` | `JointState` |
| `get_control_mode` | `/robot_arm/control_mode` | `ControlMode` topic（latched） |
| `move_to_stowed/observe/pose`, `move_relative` | `/robot_arm/move_to_pose` | `ArmMoveToPose` action |
| `move_to_joint` | `/robot_arm/move_to_joint` | `ArmMoveToJoint` action |
| `shot_linear*`, `dolly/truck/crane`, `shot_orbit`, `arc_around` | `/robot_arm/trajectory_shot` | `ArmTrajectoryShot` action |
| `track_target_start/stop` | `/robot_arm/track_target` | `ArmTrackTarget` action |
| `publish_cartesian_velocity`, `jog_cartesian` | `/robot_arm/follow_command` | `ArmFollowCommand` topic |
| `publish_joint_velocity`, `jog_joint` | `/robot_arm/cmd/joint_velocity` | `ArmJointVelocityCommand` topic |
| `enable/disable` `homing` `reset_error` `stop` | `/robot_arm/enable` `/homing` `/reset_error` `/stop` | service |
| `switch_control_mode`, `enter/exit_velocity_mode` | `/robot_arm/switch_control_mode` | `SwitchControlMode` service |
| `GimbalV2Client.rotate_to*` | `/robot_gimbal_v2/rotate_to_angle` | `RotateToAngle` action |
| `set_position_stream`, `freeze/go_zero/start_motor/stop_motor/gyro_calib` | `/robot_gimbal_v2/gimbal_cmd` | `GimbalCommand` topic |
| `publish_velocity`, `jog` | `/robot_gimbal_v2/cmd_vel` | `Twist`（angular.z=pan, .x=roll, .y=tilt） |
| `get_status`, `get_angles` | `/robot_gimbal_v2/status`, `/robot_gimbal_v2/joint_states_raw` | topic |
| `set_forward_cmd_enable` | `/robot_gimbal_v2/set_forward_cmd_enable` | service |

## 约定与坑

- **坐标 / 单位**：末端位姿在机械臂 `base_link` 系，位置米、姿态度（`ArmPose`：pitch 正抬头、yaw 按 msg 注释正值向右）；
  `dolly` 正 = +x 推进、`truck` 正 = +y（臂左侧）、`crane` 正 = +z 上升。速度流线速度 m/s、角速度**度/秒**。
  云台 rad / rad/s（URDF 关节系 pan=Joint4 / roll=Joint5 / tilt=Joint6，`rotate_to_deg` 做了度转换）。
- **位置类动作 6 轴一起动，到位判据只看臂 J1-3**（云台回读不收敛不会卡住）；`result.actual_pose` 是含云台的真实末端。
- **运镜录像时机**：`ArmStatus.camera_ready` 从到达起拍点起、到运镜结束为 true。`shot_*` / `dolly/truck/crane/arc_around`
  都接受 `on_camera_ready=回调`，在上升沿调一次；也可自己 `wait_camera_ready()`。
- **速度流有模式闸**：不切 `JOINT_VELOCITY` 指令会被 Commander 忽略并告警。`jog_*` 默认 `auto_mode=True`
  自动切进切出；自己用 `publish_*` 就要先 `enter_velocity_mode()`，并 ≥3Hz 持续发（断流 0.3s 即停）。
  默认后端切模式不切控制器、机械臂原地不动。
- **goal 被拒绝 = 执行端非空闲**：Commander 同一时刻只接一条 goal；急停后是 STOPPED、失败后是 ERROR，
  都要 `reset_error()` 才收新 goal。`stop()` 在非运动状态下是空操作。
- **云台直连与机械臂共用 J4-6**：臂侧 `arm_controller` 经 `/robot_gimbal_v2/forward_cmd` 驱动云台（指令变化 >0.01rad 才发），
  板端优先级 FREEZE > 显式位置（gimbal_cmd POSITION / action）> 转发流 > 速度流。直接转云台后，机械臂下一次
  位置类动作会把云台带回规划角；要云台完全听上层，`set_forward_cmd_enable(False)`，**用完打开**，否则臂侧 TF 与实物脱节。
  想要「臂 + 云台一起摆好」的正常做法是走 `move_to_pose` 的 roll/pitch/yaw，不用直连。
- **云台话题名带 `_v2`**：`algo_director` 里 `gimbal_controller.cpp` 还在发 `/robot_gimbal/gimbal_cmd` 和
  `/rotate_to_angle`（V1 时代的名字），板端 `robot_gimbal_node_v2` 实际监听的是 `/robot_gimbal_v2/...`，本包用后者。
- **超时即取消**：阻塞调用超过 `timeout_sec` 会自动 cancel goal 并返回 `client timeout`；Ctrl-C 会取消当前 goal 并调 `stop()`。
- **不要在 ROS 回调里调用这些阻塞方法**（会死锁）；本包用 wall-clock 做速度流节拍，不受 `use_sim_time` 影响。

## 验证记录

### 2026-09-08 Gazebo 机械臂仿真（`robot_arm_gazebo gazebo.launch.py controller:=commander gui:=false`）

完整物理 + MoveIt IK 链，与实机同一个 `arm_commander_node`。**全部接口通过**：

- CLI `plan`（9 步）：`observe → dolly 5cm → truck 5cm 往返 → crane 5cm → arc_around(-30°→30°, r0.4)
  → linear(显式起终点) → orbit(方位 20°→-20°、仰角 0→15°、半径 0.40→0.35、原路返回) → jog → stow`，全部 `reached`。
- Python API：`wait_ready / get_pose / get_joints / move_to_observe / dolly(on_camera_ready 回调在到起拍点后 0.06s 触发)
  / truck / crane(return_to_start) / move_to_pose / arc_around / move_relative(含姿态增量) / jog_cartesian(含 wyaw)
  / jog_joint / move_to_stowed`，全部 `reached`；速度点动后立刻发轨迹也正常。
- Commander 侧：13 段批量 IK 全部下发成功，0 次 IK 无解，日志零 ERROR / WARN。
- 唯一非 `reached`：环绕结束后（云台 J4 停在约 30°）立刻 `move_to_joint(0.1, 0, 0, relative=True)` 返回 `collision`。
  这是 Commander 的自碰撞闸按 6 轴组合位形判的（`robot_arm/CLAUDE.md` 速度控制坑第 4 条），不是接口问题；
  云台在大角度时先用 `move_to_pose` 把 6 轴摆正再走关节空间。
- `enable` 在仿真里返回「仿真模式，跳过」，属正常。

### 2026-09-08 mock 臂（`real.launch.py … arm_sim_mode:=true`）

- 机械臂全部 service / MoveToPose / MoveToJoint / 速度流 / Python API 通过（见上一节同项）。
- 早先几次 `dolly / orbit` 返回 `unreachable`、Commander 报 `IK 首帧无解 err=-15`（MoveIt `INVALID_GROUP_NAME`），
  **根因已查明**：那一版 `arm_commander_node` 二进制的批量 IK 用的规划组名是 `arm_stream`，SRDF 里没有这个组
  （move_group 日志 `Group 'arm_stream' not found in model`）；它来自同机另一会话对 `robot_arm_node` 的中间态编译，
  当前源码用 `arm`，重编后同一批指令在 mock 与 Gazebo 下都 `reached`。与本客户端无关。
- 速度点动停帧后紧接轨迹动作曾复现一次超时：Commander 按看门狗（0.3s）发完最后几帧位置流会覆盖刚收到的轨迹。
  已在 `jog_*` 停帧后静置 `VELOCITY_STREAM_SETTLE_SEC=0.5s` 修掉；手动 `publish_*` 收流也要留这个间隔。
- Commander 处于 ERROR / STOPPED 时新 goal 会被接受后立刻中止且不带 exit_reason，客户端报成
  `FAIL(ABORTED（执行端直接中止…先 reset_error）)`。
- 云台 server 不在线时对应步骤在 5s 内返回「action server 不可用」，`plan --continue-on-error` 可跨过继续；
  云台实物联调尚未做，接口名与字段按 `robot_gimbal_node_v2` 源码核对。

### 2026-09-08 实机跑通 shot_plan_example.json（Jetson，Wqh_ws overlay）

`plan examples/shot_plan_example.json --continue-on-error` 共 11 步，**臂侧 9 步全部 `reached`**：
`enable → observe → wait → dolly 0.10 → truck -0.08(return_to_start) → crane 0.05 → arc(-30°→30°, r0.4)
→ linear(绝对起终点) → stow`；结束时 `pose_state=STOWED`、`error_code=0`、J1-3 ≈ 0。

- 失败的只有第 9、10 步 `gimbal_rotate`（板端 `timeout`），根因见下条。
- **进程退出码 1 是设计如此**：`--continue-on-error` 只是不中断，但只要有步骤失败就以 1 退出，便于脚本判断。
- 实机残差（均在 Commander 容差 0.010m 内，属正常）：`dolly` 指令终点 x=0.400、下一步实测起点 x=0.395；
  `truck` 原路返回后 y 残留 -0.0087。
- 姿态会沿相对运镜链累积：`dolly/truck/crane` 以**当前末端位姿**为起点，而该位姿含云台回读，
  三步下来 roll 从 0.19° 漂到 1.89°、yaw 从 -0.09° 到 -1.00°。上层连续下发多条相对运镜时，
  建议每隔几步用绝对 `pose` / `linear` 归一次基准。

### 2026-09-08 实机首测（Jetson，Wqh_ws overlay）

- `--dry-run / status / enable` 正常；`status` 同时收到云台板 `/robot_gimbal_v2/status`。
- `gimbal-rotate -45 nan 0`（RotateToAngle）**板端返回 timeout、进度一直 0%**。已定位到云台板上 Wqh_ws 版
  `robot_gimbal_node_v2` 的语义不一致，不是本 API：该版 `params.yaml` 开了 `fpv_on_startup / forward_cmd_use_fpv /
  use_motor_angle`，即转发流与反馈都按**框架角（URDF 关节角）**处理；但 `RotateToAngle` 的执行路径仍是
  `enter_traj_locked → angle_spec`（**IMU 世界绝对姿态角** + 底座补偿），目标与反馈不在一个坐标系，臂末端 yaw 约 30°
  时 pan 反馈永远追不上目标（实测 pan 从 -45.7° 跑到 +24.7°），tilt 因底座接近水平恰好到位。
  **决定性证据**：同一次 plan 里第 10 步 `rotate_to(pan=0, tilt=0)` 超时失败，紧接着第 11 步 `stow` 经
  JTC → `forward_cmd` 把 J4-6 拉到零并成功（跑完实测云台 pan=0.008 / roll=-0.003 / tilt=0.002 rad）——
  同一个「回零」目标，走转发流通、走 action 不通，问题精确定位在板端 action 执行路径。
  **临时办法**：云台朝向走 `move_to_pose / move_relative(dyaw=…)`（经 Commander IK → JTC → forward_cmd，该版按框架角执行）；
  **根治**：在该节点里让 RotateToAngle 在 FPV/框架角配置下走 `fpv_angle_spec`（与 forward_cmd 同一语义）。

### 上实机建议顺序

1. `--dry-run` 看等效指令；2. `status`（error_code 应为 0）；3. `enable`；4. `observe --speed slow`；
5. `dolly 0.05 --speed slow`；6. 云台直连先 `gimbal-rotate 0 nan 0`（小角度、单轴）；7. `stow`。
任一步出问题，用日志里打印的等效 ros2 命令原样复现，即可区分是 API 还是 Commander。
