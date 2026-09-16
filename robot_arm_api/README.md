# robot_arm_api — 机械臂对外接口的 Python 客户端库 + 调用 Demo

给**上层**（Director、大模型运镜规划、任何想直接调机械臂的人）用：把 Arm Commander 对外暴露的
`/robot_arm/*` action / service / topic 包成**阻塞式方法**，附云台 V2 直连客户端（云台 = 臂 J4-6）、
「JSON 运镜步骤表」执行器和命令行 demo。每次下发都会把等价的
`ros2 action send_goal / service call / topic pub` 打印到日志，可直接复制到终端复现。

```text
robot_arm_api/
├── robot_arm_api/
│   ├── arm_commander_client.py   ★ 库：ArmCommanderClient / GimbalV2Client / ArmApi / execute_plan
│   ├── reach_check.py            ★ 可达性 / 余量盒子 / 步骤表预检 / 能力卡（纯 numpy，给大模型运镜决策用）
│   ├── reach_fit.py              ★ 可达区多项式拟合：给大模型一条能自己代入验算的公式
│   ├── llm_shot_loop.py          ★ 大模型单步运镜闭环：读关节角 → 余量 → 大模型出一步 → 校验/夹取 → 执行
│   ├── shot_spec.py              ★ 《拍摄接口规范 v9》分镜 JSON：解析、第 8 章校验、A 型参考轨迹、B 型按目标快照展开
│   ├── shot_compiler.py          ★ 规范分镜 → Commander 原语（LINEAR / ORBIT / PTP）：坐标系、光心↔法兰、容差内细分、档位量化、可达性
│   ├── shot_executor.py          ★ 逐条下发 + 第 9 章反馈（phase / progress / error / in_tolerance / degradation / target_status）
│   ├── arm_commander_demo.py     命令行 demo（子命令 = 库方法；plan = JSON 步骤表；shot / shot-compile = 规范分镜；check/headroom/card/llm-step）
│   └── __init__.py               re-export（无 ROS 环境时只导出 reach_check 那一半）
├── test/                         pytest：test_reach_check（正解对拍 pinocchio、逆解往返、可达判定、余量、预检、能力卡）
│                                        test_reach_fit（拟合保守性、覆盖率、公式文本）
│                                        test_llm_shot_loop（提示词 / schema、夹取投影与降级、用户反馈、OpenAI 兼容请求）
│                                        test_shot_spec / test_shot_compiler / test_shot_executor（规范校验规则、几何、编译、假臂执行与反馈）
├── examples/shot_plan_example.json   JSON 运镜步骤表示例
├── examples/spec_v9/                 规范 5.5 / 6.6 的示例分镜（a_* A 型、b_* B 型）+ 一条臂系可达的 100° 圆弧
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

**板上两个工作空间都装了本包**（2026-09-09）：

| 位置 | 路径 | 怎么用 |
| --- | --- | --- |
| 公用 | `~/E7009_ws/src/E7009/robot_arm/robot_arm_api` | 只 source `/opt/ros/humble` + `~/E7009_ws/install`，**别 source Wqh_ws** |
| 个人 overlay | `~/Wqh_ws/src/robot_arm/robot_arm_api` | 再 source `~/Wqh_ws/install`（overlay 会盖住公用那份） |

同时 source 两个时 overlay 优先，`ros2 pkg prefix robot_arm_api` 打出哪条路径就是在跑哪一份 —— 排查前先确认这个。

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
$R joint-one 2 0.1 --relative                # ★单关节：只动 J2 转 +0.1 rad（编号 1-6）
$R joint-one 4 0.26                          # ★单关节：J4=云台 pan，自动路由到云台（rad）
$R jog-joint-one 1 0.2 --duration 1.0        # ★单关节点动：编号 1-6，rad/s
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
    api.move_single_joint(2, 0.1, relative=True)                   # 单关节：只动 J2（编号 1-6）
    api.move_single_joint(4, 0.26)                                 # J4=云台 pan，自动路由
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
| `joint_one` | `index(1-6) value, speed, relative, duration_sec` | `ArmApi.move_single_joint`（1-3 臂 / 4-6 云台） |
| `jog_joint_one` | `index(1-6) velocity, duration_sec, auto_mode` | `ArmApi.jog_single_joint` |
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

## 《拍摄接口规范 v9》分镜 JSON → Commander（`shot_spec` / `shot_compiler` / `shot_executor`）

`~/eMeetWork_sail/摄影机器人规范/摄影机器人-拍摄接口规范-v9.md` 定义的层级 2 运镜 JSON（`type` + `waypoints` +
`segments` + `tolerance` [+ `av`]），和上面的 op 步骤表是两套东西：规范里只有几何量（光心世界坐标、`look_at`、弧度、
`duration`/`speed`），没有推拉摇移这类动作词。本包用三层把它接到现有 Commander 接口上，**不改 Commander**：

```text
规范 JSON ─▶ shot_spec.parse_shot ─▶ (B 型) tracking_to_static ─▶ shot_compiler.compile ─▶ shot_executor.run
              第 8 章校验                 按目标快照展开成 A 型         → LINEAR / ORBIT / PTP      逐条 goal + 第 9 章反馈
```

```python
from robot_arm_api import ArmApi, ArmModel, ShotCompiler, ShotExecutor, WorldFrame, make_shot_feedback_publisher

model = ArmModel.from_share_files()
world = WorldFrame.from_chassis(x=0.0, y=0.0, yaw=0.0, arm_base_height=0.31)   # odom → arm_base_link；或 lookup_world_frame(node)
with ArmApi() as api:
    api.arm.wait_ready(); api.arm.enable()
    executor = ShotExecutor(api, ShotCompiler(model, world=world),
                            feedback_publisher=make_shot_feedback_publisher(api.node),   # /robot_arm/shot_feedback
                            on_camera_ready=lambda av: start_recording(av))              # av 组由相机模块消费
    report = executor.run(shot_json)          # 阻塞到 done / 失败 / 取消
    print(report.user_line(), report.exit_reason, report.warnings)
```

命令行：`shot-compile 文件`（离线校验 + 编译，打印原语与告警）、`shot 文件 [--chassis X Y YAW | --tf] [--dry-run]`
（执行；B 型加 `--target-topic` 或 `--target-json`）。文件可以是单条分镜、分镜数组或大模型输出文件（`--shot-index` 选镜）。

**编译规则**（对照规范章节）：

| 规范 | 这里怎么做 |
| --- | --- |
| 4.1 世界系 odom、受控体相机光心 | `WorldFrame`（odom → arm_base_link）换到臂基座系；`CameraFrames` 用 URDF 里 gimbal_tool0 → camera_optical_frame 的固定变换（光轴 +Z = 法兰 +X，偏 4.4 cm）把光心位姿换成 Commander 的法兰 ArmPose |
| 5.2 `look_at` + `roll` | `camera_rotation()`：右向量水平（roll=0 = 画面水平），roll 正 = 从相机背后看顺时针，与 Commander 的 `aim_quat`（roll 0）同一个旋转；正俯视时取世界 +X 为画面上方 |
| 5.3.2–5.3.4 line / arc / spline | 每段先试"一条原语能否在 `tolerance × 0.5` 内复现参考轨迹"：直线 / 纯摇镜 → LINEAR；`look_at` 恒定的圆弧 → ORBIT（球心 = 法兰光轴射线的交点）；都不行按弧长细分成 N ≤ 8 条 LINEAR 弦。偏差按**法兰**插值再换回光心算，所以 4.4 cm 偏置的影响也算进去了 |
| 5.3.5 `law` | Commander 每条原语都是 Ruckig 静止→静止 S 曲线（= `s_curve` / `ease_in_out`）；其余 law 告警；末段 `constant` / `ease_in` 按 4.1 标 `degradation: slowed` |
| 5.3.6 `duration` / `speed` | 折成 slow / normal / fast（`arm_params.yaml speed_profiles`，位置 0.02/0.05/0.10 m/s、姿态 0.05/0.10/0.20 rad/s）中最接近的档；比 fast 还快 → `slowed`；比 slow 还慢 → 告警（会比要求的快） |
| 5.3.7 段内插值 | `look_at` 点线性插值；纯摇镜按光轴**方向**在大圆上按角度插值（与规则 13 的"扫角 < 180° 才唯一"一致），roll 线性；大圆经过正上 / 正下方（画面上方向无定义）按规则 13 报错。Commander 对姿态做的是 slerp（两端俯仰相同时 = 绕竖轴的等俯角小圆），两者在带俯仰的大幅摇镜里会差几度，超出 aim 预算就细分成多条 LINEAR |
| 5.4 / 6.5 `tolerance` | 编译期只允许吃一半（`tolerance_budget`），执行期 `error` / `in_tolerance` 按全额判 |
| 6 B 型 | **没有闭环**：开始时读一帧 `target`（`make_target_provider` 订 std_msgs/String JSON 或 PointStamped），`tracking_to_static` 按 d/az/el/position + uv 展开成 A 型开环执行；段上附 `TrackInfo`，参考轨迹按 6.4.3：位置在目标球坐标 (d, az, el) 里插值（line 与 Commander ORBIT 同一几何，环绕 / 螺旋 / 整圈 Δaz=2π 都能编成一条 ORBIT；spline 按弧长归一化、只能细分），`uv` 段内线性、每一点光轴由目标 + uv(s) 定；段间复查 id（变 → `id_changed` 停）、`target_static` 按纵深 / 横向 / 竖向分轴复查位置（阈值 = 对应容差与 2 cm 感知噪声地板取大，动 → `moved` 停）、过期只告警；`time_scale` 恒 1.0 |
| 8 校验 | 规则 1–24、29–35、37–40 在 `validate_shot`（21 需 `lens` / `aspect`，40 需 `base_fps`；35 只对**显式**写出的 SUBJECT_ANCHOR / TARGET 报错，缺省值由规则引擎按型填）；25 / 26 在编译期用 `check_pose` 沿每条原语的法兰路径采样；**28 只告警**（规范自己的例 6.6.3 就踩线，与规范维护方对齐后再定）；36 需要目标尺寸，不在本层 |
| 9 反馈 | `ShotFeedback`：`phase`（segment / hold / done）、`hold_elapsed`、`segment_index`、`progress`（**由反馈里的 current_pose 在原语几何上投影得到**——Commander 的 `progress_percent` 在运镜段内是按 0.1×timeout 归一化的时间，只用来判断"还没开始动"：LINEAR ≤ 50、ORBIT ≤ 60；LINEAR 的转角大于弦长时按转角占比，免受纯摇镜短弦的位置噪声影响）、`error` / `in_tolerance`（A 型 position / aim / roll 三项；B 型按规范给 d / az / el / u / v / roll 六项或 position / u / v / roll 四项，相对锁定的目标算；hold 期相对臂所停的 waypoint）、`deviation_cause`（只会给 none / limit）、`degradation`；B 型多 `time_scale` / `target_status`。反馈回调与主线程共用一把锁，goal 结果返回后的迟到反馈按代号丢弃 |
| `hold` | 本地睡眠，后面还有 goal 时扣掉 Commander 自带的 1 s 起点停顿（`posture.dwell_at_start_sec`），停顿期间仍报 `phase: hold`；`plan_steps()` 里的 `wait` 同一口径 |
| 7 `av` | 只校验、不执行：到达起拍点即调一次 `on_camera_ready(av)`（首点带 hold 时在 PTP 到位后，否则在第一条运镜 goal 的 camera_ready 上升沿） |

**能力边界**（报告 / 告警里会明说）：单臂 6 轴、底盘不动，所以规范例子里 1–2 m 的机位会被规则 25 拒掉（那是走位 + 全身
协同的活，见 robot_wholebody）；每条原语之间机械臂会停住（Commander 每条 goal 先 PTP 到起点、停 1 s 再动），细分越多停顿越多；
没有避障；B 型 `uv` 不闭环。

## 可达性判定与余量盒子（`reach_check.py`，给大模型运镜决策用）

大模型是概率生成器不是约束求解器：让它直接吐末端坐标再"事后拦"永远拦不干净。`reach_check` 提供两层保障，
共用一个从 URDF 现算的运动学模型（换云台、改限位不用改代码）：

| 层 | 函数 | 作用 |
| --- | --- | --- |
| **余量盒子**（主） | `headroom(model, joints, subject=None)` | 从当前关节角出发，各方向**连续运动**还能走多远：`dolly/truck/crane` (m)、`dyaw/dpitch` (°)、给了主体再加环绕方位 `arc_az`。把区间塞进提示词，大模型只在区间里选数；`headroom_schema(h)` 把区间写成单步 JSON schema 的 `minimum/maximum`，支持 structured output 的模型 API 可当硬约束 |
| **事后校验**（保险丝） | `check_pose(model, pose)` / `check_plan(model, steps, joints, mode)` | 单点判定 / 整张 JSON 步骤表逐条推演。不可达给**人话原因**（"超出臂长：肩到末端 0.66 m，最大 0.60 m"、"J3 = −2.45 rad 距限位只剩 0.05 rad"）与最近可行位姿 `suggestion`；`mode='clip'` 把超余量的路径类步骤夹到边界继续推演，`PlanReport.summary()` 直接回喂大模型 |
| 背景 | `capability_card(model, joints=None)` / `capability_data(model)` | 能力卡文本 / 结构化数据：r→离地高静态表、J1 可用范围、舒适区、朝向规则；给了关节角再附"当前状态 + 余量区间" |

```python
from robot_arm_api import ArmModel, headroom, check_plan, capability_card

model = ArmModel.from_share_files()            # xacro 现场展开 arm.urdf.xacro（含云台 V2）；或
# model = ArmModel.from_urdf_string(robot_description)   # 运行时直接吃 /robot_description
q = [0.0, 1.0, -1.5, 0.0, 0.3, 0.0]            # 当前关节角（arm_nominal）
print(capability_card(model, q, subject=[0.85, 0.0, 0.68]))   # 喂给大模型的文本
h = headroom(model, q)                         # {'dolly': (-0.64, 0.11), 'crane': (-0.31, 0.07), ...}
report = check_plan(model, steps, q, mode='clip')            # steps = 大模型输出的 JSON 步骤表
print(report.summary())                        # 每步一行：✓ / ⚠ 夹取到 57%（…）/ ✗ 走到 30% 处不可达：…
```

命令行（不下发、不需要 Commander 在跑；关节角缺省从 `/joint_states` 读 2 s，读不到用 `--joints`）：

```bash
R="ros2 run robot_arm_api arm_commander_demo.py"
$R check shot_plan.json [--clip] [--json]           # 步骤表预检，exit 0 = 整表可执行
$R headroom [--subject 0.85 0 0.68] [--schema]      # 余量区间 / 单步 JSON schema
$R card [--joints 0 1 -1.5 0 0.3 0]                 # 能力卡文本
```

**约定与口径**

- 位姿沿用 `ArmPose` 语义：机械臂 `base_link` 系、末端 `gimbal_tool0`、米 / 度，
  `R = Rz(yaw)·Ry(pitch)·Rx(roll)`（与 Commander `motion::rpy_to_quat` 一致）。注意这意味着
  **`pitch` 正 = 俯视**（`ArmPose.msg` 注释里"正值抬头 / 正值向右转"与 Commander 的实际数学相反，
  本模块按代码实际行为走）；`pose_from_look_at(pos, look_at)` 与 `aim_quat` 同约定。
- 余量默认 `Margins(dist_m=0.05, joint_rad=(10°, 0.1, 0.1, 0.1, 0.1, 0.1))`：臂长两端各留 5 cm，J1 留 10°
  （实机 J1 软限位 / 回绕编码器的历史问题），其余关节 0.1 rad。
- **连续 vs 换分支**：`headroom` 与 `check_plan` 的路径类步骤按"从当前位形连续运动、不换几何分支"判定
  （运动语义）；`check_pose(continuous=False)`（默认）与 `pose / move_rel` 步骤允许任何限位内的解
  （"这个位姿臂能不能摆出来"）。两者结论可能不同，这是有意的。
- 这台臂的结构：J1 偏航 + J2/J3 平行轴平面二连杆（闭式位置解、肘唯一分支）+ 云台 3 轴（数值姿态解）。
  J1 ±150° 的正向 / 反折两支几乎覆盖全部方位，末端还能沿 y≈0.0265 的臂平面越过基座顶部后退——
  几何上可达，但**自碰撞不在本模块范围内**，执行前仍由 Commander 的 `/check_state_validity` 兜底。
- `from_share_files()` 需要 source ROS 与工作空间（xacro + 两个描述包），拿不到时抛 `RuntimeError`；
  其余全部纯 numpy。

### 可达区公式（`reach_fit.py`）——给大模型一条能自己代入验算的式子

静态表是离散采样，公式是连续函数：大模型要输出**绝对位置**（`op=pose`）时，可以自己把 (x, y, z) 代进去验算。

```bash
ros2 run robot_arm_api arm_commander_demo.py fit          # 公式文本；--json 给系数
```

```text
臂末端可达区用 r 的 3 次多项式描述（r = 到臂基座竖轴的水平距离，h = 离地高，单位 m）：
  r ∈ [0.24, 0.56]
  h_max(r) = - 6.505·r^3 + 5.347·r^2 - 2.040·r + 1.373
  h_min(r) = 12.628·r^3 - 12.005·r^2 + 3.650·r - 0.127
  点 (r, h) 可达 ⇔ r 在范围内 且 h_min(r) ≤ h ≤ h_max(r)。
  换算：r = sqrt(x² + y²)，h = z + 0.31（x, y, z 为机械臂 base_link 系）。
  公式已向内收缩（各 ≤ 2 cm）保证保守：公式内的点全部真可达，覆盖真实可达区 94%。
```

两个**必须知道**的边界条件（都在 `formula_text()` 里写给大模型了）：

- **公式只覆盖 r ∈ [0.24, 0.56]**。r 再小时，肩部周围「肩到末端距离 ≥ `dist_min`+余量」的球够不到，
  可达高度被切成**上下两段**（r=0.15：主段 0.72~1.15，另有一小段 0.27~0.42）——带洞的区域没法用
  `h_min ≤ h ≤ h_max` 表达，硬拟就会把洞判成可达。这段交给 `capability_card` 的**分段表**
  （`capability_data()['table'][i]['bands']`）和 `headroom`。
- **保守优先**：拟合后按最大残差 + 安全量整体内移，再逐点网格复核，发现假可达继续内移。
  `coverage` 告诉你为此损失了多少真实可达区（当前 94%）。宁可少给，不能给出做不到的点。

把公式塞进提示词让大模型自检：`PromptBuilder(model, region_fit=fit)`，或 CLI 的 `llm-step --formula`。
注意它只管**位置**；朝向能不能做出来仍由 `check_pose` / `check_plan` 兜底。

### 超范围动作的降级执行（`check_plan(mode='project')`）

不想让大模型为"推 1 米"这种越界动作反复重来时，用 `project` 模式：**能做多少做多少**，执行后用人话告诉用户。

| 动作类型 | reject | clip | **project** |
| --- | --- | --- | --- |
| 路径类 `dolly/truck/crane/linear/arc/orbit` | 拒绝 | 夹到可达边界 | 夹到可达边界 |
| 整点类 `pose / move_rel` | 拒绝 | 拒绝 | **投影到最近可行位姿** |
| `joint` | 拒绝 | 拒绝 | **夹进关节可用范围** |

`StepReport.adjust_kind` 标明改写方式（`'clip'` / `'project'` / `None`），`clipped` 是改写后的参数
（`move_rel` 投影后仍写回**增量**字段，执行器才认得）。一步都走不了（`fraction == 0`）仍然判失败——
"能做多少做多少"不等于"假装做了"。朝向做不出来的位姿投影也救不了，仍然拒绝。

### 面向最终用户的文字反馈

`StepRecord.user_line()` / `format_user_report(records)` 给的是**口语**，不带 op 名、关节名、弧度：

```text
向左平移 1.00 米 超出机械臂行程，最多只能向左平移 29 厘米，已按这个幅度执行
移动到位置（前 0.90 米、左 0.00 米、高 0.60 米） 超出机械臂可达范围，已改到最近能到的位置执行
共执行 2 个动作，其中 2 个因超出机械臂能力已降级执行。
```

三个出口并行（`fan_out` 合流，互不影响）：

| 出口 | 怎么拿 |
| --- | --- |
| **ROS 话题** `/robot_arm/llm_feedback`（`std_msgs/String`） | 默认就发；`run_llm_shot_loop(feedback_topic=...)` 换名、`None` 关闭 |
| 节点日志 | `[给用户]` 前缀，跟着 Commander 日志走 |
| 上层回调 | `run_llm_shot_loop(user_log=你的函数)` / 无 ROS 时 `run_step_loop(user_log=...)` |

```bash
ros2 topic echo /robot_arm/llm_feedback          # 另一个终端就能看到每步的口语反馈
$R llm-step "推成特写" --mode project --formula --llm openai   # 降级执行 + 公式自检，不回喂重来
$R llm-step "…" --feedback-topic none            # 只要日志、不发话题
```

话题只发**执行后**的口语说明（每步一条 + done 一条），不发中间的校验细节；整段总结由
`format_user_report(records)` 在闭环结束时给出。

### 大模型单步闭环（`llm_shot_loop.py`）

把上面两层串成"余量盒子"的调用层：**每步** 读关节角 → `headroom` → 组提示词（system = 能力卡静态部分 + 输出规则，
user = 任务 + 当前状态与余量 + 历史 + 上次被拒原因，schema = 余量区间 + done）→ 大模型只出**一步** JSON →
`check_plan(mode='clip')` 校验 / 夹取 → `run_plan_step` 执行 → 下一步；大模型输出非法或被拒就把原因回喂再要一次
（默认最多 2 次），输出 `{"op":"done"}` 或到 `--max-steps` 结束。核心 `run_step_loop` 不依赖 ROS（测试用假臂 +
脚本大模型跑通闭环），ROS 胶水 `run_llm_shot_loop` 用 `ArmApi` 读关节角、执行、可选附相机画面。

```python
from robot_arm_api import ArmApi, ArmModel, OpenAICompatClient, ManualClient, run_llm_shot_loop

model = ArmModel.from_share_files()
llm = OpenAICompatClient.from_env()      # LLM_BASE_URL / LLM_MODEL / LLM_API_KEY；或 ManualClient() 人工当模型
with ArmApi() as api:
    api.arm.wait_ready()
    records = run_llm_shot_loop(api, model, llm, '把桌上的花瓶推成特写，画面居中',
                                subject=[0.9, 0.0, 0.7], image_topic='/camera/image_raw', max_steps=8)
    for rec in records:
        print(rec.line())     # 第 2 步 truck {"distance_m": 0.06}：成功，被夹取到 {…}，模型理由：…
```

```bash
R="ros2 run robot_arm_api arm_commander_demo.py"
$R --dry-run llm-step "推近花瓶" --joints 0 1 -1.5 0 0.3 0        # 不下发、不需要臂：看提示词与校验流程
$R llm-step "推近花瓶" --llm manual --max-steps 5                  # 实机联调：人在终端里当大模型
LLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1 LLM_MODEL=qwen-vl-max LLM_API_KEY=sk-… \
$R llm-step "推近花瓶" --llm openai --image-topic /camera/image_raw --subject 0.9 0 0.7
```

- 大模型接入是 OpenAI 兼容 `/chat/completions`（DashScope 兼容模式、DeepSeek、vLLM、Ollama…都行）；
  支持 `response_format=json_schema` 的服务会把余量区间当**硬约束**，不支持的自动退到 `json_object`，
  区间约束就只靠提示词 + 执行侧校验。附图走 OpenAI 的 `image_url`（JPEG base64，需 cv2）。
- **千问（DashScope）配置**：`LLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1`、
  `LLM_MODEL=<控制台里的模型名>`、`LLM_API_KEY=sk-…`，并加 `LLM_EXTRA_JSON='{"enable_thinking": false}'`
  ——千问 3.x 思考模式下**非流式**调用会被拒，本客户端不走流式。`LLM_TIMEOUT_SEC` 可调超时（默认 60）。
- 允许的 op：`dolly / truck / crane / move_rel / arc / linear / pose / wait / done`；其它一律拒绝回喂。
- `--mode project` 时超范围动作直接降级执行、**不回喂大模型重来**（只有 JSON 非法 / op 不认识才重试）。
- **会动臂**。第一次先 `--dry-run`；实机用 `--llm manual` 手敲小步（≤ 5 cm）确认链路，再换真模型；
  Ctrl-C 会取消当前 goal 并急停。

**测试**：`colcon test --packages-select robot_arm_api && colcon test-result --verbose`
（或在包目录 `python -m pytest test/`，需 source ROS；正解对拍用 venv 里的 pinocchio，缺失自动 skip）。

## 方法 ↔ ROS 接口对照

| 方法 | ROS 接口 | 类型 |
| --- | --- | --- |
| `ArmCommanderClient.get_status/get_pose/is_moving/...` | `/robot_arm/arm_status` | `ArmStatus` topic（10Hz） |
| `get_joints` | `/joint_states` | `JointState` |
| `get_control_mode` | `/robot_arm/control_mode` | `ControlMode` topic（latched） |
| `move_to_stowed/observe/pose`, `move_relative` | `/robot_arm/move_to_pose` | `ArmMoveToPose` action |
| `move_to_joint` | `/robot_arm/move_to_joint` | `ArmMoveToJoint` action |
| `ArmApi.move_single_joint(1-3)` / `jog_single_joint(1-3)` | 同上 / `/robot_arm/cmd/joint_velocity` | 补齐另两轴后转发 |
| `ArmApi.move_single_joint(4-6)` / `jog_single_joint(4-6)` | `/robot_gimbal_v2/rotate_to_angle` / `cmd_vel` | 路由到云台，其余轴 NaN=不动 |
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
| `llm_shot_loop` 的用户反馈 | `/robot_arm/llm_feedback` | `std_msgs/String`（本包发布，供上层订阅） |
| `shot_executor` 的第 9 章反馈 | `/robot_arm/shot_feedback` | `std_msgs/String`（JSON，本包发布） |
| `shot_executor` 的 B 型目标输入 | 分镜 `target` 字段指定 | `std_msgs/String`（6.2.4 JSON）或 `geometry_msgs/PointStamped` |

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

### 2026-09-08 实机首测（Jetson，Wqh_ws overlay）

- `--dry-run / status / enable` 正常；`status` 同时收到云台板 `/robot_gimbal_v2/status`。
- `gimbal-rotate`（RotateToAngle）曾**稳定返回 timeout**，**2026-09-09 已在云台板修复并实机验证通过**。

  **根因**：云台板把反馈源切成了编码器框架角（`use_motor_angle: true`，9/7 修「起手云台乱转」时改的），
  转发流也切成框架角（`forward_cmd_use_fpv: true`）；但 `RotateToAngle` / `gimbal_cmd POSITION` 的**命令路径仍是
  `angle_spec`，即 IMU 世界绝对姿态角**。命令侧与到位判据侧隔着「IMU 零点 vs 编码器零点」的固定偏差，
  `|cur - tgt|` 永远进不了 `at_target_pos_tol`（0.0175 rad）→ 必然超时。
  实测（臂在收纳位、只动云台，排除底座旋转）：命令 pan=+0.1745 停在编码器 +0.059（差 0.116）；
  命令 pan=0 停在 -0.122（差 0.122）。两次偏差一致 ≈ 0.118 rad，正是那个零点差。
  `at_target_locked` 的注释里写着的前提「`cur_*` 来自 IMU 是惯性角」，正是被 `use_motor_angle: true` 打破的。

  **修复**：`robot_gimbal_node.cpp` 的 `push_manual_locked()` `Mode::TRAJ` 分支增加框架角通路 ——
  `use_motor_angle_` 为真时改用 `fpv_angle_spec` 下发**框架角**、`tgt_inertial_*` 也存框架角，
  命令 / 下发值 / 判据三者同系，与一直好用的 `forward_cmd_use_fpv` 转发流同一套语义。
  `use_motor_angle=false` 的老路径完全不变。

  **实机验证**（2026-09-09，臂在收纳位）：`gimbal-rotate 15 nan -10` → `reached`，编码器 pan=0.2609（目标 0.2618）、
  tilt=-0.1583（目标 -0.1745）；`gimbal-rotate 0 nan 0` → `reached`，pan=0.0169、tilt=-0.0169。
  tilt 极性正确（`motor_sign_tilt=-1.0` 只作用于回读，不影响命令）。roll 传 NaN 时按「不关心」保持，正常。

  **第二处修复（同日，也已验证）**：动作完成后 `mode_` 原本退回 `IDLE`，而 `IDLE` 发的 `hold_spec()` 是
  **IMU 惯性保持**，于是刚摆好的框架角会随 IMU 偏航漂走（改前实测静置 0.013°/s，陀螺校准刚完更快：
  两次动作之间无任何指令，pan 自己从 -0.017 漂到 1.362 rad）。新增 `settle_after_traj_locked()` 替换
  TRAJ 退出时的三处 `mode_ = Mode::IDLE`：到位锁目标角、超时/取消锁当前角，统一转入持久 FPV 框架角保持
  （`use_motor_angle=false` 时仍走 IDLE，老行为不变）。刻意**不** `++cmd_gen_`，否则等结果的 action 会误判 preempted。
  验证：静置 60s 漂移 0.00035 rad ≈ **0.0006°/s（改善约 20 倍**，已是编码器 0.01° 量化噪声量级）；
  `15 nan -10` → reached 且停在 pan 0.2616 / tilt -0.1747；取消 → `cancelled` 并就地停住；
  后到命令 → 前一条正确报 `preempted`。
  代价：FPV 变常驻，而 FPV 下倾角保护失效（该板本就 `fpv_on_startup: true`，实际暴露面变化不大）。

### 上实机建议顺序

1. `--dry-run` 看等效指令；2. `status`（error_code 应为 0）；3. `enable`；4. `observe --speed slow`；
5. `dolly 0.05 --speed slow`；6. 云台直连先 `gimbal-rotate 0 nan 0`（小角度、单轴）；7. `stow`。
任一步出问题，用日志里打印的等效 ros2 命令原样复现，即可区分是 API 还是 Commander。
