# robot_arm_driver — 臂 J1-3 CANopen 硬件层（ros2_canopen）

robot_arm 机械臂 J1-3（VEICHI **RB200-CA** 关节模组 ×3，CiA402/SocketCAN）的
ros2_control 硬件层。自研 CANopen 栈（CANopenLinux + ArmHardwareInterface + arm_node）
已于 2026-07 全面下线，替换为本方案。迁移全记录见 `docs/ros2_canopen迁移.md`。

## 目录结构

```
robot_arm_driver/               ← 容器目录（本身不是 ament 包，没有 package.xml）
├── arm_driver/                 ← ament 包，包名 robot_arm_driver（$(find robot_arm_driver) 指这里）
│   ├── config/
│   │   ├── can.yaml            ← ★ CAN 口唯一修改点 ★
│   │   ├── canopen/
│   │   │   ├── bus.yml         ← 总线拓扑：节点/模式/换算/PDO/初始化 SDO
│   │   │   └── RB200-CA.eds    ← 从站对象字典（按厂商手册第 4 章逐条转写）
│   │   └── test_controllers.yaml   ← 自测栈控制器配置（jsb + arm_controller）
│   ├── include/robot_arm_driver/   ← 共享头（包内自用，不 install/export）
│   │   ├── motor_unit_converter.hpp ← 单位换算 rad↔pp（单一权威，524288 counts/rev）
│   │   └── sdo_client.hpp      ← 极简 SDO 客户端 + 主站心跳（标定工具共用）
│   ├── launch/
│   │   ├── test_arm.launch.py  ← 自测入口（mock / vcan 假从站 / 真机 三合一）
│   │   └── motor_debug.launch.py   ← 单电机调试平台（后端+GUI，一个 launch 一个电机）
│   ├── scripts/                ← Python 脚本（新增 .py 必须进 CMake install(PROGRAMS)）
│   │   └── motor_debug_gui.py  ← 调试平台 PyQt5 前端（只做界面，不碰 CAN）
│   └── src/                    ← 按角色分类（2026-08-17 整理）
│       ├── hardware/           ← ros2_control 硬件插件
│       │   └── unwrap_robot_system.cpp ← RobotSystem + 绝对编码器 2π 折算
│       ├── nodes/              ← 常驻 ROS 节点
│       │   ├── arm_driver_services.cpp ← /arm_node/{enable,disable,recover}
│       │   └── motor_debug_backend.cpp ← ★ 调试平台总线后端（见下节）★
│       ├── tools/              ← 一次性 CLI 工具（占总线，先停 ros2_control 栈再跑）
│       │   ├── set_encoder_zero.cpp   ← 编码器零点标定（HM 方法 35，断电保持）
│       │   ├── set_motor_limits.cpp   ← 限速/软限位/最大转矩 查看与设定（rad 输入）
│       │   └── unit_convert.cpp       ← 命令行换算器 rad↔pp（改 bus.yml 时用）
│       └── deprecated/         ← 停用但保留参考（不编译）
│           └── gimbal_v2_bridge.cpp   ← 云台桥（2026-07-31 停用，缘由见 CMakeLists）
└── ros2_canopen/               ← vendored 上游快照（humble 0.2.13 精简版，见其 VENDOR.md）
```

> **为什么是容器目录？** colcon 扫到 `package.xml` 就不再深入子目录——ros2_canopen
> 若直接放进包里会被构建系统忽略。因此包本体挪到 `arm_driver/` 子目录（包名不变、
> 外部引用不受影响），与 vendored 源码平级共存。

## 架构

```
JointTrajectoryController (arm_controller)          ← 上层照旧（MoveIt/commander/…）
        │ 命令接口 ↔ 402 模式：position→IP(7)  velocity→PV(3)  effort→PT(4)
canopen_ros2_control/RobotSystem                    ← URDF backend=real 挂载
        │ 内嵌 DeviceContainer（lely master + 3×Cia402Driver）
CANopen 总线  SYNC 10ms │ RPDO2: 60C1:01(IP目标)+60FF(PV目标) │ TPDO2: 6064+606C(反馈)
        │ SocketCAN（接口名见 config/can.yaml）
RB200-CA ×3（node 1/2/3，波特率 500k）
```

- **IP 模式目标写 60C1:01**（RB200 特性，607A 仅 PP 用），插补周期 60C2 必须
  = SYNC 周期 = 10ms（三处联动，改周期要一起改）
- **单位**：与你交互的一切（/joint_states、JTC 轨迹、MoveIt、标定工具入参）都是 **rad**；
  pp（指令单位）只存在于 CAN 总线上。换算单一权威 = `include/robot_arm_driver/motor_unit_converter.hpp`
  （自研栈 1:1 保留件，524288 counts/rev 实测校准）——运行时驱动按 bus.yml 的
  `scale = 83443.026748 counts/rad`（同源数值）换算；手工换算用
  `ros2 run robot_arm_driver unit_convert`（rad2pp / pp2rad / 无参数=速查表）
- 方向反了改从站 **Pn002**（或 607E 极性位），不要改 scale 符号

## 快速开始

```bash
# 编译（工作区惯例必须带 --symlink-install，混用会报 existing path cannot be removed）
colcon build --symlink-install --packages-up-to robot_arm_driver

# ① mock 干跑（不碰 CAN，验证配置/加载）
ros2 launch robot_arm_driver test_arm.launch.py mock:=true

# ② vcan 假从站全链路（已验证通过：SDO 配置→402 使能→IP 轨迹→反馈闭环）
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
ros2 launch robot_arm_driver test_arm.launch.py can_interface:=vcan0 fake_slaves:=true demo:=true

# ③ 真机（先过 docs/ros2_canopen迁移.md 的首上电检查单）
sudo ip link set can0 up type can bitrate 500000 && sudo ip link set can0 txqueuelen 128
ros2 launch robot_arm_driver test_arm.launch.py            # demo:=true 确认安全后再加
```

**通过标准**：日志出现 `Initialisation successful` → jsb / arm_controller 均
`Configured and activated` → `/joint_states` 有 Joint1-3 → `demo:=true` 时
Joint1 三秒走到 0.1 rad 再回零。

整机（含云台 J4-6、MoveIt、commander）用 `ros2 launch robot_arm_bringup real.launch.py`。

## 配置

| 文件 | 作用 | 备注 |
|---|---|---|
| `config/can.yaml` | **CAN 口唯一修改点**，test_arm 与 real.launch 默认值都读它 | symlink 安装，改完免编译；命令行 `can_interface:=xxx` 可临时覆盖 |
| `config/canopen/bus.yml` | 节点 1/2/3、模式注册、scale、PDO 映射、上线 SDO（含 `boot_timeout_ms: 2000`） | 构建时 dcfgen 校验并生成 master.dcf / joint_N.bin |
| `config/canopen/RB200-CA.eds` | 从站对象字典 | 依据《RB200-CA 简版手册 V1.0》转写，非厂商官方；拿到官方 EDS 后替换比对 |
| `config/test_controllers.yaml` | 自测栈 JTC 配置 | JTC 只 claim position（RobotSystem 一关节同时只允许一个命令接口） |

## 单电机调试平台（motor_debug_backend + motor_debug_gui.py）

单个电机的集中调试入口，**不依赖 ros2_control 栈**，一个 launch 拉一个电机：
**C++ 总线后端**（`motor_debug_backend`，独占 SocketCAN + SDO + 主站心跳，402 状态机
与单位换算权威都在这边）+ **PyQt5 前端**（`scripts/motor_debug_gui.py`，只做界面，
经话题/服务交互、不碰 CAN，换算系数启动时从后端参数取）。接口全部复用
`canopen_interfaces`（COTargetDouble / COReadID / COWriteID），无自定义消息。

功能：402 使能/失能/故障复位/NMT 复位、**PP/PV/PT 三模式点动**（rad 系输入，自动
换算 pp；PP 带 ±步进点动，PV 带按住即动/松开即停方向点动）、**一键回零**（任意
模式下走 PP 回 0 rad）、**一键软限位**（jog 到边界后把当前位置写成 607D 下/上限；
清除 = 写回默认满量程即不生效，均配固化按钮）、**编码器置零**（同
set_encoder_zero 的 HM 方法 35 流程）、**限制参数**查看/设定（同 set_motor_limits
对象表）、**SDO 裸读写**、EEPROM 固化、100ms 实时反馈（位置/速度/转矩/**电流
6078**（配 6075 额定算负载率）/状态字/模式/故障码）+ 30s 滚动实时曲线（含电流）。
三模式目标都带**滑杆**（滑杆↔数值框双向同步，拖动只改数值、**松手才发送一次**——
与 joint_position_gui v1.1 同一教训：PP 拖动即发会触发驱动器不停重规划）。界面为仪表盘式布局（2026-08-17 v2）：左栏大字
状态+伺服+工具弹窗，右栏模式分段切换、执行/停止固定位置；未使能时运动区置灰、
后端断线整体置灰。

```bash
# ⚠️ 后端独占总线控制字：先停 ros2_control 栈（test_arm / real.launch）
ros2 launch robot_arm_driver motor_debug.launch.py node_id:=1          # J1，CAN 口读 can.yaml
ros2 launch robot_arm_driver motor_debug.launch.py node_id:=2 can_interface:=vcan0
```

- 只跟指定节点通信——**台架上总线挂几个电机都行**，不需要裁剪 bus.yml/URDF；
  后端节点名带 node_id（`motor_debug_backend_<n>`），多开互不串
- 任一进程退出（关窗 / 后端连不上 CAN / Ctrl-C）整个 launch 一起退；后端退出路径
  无条件清零 PV/PT 目标并写 Shutdown 失能，不吃 ros2_control_node 那个 SIGABRT
  不失能的坑（已知问题第 2 条）
- ⚠️ 力矩模式下"0"不是"停"是自由下垂；失能/急停会让重力关节下沉，先确认下方无人
- PP/PV/PT 都是驱动器内部闭环（SDO 写目标即可）；IP 模式需要 SYNC+PDO，
  属于 ros2_control 栈，本工具不做
- 后端服务也可脱离 GUI 用命令行调（`ros2 service call /motor_debug_backend_1/enable
  std_srvs/srv/Trigger` 等），脚本化测试直接打服务即可

## 运行时接口

- **使能/失能/故障恢复**（commander 兼容，由 arm_driver_services 提供）：

  ```bash
  ros2 service call /arm_node/enable  std_srvs/srv/Trigger   # 组件 active + 402 使能
  ros2 service call /arm_node/disable std_srvs/srv/Trigger   # 停控制器 + 失力矩（急停用这个）
  ros2 service call /arm_node/recover std_srvs/srv/Trigger   # inactive→active（含故障复位）
  ```

- **编码器零点标定**（C++ 独立工具，HM 方法 35，零点写入编码器片内、断电保持）：

  ```bash
  # ⚠️ 先停掉 ros2_control 栈；关节手动摆到机械零位、静止后执行
  ros2 run robot_arm_driver set_encoder_zero can0 1        # 只标 J1
  ros2 run robot_arm_driver set_encoder_zero can0 1 2 3    # 三关节依次标零
  # 标完重启驱动器（断电重上），再起栈确认 /joint_states ≈ 0
  ```

  机制：6098h=35（以当前位置为零点，无运动）→ 控制字 bit4 触发 → 6064h 归零。
  与旧栈同名工具同一机制（实测可用），已在 vcan 假从站上回归通过。

- **速度/加速度上限、软限位、最大转矩**（C++ 独立工具，rad 单位输入、自动换算 pp）：

  ```bash
  ros2 run robot_arm_driver set_motor_limits can0 1                       # 只读显示（pp↔rad 双单位）
  ros2 run robot_arm_driver set_motor_limits can0 1 --vel 1.5 --acc 2.0   # 改 PP 速度/加速度
  ros2 run robot_arm_driver set_motor_limits can0 2 --min -0.981 --max 2.959 --save  # 软限位并固化
  # 选项：--vel --maxvel --acc --dec --qstop (rad/s, rad/s²) | --min --max (rad) | --torque (%) | --save
  ```

  ⚠️ 6081/6083/6084/6085 每次起栈会被 bus.yml SDO 段重写（这是它们的长期配置点，
  工具会打印 rad→pp 换算值方便回填）；软限位 607D 不在 bus.yml 里，工具 + `--save`
  即正式配置方式。

- **SDO 直读直写**（每个关节驱动节点自带，栈运行时的诊断用）：

  ```bash
  ros2 service call /joint_1/sdo_read canopen_interfaces/srv/CORead "{index: 0x603F, subindex: 0}"  # 故障码
  # 参数存 EEPROM：1010:02h 写 ASCII "save"（0x65766173）
  ```

- **模式切换 = 控制器切换**：JTC 抓 position 接口 → 自动切 IP；换成速度/力矩类
  控制器抓 velocity/effort 接口 → 自动切 PV/PT，无需手动写 6060。

## 已知问题与坑（真机也适用）

1. **`boot_timeout_ms` 必须显式配**：402 驱动 boot 等待默认 20ms，必超时。bus.yml
   已配 2000ms，别删。
2. **Ctrl-C 退出时 ros2_control_node 以 SIGABRT 挂掉**（`terminate called without an
   active exception`，exit code -6）：0.2.13 上游 DeviceContainer 析构 bug。
   **⚠️ 2026-08-12 实机实测修正：此前本条写「发生在电机已失力矩之后，无功能影响」是错的。**
   实测抓 RPDO1 共 11631 帧，控制字一直是 `0x001F`（Operation Enabled）直到总线静默 ——
   **进程死时电机仍然使能带着力矩**，机械臂"硬"着不放；且 master 没干净关闭会让驱动器/
   PCAN 卡在坏状态（下次 launch 报 `SDO protocol timed out`、can0 变 NO-CARRIER，
   见 CLAUDE.md 高频坑第 1 条）。
   这条路径上**任何进程内钩子都拿不到执行机会**（生命周期 `on_shutdown`/`on_cleanup`
   与 `rclcpp::on_shutdown` 都验证过，`UnwrapRobotSystem` 里保留的重写只对正常
   deactivate 路径有效）。
   **当前正确做法**：关停前先显式失能 —— `ros2 service call /arm_node/disable
   std_srvs/srv/Trigger`（或 `/robot_arm/enable "{enable: false}"`），再 Ctrl-C。
   崩溃后的兜底：`cansend can0 000#8100`（NMT Reset Node 广播，驱动器复位即掉力矩）。
   ⚠️ 失能会让 J2/J3 失去支撑、机械臂下沉，操作前先确认下方无人无障碍。
   **TODO**：做成独立 SocketCAN 工具（写控制字 0x0006，与 `set_encoder_zero` 同套路）
   并挂进 launch 的 `OnShutdown`，把这一步自动化。
3. **假从站要比主站先起**：test_arm.launch 的 fake_slaves 模式已内置主站延迟 3s。
4. EDS 为手册转写，个别对象手册自身有出入（6070 类型、60C2:01 范围等）；真机 SDO
   Abort 时用 `candump` + 厂商 VCSDSoft_L 工具核对。
5. 升级 vendored ros2_canopen 的方法见 `ros2_canopen/VENDOR.md`（保留 9 包精简版）。
6. **全编译时 colcon 报 `1 package had stderr output: lely_core_libraries`**：
   正常噪音、非失败——该 vendored 库构建时打补丁的提示、setuptools/autoconf
   过时告警、libtool relinking 都走 stderr，每次编译稳定出现。真正的失败
   看 Summary 里有无 `failed` / 输出里有无 `error:`。

## 相关文档

- `docs/ros2_canopen迁移.md` — 迁移全记录、验证状态、真机首上电检查单
- `docs/RB200-CA机器人关节模组使用说明书（简版）V1.0.pdf` — 厂商手册（对象字典出处）
- `ros2_canopen/VENDOR.md` — vendored 快照来源/精简清单/升级方法
