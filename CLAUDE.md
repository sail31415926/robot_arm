# robot_arm — Claude Code 工作约定

架构、包结构、控制模式、Commander 接口详见 [README.md](README.md)，此处不重复，只记录**动手改代码前必须知道的约定和坑**。

## 环境与编译

- 每个终端先 `source /opt/ros/humble/setup.bash`，再 `source .venv/bin/activate`（工作空间根目录的 venv，`--system-site-packages`），编译后再 `source install/setup.bash`。顺序不能反。
- pip 依赖以 `requirements.txt` 为准，**numpy 必须 <2**（1.26.4）。
- 编译统一在工作空间根目录（`~/E7009_ws`）执行 `colcon build --symlink-install --packages-select <包名>`。
- vendored 的 ros2_canopen（约 10 个包）编译很重，建议限制并行度（如 `--parallel-workers 2` 或 `MAKEFLAGS=-j4`），否则可能把机器卡死。

## 高频坑

- **ros2_control_node 崩溃/被杀后，CAN 总线可能残留异常**：CANopen master 没有干净关闭时，驱动器或 PCAN-USB 适配器会卡在坏状态，下次 launch 在硬件激活阶段报 `SDO protocol timed out` / `Failed to activate 'JointX'`，can0 甚至变 NO-CARRIER。重新 launch 前先 `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 500000`，再 `timeout 3 candump can0` 确认 0x701/702/703 三个心跳都在；心跳缺失就断电重启机械臂。另：启动时每关节刷 ~11 条 `AsyncUpload:6502 General error` 是驱动例行查询、无害；但 **timed out 不是**，那是总线问题。
- **Ctrl-C 关停不会让电机失能**（2026-08-12 实机实测，安全相关）：`ros2_control_node` 退出时以 **SIGABRT** 挂掉（vendored 0.2.13 DeviceContainer 析构 bug），**进程死时驱动器仍处于 Operation Enabled、带着力矩** —— 抓 RPDO1 共 11631 帧，控制字一直是 `0x001F` 直到总线静默，全程没有 halt 报文。上游只在 `on_deactivate` 里 `halt_motor()`，而这条路径根本走不到那里；**任何进程内钩子都拿不到执行机会**（生命周期 `on_shutdown`/`on_cleanup` 与 `rclcpp::on_shutdown` 都实测无效，`UnwrapRobotSystem` 里保留的重写只对正常 deactivate 路径有效）。**这个带力矩状态会一直持续到断电**：RB200-CA 不会自我保护 —— EDS 的 `1016:01`（消费者心跳，即驱动器监测**主站**心跳的超时）默认 0 = 禁用，`bus.yml` 也没配，驱动器根本不知道上位机已经死了。（想加第二道防线就给各关节 `sdo:` 段配 `1016:01`，让它在主站心跳消失后自己报 ER.E20/E21 断力矩；未验证，需先确认 RB200 的该保护是自由停机还是斜坡停机、以及是否需要手动 recover 才能再起栈。）

  **2026-08-21 起有了失能工具，但关停钩子默认不挂**（`auto_disable_on_shutdown`，默认 `false`）—— 这是**产品决定**：默认保持既有行为「Ctrl-C 后臂停在原地、保持力矩、不下沉」。两边都有真实风险，别擅自改默认值：
  - **默认（false）**：不下沉，但驱动器带电且无人控制 → ① 你以为关了去搬臂时它会顶回来（伤手/顶坏谐波减速器）② 臂压住东西时推不开 ③ master 没干净关闭 → 下次 launch 报 SDO timed out / NO-CARRIER（=上面高频坑第 1 条，**这条与下沉无关**，手动跑一次工具就是解药）。
  - **开启（true）**：`real.launch.py` / `test_arm.launch.py` 挂两个钩子 —— `OnProcessExit`(ros2_control_node) 是主路径（CM 一死总线就空闲，SDO 最干净），`OnShutdown` 兜底（CM 从未起来 / launch 被 SIGTERM，此时主站可能还在 200Hz 写 0x001F，SDO 抢不过 → 工具自动升到 NMT 层）；两条幂等。代价是断力矩那刻 J2/J3 **下沉**（`60FE` 抱闸输出在 RB200-CA 上只读、驱动器自管，上位机控制不了）。下沉幅度**未实机实测**——仿真那个「J2 从 +0.500 砸到下限」是 Gazebo 关节无摩擦的最坏情况，真机谐波减速器摩擦大得多，可能只是慢慢垂下来。

  **手动失能随时可用**（不受开关影响，推荐关停前顺手跑，尤其是为了避开高频坑第 1 条）：`ros2 run robot_arm_driver disable_motors can0`；或 `ros2 service call /arm_node/disable std_srvs/srv/Trigger`；最终兜底 `cansend can0 000#8100`。工具顺序：Quick Stop `0x0002`（按 6085 斜坡减速，605A 默认 2 ⇒ 停稳后自动到 Switch on disabled）→ 若停在 Quick stop active 仍带力矩（605A=5/6 锁轴）补 `0x0000` → Shutdown `0x0006` + 读状态字确认 → 失败则 `NMT Reset Node`。它**屏蔽 SIGINT/SIGTERM/SIGHUP/SIGPIPE**（否则挂在钩子上会在写控制字前被关停信号打死；SIGPIPE 那条最隐蔽——launch 先退出会关掉 stdout），改它时别把这个去掉。⚠️ **截至 2026-08-21 工具未经真机验证**（开发机无 CAN 硬件、sudo 拿不到密码建不了 vcan）；验证路径见 `robot_arm_driver/README.md` 工具章节的 vcan 假从站方案。
- **节点里永远不要硬编码 `use_sim_time=True`**：实物上没有 `/clock`，`use_sim_time=true` 的节点**所有 ROS 定时器永不触发**（症状极隐蔽：订阅回调都正常、只有 timer 驱动的逻辑静默死掉，如 GUI 的 TF 位姿面板卡 `--`）。正确做法是 launch 按后端传参（`robot_arm_bringup.launch_common.gui_node(use_sim_time=...)`，gazebo=true、mujoco/real=false——MuJoCo 暂不发 /clock）。
- **编译完新包要重新 source**：终端里 `install/setup.bash` 是 source 时刻的快照，之后新编译的包（launch 报 `package 'xxx' not found`，而 install/ 里明明有）在该终端不可见，重新 source 即可。

- **新增 Python 脚本必须手动登记安装**：`robot_arm_debug`、`robot_arm_mujoco` 是 ament_cmake 包，用 `install(PROGRAMS ...)` 安装脚本。新增 `.py` 后忘了加进对应 `CMakeLists.txt`，编译不会报错，但运行时节点找不到/ModuleNotFoundError 崩溃。
- **云台控制路径（2026-07-28 起全面用 V2，V1 已删）**：臂侧 J4-6 经 `robot_gimbal_driver_v2/GimbalForwardingInterface`（转发插件，不碰串口）接入，**直连板端原生话题**（板端 `robot_gimbal_node_v2` 自己订阅 `/robot_gimbal_v2/forward_cmd`、自己发布 `/robot_gimbal_v2/joint_states_raw`），臂侧**不再拉 `gimbal_v2_bridge`**（2026-07-31 停用：会双重驱动，且把转发流升级成 `GimbalCommand.POSITION`，而板端 POSITION 会解冻 FROZEN → JTC 保持流能解冻 FREEZE；源码保留未删，`absolute_mode` 换算若要重启用须换不重叠的接法）。**执行节点 `robot_gimbal_node_v2`（串口唯一拥有者）跑在云台板端，不在本工作空间启动** —— 这是与 V1 最大的差异：臂侧和云台是**两台机器**，必须同网段 + 同 `ROS_DOMAIN_ID`、且**不能设 `ROS_LOCALHOST_ONLY=1`**，否则回读永远收不到（转发插件刷 `No feedback from robot_gimbal_node yet` 并退化为指令回显）。V1 的包（`robot_gimbal_driver` / `_node` / `_interfaces` / `_bringup`）与 `robot_camera_node` 已随换代删除，遇到 `/robot_gimbal/...`（无 `_v2`）话题名说明那台机器装的还是旧版，重编即可。详见工作空间 `docs/云台控制路径融合方案.md`。
- **笛卡尔动作的到位判据不能用末端位姿**（2026-07-31 修）：换代 V2 后规划末端是 `gimbal_tool0`，它在**云台 J4-6 之后**，末端位姿 = f(J1..J6)。云台回读一旦不收敛或有静差（板端未上电、跨机 DDS 不通、GCU 自稳环让 IMU 角与关节指令不一致），`is_at_pose` 就永远不满足 → 所有笛卡尔动作走到 timeout 报错，哪怕臂早就到位了。现在的约定是：**规划/下发仍是 6 轴（云台跟着动），到位判据只看臂 J1-3**——统一走 `commander/motion_policy.hpp` 的 `is_at_joints_prefix(cur, 末点关节解, ARM_JOINT_COUNT)`，末点关节解由 `plan_and_execute` / `solve_and_send` 出参带出。新增等待循环别再用 `is_at_pose`（它只留作拿不到关节解时的退化兜底）。

- **Gazebo 起不来先查残留 gzserver**：Gazebo Classic 的 gzserver **不随 launch 一起退出**（Ctrl-C 常留下它）。残留进程占着 11345 端口，下次 launch 的 gzserver 直接 `bind: Address already in use` → exit 255，表现为「没有物理、没有图像、spawn_entity 报 /spawn_entity unavailable、spawner 全 FATAL」，而 rqt_image_view 只剩灰度渐变占位画面（很容易误判成相机配置错了）。起 launch 前先 `pgrep -x gzserver`，有就 `pkill -9 -x gzserver; pkill -9 -x gzclient`。注意**用 `-x`（精确进程名）不要用 `-f`**，`pkill -f gzserver` 会把你自己那条包含该字符串的命令一起杀掉。

- **总线接口是硬约定**：新增任何上层控制节点，命令只发 `/arm_controller/joint_trajectory`（或 MoveIt 走 `/arm_controller/follow_joint_trajectory` Action），状态只从 `/joint_states` 读。不要绕过总线直连某个后端，否则破坏 Gazebo / MuJoCo / 实物三后端无感切换。
- **控制模式只经 `mode_manager_node` 切**（2026-07 新增，见 README「控制模式仲裁」）：切位置/速度/力矩 = 调 `/robot_arm/switch_control_mode`（内部 `switch_controller`），**不要**自己直调 `controller_manager/switch_controller` 或旧 `/arm_node/set_mode_pv` 抢控制器，否则与 ModeManager 打架。速度命令发产品总线 `/robot_arm/cmd/joint_velocity`（类型 `ArmJointVelocityCommand`，2026-08-04 由裸 `Float64MultiArray` 换成类型化消息；ModeManager relay 到控制器 + 限幅 + 限位刹车 + 断流看门狗 + 急停闩锁），别直发 `/arm_velocity_controller/commands`。`real`/`gazebo` 的控制器名（`arm_controller` / `arm_velocity_controller`）必须一致，否则破坏后端无感。**`JOINT_EFFORT` 已于 2026-08-11 在仿真开通**（`arm_effort_controller` + URDF effort 接口 + pinocchio 重力补偿，见下条）；实物侧 `real.launch.py` 故意不配 `effort_controller`，切换会被拒。`ADMITTANCE` 仍为 P3 预留（需 F/T 传感器）。
- **力矩模式的「零」不是「停」**（2026-08-11 开通 JOINT_EFFORT 时踩到）：位置/速度模式下发 0 就是停住，力矩模式下发 0 是**自由下垂** —— 实测切进力矩模式后 J2 立刻从 +0.500 砸到下限 -0.981、J3 从 -1.000 砸到上限 +0.020。所以 `mode_manager_node` 的力矩通路是**定时节拍**（`effort_rate_hz`，默认 100Hz）而不是「收到指令才发」：下发 = `g(q)`（pinocchio 从 `/robot_description` 建模）+ 用户增量 − `d·q̇`（`effort_damping`）。三条连带约定：① 用户指令只是「重力之上的增量」，存起来由节拍消费，不直接下发；② `publishZero()` 对力矩模式**只清用户增量、不发字面 0**，急停/断流同理（力矩模式下"停住"就是继续托住）；③ 必须持续发 —— `ForwardCommandController` 会一直把最后一帧写给硬件，位形一变那帧重力力矩就不再平衡。`GravityModel::gravity()` 是 **fail-closed**：位形不全就整体放弃，用默认值顶替缺失关节会算出一个错误的**主动力**，比不补偿危险得多。纯重力补偿无耗散，仿真里仍有约 0.01 rad/s 单向漂移（Gazebo 关节无摩擦；真机谐波减速器摩擦大得多），要彻底锁住得加位置弹簧项，那已经是阻抗控制。
- **速度控制走的是 joint_trajectory，不是速度控制器**（2026-08-04 定案）：`JOINT_VELOCITY` 模式**不切控制器**，两条速度总线（`/robot_arm/cmd/joint_velocity`、`/robot_arm/follow_command`）都由 Commander 的 `VelocityStreamServer` 解算成 q̇、积分成角度，50Hz 位置流发到**和位置模式同一条** `/arm_controller/joint_trajectory`。切模式只是更新语义标志（`mode_manager_node` 的 `velocity_backend` 参数，默认 `trajectory`）。老的 PV 路径（切 `arm_velocity_controller` → CiA402 PV(3)）保留为 `velocity_backend:=velocity_controller`，只在需要驱动器内部速度环时才启用。**改回 PV 前先读下面第 5、6 条**，那些坑会一并回来。
- **速度控制的坑**（2026-08-04 开通速度接口时踩到，前 4 条与后端无关、永远有效）：
  1. **JTC 位置流千万别按主环频率发**：JTC 每收到一条新轨迹就丢弃旧的重新插值，100Hz 抢占会让它一直在「重启轨迹」而跟不动 —— 实测末端只有指令的 **20%~70%**（还随位形飘），压到 **50Hz**（`velocity_stream.rate_hz`）立刻回到 98%~100%。本仓 `servo_config.yaml` 把 MoveIt Servo 压到 50Hz 是同一个原因。另外单点的 `time_from_start` 必须与位置匹配成正确斜率（发「设定点 + q̇×lookahead」），否则执行速度被稀释成 `q̇ × dt/lookahead`。
     **⚠️ 那个 98%~100% 是 Gazebo 数据，别外推到实机**（2026-08-12 实测）：仿真位置接口是 `SetPosition` 运动学瞬移、实际位置恒等于设定点；实机 J1 @0.15rad/s 在 50Hz 下只有 **71%~74%**，且速度在 0~0.246 之间摆动（标准差 0.044）—— 用户报的"抖动很大"就是这个。机制是"周期(20ms) < 计划时长(lookahead 50ms)"使每条计划只执行 40%，而计划开头是样条最慢的一段。改成 20Hz（周期==时长）跟踪率回到 100.4%、领先量从 0.097 降到 0.022rad，但每段样条自身的加减速形状暴露成更大纹波（标准差 0.117）。另外两条已否掉的思路记在 `velocity_stream_server.cpp` 的 `rate_hz_` 注释里：把前瞻基准改成实测位置（臂几乎不动，只有 0.0064rad/s —— **那个领先量正是位置环产生速度所必需的误差**，不能消）、把时长按真实距离/q̇ 拉长（同样不动）。结论：调常数只能在跟踪率与纹波之间取舍，治本要换驱动器内部速度环 PV(3)（`velocity_backend:=velocity_controller`）。
  2. **积分步长不能用 ROS 时钟**：Gazebo 的 `/clock` 只有 10Hz，而速度环是 wall timer —— 用 `node->now()` 求 dt 的话，九成的 tick 看到 dt=0、第十拍看到 0.1s 再被上限一削，系统性丢步（实测积分速度只剩一半）。积分用 `steady_clock`。
  3. **Gazebo 位置接口是 `SetPosition`（运动学瞬移），不是伺服**：这既意味着轨迹跟踪在仿真里"完美"，也意味着任何陈旧位置命令都会表现为瞬移。仿真里看到的跟踪质量不能直接外推到实物。
  4. **自碰撞闸看的是 6 轴组合位形**：`MoveToJoint` 用当前云台回读填 J4-6，云台停在大角度时，臂目标本身没问题也会被 `/check_state_validity` 判 `collision`。批量测试脚本记得先把 6 轴一起摆正，否则会误判成"动作坏了"（排查过一轮）。
  5. **【PV 后端才有】换模会留下陈旧命令**：ros2_control 的命令接口在控制器停用后**值原样保留**。速度模式下机械臂走开了，位置命令缓冲还停在切换前那一刻，切回轨迹模式时被回放 —— Gazebo 里瞬移回旧位姿（实测 J1 跳 0.72rad），实物上是 IP 模式一次没人预期的高速运动。实物已在 `UnwrapRobotSystem::perform_command_mode_switch` 里把 `target_position` 播种成当前实测位置（**未经实机验证**）；Gazebo 改不了（`GazeboSystem` 命令数组在 pImpl 私有类里，子类连析构都实例化不出来，`on_activate` 是只 return SUCCESS 的空壳），只能由 ModeManager 在切换**前**快照位置、切换后补一条复位轨迹拉回来（仍会看到往返）。
  6. **【PV 后端才有】速度模式下云台没人管 + 切换竞态**：`arm_controller` 在两个后端都 claim J1-6，一被停云台就没有命令通道（Gazebo 里重力下垂带偏末端）。解法是 ModeManager 的 `hold_controllers`（传 `['gimbal_controller']`）。**停**保持控制器必须和主切换在同一次 STRICT 调用里原子完成 —— 分两次调的话第一次 deactivate 还没被实时循环应用，第二次就要 activate 同样 claim J4-6 的 `arm_controller`，STRICT 判资源冲突而失败，机械臂卡在速度模式回不去（实测复现）。**启**只能在主切换之后单独调（BEST_EFFORT）。保持控制器的死活由 ModeManager 自己记账，别在切换回调里嵌套调 `list_controllers` 去查（试过，段错误）。
  7. **轨迹类动作在速度模式下曾会静默失效**：`MoveToPose`/`MoveToJoint`/`TrajectoryShot` 全靠 JTC 执行，PV 后端下 JTC 被停用，轨迹发出去不动、只等到超时（"点收纳位没反应"就是这个）。现在这三个动作执行前会**自动切回 TRAJECTORY**（`ArmCommanderNode::ensure_trajectory_mode`，仍走 mode_manager 唯一入口）；trajectory 后端下这一步也顺带把速度流关掉，交接干净。
  8. **调试时先确认只有一套栈在跑**：`pkill -x arm_commander_node` **杀不掉**它 —— `/proc/comm` 只有 15 字符，进程名被截断成 `arm_commander_n`，`-x` 精确匹配失败且静默。结果是每次 relaunch 都叠一套，多个 commander 同时往总线写，测出来的数全是脏的（排查了很久）。用 `pgrep -af` 按完整 cmdline 杀，或 `ps -eo pid,args | grep E7009_ws/install`。

## robot_arm_driver / ros2_canopen

- `ros2_canopen/` 是**精简 vendored** 的第三方栈，改动前先读 [robot_arm_driver/ros2_canopen/VENDOR.md](robot_arm_driver/ros2_canopen/VENDOR.md)，尽量把定制放在 `arm_driver/` 而不是改 vendored 源码。
- EDS（`RB200-CA.eds`）和 `bus.yml` 是按 RB200-CA 手册**自己写的**，不是厂商提供，改对象字典要对照手册（工作空间 `docs/RB200-CA*` 有手册和 SOP）。IP 插值模式的位置命令写 **60C1:01**。
- 驱动层自测入口：`ros2 launch robot_arm_driver test_arm.launch.py`（mock / vcan 假从站 / 真机三合一），迁移细节见工作空间 `docs/ros2_canopen迁移.md`。
- 旧 `arm_node`（自研 CANopenLinux 栈）已于 2026-07 下线，不要再往上面加功能。

## git 提交与 Gerrit

- **仓库结构**：`robot_arm/`、`robot_gimbal/` 是各自独立的 git 仓库（嵌套在工作空间仓库内），提交要分别在各自目录内做；工作空间根目录的 `docs/` 属于外层仓库。
- **提交信息**：conventional commits + 中文描述（`feat(scope): ...` / `fix:` / `docs:` / `chore:`），正文写清动机和关键决策；参考 `git log` 既有风格。
- **Change-Id 必须有**（Gerrit 拒收没有的提交），标准流程（见 `docs/SETUP.md`）：

  ```bash
  # ① 装 commit-msg 钩子（每个新克隆仓库一次；已装过跳过）
  gitdir=$(git rev-parse --git-dir); scp -p -P 29418 <user>@<gerrit-host>:hooks/commit-msg ${gitdir}/hooks/
  # ② 已有提交漏了 Change-Id 时补（钩子装好后 amend 会自动加）
  git commit --amend --no-edit
  # ③ 推送走 Gerrit 评审流（不是直推 master）
  git push origin HEAD:refs/for/master
  ```

  离线时钩子也可从兄弟仓库复制：`cp ../robot_arm/.git/hooks/commit-msg <目标仓库>/.git/hooks/`。Gerrit 地址 `ssh://<user>@<gerrit-host>:29418/E7009/<repo>`。
- **推送前先同步远端**：`git fetch origin` 后若远端有新提交，`git rebase origin/master` 再推，冲突时注意语义级冲突（别只看文本——曾发生"我方废弃的功能远端正在用"的情况，要看对方代码意图再整合）。
- **提交范围要干净**：只提交本次工作相关文件；与任务无关的脏文件（他人的 .gitignore 改动、未跟踪目录）留给对应负责人。大二进制（如 26MB 的 `RTB.mltbx` 第三方安装包）不进 git，加 `.gitignore` 并注明获取方式。
- 提交/推送由用户明确要求时才做，不要顺手提交。

## 文档位置

- 设计文档在**工作空间根目录** `docs/`（不在本包内）：分层重构与 Sim2Real 方案、ros2_canopen 迁移、Commander 框架、9dof 全身控制设计等。
- 拍摄朝向/可达域的数学定义以 `robot_arm_matlab/机械臂拍摄朝向与可达域分析.md` 为准（roll=0，朝向 = 2DOF：α pan / β tilt）。
