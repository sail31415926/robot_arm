# robot_arm — Claude Code 工作约定

架构、包结构、控制模式、Commander 接口详见 [README.md](README.md)，此处不重复，只记录**动手改代码前必须知道的约定和坑**。

## 环境与编译

- 每个终端先 `source /opt/ros/humble/setup.bash`，再 `source .venv/bin/activate`（工作空间根目录的 venv，`--system-site-packages`），编译后再 `source install/setup.bash`。顺序不能反。
- pip 依赖以 `requirements.txt` 为准，**numpy 必须 <2**（1.26.4）。
- 编译统一在工作空间根目录（`~/E7009_ws`）执行 `colcon build --symlink-install --packages-select <包名>`。
- vendored 的 ros2_canopen（约 10 个包）编译很重，建议限制并行度（如 `--parallel-workers 2` 或 `MAKEFLAGS=-j4`），否则可能把机器卡死。

## 高频坑

- **ros2_control_node 崩溃/被杀后，CAN 总线可能残留异常**：CANopen master 没有干净关闭时，驱动器或 PCAN-USB 适配器会卡在坏状态，下次 launch 在硬件激活阶段报 `SDO protocol timed out` / `Failed to activate 'JointX'`，can0 甚至变 NO-CARRIER。重新 launch 前先 `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 500000`，再 `timeout 3 candump can0` 确认 0x701/702/703 三个心跳都在；心跳缺失就断电重启机械臂。另：启动时每关节刷 ~11 条 `AsyncUpload:6502 General error` 是驱动例行查询、无害；但 **timed out 不是**，那是总线问题。
- **节点里永远不要硬编码 `use_sim_time=True`**：实物上没有 `/clock`，`use_sim_time=true` 的节点**所有 ROS 定时器永不触发**（症状极隐蔽：订阅回调都正常、只有 timer 驱动的逻辑静默死掉，如 GUI 的 TF 位姿面板卡 `--`）。正确做法是 launch 按后端传参（`_arm_launch_common.gui_node(use_sim_time=...)`，gazebo/mujoco=true、real=false）。
- **编译完新包要重新 source**：终端里 `install/setup.bash` 是 source 时刻的快照，之后新编译的包（launch 报 `package 'xxx' not found`，而 install/ 里明明有）在该终端不可见，重新 source 即可。

- **新增 Python 脚本必须手动登记安装**：`robot_arm_debug`、`robot_arm_mujoco` 是 ament_cmake 包，用 `install(PROGRAMS ...)` 安装脚本。新增 `.py` 后忘了加进对应 `CMakeLists.txt`，编译不会报错，但运行时节点找不到/ModuleNotFoundError 崩溃。
- **云台控制路径（2026-07 融合后）**：`robot_gimbal_node` 是**唯一 HID 拥有者**且为 `real.launch.py` 必需常驻组件；机械臂侧 J4-6 经 `GimbalForwardingInterface`（无 HID 转发插件）接入。不要在插件或其他进程里直连云台 HID。`robot_camera_node` 只占 V4L2，与 HID 不冲突。详见工作空间 `docs/云台控制路径融合方案.md`。
- **总线接口是硬约定**：新增任何上层控制节点，命令只发 `/arm_controller/joint_trajectory`（或 MoveIt 走 `/arm_controller/follow_joint_trajectory` Action），状态只从 `/joint_states` 读。不要绕过总线直连某个后端，否则破坏 Gazebo / MuJoCo / 实物三后端无感切换。

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
  gitdir=$(git rev-parse --git-dir); scp -p -P 29418 zoulongyou@192.168.16.75:hooks/commit-msg ${gitdir}/hooks/
  # ② 已有提交漏了 Change-Id 时补（钩子装好后 amend 会自动加）
  git commit --amend --no-edit
  # ③ 推送走 Gerrit 评审流（不是直推 master）
  git push origin HEAD:refs/for/master
  ```

  离线时钩子也可从兄弟仓库复制：`cp ../robot_arm/.git/hooks/commit-msg <目标仓库>/.git/hooks/`。Gerrit 地址 `ssh://<user>@192.168.16.75:29418/E7009/<repo>`。
- **推送前先同步远端**：`git fetch origin` 后若远端有新提交，`git rebase origin/master` 再推，冲突时注意语义级冲突（别只看文本——曾发生"我方废弃的功能远端正在用"的情况，要看对方代码意图再整合）。
- **提交范围要干净**：只提交本次工作相关文件；与任务无关的脏文件（他人的 .gitignore 改动、未跟踪目录）留给对应负责人。大二进制（如 26MB 的 `RTB.mltbx` 第三方安装包）不进 git，加 `.gitignore` 并注明获取方式。
- 提交/推送由用户明确要求时才做，不要顺手提交。

## 文档位置

- 设计文档在**工作空间根目录** `docs/`（不在本包内）：分层重构与 Sim2Real 方案、ros2_canopen 迁移、Commander 框架、9dof 全身控制设计等。
- 拍摄朝向/可达域的数学定义以 `robot_arm_matlab/机械臂拍摄朝向与可达域分析.md` 为准（roll=0，朝向 = 2DOF：α pan / β tilt）。
