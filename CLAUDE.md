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
- **HID 设备互斥**：云台（Joint4-6）的 HID 只能被一个进程占用。`real.launch.py` 现行路径是单 controller_manager 经 `CameraHardwareInterface` 管 J4-6；不要同时再起独立的 `robot_gimbal_node`，会互相抢设备。`robot_camera_node` 只占 V4L2，与 HID 不冲突。
- **总线接口是硬约定**：新增任何上层控制节点，命令只发 `/arm_controller/joint_trajectory`（或 MoveIt 走 `/arm_controller/follow_joint_trajectory` Action），状态只从 `/joint_states` 读。不要绕过总线直连某个后端，否则破坏 Gazebo / MuJoCo / 实物三后端无感切换。

## robot_arm_driver / ros2_canopen

- `ros2_canopen/` 是**精简 vendored** 的第三方栈，改动前先读 [robot_arm_driver/ros2_canopen/VENDOR.md](robot_arm_driver/ros2_canopen/VENDOR.md)，尽量把定制放在 `arm_driver/` 而不是改 vendored 源码。
- EDS（`RB200-CA.eds`）和 `bus.yml` 是按 RB200-CA 手册**自己写的**，不是厂商提供，改对象字典要对照手册（工作空间 `docs/RB200-CA*` 有手册和 SOP）。IP 插值模式的位置命令写 **60C1:01**。
- 驱动层自测入口：`ros2 launch robot_arm_driver test_arm.launch.py`（mock / vcan 假从站 / 真机三合一），迁移细节见工作空间 `docs/ros2_canopen迁移.md`。
- 旧 `arm_node`（自研 CANopenLinux 栈）已于 2026-07 下线，不要再往上面加功能。

## 文档位置

- 设计文档在**工作空间根目录** `docs/`（不在本包内）：分层重构与 Sim2Real 方案、ros2_canopen 迁移、Commander 框架、9dof 全身控制设计等。
- 拍摄朝向/可达域的数学定义以 `robot_arm_matlab/机械臂拍摄朝向与可达域分析.md` 为准（roll=0，朝向 = 2DOF：α pan / β tilt）。
