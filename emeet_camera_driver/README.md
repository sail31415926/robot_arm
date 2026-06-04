# EMEET Integrated Camera Driver

本模块负责 EMEET 摄像头的硬件驱动与 PTZ 控制，集成了 UVC 视频流采集与 HID 硬件控制功能。

## 0. 环境安装与编译 (从零开始)

如果你是第一次拿到项目代码，请按照以下步骤完成环境配置与编译：

```bash
cd ~/eMeet_ws
# 1. 环境初始化：自动安装系统依赖、创建 .venv 并配置 Python 库 (NumPy, OpenCV, torch 等)
bash src/E7009/robot_arm/emeet_camera_driver/scripts/setup_local_env.sh

# 2. 编译项目：完成 C++ 节点编译与 Python 脚本安装
bash src/E7009/robot_arm/emeet_camera_driver/scripts/build.sh
```

## 运行流程 (从新终端开始)

### 1. 一键启动驱动与测试 UI

这是最推荐的测试方式，只需一个指令即可同时开启相机驱动和验证工具。

**注意**：在启动前，请确保你已经通过 `bash src/E7009/robot_arm/emeet_camera_driver/scripts/build.sh` 完成了项目编译。

打开终端：

```bash
cd ~/eMeet_ws
# 激活局部环境
source .venv/bin/activate
# 加载 ROS 2 工作空间
source install/setup.bash
# 启动集成测试 Launch 文件
ros2 launch emeet_camera_driver test_all.launch.py
```

#### 画面不显示时如何自检（严谨）

`test_all.launch.py` 会将驱动私有话题 `~/image_raw` 重映射到统一话题 `/camera/image_raw`。请用下面命令确认发布者存在：

```bash
ros2 topic info -v /camera/image_raw
```

如果 `Publisher count: 1` 且发布者是 `emeet_camera_node`，说明 UVC 已出流；若为 0，说明驱动未成功启动或未加载正确的 launch 文件。

### 画面流与自检命令

驱动会发布图像话题供显示和算法使用：\n- **画面流**：`/camera/image_raw`（对应驱动私有话题 `~/image_raw`）\n\n自检命令：\n```bash\nros2 topic info -v /camera/image_raw\n```

### 画质档位（参数）

在配置文件中可调整画面流参数：`width/height/fps/pixel_format`\n\n> 注意：当分辨率升到 4K 时，发布 `bgr8` 原始图会显著增加 CPU/内存带宽压力；请根据实际算法算力进行选择，并根据需要开启/关闭 raw 或 compressed 发布（参数 `publish_display_raw/publish_display_compressed`）。

#### 关于 GUI 显示环境

测试 UI 为 PyQt5 程序，需要图形显示环境（Linux 桌面 / VNC / X11 转发）。如果在纯 SSH 环境下出现 `qt.qpa.xcb: could not connect to display`，请改用 Linux 本机桌面终端或配置好 X11/VNC 后再运行 UI。

### 2. 手动分步启动 (开发调试用)

#### 启动硬件驱动节点

```bash
# ... 进入环境并 source ...
ros2 run emeet_camera_driver camera_driver_node
```

#### 启动验证工具 (GUI)

```bash
# ... 进入环境并 source ...
# 方式 A: 通过 ROS2 运行 (推荐)
ros2 run emeet_camera_driver camera_test_gui.py

# 方式 B: 直接运行源码脚本
python3 src/E7009/robot_arm/emeet_camera_driver/test/camera_test_gui.py
```

## 注意事项

- 本驱动会自动识别 Pixy, E7002, Piko 等型号。
- 如果遇到 UVC 无法打开，请检查 `/dev/video*` 权限或是否有其他程序占用。
- 云台速度控制建议“按住方向键”，UI 会以固定频率持续发布 `/emeet_camera_node/cmd_vel`；松开会自动发送 0 速度停止。
- 如果出现“云台自己动/启动自动回中”，这是设备处于 Follow/Tracking 模式导致。驱动默认会在 HID 连接后自动设置 Standard(0x00) 模式（带延迟与重试）。如需关闭该行为，可在 launch 参数中设置 `force_standard_mode_on_startup:=false`。

### 常见问题：Service 返回 HID not connected / 云台不响应

如果调用 `/emeet_camera_node/go_to_angle` 返回 `HID not connected.`，说明驱动未成功连接 HID 控制接口（此时速度/角度控制都不会生效）。

请按顺序自检：

```bash
# 1) 确认 hidraw 节点存在且权限正常
ls -l /dev/hidraw*

# 2) 通过 udev 查看设备是否是 Pixy (328f:00c0) 以及 interface 信息
udevadm info -a -n /dev/hidraw5 | head -n 80

# 3) 用 Python/hidapi 验证 open_path 是否可用（能打开说明系统侧 hidapi 没问题）
python3 - <<'PY'
import hid
VID=0x328f
PID=0x00c0
ds = hid.enumerate(VID, PID)
print("count =", len(ds))
for i,d in enumerate(ds):
    print(i, "path=", d.get("path"), "if=", d.get("interface_number"))
assert ds
dev = hid.device()
dev.open_path(ds[0]["path"])
print("open_path OK")
dev.close()
PY
```
