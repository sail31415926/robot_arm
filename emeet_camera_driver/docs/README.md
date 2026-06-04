# EMEET Integrated Camera & YOLO Workspace

这是一个集成了 EMEET 工业相机驱动与 YOLO 2D 目标检测算法的 ROS 2 工作空间。

## 目录结构
- `src/E7009/robot_arm/emeet_camera_driver`: 相机硬件驱动与控制模块。
- `src/yolo_ros`: 基于 YOLO 的 2D 目标检测与跟踪模块。

## 快速开始 (环境准备与编译)

### 1. 初始化局部环境
本项目使用 `.venv` 局部 Python 环境，以确保 NumPy 2.x 等依赖的隔离。
```bash
# 运行环境安装脚本 (只需运行一次)
bash src/E7009/robot_arm/emeet_camera_driver/scripts/setup_local_env.sh
```

### 2. 编译项目
```bash
# 运行一键编译脚本
bash src/E7009/robot_arm/emeet_camera_driver/scripts/build.sh
```

## 运行与测试
请进入各子包目录查看详细的 `README.md` 指引：
- [相机驱动运行指引](../src/E7009/robot_arm/emeet_camera_driver/README.md)
- [YOLO 检测运行指引](../src/yolo_ros/README.md)

## 开发规范
- 所有的运行测试脚本均放在各包下的 `test/` 目录中。
- 仅环境配置与编译使用 `.sh` 脚本，日常运行使用标准的 `ros2` 指令与 Python 脚本。
