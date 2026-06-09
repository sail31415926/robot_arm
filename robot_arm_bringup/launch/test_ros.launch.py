"""
@file   test_ros.launch.py
@brief  环境验证程序（移植到新设备时用）

把代码移植到新机器后，先跑这个检查整套运行环境是否就绪：
Python 依赖、ROS 核心/系统包、本工作空间已编译包、GPU 渲染。
逐项输出 [PASS]/[FAIL]/[WARN]/[SKIP]，结尾给汇总并以退出码反映结果
（有 FAIL → 退出码 1；只有 WARN → 退出码 0）。

用法（务必先 source 环境）：
  source /opt/ros/humble/setup.bash
  source .venv/bin/activate            # 若用 venv
  source install/setup.bash            # 验证工作空间包需要
  ros2 launch robot_arm_bringup test_ros.launch.py

说明：
  - 校验项对齐 requirements.txt 与当前包列表，新增依赖时请同步本文件。
  - 仿真(mujoco) / 强化学习(gymnasium 等) / GPU 缺失只告警，不算环境不达标。

@date    2026-06-08
@copyright Copyright (c) 2026 eMeet
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


# 环境检查脚本（用当前 PATH 上的 python3 执行，即实际跑节点的解释器）
ENV_CHECK = r'''
import importlib, sys, os, shutil, subprocess

GREEN, RED, YEL, CYAN, GREY, RST = "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[90m", "\033[0m"
fails, warns = [], []

def line(tag, color, name, info=""):
    print(f"  [{color}{tag:^4}{RST}] {name:<26} {GREY}{info}{RST}")

def check_py(mod, label=None, required=True, max_major=None, note=""):
    """检查一个 python 模块可导入；max_major 指定主版本上限(不含)。"""
    label = label or mod
    try:
        m = importlib.import_module(mod)
        ver = getattr(m, "__version__", "") or getattr(m, "version", "")
        ver = str(ver) if not callable(ver) else ""
        if max_major is not None and ver:
            try:
                if int(ver.split(".")[0]) >= max_major:
                    line("FAIL", RED, label, f"{ver} 违反 <{max_major} 约束"); fails.append(label); return
            except ValueError:
                pass
        line("PASS", GREEN, label, f"{ver}  {note}".strip())
    except Exception as e:
        if required:
            line("FAIL", RED, label, f"导入失败: {type(e).__name__}: {e}"); fails.append(label)
        else:
            line("WARN", YEL, label, f"未安装（{note}）" if note else "未安装"); warns.append(label)

def check_ros_pkg(pkg, required=True):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(pkg)
        line("PASS", GREEN, pkg)
    except Exception as e:
        if required:
            line("FAIL", RED, pkg, f"未找到（apt 没装 / 未 source）"); fails.append(pkg)
        else:
            line("WARN", YEL, pkg, "未找到"); warns.append(pkg)

def header(title):
    print(f"\n{CYAN}== {title} =={RST}")

print(f"\n{CYAN}========== 环境验证 (robot_arm) =========={RST}")

header("Python 解释器")
v = sys.version_info
line("PASS" if v >= (3, 8) else "WARN", GREEN if v >= (3,8) else YEL,
     "python3", f"{v.major}.{v.minor}.{v.micro}  ({sys.executable})")
in_venv = sys.prefix != sys.base_prefix
line("PASS" if in_venv else "WARN", GREEN if in_venv else YEL,
     "virtualenv", "已激活" if in_venv else "未激活（裸系统 python）")

header("必需 pip 依赖")
check_py("numpy", max_major=2, note="必须 <2，否则 cv2 ABI 冲突")
check_py("yaml", label="PyYAML")
check_py("ruckig", note="在线轨迹生成")
check_py("cv2", label="opencv", note="视觉/IBVS")
check_py("PyQt5", note="控制 GUI")

header("GUI (apt python3-tk)")
check_py("tkinter", note="滑块/运镜 GUI")

header("仿真 / 强化学习 (按需)")
check_py("mujoco", required=False, note="跑 MuJoCo 仿真需要")
check_py("gymnasium", required=False, note="仅 robot_arm_rl")
check_py("stable_baselines3", label="stable_baselines3", required=False, note="仅 robot_arm_rl")
check_py("torch", required=False, note="仅 robot_arm_rl")

header("ROS 核心")
ros_distro = os.environ.get("ROS_DISTRO", "")
line("PASS" if ros_distro else "FAIL", GREEN if ros_distro else RED,
     "ROS_DISTRO", ros_distro or "未 source ROS!")
if not ros_distro:
    fails.append("ROS_DISTRO")
check_py("rclpy", note="来自系统 ROS")

header("ROS 系统包 (apt)")
for p in ["moveit_ros_move_group", "moveit_servo", "gazebo_ros",
          "controller_manager", "robot_state_publisher"]:
    check_ros_pkg(p)
check_py("cv_bridge", note="ROS 图像桥")
check_py("tf_transformations", note="TF 变换")

header("工作空间包 (需先 colcon build + source install)")
# 注：robot_gimbal_description 当前只是资源目录(无 package.xml)，不是 ROS 包，故不在此列。
for p in ["robot_arm_interfaces", "robot_arm_description", "robot_arm_driver",
          "robot_arm_node", "robot_arm_bringup",
          "robot_gimbal_interfaces", "robot_gimbal_driver",
          "robot_gimbal_node", "robot_gimbal_bringup"]:
    check_ros_pkg(p)

header("GPU 渲染 (可选，仅告警)")
if shutil.which("nvidia-smi"):
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name,temperature.gpu",
                              "--format=csv,noheader"], capture_output=True, text=True, timeout=10)
        txt = (out.stdout or out.stderr).strip()
        if "ERR" in txt.upper() or out.returncode != 0:
            line("WARN", YEL, "nvidia-smi", f"GPU 异常: {txt[:60]} → 重启可恢复"); warns.append("GPU")
        else:
            line("PASS", GREEN, "nvidia-smi", txt.splitlines()[0][:60])
    except Exception as e:
        line("WARN", YEL, "nvidia-smi", f"调用失败: {e}"); warns.append("GPU")
    # NVIDIA PRIME offload 渲染器
    try:
        if shutil.which("glxinfo"):
            env = dict(os.environ, __NV_PRIME_RENDER_OFFLOAD="1", __GLX_VENDOR_LIBRARY_NAME="nvidia")
            g = subprocess.run(["glxinfo"], capture_output=True, text=True, timeout=15, env=env)
            rend = next((l.split(":",1)[1].strip() for l in g.stdout.splitlines()
                         if "OpenGL renderer" in l), "?")
            ok = "nvidia" in rend.lower() or "rtx" in rend.lower() or "geforce" in rend.lower()
            line("PASS" if ok else "WARN", GREEN if ok else YEL, "NVIDIA offload",
                 rend[:55] if ok else f"{rend[:40]} (offload 不通→软件渲染)")
            if not ok: warns.append("offload")
        else:
            line("SKIP", GREY, "NVIDIA offload", "无 glxinfo (mesa-utils 未装)")
    except Exception as e:
        line("WARN", YEL, "NVIDIA offload", f"检测失败: {e}")
else:
    line("SKIP", GREY, "nvidia-smi", "无 NVIDIA 驱动 / 纯 CPU 设备")

print(f"\n{CYAN}========== 汇总 =========={RST}")
if fails:
    print(f"  {RED}✗ 环境不达标：{len(fails)} 项必需检查失败 → {', '.join(fails)}{RST}")
    if warns:
        print(f"  {YEL}  另有 {len(warns)} 项告警: {', '.join(warns)}{RST}")
    print(f"  {GREY}  修复提示：缺 pip 依赖→重跑 setup_venv.sh；缺 ROS 包→apt 安装；缺工作空间包→colcon build 并 source install/setup.bash{RST}")
    sys.exit(1)
elif warns:
    print(f"  {GREEN}✓ 必需项全部通过{RST}，{YEL}{len(warns)} 项可选告警: {', '.join(warns)}{RST}")
    sys.exit(0)
else:
    print(f"  {GREEN}✓ 全部通过，环境就绪！{RST}")
    sys.exit(0)
'''


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '-c', ENV_CHECK],
            output='screen',
            name='env_check',
        ),
    ])
