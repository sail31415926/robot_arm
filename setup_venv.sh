#!/usr/bin/env bash
# =============================================================================
# robot_arm 一键虚拟环境配置脚本
# -----------------------------------------------------------------------------
# 作用：为 robot_arm 包族创建/配置 Python 虚拟环境，安装 requirements.txt 中的
#       纯 pip 依赖。ROS 2 的包通过 --system-site-packages 从系统 ROS 注入。
#
# 用法：
#   bash src/E7009/robot_arm/setup_venv.sh              # 默认在工作区根目录用 .venv
#   VENV_DIR=/path/to/venv bash .../setup_venv.sh       # 指定 venv 路径
#   ROS_SETUP=/opt/ros/humble/setup.bash bash .../setup_venv.sh
#
# 前置要求（apt，需提前装好，脚本不自动 sudo 安装）：
#   ROS 2 Humble、python3-venv、python3-tk、ros-humble-moveit、
#   ros-humble-tf-transformations、ros-humble-ros2-control(controllers)、
#   ros-humble-gazebo-ros(2-control)、ros-humble-cv-bridge
# =============================================================================
set -euo pipefail

# --- 路径解析 ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../src/E7009/robot_arm
REQ_FILE="${SCRIPT_DIR}/requirements.txt"
# 工作区根：robot_arm -> E7009 -> src -> <ws>
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

VENV_DIR="${VENV_DIR:-${WS_ROOT}/.venv}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

echo "==> robot_arm venv 配置"
echo "    工作区根 : ${WS_ROOT}"
echo "    venv     : ${VENV_DIR}"
echo "    依赖文件 : ${REQ_FILE}"
echo "    ROS setup: ${ROS_SETUP}"

# --- 检查 ROS ---
if [ ! -f "${ROS_SETUP}" ]; then
  echo "!! 找不到 ROS setup: ${ROS_SETUP}" >&2
  echo "   请先安装 ROS 2 Humble，或用 ROS_SETUP=... 指定路径。" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"
echo "    ROS_DISTRO=${ROS_DISTRO:-未设置}"

# --- 创建 venv（必须带 --system-site-packages 以复用 ROS 的 rclpy 等）---
if [ ! -f "${VENV_DIR}/pyvenv.cfg" ]; then
  echo "==> 创建虚拟环境（--system-site-packages）..."
  python3 -m venv --system-site-packages "${VENV_DIR}"
else
  echo "==> 已存在 venv，复用之"
  if ! grep -q "include-system-site-packages = true" "${VENV_DIR}/pyvenv.cfg"; then
    echo "!! 警告：现有 venv 未启用 system-site-packages，rclpy 等 ROS 包可能不可用。" >&2
  fi
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

# --- 安装依赖 ---
echo "==> 升级 pip ..."
python -m pip install --upgrade pip

echo "==> 安装 robot_arm pip 依赖 ..."
# torch 默认装 CPU 版（与已验证环境一致）；若需 GPU，注释掉下一行并改用对应 CUDA 轮子。
pip install --index-url https://download.pytorch.org/whl/cpu "torch==2.12.0" || \
  pip install "torch==2.12.0"
pip install -r "${REQ_FILE}"

# --- 校验关键约束 ---
echo "==> 校验环境 ..."
python - <<'PY'
import importlib, sys
ok = True
def check(mod, attr_ver="__version__", note=""):
    global ok
    try:
        m = importlib.import_module(mod)
        v = getattr(m, attr_ver, "?")
        print(f"  [ok] {mod:<18} {v}  {note}")
        return v
    except Exception as e:
        ok = False
        print(f"  [XX] {mod:<18} import 失败: {e}")
        return None

nv = check("numpy")
check("scipy"); check("pinocchio", note="(pin)"); check("ruckig")
check("mujoco"); check("pymeshlab"); check("cv2")
check("gymnasium"); check("stable_baselines3"); check("torch")
# ROS 注入校验
check("rclpy", attr_ver="__name__", note="(来自系统 ROS)")

# numpy<2 约束
if nv and int(str(nv).split(".")[0]) >= 2:
    print(f"  [XX] numpy 版本 {nv} >= 2，违反约束（必须 <2）"); ok = False

sys.exit(0 if ok else 1)
PY

echo ""
echo "==> 完成。后续使用："
echo "    source ${ROS_SETUP}"
echo "    source ${VENV_DIR}/bin/activate"
echo "    # 别忘了 source ${WS_ROOT}/install/setup.bash 才能用编译出的包"
