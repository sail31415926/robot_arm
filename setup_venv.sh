#!/usr/bin/env bash
# =============================================================================
# robot_arm 一键环境配置脚本
# -----------------------------------------------------------------------------
# 作用：给 robot_arm 包族准备 Python 运行环境 ——
#       ① 体检 apt 前置依赖（只检查、不自动 sudo 安装）
#       ② 创建/复用 --system-site-packages 的 venv（让 ROS 的 rclpy 等透进来）
#       ③ 装 requirements.txt 的核心依赖，可选组按开关装
#       ④ 校验关键 import 与 numpy<2 约束
#
# 用法：
#   bash setup_venv.sh                  # 只装核心依赖（默认，最快，够跑仿真/实机）
#   bash setup_venv.sh --with-mujoco    # 追加 MuJoCo 仿真后端
#   bash setup_venv.sh --with-rl        # 追加 RL 训练（gymnasium/sb3/torch，下载较大）
#   bash setup_venv.sh --with-analysis  # 追加 MATLAB 离线分析（scipy/matplotlib）
#   bash setup_venv.sh --all            # 三个可选组全装
#   bash setup_venv.sh --check          # 只体检现有环境，不安装任何东西
#   bash setup_venv.sh --skip-apt-check # 跳过 apt 前置检查（包名不一致时的逃生口）
#   bash setup_venv.sh --help
#
# 环境变量：
#   VENV_DIR=/path/to/venv     指定 venv 位置（默认 <工作区根>/.venv）
#   ROS_SETUP=/opt/ros/humble/setup.bash
#
# 依赖清单的唯一权威是 requirements.txt / requirements-optional.txt，
# 本脚本不硬编码任何版本号。
# =============================================================================
set -euo pipefail

# --- 开关默认值 ---
WANT_MUJOCO=0
WANT_RL=0
WANT_ANALYSIS=0
CHECK_ONLY=0
SKIP_APT_CHECK=0

usage() {
  # 打印文件头注释块（第 3 行到 "set -euo" 之前的分隔线），去掉行首 "# "
  sed -n '3,/^# =\{10,\}$/p' "${BASH_SOURCE[0]}" | sed '$d; s/^#\( \|$\)//'
  exit 0
}

while [ $# -gt 0 ]; do
  case "$1" in
    --with-mujoco)    WANT_MUJOCO=1 ;;
    --with-rl)        WANT_RL=1 ;;
    --with-analysis)  WANT_ANALYSIS=1 ;;
    --all)            WANT_MUJOCO=1; WANT_RL=1; WANT_ANALYSIS=1 ;;
    --check)          CHECK_ONLY=1 ;;
    --skip-apt-check) SKIP_APT_CHECK=1 ;;
    -h|--help)        usage ;;
    *) echo "!! 未知参数：$1（用 --help 看用法）" >&2; exit 2 ;;
  esac
  shift
done

# --- 路径解析 ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../src/E7009/robot_arm
REQ_CORE="${SCRIPT_DIR}/requirements.txt"
REQ_OPT="${SCRIPT_DIR}/requirements-optional.txt"
# 工作区根：robot_arm -> E7009 -> src -> <ws>
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

VENV_DIR="${VENV_DIR:-${WS_ROOT}/.venv}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

echo "==> robot_arm 环境配置"
echo "    工作区根 : ${WS_ROOT}"
echo "    venv     : ${VENV_DIR}"
echo "    核心依赖 : ${REQ_CORE}"
echo "    可选依赖 : ${REQ_OPT}"
echo "    ROS setup: ${ROS_SETUP}"
if [ "${CHECK_ONLY}" = 1 ]; then
  echo "    模式     : 只体检，不安装"
else
  _groups=""
  [ "${WANT_MUJOCO}"   = 1 ] && _groups="${_groups} mujoco"
  [ "${WANT_RL}"       = 1 ] && _groups="${_groups} rl"
  [ "${WANT_ANALYSIS}" = 1 ] && _groups="${_groups} analysis"
  echo "    安装组   : 核心${_groups:+ +}${_groups:-（可选组全部跳过，需要时加 --with-* / --all）}"
fi
echo ""

# --- 检查依赖文件 ---
for f in "${REQ_CORE}" "${REQ_OPT}"; do
  [ -f "$f" ] || { echo "!! 找不到依赖文件：$f" >&2; exit 1; }
done

# --- 检查 ROS ---
if [ ! -f "${ROS_SETUP}" ]; then
  echo "!! 找不到 ROS setup: ${ROS_SETUP}" >&2
  echo "   请先安装 ROS 2 Humble，或用 ROS_SETUP=... 指定路径。" >&2
  exit 1
fi
# ROS 的 setup.bash 会引用未绑定变量（AMENT_TRACE_SETUP_FILES 等），
# 在 `set -u` 下 source 会直接报错，故临时关掉 -u。
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
echo "==> ROS_DISTRO=${ROS_DISTRO:-未设置}"

# =============================================================================
# ① apt 前置依赖体检（只报告，不安装）
# =============================================================================
if [ "${SKIP_APT_CHECK}" = 1 ]; then
  echo "==> 跳过 apt 前置检查（--skip-apt-check）"
else
  echo "==> 体检 apt 前置依赖 ..."
  ROS_SHARE="$(dirname "${ROS_SETUP}")/share"
  MISSING_APT=()

  # ROS 包 → apt 包名（用 share 目录判断，比 `ros2 pkg prefix` 快得多）
  check_ros_pkg() {   # $1=ros包名  $2=apt包名
    if [ -d "${ROS_SHARE}/$1" ]; then
      printf '  [ok] %-28s (%s)\n' "$1" "$2"
    else
      printf '  [XX] %-28s 缺失 → apt: %s\n' "$1" "$2"
      MISSING_APT+=("$2")
    fi
  }
  check_ros_pkg moveit                 ros-humble-moveit
  check_ros_pkg pick_ik                ros-humble-pick-ik
  check_ros_pkg controller_manager     ros-humble-ros2-control
  check_ros_pkg joint_trajectory_controller ros-humble-ros2-controllers
  check_ros_pkg gazebo_ros             ros-humble-gazebo-ros-pkgs
  check_ros_pkg gazebo_ros2_control    ros-humble-gazebo-ros2-control
  check_ros_pkg cv_bridge              ros-humble-cv-bridge
  check_ros_pkg image_transport        ros-humble-image-transport
  check_ros_pkg pinocchio              ros-humble-pinocchio
  check_ros_pkg ruckig                 ros-humble-ruckig
  check_ros_pkg diagnostic_updater     ros-humble-diagnostic-updater
  check_ros_pkg xacro                  ros-humble-xacro
  check_ros_pkg robot_state_publisher  ros-humble-robot-state-publisher
  check_ros_pkg rviz2                  ros-humble-rviz2

  # 命令行工具（robot_arm_driver 里 vendored 的 lely CANopen 库编译要用）
  check_cmd() {       # $1=命令  $2=apt包名  $3=用途
    if command -v "$1" >/dev/null 2>&1; then
      printf '  [ok] %-28s (%s)\n' "$1" "$2"
    else
      printf '  [XX] %-28s 缺失 → apt: %-24s %s\n' "$1" "$2" "$3"
      MISSING_APT+=("$2")
    fi
  }
  check_cmd autoconf  autoconf  "编译 lely_core_libraries 用"
  check_cmd automake  automake  "编译 lely_core_libraries 用"
  check_cmd libtool   libtool   "编译 lely_core_libraries 用"

  # tkinter 只能由 apt 提供（pip 装不了），多个调试 GUI 依赖它
  if python3 -c "import tkinter" >/dev/null 2>&1; then
    printf '  [ok] %-28s (%s)\n' "tkinter" "python3-tk"
  else
    printf '  [XX] %-28s 缺失 → apt: %s\n' "tkinter" "python3-tk"
    MISSING_APT+=("python3-tk")
  fi

  # cv2 走 apt 的完整版（带 GUI）—— 不用 pip headless 版，理由见 requirements.txt
  if python3 -c "import cv2" >/dev/null 2>&1; then
    printf '  [ok] %-28s (%s)\n' "cv2" "python3-opencv"
  else
    printf '  [XX] %-28s 缺失 → apt: %s\n' "cv2" "python3-opencv"
    MISSING_APT+=("python3-opencv")
  fi

  # python3-venv：缺了下面建 venv 会以很难懂的方式失败，提前拦
  if ! python3 -c "import venv, ensurepip" >/dev/null 2>&1; then
    printf '  [XX] %-28s 缺失 → apt: %s\n' "python3-venv" "python3-venv"
    MISSING_APT+=("python3-venv")
  else
    printf '  [ok] %-28s (%s)\n' "python3-venv" "python3-venv"
  fi

  # 软提示：只做实机才需要，不算失败
  if command -v candump >/dev/null 2>&1; then
    printf '  [ok] %-28s (%s)\n' "candump" "can-utils"
  else
    printf '  [--] %-28s 未装（只影响实机调 CAN）→ apt: can-utils\n' "candump"
  fi

  if [ ${#MISSING_APT[@]} -gt 0 ]; then
    # 去重
    mapfile -t MISSING_APT < <(printf '%s\n' "${MISSING_APT[@]}" | sort -u)
    echo ""
    echo "!! 缺少 ${#MISSING_APT[@]} 个 apt 前置依赖，先装好再重跑本脚本：" >&2
    echo "" >&2
    echo "   sudo apt install -y ${MISSING_APT[*]}" >&2
    echo "" >&2
    echo "   （包名与你的系统不一致时可用 --skip-apt-check 跳过本检查）" >&2
    exit 1
  fi
  echo "    apt 前置依赖齐全。"
fi
echo ""

# =============================================================================
# ② 创建 / 复用 venv
# =============================================================================
if [ "${CHECK_ONLY}" = 1 ]; then
  if [ ! -f "${VENV_DIR}/pyvenv.cfg" ]; then
    echo "!! venv 不存在：${VENV_DIR}（去掉 --check 即可创建）" >&2
    exit 1
  fi
  echo "==> 复用现有 venv（--check 模式不改动它）"
else
  if [ ! -f "${VENV_DIR}/pyvenv.cfg" ]; then
    echo "==> 创建虚拟环境（--system-site-packages）..."
    python3 -m venv --system-site-packages "${VENV_DIR}"
  else
    echo "==> 已存在 venv，复用之"
  fi
fi

# --- 必须带 system-site-packages，否则 rclpy 等 ROS 包不可用 ---
if ! grep -q "include-system-site-packages = true" "${VENV_DIR}/pyvenv.cfg"; then
  echo "!! 现有 venv 未启用 system-site-packages，rclpy 等 ROS 包将不可用。" >&2
  echo "   建议删掉重建：rm -rf ${VENV_DIR} && bash ${BASH_SOURCE[0]}" >&2
  exit 1
fi

# venv 的 activate 会引用未设的 $PS1，同样要临时关掉 -u
set +u
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
set -u

# =============================================================================
# ③ 安装依赖
# =============================================================================
# 从 requirements-optional.txt 里抽取某个 "# [组名]" 分组下的包
extract_group() {   # $1=组名
  awk -v want="[$1]" '
    /^#[[:space:]]*\[/ { inseg = ($2 == want); next }
    inseg && NF && $0 !~ /^[[:space:]]*#/ { print $1 }
  ' "${REQ_OPT}"
}

install_group() {   # $1=组名
  local grp="$1" pkgs=()
  mapfile -t pkgs < <(extract_group "${grp}")
  if [ ${#pkgs[@]} -eq 0 ]; then
    echo "!! requirements-optional.txt 里没有 [${grp}] 分组或分组为空" >&2
    return 1
  fi
  echo "==> 安装可选组 [${grp}]：${pkgs[*]}"

  # torch 必须**先**装、且走 PyTorch 官方 CPU 源。
  # 为什么这个顺序是关键：stable-baselines3 依赖 torch，若让它自己去默认源解析，
  # pip 会拉 CUDA 版并连带 cuda-toolkit + 20 多个 nvidia-* 包（好几 GB）。
  # 先把 CPU 版 torch 装好，后面 sb3 看到 torch 已满足就不会再动它。
  local rest=() t
  for t in "${pkgs[@]}"; do
    case "$t" in
      torch==*|torch)
        echo "    torch 走 CPU 轮子源：$t"
        if ! pip install --index-url https://download.pytorch.org/whl/cpu "$t"; then
          echo "" >&2
          echo "!! 从 PyTorch CPU 源安装 $t 失败。" >&2
          echo "   这里**故意不回退默认源** —— 默认源会装 CUDA 版并连带几 GB 的" >&2
          echo "   cuda-toolkit / nvidia-* 依赖，对本工程（CPU 推理）纯属浪费。" >&2
          echo "   处理：检查网络/代理后重试；确实要 GPU 版就自行手动装：" >&2
          echo "     pip install $t" >&2
          return 1
        fi
        ;;
      *) rest+=("$t") ;;
    esac
  done
  if [ ${#rest[@]} -gt 0 ]; then pip install "${rest[@]}"; fi
  return 0
}

if [ "${CHECK_ONLY}" = 1 ]; then
  echo "==> 跳过安装（--check）"
else
  echo "==> 升级 pip ..."
  python -m pip install --upgrade pip

  echo "==> 安装核心依赖 ..."
  pip install -r "${REQ_CORE}"

  # 用显式 if（而非 `[ ] && cmd`）：后者作为分支最后一条语句时，
  # 条件不成立会让整个分支返回非 0，读代码的人容易误判会被 set -e 杀掉。
  if [ "${WANT_MUJOCO}"   = 1 ]; then install_group mujoco;   fi
  if [ "${WANT_RL}"       = 1 ]; then install_group rl;       fi
  if [ "${WANT_ANALYSIS}" = 1 ]; then install_group analysis; fi
fi
echo ""

# =============================================================================
# ④ 校验
# =============================================================================
echo "==> 校验环境 ..."
REQ_CORE="${REQ_CORE}" REQ_OPT="${REQ_OPT}" VENV_DIR="${VENV_DIR}" python - <<'PY'
import importlib, os, re, sys, warnings

warnings.filterwarnings("ignore")   # 版本不匹配的警告下面会显式报，不必刷屏

hard_fail = False
warn_count = 0
VENV = os.path.realpath(os.environ["VENV_DIR"])

# pip 发行名 → import 名（其余同名）
DIST2MOD = {
    "PyYAML": "yaml",
    "opencv-python-headless": "cv2",
    "stable-baselines3": "stable_baselines3",
}

def parse_pins(path):
    """从 requirements 文件解析 {import名: 期望版本}，只认 '==' 的硬 pin。"""
    pins = {}
    if not path or not os.path.isfile(path):
        return pins
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        m = re.match(r"^([A-Za-z0-9._-]+)==([A-Za-z0-9._+]+)$", line)
        if m:
            dist, ver = m.group(1), m.group(2)
            pins[DIST2MOD.get(dist, dist.replace("-", "_"))] = (dist, ver)
    return pins

PINS = {}
PINS.update(parse_pins(os.environ.get("REQ_CORE")))
PINS.update(parse_pins(os.environ.get("REQ_OPT")))

def version_of(m):
    v = getattr(m, "__version__", None)
    if v:
        return str(v)
    if m.__name__ == "PyQt5":          # PyQt5 没有 __version__
        try:
            from PyQt5.QtCore import PYQT_VERSION_STR
            return PYQT_VERSION_STR
        except Exception:
            return "?"
    return "?"

def origin_of(m):
    """判断模块来自 venv 还是系统 site-packages —— 版本漂移多半是被系统包遮挡。"""
    f = getattr(m, "__file__", None) or ""
    if not f:
        return "内置"
    f = os.path.realpath(f)
    if f.startswith(VENV):
        return "venv"
    if "/opt/ros/" in f:                      # 必须先判 ROS：它也在 dist-packages 下
        return "ROS"
    if "/dist-packages/" in f or f.startswith("/usr/lib/python"):
        return "系统"
    return "其他"

def check(mod, *, required, note=""):
    """required=True 缺失即硬失败；False 只提示。版本与 pin 不符时告警（不失败）。

    pin 比对规则：
      - 必需包：无论来自 venv 还是系统，都严格比 pin（不符即 [!!]）。
      - 可选包：只有当它确实被 pip 装进了本 venv 时才比 pin。若来自系统而
        对应的可选组根本没装，那是正常状态，只做提示、不算漂移 —— 否则
        每台机器都会因为系统自带 scipy/matplotlib 而无谓报警。
    """
    global hard_fail, warn_count
    try:
        m = importlib.import_module(mod)
    except Exception as e:
        if required:
            hard_fail = True
            print(f"  [XX] {mod:<20} {'':<14} import 失败（必需）: {e}")
        else:
            print(f"  [--] {mod:<20} {'':<14} {'':<6} 未安装（可选）{note}")
        return None

    ver, src = version_of(m), origin_of(m)
    pin = PINS.get(mod)
    flag, tail = "ok", note
    compare = bool(pin) and ver != "?" and (required or src == "venv")
    if compare and ver.split("+")[0] != pin[1]:
        flag = "!!"
        tail = f"← requirements 要求 {pin[0]}=={pin[1]}，实际是 {src} 里的 {ver}"
        warn_count += 1
    elif pin and not compare:
        # 可选组没装、用的是系统版：说明状态即可
        tail = f"{note}（本组未安装，这是系统自带版）"
    print(f"  [{flag}] {mod:<20} {ver:<14} {src:<6} {tail}")
    return ver

print("  --- 必需 ---")
nv = check("numpy",   required=True)
check("yaml",         required=True, note="(PyYAML)")
check("ruckig",       required=True)
cv = check("cv2",     required=True, note="(apt python3-opencv；不用 pip headless 版)")
check("PyQt5",        required=True)
check("tkinter",      required=True, note="(apt python3-tk)")
check("rclpy",        required=True, note="(来自系统 ROS，验证 system-site-packages 生效)")

# cv2 必须是带 GUI 的构建：red_box_detector 默认 show_window:=true 会调 imshow，
# headless 构建下会抛 "The function is not implemented"。
if cv:
    try:
        import cv2, re
        seg = re.search(r"  GUI:.*?\n\n", cv2.getBuildInformation(), re.S)
        seg = seg.group(0) if seg else ""
        if not any(k in seg.upper() for k in ("GTK", "QT", "COCOA", "WIN32")):
            warn_count += 1
            print("  [!!] cv2 是 headless 构建（无 GUI）——red_box_detector 的")
            print("       show_window:=true 会崩。装 apt python3-opencv，或 pip 装")
            print("       完整版 opencv-python==4.11.0.86（勿用 -headless / 勿升 4.12+）。")
    except Exception:
        pass   # 检测本身失败不影响主流程

print("  --- 可选 ---")
check("mujoco",              required=False, note="→ --with-mujoco")
check("gymnasium",           required=False, note="→ --with-rl")
check("stable_baselines3",   required=False, note="→ --with-rl")
check("torch",               required=False, note="→ --with-rl")
check("scipy",               required=False, note="→ --with-analysis")
check("matplotlib",          required=False, note="→ --with-analysis")

# numpy<2 硬约束：>=2 会与 cv2 等发生 ABI 冲突
if nv and nv != "?":
    if int(nv.split(".")[0]) >= 2:
        print(f"  [XX] numpy 版本 {nv} >= 2，违反约束（必须 <2）")
        hard_fail = True

if warn_count:
    print("")
    print(f"  ⚠ {warn_count} 处与 requirements 不一致（上面 [!!] 标出）。")
    print("    常见原因：系统 apt 版遮挡了 pip 版，或 venv 里的安装残缺。")
    print("    修复：核心组 → pip install --force-reinstall -r requirements.txt")
    print("          可选组 → bash setup_venv.sh --with-mujoco | --with-rl | --with-analysis")
    print("    （可选组没装就用不到那个功能，[--] 是正常的，不必理）")

sys.exit(1 if hard_fail else 0)
PY

# =============================================================================
# 收尾
# =============================================================================
echo ""
if [ "${CHECK_ONLY}" = 1 ]; then
  echo "==> 体检通过。"
else
  echo "==> 配置完成。"
fi
echo ""
echo "    每个新终端按顺序执行（顺序不能反）："
echo "      source ${ROS_SETUP}"
echo "      source ${VENV_DIR}/bin/activate"
echo "      source ${WS_ROOT}/install/setup.bash     # 编译后才有"
echo ""
if [ "${CHECK_ONLY}" != 1 ] \
   && [ "${WANT_MUJOCO}" = 0 ] && [ "${WANT_RL}" = 0 ] && [ "${WANT_ANALYSIS}" = 0 ]; then
  echo "    本次只装了核心依赖（够跑 Gazebo 仿真 / 实机 / 调试 GUI）。"
  echo "    需要 MuJoCo / RL 训练 / MATLAB 离线分析时再追加："
  echo "      bash ${BASH_SOURCE[0]} --with-mujoco | --with-rl | --with-analysis | --all"
fi
