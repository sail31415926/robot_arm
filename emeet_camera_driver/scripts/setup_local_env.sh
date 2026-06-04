#!/bin/bash
# EMEET Integrated Camera Driver - Smart Local Environment Setup
# This script ensures the local .venv is healthy and contains all necessary packages.

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="$( cd "$SCRIPT_DIR/../../../../.." &> /dev/null && pwd )"

echo -e "${GREEN}>>> Starting Smart Environment Setup in: $WS_DIR${NC}"

# 1. System Dependencies Check
echo -e "${YELLOW}[1/5] Checking system dependencies...${NC}"
# Qt GUI (PyQt5) runtime deps for xcb platform plugin are included to avoid
# "Could not load the Qt platform plugin xcb" on desktop/X11 environments.
SYSTEM_DEPS=(
    "libudev-dev"
    "libhidapi-dev"
    "pkg-config"
    "python3-venv"
    "libxcb-xinerama0"
    "libxcb-cursor0"
    "libxkbcommon-x11-0"
    "libx11-xcb1"
    "libxcb-icccm4"
    "libxcb-image0"
    "libxcb-keysyms1"
    "libxcb-render-util0"
    "libxcb-xfixes0"
)
MISSING_DEPS=()

for dep in "${SYSTEM_DEPS[@]}"; do
    if ! dpkg -l | grep -q "^ii  $dep "; then
        MISSING_DEPS+=("$dep")
    fi
done

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${YELLOW}Missing system packages: ${MISSING_DEPS[*]}${NC}"
    echo -e "${YELLOW}Attempting to install missing system dependencies (requires sudo)...${NC}"
    sudo apt-get update && sudo apt-get install -y "${MISSING_DEPS[@]}"
else
    echo -e "${GREEN}All system dependencies are present.${NC}"
fi

# 2. Virtual Environment Management
echo -e "${YELLOW}[2/5] Managing Python virtual environment...${NC}"
if [ ! -d "$WS_DIR/.venv" ]; then
    echo -e "${YELLOW}Creating new .venv with --system-site-packages...${NC}"
    python3 -m venv --system-site-packages "$WS_DIR/.venv"
else
    # Check if existing .venv has system-site-packages (crucial for ROS2)
    if [ ! -f "$WS_DIR/.venv/pyvenv.cfg" ] || ! grep -q "include-system-site-packages = true" "$WS_DIR/.venv/pyvenv.cfg"; then
        echo -e "${RED}Warning: Existing .venv does NOT include system-site-packages.${NC}"
        echo -e "${YELLOW}Re-creating .venv to ensure ROS2 compatibility...${NC}"
        rm -rf "$WS_DIR/.venv"
        python3 -m venv --system-site-packages "$WS_DIR/.venv"
    else
        echo -e "${GREEN}Existing .venv is valid.${NC}"
    fi
fi

# 3. Activate and Check Python Packages
source "$WS_DIR/.venv/bin/activate"
echo -e "${GREEN}Activated .venv: $(which python)${NC}"

echo -e "${YELLOW}[3/5] Checking Python packages...${NC}"
pip install --upgrade pip -q

# Function to check and fix packages
check_and_fix() {
    pkg_name=$1
    install_name=$2
    version_constraint=$3 # optional, e.g., ">=2.0"
    
    echo -n "Checking $pkg_name... "
    if pip show "$pkg_name" &> /dev/null; then
        current_version=$(pip show "$pkg_name" | grep Version | awk '{print $2}')
        echo -e "${GREEN}Found v$current_version${NC}"
        # For simplicity, we re-install to ensure constraints are met
        if [[ ! -z "$version_constraint" ]]; then
             pip install "$pkg_name$version_constraint" -q
        fi
    else
        echo -e "${YELLOW}Not found. Installing $install_name...${NC}"
        pip install "$install_name" -q
    fi
}

# Install torch with best effort CUDA support.
# - If NVIDIA GPU is detected, try CUDA wheels first.
# - Otherwise fall back to CPU wheels.
install_torch() {
    echo -e "${YELLOW}Checking torch (GPU-aware)...${NC}"
    if python3 -c "import torch" &>/dev/null; then
        python3 - <<'PY'
import torch
print(f"Found torch {torch.__version__}, cuda_available={torch.cuda.is_available()}")
PY
    else
        echo -e "${YELLOW}torch not found. Installing...${NC}"
    fi

    if command -v nvidia-smi &>/dev/null; then
        echo -e "${YELLOW}NVIDIA GPU detected (nvidia-smi present). Trying CUDA torch wheels...${NC}"
        # Prefer cu124 (common on Ubuntu 22.04 + CUDA 12.4), fall back to cu121, then CPU.
        python3 -m pip install -U --index-url https://download.pytorch.org/whl/cu124 torch torchvision torchaudio -q || \
        python3 -m pip install -U --index-url https://download.pytorch.org/whl/cu121 torch torchvision torchaudio -q || \
        python3 -m pip install -U --index-url https://download.pytorch.org/whl/cpu  torch torchvision torchaudio -q
    else
        echo -e "${YELLOW}No NVIDIA GPU detected. Installing CPU torch wheels...${NC}"
        python3 -m pip install -U --index-url https://download.pytorch.org/whl/cpu torch torchvision torchaudio -q
    fi

    python3 - <<'PY'
import torch
print(f"torch={torch.__version__}, cuda_available={torch.cuda.is_available()}, device_count={torch.cuda.device_count()}")
if torch.cuda.is_available() and torch.cuda.device_count() > 0:
    print("gpu0=", torch.cuda.get_device_name(0))
PY
}

# Specific check for conflicting opencv versions
# We need to uninstall from both the venv and the user's local site-packages
# to prevent Qt platform plugin conflicts.
echo -e "${YELLOW}Cleaning up conflicting OpenCV versions...${NC}"
# Uninstall from current environment
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless -q
# Force uninstall from user local directory to prevent pollution
python3 -m pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless -q 2>/dev/null
# Clean up potential leftover cv2/qt/plugins directories in .local
rm -rf ~/.local/lib/python3.10/site-packages/cv2/qt/plugins 2>/dev/null

# Check requirements
# NOTE: ROS 2 Humble cv_bridge requires NumPy <2.0 (ABI compatibility).
check_and_fix "numpy" "numpy<2.0" "<2.0"
check_and_fix "opencv-python-headless" "opencv-python-headless" ""
check_and_fix "hidapi" "hidapi" ""
check_and_fix "PyQt5" "PyQt5" ""
check_and_fix "ultralytics" "ultralytics" ""
install_torch

# 3.1 Check for YOLO models
echo -e "${YELLOW}[3.1/5] Checking YOLO models...${NC}"
MODEL_DIR="$WS_DIR/src/tof_perception/models"
DEFAULT_MODEL="$MODEL_DIR/yolo26s.pt"
if [ ! -f "$DEFAULT_MODEL" ]; then
    echo -e "${RED}Warning: Default model $DEFAULT_MODEL not found in $MODEL_DIR${NC}"
    echo -e "${YELLOW}Please ensure yolo26s.pt is placed in the src/tof_perception/models directory.${NC}"
else
    echo -e "${GREEN}Default model yolo26s.pt found.${NC}"
fi

# 4. Script Permissions Authorization
echo -e "${YELLOW}[4/5] Authorizing script permissions...${NC}"
find "$WS_DIR/src/E7009/robot_arm/emeet_camera_driver/test" -name "*.py" -exec chmod +x {} +
find "$WS_DIR/src/E7009/robot_arm/emeet_camera_driver/scripts" -name "*.sh" -exec chmod +x {} +
find "$WS_DIR/src/tof_perception/scripts" -type f -exec chmod +x {} +
echo -e "${GREEN}Permissions updated.${NC}"

# Normalize line endings for Linux execution (avoid /usr/bin/env: python3\r)
echo -e "${YELLOW}Normalizing line endings (CRLF -> LF)...${NC}"
find "$WS_DIR/src/E7009/robot_arm/emeet_camera_driver/test" -name "*.py" -exec sed -i 's/\r$//' {} +
find "$WS_DIR/src/E7009/robot_arm/emeet_camera_driver/scripts" -name "*.sh" -exec sed -i 's/\r$//' {} +
find "$WS_DIR/src/tof_perception/scripts" -type f -exec sed -i 's/\r$//' {} +
echo -e "${GREEN}Line endings normalized.${NC}"

# 5. Final Verification
echo -e "${YELLOW}[5/5] Final verification...${NC}"
python3 -c "import numpy; import cv2; import hid; from PyQt5 import QtWidgets; import ultralytics; import torch; print('✅ Python dependencies verified.')"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Environment setup/validation successful!${NC}"
    echo -e "To activate: ${YELLOW}source .venv/bin/activate${NC}"
else
    echo -e "${RED}Environment verification failed. Please check the logs above.${NC}"
    exit 1
fi
