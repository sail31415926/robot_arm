#!/bin/bash
# EMEET Integrated Camera Driver - Build Script

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="$( cd "$SCRIPT_DIR/../../.." &> /dev/null && pwd )"

cd "$WS_DIR"

# 自动加载 ROS 2 环境 (如果当前环境未加载)
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "Sourcing ROS 2 Humble..."
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
        exit 1
    fi
fi

echo "Building EMEET Integrated Camera Driver..."

# Safety check: ensure workspace has ROS packages
if [ ! -d "src" ]; then
    echo "Error: workspace src/ directory not found. Are you in the correct workspace?"
    exit 1
fi

PKG_XML_COUNT="$(find src -maxdepth 2 -name package.xml 2>/dev/null | wc -l | tr -d ' ')"
if [ "$PKG_XML_COUNT" = "0" ]; then
    echo "Error: no ROS packages found under src/ (no package.xml)."
    echo "Do NOT delete your workspace directory while a terminal is inside it."
    exit 1
fi

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "Build successful."
    echo "Source environment: source install/setup.bash"
else
    echo "Build failed."
    exit 1
fi
