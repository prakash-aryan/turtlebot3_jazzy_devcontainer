#!/bin/bash
set -e

echo "=========================================="
echo "Post-start: Building TurtleBot3 workspace"
echo "=========================================="

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Navigate to workspace
cd /workspace/turtlebot3_ws

# Wait for filesystem
sleep 2

# Verify packages exist
PACKAGE_COUNT=$(find src/ -name "package.xml" 2>/dev/null | wc -l)

if [ "$PACKAGE_COUNT" -eq 0 ]; then
    # Try ls method for Windows mounts
    PACKAGE_COUNT=$(ls src/*/package.xml 2>/dev/null | wc -l)
fi

echo "Found $PACKAGE_COUNT ROS2 packages"

if [ "$PACKAGE_COUNT" -eq 0 ]; then
    echo "[ERROR] No packages found!"
    echo "Something went wrong with post-create.sh"
    echo "The container may not work correctly."
    exit 1
fi

echo ""

# Check if already built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    
    if ros2 pkg list | grep -q turtlebot3; then
        echo "[OK] Workspace already built"
        
        echo ""
        echo "=========================================="
        echo "[OK] TurtleBot3 Jazzy ready!"
        echo "=========================================="
        echo ""
        echo "Quick start:"
        echo "  tb3_empty  - Launch empty world"
        echo "  tb3_teleop - Keyboard control"
        echo "  VNC: http://localhost:6080 (password: ros)"
        echo "=========================================="
        exit 0
    fi
fi

# Build workspace
echo "Building workspace... (first time: 2-3 minutes)"
echo ""

rm -rf build install log

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo ""
    echo "[ERROR] Build failed!"
    exit 1
fi

echo ""
echo "[OK] Build successful!"
source install/setup.bash

echo ""
echo "TurtleBot3 packages:"
ros2 pkg list | grep turtlebot3

echo ""
echo "=========================================="
echo "[OK] TurtleBot3 Jazzy ready!"
echo "=========================================="
echo ""
echo "Quick start:"
echo "  tb3_empty  - Launch empty world"
echo "  tb3_teleop - Keyboard control"
echo "  VNC: http://localhost:6080 (password: ros)"
echo "=========================================="