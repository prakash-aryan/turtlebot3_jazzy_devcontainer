#!/bin/bash
set -e

echo "=========================================="
echo "Post-start: Building TurtleBot3 workspace"
echo "=========================================="

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Navigate to workspace
cd /workspace/turtlebot3_ws

# Wait a moment for filesystem to sync (important for mounted volumes)
sleep 2

echo "Checking workspace structure..."
ls -la src/ 2>/dev/null || mkdir -p src

# Wait for source directory to be populated (retry up to 5 times)
MAX_RETRIES=5
RETRY_COUNT=0
while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    if [ -d "src/turtlebot3" ]; then
        echo "[OK] Found src/turtlebot3"
        break
    fi
    echo "Waiting for source files to sync... (attempt $((RETRY_COUNT + 1))/$MAX_RETRIES)"
    sleep 2
    RETRY_COUNT=$((RETRY_COUNT + 1))
done

if [ ! -d "src/turtlebot3" ]; then
    echo "[ERROR] Source files not found after waiting"
    echo "Current src/ contents:"
    ls -la src/ 2>/dev/null || echo "  src/ is empty"
    echo ""
    echo "Please run manually: bash .devcontainer/post-create.sh"
    exit 0  # Don't fail, just exit
fi

# Count packages
PACKAGE_COUNT=$(find src/ -type f -name "package.xml" 2>/dev/null | wc -l)
echo "Found $PACKAGE_COUNT ROS2 packages"
echo ""

if [ $PACKAGE_COUNT -eq 0 ]; then
    echo "[WARNING] No package.xml files found"
    echo "Run: bash .devcontainer/post-create.sh"
    exit 0
fi

# Check if already built and packages are available
if [ -f "install/setup.bash" ]; then
    echo "Sourcing existing build..."
    source install/setup.bash
    
    if ros2 pkg list | grep -q turtlebot3; then
        echo "[OK] Workspace already built and working"
        
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
        exit 0
    fi
fi

# Need to build
echo "Building workspace... (this takes 2-3 minutes)"
echo ""

# Clean old builds
rm -rf build install log

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "[OK] Build successful!"
    source install/setup.bash
    
    echo ""
    echo "TurtleBot3 packages:"
    ros2 pkg list | grep turtlebot3 || echo "  Warning: packages not detected"
    
    echo ""
    echo "=========================================="
    echo "[OK] TurtleBot3 Jazzy ready!"
    echo "=========================================="
    echo ""
    echo "Quick start:"
    echo "  tb3_empty  - Launch empty world"
    echo "  tb3_teleop - Keyboard control"
    echo "  VNC: http://localhost:6080 (password: ros)"
    echo ""
    echo "Useful commands:"
    echo "  cb - Rebuild workspace"
    echo "  sb - Source workspace"
    echo "=========================================="
else
    echo ""
    echo "[ERROR] Build failed!"
    echo "Try manually: cd /workspace/turtlebot3_ws && cb"
    exit 1
fi