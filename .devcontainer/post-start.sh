#!/bin/bash
set -e

echo "=========================================="
echo "Post-start: Building TurtleBot3 workspace"
echo "=========================================="

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Navigate to workspace
cd /workspace/turtlebot3_ws

# Check if src has packages
if [ -d "src/turtlebot3" ]; then
    echo "Building workspace..."
    
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        echo "✓ Build successful!"
        
        source install/setup.bash
        
        echo ""
        echo "TurtleBot3 packages:"
        ros2 pkg list | grep turtlebot3
        
        echo ""
        echo "=========================================="
        echo "✓ TurtleBot3 Jazzy ready!"
        echo "=========================================="
        echo ""
        echo "Quick start:"
        echo "  tb3_empty  - Launch empty world"
        echo "  tb3_teleop - Keyboard control"
        echo "  VNC: http://localhost:6080 (password: ros)"
        echo "=========================================="
    else
        echo "⚠ Build failed"
        exit 1
    fi
else
    echo "⚠ TurtleBot3 source not found"
fi