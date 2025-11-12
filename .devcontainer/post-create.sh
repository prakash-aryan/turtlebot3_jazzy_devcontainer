#!/bin/bash
set -e

echo "=========================================="
echo "Post-create: Setting up TurtleBot3 Jazzy"
echo "=========================================="

# Detect GPU capabilities
echo ""
echo "Detecting graphics capabilities..."
if command -v glxinfo &> /dev/null; then
    if glxinfo | grep -q "direct rendering: Yes"; then
        echo "[OK] Hardware GPU acceleration available"
        USE_SOFTWARE_RENDERING=false
    else
        echo "[WARNING] No hardware GPU acceleration detected"
        echo "[INFO] Will use software rendering for stability"
        USE_SOFTWARE_RENDERING=true
    fi
else
    echo "[WARNING] Cannot detect GPU - will use software rendering"
    USE_SOFTWARE_RENDERING=true
fi

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Setup bashrc for current user
cat >> ~/.bashrc << 'BASHRC_EOF'

# ROS2 Jazzy setup
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/turtlebot3_ws/install/setup.bash ]; then
    source /workspace/turtlebot3_ws/install/setup.bash
fi

# Environment variables
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_VERSION=harmonic
export QT_QPA_PLATFORM=xcb

BASHRC_EOF

# Add software rendering fallback if needed
if [ "$USE_SOFTWARE_RENDERING" = true ]; then
    cat >> ~/.bashrc << 'BASHRC_EOF'
# Software rendering (for GPU compatibility)
export LIBGL_ALWAYS_SOFTWARE=1
echo "[INFO] Using software rendering for Gazebo (GPU not available)"
BASHRC_EOF
fi

# Add aliases
cat >> ~/.bashrc << 'BASHRC_EOF'

# Useful aliases
alias cb='cd /workspace/turtlebot3_ws && colcon build --symlink-install'
alias sb='source /workspace/turtlebot3_ws/install/setup.bash'
alias tb3_empty='ros2 launch turtlebot3_gazebo empty_world.launch.py'
alias tb3_world='ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'
alias tb3_house='ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py'
alias tb3_teleop='ros2 run turtlebot3_teleop teleop_keyboard'
alias tb3_slam='ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True'
alias tb3_nav='ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/my_map.yaml'

# Gazebo with software rendering fallback
alias tb3_empty_sw='LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo empty_world.launch.py'
alias tb3_world_sw='LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'
alias tb3_house_sw='LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py'
BASHRC_EOF

# Fix workspace permissions
sudo chown -R $(whoami):$(whoami) /workspace/turtlebot3_ws

# Create src directory if it doesn't exist
mkdir -p /workspace/turtlebot3_ws/src

# Navigate to workspace src
cd /workspace/turtlebot3_ws/src

# Check if repositories exist AND have content (not empty directories)
NEED_CLONE=false

if [ ! -d "turtlebot3" ]; then
    NEED_CLONE=true
    echo "TurtleBot3 directory not found"
elif [ ! -f "turtlebot3/turtlebot3/package.xml" ]; then
    NEED_CLONE=true
    echo "TurtleBot3 directory exists but is empty - will re-clone"
    rm -rf DynamixelSDK turtlebot3 turtlebot3_msgs turtlebot3_simulations
fi

if [ "$NEED_CLONE" = true ]; then
    echo "Cloning TurtleBot3 repositories..."
    
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    
    echo "[OK] Repositories cloned"
else
    echo "[OK] TurtleBot3 repositories already exist with content"
fi

# Verify we have package.xml files
cd /workspace/turtlebot3_ws
PACKAGE_COUNT=$(find src/ -name "package.xml" 2>/dev/null | wc -l)
echo "Verified $PACKAGE_COUNT package.xml files"

# Back to workspace root
cd /workspace/turtlebot3_ws

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init || true
fi

# Update rosdep
echo "Updating rosdep..."
rosdep update

# Install dependencies
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y || true

# Create maps directory
mkdir -p ~/maps

# Create a helpful GPU info file
cat > ~/GPU_INFO.txt << 'EOF'
GPU Detection Results
=====================

EOF

if command -v glxinfo &> /dev/null; then
    echo "GPU Vendor: $(glxinfo | grep "OpenGL vendor" || echo "Unknown")" >> ~/GPU_INFO.txt
    echo "GPU Renderer: $(glxinfo | grep "OpenGL renderer" || echo "Unknown")" >> ~/GPU_INFO.txt
    echo "Direct Rendering: $(glxinfo | grep "direct rendering" || echo "Unknown")" >> ~/GPU_INFO.txt
else
    echo "glxinfo not available" >> ~/GPU_INFO.txt
fi

echo "" >> ~/GPU_INFO.txt
echo "If Gazebo crashes, try: tb3_empty_sw (software rendering)" >> ~/GPU_INFO.txt

echo "=========================================="
echo "[OK] Post-create setup complete!"
if [ "$USE_SOFTWARE_RENDERING" = true ]; then
    echo "[INFO] Software rendering enabled (see ~/GPU_INFO.txt)"
fi
echo "=========================================="