#!/bin/bash
set -e

echo "=========================================="
echo "Post-create: Setting up TurtleBot3 Jazzy"
echo "=========================================="

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Setup bashrc for current user
cat >> ~/.bashrc << 'EOF'

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

# Useful aliases
alias cb='cd /workspace/turtlebot3_ws && colcon build --symlink-install'
alias sb='source /workspace/turtlebot3_ws/install/setup.bash'
alias tb3_empty='ros2 launch turtlebot3_gazebo empty_world.launch.py'
alias tb3_world='ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'
alias tb3_house='ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py'
alias tb3_teleop='ros2 run turtlebot3_teleop teleop_keyboard'
alias tb3_slam='ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True'
alias tb3_nav='ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/my_map.yaml'
EOF

# Fix workspace permissions
sudo chown -R $(whoami):$(whoami) /workspace/turtlebot3_ws

# Create src directory if it doesn't exist
mkdir -p /workspace/turtlebot3_ws/src

# Navigate to workspace src
cd /workspace/turtlebot3_ws/src

# Clone TurtleBot3 repositories if not exist
if [ ! -d "turtlebot3" ]; then
    echo "Cloning TurtleBot3 repositories..."
    
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    
    echo "âœ“ Repositories cloned"
else
    echo "âœ“ TurtleBot3 repositories already exist"
fi

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

echo "=========================================="
echo "âœ“ Post-create setup complete!"
echo "=========================================="