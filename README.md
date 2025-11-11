# TurtleBot3 ROS2 Jazzy Development Container

Complete ROS2 Jazzy development environment with TurtleBot3 Burger simulation in Gazebo Harmonic, packaged as a VS Code Dev Container for cross-platform robotics development.

---

## Table of Contents

- [Overview](#overview)
- [How It Works](#how-it-works)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
  - [Quick Launch Aliases](#quick-launch-aliases)
  - [Common Workflows](#common-workflows)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Development Guide](#development-guide)
- [Troubleshooting](#troubleshooting)
  - [Docker Issues](#docker-issues)
  - [Container Build Issues](#container-build-issues)
  - [Port and Network Issues](#port-and-network-issues)
  - [Performance Issues](#performance-issues)
  - [VNC and Display Issues](#vnc-and-display-issues)
  - [Gazebo Issues](#gazebo-issues)
  - [ROS2 Issues](#ros2-issues)
  - [Build and Compilation Issues](#build-and-compilation-issues)
- [Learning Resources](#learning-resources)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project provides a complete, ready-to-use ROS2 Jazzy development environment running inside a Docker container. No need to install ROS2, Gazebo, or manage dependencies on your host system everything runs in an isolated, reproducible environment that works identically on Windows, macOS, and Linux.

### Features

- **ROS2 Jazzy** (Ubuntu 24.04 Noble)
- **Gazebo Harmonic** simulator with GPU support
- **TurtleBot3 Burger** robot packages
- **Navigation2** and **SLAM** (Cartographer)
- **noVNC Desktop** for browser-based GUI access
- **Pre-configured development environment** with VS Code extensions
- **Platform-agnostic**: Works on Windows, macOS, and Linux

---

## How It Works

### Architecture Overview
<img src="architecture.png">

### Container Startup Sequence
<img src="startupsequence.png">

### Technology Stack

**Docker Containers**: Lightweight, portable environments that package your entire development setup (OS, ROS2, Gazebo, dependencies) into a reproducible unit that runs identically everywhere.

**VS Code Dev Containers**: Develop inside a Docker container seamlessly your code editor, terminal, and debugger work as if running locally.

**ROS2 (Robot Operating System)**: A flexible framework for robot software providing libraries and tools for robot application development.

**Gazebo**: A 3D robot simulator with accurate physics, sensors, and actuator simulation.

**noVNC**: Browser-based VNC client for accessing a full Linux desktop through your web browser without installing additional software.

### Why Use Containers?

1. **Consistency**: Everyone gets the exact same environment
2. **Isolation**: No conflicts with other software on your system
3. **Portability**: Works on Windows, Mac, and Linux
4. **Clean**: Delete the container your system remains unchanged
5. **Version Control**: The entire environment is defined in code

### Data Persistence

Your workspace folder is "mounted" into the container:
- Files you create/edit are saved on your real computer
- Deleting the container doesn't delete your code
- Files are accessible from both inside and outside the container

---

## Prerequisites

### Required Software

- **Docker**:
  - Windows/Mac: [Docker Desktop](https://www.docker.com/products/docker-desktop/)
  - Linux: [Docker Engine](https://docs.docker.com/engine/install/)
- **VS Code**: [Download here](https://code.visualstudio.com/)
- **Dev Containers Extension**: Install from VS Code Extensions (Ctrl+Shift+X, search "Dev Containers")
<img src="extension.png">
### System Requirements

- **Minimum**: 4 CPU cores, 8GB RAM, 20GB free disk space
- **Recommended**: 6+ CPU cores, 16GB RAM, 50GB free disk space
- **Operating Systems**:
  - Windows 10/11 with Docker Desktop
  - macOS 10.15 or later
  - Any modern Linux distribution with Docker installed

---

## Installation

### Step 1: Install Docker

#### Windows/macOS
1. Download and install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
2. Use default settings during installation
3. Restart your computer after installation
4. Start Docker Desktop from your applications menu
5. Verify installation:
   ```bash
   docker --version
   docker run hello-world
   ```

#### Linux (Ubuntu/Debian example)
```bash
# Install Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in, then verify
docker --version
docker run hello-world
```

For other Linux distributions, follow the [official Docker installation guide](https://docs.docker.com/engine/install/).

### Step 2: Install VS Code

1. Download VS Code from [code.visualstudio.com](https://code.visualstudio.com/)
2. Install with default settings
3. Launch VS Code

### Step 3: Install Dev Containers Extension

1. In VS Code, click the Extensions icon (Ctrl+Shift+X)
2. Search for "Dev Containers"
3. Install the "Dev Containers" extension by Microsoft
4. Reload VS Code if prompted

### Step 4: Get the Project

**Option A: Clone with Git**
```bash
git clone https://github.com/prakash-aryan/turtlebot3_jazzy_devcontainer.git
cd turtlebot3_jazzy_devcontainer
```

**Option B: Download ZIP**
1. Download the project files
2. Extract to your preferred location
3. Navigate to the extracted folder

---

## Quick Start

### First-Time Setup (15 minutes)

1. **Open the project in VS Code:**
   ```bash
   cd turtlebot3_jazzy_devcontainer
   code .
   ```

2. **Reopen in Container:**
   - VS Code will detect the dev container configuration
   - Click "Reopen in Container" when prompted
   - Or press `F1` → "Dev Containers: Reopen in Container"
   <img src="reopen.png">

3. **Wait for the build** (10-15 minutes first time):
   - Downloads Ubuntu 24.04 base image (~1GB)
   - Installs ROS2 Jazzy packages (~2GB)
   - Installs Gazebo Harmonic (~500MB)
   - Clones TurtleBot3 repositories
   - Builds ROS2 workspace

4. **Verify the setup:**
   ```bash
   bash verify-setup.sh
   ```
   You should see green checkmarks for all tests.

5. **Launch your first simulation:**
   ```bash
   # Terminal 1: Start Gazebo with TurtleBot3
   tb3_empty
   
   # Terminal 2: Control the robot
   tb3_teleop
   ```

6. **Access the GUI:**
   - Open browser: http://localhost:6080
   - Password: `ros`
   - You'll see the full desktop with Gazebo running!

### Control the Robot

In the teleop terminal, use these keys:
- `i` - Move forward
- `,` - Move backward
- `j` - Rotate left
- `l` - Rotate right
- `k` - Stop

---

## Usage

### Quick Launch Aliases

Pre-configured shortcuts are available:

| Alias | Description |
|-------|-------------|
| `cb` | Build workspace with colcon |
| `sb` | Source workspace environment |
| `tb3_empty` | Launch empty world |
| `tb3_world` | Launch TurtleBot3 world (obstacles) |
| `tb3_house` | Launch house environment |
| `tb3_teleop` | Keyboard teleoperation control |
| `tb3_slam` | Start SLAM with Cartographer |
| `tb3_nav` | Start Navigation2 |

### Manual Commands

**Launch Gazebo Worlds:**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Teleoperation:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**SLAM (Mapping):**
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

**Navigation:**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/my_map.yaml
```

### Common Workflows

#### 1. Basic Simulation and Control

**Terminal 1 - Start simulation:**
```bash
tb3_empty
```

**Terminal 2 - Control robot:**
```bash
tb3_teleop
# Use keyboard: i=forward, j=left, l=right, ,=backward, k=stop
```

#### 2. SLAM (Create a Map)

**Terminal 1 - Launch simulation:**
```bash
tb3_world  # or tb3_house for more complex environment
```

**Terminal 2 - Start SLAM:**
```bash
tb3_slam
# RViz2 opens automatically - you'll see the map being built in real-time
```

**Terminal 3 - Drive robot to explore:**
```bash
tb3_teleop
# Drive slowly to map the entire environment
```

**Save your map:**
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
# Creates my_map.pgm (image) and my_map.yaml (metadata)
```

#### 3. Autonomous Navigation

**Terminal 1 - Launch simulation:**
```bash
tb3_world
```

**Terminal 2 - Start navigation with your saved map:**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/maps/my_map.yaml
```

**In RViz2 (access via VNC at http://localhost:6080):**
1. Click "2D Pose Estimate" button in the toolbar
2. Click on the map where the robot is located
3. Drag to set the robot's orientation
4. Click "Navigation2 Goal" button
5. Click on the destination you want the robot to reach
6. Watch the robot autonomously navigate to the goal!

#### 4. Exploring Different Worlds

```bash
# Simple empty world - good for testing basic movement
tb3_empty

# TurtleBot3 world - includes obstacles for navigation testing
tb3_world

# House environment - complex indoor space for advanced testing
tb3_house
```

---

## Project Structure

### Directory Layout

```
turtlebot3_jazzy_devcontainer/
├── .devcontainer/              # Dev container configuration
│   ├── Dockerfile              # Container image definition
│   ├── devcontainer.json       # VS Code settings
│   ├── post-create.sh          # Initial setup script (runs once)
│   └── post-start.sh           # Build script (runs on start)
├── src/                        # ROS2 source packages (auto-created)
│   ├── DynamixelSDK/          # Motor control SDK
│   ├── turtlebot3/            # Core TurtleBot3 packages
│   ├── turtlebot3_msgs/       # Message definitions
│   └── turtlebot3_simulations/# Gazebo simulation packages
├── build/                      # Build artifacts (auto-created)
├── install/                    # Installed packages (auto-created)
├── log/                        # Build logs (auto-created)
├── .gitignore                  # Git ignore rules
├── README.md                   # This file
└── verify-setup.sh             # Setup verification script
```

### Configuration Files

#### .devcontainer/Dockerfile
Defines the container image with:
- Base: `osrf/ros:jazzy-desktop-full-noble` (Ubuntu 24.04)
- ROS2 Jazzy complete installation
- Gazebo Harmonic simulator
- Development tools and dependencies
- OpenGL/Mesa utilities for GPU support

#### .devcontainer/devcontainer.json
Configures VS Code integration:
- Workspace settings
- VS Code extensions to install
- Port forwarding (6080 for noVNC, 5901 for VNC)
- Environment variables
- Lifecycle scripts

#### .devcontainer/post-create.sh
Runs once when container is first created:
- Sources ROS2 environment
- Configures bash with ROS2 setup
- Adds useful aliases
- Clones TurtleBot3 repositories
- Installs dependencies

#### .devcontainer/post-start.sh
Runs every time container starts:
- Builds ROS2 workspace with colcon
- Verifies build success
- Displays quick start instructions

### ROS2 Workspace Structure

```
/workspace/turtlebot3_ws/
├── src/           # Source code (you work here)
├── build/         # Build artifacts (generated, don't edit)
├── install/       # Installed binaries (generated)
└── log/           # Build logs (generated)
```

### Environment Variables

Automatically set in the container:

```bash
ROS_DISTRO=jazzy                    # ROS2 version
ROS_DOMAIN_ID=30                    # Network isolation
TURTLEBOT3_MODEL=burger             # Robot model
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # DDS implementation
GZ_VERSION=harmonic                 # Gazebo version
DISPLAY=:1                          # VNC display
QT_QPA_PLATFORM=xcb                 # Qt platform
```

### Installed Components

**ROS2 Packages:**
- `ros-jazzy-desktop-full` - Complete ROS2 desktop
- `ros-jazzy-navigation2` - Navigation stack
- `ros-jazzy-cartographer` - SLAM
- `ros-jazzy-ros-gz` - ROS-Gazebo bridge
- `ros-jazzy-rviz2` - 3D visualization
- `ros-jazzy-rqt` - Qt-based GUI tools

**Simulation:**
- Gazebo Harmonic
- TurtleBot3 models (Burger, Waffle, Waffle Pi)
- Simulation worlds (Empty, TurtleBot3 World, House)

**Development Tools:**
- Python 3.12
- colcon build system
- vcstool
- Git, Vim, cURL, Wget

---

## Configuration

### Adjusting Container Resources

If the container runs slowly, allocate more resources:

**Docker Desktop (Windows/Mac):**
1. Open Docker Desktop
2. Settings → Resources
3. Adjust:
   - CPUs: 4-6 cores recommended
   - Memory: 8-16 GB recommended
   - Disk: 50 GB recommended
4. Apply & Restart

**Linux:**
Resources are not limited by default on Linux.

### Changing Robot Model

Edit `.devcontainer/devcontainer.json`:
```json
"remoteEnv": {
  "TURTLEBOT3_MODEL": "waffle"  // Options: burger, waffle, waffle_pi
}
```
Rebuild container after changes: F1 → "Dev Containers: Rebuild Container"

### Changing VNC Port

If port 6080 is already in use, edit `.devcontainer/devcontainer.json`:
```json
"forwardPorts": [6081, 5901],  // Changed from 6080 to 6081
"portsAttributes": {
  "6081": {  // Update this too
    "label": "Desktop (noVNC)",
    "onAutoForward": "openBrowser"
  }
}
```
Then access: http://localhost:6081

### Modifying the Environment

Edit `post-create.sh` to add:
- Custom aliases
- Additional environment variables
- Extra software installations
- Configuration scripts

After modifications, rebuild: F1 → "Dev Containers: Rebuild Container"

---

## Development Guide

### Building the Workspace

```bash
# Build all packages
cd /workspace/turtlebot3_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Or use aliases
cb  # build
sb  # source
```

### Creating Custom Packages

```bash
# Navigate to workspace source directory
cd /workspace/turtlebot3_ws/src

# Create a new C++ package
ros2 pkg create --build-type ament_cmake my_package

# Or create a Python package
ros2 pkg create --build-type ament_python my_package

# Build the new package
cd /workspace/turtlebot3_ws
colcon build --packages-select my_package

# Source and use
source install/setup.bash
```

### Adding Existing Packages

```bash
cd /workspace/turtlebot3_ws/src

# Clone the package repository
git clone <package-repository-url>

# Build
cd ..
colcon build --symlink-install

# Source
source install/setup.bash
```

### Debugging

```bash
# List all topics
ros2 topic list

# Echo a specific topic
ros2 topic echo /scan

# List all nodes
ros2 node list

# Get node info
ros2 node info /turtlebot3_node

# Check topic frequency
ros2 topic hz /odom

# Visualize data in RViz (in VNC desktop)
rviz2
```

### Build System (colcon)

```bash
# Build all packages
colcon build

# Build with symlink install (faster rebuilds)
colcon build --symlink-install

# Build specific package
colcon build --packages-select turtlebot3_gazebo

# Build up to a package (including dependencies)
colcon build --packages-up-to turtlebot3_gazebo

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build
rm -rf build install log
colcon build --symlink-install
```

### Version Control Best Practices

**Do commit:**
- Source code in `src/`
- Configuration files
- Custom launch files
- Documentation

**Don't commit:**
- `build/` directory
- `install/` directory
- `log/` directory
- `.vscode/` workspace files (already in .gitignore)
- Temporary files

---

## Troubleshooting

### Docker Issues

#### Docker Desktop Not Starting

**Symptoms:**
- Docker daemon is not running
- Container won't start

**Solutions:**

1. **Enable Virtualization in BIOS:**
   - Restart computer and enter BIOS (F2, F10, or Del key)
   - Enable "Virtualization Technology" (Intel VT-x or AMD-V)
   - Save and exit

2. **Restart Docker Service:**
   
   Windows (PowerShell as Administrator):
   ```powershell
   net stop com.docker.service
   net start com.docker.service
   ```
   
   macOS/Linux:
   ```bash
   # Restart Docker Desktop application
   # Or restart docker daemon on Linux
   sudo systemctl restart docker
   ```

3. **Reset Docker Desktop:**
   - Right-click Docker Desktop system tray icon
   - Troubleshoot → Reset to factory defaults
   - Restart Docker

#### Docker Permission Denied (Linux)

**Symptoms:**
- "permission denied while trying to connect to Docker daemon"

**Solution:**
```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in, or:
newgrp docker

# Verify
docker run hello-world
```

#### Docker Out of Memory

**Symptoms:**
- Container build fails partway through
- "no space left on device" errors

**Solutions:**

1. **Increase Docker Resources** (Docker Desktop):
   - Settings → Resources
   - Memory: 8GB minimum, 16GB recommended
   - CPUs: 4-6 cores
   - Disk: 50GB+ recommended

2. **Clean Up Docker:**
   ```bash
   # Remove unused images
   docker image prune -a
   
   # Remove unused volumes
   docker volume prune
   
   # Remove everything unused
   docker system prune -a --volumes
   ```

### Container Build Issues

#### Container Build Fails

**Symptoms:**
- Build stops with errors
- Network timeout errors

**Solutions:**

1. **Check Internet Connection:**
   ```bash
   ping google.com
   ping packages.osrfoundation.org
   ```

2. **Rebuild Without Cache:**
   - F1 → "Dev Containers: Rebuild Container Without Cache"

3. **Check Docker Disk Space:**
   ```bash
   docker system df
   # Clean if needed
   docker system prune -a
   ```

#### Post-Create Script Fails

**Symptoms:**
- Container builds but setup incomplete
- TurtleBot3 packages not found

**Solution:**
```bash
# Manually run setup
cd /workspace/turtlebot3_ws
bash .devcontainer/post-create.sh

# Check script permissions
chmod +x .devcontainer/post-create.sh
chmod +x .devcontainer/post-start.sh

# Run with debugging
bash -x .devcontainer/post-create.sh
```

### Port and Network Issues

#### Port 6080 Already in Use

**Symptoms:**
- VNC web page won't load
- "Port already allocated" error

**Solutions:**

1. **Find What's Using the Port:**
   
   Windows (PowerShell):
   ```powershell
   netstat -ano | findstr :6080
   # Note the PID (last column)
   taskkill /PID <PID> /F
   ```
   
   macOS/Linux:
   ```bash
   lsof -i :6080
   kill <PID>
   ```

2. **Change VNC Port:**
   Edit `.devcontainer/devcontainer.json`:
   ```json
   "forwardPorts": [6081, 5901],
   ```
   Then access: http://localhost:6081

3. **Manually Forward Port in VS Code:**
   - View → Ports
   - Click "Forward a Port"
   - Enter 6080

#### Cannot Access VNC

**Symptoms:**
- http://localhost:6080 doesn't load
- Connection refused

**Solutions:**

1. **Check Port Forwarding:**
   - In VS Code, check "Ports" tab (View → Ports)
   - Port 6080 should be listed and forwarded

2. **Check noVNC Service:**
   ```bash
   # Inside container
   ps aux | grep vnc
   sudo supervisorctl status desktop-lite
   
   # Restart if needed
   sudo supervisorctl restart desktop-lite
   ```

3. **Try Direct VNC:**
   - Use a VNC client (TigerVNC, RealVNC)
   - Connect to: localhost:5901
   - Password: ros

### Performance Issues

#### Slow Performance

**Symptoms:**
- Container builds very slowly
- Gazebo lags severely
- High CPU/memory usage

**Solutions:**

1. **Allocate More Resources:**
   - Docker Desktop → Settings → Resources
   - CPUs: 4-6 cores
   - Memory: 8-16 GB

2. **Use Software Rendering (if GPU issues):**
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   tb3_empty
   ```

3. **Optimize Docker (Windows):**
   - Docker Desktop → Settings → General
   - Enable "Use WSL 2 based engine"
   - Restart Docker

4. **Optimize Docker (macOS):**
   - Docker Desktop → Settings → General
   - Enable "Use VirtioFS for file sharing"

5. **Clean Build Cache:**
   ```bash
   cd /workspace/turtlebot3_ws
   rm -rf build install log
   colcon build --symlink-install
   ```

### VNC and Display Issues

#### VNC Shows Black Screen

**Symptoms:**
- VNC loads but shows only black screen
- No desktop visible

**Solutions:**

1. **Wait for Desktop Initialization:**
   - Desktop can take 30-90 seconds to start
   - Refresh browser page

2. **Restart VNC Server:**
   ```bash
   sudo supervisorctl restart desktop-lite
   # Wait 30 seconds, then refresh browser
   ```

3. **Check Display Variable:**
   ```bash
   echo $DISPLAY
   # Should show: :1
   
   # If not:
   export DISPLAY=:1
   ```

#### VNC Password Not Working

**Solution:**
```bash
# Reset VNC password
rm -rf ~/.vnc/passwd
vncpasswd
# Enter: ros
# Verify: ros
# View-only: n

# Restart VNC
sudo supervisorctl restart desktop-lite
```

#### Display Resolution Wrong

**Solution:**
```bash
# Kill VNC
vncserver -kill :1

# Start with custom resolution
vncserver :1 -geometry 1920x1080 -depth 24

# Or for smaller screens
vncserver :1 -geometry 1280x720 -depth 24
```

### Gazebo Issues

#### Gazebo Won't Start

**Symptoms:**
- `tb3_empty` command does nothing
- Error messages in terminal

**Solutions:**

1. **Kill Existing Gazebo Processes:**
   ```bash
   pkill -9 gz
   pkill -9 gzserver
   pkill -9 gzclient
   tb3_empty
   ```

2. **Check GPU/Graphics:**
   ```bash
   # Test OpenGL
   glxinfo | grep "direct rendering"
   # Should say "Yes"
   
   # If "No", use software rendering
   export LIBGL_ALWAYS_SOFTWARE=1
   tb3_empty
   ```

3. **Check Model Environment:**
   ```bash
   echo $TURTLEBOT3_MODEL
   # Should output: burger
   
   # Set if empty
   export TURTLEBOT3_MODEL=burger
   ```

4. **Launch with Verbose Output:**
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py --verbose
   ```

#### No Robot Model in Gazebo

**Symptoms:**
- Gazebo opens but is empty
- No TurtleBot3 appears

**Solutions:**

1. **Verify Model Variable:**
   ```bash
   echo $TURTLEBOT3_MODEL
   # Must output: burger, waffle, or waffle_pi
   
   export TURTLEBOT3_MODEL=burger
   ```

2. **Check Package Installation:**
   ```bash
   ros2 pkg list | grep turtlebot3
   
   # If missing, rebuild
   cd /workspace/turtlebot3_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Source Workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /workspace/turtlebot3_ws/install/setup.bash
   # Or use: sb
   ```

#### Gazebo Crashes

**Solutions:**

1. **Use Software Rendering:**
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   tb3_empty
   ```

2. **Check Logs:**
   ```bash
   cat ~/.gz/gazebo/server.log
   cat ~/.gz/gazebo/client.log
   cat /workspace/turtlebot3_ws/log/latest*/events.log
   ```

3. **Clear Gazebo Cache:**
   ```bash
   rm -rf ~/.gz/
   tb3_empty
   ```

### ROS2 Issues

#### ROS2 Topics Not Visible

**Symptoms:**
- `ros2 topic list` shows nothing
- Nodes not communicating

**Solutions:**

1. **Check ROS_DOMAIN_ID:**
   ```bash
   echo $ROS_DOMAIN_ID
   # Should output: 30
   
   export ROS_DOMAIN_ID=30
   ```

2. **Source Workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /workspace/turtlebot3_ws/install/setup.bash
   # Or use: sb
   ```

3. **Check if Simulation is Running:**
   ```bash
   ros2 node list
   # Should see nodes if simulation is running
   ```

#### Package Not Found

**Symptoms:**
- "Package 'turtlebot3_XXX' not found"
- Cannot launch nodes

**Solutions:**

1. **Rebuild Workspace:**
   ```bash
   cd /workspace/turtlebot3_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Check Package Exists:**
   ```bash
   ros2 pkg list | grep turtlebot3
   ls /workspace/turtlebot3_ws/src
   ```

3. **Re-clone Repositories:**
   ```bash
   cd /workspace/turtlebot3_ws/src
   rm -rf turtlebot3*
   bash /workspace/turtlebot3_ws/.devcontainer/post-create.sh
   ```

### Build and Compilation Issues

#### Build Failures

**Symptoms:**
- `colcon build` fails with errors
- Compilation errors

**Solutions:**

1. **Clean and Rebuild:**
   ```bash
   cd /workspace/turtlebot3_ws
   rm -rf build install log
   colcon build --symlink-install
   ```

2. **Install Dependencies:**
   ```bash
   sudo apt update
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build with Verbose Output:**
   ```bash
   colcon build --symlink-install --event-handlers console_direct+
   ```

4. **Build One Package at a Time:**
   ```bash
   colcon build --packages-select turtlebot3_gazebo
   ```

#### Dependency Errors

**Solution:**
```bash
# Update rosdep
rosdep update

# Install all dependencies
cd /workspace/turtlebot3_ws
rosdep install --from-paths src --ignore-src -y -r

# Install specific package if needed
sudo apt install ros-jazzy-<package-name>
```

### Nuclear Option: Complete Reset

When nothing else works:

1. Close VS Code
2. Stop all containers:
   ```bash
   docker stop $(docker ps -aq)
   ```
3. Remove containers:
   ```bash
   docker rm $(docker ps -aq)
   ```
4. Remove volumes (optional - will delete data):
   ```bash
   docker volume prune
   ```
5. Restart Docker Desktop
6. Open project in VS Code
7. Rebuild container: F1 → "Dev Containers: Rebuild Container Without Cache"

---

## Learning Resources

### Official Documentation
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/) - Complete ROS2 reference
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) - Official TurtleBot3 guide
- [Gazebo Documentation](https://gazebosim.org/docs) - Simulator reference
- [Navigation2 Tutorials](https://docs.nav2.org/) - Autonomous navigation guide
- [Dev Containers Documentation](https://code.visualstudio.com/docs/devcontainers/containers) - VS Code dev containers

### Community Resources
- [ROS Discourse](https://discourse.ros.org/) - Q&A forum
- [ROS Answers](https://answers.ros.org/) - Technical questions
- [TurtleBot3 Issues](https://github.com/ROBOTIS-GIT/turtlebot3/issues) - Bug reports
- [Docker Forums](https://forums.docker.com/) - Docker help

---

## Contributing

Contributions are welcome! This project aims to provide an accessible, platform-agnostic ROS2 development environment.

### How to Contribute

1. **Report Issues**: Found a bug? [Open an issue](https://github.com/prakash-aryan/turtlebot3_jazzy_devcontainer/issues)
2. **Suggest Features**: Have an idea? Create a feature request
3. **Submit Pull Requests**: 
   - Fork the repository
   - Create a feature branch
   - Make your changes
   - Submit a pull request with a clear description

### Guidelines

- Maintain platform-agnostic design (Windows, macOS, Linux)
- Test changes in the dev container environment
- Update documentation for new features
- Follow ROS2 naming conventions
- Keep Docker images efficient

### Maintainer

**Prakash Aryan**
- GitHub: [@prakash-aryan](https://github.com/prakash-aryan)
- Email: prakash.aryan@unibe.ch

---

## License

This project is provided as-is for educational and research purposes.

The project integrates several open-source components:
- ROS2 Jazzy (Apache 2.0 License)
- Gazebo (Apache 2.0 License)
- TurtleBot3 (Apache 2.0 License)

See individual component licenses for details.

---

## Acknowledgments

- **ROBOTIS** for creating and maintaining TurtleBot3
- **Open Robotics** for ROS2 and Gazebo development
- **Open Source Robotics Foundation** for supporting the robotics community
- **Microsoft** for the VS Code Dev Containers extension

---



---

**Version**: 1.0 (ROS2 Jazzy)  
**Last Updated**: November 2024  
**Maintained by**: Prakash Aryan (prakash.aryan@unibe.ch)