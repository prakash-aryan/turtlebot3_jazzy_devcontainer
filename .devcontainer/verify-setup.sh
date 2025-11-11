#!/bin/bash

# Quick verification script for TurtleBot3 Jazzy setup
# Run this after container is built to verify everything works

echo "=========================================="
echo "TurtleBot3 Jazzy Setup Verification"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test function
test_command() {
    local description=$1
    local command=$2
    
    echo -n "Testing: $description... "
    if eval $command > /dev/null 2>&1; then
        echo -e "${GREEN}âœ“ PASS${NC}"
        return 0
    else
        echo -e "${RED}âœ— FAIL${NC}"
        return 1
    fi
}

# Counter
passed=0
failed=0

# Source ROS2
source /opt/ros/jazzy/setup.bash 2>/dev/null
if [ -f /workspace/turtlebot3_ws/install/setup.bash ]; then
    source /workspace/turtlebot3_ws/install/setup.bash 2>/dev/null
fi

echo "1. System Checks"
echo "----------------------------------------"

if test_command "ROS2 Jazzy installed" "ros2 --version"; then
    ((passed++))
else
    ((failed++))
fi

if test_command "Gazebo Harmonic installed" "gz sim --version"; then
    ((passed++))
else
    ((failed++))
fi

if test_command "Python 3 available" "python3 --version"; then
    ((passed++))
else
    ((failed++))
fi

if test_command "colcon build tool" "colcon --version"; then
    ((passed++))
else
    ((failed++))
fi

echo ""
echo "2. ROS2 Package Checks"
echo "----------------------------------------"

if test_command "TurtleBot3 packages" "ros2 pkg list | grep turtlebot3"; then
    ((passed++))
else
    ((failed++))
fi

if test_command "Navigation2 packages" "ros2 pkg list | grep nav2"; then
    ((passed++))
else
    ((failed++))
fi

if test_command "Cartographer packages" "ros2 pkg list | grep cartographer"; then
    ((passed++))
else
    ((failed++))
fi

echo ""
echo "3. Environment Variables"
echo "----------------------------------------"

echo -n "Checking ROS_DISTRO... "
if [ "$ROS_DISTRO" = "jazzy" ]; then
    echo -e "${GREEN}âœ“ PASS${NC} (jazzy)"
    ((passed++))
else
    echo -e "${RED}âœ— FAIL${NC} (found: $ROS_DISTRO)"
    ((failed++))
fi

echo -n "Checking TURTLEBOT3_MODEL... "
if [ ! -z "$TURTLEBOT3_MODEL" ]; then
    echo -e "${GREEN}âœ“ PASS${NC} ($TURTLEBOT3_MODEL)"
    ((passed++))
else
    echo -e "${YELLOW}âš  WARNING${NC} (not set, using default)"
fi

echo -n "Checking GZ_VERSION... "
if [ "$GZ_VERSION" = "harmonic" ]; then
    echo -e "${GREEN}âœ“ PASS${NC} (harmonic)"
    ((passed++))
else
    echo -e "${YELLOW}âš  WARNING${NC} (found: $GZ_VERSION)"
fi

echo ""
echo "4. Directory Structure"
echo "----------------------------------------"

echo -n "Checking workspace... "
if [ -d "/workspace/turtlebot3_ws" ]; then
    echo -e "${GREEN}âœ“ PASS${NC}"
    ((passed++))
else
    echo -e "${RED}âœ— FAIL${NC}"
    ((failed++))
fi

echo -n "Checking source directory... "
if [ -d "/workspace/turtlebot3_ws/src" ]; then
    echo -e "${GREEN}âœ“ PASS${NC}"
    ((passed++))
else
    echo -e "${RED}âœ— FAIL${NC}"
    ((failed++))
fi

echo -n "Checking TurtleBot3 source... "
if [ -d "/workspace/turtlebot3_ws/src/turtlebot3" ]; then
    echo -e "${GREEN}âœ“ PASS${NC}"
    ((passed++))
else
    echo -e "${RED}âœ— FAIL${NC}"
    ((failed++))
fi

echo ""
echo "5. Build Status"
echo "----------------------------------------"

echo -n "Checking build directory... "
if [ -d "/workspace/turtlebot3_ws/build" ]; then
    echo -e "${GREEN}âœ“ PASS${NC}"
    ((passed++))
else
    echo -e "${YELLOW}âš  WARNING${NC} (not built yet)"
fi

echo -n "Checking install directory... "
if [ -d "/workspace/turtlebot3_ws/install" ]; then
    echo -e "${GREEN}âœ“ PASS${NC}"
    ((passed++))
else
    echo -e "${YELLOW}âš  WARNING${NC} (not built yet)"
fi

echo ""
echo "=========================================="
echo "Verification Results"
echo "=========================================="
echo -e "Tests Passed: ${GREEN}$passed${NC}"
if [ $failed -gt 0 ]; then
    echo -e "Tests Failed: ${RED}$failed${NC}"
else
    echo -e "Tests Failed: $failed"
fi

echo ""

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}âœ“ All critical checks passed!${NC}"
    echo ""
    echo "Your TurtleBot3 Jazzy environment is ready!"
    echo ""
    echo "Quick start:"
    echo "  1. Terminal 1: tb3_empty"
    echo "  2. Terminal 2: tb3_teleop"
    echo "  3. Access VNC at http://localhost:6080"
    echo ""
else
    echo -e "${RED}âœ— Some checks failed${NC}"
    echo ""
    echo "Please review the failed tests above."
    echo "Try rebuilding the workspace with: cb"
    echo ""
fi

echo "=========================================="