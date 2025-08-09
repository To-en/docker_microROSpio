#!/bin/bash

# Exit immidiately if command fails
set -e

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
echo "✓ ROS2 Jazzy environment sourced"

# Verify ROS2 is available
if command -v ros2 >/dev/null 2>&1; then
    echo "✓ ROS2 command available"
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
else
    echo "✗ ROS2 command not found"
fi

# Export PlatformIO to PATH variable
export PATH=$PATH:$HOME/.local/bin
echo "✓ PlatformIO added to PATH"

# Verify PlatformIO is available
if command -v pio >/dev/null 2>&1; then
    echo "✓ PlatformIO command available"
else
    echo "✗ PlatformIO command not found"
fi

# clone microROS setup if it doesn't exists
if [ -f "/home/developer/microros_ws/src/micro_ros_setup" ]; then
    cd ~/microros_ws/src
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup
    cd ~
fi

# // Thes rest of the installtion is up to the user

echo "Provided arguments: $@"
# Execute the provided command argument
# ---- Script using argument

exec "$@"


