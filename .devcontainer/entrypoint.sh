#!/bin/bash

# Exit immidiately if command fails
set -e

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
echo "✓ ROS2 Jazzy environment sourced"

# Source microROS workspace if it exists
if [ -f "/home/developer/microros_ws/install/local_setup.bash" ]; then
    source /home/developer/microros_ws/install/local_setup.bash
    echo "✓ MicroROS workspace sourced"
fi

# Add PlatformIO to PATH
export PATH=$PATH:$HOME/.local/bin
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.profile
echo "✓ PlatformIO added to PATH"

# Verify ROS2 is available
if command -v ros2 >/dev/null 2>&1; then
    echo "✓ ROS2 command available"
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
else
    echo "✗ ROS2 command not found"
fi

# Verify PlatformIO is available
if command -v pio >/dev/null 2>&1; then
    echo "✓ PlatformIO command available"
else
    echo "✗ PlatformIO command not found"
fi

echo "Provided arguments: $@"

# Execute the provided command argument
exec "$@"