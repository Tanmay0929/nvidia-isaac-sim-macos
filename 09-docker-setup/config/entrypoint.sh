#!/bin/bash

# ROS2 Docker Entrypoint Script
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Set up environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_DISTRO=${ROS_DISTRO:-humble}

# Display ROS2 information
echo "=========================================="
echo "ROS2 Development Environment Ready!"
echo "=========================================="
echo "ROS Distribution: $ROS_DISTRO"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo "Workspace: /ros2_ws"
echo "Host Workspace: /host_workspace"
echo "=========================================="

# Execute the command
exec "$@"
