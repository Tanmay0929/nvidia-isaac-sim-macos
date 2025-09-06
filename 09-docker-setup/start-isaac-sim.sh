#!/bin/bash

# Start Isaac Sim with ROS2 Integration
set -e

echo "=========================================="
echo "Starting Isaac Sim with ROS2 Integration"
echo "=========================================="

# Check if Isaac Sim is installed
ISAAC_SIM_PATH="/Applications/NVIDIA-Omniverse/Isaac-Sim"

if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "❌ Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "Please install Isaac Sim from NVIDIA Omniverse first."
    echo "Download from: https://www.nvidia.com/en-us/omniverse/"
    exit 1
fi

# Check if ROS2 container is running
if ! docker ps | grep -q "isaac-sim-ros2-dev"; then
    echo "❌ ROS2 container is not running."
    echo "Please start it first with: ./start-ros2-container.sh"
    exit 1
fi

# Set environment variables for Isaac Sim
export ROS_DOMAIN_ID=0
export ROS_DISTRO=humble

echo "🚀 Starting Isaac Sim..."
echo "ROS2 container is running and ready for connection."
echo ""

# Start Isaac Sim
cd "$ISAAC_SIM_PATH"
./isaac-sim.sh --ext-folder /Users/tpancholi/dev/nvidia-issac-sim/08-ros2-integration/exts

echo ""
echo "✅ Isaac Sim started with ROS2 integration!"
echo ""
echo "Next steps:"
echo "1. In Isaac Sim, go to Window → Extensions"
echo "2. Enable the ROS2 Bridge extension"
echo "3. Configure the bridge to connect to localhost"
echo "4. Start your simulation!"
