#!/bin/bash

# Start Development Environment Script
# This script starts the complete development environment for Isaac Sim

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

echo "=========================================="
echo "Starting Isaac Sim Development Environment"
echo "=========================================="

# Check if we're in the right directory
if [ ! -f "README.md" ] || [ ! -d "09-docker-setup" ]; then
    print_error "Please run this script from the nvidia-issac-sim root directory"
    exit 1
fi

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker Desktop first."
    exit 1
fi

print_success "Docker is running"

# Check if Isaac Sim is installed
ISAAC_SIM_PATH="/Applications/NVIDIA-Omniverse/Isaac-Sim"
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    print_warning "Isaac Sim not found at $ISAAC_SIM_PATH"
    print_status "Please install Isaac Sim from NVIDIA Omniverse first."
    echo "Download from: https://www.nvidia.com/en-us/omniverse/"
    echo ""
    read -p "Press Enter to continue after installing Isaac Sim..."
fi

# Start ROS2 container
print_status "Starting ROS2 development container..."
cd 09-docker-setup

if [ ! -f "start-ros2-container.sh" ]; then
    print_error "ROS2 setup scripts not found. Please run setup-environment.sh first."
    exit 1
fi

./start-ros2-container.sh

# Wait for container to be ready
print_status "Waiting for ROS2 container to be ready..."
sleep 5

# Check if container is running
if docker ps | grep -q "isaac-sim-ros2-dev"; then
    print_success "ROS2 container is running"
else
    print_error "Failed to start ROS2 container"
    exit 1
fi

# Go back to root directory
cd ..

# Start Isaac Sim
print_status "Starting Isaac Sim..."
cd 09-docker-setup

if [ ! -f "start-isaac-sim.sh" ]; then
    print_error "Isaac Sim startup script not found."
    exit 1
fi

# Start Isaac Sim in background
./start-isaac-sim.sh &
ISAAC_SIM_PID=$!

# Go back to root directory
cd ..

# Wait a moment for Isaac Sim to start
print_status "Waiting for Isaac Sim to start..."
sleep 10

# Check if Isaac Sim is running
if ps -p $ISAAC_SIM_PID > /dev/null; then
    print_success "Isaac Sim is starting (PID: $ISAAC_SIM_PID)"
else
    print_warning "Isaac Sim may not have started properly"
fi

# Display status
echo ""
echo "=========================================="
echo "Development Environment Status"
echo "=========================================="
echo "✅ ROS2 Container: Running"
echo "✅ Isaac Sim: Starting"
echo ""
echo "Container Information:"
echo "- Name: isaac-sim-ros2-dev"
echo "- Network: host mode"
echo "- ROS Domain ID: 0"
echo ""
echo "Useful Commands:"
echo "- Access ROS2 container: docker exec -it isaac-sim-ros2-dev bash"
echo "- View ROS2 topics: docker exec -it isaac-sim-ros2-dev ros2 topic list"
echo "- View ROS2 nodes: docker exec -it isaac-sim-ros2-dev ros2 node list"
echo "- Stop environment: ./11-scripts/stop-development.sh"
echo ""
echo "Next Steps:"
echo "1. Wait for Isaac Sim to fully load"
echo "2. Enable ROS2 Bridge extension in Isaac Sim"
echo "3. Start your simulation!"
echo "=========================================="

# Save PID for later use
echo $ISAAC_SIM_PID > .isaac_sim.pid
print_success "Isaac Sim PID saved to .isaac_sim.pid"

print_success "Development environment started successfully!"
