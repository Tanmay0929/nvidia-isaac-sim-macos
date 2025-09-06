#!/bin/bash

# Stop Development Environment Script
# This script stops all development services

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
echo "Stopping Isaac Sim Development Environment"
echo "=========================================="

# Check if we're in the right directory
if [ ! -f "README.md" ] || [ ! -d "09-docker-setup" ]; then
    print_error "Please run this script from the nvidia-issac-sim root directory"
    exit 1
fi

# Stop Isaac Sim
print_status "Stopping Isaac Sim..."

if [ -f ".isaac_sim.pid" ]; then
    ISAAC_SIM_PID=$(cat .isaac_sim.pid)
    if ps -p $ISAAC_SIM_PID > /dev/null; then
        print_status "Stopping Isaac Sim (PID: $ISAAC_SIM_PID)..."
        kill $ISAAC_SIM_PID
        sleep 3
        
        # Force kill if still running
        if ps -p $ISAAC_SIM_PID > /dev/null; then
            print_warning "Force stopping Isaac Sim..."
            kill -9 $ISAAC_SIM_PID
        fi
        
        print_success "Isaac Sim stopped"
    else
        print_status "Isaac Sim was not running"
    fi
    rm -f .isaac_sim.pid
else
    print_status "No Isaac Sim PID file found"
fi

# Stop ROS2 containers
print_status "Stopping ROS2 containers..."
cd 09-docker-setup

if [ -f "docker-compose.yml" ]; then
    docker-compose down
    print_success "ROS2 containers stopped"
else
    print_warning "Docker compose file not found"
fi

# Go back to root directory
cd ..

# Check for any remaining Isaac Sim processes
print_status "Checking for remaining Isaac Sim processes..."
ISAAC_PROCESSES=$(ps aux | grep -i "isaac" | grep -v grep | wc -l)
if [ "$ISAAC_PROCESSES" -gt 0 ]; then
    print_warning "Found $ISAAC_PROCESSES Isaac Sim related processes still running"
    print_status "You may want to manually stop them if needed"
fi

# Check for any remaining Docker containers
print_status "Checking for remaining Docker containers..."
DOCKER_CONTAINERS=$(docker ps -q | wc -l)
if [ "$DOCKER_CONTAINERS" -gt 0 ]; then
    print_warning "Found $DOCKER_CONTAINERS Docker containers still running"
    print_status "You may want to stop them manually if needed"
fi

# Display final status
echo ""
echo "=========================================="
echo "Development Environment Stopped"
echo "=========================================="
echo "✅ Isaac Sim: Stopped"
echo "✅ ROS2 Containers: Stopped"
echo ""
echo "To start again:"
echo "./11-scripts/start-development.sh"
echo ""
echo "To clean up completely:"
echo "./11-scripts/clean-environment.sh"
echo "=========================================="

print_success "Development environment stopped successfully!"
