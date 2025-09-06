#!/bin/bash

# Check Development Environment Status Script
# This script checks the status of all development services

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
echo "Isaac Sim Development Environment Status"
echo "=========================================="

# Check if we're in the right directory
if [ ! -f "README.md" ] || [ ! -d "09-docker-setup" ]; then
    print_error "Please run this script from the nvidia-issac-sim root directory"
    exit 1
fi

# Check system information
print_status "System Information:"
echo "OS: $(uname -s) $(uname -m)"
echo "Hostname: $(hostname)"
echo "User: $(whoami)"
echo "Date: $(date)"
echo ""

# Check Docker status
print_status "Docker Status:"
if command -v docker &> /dev/null; then
    if docker info > /dev/null 2>&1; then
        print_success "Docker is installed and running"
        echo "Version: $(docker --version)"
        echo "Docker Compose: $(docker-compose --version 2>/dev/null || echo 'Not installed')"
        
        # Check Docker resources
        DOCKER_MEMORY=$(docker system info --format '{{.MemTotal}}' 2>/dev/null || echo "0")
        DOCKER_MEMORY_GB=$((DOCKER_MEMORY / 1024 / 1024 / 1024))
        echo "Allocated Memory: ${DOCKER_MEMORY_GB}GB"
        
        # Check running containers
        RUNNING_CONTAINERS=$(docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep -v "NAMES" | wc -l)
        echo "Running Containers: $RUNNING_CONTAINERS"
        
        if [ "$RUNNING_CONTAINERS" -gt 0 ]; then
            echo "Container Details:"
            docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
        fi
    else
        print_error "Docker is installed but not running"
    fi
else
    print_error "Docker is not installed"
fi
echo ""

# Check Isaac Sim status
print_status "Isaac Sim Status:"
ISAAC_SIM_PATH="/Applications/NVIDIA-Omniverse/Isaac-Sim"
if [ -d "$ISAAC_SIM_PATH" ]; then
    print_success "Isaac Sim is installed at: $ISAAC_SIM_PATH"
    
    # Check if Isaac Sim is running
    if [ -f ".isaac_sim.pid" ]; then
        ISAAC_SIM_PID=$(cat .isaac_sim.pid)
        if ps -p $ISAAC_SIM_PID > /dev/null; then
            print_success "Isaac Sim is running (PID: $ISAAC_SIM_PID)"
        else
            print_warning "Isaac Sim PID file exists but process is not running"
        fi
    else
        print_status "Isaac Sim is not running (no PID file)"
    fi
    
    # Check for Isaac Sim processes
    ISAAC_PROCESSES=$(ps aux | grep -i "isaac" | grep -v grep | wc -l)
    if [ "$ISAAC_PROCESSES" -gt 0 ]; then
        echo "Isaac Sim related processes: $ISAAC_PROCESSES"
    fi
else
    print_error "Isaac Sim is not installed at expected location"
fi
echo ""

# Check ROS2 container status
print_status "ROS2 Container Status:"
if docker ps | grep -q "isaac-sim-ros2-dev"; then
    print_success "ROS2 development container is running"
    
    # Check container health
    CONTAINER_STATUS=$(docker inspect isaac-sim-ros2-dev --format='{{.State.Status}}')
    echo "Container Status: $CONTAINER_STATUS"
    
    # Check container resources
    CONTAINER_CPU=$(docker stats isaac-sim-ros2-dev --no-stream --format "{{.CPUPerc}}")
    CONTAINER_MEM=$(docker stats isaac-sim-ros2-dev --no-stream --format "{{.MemUsage}}")
    echo "CPU Usage: $CONTAINER_CPU"
    echo "Memory Usage: $CONTAINER_MEM"
    
    # Check ROS2 topics
    echo "ROS2 Topics:"
    docker exec isaac-sim-ros2-dev ros2 topic list 2>/dev/null | head -10 || echo "No topics available"
    
else
    print_warning "ROS2 development container is not running"
fi
echo ""

# Check Python environment
print_status "Python Environment:"
if command -v python3 &> /dev/null; then
    print_success "Python 3 is installed"
    echo "Version: $(python3 --version)"
    echo "Path: $(which python3)"
    
    # Check Python packages
    if python3 -c "import omni" 2>/dev/null; then
        print_success "Omniverse Python packages are available"
    else
        print_warning "Omniverse Python packages not available"
    fi
else
    print_error "Python 3 is not installed"
fi
echo ""

# Check Git status
print_status "Git Status:"
if command -v git &> /dev/null; then
    print_success "Git is installed"
    echo "Version: $(git --version)"
    
    # Check repository status
    if [ -d ".git" ]; then
        echo "Repository: $(git remote get-url origin 2>/dev/null || echo 'Local repository')"
        echo "Branch: $(git branch --show-current 2>/dev/null || echo 'Unknown')"
        echo "Last commit: $(git log -1 --format='%h - %s (%cr)' 2>/dev/null || echo 'No commits')"
    else
        print_warning "Not a Git repository"
    fi
else
    print_error "Git is not installed"
fi
echo ""

# Check system resources
print_status "System Resources:"
echo "Memory Usage:"
top -l 1 | grep -E "PhysMem|CPU usage" | head -2

echo "Disk Usage:"
df -h | grep -E "Filesystem|/$" | head -2

# Check network connectivity
print_status "Network Connectivity:"
if ping -c 1 google.com > /dev/null 2>&1; then
    print_success "Internet connection is available"
else
    print_warning "Internet connection may be limited"
fi

# Check ports
print_status "Port Usage:"
echo "Ports in use:"
lsof -i -P -n | grep LISTEN | head -5 || echo "No ports found"

echo ""
echo "=========================================="
echo "Status Summary"
echo "=========================================="

# Overall status
OVERALL_STATUS="HEALTHY"
if ! docker info > /dev/null 2>&1; then
    OVERALL_STATUS="DOCKER_ISSUE"
elif [ ! -d "$ISAAC_SIM_PATH" ]; then
    OVERALL_STATUS="ISAAC_SIM_ISSUE"
elif ! docker ps | grep -q "isaac-sim-ros2-dev"; then
    OVERALL_STATUS="ROS2_CONTAINER_ISSUE"
fi

case $OVERALL_STATUS in
    "HEALTHY")
        print_success "Overall Status: HEALTHY - All systems operational"
        ;;
    "DOCKER_ISSUE")
        print_error "Overall Status: DOCKER_ISSUE - Docker needs attention"
        ;;
    "ISAAC_SIM_ISSUE")
        print_error "Overall Status: ISAAC_SIM_ISSUE - Isaac Sim needs attention"
        ;;
    "ROS2_CONTAINER_ISSUE")
        print_warning "Overall Status: ROS2_CONTAINER_ISSUE - ROS2 container needs attention"
        ;;
esac

echo ""
echo "Quick Commands:"
echo "- Start environment: ./11-scripts/start-development.sh"
echo "- Stop environment: ./11-scripts/stop-development.sh"
echo "- Clean environment: ./11-scripts/clean-environment.sh"
echo "=========================================="
