#!/bin/bash

# Isaac Sim Environment Setup Script
# This script sets up the development environment for Isaac Sim learning

set -e

echo "=========================================="
echo "Isaac Sim Environment Setup"
echo "=========================================="

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

# Check if running on macOS
if [[ "$OSTYPE" != "darwin"* ]]; then
    print_error "This script is designed for macOS. Please use the appropriate setup for your system."
    exit 1
fi

# Check if running on Intel Mac
if [[ $(uname -m) != "x86_64" ]]; then
    print_warning "This script is optimized for Intel Macs. Apple Silicon Macs may need different configuration."
fi

print_status "Setting up Isaac Sim development environment..."

# Check prerequisites
print_status "Checking prerequisites..."

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed. Please install Docker Desktop first."
    echo "Download from: https://www.docker.com/products/docker-desktop/"
    exit 1
fi

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker Desktop first."
    exit 1
fi

print_success "Docker is installed and running"

# Check if Isaac Sim is installed
ISAAC_SIM_PATH="/Applications/NVIDIA-Omniverse/Isaac-Sim"
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    print_warning "Isaac Sim not found at $ISAAC_SIM_PATH"
    print_status "Please install Isaac Sim from NVIDIA Omniverse:"
    echo "1. Go to https://www.nvidia.com/en-us/omniverse/"
    echo "2. Download Omniverse Launcher"
    echo "3. Install Isaac Sim through the launcher"
    echo ""
    read -p "Press Enter to continue after installing Isaac Sim..."
fi

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    print_error "Python 3 is not installed. Please install Python 3.8 or later."
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
print_success "Python $PYTHON_VERSION is installed"

# Check if Git is installed
if ! command -v git &> /dev/null; then
    print_error "Git is not installed. Please install Git first."
    exit 1
fi

print_success "Git is installed"

# Create necessary directories
print_status "Creating project directories..."

# Create workspace directory if it doesn't exist
WORKSPACE_DIR="/Users/tpancholi/dev/nvidia-issac-sim"
if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$WORKSPACE_DIR"
    print_success "Created workspace directory: $WORKSPACE_DIR"
fi

# Create backup directory
BACKUP_DIR="$WORKSPACE_DIR/backups"
mkdir -p "$BACKUP_DIR"
print_success "Created backup directory: $BACKUP_DIR"

# Create logs directory
LOGS_DIR="$WORKSPACE_DIR/logs"
mkdir -p "$LOGS_DIR"
print_success "Created logs directory: $LOGS_DIR"

# Set up environment variables
print_status "Setting up environment variables..."

# Create .env file
ENV_FILE="$WORKSPACE_DIR/.env"
cat > "$ENV_FILE" << EOF
# Isaac Sim Environment Variables
export ISAAC_SIM_PATH="/Applications/NVIDIA-Omniverse/Isaac-Sim"
export ROS_DOMAIN_ID=0
export ROS_DISTRO=humble
export WORKSPACE_DIR="$WORKSPACE_DIR"
export PYTHONPATH="\$PYTHONPATH:$WORKSPACE_DIR"

# Docker settings
export DOCKER_COMPOSE_FILE="$WORKSPACE_DIR/09-docker-setup/docker-compose.yml"
export ROS2_CONTAINER_NAME="isaac-sim-ros2-dev"

# Aliases for convenience
alias isaac-sim="cd \$ISAAC_SIM_PATH && ./isaac-sim.sh"
alias ros2-dev="docker exec -it \$ROS2_CONTAINER_NAME bash"
alias ros2-topics="docker exec -it \$ROS2_CONTAINER_NAME ros2 topic list"
alias ros2-nodes="docker exec -it \$ROS2_CONTAINER_NAME ros2 node list"
EOF

print_success "Created environment file: $ENV_FILE"

# Add to shell profile
SHELL_PROFILE=""
if [ -f "$HOME/.zshrc" ]; then
    SHELL_PROFILE="$HOME/.zshrc"
elif [ -f "$HOME/.bash_profile" ]; then
    SHELL_PROFILE="$HOME/.bash_profile"
elif [ -f "$HOME/.bashrc" ]; then
    SHELL_PROFILE="$HOME/.bashrc"
fi

if [ -n "$SHELL_PROFILE" ]; then
    # Check if already added
    if ! grep -q "nvidia-issac-sim" "$SHELL_PROFILE"; then
        echo "" >> "$SHELL_PROFILE"
        echo "# Isaac Sim Environment" >> "$SHELL_PROFILE"
        echo "source $ENV_FILE" >> "$SHELL_PROFILE"
        print_success "Added environment variables to $SHELL_PROFILE"
    else
        print_status "Environment variables already added to $SHELL_PROFILE"
    fi
fi

# Set up Docker resources
print_status "Configuring Docker resources..."

# Check Docker resources
DOCKER_MEMORY=$(docker system info --format '{{.MemTotal}}' 2>/dev/null || echo "0")
DOCKER_MEMORY_GB=$((DOCKER_MEMORY / 1024 / 1024 / 1024))

if [ "$DOCKER_MEMORY_GB" -lt 8 ]; then
    print_warning "Docker has less than 8GB memory allocated ($DOCKER_MEMORY_GB GB)"
    print_status "Please increase Docker memory allocation:"
    echo "1. Open Docker Desktop"
    echo "2. Go to Settings → Resources → Advanced"
    echo "3. Set Memory to at least 8GB"
    echo "4. Click Apply & Restart"
fi

# Create useful aliases
print_status "Creating useful aliases..."

ALIASES_FILE="$WORKSPACE_DIR/aliases.sh"
cat > "$ALIASES_FILE" << 'EOF'
#!/bin/bash

# Isaac Sim Development Aliases

# Quick navigation
alias cd-isaac="cd /Users/tpancholi/dev/nvidia-issac-sim"
alias cd-examples="cd /Users/tpancholi/dev/nvidia-issac-sim/13-examples"
alias cd-tutorials="cd /Users/tpancholi/dev/nvidia-issac-sim/14-tutorials"
alias cd-ros2="cd /Users/tpancholi/dev/nvidia-issac-sim/08-ros2-integration"

# Docker commands
alias docker-start="cd /Users/tpancholi/dev/nvidia-issac-sim/09-docker-setup && ./start-ros2-container.sh"
alias docker-stop="cd /Users/tpancholi/dev/nvidia-issac-sim/09-docker-setup && docker-compose down"
alias docker-logs="docker logs isaac-sim-ros2-dev"

# ROS2 commands
alias ros2-shell="docker exec -it isaac-sim-ros2-dev bash"
alias ros2-topics="docker exec -it isaac-sim-ros2-dev ros2 topic list"
alias ros2-nodes="docker exec -it isaac-sim-ros2-dev ros2 node list"
alias ros2-info="docker exec -it isaac-sim-ros2-dev ros2 node info"

# Isaac Sim commands
alias isaac-start="cd /Users/tpancholi/dev/nvidia-issac-sim/09-docker-setup && ./start-isaac-sim.sh"
alias isaac-path="echo /Applications/NVIDIA-Omniverse/Isaac-Sim"

# Development commands
alias build-ros2="cd /Users/tpancholi/dev/nvidia-issac-sim/09-docker-setup && ./build-ros2-image.sh"
alias clean-docker="docker system prune -f && docker volume prune -f"

# Utility commands
alias show-ports="lsof -i -P -n | grep LISTEN"
alias check-gpu="nvidia-smi 2>/dev/null || echo 'NVIDIA GPU not detected'"
alias check-resources="top -l 1 | grep -E 'PhysMem|CPU usage'"
EOF

chmod +x "$ALIASES_FILE"
print_success "Created aliases file: $ALIASES_FILE"

# Create project template
print_status "Creating project template..."

TEMPLATE_DIR="$WORKSPACE_DIR/templates"
mkdir -p "$TEMPLATE_DIR"

# Create Python project template
cat > "$TEMPLATE_DIR/python_project_template.py" << 'EOF'
"""
Isaac Sim Python Project Template

This template provides a basic structure for Isaac Sim Python projects.
Copy this file and modify it for your specific needs.

Author: Your Name
Date: $(date +%Y-%m-%d)
"""

import omni.usd
import omni.kit.commands
from pxr import UsdGeom, Gf
import numpy as np
import time

class IsaacSimProject:
    def __init__(self):
        """Initialize the project."""
        self.stage = omni.usd.get_context().get_stage()
        self.project_name = "MyProject"
        
    def setup_scene(self):
        """Set up the simulation scene."""
        print(f"Setting up scene for {self.project_name}...")
        # Add your scene setup code here
        
    def run_simulation(self):
        """Run the simulation."""
        print(f"Running simulation for {self.project_name}...")
        # Add your simulation code here
        
    def cleanup(self):
        """Clean up resources."""
        print(f"Cleaning up {self.project_name}...")
        # Add your cleanup code here

def main():
    """Main function."""
    print("Starting Isaac Sim Project...")
    
    # Create project instance
    project = IsaacSimProject()
    
    try:
        # Setup scene
        project.setup_scene()
        
        # Run simulation
        project.run_simulation()
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Cleanup
        project.cleanup()

if __name__ == "__main__":
    main()
EOF

print_success "Created project template"

# Create README for scripts
cat > "$WORKSPACE_DIR/11-scripts/README.md" << 'EOF'
# Development Scripts

This directory contains helpful scripts for Isaac Sim development.

## Available Scripts

### setup-environment.sh
Sets up the complete development environment for Isaac Sim learning.

**Usage:**
```bash
./setup-environment.sh
```

### start-development.sh
Starts the complete development environment (Docker + Isaac Sim).

**Usage:**
```bash
./start-development.sh
```

### stop-development.sh
Stops all development services.

**Usage:**
```bash
./stop-development.sh
```

### clean-environment.sh
Cleans up Docker containers, images, and volumes.

**Usage:**
```bash
./clean-environment.sh
```

### check-status.sh
Checks the status of all development services.

**Usage:**
```bash
./check-status.sh
```

## Environment Variables

After running setup-environment.sh, the following environment variables are available:

- `ISAAC_SIM_PATH`: Path to Isaac Sim installation
- `ROS_DOMAIN_ID`: ROS2 domain ID (default: 0)
- `ROS_DISTRO`: ROS2 distribution (default: humble)
- `WORKSPACE_DIR`: Path to workspace directory

## Aliases

Useful aliases are created in `aliases.sh`:

- `cd-isaac`: Navigate to workspace
- `docker-start`: Start ROS2 container
- `docker-stop`: Stop ROS2 container
- `ros2-shell`: Access ROS2 container shell
- `isaac-start`: Start Isaac Sim

## Troubleshooting

If you encounter issues:

1. Check Docker is running: `docker info`
2. Check container status: `docker ps`
3. View container logs: `docker logs isaac-sim-ros2-dev`
4. Restart services: `./stop-development.sh && ./start-development.sh`
EOF

print_success "Created scripts README"

# Final setup
print_status "Performing final setup..."

# Make scripts executable
chmod +x "$WORKSPACE_DIR/11-scripts"/*.sh 2>/dev/null || true

# Create log file
LOG_FILE="$LOGS_DIR/setup.log"
echo "Isaac Sim Environment Setup - $(date)" > "$LOG_FILE"
echo "==========================================" >> "$LOG_FILE"
echo "System: $(uname -a)" >> "$LOG_FILE"
echo "Python: $(python3 --version)" >> "$LOG_FILE"
echo "Docker: $(docker --version)" >> "$LOG_FILE"
echo "Git: $(git --version)" >> "$LOG_FILE"
echo "==========================================" >> "$LOG_FILE"

print_success "Setup completed successfully!"

echo ""
echo "=========================================="
echo "Setup Summary"
echo "=========================================="
echo "✅ Environment variables configured"
echo "✅ Aliases created"
echo "✅ Project templates ready"
echo "✅ Scripts made executable"
echo "✅ Logs directory created"
echo ""
echo "Next steps:"
echo "1. Source your shell profile: source ~/.zshrc (or ~/.bash_profile)"
echo "2. Start development: ./11-scripts/start-development.sh"
echo "3. Begin learning: cd 01-getting-started"
echo ""
echo "For help, check: 10-documentation/macos-setup-guide.md"
echo "=========================================="
